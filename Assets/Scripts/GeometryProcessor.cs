using UnityEngine;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System;
using System.IO;
using Enumerable = System.Linq.Enumerable;
using Random = UnityEngine.Random; // Required for Path.Combine

public class GeometryProcessor : MonoBehaviour
{
    // --- Structs (match C++ side) ---
    // Keep these matching the C++ header definitions
    [StructLayout(LayoutKind.Sequential)]
    private struct ObjectGeometry {
        public int objectId; public IntPtr vertices; public IntPtr triangles; public IntPtr faceNormals;
        public IntPtr faceCenters; public IntPtr triangleAreas; public int triangleCount; public IntPtr transform;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct ReferencePoint {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] position;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] normal;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct AudioParameters {
        public float amplitude; public float baseFreq; public float dopplerFreq; public float sigma;
        public float roundTripTime; public float phase;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct ProcessedPoint {
        public int objectId; public int triangleIndex;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public float[] position;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public float[] normal;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public float[] velocity;
        public AudioParameters parameters; public float facingFactor; public float area;
    }

    // --- Native plugin function imports ---
    private const string PluginName = "geometry_processor";
    [DllImport(PluginName, CallingConvention = CallingConvention.Cdecl)] private static extern void InitializeProcessor(int sampleRate, int blockSize);
    [DllImport(PluginName, CallingConvention = CallingConvention.Cdecl)] private static extern void SetHrtfPath(string sofaFilePath);
    [DllImport(PluginName, CallingConvention = CallingConvention.Cdecl)] private static extern void SetListenerOrientation([In] float[] listenerForward, [In] float[] listenerUp);
    [DllImport(PluginName, CallingConvention = CallingConvention.Cdecl)] private static extern void UpdateTime(float deltaTime);
    [DllImport(PluginName, CallingConvention = CallingConvention.Cdecl)] private static extern void SetRadarParameters(float pulseRepFreq, float speedOfSound, float pulseWidth);
    [DllImport(PluginName, CallingConvention = CallingConvention.Cdecl)] private static extern void DebugOscillators();
    [DllImport(PluginName, CallingConvention = CallingConvention.Cdecl)] private static extern void ProcessObjectGeometry(
        [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 1)] ObjectGeometry[] objects, int objectCount,
        [In] float[] listenerPosition, [In] float[] listenerForward, float maxDistance, float reflectionRadius,
        [Out, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 7)] ProcessedPoint[] outputPoints, ref int outputPointCount // SizeParamIndex = 7
    );
    [DllImport(PluginName, CallingConvention = CallingConvention.Cdecl)]
    private static extern void ProcessReferencePoints(
        [In,  MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 1)]
        ReferencePoint[] points, int pointCount,
        [In] float[] listenerPosition,
        float maxDistance,
        [Out, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 5)] 
        ProcessedPoint[] outputPoints,
        int outputBufferSize, 
        ref int outputPointCount
    );

    [DllImport(PluginName)] private static extern void SynthesizeAudio(
        [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 1)] ProcessedPoint[] points, int pointCount,
        [Out] float[] outputBuffer, int channelCount, int samplesPerChannel
    );
    [DllImport(PluginName)] private static extern void Cleanup();

    // --- Configuration ---
    [Header("Audio Settings")]
    [SerializeField] private int sampleRate = 44100;
    [SerializeField] private int blockSize = 1024;
    [Header("Radar Settings")]
    [SerializeField] private float maxDistance = 10f;
    [SerializeField] private LayerMask objectLayers = -1;
    [SerializeField] public float pulseRepFreq = 2.0f;
    [SerializeField] public float sigma = 0.05f;
    [SerializeField] public float speedOfSound = 343f;
    [SerializeField] public float reflectionRadius = 0.3f;
    [Header("HRTF Settings")]
    [Tooltip("Name of the SOFA file located in Assets/StreamingAssets")]
    [SerializeField] private string sofaFileName = "mit_kemar_normal_pinna.sofa";
    [Header("Debugging")]
    [SerializeField] private bool showDebugVisualization = true;
    [SerializeField] private int maxOutputPoints = 256; // Match C++ MAX_OSCILLATORS if possible

    // --- Runtime data ---
    private Camera mainCamera;
    private AudioSource audioSource;
    private List<GCHandle> pinnedArrays = new List<GCHandle>();
    private ProcessedPoint[] processedPoints;
    private float[] audioBuffer;
    private MeshFilter[] nearbyMeshes;
    private AudioClip clipA, clipB;
    private bool nextClipIsA = false;
    private int audioBufferLength;
    private float timeToGenerateNextBuffer = 0f;
    private double nextSchedTime = 0;
    private bool audioStarted = false;
    private float[] listenerForwardArray = new float[3];
    private float[] listenerUpArray = new float[3];
    private float[] listenerPositionArray = new float[3];
    private float lastPulseRepFreq, lastSigma, lastSpeedOfSound;
    
    [Range(0f, 2f)]
    [Tooltip("Overall gain for all echo audio. 1 = unity, >1 = louder, <1 = quieter")]
    public float masterVolume = 1f;

    // ** NEW: Store the actual count of valid points from C++ **
    private int currentValidPointCount = 0;
    private List<Vector3> fixedDirs;
    private int numRays = 64;
    private ReferencePoint[] refsArray;

    void Start()
    {
        refsArray = new ReferencePoint[numRays];
        // Precompute a deterministic, evenly-distributed set of directions:
        fixedDirs = new List<Vector3>(numRays);
        float phi = (1 + Mathf.Sqrt(5)) / 2;                   // golden ratio
        float twoPiPhi = 2 * Mathf.PI * phi;
        for (int i = 0; i < numRays; i++)
        {
            float t = (float)i / numRays;
            float inclination = Mathf.Acos(1 - 2 * t);
            float azimuth     = twoPiPhi * i;

            // Spherical → Cartesian
            float x = Mathf.Sin(inclination) * Mathf.Cos(azimuth);
            float y = Mathf.Sin(inclination) * Mathf.Sin(azimuth);
            float z = Mathf.Cos(inclination);

            // apply your horizontal bias:
            Vector3 dir = new Vector3(x, y, z);
            dir.y *= verticalBias;
            dir.Normalize();

            fixedDirs.Add(dir);
        }
        
        mainCamera = Camera.main;
        if (!mainCamera) { Debug.LogError("Main Camera not found!"); enabled = false; return; }

        audioSource = gameObject.GetComponent<AudioSource>();
        if (!audioSource) { audioSource = gameObject.AddComponent<AudioSource>(); }
        audioSource.playOnAwake = false; audioSource.spatialBlend = 0; audioSource.loop = true; audioSource.volume = 1.0f;

        audioBufferLength = blockSize * 2;
        audioBuffer = new float[audioBufferLength];
        processedPoints = new ProcessedPoint[maxOutputPoints];
        for (int i = 0; i < processedPoints.Length; i++) { // Pre-initialize sub-arrays
            processedPoints[i].position = new float[3]; processedPoints[i].normal = new float[3]; processedPoints[i].velocity = new float[3];
        }

        clipA = AudioClip.Create("ProcessedAudioA", blockSize, 2, sampleRate, false);
        clipB = AudioClip.Create("ProcessedAudioB", blockSize, 2, sampleRate, false);
        float[] silence = new float[audioBufferLength];
        clipA.SetData(silence, 0); clipB.SetData(silence, 0);

        InitializeProcessor(sampleRate, blockSize); // Initial init before path

        string fullHrtfPath = "";
        if (!string.IsNullOrEmpty(sofaFileName)) {
            fullHrtfPath = Path.Combine(Application.streamingAssetsPath, sofaFileName);
            if (File.Exists(fullHrtfPath)) {
                Debug.Log($"Setting HRTF path: {fullHrtfPath}");
                SetHrtfPath(fullHrtfPath);
                InitializeProcessor(sampleRate, blockSize); // Re-initialize AFTER setting path
            } else {
                Debug.LogError($"SOFA file '{sofaFileName}' not found at: {fullHrtfPath}. Spatialization disabled.");
                SetHrtfPath(null); InitializeProcessor(sampleRate, blockSize);
            }
        } else {
             Debug.LogWarning("SOFA Filename empty. Spatialization disabled.");
             SetHrtfPath(null); InitializeProcessor(sampleRate, blockSize);
        }

        SetRadarParameters(pulseRepFreq, speedOfSound, sigma);
        lastPulseRepFreq = pulseRepFreq; lastSigma = sigma; lastSpeedOfSound = speedOfSound;

        audioStarted = false; nextSchedTime = AudioSettings.dspTime + 0.1; timeToGenerateNextBuffer = Time.time;
    }
    
    [SerializeField] private float geometryUpdateThreshold = 0.5f;
    private Vector3 lastGeometryUpdatePosition;
    private ObjectGeometry[] cachedGeometryArray;

    
    private float verticalBias = 0.05f;
    
    void Update()
    {
        if (!mainCamera) return;

        UpdateTime(Time.deltaTime);
        Vector3 camPos = mainCamera.transform.position;
        Vector3 camFwd = mainCamera.transform.forward;
        Vector3 camUp = mainCamera.transform.up;

        listenerPositionArray[0] = camPos.x; listenerPositionArray[1] = camPos.y; listenerPositionArray[2] = camPos.z;
        listenerForwardArray[0] = camFwd.x; listenerForwardArray[1] = camFwd.y; listenerForwardArray[2] = camFwd.z;
        listenerUpArray[0] = camUp.x; listenerUpArray[1] = camUp.y; listenerUpArray[2] = camUp.z;
        SetListenerOrientation(listenerForwardArray, listenerUpArray);

        if (Time.time >= timeToGenerateNextBuffer)
        {
            // Raycast along each fixed direction:
            currentValidPointCount = 0;
            foreach (var dir in fixedDirs)
            {
                if (Physics.Raycast(camPos, dir, out var hit, maxDistance, objectLayers))
                {
                    refsArray[currentValidPointCount++] = new ReferencePoint {
                        position = new[] { hit.point.x,  hit.point.y,  hit.point.z },
                        normal   = new[] { hit.normal.x, hit.normal.y, hit.normal.z }
                    };
                }
            }
            // pin refs into a ReferencePoint[] and call:
            ProcessReferencePoints(refsArray, currentValidPointCount,
                listenerPositionArray,
                maxDistance,
                processedPoints, numRays, ref currentValidPointCount);

            AudioClip clipToUpdate = nextClipIsA ? clipA : clipB;
            clipToUpdate.SetData(audioBuffer, 0);

            if (!audioStarted)
            {
                audioSource.clip = clipToUpdate;
                audioSource.PlayScheduled(nextSchedTime);
                audioStarted = true;
            }
            else
            {
                audioSource.SetScheduledEndTime(nextSchedTime);
                audioSource.PlayScheduled(nextSchedTime);
            }

            nextClipIsA = !nextClipIsA;
            nextSchedTime += (double)blockSize / sampleRate;
            float generationLeadTime = 0.02f;
            timeToGenerateNextBuffer = Time.time + Mathf.Max(0, (float)(nextSchedTime - AudioSettings.dspTime) - generationLeadTime);
            SynthesizeAudio(processedPoints, 0, audioBuffer, 2, blockSize);
        }

        if (showDebugVisualization)
        {
            VisualizeEchoPoints();
        }
    }

    private void SynthesizeEmptyAudio() {
         currentValidPointCount = 0; // No points found
         // Ensure buffer exists before calling native function
         if (processedPoints == null || processedPoints.Length != maxOutputPoints) {
             processedPoints = new ProcessedPoint[maxOutputPoints];
             // Initialize sub-arrays
             for (int i = 0; i < processedPoints.Length; i++) {
                 processedPoints[i].position = new float[3]; processedPoints[i].normal = new float[3]; processedPoints[i].velocity = new float[3];
             }
         }
         SynthesizeAudio(processedPoints, 0, audioBuffer, 2, blockSize);
    }

    private void VisualizeEchoPoints() {
        if (processedPoints == null) return;

        // ** Use currentValidPointCount instead of processedPoints.Length **
        int countToDraw = currentValidPointCount;

        for(int i = 0; i < countToDraw; ++i) { // Iterate only up to the valid count
            // Basic check if position data exists (already initialized in Start/Process)
             if (processedPoints[i].position == null || processedPoints[i].position.Length < 3) continue;

             // Check for zero position AND zero amplitude as indicator of unused slot
             if (Mathf.Approximately(processedPoints[i].position[0], 0f) &&
                 Mathf.Approximately(processedPoints[i].position[1], 0f) &&
                 Mathf.Approximately(processedPoints[i].position[2], 0f) &&
                 Mathf.Approximately(processedPoints[i].parameters.amplitude, 0f))
             {
                 continue;
             }

            Vector3 pos = new Vector3(processedPoints[i].position[0], processedPoints[i].position[1], processedPoints[i].position[2]);
            Vector3 norm = Vector3.zero;
            if (processedPoints[i].normal != null && processedPoints[i].normal.Length >= 3) {
                 norm = new Vector3(processedPoints[i].normal[0], processedPoints[i].normal[1], processedPoints[i].normal[2]);
            }
            Debug.DrawRay(pos, Vector3.up * 0.05f, Color.green); Debug.DrawRay(pos, Vector3.down * 0.05f, Color.green);
            Debug.DrawRay(pos, Vector3.left * 0.05f, Color.green); Debug.DrawRay(pos, Vector3.right * 0.05f, Color.green);
            Debug.DrawRay(pos, Vector3.forward * 0.05f, Color.green); Debug.DrawRay(pos, Vector3.back * 0.05f, Color.green);
            if (norm != Vector3.zero) Debug.DrawRay(pos, norm * 0.2f, Color.blue);
        }
    }

    void OnDestroy() {
        if (audioSource != null && audioSource.isPlaying) { audioSource.Stop(); }
        foreach (var handle in pinnedArrays) { if (handle.IsAllocated) handle.Free(); }
        pinnedArrays.Clear();
        Debug.Log("Cleaning up native GeometryProcessor...");
        Cleanup();
    }

    void OnDrawGizmos() {
        if (!mainCamera) { mainCamera = Camera.main; if (!mainCamera) return; }
        // Gizmos.color = Color.yellow; Gizmos.DrawWireSphere(mainCamera.transform.position, maxDistance);
        // Gizmos.color = Color.red; Gizmos.DrawWireSphere(mainCamera.transform.position, reflectionRadius);

        if (Application.isPlaying && showDebugVisualization && processedPoints != null) {
            Gizmos.color = Color.cyan;
            // ** Use currentValidPointCount here too **
            int countToDraw = currentValidPointCount;
            for(int i = 0; i < countToDraw; ++i) {
                 if (processedPoints[i].position == null || processedPoints[i].position.Length < 3) continue;
                 if (Mathf.Approximately(processedPoints[i].position[0], 0f) && Mathf.Approximately(processedPoints[i].position[1], 0f) && Mathf.Approximately(processedPoints[i].position[2], 0f) && Mathf.Approximately(processedPoints[i].parameters.amplitude, 0f)) continue;
                Vector3 pos = new Vector3(processedPoints[i].position[0], processedPoints[i].position[1], processedPoints[i].position[2]);
                Gizmos.DrawSphere(pos, 0.06f);
            }
        }
    }
}
