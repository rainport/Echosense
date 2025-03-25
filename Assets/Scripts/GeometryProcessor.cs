using UnityEngine;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System;

public class GeometryProcessor : MonoBehaviour
{
    [StructLayout(LayoutKind.Sequential)]
    private struct ObjectGeometry
    {
        public IntPtr vertices;      // float* - All vertices
        public IntPtr triangles;     // int* - Triangle indices (3 per face)
        public IntPtr faceNormals;   // float* - Computed face normals
        public IntPtr faceCenters;   // float* - Computed face centers
        public IntPtr triangleAreas; // float* - Computed triangle areas
        public int triangleCount;    // Number of triangles
        public IntPtr transform;     // float* - 4x4 world transform matrix
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct AudioParameters
    {
        public float amplitude;        // Base amplitude scaling
        public float baseFreq;         // Base/carrier frequency
        public float dopplerFreq;      // Frequency after Doppler shift
        public float sigma;            // Width of Gaussian pulse envelope
        public float roundTripTime;    // Time for pulse to reach target and return
        public float phase;            // Phase offset
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct ProcessedPoint
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] position;
        
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] normal;
        
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] velocity;
        
        public AudioParameters parameters;
        public float facingFactor;
        public float area;            // Triangle area
    }

    // Native plugin function imports
    [DllImport("geometry_processor")]
    private static extern void InitializeProcessor(int sampleRate, int blockSize);
    
    [DllImport("geometry_processor")]
    private static extern void UpdateTime(float deltaTime);

    [DllImport("geometry_processor")]
    private static extern void SetRadarParameters(float pulseRepFreq, float speedOfSound, float pulseWidth);

    [DllImport("geometry_processor")]
    private static extern void DebugOscillators();

    [DllImport("geometry_processor")]
    private static extern void DebugBuffer();

    [DllImport("geometry_processor")]
    private static extern void ProcessObjectGeometry(
        [In] ObjectGeometry[] objects,
        int objectCount,
        [In] float[] listenerPosition,
        [In] float[] listenerForward,
        float maxDistance,
        float reflectionRadius,
        [Out] ProcessedPoint[] outputPoints,
        ref int outputPointCount
    );

    [DllImport("geometry_processor")]
    private static extern void SynthesizeAudio(
        [In] ProcessedPoint[] points,
        int pointCount,
        [Out] float[] outputBuffer,
        int bufferSize
    );

    [DllImport("geometry_processor")]
    private static extern void Cleanup();
        
    // Configuration
    [SerializeField] private int sampleRate = 44100;
    [SerializeField] private int blockSize = 4096; // Reduced from 8192 for smoother updates
    [SerializeField] private float maxDistance = 10f;
    [SerializeField] private LayerMask objectLayers = -1;
    [SerializeField] private float pulseRepFreq = 2.0f; // Pulses per second
    [SerializeField] private float sigma = 0.5f; // Pulse width
    [SerializeField] private float speedOfSound = 8f; // Slower than actual for dramatic effect
    [SerializeField] private float reflectionRadius = 0.3f; // 'r' parameter for d*sin(θ)≤r
    [SerializeField] private bool showDebugVisualization = true;

    // Runtime data
    private Camera mainCamera;
    private AudioSource audioSource;
    private List<GCHandle> pinnedArrays = new List<GCHandle>();
    private ProcessedPoint[] processedPoints = new ProcessedPoint[1024];
    private float[] audioBuffer;
    private MeshFilter[] nearbyMeshes;

    // Double-buffering audio clips
    private AudioClip clipA;
    private AudioClip clipB;
    private bool usingClipA = true;
    
    // Timing control for audio updates
    private float lastAudioUpdateTime = 0f;
    private float audioUpdateInterval = 0.05f; // 50ms update interval

    void Start()
    {
        mainCamera = Camera.main;
        
        // Setup audio with double buffering
        audioSource = gameObject.AddComponent<AudioSource>();
        audioSource.playOnAwake = true;
        audioSource.spatialBlend = 0;
        audioSource.loop = false; // Important: we're managing playback manually
        audioSource.volume = 1.0f;
        
        // Allocate buffer for audio synthesis
        audioBuffer = new float[blockSize];
        
        // Create two audio clips for double buffering
        clipA = AudioClip.Create("ProcessedAudioA", blockSize, 1, sampleRate, false);
        clipB = AudioClip.Create("ProcessedAudioB", blockSize, 1, sampleRate, false);
        
        // Fill both clips with silence initially
        float[] silence = new float[blockSize];
        clipA.SetData(silence, 0);
        clipB.SetData(silence, 0);
        
        // Start with clip A
        audioSource.clip = clipA;
        audioSource.Play();
        
        // Initialize the native processor
        InitializeProcessor(sampleRate, blockSize);
        SetRadarParameters(pulseRepFreq, speedOfSound, sigma);
        
        // Initialize timing
        lastAudioUpdateTime = Time.time;
    }

    void Update()
    {
        // Toggle debug visualization
        if (Input.GetKeyDown(KeyCode.F1))
        {
            showDebugVisualization = !showDebugVisualization;
            Debug.Log($"Debug visualization: {(showDebugVisualization ? "ON" : "OFF")}");
        }
        
        // Add debug keys for the new functions
        if (Input.GetKeyDown(KeyCode.F2))
        {
            Debug.Log("Debugging oscillator states:");
            DebugOscillators();
        }
        
        if (Input.GetKeyDown(KeyCode.F3))
        {
            Debug.Log("Debugging audio buffer:");
            DebugBuffer();
        }
        
        // Update time in the C++ processor
        UpdateTime(Time.deltaTime);
        
        // Find nearby objects with meshes
        Collider[] nearbyColliders = Physics.OverlapSphere(
            mainCamera.transform.position, maxDistance, objectLayers
        );
        
        var meshFilters = new List<MeshFilter>();
        foreach (var collider in nearbyColliders)
        {
            var meshFilter = collider.GetComponent<MeshFilter>();
            if (meshFilter != null && meshFilter.mesh != null)
            {
                meshFilters.Add(meshFilter);
            }
        }
        nearbyMeshes = meshFilters.ToArray();

        if (nearbyMeshes.Length == 0) return;

        // Process geometry
        ProcessGeometry();
        
        // Update audio with double buffering
        UpdateAudio();

        if (showDebugVisualization)
        {
            VisualizeEchoPoints();
        }
    }

    private void UpdateAudio()
    {
        // Only update at our desired interval to avoid too many audio switches
        if (Time.time - lastAudioUpdateTime < audioUpdateInterval)
            return;
            
        lastAudioUpdateTime = Time.time;
        
        // Check if we need to switch clips
        bool shouldSwitch = false;
        
        // Switch when we're near the end of the current clip or if it's not playing
        if (audioSource.isPlaying)
        {
            // Switch when we're about 80% through the current clip
            shouldSwitch = audioSource.time >= (audioSource.clip.length * 0.8f);
        }
        else
        {
            // If not playing, we should start/switch
            shouldSwitch = true;
        }
        
        if (shouldSwitch)
        {
            // Update the clip that's NOT currently playing
            if (usingClipA)
            {
                // We're playing clip A, so update clip B
                clipB.SetData(audioBuffer, 0);
                
                // Switch to clip B
                audioSource.clip = clipB;
                usingClipA = false;
            }
            else
            {
                // We're playing clip B, so update clip A
                clipA.SetData(audioBuffer, 0);
                
                // Switch to clip A
                audioSource.clip = clipA;
                usingClipA = true;
            }
            
            // Make sure it's playing
            if (!audioSource.isPlaying)
            {
                audioSource.Play();
            }
        }
    }

    private void VisualizeEchoPoints()
    {
        if (processedPoints == null) return;
        
        int visiblePoints = 0;
        
        foreach (var point in processedPoints)
        {
            if (point.position == null || point.normal == null) continue;
            
            visiblePoints++;
            
            Vector3 pos = new Vector3(point.position[0], 
                                    point.position[1], 
                                    point.position[2]);
            Vector3 norm = new Vector3(point.normal[0],
                                    point.normal[1], 
                                    point.normal[2]);
            
            // Draw a cross to mark the position
            float size = 0.05f;
            Debug.DrawRay(pos, Vector3.up * size, Color.green);
            Debug.DrawRay(pos, Vector3.right * size, Color.green);
            Debug.DrawRay(pos, Vector3.forward * size, Color.green);
            
            // Draw normal direction
            Debug.DrawRay(pos, norm * 0.3f, Color.blue);
        }
        
        // Log visible point count periodically
        if (Time.frameCount % 60 == 0 && visiblePoints > 0)
        {
            Debug.Log($"Visible Points: {visiblePoints}");
        }
    }

    private void ProcessGeometry()
    {
        // Clean up any previously pinned arrays
        foreach (var handle in pinnedArrays)
        {
            if (handle.IsAllocated)
                handle.Free();
        }
        pinnedArrays.Clear();

        // Prepare object geometries
        var geometries = new ObjectGeometry[nearbyMeshes.Length];
        
        for (int i = 0; i < nearbyMeshes.Length; i++)
        {
            var mesh = nearbyMeshes[i].mesh;
            var transform = nearbyMeshes[i].transform;
            var vertices = mesh.vertices;
            var triangles = mesh.triangles;
            
            // Compute face normals, centers, and areas
            var faceNormals = new Vector3[triangles.Length / 3];
            var faceCenters = new Vector3[triangles.Length / 3];
            var triangleAreas = new float[triangles.Length / 3];
            
            for (int f = 0; f < triangles.Length; f += 3)
            {
                Vector3 v1 = vertices[triangles[f]];
                Vector3 v2 = vertices[triangles[f + 1]];
                Vector3 v3 = vertices[triangles[f + 2]];
                
                // Compute face normal
                Vector3 normal = Vector3.Cross(v2 - v1, v3 - v1).normalized;
                faceNormals[f/3] = normal;
                
                // Compute face center
                faceCenters[f/3] = (v1 + v2 + v3) / 3f;
                
                // Compute triangle area
                triangleAreas[f/3] = 0.5f * Vector3.Cross(v2 - v1, v3 - v1).magnitude;
            }
            
            // Pin all the arrays
            var verticesHandle = GCHandle.Alloc(vertices, GCHandleType.Pinned);
            var trianglesHandle = GCHandle.Alloc(triangles, GCHandleType.Pinned);
            var normalsHandle = GCHandle.Alloc(faceNormals, GCHandleType.Pinned);
            var centersHandle = GCHandle.Alloc(faceCenters, GCHandleType.Pinned);
            var areasHandle = GCHandle.Alloc(triangleAreas, GCHandleType.Pinned);
            
            pinnedArrays.Add(verticesHandle);
            pinnedArrays.Add(trianglesHandle);
            pinnedArrays.Add(normalsHandle);
            pinnedArrays.Add(centersHandle);
            pinnedArrays.Add(areasHandle);

            // Create and pin transform matrix
            var matrix = transform.localToWorldMatrix;
            var matrixArray = new float[16] {
                matrix.m00, matrix.m01, matrix.m02, matrix.m03,
                matrix.m10, matrix.m11, matrix.m12, matrix.m13,
                matrix.m20, matrix.m21, matrix.m22, matrix.m23,
                matrix.m30, matrix.m31, matrix.m32, matrix.m33
            };
            var matrixHandle = GCHandle.Alloc(matrixArray, GCHandleType.Pinned);
            pinnedArrays.Add(matrixHandle);

            // Fill geometry struct
            geometries[i] = new ObjectGeometry
            {
                vertices = verticesHandle.AddrOfPinnedObject(),
                triangles = trianglesHandle.AddrOfPinnedObject(),
                faceNormals = normalsHandle.AddrOfPinnedObject(),
                faceCenters = centersHandle.AddrOfPinnedObject(),
                triangleAreas = areasHandle.AddrOfPinnedObject(),
                triangleCount = triangles.Length / 3,
                transform = matrixHandle.AddrOfPinnedObject()
            };
        }

        // Prepare listener data
        var listenerPos = mainCamera.transform.position;
        var listenerFwd = mainCamera.transform.forward;
        var listenerPosArray = new float[] { listenerPos.x, listenerPos.y, listenerPos.z };
        var listenerFwdArray = new float[] { listenerFwd.x, listenerFwd.y, listenerFwd.z };

        // Process geometry
        int pointCount = 0;
        ProcessObjectGeometry(
            geometries,
            geometries.Length,
            listenerPosArray,
            listenerFwdArray,
            maxDistance,
            reflectionRadius,
            processedPoints,
            ref pointCount
        );
        
        // Generate audio
        SynthesizeAudio(
            processedPoints,
            pointCount,
            audioBuffer,
            blockSize
        );
    }

    void OnDestroy()
    {
        // Clean up pinned arrays
        foreach (var handle in pinnedArrays)
        {
            if (handle.IsAllocated)
                handle.Free();
        }
        pinnedArrays.Clear();
        
        // Cleanup native processor
        Cleanup();
    }

    void OnDrawGizmos()
    {
        if (!mainCamera) return;

        // Draw detection sphere
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(mainCamera.transform.position, maxDistance);
        
        // Optionally visualize the reflection radius parameter
        if (showDebugVisualization)
        {
            // Draw a small sphere to represent the reflection radius parameter
            Gizmos.color = Color.red;
            float visualScale = 0.5f; // Scale to make it visible in the scene
            Gizmos.DrawWireSphere(mainCamera.transform.position, reflectionRadius * visualScale);
        }
        
        // Draw processed points in the editor
        if (Application.isPlaying && processedPoints != null && showDebugVisualization)
        {
            foreach (var point in processedPoints)
            {
                if (point.position == null || point.normal == null) continue;
                
                Vector3 pos = new Vector3(point.position[0], 
                                        point.position[1], 
                                        point.position[2]);
                Vector3 normal = new Vector3(point.normal[0],
                                        point.normal[1],
                                        point.normal[2]);
                
                // Draw face center
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(pos, 0.05f);
                
                // Draw normal direction
                Gizmos.color = Color.blue;
                Gizmos.DrawRay(pos, normal * 0.3f);
            }
        }
    }
}