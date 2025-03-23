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
    }

    // Native plugin function imports
    [DllImport("geometry_processor")]
    private static extern void InitializeProcessor(int sampleRate, int blockSize);
    
    [DllImport("geometry_processor")]
    private static extern void UpdateTime(float deltaTime);

    [DllImport("geometry_processor")]
    private static extern void SetRadarParameters(float pulseRepFreq, float speedOfSound, float pulseWidth);

    [DllImport("geometry_processor")]
    private static extern void ProcessObjectGeometry(
        [In] ObjectGeometry[] objects,
        int objectCount,
        [In] float[] listenerPosition,
        [In] float[] listenerForward,
        float maxDistance,
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
    [SerializeField] private int blockSize = 65536;
    [SerializeField] private float maxDistance = 10f;
    [SerializeField] private LayerMask objectLayers = -1;
    [SerializeField] private float pulseRepFreq = 2.0f; // Pulses per second
    [SerializeField] private float sigma = 0.5f; // Add this
    [SerializeField] private float speedOfSound = 8; // Slower than actual for dramatic effect


    // Runtime data
    private Camera mainCamera;
    private AudioSource audioSource;
    private List<GCHandle> pinnedArrays = new List<GCHandle>();
    private ProcessedPoint[] processedPoints = new ProcessedPoint[1024];
    private float[] audioBuffer;
    private MeshFilter[] nearbyMeshes;

    void Start()
    {
        mainCamera = Camera.main;
        
        // Setup audio
        audioSource = gameObject.AddComponent<AudioSource>();
        audioSource.playOnAwake = true;
        audioSource.spatialBlend = 0;
        audioSource.loop = true;
        
        audioBuffer = new float[blockSize];
        var clip = AudioClip.Create("ProcessedAudio", blockSize, 1, sampleRate, false);
        audioSource.clip = clip;
        
        InitializeProcessor(sampleRate, blockSize);
        audioSource.Play();

        SetRadarParameters(pulseRepFreq, speedOfSound, sigma);
    }

    void Update()
    {
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

        //VisualizeEchoTiming();

        Debug.Log($"Found {nearbyMeshes.Length} meshes:");
        foreach (var meshFilter in nearbyMeshes)
        {
            Mesh mesh = meshFilter.mesh;
            Debug.Log($"Mesh '{meshFilter.gameObject.name}': {mesh.triangles.Length/3} triangles");
        }
    }

    private void VisualizeEchoTiming()
    {
        if (processedPoints != null)
        {
            foreach (var point in processedPoints)
            {
                if (point.position == null || point.normal == null) continue;
                
                Vector3 pos = new Vector3(point.position[0], 
                                        point.position[1], 
                                        point.position[2]);
                Vector3 norm = new Vector3(point.normal[0],
                                        point.normal[1], 
                                        point.normal[2]);
                                        
                float dist = Vector3.Distance(mainCamera.transform.position, pos);
                float speedOfSound = 8.0f;
                float roundTrip = 2f * dist / speedOfSound;
                
                // Delay time indicator
                Vector3 lineStart = pos + Vector3.up * 0.3f;
                Vector3 lineEnd = lineStart + Vector3.right * (roundTrip * 0.5f);
                Color timeColor = Color.Lerp(Color.green, Color.red, 
                                        Mathf.Min(1f, roundTrip / 2.0f));
                Debug.DrawLine(lineStart, lineEnd, timeColor);
                
                // Draw normal vector in blue
                Debug.DrawLine(pos, pos + norm * 0.5f, Color.blue);
                
                // Draw line to listener in yellow
                Vector3 toListener = (mainCamera.transform.position - pos).normalized;
                Debug.DrawLine(pos, pos + toListener * 0.3f, Color.yellow);
                
                // Facing factor (dot product) visualization
                float facingFactor = Vector3.Dot(norm, toListener);
                Color facingColor = facingFactor > 0.7f ? Color.green : 
                                (facingFactor > 0.4f ? Color.yellow : Color.red);
                Debug.DrawLine(pos, pos + Vector3.up * (0.1f + facingFactor * 0.2f), facingColor);
            }
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
            
            // Compute face normals and centers
            var faceNormals = new Vector3[triangles.Length / 3];
            var faceCenters = new Vector3[triangles.Length / 3];
            
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
            }
            
            // Pin all the arrays
            var verticesHandle = GCHandle.Alloc(vertices, GCHandleType.Pinned);
            var trianglesHandle = GCHandle.Alloc(triangles, GCHandleType.Pinned);
            var normalsHandle = GCHandle.Alloc(faceNormals, GCHandleType.Pinned);
            var centersHandle = GCHandle.Alloc(faceCenters, GCHandleType.Pinned);
            
            pinnedArrays.Add(verticesHandle);
            pinnedArrays.Add(trianglesHandle);
            pinnedArrays.Add(normalsHandle);
            pinnedArrays.Add(centersHandle);

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
            processedPoints,
            ref pointCount
        );

        Debug.Log($"Processed {pointCount} points with radar");

        // Generate audio
        SynthesizeAudio(
            processedPoints,
            pointCount,
            audioBuffer,
            blockSize
        );

        if (processedPoints != null && pointCount > 0) {
            Debug.Log($"First point - Sigma: {processedPoints[0].parameters.sigma}, " +
                    $"Freq: {processedPoints[0].parameters.baseFreq}, " +
                    $"Doppler: {processedPoints[0].parameters.dopplerFreq}");
        }

        float sum = 0;
        for(int i = 0; i < audioBuffer.Length; i++) {
            sum += Mathf.Abs(audioBuffer[i]);
        }
        Debug.Log($"Audio buffer sum: {sum}, average: {sum/audioBuffer.Length}");

        // Update audio clip
        audioSource.clip.SetData(audioBuffer, 0);
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

        // Draw processed points and their normals if in play mode
        if (Application.isPlaying && processedPoints != null)
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
                float normalLength = 0.2f;
                Gizmos.DrawRay(pos, normal * normalLength);

                // Visualize velocity and doppler if available
                // if (point.velocity != null && point.velocity.Length == 3)
                // {
                //     Vector3 velocity = new Vector3(point.velocity[0], 
                //                                  point.velocity[1], 
                //                                  point.velocity[2]);
                    
                //     // Skip if velocity is too small to visualize
                //     if (velocity.magnitude < 0.01f) continue;
                    
                //     // Calculate approach/recede color
                //     Vector3 toListener = (mainCamera.transform.position - pos).normalized;
                //     float relativeVel = Vector3.Dot(velocity, toListener);
                    
                //     // Red for approaching, blue for receding
                //     Gizmos.color = relativeVel > 0 ? 
                //         Color.Lerp(Color.white, Color.red, Mathf.Min(1f, relativeVel * 5f)) : 
                //         Color.Lerp(Color.white, Color.blue, Mathf.Min(1f, -relativeVel * 5f));
                        
                //     // Draw velocity vector
                //     Gizmos.DrawRay(pos + normal * normalLength, velocity * 0.5f);
                    
                //     // Visualize doppler shift as text
                //     if (point.parameters.baseFreq > 0)
                //     {
                //         float dopplerShift = point.parameters.dopplerFreq - point.parameters.baseFreq;
                //         string shiftText = dopplerShift > 0 ? 
                //             $"+{dopplerShift:F1} Hz" : $"{dopplerShift:F1} Hz";
                            
                //         // Show doppler shift text next to the point
                //         #if UNITY_EDITOR
                //         UnityEditor.Handles.Label(pos + Vector3.up * 0.2f, shiftText);
                //         #endif
                //     }
                // }
            }
        }
    }
}