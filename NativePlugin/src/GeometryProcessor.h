// GeometryProcessor.h
#pragma once

#ifdef _WIN32
    #define EXPORT_API __declspec(dllexport)
#else
    #define EXPORT_API
#endif

#include <vector>
#include <cmath>

// Object geometry structure passed from Unity
struct ObjectGeometry {
    float* vertices;      // Array of vertex positions (x,y,z triplets)
    int* triangles;       // Array of triangle indices
    float* faceNormals;   // Array of face normal vectors
    float* faceCenters;   // Array of face center positions
    int triangleCount;    // Number of triangles
    float* transform;     // 4x4 world transform matrix
};

// Radar configuration parameters
struct RadarParameters {
    float pulseRepFreq;     // Pulse repetition frequency (Hz)
    float speedOfSound;     // Speed of sound in medium (m/s)
    float pulseWidth;  // Add this
    float currentTime;      // Current simulation time (s)
    float lastFrameTime;    // Time of last frame (s)
    float frameTime;        // Time elapsed since last frame (s)
};

struct ActiveEcho {
    float startTime;      // Absolute start time (radarParams.currentTime)
    float sigma;          // Envelope width
    float frequency;      // Echo frequency (with Doppler)
    float amplitude;      // Echo amplitude
    float targetAmplitude; // Target amplitude to smoothly approach
    float phase;          // Phase continuity
    float position[3];    // Position in 3D space
    bool isActive;        // Whether this echo is still alive
};

// Audio parameters for each processed point
struct AudioParameters {
    float amplitude;        // Base amplitude scaling
    float baseFreq;         // Base/carrier frequency
    float dopplerFreq;      // Frequency after Doppler shift
    float sigma;            // Width of Gaussian pulse envelope
    float roundTripTime;    // Time for pulse to reach target and return
    float phase;            // Phase offset
};

// Output structure for processed points
struct ProcessedPoint {
    float position[3];      // World space position
    float normal[3];        // World space normal
    float velocity[3];      // World space velocity
    AudioParameters params;
    float facingFactor;     // How directly this point faces the listener
};

extern "C" {
    // Initialize the processor with audio configuration
    EXPORT_API void InitializeProcessor(int sampleRate, int blockSize);
    
    // Update simulation time (call each frame)
    EXPORT_API void UpdateTime(float deltaTime);
    
    // Process geometry and compute audio parameters
    EXPORT_API void ProcessObjectGeometry(
        const ObjectGeometry* objects,    // Array of object geometries
        int objectCount,                  // Number of objects
        const float* listenerPosition,    // Listener's world position
        const float* listenerForward,     // Listener's forward vector
        float maxDistance,                // Maximum distance to process
        ProcessedPoint* outputPoints,     // Array to receive processed points
        int* outputPointCount             // Number of points processed
    );
    
    // Generate audio from processed points
    EXPORT_API void SynthesizeAudio(
        const ProcessedPoint* points,     // Array of processed points
        int pointCount,                   // Number of points
        float* outputBuffer,              // Audio output buffer
        int bufferSize                    // Size of output buffer
    );
    
    EXPORT_API void Cleanup();
};