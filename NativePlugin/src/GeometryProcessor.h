// GeometryProcessor.h
#pragma once

#ifdef _WIN32
    #define EXPORT_API __declspec(dllexport)
#else
    #define EXPORT_API
#endif

#include <vector>
#include <cmath>
#include <cstdint>
#include <string>
#include <map>
#include <algorithm> // Needed for std::min

// --- KissFFT Includes ---
#include "kiss_fft.h" // Provides kiss_fft_cpx and kiss_fft_scalar definitions

// Forward declaration for the state struct pointer type is okay
struct kiss_fftr_state;
typedef struct kiss_fftr_state* kiss_fftr_cfg;


// --- Global Constants ---
const int MAX_OSCILLATORS = 256;


// --- Struct Definitions for the C API ---
struct ObjectGeometry {
    int objectId; float* vertices; int* triangles; float* faceNormals;
    float* faceCenters; float* triangleAreas; int triangleCount; float* transform;
};
struct ReferencePoint {
  float position[3];
  float normal[3];
};
struct AudioParameters {
    float amplitude; float baseFreq; float dopplerFreq; float sigma;
    float roundTripTime; float phase;
};
struct ProcessedPoint {
    int objectId; int triangleIndex; float position[3]; float normal[3];
    float velocity[3]; AudioParameters params; float facingFactor; float area;
};

// --- Internal Structs (Used only within C++) ---
struct RadarParameters {
    float pulseRepFreq; float speedOfSound; float pulseWidth;
    double currentTime; double lastFrameTime; double frameTime;
};

// Oscillator struct with inline method definitions
struct Oscillator {
    bool active; int objectId; int triangleIndex;
    float frequency; float targetFrequency; float amplitude; float targetAmplitude;
    float phase; float position[3]; float normal[3]; float roundTripTime; float sigma;

    // Inline definition for reset
    inline void reset() {
        active = false; objectId = -1; triangleIndex = -1; frequency = 0.0f; targetFrequency = 0.0f;
        amplitude = 0.0f; targetAmplitude = 0.0f; phase = 0.0f; roundTripTime = 0.0f; sigma = 0.05f; // Default sigma
        position[0] = position[1] = position[2] = 0.0f; normal[0] = normal[1] = normal[2] = 0.0f;
    }

    // ** Corrected inline definition for update **
    inline void update(float deltaTime) {
        if (!active) return; // Do nothing if inactive

        // --- Deactivation Logic ---
        // Check if the target amplitude is essentially zero
        if (targetAmplitude < 1e-4f) {
            // If target is zero, decay amplitude faster to ensure quick deactivation
            // Adjust the decay factor (e.g., 0.8f, 0.7f) if needed. Smaller values decay faster.
            amplitude *= 0.8f;

            // If amplitude has decayed enough, reset the oscillator completely
            if (amplitude < 1e-4f) {
                // Optional: Log deactivation for debugging
                // printf("[OSC DEBUG] Oscillator %d:%d deactivated (Target: %.4f, Current: %.4f)\n", objectId, triangleIndex, targetAmplitude, amplitude);
                reset(); // Call the reset method
                return;  // Exit early, no need for further smoothing if reset
            }
        } else {
            // --- Smoothing Logic (if target amplitude is non-zero) ---
            // Smooth amplitude towards the target
            // Ensure deltaTime is non-negative before using it in std::min
             float safeDeltaTime = std::max(0.0f, deltaTime);
             float lerpFactorAmp = std::min(10.0f * safeDeltaTime, 1.0f); // Amplitude smoothing factor
            amplitude = amplitude * (1.0f - lerpFactorAmp) + targetAmplitude * lerpFactorAmp;
        }

        // --- Frequency Smoothing (Always smooth if active) ---
        // Smooth frequency towards the target, regardless of amplitude target
        // Ensure deltaTime is non-negative before using it in std::min
        float safeDeltaTime = std::max(0.0f, deltaTime);
        float lerpFactorFreq = std::min(10.0f * safeDeltaTime, 1.0f); // Can use same or different factor
        frequency = frequency * (1.0f - lerpFactorFreq) + targetFrequency * lerpFactorFreq;
    }
};


// --- GeometryProcessor Implementation Class (Internal) ---
// (Class definition remains the same as in geometry_processor_h_update_4)
class GeometryProcessorImpl {
private:
    // Basic configuration
    int sampleRate;
    int blockSize;
    float currentMaxDistance;
    // Listener data
    float listenerPosition[3];
    float listenerVelocity[3];
    float listenerForward[3];
    float listenerUp[3];
    // Radar/pulse parameters
    RadarParameters radarParams;
    // Oscillator bank
    Oscillator oscillators[MAX_OSCILLATORS]; // Uses the struct defined above
    std::map<uint64_t, int> activeOscillatorMap;
    // HRTF and Convolution Data
    std::string hrtfPath;
    bool hrtfLoaded = false;
    struct MYSOFA_HRTF* hrtfData = nullptr;
    int hrirLength = 0;
    int hrtfMeasurements = 0;
    int hrtfSampleRate = 0;
    // KissFFT configurations
    kiss_fftr_cfg fftConfig = nullptr;
    kiss_fftr_cfg ifftConfig = nullptr;
    int fftSize = 0;
    int fftComplexSize = 0;
    // Buffers
    std::vector<kiss_fft_scalar> fftRealBuffer;
    std::vector<kiss_fft_cpx> fftCpxBuffer;
    std::vector<kiss_fft_cpx> fftCpxBuffer2;
    std::vector<std::vector<kiss_fft_cpx>> cachedLeftHrirFfts;
    std::vector<std::vector<kiss_fft_cpx>> cachedRightHrirFfts;
    std::vector<kiss_fft_scalar> overlapLeft;
    std::vector<kiss_fft_scalar> overlapRight;
    std::vector<kiss_fft_scalar> currentHrirLeft;
    std::vector<kiss_fft_scalar> currentHrirRight;
    std::vector<kiss_fft_scalar> monoBlockBuffer;
    // Working memory
    std::vector<ProcessedPoint> workingPoints;
    // Private Helper Methods (Declarations)
    float dot(const float* a, const float* b);
    float length(const float* v);
    float distSq(const float* a, const float* b);
    void normalize(float* v);
    void subtract(const float* a, const float* b, float* result);
    void cross(const float* a, const float* b, float* result);
    void transformPoint(const float* point, const float* matrix, float* result);
    void transformNormal(const float* normal, const float* matrix, float* result);
    uint64_t getOscillatorKey(int objectId, int triangleIndex);
    void calculateRelativeAzimuthElevation(const float* pointPosition, float& azimuth, float& elevation);
    int findNearestHrirIndex(float targetAzimuth, float targetElevation);
    bool precomputeHrirFfts();
    AudioParameters computeAudioParams(const ProcessedPoint& point);
    void assignPointsToOscillators(const ProcessedPoint* points, int pointCount, int maxOutputPoints);
    void cleanupResources();
public:
    // Constructor & Destructor
    GeometryProcessorImpl();
    ~GeometryProcessorImpl();
    // Public Interface Methods (Declarations)
    void initialize(int sr, int block);
    void setHrtfPath(const char* sofaFilePath);
    void setListenerOrientation(const float* forward, const float* up);
    void updateTime(float deltaTime);
    void setRadarParameters(float pulseRepFreq, float speedOfSound, float pulseWidth);
    void processGeometry(const ObjectGeometry* objects, int objectCount,
                         const float* listenerPos, const float* listenerFwdIgnored,
                         float maxDist, float reflectionRadius,
                         ProcessedPoint* outputPoints, int outputBufferSize, int* outputPointCount);
    void processReferencePoints(const ReferencePoint* refs, int refCount,
                         const float* listenerPos, float maxDist,
                         ProcessedPoint* outputPoints, int outputBufferSize, int* outputPointCount);
    void synthesizeAudio(const ProcessedPoint* pointsIgnored, int pointCountIgnored,
                         float* outputBuffer, int channelCount, int samplesPerChannel);
    void generateTimeFrequencyData(int timeSteps, int frequencyBins, float* mags, float* phases, float* timeRng, float* freqRng);
    void debugOscillators();
    void cleanup();
};

// --- C API Function Declarations ---
// (Keep these exactly as they were)
extern "C" {
    EXPORT_API void InitializeProcessor(int sampleRate, int blockSize);
    EXPORT_API void SetHrtfPath(const char* sofaFilePath);
    EXPORT_API void SetListenerOrientation(const float* listenerForward, const float* listenerUp);
    EXPORT_API void UpdateTime(float deltaTime);
    EXPORT_API void SetRadarParameters(float pulseRepFreq, float speedOfSound, float pulseWidth);
    EXPORT_API void ProcessObjectGeometry(
        const ObjectGeometry* objects, int objectCount, const float* listenerPosition,
        const float* listenerForward, float maxDistance, float reflectionRadius,
        ProcessedPoint* outputPoints, int* outputPointCount
    );
    EXPORT_API void SynthesizeAudio(
        const ProcessedPoint* points, int pointCount, float* outputBuffer,
        int channelCount, int samplesPerChannel
    );
    EXPORT_API void GetTimeFrequencyData(
        int timeSteps, int frequencyBins, float* outputMagnitudes, float* outputPhases,
        float* outputTimeRange, float* outputFreqRange
    );
    EXPORT_API void ProcessReferencePoints(
        const ReferencePoint* points, int pointCount,
        const float* listenerPosition,
        float maxDistance,
        ProcessedPoint* outputPoints,
        int outputBufferSize,
        int* outputPointCount
  );
    EXPORT_API void DebugOscillators();
    EXPORT_API void DebugBuffer(); // Deprecated
    EXPORT_API void Cleanup();
}; // extern "C"

