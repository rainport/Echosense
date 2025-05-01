#include "GeometryProcessor.h" // Includes updated class definition from geometry_processor_h_update_4
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <cstring> // For std::memcpy, std::memset
#include <limits>   // For numeric_limits
#include <map>      // For oscillator matching
#include <stdexcept> // For exceptions

// --- Library Includes ---
#include "mysofa.h"      // For SOFA file handling
#include "kiss_fft.h"    // Core KissFFT
// Make sure kiss_fft.h includes or defines kiss_fft_scalar and kiss_fft_cpx
#include "kiss_fftr.h"   // KissFFT for real-valued signals

// Define M_PI if not available (e.g., on Windows with MSVC)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Simple logging helper (can be enhanced)
#define LOG_INFO(fmt, ...) printf("[INFO] GeometryProcessor: " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) printf("[WARN] GeometryProcessor: " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) printf("[ERROR] GeometryProcessor: " fmt "\n", ##__VA_ARGS__)


// --- Implementation of GeometryProcessorImpl Methods ---
// (Ensure GeometryProcessorImpl class definition exists in GeometryProcessor.h)

// Constructor
GeometryProcessorImpl::GeometryProcessorImpl()
    : sampleRate(44100), blockSize(1024), currentMaxDistance(10.0f),
      hrtfLoaded(false), hrtfData(nullptr), hrirLength(0), hrtfMeasurements(0), hrtfSampleRate(0),
      fftConfig(nullptr), ifftConfig(nullptr), fftSize(0), fftComplexSize(0)
{
    // Initialize radar parameters with defaults
    radarParams = { 2.0f, 343.0f, 0.05f, 0.0, 0.0, 0.0 }; // PRF, SoS, Sigma, CurrentTime, LastFrameTime, FrameTime
    // Initialize listener state
    listenerPosition[0] = listenerPosition[1] = listenerPosition[2] = 0.0f;
    listenerVelocity[0] = listenerVelocity[1] = listenerVelocity[2] = 0.0f;
    listenerForward[0] = listenerForward[1] = 0.0f; listenerForward[2] = 1.0f; // Default forward Z
    listenerUp[0] = listenerUp[2] = 0.0f; listenerUp[1] = 1.0f; // Default up Y
    // Reset all oscillators
    for (int i = 0; i < MAX_OSCILLATORS; i++) { oscillators[i].reset(); }
    activeOscillatorMap.clear();
    LOG_INFO("GeometryProcessorImpl constructed.");
}

// Destructor
GeometryProcessorImpl::~GeometryProcessorImpl() {
    cleanupResources();
    LOG_INFO("GeometryProcessorImpl destructed.");
}

// --- ** ADDED MISSING HELPER FUNCTION DEFINITIONS ** ---

float GeometryProcessorImpl::dot(const float* a, const float* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float GeometryProcessorImpl::length(const float* v) {
    return std::sqrt(dot(v, v));
}

float GeometryProcessorImpl::distSq(const float* a, const float* b) {
    float dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
    return dx*dx + dy*dy + dz*dz;
}

void GeometryProcessorImpl::normalize(float* v) {
    float len = length(v);
    if (len > 1e-6f) { // Avoid division by zero
        v[0] /= len;
        v[1] /= len;
        v[2] /= len;
    } else { // Handle zero vector case
        v[0] = 0.0f;
        v[1] = 0.0f;
        v[2] = 0.0f;
    }
}

void GeometryProcessorImpl::subtract(const float* a, const float* b, float* result) {
    result[0] = a[0]-b[0];
    result[1] = a[1]-b[1];
    result[2] = a[2]-b[2];
}

void GeometryProcessorImpl::cross(const float* a, const float* b, float* result) {
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

// Transform point by 4x4 matrix (column-major)
void GeometryProcessorImpl::transformPoint(const float* point, const float* matrix, float* result) {
    float x = point[0], y = point[1], z = point[2];
    // Calculate w component for perspective division
    float w = matrix[3] * x + matrix[7] * y + matrix[11] * z + matrix[15];
    // Avoid division by zero or very small numbers
    w = (std::abs(w) < 1e-6f) ? 1.0f : w;
    // Apply transformation and perspective divide
    result[0] = (matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12]) / w;
    result[1] = (matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13]) / w;
    result[2] = (matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14]) / w;
}

// Transform normal by 3x3 upper-left part of 4x4 matrix (column-major)
void GeometryProcessorImpl::transformNormal(const float* normal, const float* matrix, float* result) {
    float x = normal[0], y = normal[1], z = normal[2];
    // Use only the rotation/scale part of the matrix
    result[0] = matrix[0] * x + matrix[4] * y + matrix[8] * z;
    result[1] = matrix[1] * x + matrix[5] * y + matrix[9] * z;
    result[2] = matrix[2] * x + matrix[6] * y + matrix[10] * z;
    // Re-normalize the transformed normal as scaling might affect length
    normalize(result);
}

// Helper to create a unique key for the oscillator map
uint64_t GeometryProcessorImpl::getOscillatorKey(int objectId, int triangleIndex) {
     // Combine objectId and triangleIndex into a 64-bit key
     // Assuming objectId fits in 32 bits and triangleIndex fits in 32 bits.
     return (static_cast<uint64_t>(objectId) << 32) | static_cast<uint32_t>(triangleIndex);
}

// --- End of Added Helper Function Definitions ---


// Cleanup allocated resources
void GeometryProcessorImpl::cleanupResources() {
     LOG_INFO("Cleaning up GeometryProcessor resources...");
     hrtfLoaded = false; hrtfPath.clear();
     if (hrtfData) { mysofa_free(hrtfData); hrtfData = nullptr; LOG_INFO("  libmysofa HRTF data freed."); }
     if (fftConfig) { kiss_fft_free(fftConfig); fftConfig = nullptr; LOG_INFO("  KissFFT forward config freed."); }
     if (ifftConfig) { kiss_fft_free(ifftConfig); ifftConfig = nullptr; LOG_INFO("  KissFFT inverse config freed."); }

     // Clear vectors and release memory
     #define CLEAR_VEC(v) v.clear(); v.shrink_to_fit();
     CLEAR_VEC(fftRealBuffer); CLEAR_VEC(fftCpxBuffer); CLEAR_VEC(fftCpxBuffer2);
     // Clear cached FFTs
     for(auto& vec : cachedLeftHrirFfts) { CLEAR_VEC(vec); }
     for(auto& vec : cachedRightHrirFfts) { CLEAR_VEC(vec); }
     CLEAR_VEC(cachedLeftHrirFfts); CLEAR_VEC(cachedRightHrirFfts);
     // Clear other buffers
     CLEAR_VEC(overlapLeft); CLEAR_VEC(overlapRight);
     CLEAR_VEC(currentHrirLeft); CLEAR_VEC(currentHrirRight);
     CLEAR_VEC(monoBlockBuffer);
     #undef CLEAR_VEC
     LOG_INFO("  Buffers cleared.");
     activeOscillatorMap.clear(); LOG_INFO("  Oscillator map cleared.");
}

// Helper function to precompute HRIR FFTs
bool GeometryProcessorImpl::precomputeHrirFfts() {
    // Check prerequisites
    if (!hrtfLoaded || !hrtfData || !fftConfig || hrtfMeasurements <= 0 || hrirLength <= 0 || fftComplexSize <= 0) {
        LOG_ERROR("Cannot precompute HRIR FFTs: Prerequisites not met (hrtfLoaded=%d, hrtfData=%p, fftConfig=%p, M=%d, N=%d, fftCpxSize=%d).",
                  hrtfLoaded, (void*)hrtfData, (void*)fftConfig, hrtfMeasurements, hrirLength, fftComplexSize);
        return false;
    }

    LOG_INFO("Precomputing HRIR FFTs for %d measurements...", hrtfMeasurements);
    try {
        // Ensure cache vectors are clear before resizing
        cachedLeftHrirFfts.clear();
        cachedRightHrirFfts.clear();
        // Resize the outer vectors
        cachedLeftHrirFfts.resize(hrtfMeasurements);
        cachedRightHrirFfts.resize(hrtfMeasurements);

        const float* hrir_base_ptr = hrtfData->DataIR.values;
        int R = hrtfData->R; // Should be 2
        int N = hrirLength;
        size_t total_ir_elements = (size_t)hrtfMeasurements * R * N;

        // Ensure temporary real buffer is correctly sized
        if (fftRealBuffer.size() != fftSize) fftRealBuffer.resize(fftSize);

        for (int i = 0; i < hrtfMeasurements; ++i) {
            // Resize inner vectors for this measurement's FFT data
            cachedLeftHrirFfts[i].resize(fftComplexSize);
            cachedRightHrirFfts[i].resize(fftComplexSize);

            // Calculate offsets and perform bounds check
            size_t offset_l = (size_t)i * R * N + 0 * N; // Left channel offset
            size_t offset_r = (size_t)i * R * N + 1 * N; // Right channel offset
            if (offset_l + N > total_ir_elements || offset_r + N > total_ir_elements) {
                LOG_ERROR("HRIR offset out of bounds during precomputation for measurement index %d.", i);
                cachedLeftHrirFfts.clear(); cachedRightHrirFfts.clear(); // Clear potentially partial cache
                return false;
            }
            const float* hrir_l_ptr = hrir_base_ptr + offset_l;
            const float* hrir_r_ptr = hrir_base_ptr + offset_r;

            // Compute FFT for Left HRIR
            std::memset(fftRealBuffer.data(), 0, fftSize * sizeof(kiss_fft_scalar)); // Zero pad
            std::copy(hrir_l_ptr, hrir_l_ptr + N, fftRealBuffer.begin()); // Copy HRIR data
            kiss_fftr(fftConfig, fftRealBuffer.data(), cachedLeftHrirFfts[i].data()); // Perform FFT

            // Compute FFT for Right HRIR
            std::memset(fftRealBuffer.data(), 0, fftSize * sizeof(kiss_fft_scalar)); // Zero pad
            std::copy(hrir_r_ptr, hrir_r_ptr + N, fftRealBuffer.begin()); // Copy HRIR data
            kiss_fftr(fftConfig, fftRealBuffer.data(), cachedRightHrirFfts[i].data()); // Perform FFT
        }
    } catch (const std::exception& e) {
        LOG_ERROR("!!! Exception during HRIR FFT precomputation: %s", e.what());
        cachedLeftHrirFfts.clear(); cachedRightHrirFfts.clear(); // Clean up partial cache on error
        return false;
    }

    LOG_INFO("HRIR FFT precomputation complete.");
    return true;
}

// Initialize or re-initialize the processor
void GeometryProcessorImpl::initialize(int sr, int block) {
    LOG_INFO("Initializing GeometryProcessor: SR=%d, BlockSize=%d", sr, block);
    // Cleanup if already initialized
    if (hrtfLoaded || fftConfig || ifftConfig || hrtfData) {
         LOG_INFO("  Re-initializing: Cleaning up previous resources...");
         cleanupResources();
    }
    // Set basic parameters
    sampleRate = sr; blockSize = block;
    // Reset state
    for (int i = 0; i < MAX_OSCILLATORS; i++) { oscillators[i].reset(); }
    activeOscillatorMap.clear();
    radarParams.currentTime = 0.0; radarParams.lastFrameTime = 0.0; radarParams.frameTime = 0.0;

    // Attempt to load SOFA file if path is provided
    if (!hrtfPath.empty()) {
        LOG_INFO("Attempting to load HRTF data from: %s", hrtfPath.c_str());
        int err = 0;
        hrtfData = mysofa_load(hrtfPath.c_str(), &err); // Load SOFA file

        // Check for loading errors
        if (!hrtfData || err != MYSOFA_OK) {
            LOG_ERROR("!!! Error loading SOFA file: %s (libmysofa error code: %d)", hrtfPath.c_str(), err);
            hrtfData = nullptr; // Ensure hrtfData is null on failure
        } else {
            // Validate the loaded SOFA data
            bool valid = true;
            // Check Sample Rate
            if (!hrtfData->DataSamplingRate.values || hrtfData->DataSamplingRate.elements < 1) { LOG_ERROR("!!! Error: SOFA missing DataSamplingRate."); valid = false; }
            else { hrtfSampleRate = (int)hrtfData->DataSamplingRate.values[0]; if (hrtfSampleRate != sampleRate) { LOG_WARN("!!! Warning: HRTF SR (%d) != Engine SR (%d). Disabling HRTF.", hrtfSampleRate, sampleRate); valid = false; } }
            // Check other essential attributes
            hrirLength = hrtfData->N; hrtfMeasurements = hrtfData->M;
            if (hrirLength <= 0) { LOG_ERROR("!!! Error: SOFA invalid HRIR length (N=%d).", hrirLength); valid = false; }
            if (hrtfMeasurements <= 0) { LOG_ERROR("!!! Error: SOFA invalid measurement count (M=%d).", hrtfMeasurements); valid = false; }
            if (hrtfData->R != 2) { LOG_WARN("!!! Warning: SOFA expected R=2, found R=%d. Disabling HRTF.", hrtfData->R); valid = false; }
            if (!hrtfData->SourcePosition.values) { LOG_ERROR("!!! Error: SOFA missing source position data."); valid = false; }
            if (!hrtfData->DataIR.values) { LOG_ERROR("!!! Error: SOFA missing impulse response data."); valid = false; }

            // If validation failed, cleanup and disable HRTF
            if (!valid) {
                mysofa_free(hrtfData); hrtfData = nullptr; hrtfLoaded = false;
                LOG_WARN("SOFA validation failed. Spatialization disabled.");
            } else {
                // Validation passed, proceed with setup
                hrtfLoaded = true;
                LOG_INFO("SOFA loaded: SR=%d, N=%d (HRIR Length), R=%d, M=%d (Measurements)", hrtfSampleRate, hrirLength, hrtfData->R, hrtfMeasurements);

                // Initialize KissFFT
                fftSize = 1; while (fftSize < (blockSize + hrirLength - 1)) fftSize *= 2; // Calculate FFT size
                fftComplexSize = fftSize / 2 + 1;
                LOG_INFO("FFT Size: %d (Complex Bins: %d)", fftSize, fftComplexSize);
                // Allocate FFT configurations
                fftConfig = kiss_fftr_alloc(fftSize, 0, nullptr, nullptr); // Forward
                ifftConfig = kiss_fftr_alloc(fftSize, 1, nullptr, nullptr); // Inverse

                // Check if FFT allocation succeeded
                if (!fftConfig || !ifftConfig) {
                    LOG_ERROR("!!! Error: KissFFT config allocation failed.");
                    cleanupResources(); // Cleanup everything allocated so far
                    hrtfLoaded = false; // Disable HRTF
                } else {
                    LOG_INFO("KissFFT configurations allocated.");
                    // Allocate necessary buffers
                    try {
                        fftRealBuffer.resize(fftSize);
                        fftCpxBuffer.resize(fftComplexSize);
                        fftCpxBuffer2.resize(fftComplexSize);
                        overlapLeft.resize(hrirLength - 1, 0.0f);
                        overlapRight.resize(hrirLength - 1, 0.0f);
                        currentHrirLeft.resize(hrirLength); // Still needed for precomputation step
                        currentHrirRight.resize(hrirLength);
                        monoBlockBuffer.resize(blockSize);
                        LOG_INFO("Core FFT Buffers allocated.");

                        // Precompute HRIR FFTs and store them in the cache
                        if (!precomputeHrirFfts()) {
                             LOG_ERROR("Failed to precompute HRIR FFTs. Disabling HRTF.");
                             cleanupResources(); // Cleanup everything if precomputation fails
                             hrtfLoaded = false;
                        }

                    } catch (const std::exception& e) {
                        LOG_ERROR("!!! Exception during buffer allocation: %s", e.what());
                        cleanupResources(); // Cleanup on allocation error
                        hrtfLoaded = false;
                    }
                }
            }
        }
    } else {
        // No HRTF path provided
        LOG_WARN("HRTF path not set. Spatialization disabled.");
        hrtfLoaded = false;
    }
    LOG_INFO("Initialization complete. HRTF Loaded: %s", hrtfLoaded ? "Yes" : "No");
}

// Set the path for the SOFA file
void GeometryProcessorImpl::setHrtfPath(const char* sofaFilePath) {
    std::string newPath = (sofaFilePath && strlen(sofaFilePath) > 0) ? sofaFilePath : "";
    if (newPath != hrtfPath) {
        hrtfPath = newPath;
        LOG_INFO("HRTF path %s.", hrtfPath.empty() ? "cleared" : ("set to: " + hrtfPath).c_str());
        // If already initialized, resources need to be cleaned up before re-initializing
        if (hrtfLoaded || hrtfData || fftConfig) {
             LOG_INFO("  HRTF path changed after initialization. Cleaning up resources. Call InitializeProcessor again to apply.");
             cleanupResources(); // Cleanup existing resources
        }
        hrtfLoaded = false; // Mark as not loaded until next initialization
    }
}

// Set listener orientation vectors
void GeometryProcessorImpl::setListenerOrientation(const float* forward, const float* up) {
    if (forward) std::copy(forward, forward + 3, listenerForward);
    if (up) std::copy(up, up + 3, listenerUp);
    // Ensure vectors are normalized
    normalize(listenerForward);
    normalize(listenerUp);
}

// Update internal simulation time
void GeometryProcessorImpl::updateTime(float deltaTime) {
    // Prevent negative or excessively large delta times which might cause issues
    deltaTime = std::max(0.0f, std::min(deltaTime, 0.1f)); // Clamp delta time e.g., 0 to 100ms
    radarParams.lastFrameTime = radarParams.currentTime;
    radarParams.currentTime += static_cast<double>(deltaTime);
    radarParams.frameTime = static_cast<double>(deltaTime);
    // Note: Oscillator smoothing/update now happens at the end of assignPointsToOscillators
}

// Set radar simulation parameters
void GeometryProcessorImpl::setRadarParameters(float pulseRepFreq, float speedOfSound, float pulseWidth) {
    radarParams.pulseRepFreq = std::max(0.1f, pulseRepFreq); // Avoid zero PRF
    radarParams.speedOfSound = std::max(1.0f, speedOfSound); // Avoid zero speed
    radarParams.pulseWidth = std::max(0.001f, pulseWidth);   // Avoid zero/negative sigma
}

// Process scene geometry to find reflection points
void GeometryProcessorImpl::processGeometry(const ObjectGeometry* objects, int objectCount,
                const float* listenerPos, const float* listenerFwdIgnored,
                float maxDist, float reflectionRadius,
                ProcessedPoint* outputPoints, int outputBufferSize, int* outputPointCount)
{
    // Validate input pointers and buffer size
    if (!outputPoints || !outputPointCount || outputBufferSize <= 0) {
        LOG_ERROR("Invalid output parameters in processGeometry (outputPoints=%p, outputPointCount=%p, bufferSize=%d).",
                  (void*)outputPoints, (void*)outputPointCount, outputBufferSize);
        if(outputPointCount) *outputPointCount = 0;
        return;
    }
    *outputPointCount = 0; // Initialize output count

    // Update Listener State (Position and Velocity)
    currentMaxDistance = maxDist;
    if (radarParams.frameTime > 1e-9) { // Check against a very small number
        for (int i = 0; i < 3; i++) {
            listenerVelocity[i] = (listenerPos[i] - listenerPosition[i]) / static_cast<float>(radarParams.frameTime);
        }
    } else { // If deltaTime is zero or negative, assume zero velocity
        for (int i = 0; i < 3; i++) { listenerVelocity[i] = 0.0f; }
    }
    std::copy(listenerPos, listenerPos + 3, listenerPosition); // Store current position

    // Process Geometry from input objects
    workingPoints.clear(); // Clear the temporary list for this frame
    int currentOutputIndex = 0;
    const int maxPointsToGenerate = outputBufferSize; // Limit based on C# buffer

    for (int objIdx = 0; objIdx < objectCount; ++objIdx) {
        const ObjectGeometry& geometry = objects[objIdx];
        // Validate essential pointers for this object
        if (!geometry.vertices || !geometry.triangles || !geometry.faceCenters || !geometry.faceNormals || !geometry.transform || geometry.triangleCount <= 0) {
            // LOG_WARN("Skipping object %d due to invalid geometry data pointers or zero triangles.", geometry.objectId);
            continue;
        }

        for (int triIdx = 0; triIdx < geometry.triangleCount; ++triIdx) {
             // Check if we have reached the capacity of the output buffer
             if (currentOutputIndex >= maxPointsToGenerate) break;

             // Get local space data (safer access with checks if needed)
             const float* localCenter = &geometry.faceCenters[triIdx * 3];
             const float* localNormal = &geometry.faceNormals[triIdx * 3];

             // Transform to world space
             float worldCenter[3], worldNormal[3];
             transformPoint(localCenter, geometry.transform, worldCenter);
             transformNormal(localNormal, geometry.transform, worldNormal);

             // Calculate distance and direction from listener
             float fromListener[3]; subtract(worldCenter, listenerPosition, fromListener);
             float distance = length(fromListener);

             // --- Culling checks ---
             if (distance > currentMaxDistance || distance < 1e-3f) continue; // Distance cull
             float normalizedFromListener[3]; std::copy(fromListener, fromListener + 3, normalizedFromListener); normalize(normalizedFromListener);
             float toListener[3] = {-normalizedFromListener[0], -normalizedFromListener[1], -normalizedFromListener[2]};
             float normalDotListener = dot(worldNormal, toListener);
             if (normalDotListener <= 1e-6f) continue; // Back-face cull (use small epsilon)

             // Reflection radius check
             float sinThetaSq = 1.0f - normalDotListener * normalDotListener;
             float sinTheta = std::sqrt(std::max(0.0f, sinThetaSq)); // Ensure non-negative argument for sqrt
             float reflectionValue = distance * sinTheta;
             float safeReflectionRadius = std::max(0.01f, reflectionRadius);
             if (reflectionValue > safeReflectionRadius) continue; // Cone cull

             // --- Point Passed Culling ---
             ProcessedPoint point;
             point.objectId = geometry.objectId; point.triangleIndex = triIdx;
             std::copy(worldCenter, worldCenter + 3, point.position);
             std::copy(worldNormal, worldNormal + 3, point.normal);
             point.velocity[0] = point.velocity[1] = point.velocity[2] = 0.0f; // Assume static world
             point.facingFactor = std::max(0.0f, 1.0f - (reflectionValue / safeReflectionRadius));
             point.area = (geometry.triangleAreas && triIdx < geometry.triangleCount) // Check area pointer and index
                          ? std::max(0.01f, std::min(geometry.triangleAreas[triIdx], 10.0f))
                          : 1.0f; // Default area if missing or index out of bounds
             point.params = computeAudioParams(point); // Calculate audio params

             workingPoints.push_back(point); // Add to temporary list
             currentOutputIndex++;
        }
         // If buffer is full, stop processing more objects
         if (currentOutputIndex >= maxPointsToGenerate) break;
    }

    // Update oscillator states based on the detected points
    assignPointsToOscillators(workingPoints.data(), workingPoints.size(), maxPointsToGenerate);

    // Copy the processed points (up to the buffer size) to the output array for C#
    int numPointsToCopy = workingPoints.size(); // Already limited by maxPointsToGenerate
    if (numPointsToCopy > 0) {
        std::copy(workingPoints.begin(), workingPoints.end(), outputPoints);
    }
    *outputPointCount = numPointsToCopy; // Set the actual number of points written
}

// Assign detected points to oscillators, managing activation/deactivation
void GeometryProcessorImpl::assignPointsToOscillators(const ProcessedPoint* points, int pointCount, int maxOutputPoints) {
     // Validate input pointer
     if (!points && pointCount > 0) {
          LOG_ERROR("assignPointsToOscillators called with null points array but pointCount > 0.");
          pointCount = 0;
     }
     // Ensure pointCount doesn't exceed the actual buffer size it came from (safety)
     pointCount = std::min(pointCount, maxOutputPoints);

     std::vector<bool> oscillator_updated_this_frame(MAX_OSCILLATORS, false);

     // --- Match and Update Oscillators ---
     for (int p = 0; p < pointCount; ++p) {
         const ProcessedPoint& point = points[p];
         uint64_t key = getOscillatorKey(point.objectId, point.triangleIndex);
         int targetOscIndex = -1;

         // 1. Try to find existing oscillator via map
         auto map_it = activeOscillatorMap.find(key);
         if (map_it != activeOscillatorMap.end()) {
             targetOscIndex = map_it->second;
             // Validate map entry
             if (targetOscIndex < 0 || targetOscIndex >= MAX_OSCILLATORS || !oscillators[targetOscIndex].active ||
                 oscillators[targetOscIndex].objectId != point.objectId || oscillators[targetOscIndex].triangleIndex != point.triangleIndex)
             {
                 activeOscillatorMap.erase(map_it); // Remove bad entry
                 targetOscIndex = -1; // Force search
             }
         }

         // 2. If no match, find best available slot
         if (targetOscIndex == -1) {
             int bestInactive = -1; int quietestActive = -1; float minAmp = std::numeric_limits<float>::max();
             for (int i = 0; i < MAX_OSCILLATORS; ++i) {
                 if (!oscillators[i].active) { bestInactive = i; break; } // Found inactive
                 else if (oscillators[i].amplitude < minAmp) { minAmp = oscillators[i].amplitude; quietestActive = i; } // Track quietest active
             }
             targetOscIndex = (bestInactive != -1) ? bestInactive : quietestActive; // Prefer inactive

             // If reusing an active oscillator, remove its old map entry
             if (targetOscIndex != -1 && oscillators[targetOscIndex].active) {
                  uint64_t oldKey = getOscillatorKey(oscillators[targetOscIndex].objectId, oscillators[targetOscIndex].triangleIndex);
                  activeOscillatorMap.erase(oldKey);
             }
         }

         // 3. Update the chosen oscillator
         if (targetOscIndex >= 0 && targetOscIndex < MAX_OSCILLATORS) {
             Oscillator& osc = oscillators[targetOscIndex];
             uint64_t currentKey = getOscillatorKey(point.objectId, point.triangleIndex);

             // Activate/reset if it was inactive or stolen
             if (!osc.active || osc.objectId != point.objectId || osc.triangleIndex != point.triangleIndex) {
                 bool wasActive = osc.active; // Check if it was stolen
                 osc.reset();
                 osc.objectId = point.objectId; osc.triangleIndex = point.triangleIndex;
                 osc.frequency = point.params.dopplerFreq; // Start at target freq
                 osc.phase = 0.0f; // Reset phase on activation/steal
                 osc.active = true;
                 activeOscillatorMap[currentKey] = targetOscIndex; // Add/update map entry
             }

             // Update parameters from the current point
             std::copy(point.position, point.position + 3, osc.position);
             std::copy(point.normal, point.normal + 3, osc.normal);
             osc.roundTripTime = point.params.roundTripTime;
             osc.sigma = point.params.sigma;
             osc.targetFrequency = point.params.dopplerFreq;
             osc.targetAmplitude = point.params.amplitude; // Set target amplitude

             oscillator_updated_this_frame[targetOscIndex] = true; // Mark as updated
         } else {
              LOG_WARN("Could not find available oscillator index for point %d:%d!", point.objectId, point.triangleIndex);
         }
     }

     // --- Handle Non-Updated and Update All Oscillators ---
     for (int i = 0; i < MAX_OSCILLATORS; ++i) {
         // If oscillator is active but wasn't updated by a point this frame, start fade out
         if (oscillators[i].active && !oscillator_updated_this_frame[i]) {
             oscillators[i].targetAmplitude = 0.0f; // Set target to zero
             uint64_t key = getOscillatorKey(oscillators[i].objectId, oscillators[i].triangleIndex);
             activeOscillatorMap.erase(key); // Remove from map
         }

         // Update smoothing (lerp towards target) and check for deactivation
         if (radarParams.frameTime >= 0) { // Only update if time delta is non-negative
            oscillators[i].update(static_cast<float>(radarParams.frameTime));
         }

         // Safety: Ensure inactive oscillators are definitely removed from map and reset IDs
         if (!oscillators[i].active) {
             // If objectId is valid, try removing potential lingering map entry
             if (oscillators[i].objectId != -1 && oscillators[i].triangleIndex != -1) {
                  uint64_t key = getOscillatorKey(oscillators[i].objectId, oscillators[i].triangleIndex);
                  activeOscillatorMap.erase(key);
             }
             // Ensure IDs are reset after deactivation
             oscillators[i].objectId = -1;
             oscillators[i].triangleIndex = -1;
         }
     }
}


// Compute audio parameters for a given reflection point
AudioParameters GeometryProcessorImpl::computeAudioParams(const ProcessedPoint& point) {
     AudioParameters params;
     // Calculate distance from listener to point
     float directionToPoint[3]; subtract(point.position, listenerPosition, directionToPoint);
     float distance = length(directionToPoint);
     float safeDistance = std::max(distance, 0.1f); // Avoid division by zero

     // Amplitude based on facing factor, area, distance
     float areaContribution = std::max(std::sqrt(point.area) * 0.5f, 0.1f);
     params.amplitude = point.facingFactor * (areaContribution / safeDistance) * 2.0f; // Adjust scaling factor as needed
     params.amplitude = std::min(params.amplitude, 1.0f); // Clamp max amplitude

     // Base frequency
     params.baseFreq = 440.0f; // A4 note

     // Doppler shift
     float directionNormalized[3]; std::copy(directionToPoint, directionToPoint + 3, directionNormalized); normalize(directionNormalized);
     float listenerRadialVelocity = dot(listenerVelocity, directionNormalized); // Listener's velocity towards/away from point
     float sourceRadialVelocity = 0.0f; // Assume static geometry for now dot(point.velocity, directionNormalized);
     float relativeVelocity = listenerRadialVelocity - sourceRadialVelocity;
     float speedOfSound = radarParams.speedOfSound;
     float dopplerFactor = 1.0f;
     // Check for division by zero or near-zero denominator
     if (std::abs(speedOfSound + relativeVelocity) > 1e-6f) {
         dopplerFactor = speedOfSound / (speedOfSound + relativeVelocity);
     } else {
         // Handle case where relative velocity nearly cancels speed of sound (approaching at SoS)
         dopplerFactor = 10.0f; // Arbitrarily large factor, frequency will be clamped later
     }
     params.dopplerFreq = params.baseFreq * dopplerFactor;
     // Clamp frequency to a reasonable range (e.g., half to double base)
     params.dopplerFreq = std::max(params.baseFreq * 0.5f, std::min(params.dopplerFreq, params.baseFreq * 2.0f));

     // Pulse width (sigma)
     params.sigma = radarParams.pulseWidth;

     // Round trip time
     params.roundTripTime = (speedOfSound > 1e-6f) ? (2.0f * distance / speedOfSound) : 0.0f;

     // Phase initialization (oscillator manages its own phase accumulation)
     params.phase = 0.0f;

     return params;
}

// Calculate azimuth/elevation relative to listener
void GeometryProcessorImpl::calculateRelativeAzimuthElevation(const float* pointPosition, float& azimuth, float& elevation) {
    float direction[3]; subtract(pointPosition, listenerPosition, direction);
    float dist = length(direction);
    if (dist < 1e-6f) { azimuth = 0.0f; elevation = 0.0f; return; } // Point is at listener position
    normalize(direction); // Unit vector from listener to point

    float listenerRight[3]; cross(listenerUp, listenerForward, listenerRight); normalize(listenerRight); // Calculate right vector

    // Project direction onto listener's local axes
    float projForward = dot(direction, listenerForward);
    float projRight = dot(direction, listenerRight);
    float projUp = dot(direction, listenerUp);

    // Calculate azimuth (angle in horizontal plane) and elevation
    azimuth = std::atan2(projRight, projForward); // Radians, -pi to +pi
    projUp = std::max(-1.0f, std::min(1.0f, projUp)); // Clamp for asin
    elevation = std::asin(projUp); // Radians, -pi/2 to +pi/2

    // Convert to degrees
    azimuth = azimuth * 180.0f / M_PI;
    elevation = elevation * 180.0f / M_PI;

    // --- AZIMUTH CORRECTION ---
    // Flip the sign of the azimuth to potentially match SOFA convention
    // where +90 is typically RIGHT, whereas atan2 often gives +90 LEFT.
    azimuth = -azimuth;
    // --- END AZIMUTH CORRECTION ---


    // Adjust azimuth to 0-360 range (common SOFA convention: 0 front, 90 right, 180 back, 270 left)
    if (azimuth < 0) azimuth += 360.0f;
}

// Find nearest HRIR measurement index in SOFA data
int GeometryProcessorImpl::findNearestHrirIndex(float targetAzimuth, float targetElevation) {
    if (!hrtfLoaded || !hrtfData || !hrtfData->SourcePosition.values || hrtfMeasurements <= 0) return -1; // Not ready

    int nearestIndex = -1;
    float minDistanceSq = std::numeric_limits<float>::max();
    const float* positions = hrtfData->SourcePosition.values;
    int numCoords = hrtfData->SourcePosition.elements / hrtfMeasurements; // Coords per measurement (should be >= 2)
    if (numCoords < 2) { LOG_ERROR("HRTF SourcePosition data has < 2 coordinates per measurement."); return -1; }

    // Iterate through all measurements
    for (int i = 0; i < hrtfMeasurements; ++i) {
        float az = positions[i * numCoords + 0]; // Azimuth from SOFA
        float el = positions[i * numCoords + 1]; // Elevation from SOFA

        // Calculate angular difference, handling azimuth wrap-around (-180 to +180)
        float deltaAz = targetAzimuth - az;
        while (deltaAz <= -180.0f) deltaAz += 360.0f;
        while (deltaAz > 180.0f) deltaAz -= 360.0f;
        float deltaEl = targetElevation - el;

        // Squared distance in Az-El space
        float distanceSq = deltaAz * deltaAz + deltaEl * deltaEl;

        // Update if this is the closest found so far
        if (distanceSq < minDistanceSq) {
            minDistanceSq = distanceSq;
            nearestIndex = i;
        }
    }
    return nearestIndex;
}

// Synthesize stereo audio output
void GeometryProcessorImpl::synthesizeAudio(const ProcessedPoint* pointsIgnored, int pointCountIgnored,
                     float* outputBuffer, int channelCount, int samplesPerChannel)
{
    // --- Basic Validation ---
    if (!outputBuffer) { LOG_ERROR("synthesizeAudio outputBuffer is NULL."); return; }
    if (channelCount != 2) { LOG_ERROR("synthesizeAudio requires channelCount=2."); std::memset(outputBuffer, 0, samplesPerChannel * channelCount * sizeof(float)); return; }
    if (samplesPerChannel != blockSize) { LOG_ERROR("synthesizeAudio samplesPerChannel (%d) != blockSize (%d).", samplesPerChannel, blockSize); std::memset(outputBuffer, 0, samplesPerChannel * channelCount * sizeof(float)); return; }

    int bufferSize = samplesPerChannel * channelCount; // Total floats
    std::memset(outputBuffer, 0, bufferSize * sizeof(float)); // Zero the output buffer

    // --- Determine if HRTF should be used (check cache readiness) ---
    bool useHrtf = hrtfLoaded && fftConfig && ifftConfig && hrirLength > 0 && blockSize > 0 &&
                   !cachedLeftHrirFfts.empty() && !cachedRightHrirFfts.empty() &&
                   cachedLeftHrirFfts.size() == hrtfMeasurements && cachedRightHrirFfts.size() == hrtfMeasurements;

    // --- Timing Calculation ---
    double blockStartTime = radarParams.currentTime; // Time at the start of this block generation
    float pulseInterval = (radarParams.pulseRepFreq > 1e-6f) ? (1.0f / radarParams.pulseRepFreq) : 1.0f;
    // Find the start time of the most recent pulse before this block
    double lastPulseTime = std::floor(blockStartTime / pulseInterval) * pulseInterval;
    float sampleDuration = 1.0f / static_cast<float>(sampleRate);

    // --- MONO Path (if HRTF not used or not ready) ---
    if (!useHrtf) {
        for (int i = 0; i < samplesPerChannel; ++i) {
            double timeInBlock = i * sampleDuration;
            double t = blockStartTime + timeInBlock; // Absolute time for this sample
            float monoSample = 0.0f;

            for (int j = 0; j < MAX_OSCILLATORS; ++j) {
                if (!oscillators[j].active || oscillators[j].amplitude < 1e-5f) continue; // Skip inactive/quiet

                const Oscillator& osc = oscillators[j];
                double echoTime = lastPulseTime + osc.roundTripTime;
                if (t < echoTime) continue; // Echo hasn't arrived yet

                double timeSinceEcho = t - echoTime;
                // Optimization: skip if echo is long past
                if (timeSinceEcho > 10.0 * osc.sigma) continue;

                // Gaussian envelope
                float sigmaSq = osc.sigma * osc.sigma; if (sigmaSq < 1e-12f) sigmaSq = 1e-12f; // Avoid division by zero
                float envelope = std::exp(-(timeSinceEcho * timeSinceEcho) / (2.0f * sigmaSq));
                // Fade-in
                float fadeIn = std::min(1.0f, static_cast<float>(timeSinceEcho) / 0.005f); // 5ms fade

                // Calculate sample value using *current* phase + time offset within block
                double phaseForSample = osc.phase + 2.0 * M_PI * osc.frequency * timeInBlock;
                monoSample += osc.amplitude * envelope * fadeIn * std::sin(static_cast<float>(phaseForSample));
            }
             // Simple limiter
             monoSample = std::max(-0.95f, std::min(0.95f, monoSample));
             // Write to output buffer (interleaved)
            outputBuffer[i * 2] = monoSample;     // Left channel
            outputBuffer[i * 2 + 1] = monoSample; // Right channel
        }

        // Update oscillator persistent phase for the start of the *next* block
        double phaseIncrementPerBlock = 2.0 * M_PI * blockSize * sampleDuration;
        for (int j = 0; j < MAX_OSCILLATORS; ++j) {
             if (oscillators[j].active) {
                 oscillators[j].phase = std::fmod(oscillators[j].phase + phaseIncrementPerBlock * oscillators[j].frequency, 2.0 * M_PI);
                 if(oscillators[j].phase < 0) oscillators[j].phase += 2.0 * M_PI; // Ensure positive phase
             }
        }
        return; // Finished mono processing
    }

    // --- HRTF STEREO Path (Using Cached FFTs) ---
    // Safety check buffer sizes again
    if (monoBlockBuffer.size() != blockSize || fftRealBuffer.size() != fftSize || fftCpxBuffer.size() != fftComplexSize || fftCpxBuffer2.size() != fftComplexSize || overlapLeft.size() != (hrirLength - 1)) {
        LOG_ERROR("HRTF buffers not correctly sized! Falling back to silence.");
        std::memset(outputBuffer, 0, bufferSize * sizeof(float)); return;
    }
    float fftScale = 1.0f / static_cast<float>(fftSize); // Scaling for iFFT

    // Process each active oscillator
    for (int oscIndex = 0; oscIndex < MAX_OSCILLATORS; ++oscIndex) {
        if (!oscillators[oscIndex].active || oscillators[oscIndex].amplitude < 1e-5f) continue; // Skip inactive/quiet
        Oscillator& osc = oscillators[oscIndex]; // Need non-const ref to update phase

        // 1. Get HRIR Index
        float azimuth = 0.0f, elevation = 0.0f;
        calculateRelativeAzimuthElevation(osc.position, azimuth, elevation);
        int hrirIndex = findNearestHrirIndex(azimuth, elevation);

        // Check index validity against cache size
        if (hrirIndex < 0 || hrirIndex >= hrtfMeasurements) { // Check against measurements count used for cache size
             // LOG_WARN("Invalid HRIR index %d for oscillator %d:%d. Skipping HRTF.", hrirIndex, osc.objectId, osc.triangleIndex);
             continue; // Skip HRTF for this oscillator
        }

        // 2. Get Precomputed HRIR FFTs from Cache
        const std::vector<kiss_fft_cpx>& currentLeftFft = cachedLeftHrirFfts[hrirIndex];
        const std::vector<kiss_fft_cpx>& currentRightFft = cachedRightHrirFfts[hrirIndex];

        // Validate cache entry size (should match fftComplexSize)
        if (currentLeftFft.size() != fftComplexSize || currentRightFft.size() != fftComplexSize) {
            LOG_ERROR("Cached HRIR FFT size mismatch for index %d (Expected %d, Got L:%zu R:%zu). Skipping HRTF.",
                      hrirIndex, fftComplexSize, currentLeftFft.size(), currentRightFft.size());
            continue;
        }

        // 3. Generate Mono Source Signal Block for this oscillator
        std::fill(monoBlockBuffer.begin(), monoBlockBuffer.end(), 0.0f);
        double accumulatedPhase = osc.phase; // Start phase for this block

        for (int i = 0; i < samplesPerChannel; ++i) {
             double timeInBlock = i * sampleDuration;
             double t = blockStartTime + timeInBlock;
             double echoTime = lastPulseTime + osc.roundTripTime;

             if (t >= echoTime) { // Check if echo is active in this sample
                 double timeSinceEcho = t - echoTime;
                 if (timeSinceEcho > 10.0 * osc.sigma) continue; // Optimization

                 float sigmaSq = osc.sigma * osc.sigma; if (sigmaSq < 1e-12f) sigmaSq = 1e-12f;
                 float envelope = std::exp(-(timeSinceEcho * timeSinceEcho) / (2.0f * sigmaSq));
                 float fadeIn = std::min(1.0f, static_cast<float>(timeSinceEcho) / 0.005f);

                 // Generate sample using accumulated phase
                 monoBlockBuffer[i] = osc.amplitude * envelope * fadeIn * std::sin(static_cast<float>(accumulatedPhase));

                 // Increment phase for the *next* sample
                 double phaseIncrement = 2.0 * M_PI * osc.frequency * sampleDuration;
                 accumulatedPhase += phaseIncrement;
             }
        }
        // Update the oscillator's persistent phase for the start of the next block
        osc.phase = std::fmod(accumulatedPhase, 2.0 * M_PI);
        if (osc.phase < 0) osc.phase += 2.0 * M_PI; // Ensure positive phase

        // 4. FFT of Mono Source Block
        std::memset(fftRealBuffer.data(), 0, fftSize * sizeof(kiss_fft_scalar)); // Zero pad
        std::copy(monoBlockBuffer.begin(), monoBlockBuffer.end(), fftRealBuffer.begin()); // Copy signal
        kiss_fftr(fftConfig, fftRealBuffer.data(), fftCpxBuffer.data()); // FFT -> fftCpxBuffer

        // 5. Convolution (Left Channel)
        // Multiply Input FFT (fftCpxBuffer) by Cached Left HRIR FFT (currentLeftFft) -> fftCpxBuffer2
        for (int k = 0; k < fftComplexSize; ++k) {
            kiss_fft_cpx iC = fftCpxBuffer[k];
            kiss_fft_cpx hC = currentLeftFft[k]; // From cache
            fftCpxBuffer2[k].r = iC.r * hC.r - iC.i * hC.i;
            fftCpxBuffer2[k].i = iC.r * hC.i + iC.i * hC.r;
        }
        // 6. Inverse FFT (Left)
        kiss_fftri(ifftConfig, fftCpxBuffer2.data(), fftRealBuffer.data()); // Result -> fftRealBuffer
        // 7. Overlap-Add (Left)
        for (int i = 0; i < samplesPerChannel; ++i) {
            float convolvedSample = fftRealBuffer[i] * fftScale; // Apply scaling
            outputBuffer[i * 2] += convolvedSample; // Add to main output
            // Add overlap from previous block
            if (i < overlapLeft.size()) {
                outputBuffer[i * 2] += overlapLeft[i];
            }
        }
        // Save tail for next block's overlap
        for (int i = 0; i < overlapLeft.size(); ++i) {
            // Safety check index for fftRealBuffer access
            if (blockSize + i < fftRealBuffer.size()) {
                 overlapLeft[i] = fftRealBuffer[blockSize + i] * fftScale;
            } else {
                 overlapLeft[i] = 0.0f; // Should not happen if fftSize is correct
                 LOG_WARN("Overlap index %d out of bounds for fftRealBuffer (size %zu)", blockSize + i, fftRealBuffer.size());
            }
        }

        // 8. Convolution (Right Channel)
        // Multiply Input FFT (fftCpxBuffer) by Cached Right HRIR FFT (currentRightFft) -> fftCpxBuffer2
        for (int k = 0; k < fftComplexSize; ++k) {
            kiss_fft_cpx iC = fftCpxBuffer[k]; // Reuse input FFT
            kiss_fft_cpx hC = currentRightFft[k]; // From cache
            fftCpxBuffer2[k].r = iC.r * hC.r - iC.i * hC.i;
            fftCpxBuffer2[k].i = iC.r * hC.i + iC.i * hC.r;
        }
        // 9. Inverse FFT (Right)
        kiss_fftri(ifftConfig, fftCpxBuffer2.data(), fftRealBuffer.data()); // Result -> fftRealBuffer
        // 10. Overlap-Add (Right)
        for (int i = 0; i < samplesPerChannel; ++i) {
            float convolvedSample = fftRealBuffer[i] * fftScale; // Apply scaling
            outputBuffer[i * 2 + 1] += convolvedSample; // Add to main output
            // Add overlap from previous block
            if (i < overlapRight.size()) {
                outputBuffer[i * 2 + 1] += overlapRight[i];
            }
        }
        // Save tail for next block's overlap
        for (int i = 0; i < overlapRight.size(); ++i) {
             // Safety check index for fftRealBuffer access
            if (blockSize + i < fftRealBuffer.size()) {
                 overlapRight[i] = fftRealBuffer[blockSize + i] * fftScale;
            } else {
                 overlapRight[i] = 0.0f; // Should not happen
                 LOG_WARN("Overlap index %d out of bounds for fftRealBuffer (size %zu)", blockSize + i, fftRealBuffer.size());
            }
        }
    } // End loop over oscillators

    // --- Final Limiter ---
    float maxAmp = 0.0f;
    for(int i=0; i < bufferSize; ++i) { maxAmp = std::max(maxAmp, std::abs(outputBuffer[i])); }
    if (maxAmp > 0.95f) {
        float scale = 0.95f / maxAmp;
        for(int i=0; i < bufferSize; ++i) { outputBuffer[i] *= scale; }
    }
}


// Generate Time-Frequency Data for visualization
void GeometryProcessorImpl::generateTimeFrequencyData(int timeSteps, int frequencyBins, float* outputMagnitudes, float* outputPhases, float* outputTimeRange, float* outputFreqRange) {
     // Validate inputs
     if (!outputMagnitudes || !outputPhases || !outputTimeRange || !outputFreqRange || timeSteps <= 0 || frequencyBins <= 0) {
         LOG_ERROR("Invalid parameters for GetTimeFrequencyData.");
         // Avoid writing to potentially invalid pointers if some are null
         return;
     }

     int totalBins = timeSteps * frequencyBins;
     // Initialize output arrays
     std::fill(outputMagnitudes, outputMagnitudes + totalBins, 0.0f);
     std::fill(outputPhases, outputPhases + totalBins, 0.0f);

     // Define time/frequency range for the plot
     double currentTime = radarParams.currentTime;
     double timeWindowDuration = 1.0; // Duration of the plot window in seconds
     outputTimeRange[0] = static_cast<float>(currentTime - timeWindowDuration / 2.0);
     outputTimeRange[1] = static_cast<float>(currentTime + timeWindowDuration / 2.0);
     outputFreqRange[0] = 100.0f; // Min frequency (Hz)
     outputFreqRange[1] = 1000.0f; // Max frequency (Hz)

     // Calculate step sizes
     double timeStep = (timeSteps > 1) ? (outputTimeRange[1] - outputTimeRange[0]) / (timeSteps - 1) : 0.0;
     double freqStep = (frequencyBins > 1) ? (outputFreqRange[1] - outputFreqRange[0]) / (frequencyBins - 1) : 0.0;

     // Find the relevant pulse time
     float pulseInterval = (radarParams.pulseRepFreq > 1e-6f) ? (1.0f / radarParams.pulseRepFreq) : 1.0f;
     double lastPulseTime = std::floor(currentTime / pulseInterval) * pulseInterval;

     // Add contributions from active oscillators
     for (int oscIndex = 0; oscIndex < MAX_OSCILLATORS; oscIndex++) {
         const Oscillator& osc = oscillators[oscIndex];
         if (!osc.active || osc.amplitude < 1e-4f) continue; // Skip inactive/quiet

         double echoTime = lastPulseTime + osc.roundTripTime; // Expected echo arrival time

         // Check if echo falls within the plot's time range
         if (echoTime < outputTimeRange[0] || echoTime > outputTimeRange[1]) continue;

         // Find the corresponding time bin
         int timeBin = static_cast<int>((echoTime - outputTimeRange[0]) / timeStep + 0.5);
         if (timeBin < 0 || timeBin >= timeSteps) continue; // Should be within bounds due to check above

         // Add contribution across frequency bins (Gaussian spread around oscillator frequency)
         for (int f = 0; f < frequencyBins; f++) {
              double freq = outputFreqRange[0] + f * freqStep; // Frequency of this bin
              double freqDistance = std::abs(freq - osc.frequency);
              double freqEnv = std::exp(-(freqDistance * freqDistance) / (2.0 * 50.0 * 50.0)); // Gaussian spread (50Hz std dev)
              if (freqEnv < 0.01) continue; // Skip negligible contributions

              int idx = timeBin * frequencyBins + f; // Calculate flat array index
              if (idx < 0 || idx >= totalBins) continue; // Safety bounds check

              // Combine magnitudes and phases using complex addition
              float mag = osc.amplitude * static_cast<float>(freqEnv); // Contribution magnitude
              float phase = std::fmod(osc.phase, 2.0f * M_PI); if (phase < 0) phase += 2.0f * M_PI; // Contribution phase (0-2pi)

              float currentMag = outputMagnitudes[idx]; float currentPhase = outputPhases[idx];
              // Convert to Cartesian
              float currentReal = currentMag * std::cos(currentPhase); float currentImag = currentMag * std::sin(currentPhase);
              float addReal = mag * std::cos(phase); float addImag = mag * std::sin(phase);
              // Add
              float newReal = currentReal + addReal; float newImag = currentImag + addImag;
              // Convert back to Polar
              outputMagnitudes[idx] = std::sqrt(newReal*newReal + newImag*newImag);
              outputPhases[idx] = (outputMagnitudes[idx] > 1e-6f) ? std::atan2(newImag, newReal) : 0.0f;
              if (outputPhases[idx] < 0) outputPhases[idx] += 2.0f * M_PI; // Ensure 0-2pi range
         }
     }

     // Normalize magnitudes for better visualization (optional)
     float maxMag = 1e-6f;
     for(int i=0; i<totalBins; ++i) maxMag = std::max(maxMag, outputMagnitudes[i]);
     if (maxMag > 1e-6f) {
         for(int i=0; i<totalBins; ++i) outputMagnitudes[i] /= maxMag;
     }
}

// Print debug info for active oscillators
void GeometryProcessorImpl::debugOscillators() {
    int activeCount = 0;
    for(int i=0; i<MAX_OSCILLATORS; ++i) {
        if (oscillators[i].active) activeCount++;
    }
    printf("--- Oscillator Debug --- Active: %d / %d ---\n", activeCount, MAX_OSCILLATORS);
    int printed = 0;
    for (int i = 0; i < MAX_OSCILLATORS && printed < 20; i++) { // Print details for up to 20 active oscillators
        if (oscillators[i].active) {
             printf("  [%3d] ID:%d:%-4d F:%6.1f(T:%6.1f) A:%.3f(T:%.3f) Ph:%.2f RTT:%.3f Sig:%.3f\n",
                   i, oscillators[i].objectId, oscillators[i].triangleIndex,
                   oscillators[i].frequency, oscillators[i].targetFrequency,
                   oscillators[i].amplitude, oscillators[i].targetAmplitude, oscillators[i].phase,
                   oscillators[i].roundTripTime, oscillators[i].sigma);
             printed++;
        }
    }
    printf("--- Map Size: %zu ---\n", activeOscillatorMap.size()); // Print current size of the tracking map
}

// Public cleanup method calling internal cleanup
void GeometryProcessorImpl::cleanup() {
    cleanupResources();
}


// --- Global Instance and C API Implementation ---
static GeometryProcessorImpl* processorInstance = nullptr;

// C API functions calling the GeometryProcessorImpl instance methods
extern "C" {
    EXPORT_API void InitializeProcessor(int sampleRate, int blockSize) {
        // Create instance if it doesn't exist
        if (!processorInstance) {
             try { processorInstance = new GeometryProcessorImpl(); }
             catch (const std::exception& e) { LOG_ERROR("Failed to allocate GeometryProcessorImpl: %s", e.what()); return; }
             catch (...) { LOG_ERROR("Failed to allocate GeometryProcessorImpl: Unknown exception."); return; }
        }
        // Call initialize on the instance
        if (processorInstance) processorInstance->initialize(sampleRate, blockSize);
    }

    EXPORT_API void SetHrtfPath(const char* sofaFilePath) {
        if (processorInstance) processorInstance->setHrtfPath(sofaFilePath);
        else LOG_WARN("SetHrtfPath called but processor is not initialized.");
    }

    EXPORT_API void SetListenerOrientation(const float* listenerForward, const float* listenerUp) {
        if (processorInstance) processorInstance->setListenerOrientation(listenerForward, listenerUp);
        else LOG_WARN("SetListenerOrientation called but processor is not initialized.");
    }

    EXPORT_API void UpdateTime(float deltaTime) {
        // Avoid logging warning every frame if not initialized
        if (processorInstance) processorInstance->updateTime(deltaTime);
    }

    EXPORT_API void SetRadarParameters(float pulseRepFreq, float speedOfSound, float pulseWidth) {
        if (processorInstance) processorInstance->setRadarParameters(pulseRepFreq, speedOfSound, pulseWidth);
        else LOG_WARN("SetRadarParameters called but processor is not initialized.");
    }

    EXPORT_API void ProcessObjectGeometry(const ObjectGeometry* objects, int objectCount,
                                        const float* listenerPosition, const float* listenerForward,
                                        float maxDistance, float reflectionRadius,
                                        ProcessedPoint* outputPoints, int* outputPointCount) {
        // C API Wrapper: Validate pointers and pass buffer size assumption
        if (!outputPointCount) { LOG_ERROR("ProcessObjectGeometry: outputPointCount is NULL."); return; }
        *outputPointCount = 0; // Default to zero points
        if (!processorInstance) { LOG_WARN("ProcessObjectGeometry: Processor not initialized."); return; }
        if (!outputPoints) { LOG_ERROR("ProcessObjectGeometry: outputPoints is NULL."); return; }

        // Assume C# allocates based on MAX_OSCILLATORS or similar limit
        // Pass this assumed size to the implementation function for safety checks
        const int assumedBufferSize = MAX_OSCILLATORS;
        processorInstance->processGeometry(objects, objectCount, listenerPosition, listenerForward, maxDistance, reflectionRadius, outputPoints, assumedBufferSize, outputPointCount);
    }

    EXPORT_API void SynthesizeAudio(const ProcessedPoint* points, int pointCount, float* outputBuffer, int channelCount, int samplesPerChannel) {
        if (processorInstance) {
            processorInstance->synthesizeAudio(points, pointCount, outputBuffer, channelCount, samplesPerChannel);
        } else if (outputBuffer) {
            // Zero buffer if processor not ready but buffer exists
            std::memset(outputBuffer, 0, channelCount * samplesPerChannel * sizeof(float));
        }
    }

    EXPORT_API void GetTimeFrequencyData(int timeSteps, int frequencyBins, float* outputMagnitudes, float* outputPhases, float* outputTimeRange, float* outputFreqRange) {
        // Validate all output pointers before calling implementation
        if (!outputMagnitudes || !outputPhases || !outputTimeRange || !outputFreqRange) {
            LOG_ERROR("GetTimeFrequencyData called with one or more null output pointers.");
            return;
        }
        if (processorInstance) {
            processorInstance->generateTimeFrequencyData(timeSteps, frequencyBins, outputMagnitudes, outputPhases, outputTimeRange, outputFreqRange);
        } else {
            LOG_WARN("GetTimeFrequencyData called but processor is not initialized.");
            // Zero out data if processor not ready
            int totalBins = timeSteps * frequencyBins;
            if (totalBins > 0) { // Avoid potential negative size if timeSteps/frequencyBins are invalid
                 std::fill(outputMagnitudes, outputMagnitudes + totalBins, 0.0f);
                 std::fill(outputPhases, outputPhases + totalBins, 0.0f);
            }
            outputTimeRange[0] = 0.0f; outputTimeRange[1] = 0.0f;
            outputFreqRange[0] = 0.0f; outputFreqRange[1] = 0.0f;
        }
    }

    EXPORT_API void DebugOscillators() {
        if (processorInstance) processorInstance->debugOscillators();
        else LOG_WARN("DebugOscillators called but processor is not initialized.");
    }

    EXPORT_API void DebugBuffer() { /* Deprecated */ }

    EXPORT_API void Cleanup() {
        // Safely delete the instance and set pointer to null
        if (processorInstance) {
            delete processorInstance;
            processorInstance = nullptr;
            LOG_INFO("GeometryProcessor cleaned up via API call.");
        } else {
             // LOG_INFO("Cleanup called but processor was already null.");
        }
    }
} // extern "C"
