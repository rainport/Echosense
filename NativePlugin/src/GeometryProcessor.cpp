#include "GeometryProcessor.h"
#include <algorithm>
#include <cmath>

class GeometryProcessorImpl {
private:
    int sampleRate;
    int blockSize;
    int maxFacesPerObject;
    float maxDistance;
    float listenerPosition[3];
    float previousListenerPosition[3];
    float listenerVelocity[3];
    std::vector<ProcessedPoint> workingPoints;
    std::vector<float> lastFacePositions;
    std::vector<ActiveEcho> activeEchoes;
    float echoMaxDuration;    // Maximum duration we'll keep an echo alive
    RadarParameters radarParams;
    
    // Audio continuity members
    float previousBuffer[8192];
    float previousPhase;
    float lastBufferEndTime;
    
    // Utility functions
    float dot(const float* a, const float* b) {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    }
    
    float length(const float* v) {
        return std::sqrt(dot(v, v));
    }
    
    void normalize(float* v) {
        float len = length(v);
        if (len > 0) {
            v[0] /= len;
            v[1] /= len;
            v[2] /= len;
        }
    }
    
    void subtract(const float* a, const float* b, float* result) {
        result[0] = a[0] - b[0];
        result[1] = a[1] - b[1];
        result[2] = a[2] - b[2];
    }
    
    void transformPoint(const float* point, const float* matrix, float* result) {
        for (int i = 0; i < 3; i++) {
            result[i] = matrix[i*4] * point[0] + 
                       matrix[i*4 + 1] * point[1] + 
                       matrix[i*4 + 2] * point[2] + 
                       matrix[i*4 + 3];
        }
    }
    
    void transformNormal(const float* normal, const float* matrix, float* result) {
        // Transform normal using inverse transpose of 3x3 part of matrix
        // Note: This is simplified and assumes uniform scaling
        for (int i = 0; i < 3; i++) {
            result[i] = matrix[i*4] * normal[0] + 
                       matrix[i*4 + 1] * normal[1] + 
                       matrix[i*4 + 2] * normal[2];
        }
        normalize(result);
    }
    
    AudioParameters computeAudioParams(const float* position, 
                                     const float* velocity,
                                     float facingFactor,
                                     float distance) {
        AudioParameters params;
        
        // Base amplitude falls off with distance and scales with facing factor
        params.amplitude = facingFactor / (1.0f + distance);
        
        // Center frequency could vary with height or other parameters
        params.baseFreq = 220.0f * (1.0f + position[1]);
        
        // Sigma (envelope width) could depend on object properties
        params.sigma = radarParams.pulseWidth;

        
        // Time center - could be based on distance for delay effects
        params.roundTripTime = 2.0f * distance / radarParams.speedOfSound;
        
        // Phase could create interference patterns between nearby faces
        params.phase = 0.0f;
        
        // Initialize dopplerFreq to baseFreq (will be modified later)
        params.dopplerFreq = params.baseFreq;
        
        return params;
    }

public:
    GeometryProcessorImpl() 
        : sampleRate(44100), blockSize(8192), maxFacesPerObject(500), 
          maxDistance(10.0f), previousPhase(0.0f), lastBufferEndTime(0.0f) {
        
        // Initialize radar parameters
        radarParams.pulseRepFreq = 2.0f;
        radarParams.speedOfSound = 16.0f;
        radarParams.currentTime = 0.0f;
        radarParams.lastFrameTime = 0.0f;
        radarParams.frameTime = 0.0f;

        echoMaxDuration = 5.0f;
        
        // Initialize arrays
        for (int i = 0; i < 3; i++) {
            listenerPosition[i] = 0.0f;
            previousListenerPosition[i] = 0.0f;
            listenerVelocity[i] = 0.0f;
        }
        
        // Initialize audio buffers
        std::fill(previousBuffer, previousBuffer + 8192, 0.0f);
    }
    
    void updateTime(float deltaTime) {
        radarParams.lastFrameTime = radarParams.currentTime;
        radarParams.currentTime += deltaTime;
        radarParams.frameTime = deltaTime;
    }

    void initialize(int sr, int block) {
        sampleRate = sr;
        blockSize = block;
        workingPoints.clear();
    }
    
    void cross(const float* a, const float* b, float* result) {
        result[0] = a[1] * b[2] - a[2] * b[1];
        result[1] = a[2] * b[0] - a[0] * b[2];
        result[2] = a[0] * b[1] - a[1] * b[0];
    }

    void setRadarParameters(float pulseRepFreq, float speedOfSound, float pulseWidth) {
        radarParams.pulseRepFreq = pulseRepFreq;
        radarParams.speedOfSound = speedOfSound;
        radarParams.pulseWidth = pulseWidth;
    }

    void processGeometry(const ObjectGeometry* objects, int objectCount,
                        const float* listenerPos, const float* listenerFwd,
                        float maxDist, ProcessedPoint* outputPoints,
                        int* outputPointCount) 
    {
        // Store the max distance value
        maxDistance = maxDist;
        
        // Calculate listener velocity
        if (radarParams.frameTime > 0.001f) {
            for (int i = 0; i < 3; i++) {
                listenerVelocity[i] = (listenerPos[i] - previousListenerPosition[i]) / radarParams.frameTime;
            }
        }
        
        // Store listener position for distance calculations
        for (int i = 0; i < 3; i++) {
            previousListenerPosition[i] = listenerPosition[i];
            listenerPosition[i] = listenerPos[i];
        }

        workingPoints.clear();
        
        // Resize or initialize the last positions array if needed
        int totalMaxFaces = objectCount * maxFacesPerObject;
        if (lastFacePositions.size() < totalMaxFaces * 3) {
            lastFacePositions.resize(totalMaxFaces * 3, 0.0f);
        }
        
        int totalFaceCount = 0;
        
        for (int obj = 0; obj < objectCount; obj++) {
            const ObjectGeometry& geometry = objects[obj];
            int stride = 1;  // Default stride of 1 processes all faces

            // Adjust stride based on triangle count (adaptive sampling)
            if (geometry.triangleCount > 200) {
                stride = geometry.triangleCount / 100;  // Target ~100 faces for high-poly objects
                stride = std::max(1, stride);  // Ensure minimum stride of 1
            }

            // Then modify your face loop to use the stride
            for (int f = 0; f < geometry.triangleCount; f += stride) {
                int faceIndex = (totalFaceCount++) * 3;
                
                // Skip if we've exceeded our per-object face limit
                if (faceIndex >= totalMaxFaces * 3) break;
                
                // Get face center and normal
                const float* faceCenter = &geometry.faceCenters[f * 3];
                const float* faceNormal = &geometry.faceNormals[f * 3];
                
                // Transform to world space
                float worldCenter[3], worldNormal[3];
                transformPoint(faceCenter, geometry.transform, worldCenter);
                transformNormal(faceNormal, geometry.transform, worldNormal);
                
                // Calculate direction and distance to listener
                float toListener[3];
                subtract(listenerPos, worldCenter, toListener);
                float distance = length(toListener);
                
                // Skip if too far
                if (distance > maxDistance) continue;
                
                normalize(toListener);
                
                // Calculate facing factor (dot product between normal and direction to listener)
                float facingFactor = dot(worldNormal, toListener);
                
                // Skip if not facing listener enough
                if (facingFactor < 0.4f) continue;
                
                // Calculate velocity based on position difference
                float velocity[3] = {0, 0, 0};
                if (radarParams.frameTime > 0.001f) {  // Only if we have a reasonable frame time
                    for (int i = 0; i < 3; i++) {
                        velocity[i] = (worldCenter[i] - lastFacePositions[faceIndex + i]) / 
                                    radarParams.frameTime;
                    }
                }
                
                // Store current position for next frame
                for (int i = 0; i < 3; i++) {
                    lastFacePositions[faceIndex + i] = worldCenter[i];
                }
                
                // Calculate velocity component toward listener
                float velocityTowardListener = dot(velocity, toListener);
                
                // Create processed point for this face
                ProcessedPoint point;
                std::copy(worldCenter, worldCenter + 3, point.position);
                std::copy(worldNormal, worldNormal + 3, point.normal);
                std::copy(velocity, velocity + 3, point.velocity);
                point.facingFactor = facingFactor;
                
                // Calculate radar/audio parameters
                AudioParameters params;
                
                // Base amplitude falls off with distance and scales with facing factor
                params.amplitude = 0.5f * facingFactor / (1.0f + distance);
                
                // Base carrier frequency
                params.baseFreq = 440.0f;  // A4 note
                
                // Calculate Doppler shift
                float dopplerFactor = (radarParams.speedOfSound - velocityTowardListener) / 
                                    radarParams.speedOfSound;
                params.dopplerFreq = params.baseFreq * dopplerFactor;
                
                // Pulse width (smaller sigma = shorter pulse)
                params.sigma = radarParams.pulseWidth;                

                // Round trip time for pulse
                params.roundTripTime = 2.0f * distance / radarParams.speedOfSound;

                // Phase could be based on various factors
                params.phase = 0.0f;
                
                point.params = params;
                
                workingPoints.push_back(point);
            }
        }
        
        *outputPointCount = static_cast<int>(workingPoints.size());
        std::copy(workingPoints.begin(), 
                workingPoints.begin() + *outputPointCount,
                outputPoints);
    }
    
    void synthesizeAudio(const ProcessedPoint* points, int pointCount,
                    float* outputBuffer, int bufferSize) 
    {

        printf("Current sigma values: pulseWidth=%.4f, points[0].sigma=%.4f\n", radarParams.pulseWidth, pointCount > 0 ? points[0].params.sigma : 0.0f);

        // Clear output buffer
        std::fill(outputBuffer, outputBuffer + bufferSize, 0.0f);
        
        // Buffer settings
        float bufferDuration = static_cast<float>(bufferSize) / sampleRate;
        float currentTime = radarParams.currentTime;
        float bufferStartTime = currentTime;
        float bufferEndTime = bufferStartTime + bufferDuration;
        
        // Generate a temp buffer for audio processing
        float* tempBuffer = new float[bufferSize]();
        std::fill(tempBuffer, tempBuffer + bufferSize, 0.0f);
        
        // 1. Add new echoes from current points
        float pulseInterval = 1.0f / radarParams.pulseRepFreq;
        float lastPulseTime = std::floor(currentTime / pulseInterval) * pulseInterval;
        
        // Generate reference ping
        if (currentTime - lastPulseTime < bufferDuration) {
            // Create a reference ping active echo
            ActiveEcho refEcho;
            refEcho.startTime = lastPulseTime;
            refEcho.sigma = 0.05f;
            refEcho.frequency = 880.0f;
            refEcho.amplitude = 0.0f;  // Start at 0 and fade in
            refEcho.targetAmplitude = 0.5f;
            refEcho.phase = 0.0f;
            refEcho.isActive = true;
            activeEchoes.push_back(refEcho);
        }
        
        // Process new object echoes
        for (int p = 0; p < pointCount; p++) {
            const ProcessedPoint& point = points[p];
            
            // Calculate distance from listener to object
            float dx = point.position[0] - listenerPosition[0];
            float dy = point.position[1] - listenerPosition[1];
            float dz = point.position[2] - listenerPosition[2];
            float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            // Calculate echo timing
            float roundTripTime = 2.0f * distance / radarParams.speedOfSound;
            float echoTime = lastPulseTime + roundTripTime;

            
            
            // If the echo will be heard soon (within our tracking window)
            if (echoTime >= currentTime && echoTime < currentTime + echoMaxDuration) {

                // Direction to listener
                float toListener[3] = {
                    listenerPosition[0] - point.position[0],
                    listenerPosition[1] - point.position[1],
                    listenerPosition[2] - point.position[2]
                };
                // Normalize
                float lenToListener = std::sqrt(toListener[0]*toListener[0] + 
                                            toListener[1]*toListener[1] + 
                                            toListener[2]*toListener[2]);
                if (lenToListener > 0) {
                    toListener[0] /= lenToListener;
                    toListener[1] /= lenToListener;
                    toListener[2] /= lenToListener;
                }

                // For radar with static objects, we care about listener velocity
                // Project listener velocity onto the direction FROM listener TO object
                float listenerVelTowardObject = -dot(listenerVelocity, toListener);

                // Two-way Doppler shift (both outgoing and return paths affected)
                float dopplerFactor = (radarParams.speedOfSound + listenerVelTowardObject) / 
                                    (radarParams.speedOfSound - listenerVelTowardObject);
                float freq = 440.0f * dopplerFactor;
                float sigma = point.params.sigma;
                
                // Add to active echoes
                ActiveEcho echo;
                echo.startTime = echoTime;
                echo.sigma = sigma;
                echo.frequency = freq;
                echo.amplitude = 0.0f;  // Start at 0 and fade in
                echo.targetAmplitude = 0.4f * point.facingFactor / (1.0f + distance);
                echo.phase = 0.0f;
                std::copy(point.position, point.position + 3, echo.position);
                echo.isActive = true;
                activeEchoes.push_back(echo);

                float totalDuration = 6.0f * echo.sigma;
                printf("Echo duration: sigma=%.4f, duration=%.4f seconds\n", 
                    echo.sigma, totalDuration);
            }
        }

        // If we have no points but many echoes, accelerate decay
        bool acceleratedDecay = (pointCount == 0 && activeEchoes.size() > 30);
        
        // 2. Process all active echoes for this buffer
        for (auto& echo : activeEchoes) {
            // Calculate echo lifetime
            float echoAge = currentTime - echo.startTime;
            float totalDuration = 6.0f * echo.sigma;  // 6 sigma coverage
            
            // Smooth amplitude approach to target
            float fadeRate = acceleratedDecay ? 0.05f : 0.1f;
            echo.amplitude = echo.amplitude * (1.0f - fadeRate) + echo.targetAmplitude * fadeRate;
            
            // Extra decay when no new points are coming in
            if (acceleratedDecay) {
                echo.targetAmplitude *= 0.98f;
            }
            
            // Mark as inactive if it's expired or too quiet
            if (echoAge > totalDuration || echoAge < -echoMaxDuration || echo.amplitude < 0.001f) {
                echo.isActive = false;
                continue;
            }
            
            // Echo hasn't started yet in this buffer
            if (echo.startTime > bufferEndTime) {
                continue;
            }
            
            // Echo has already ended before this buffer
            if (echo.startTime + totalDuration < bufferStartTime) {
                echo.isActive = false;
                continue;
            }
            
            // Calculate sample range for this buffer
            int startSample = 0;
            if (echo.startTime > bufferStartTime) {
                startSample = static_cast<int>((echo.startTime - bufferStartTime) * sampleRate);
            }
            
            int endSample = bufferSize;
            if (echo.startTime + totalDuration < bufferEndTime) {
                endSample = static_cast<int>((echo.startTime + totalDuration - bufferStartTime) * sampleRate);
            }
            
            // Clamp to buffer boundaries
            startSample = std::max(0, std::min(startSample, bufferSize - 1));
            endSample = std::max(0, std::min(endSample, bufferSize));
            
            // Generate the appropriate portion of the echo
            for (int i = startSample; i < endSample; i++) {
                float t = (bufferStartTime + i / static_cast<float>(sampleRate)) - echo.startTime;
                float envelope = std::exp(-(t*t) / (2.0f * echo.sigma * echo.sigma));
                float signal = echo.amplitude * envelope * std::sin(2.0f * M_PI * echo.frequency * t + echo.phase);
                tempBuffer[i] += signal;
                
                // Store phase at the end of this echo's contribution for continuity
                if (i == endSample - 1) {
                    echo.phase = fmod(2.0f * M_PI * echo.frequency * t + echo.phase, 2.0f * M_PI);
                }
            }
        }
        
        // 3. Clean up inactive echoes
        activeEchoes.erase(
            std::remove_if(activeEchoes.begin(), activeEchoes.end(), 
                        [](const ActiveEcho& e) { return !e.isActive; }),
            activeEchoes.end());
        
        // 4. Apply smooth normalization
        static float prevMaxAmp = 0.0f;
        static float currentScale = 1.0f;
        
        // Find max amplitude
        float maxAmp = 0.0f;
        for (int i = 0; i < bufferSize; i++) {
            maxAmp = std::max(maxAmp, std::abs(tempBuffer[i]));
        }
        
        // Smooth max amplitude transitions
        maxAmp = std::max(maxAmp, prevMaxAmp * 0.5f); // Never drop too quickly
        float targetScale = (maxAmp > 0.8f) ? 0.8f / maxAmp : 1.0f;
        currentScale = currentScale * 0.7f + targetScale * 0.3f; // Smooth scaling changes
        
        // Apply scaled normalization
        for (int i = 0; i < bufferSize; i++) {
            tempBuffer[i] *= currentScale;
        }
        
        // Remember for next buffer
        prevMaxAmp = maxAmp;
        
        // 5. Force a consistent overlap for smoother transitions
        int overlapSamples = static_cast<int>(0.05f * sampleRate); // 50ms overlap
        for (int i = 0; i < overlapSamples && i < bufferSize; i++) {
            float crossfade = static_cast<float>(i) / overlapSamples;
            tempBuffer[i] = tempBuffer[i] * crossfade + 
                        previousBuffer[bufferSize - overlapSamples + i] * (1.0f - crossfade);
        }
        
        // 6. Apply gentle fade in/out to buffer edges
        int fadeLength = std::min(100, bufferSize / 10);
        for (int i = 0; i < fadeLength; i++) {
            float fadeIn = static_cast<float>(i) / fadeLength;
            tempBuffer[i] *= fadeIn;
        }
        
        for (int i = bufferSize - fadeLength; i < bufferSize; i++) {
            float fadeOut = static_cast<float>(bufferSize - i) / fadeLength;
            tempBuffer[i] *= fadeOut;
        }
        
        // Copy to output buffer
        std::copy(tempBuffer, tempBuffer + bufferSize, outputBuffer);
        
        // Store current buffer for next time
        std::copy(tempBuffer, tempBuffer + bufferSize, previousBuffer);
        
        // Log echo management info occasionally
        static int logCounter = 0;
        if (++logCounter % 100 == 0) {
            printf("Active echoes: %d, Buffer time: %.3f - %.3f\n", 
                (int)activeEchoes.size(), bufferStartTime, bufferEndTime);
        }
        
        // Cleanup
        delete[] tempBuffer;
    }
};

// Global instance
static GeometryProcessorImpl* processor = nullptr;

// External API Implementation
extern "C" {
    EXPORT_API void InitializeProcessor(int sampleRate, int blockSize) {
        if (!processor) {
            processor = new GeometryProcessorImpl();
        }
        processor->initialize(sampleRate, blockSize);
    }
    
    EXPORT_API void SetRadarParameters(float pulseRepFreq, float speedOfSound, float pulseWidth) {
        if (processor) {
            processor->setRadarParameters(pulseRepFreq, speedOfSound, pulseWidth);
        }
    }

    EXPORT_API void ProcessObjectGeometry(const ObjectGeometry* objects,
                                        int objectCount,
                                        const float* listenerPosition,
                                        const float* listenerForward,
                                        float maxDistance,
                                        ProcessedPoint* outputPoints,
                                        int* outputPointCount) {
        if (processor) {
            processor->processGeometry(objects, objectCount, listenerPosition,
                                    listenerForward, maxDistance,
                                    outputPoints, outputPointCount);
        }
    }
    
    EXPORT_API void SynthesizeAudio(const ProcessedPoint* points,
                                   int pointCount,
                                   float* outputBuffer,
                                   int bufferSize) {
        if (processor) {
            processor->synthesizeAudio(points, pointCount,
                                     outputBuffer, bufferSize);
        }
    }

    EXPORT_API void UpdateTime(float deltaTime) {
        if (processor) {
            processor->updateTime(deltaTime);
        }
    }
    
    EXPORT_API void Cleanup() {
        delete processor;
        processor = nullptr;
    }


}