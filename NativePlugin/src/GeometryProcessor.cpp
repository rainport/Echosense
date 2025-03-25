#include "GeometryProcessor.h"
#include <algorithm>
#include <cmath>

// Oscillator structure - represents a single sound source
struct Oscillator {
    bool active;                // Whether this oscillator is currently in use
    float frequency;            // Current frequency (Hz)
    float targetFrequency;      // Target frequency to approach
    float amplitude;            // Current amplitude
    float targetAmplitude;      // Target amplitude to approach
    float phase;                // Current phase (radians)
    float position[3];          // 3D position of the reflection point
    float normal[3];            // Surface normal at the reflection point
    float roundTripTime;        // Time for sound to travel to point and back
    float sigma;                // Width of the Gaussian envelope
    // Phase tracking variables
    float lastUpdateTime;     // Time when parameters were last updated
    float phaseAccumulator;   // Accumulated phase up to lastUpdateTime
    
    // Enhanced reset method
    void reset() {
        active = false;
        frequency = 0.0f;
        targetFrequency = 0.0f;
        amplitude = 0.0f;
        targetAmplitude = 0.0f;
        phase = 0.0f;
        roundTripTime = 0.0f;
        sigma = 0.05f;
        lastUpdateTime = 0.0f;
        phaseAccumulator = 0.0f;
        
        for (int i = 0; i < 3; i++) {
            position[i] = 0.0f;
            normal[i] = 0.0f;
        }
    }
    
    // Enhanced update method with phase tracking
    void update(float currentTime, float deltaTime, float smoothingRate) {
        if (!active) return;
        
        // Calculate time since last update
        float timeDelta = currentTime - lastUpdateTime;
        if (timeDelta <= 0.0f) return;
        
        // Accumulate phase up to this point (using current frequency)
        phaseAccumulator += 2.0f * M_PI * frequency * timeDelta;
        phaseAccumulator = fmod(phaseAccumulator, 2.0f * M_PI);
        
        // Now that we've accumulated phase, we can safely change frequency
        float lerpFactor = 1.0f - std::exp(-deltaTime * smoothingRate);
        
        // Store old values to check if significant changes occurred
        float oldFrequency = frequency;
        float oldAmplitude = amplitude;
        
        // Smoothly approach target values
        frequency = frequency * (1.0f - lerpFactor) + targetFrequency * lerpFactor;
        amplitude = amplitude * (1.0f - lerpFactor) + targetAmplitude * lerpFactor;
        
        // Update last update time
        lastUpdateTime = currentTime;
        
        // Deactivate if amplitude becomes negligible
        if (amplitude < 0.0001f && targetAmplitude < 0.0001f) {
            active = false;
        }
    }
    
    // Get the precise phase at a specific time
    float getPhaseAtTime(float time) {
        // Calculate time since last parameter update
        float timeSinceUpdate = time - lastUpdateTime;
        
        // Return accumulated phase plus phase since update
        return phaseAccumulator + 2.0f * M_PI * frequency * timeSinceUpdate;
    }
};

class GeometryProcessorImpl {
private:
    // Basic configuration
    int sampleRate;
    int blockSize;
    int maxFacesPerObject;
    float maxDistance;
    
    // Listener data
    float listenerPosition[3];
    float listenerVelocity[3];
    
    // Radar/pulse parameters
    RadarParameters radarParams;
    
    // Oscillator bank
    static const int MAX_OSCILLATORS = 256;
    Oscillator oscillators[MAX_OSCILLATORS];
    
    // Audio output buffer
    static const int INTERNAL_BUFFER_SIZE = 24576; // 3x typical 8192 buffer
    float* internalBuffer;
    float lastBufferEndTime;
    
    // Working memory for geometry processing
    std::vector<ProcessedPoint> workingPoints;
    
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
        for (int i = 0; i < 3; i++) {
            result[i] = matrix[i*4] * normal[0] + 
                       matrix[i*4 + 1] * normal[1] + 
                       matrix[i*4 + 2] * normal[2];
        }
        normalize(result);
    }
    
    // Apply a simple low-pass filter to a buffer segment
    void applyLowPassFilter(float* buffer, int startIdx, int endIdx, float cutoffFreq) {
        float dt = 1.0f / sampleRate;
        float RC = 1.0f / (2.0f * M_PI * cutoffFreq);
        float alpha = dt / (RC + dt);
        
        float lastSample = (startIdx > 0) ? buffer[startIdx - 1] : buffer[startIdx];
        
        for (int i = startIdx; i < endIdx; i++) {
            buffer[i] = alpha * buffer[i] + (1.0f - alpha) * lastSample;
            lastSample = buffer[i];
        }
    }
    
    // Calculate the appropriate audio parameters for a reflection point
    AudioParameters computeAudioParams(const float* position, const float* normal,
                                     float facingFactor, float distance, float area) {
        AudioParameters params;
        
        // Ensure minimum distance to prevent amplitude spikes
        float safeDistance = std::max(distance, 0.1f);
        
        // Scale area contribution 
        float areaContribution = std::sqrt(area) * 0.5f;
        areaContribution = std::max(areaContribution, 0.2f);
        
        // Base amplitude calculation - linear distance falloff
        params.amplitude = facingFactor * (areaContribution / safeDistance);
        
        // Apply scaling to keep volume reasonable
        float amplitudeScaleFactor = 2.0f; // Increased from original 0.8f for better audibility
        params.amplitude *= amplitudeScaleFactor;
        
        // Base carrier frequency
        params.baseFreq = 440.0f; // A4 note
        
        // For static scenes, Doppler is only from listener motion
        // Calculate direction from listener to reflection point
        float direction[3] = {
            position[0] - listenerPosition[0],
            position[1] - listenerPosition[1],
            position[2] - listenerPosition[2]
        };
        normalize(direction);
        
        // Project listener velocity onto this direction
        float listenerRadialVelocity = dot(listenerVelocity, direction);
        
        // Calculate Doppler shift
        float dopplerDelta = listenerRadialVelocity / 10.0f; // Scale for more subtle effect
        float speedOfSound = radarParams.speedOfSound;
        
        // Limit extreme values
        dopplerDelta = - std::max(-0.5f * speedOfSound, std::min(dopplerDelta, 0.5f * speedOfSound));
        
        float dopplerFactor = (speedOfSound - dopplerDelta) / (speedOfSound + dopplerDelta);
        params.dopplerFreq = params.baseFreq * dopplerFactor;
        
        // Ensure frequency stays in a reasonable range
        params.dopplerFreq = std::max(params.baseFreq * 0.5f, 
                                    std::min(params.dopplerFreq, params.baseFreq * 2.0f));
        
        // Pulse width
        params.sigma = radarParams.pulseWidth;
        
        // Round trip time
        params.roundTripTime = 2.0f * distance / speedOfSound;
        
        // Phase - will be handled by the oscillator
        params.phase = 0.0f;
        
        return params;
    }
    
    // Find the best oscillator to map to a given reflection point
    int findBestOscillator(const ProcessedPoint& point) {
        // First, try to find an inactive oscillator
        for (int i = 0; i < MAX_OSCILLATORS; i++) {
            if (!oscillators[i].active) {
                return i;
            }
        }
        
        // If all are active, find the one with lowest amplitude
        int lowestAmpIndex = 0;
        float lowestAmp = oscillators[0].amplitude;
        
        for (int i = 1; i < MAX_OSCILLATORS; i++) {
            if (oscillators[i].amplitude < lowestAmp) {
                lowestAmp = oscillators[i].amplitude;
                lowestAmpIndex = i;
            }
        }
        
        return lowestAmpIndex;
    }
    
    // Map reflection points to oscillators
    void assignPointsToOscillators(const ProcessedPoint* points, int pointCount, float pulseTime) {
        // First, reduce the target amplitude of all oscillators
        // This ensures unused ones will fade out
        for (int i = 0; i < MAX_OSCILLATORS; i++) {
            if (oscillators[i].active) {
                oscillators[i].targetAmplitude *= 0.95f;
            }
        }
        
        // Then assign each point to the best matching oscillator
        for (int p = 0; p < pointCount; p++) {
            const ProcessedPoint& point = points[p];
            
            // Calculate reflection time
            float roundTripTime = point.params.roundTripTime;
            float echoTime = pulseTime + roundTripTime;
            
            // Try to find an existing oscillator for this point
            bool foundMatch = false;
            int bestOscillator = -1;
            float bestDistance = 0.1f; // Distance threshold for matching
            
            for (int i = 0; i < MAX_OSCILLATORS; i++) {
                if (oscillators[i].active) {
                    // Calculate squared distance between positions
                    float dx = oscillators[i].position[0] - point.position[0];
                    float dy = oscillators[i].position[1] - point.position[1];
                    float dz = oscillators[i].position[2] - point.position[2];
                    float distSq = dx*dx + dy*dy + dz*dz;
                    
                    if (distSq < bestDistance) {
                        bestDistance = distSq;
                        bestOscillator = i;
                        foundMatch = true;
                    }
                }
            }
            
            // If no match found, find a new oscillator
            if (!foundMatch) {
                bestOscillator = findBestOscillator(point);
            }
            
            // Set parameters for this oscillator
            Oscillator& osc = oscillators[bestOscillator];
            osc.active = true;
            
            // Copy position and normal
            for (int i = 0; i < 3; i++) {
                osc.position[i] = point.position[i];
                osc.normal[i] = point.normal[i];
            }
            
            osc.roundTripTime = roundTripTime;
            osc.sigma = point.params.sigma;
            osc.targetFrequency = point.params.dopplerFreq;
            osc.targetAmplitude = point.params.amplitude * 2.0f; // Increase for better audibility
            
            // If this is a new oscillator, initialize parameters
            if (!foundMatch) {
                osc.frequency = osc.targetFrequency;
                osc.amplitude = 0.0f; // Start at 0 and fade in
                osc.phase = 0.0f;
            }
        }
    }

public:
    // Constructor
    GeometryProcessorImpl() 
        : sampleRate(44100), blockSize(8192), maxFacesPerObject(500), 
        maxDistance(10.0f), lastBufferEndTime(0.0f) {
        
        // Initialize radar parameters
        radarParams.pulseRepFreq = 2.0f;
        radarParams.speedOfSound = 16.0f;
        radarParams.currentTime = 0.0f;
        radarParams.lastFrameTime = 0.0f;
        radarParams.frameTime = 0.0f;
        radarParams.pulseWidth = 0.05f;
        
        // Initialize arrays
        for (int i = 0; i < 3; i++) {
            listenerPosition[i] = 0.0f;
            listenerVelocity[i] = 0.0f;
        }
        
        // Initialize all oscillators
        for (int i = 0; i < MAX_OSCILLATORS; i++) {
            oscillators[i].reset();
            oscillators[i].lastUpdateTime = radarParams.currentTime;
        }
        
        // Allocate audio buffer
        internalBuffer = new float[INTERNAL_BUFFER_SIZE];
        std::fill(internalBuffer, internalBuffer + INTERNAL_BUFFER_SIZE, 0.0f);
    }
    
    // Destructor
    ~GeometryProcessorImpl() {
        delete[] internalBuffer;
    }
    
    void updateTime(float deltaTime) {
        radarParams.lastFrameTime = radarParams.currentTime;
        radarParams.currentTime += deltaTime;
        radarParams.frameTime = deltaTime;
        
        // Update all oscillators with current time
        for (int i = 0; i < MAX_OSCILLATORS; i++) {
            oscillators[i].update(radarParams.currentTime, deltaTime, 10.0f);
        }
    }

    void initialize(int sr, int block) {
        sampleRate = sr;
        blockSize = block;
        workingPoints.clear();
    }
    
    void setRadarParameters(float pulseRepFreq, float speedOfSound, float pulseWidth) {
        radarParams.pulseRepFreq = pulseRepFreq;
        radarParams.speedOfSound = speedOfSound;
        radarParams.pulseWidth = pulseWidth;
    }

    void processGeometry(const ObjectGeometry* objects, int objectCount,
                    const float* listenerPos, const float* listenerFwd,
                    float maxDist, float reflectionRadius, ProcessedPoint* outputPoints,
                    int* outputPointCount) 
    {
        // Store the max distance value
        maxDistance = maxDist;
        
        // Calculate listener velocity (if frame time is valid)
        if (radarParams.frameTime > 0.001f) {
            for (int i = 0; i < 3; i++) {
                listenerVelocity[i] = (listenerPos[i] - listenerPosition[i]) / radarParams.frameTime;
            }
        }
        
        // Store current listener position
        for (int i = 0; i < 3; i++) {
            listenerPosition[i] = listenerPos[i];
        }

        workingPoints.clear();
        
        // Process each object
        for (int obj = 0; obj < objectCount; obj++) {
            const ObjectGeometry& geometry = objects[obj];
            
            // Process triangles
            for (int f = 0; f < geometry.triangleCount; f++) {
                // Get face center and normal
                const float* faceCenter = &geometry.faceCenters[f * 3];
                const float* faceNormal = &geometry.faceNormals[f * 3];
                
                // Transform to world space
                float worldCenter[3], worldNormal[3]; // of the triangle
                transformPoint(faceCenter, geometry.transform, worldCenter);
                transformNormal(faceNormal, geometry.transform, worldNormal);
                
                // Calculate vector FROM listener TO triangle and distance
                float fromListener[3];
                subtract(worldCenter, listenerPos, fromListener);
                float distance = length(fromListener);
                
                // FILTER 1: Skip if too far
                if (distance > maxDistance) continue;
                
                // Normalize direction
                float normalizedFromListener[3];
                std::copy(fromListener, fromListener + 3, normalizedFromListener);
                normalize(normalizedFromListener);
                
                // Calculate vector FROM triangle TO listener (opposite direction)
                float toListener[3] = {
                    -normalizedFromListener[0],
                    -normalizedFromListener[1],
                    -normalizedFromListener[2]
                };
                
                // Calculate dot product between normal and direction to listener
                // these are both unit magnitude so dotting just gives the angle
                float normalDotListener = dot(worldNormal, toListener);
                
                // FILTER 2: Only consider triangles in the front hemisphere
                if (normalDotListener <= 0.0f) continue;
                
                // Calculate theta - the angle between normal and direction to listener
                // invert cos(\theta) to get theta
                float theta = std::acos(normalDotListener);
                
                // FILTER 3: Apply the d * sin(θ) <= r formula
                float reflectionValue = distance * std::sin(theta);
                bool willReflect = (reflectionValue <= reflectionRadius);
                
                // Skip if this triangle won't reflect sound back to listener
                if (!willReflect) continue;
                
                // Calculate reflection quality (1.0 = perfect reflection, 0.0 = threshold)
                float reflectionQuality = 1.0f - (reflectionValue / reflectionRadius);
                reflectionQuality = std::max(0.1f, reflectionQuality);  // Ensure minimum quality
                
                // Calculate triangle area (if available)
                float triangleArea = 1.0f;  // Default value
                if (geometry.triangleAreas != nullptr && f < geometry.triangleCount) {
                    triangleArea = geometry.triangleAreas[f];
                }
                
                // Create processed point for this face
                ProcessedPoint point;
                std::copy(worldCenter, worldCenter + 3, point.position);
                std::copy(worldNormal, worldNormal + 3, point.normal);
                
                // Static scene, so velocity is zero
                for (int i = 0; i < 3; i++) {
                    point.velocity[i] = 0.0f;
                }
                
                point.facingFactor = reflectionQuality;
                point.area = triangleArea;
                
                // Calculate radar/audio parameters
                point.params = computeAudioParams(
                    worldCenter, worldNormal, reflectionQuality, distance, triangleArea);
                
                workingPoints.push_back(point);
            }
        }
        
        // Copy to output array
        *outputPointCount = static_cast<int>(workingPoints.size());
        std::copy(workingPoints.begin(), 
                workingPoints.begin() + *outputPointCount,
                outputPoints);
        
        printf("Total processed points: %d\n", *outputPointCount);
        
        // Map these points to oscillators for continuous sound
        float currentTime = radarParams.currentTime;
        float pulseInterval = 1.0f / radarParams.pulseRepFreq;
        float lastPulseTime = std::floor(currentTime / pulseInterval) * pulseInterval;
        
        assignPointsToOscillators(outputPoints, *outputPointCount, lastPulseTime);
    }

    // Modified synthesizeAudio method for better buffer transitions
    // Replace the synthesizeAudio method with this simpler, more stable approach
    void synthesizeAudio(const ProcessedPoint* points, int pointCount,
                    float* outputBuffer, int bufferSize) 
    {
        // Clear output buffer
        std::fill(outputBuffer, outputBuffer + bufferSize, 0.0f);
        
        float bufferDuration = static_cast<float>(bufferSize) / sampleRate;
        static float lastBufferEndTime = 0.0f;
        static float previousSample = 0.0f;

        // Current time information
        float currentTime = radarParams.currentTime;
        float sampleDuration = 1.0f / sampleRate;
        
        // Calculate pulse timing
        float pulseInterval = 1.0f / radarParams.pulseRepFreq;
        float lastPulseTime = std::floor(currentTime / pulseInterval) * pulseInterval;
        
        // For stability, we'll generate each sample directly without buffer shifting
        for (int i = 0; i < bufferSize; i++) {
            // Calculate the absolute time for this sample
            float t = currentTime + (i * sampleDuration);
            float sample = 0.0f;
            
            // Loop through recent pulses (current and previous)
            for (int pulseIndex = 0; pulseIndex < 3; pulseIndex++) {
                float pulseTime = lastPulseTime - (pulseIndex * pulseInterval);
                
                // Skip pulses too far in the past
                if (currentTime - pulseTime > 2.0f) continue;
                
                // 1. Add reference ping for this pulse if it's recent
                if (pulseIndex == 0 && (t - pulseTime) < 0.2f) {
                    float pingTime = t - pulseTime;
                    
                    // Gaussian envelope for ping
                    float pingSigma = 0.03f;
                    float pingEnvelope = std::exp(-(pingTime*pingTime) / (2.0f * pingSigma * pingSigma));
                    
                    // Only add if envelope is significant
                    if (pingEnvelope > 0.01f) {
                        // Clean sine wave at 880Hz with envelope
                        float pingFreq = 880.0f;
                        float pingPhase = 2.0f * M_PI * pingFreq * pingTime;
                        sample += 0.2f * pingEnvelope * std::sin(pingPhase);
                    }
                }
                
                // 2. Add contributions from oscillators (echoes)
                for (int j = 1; j < MAX_OSCILLATORS; j++) {
                    if (!oscillators[j].active || oscillators[j].amplitude < 0.001f) continue;
                    
                    // Calculate when this echo should be heard
                    float echoTime = pulseTime + oscillators[j].roundTripTime;
                    
                    // Skip if the echo hasn't arrived yet or is too old
                    if (t < echoTime) continue;
                    
                    // Calculate time since echo
                    float timeSinceEcho = t - echoTime;
                    
                    // Skip if too old relative to envelope width
                    if (timeSinceEcho > 6.0f * oscillators[j].sigma) continue;
                    
                    // *** IMPROVED ONSET HANDLING ***
                    // Use a longer fade-in time (15ms) with a smoother curve
                    float fadeIn = 1.0f;
                    float fadeInDuration = 0.015f; // 15ms fade-in (was 5ms)
                    if (timeSinceEcho < fadeInDuration) {
                        // Smoother S-shaped curve (quintic instead of cubic)
                        float ratio = timeSinceEcho / fadeInDuration;
                        fadeIn = ratio * ratio * ratio * (10.0f - 15.0f * ratio + 6.0f * ratio * ratio);
                    }
                    
                    // Calculate Gaussian envelope with a softer onset
                    // This modifies the Gaussian to have a less steep initial rise
                    float sigma = oscillators[j].sigma;
                    float envelope;
                    if (timeSinceEcho < 0.5f * sigma) {
                        // Gentler onset for the first part of the envelope
                        envelope = std::exp(-(timeSinceEcho*timeSinceEcho) / (4.0f * sigma * sigma));
                    } else {
                        // Normal Gaussian for the rest
                        envelope = std::exp(-(timeSinceEcho*timeSinceEcho) / (2.0f * sigma * sigma));
                    }
                    
                    // Skip if envelope is negligible
                    if (envelope < 0.001f) continue;
                    
                    // *** PHASE OFFSET VARIATION ***
                    // Add a small unique phase offset for each oscillator to reduce constructive interference
                    // This prevents all echoes from having perfectly aligned phases
                    float phaseOffset = fmod(j * 0.1f, 2.0f * M_PI); // Different offset for each oscillator
                    
                    // Calculate phase for this time point with the offset
                    float phase = 2.0f * M_PI * oscillators[j].frequency * timeSinceEcho + phaseOffset;
                    
                    // Add contribution with combined fade-in and envelope
                    sample += oscillators[j].amplitude * envelope * fadeIn * std::sin(phase);
                }
            }
            
            // Soft clip to prevent harsh distortion
            if (std::abs(sample) > 0.8f) {
                float sign = (sample > 0.0f) ? 1.0f : -1.0f;
                sample = sign * (0.8f + tanh((std::abs(sample) - 0.8f) * 2.0f) * 0.2f);
            }
            
            // Store the result
            outputBuffer[i] = sample;
        }
        
        // Apply a gentle low-pass filter to remove high frequency artifacts
        float* tempBuffer = new float[bufferSize];
        std::copy(outputBuffer, outputBuffer + bufferSize, tempBuffer);
        
        // Simple 1-pole filter
        float cutoff = 10000.0f;
        float RC = 1.0f / (2.0f * M_PI * cutoff);
        float dt = 1.0f / sampleRate;
        float alpha = dt / (RC + dt);
        
        float lastSample = tempBuffer[0];
        for (int i = 0; i < bufferSize; i++) {
            outputBuffer[i] = tempBuffer[i] * alpha + lastSample * (1.0f - alpha);
            lastSample = outputBuffer[i];
        }
        
        delete[] tempBuffer;
        
        // Apply a very gradual volume adjustment for consistency
        static float lastMaxAmp = 0.5f;
        float maxAmp = 0.0f;
        
        for (int i = 0; i < bufferSize; i++) {
            maxAmp = std::max(maxAmp, std::abs(outputBuffer[i]));
        }
        
        // Extremely smooth amplitude adjustment
        float targetAmp = 0.6f;
        if (maxAmp > 0.001f) {
            // Very gradual approach to target amplitude
            lastMaxAmp = lastMaxAmp * 0.98f + maxAmp * 0.02f;
            
            float scale = 1.0f;
            if (lastMaxAmp > 0.8f) {
                scale = targetAmp / lastMaxAmp;
            } else if (lastMaxAmp < 0.3f) {
                scale = std::min(2.0f, targetAmp / lastMaxAmp);
            }
            
            // Apply scaling if significant
            if (std::abs(scale - 1.0f) > 0.05f) {
                for (int i = 0; i < bufferSize; i++) {
                    outputBuffer[i] *= scale;
                }
            }
        }
        
        // Log statistics occasionally
        static int logCounter = 0;
        if (++logCounter % 100 == 0) {
            int activeCount = 0;
            for (int i = 0; i < MAX_OSCILLATORS; i++) {
                if (oscillators[i].active) activeCount++;
            }
            
            printf("Audio stats: Active oscillators=%d, MaxAmp=%.4f\n", activeCount, maxAmp);
        }

        if (lastBufferEndTime > 0.0f) {
            // Smooth transition over the first ~20 samples
            for (int i = 0; i < 20 && i < bufferSize; i++) {
                float ratio = i / 20.0f;
                float fade = ratio * ratio * (3.0f - 2.0f * ratio); // Smooth cubic fade
                outputBuffer[i] = previousSample * (1.0f - fade) + outputBuffer[i] * fade;
            }
        }

        // Store the last sample and buffer end time for next iteration
        previousSample = outputBuffer[bufferSize-1];
        lastBufferEndTime = currentTime + bufferDuration;
    }
    
    // Debug utility to analyze oscillator state
    void debugOscillators() {
        int activeCount = 0;
        float maxAmp = 0.0f;
        float avgAmp = 0.0f;
        
        for (int i = 0; i < MAX_OSCILLATORS; i++) {
            if (oscillators[i].active) {
                activeCount++;
                maxAmp = std::max(maxAmp, oscillators[i].amplitude);
                avgAmp += oscillators[i].amplitude;
            }
        }
        
        if (activeCount > 0) {
            avgAmp /= activeCount;
        }
        
        printf("Oscillator stats: %d active, max amp: %.4f, avg amp: %.4f\n", 
              activeCount, maxAmp, avgAmp);
        
        // Print details of first few active oscillators
        printf("Active oscillators:\n");
        int printCount = std::min(5, activeCount);
        int printed = 0;
        
        for (int i = 0; i < MAX_OSCILLATORS && printed < printCount; i++) {
            if (oscillators[i].active) {
                printf("  [%d] freq: %.1f Hz, amp: %.4f, pos: (%.2f, %.2f, %.2f)\n",
                      i, oscillators[i].frequency, oscillators[i].amplitude,
                      oscillators[i].position[0], oscillators[i].position[1], 
                      oscillators[i].position[2]);
                printed++;
            }
        }
    }
    
    // Analyze buffer statistics
    void debugBuffer() {
        float maxAmp = 0.0f;
        float avgAmp = 0.0f;
        int nonZeroSamples = 0;
        
        for (int i = 0; i < INTERNAL_BUFFER_SIZE; i++) {
            float amplitude = std::abs(internalBuffer[i]);
            maxAmp = std::max(maxAmp, amplitude);
            avgAmp += amplitude;
            if (amplitude > 0.0001f) nonZeroSamples++;
        }
        
        avgAmp /= INTERNAL_BUFFER_SIZE;
        
        printf("Buffer stats - Max amplitude: %.6f, Avg amplitude: %.6f, Non-zero: %d/%d\n", 
              maxAmp, avgAmp, nonZeroSamples, INTERNAL_BUFFER_SIZE);
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
                                        float reflectionRadius,
                                        ProcessedPoint* outputPoints,
                                        int* outputPointCount) {
        if (processor) {
            processor->processGeometry(objects, objectCount, listenerPosition,
                                    listenerForward, maxDistance, reflectionRadius,
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
    
    EXPORT_API void DebugOscillators() {
        if (processor) {
            processor->debugOscillators();
        }
    }
    
    EXPORT_API void DebugBuffer() {
        if (processor) {
            processor->debugBuffer();
        }
    }
    
    EXPORT_API void Cleanup() {
        delete processor;
        processor = nullptr;
    }
}