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
    
    // Initialize/reset the oscillator
    void reset() {
        active = false;
        frequency = 0.0f;
        targetFrequency = 0.0f;
        amplitude = 0.0f;
        targetAmplitude = 0.0f;
        phase = 0.0f;
        roundTripTime = 0.0f;
        sigma = 0.05f;
        
        for (int i = 0; i < 3; i++) {
            position[i] = 0.0f;
            normal[i] = 0.0f;
        }
    }
    
    // Update oscillator parameters with smooth transitions
    void update(float deltaTime) {
        if (!active) return;
        
        // Smoothly approach target values with simple lerp
        float lerpFactor = 5.0f * deltaTime; // 5.0 = smoothing rate
        lerpFactor = std::min(lerpFactor, 1.0f);
        
        frequency = frequency * (1.0f - lerpFactor) + targetFrequency * lerpFactor;
        amplitude = amplitude * (1.0f - lerpFactor) + targetAmplitude * lerpFactor;
        
        // Deactivate if amplitude becomes negligible
        if (amplitude < 0.0001f && targetAmplitude < 0.0001f) {
            active = false;
        }
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

    struct TimeFrequencyData {
        int timeSteps;
        int frequencyBins;
        float* magnitudes;     // Magnitude values
        float* phases;         // Phase values
        float minTime;
        float maxTime;
        float minFreq;
        float maxFreq;
    };

    
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
        
        // Skip oscillator 0 (reserved for reference ping)
        for (int p = 0; p < pointCount; p++) {
            const ProcessedPoint& point = points[p];
            
            // Calculate reflection time
            float roundTripTime = point.params.roundTripTime;
            
            // Find the best oscillator for this point
            int bestOscIndex = -1;
            float bestDistSq = 0.1f; // Distance threshold
            
            // Try to find an existing oscillator for this point
            for (int i = 1; i < MAX_OSCILLATORS; i++) {
                if (oscillators[i].active) {
                    // Calculate distance between positions
                    float dx = oscillators[i].position[0] - point.position[0];
                    float dy = oscillators[i].position[1] - point.position[1];
                    float dz = oscillators[i].position[2] - point.position[2];
                    float distSq = dx*dx + dy*dy + dz*dz;
                    
                    if (distSq < bestDistSq) {
                        bestDistSq = distSq;
                        bestOscIndex = i;
                    }
                }
            }
            
            // If no match found, find an inactive oscillator
            if (bestOscIndex < 0) {
                for (int i = 1; i < MAX_OSCILLATORS; i++) {
                    if (!oscillators[i].active) {
                        bestOscIndex = i;
                        break;
                    }
                }
            }
            
            // If still no match, find the quietest oscillator
            if (bestOscIndex < 0) {
                float lowestAmp = 1.0f;
                for (int i = 1; i < MAX_OSCILLATORS; i++) {
                    if (oscillators[i].amplitude < lowestAmp) {
                        lowestAmp = oscillators[i].amplitude;
                        bestOscIndex = i;
                    }
                }
            }
            
            // At this point we should have a valid oscillator index
            if (bestOscIndex > 0) {
                Oscillator& osc = oscillators[bestOscIndex];
                osc.active = true;
                
                // Set position and parameters
                for (int i = 0; i < 3; i++) {
                    osc.position[i] = point.position[i];
                    osc.normal[i] = point.normal[i];
                }
                
                osc.roundTripTime = roundTripTime;
                osc.sigma = point.params.sigma;
                osc.targetFrequency = point.params.dopplerFreq;
                osc.targetAmplitude = point.params.amplitude;
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
            // Remove this line: oscillators[i].lastUpdateTime = radarParams.currentTime;
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
        
        // Update all oscillators with the new delta time
        for (int i = 0; i < MAX_OSCILLATORS; i++) {
            // Change this line to match the simplified update method
            oscillators[i].update(deltaTime);
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
        
        // Current time and pulse timing
        float currentTime = radarParams.currentTime;
        float pulseInterval = 1.0f / radarParams.pulseRepFreq;
        float lastPulseTime = std::floor(currentTime / pulseInterval) * pulseInterval;
        
        // Duration of each sample
        float sampleDuration = 1.0f / sampleRate;
        
        // Generate reference ping if it's a recent pulse
        if (currentTime - lastPulseTime < 0.1f) {
            // Make sure oscillator 0 is a ping
            Oscillator& ping = oscillators[0];
            ping.active = true;
            ping.targetFrequency = 880.0f;
            ping.frequency = 880.0f;
            ping.targetAmplitude = 0.3f;
            ping.sigma = 0.05f;
            ping.roundTripTime = 0.0f; // Immediate
        }
        
        // Direct sample-by-sample generation
        for (int i = 0; i < bufferSize; i++) {
            // Calculate the absolute time for this sample
            float t = currentTime + (i * sampleDuration);
            float sample = 0.0f;
            
            // Sum contributions from all active oscillators
            for (int j = 0; j < MAX_OSCILLATORS; j++) {
                if (!oscillators[j].active || oscillators[j].amplitude < 0.001f) continue;
                
                // Calculate echo time
                float echoTime = lastPulseTime + oscillators[j].roundTripTime;
                
                // Skip if echo hasn't arrived yet or is too old
                if (t < echoTime) continue;
                float timeSinceEcho = t - echoTime;
                if (timeSinceEcho > 6.0f * oscillators[j].sigma) continue;
                
                // Gaussian envelope
                float envelope = std::exp(-(timeSinceEcho*timeSinceEcho) / 
                                    (2.0f * oscillators[j].sigma * oscillators[j].sigma));
                
                // Simple fade-in to prevent clicks
                float fadeIn = 1.0f;
                if (timeSinceEcho < 0.005f) { // 5ms fade-in
                    fadeIn = timeSinceEcho / 0.005f;
                }
                
                // Calculate the phase for this time point
                float phase = 2.0f * M_PI * oscillators[j].frequency * timeSinceEcho + oscillators[j].phase;
                
                // Add this oscillator's contribution
                sample += oscillators[j].amplitude * envelope * fadeIn * std::sin(phase);
            }
            
            // Simple soft clipping to prevent distortion
            if (std::abs(sample) > 0.9f) {
                float sign = (sample > 0) ? 1.0f : -1.0f;
                sample = sign * (0.9f + (std::abs(sample) - 0.9f) / 
                            (1.0f + (std::abs(sample) - 0.9f)));
            }
            
            // Store the result
            outputBuffer[i] = sample;
        }
        
        // Apply a gentle amplitude adjustment
        float maxAmp = 0.0f;
        for (int i = 0; i < bufferSize; i++) {
            maxAmp = std::max(maxAmp, std::abs(outputBuffer[i]));
        }
        
        // Scale if needed
        if (maxAmp > 0.001f) {
            float scale = 1.0f;
            if (maxAmp > 0.9f) {
                scale = 0.9f / maxAmp; // Prevent clipping
            } else if (maxAmp < 0.2f) {
                scale = 0.6f / maxAmp; // Boost weak signals
                scale = std::min(scale, 3.0f); // Limit boost
            }
            
            if (std::abs(scale - 1.0f) > 0.05f) {
                for (int i = 0; i < bufferSize; i++) {
                    outputBuffer[i] *= scale;
                }
            }
        }
    }

    TimeFrequencyData generateTimeFrequencyData(int timeSteps, int frequencyBins) {
        TimeFrequencyData result;
        result.timeSteps = timeSteps;
        result.frequencyBins = frequencyBins;
        
        // Allocate memory for both magnitude and phase data
        result.magnitudes = new float[timeSteps * frequencyBins];
        result.phases = new float[timeSteps * frequencyBins];
        std::fill(result.magnitudes, result.magnitudes + timeSteps * frequencyBins, 0.0f);
        std::fill(result.phases, result.phases + timeSteps * frequencyBins, 0.0f);
        
        // Time range: centered around current time
        float currentTime = radarParams.currentTime;
        result.minTime = currentTime - 0.5f;
        result.maxTime = currentTime + 0.5f;
        
        // Linear frequency range: 100Hz to 1000Hz
        result.minFreq = 100.0f;
        result.maxFreq = 1000.0f;
        
        // Calculate time and frequency step sizes
        float timeStep = (result.maxTime - result.minTime) / (timeSteps - 1);
        float freqStep = (result.maxFreq - result.minFreq) / (frequencyBins - 1);
        
        // Calculate most recent pulse time
        float pulseInterval = 1.0f / radarParams.pulseRepFreq;
        float lastPulseTime = std::floor(currentTime / pulseInterval) * pulseInterval;
        
        // Process each active oscillator
        for (int oscIndex = 0; oscIndex < MAX_OSCILLATORS; oscIndex++) {
            const Oscillator& osc = oscillators[oscIndex];
            if (!osc.active || osc.amplitude < 0.001f) continue;
            
            // Echo arrival time
            float echoTime = lastPulseTime + osc.roundTripTime;
            
            // Skip if echo is outside our time window
            if (echoTime < result.minTime || echoTime > result.maxTime) continue;
            
            // Calculate frequency bandwidth based on uncertainty principle
            // σf ≈ 1/(2π·σt) - the longer the time envelope, the narrower the frequency spread
            float freqSpread = 1.0f / (osc.sigma);
            
            // For each point in the time-frequency grid
            for (int t = 0; t < timeSteps; t++) {
                float time = result.minTime + t * timeStep;
                
                // Time relative to echo arrival
                float relativeTime = time - echoTime;
                
                // Gaussian envelope in time domain
                float timeEnvelope = std::exp(-(relativeTime*relativeTime) / 
                                        (2.0f * osc.sigma * osc.sigma));
                
                // Skip if envelope is negligible
                if (timeEnvelope < 0.01f) continue;
                
                for (int f = 0; f < frequencyBins; f++) {
                    // Calculate frequency for this bin (linear scale)
                    float freq = result.minFreq + f * freqStep;
                    
                    // Gaussian in frequency domain (width proportional to 1/sigma)
                    float freqDistance = (freq - osc.frequency) / freqSpread;
                    float freqEnvelope = std::exp(-(freqDistance*freqDistance) / 2.0f);
                    
                    // Calculate the magnitude contribution
                    float contribution = osc.amplitude * timeEnvelope * freqEnvelope;
                    
                    // Calculate phase for this time-frequency point
                    // We use the oscillator's base phase plus the phase accumulated over time
                    float phase = osc.phase + 2.0f * M_PI * osc.frequency * relativeTime;
                    phase = std::fmod(phase, 2.0f * M_PI); // Normalize to [0, 2π]
                    if (phase < 0) phase += 2.0f * M_PI;   // Ensure positive phase
                    
                    // Index in the flattened array
                    int idx = t * frequencyBins + f;
                    
                    // For magnitude, we use a complex addition approach to handle overlapping atoms
                    // This means converting magnitude and phase to complex numbers, adding them,
                    // then converting back to magnitude and phase
                    
                    // Get existing magnitude and phase
                    float existingMag = result.magnitudes[idx];
                    float existingPhase = result.phases[idx];
                    
                    // Convert to complex numbers
                    float existingReal = existingMag * std::cos(existingPhase);
                    float existingImag = existingMag * std::sin(existingPhase);
                    float newReal = contribution * std::cos(phase);
                    float newImag = contribution * std::sin(phase);
                    
                    // Add complex numbers
                    float resultReal = existingReal + newReal;
                    float resultImag = existingImag + newImag;
                    
                    // Convert back to magnitude and phase
                    float resultMag = std::sqrt(resultReal*resultReal + resultImag*resultImag);
                    float resultPhase = std::atan2(resultImag, resultReal);
                    
                    // Store the result
                    result.magnitudes[idx] = resultMag;
                    result.phases[idx] = resultPhase;
                }
            }
        }
        
        // Normalize the magnitude data to 0-1 range
        float maxValue = 0.0001f; // Avoid division by zero
        for (int i = 0; i < timeSteps * frequencyBins; i++) {
            maxValue = std::max(maxValue, result.magnitudes[i]);
        }
        
        if (maxValue > 0.0001f) {
            for (int i = 0; i < timeSteps * frequencyBins; i++) {
                result.magnitudes[i] /= maxValue;
            }
        }
        
        // No need to normalize phase as it's already in [0, 2π]
        
        return result;
    }

    // Updated cleanup method
    void freeTimeFrequencyData(TimeFrequencyData& data) {
        delete[] data.magnitudes;
        delete[] data.phases;
        data.magnitudes = nullptr;
        data.phases = nullptr;
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

    EXPORT_API void GetTimeFrequencyData(
        int timeSteps,
        int frequencyBins,
        float* outputMagnitudes,
        float* outputPhases,
        float* outputTimeRange,  // [minTime, maxTime]
        float* outputFreqRange   // [minFreq, maxFreq]
    ) {
        if (processor) {
            auto tfData = processor->generateTimeFrequencyData(timeSteps, frequencyBins);
            
            // Copy magnitude data
            std::copy(tfData.magnitudes, tfData.magnitudes + (timeSteps * frequencyBins), outputMagnitudes);
            
            // Copy phase data
            std::copy(tfData.phases, tfData.phases + (timeSteps * frequencyBins), outputPhases);
            
            // Copy time and frequency ranges
            outputTimeRange[0] = tfData.minTime;
            outputTimeRange[1] = tfData.maxTime;
            outputFreqRange[0] = tfData.minFreq;
            outputFreqRange[1] = tfData.maxFreq;
            
            // Clean up
            processor->freeTimeFrequencyData(tfData);
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