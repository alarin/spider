#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

struct PeakData {
    std::vector<float> amplitudes;
    std::vector<float> times;
    float period;
};

struct StabilityResult {
    bool is_stable;
    float amplitude_slope;
    float amplitude_cv;
    PeakData peak_data;
};

class AmplitudeStabilityAnalyzer {
private:
    static constexpr float MIN_DISTANCE_S = 0.1f; // Minimum 0.1s between peaks
    static constexpr float STABILITY_SLOPE_THRESHOLD = 0.1f;
    static constexpr float STABILITY_CV_THRESHOLD = 0.2f;

public:
    void addData(float angle, float time);
    void reset();
    /**
     * Analyze amplitude stability by tracking peak amplitudes over time
     * @param sampling_rate Sampling rate in Hz (default: 100Hz)
     * @return Stability analysis result
     */
    StabilityResult analyzeAmplitudeStability(float sampling_rate = 100.0f);
    
private:
    std::vector<float> _angles;
    std::vector<float> _time;
    std::vector<float> _peak_times;
    PeakData _peak_data;
    float sampling_rate = 100;
    const int BUFFER_SIZE = 200;

    /**
     * Find peaks in the signal using simple local maxima detection
     */
    PeakData findPeaks();
    
    /**
     * Calculate linear regression slope for peak amplitudes vs time
     */
    float calculateSlope();
    
    /**
     * Calculate mean of vector
     */
    float calculateMean();
    
    /**
     * Calculate standard deviation of vector
     */
    float calculateStdDev();
    
    /**
     * Calculate average period from peak times
     */
    float calculateAveragePeriod();
};