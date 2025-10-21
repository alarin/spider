#include "oscilation_detector.h"
#include <esp_log.h>

static const char* TAG = "OSC_DETECT";

void AmplitudeStabilityAnalyzer::addData(float angle, float time) {
    _angles.push_back(angle);
    _time.push_back(time);

    if (_angles.size() >= BUFFER_SIZE) {
        _angles.erase(_angles.begin());
        _time.erase(_time.begin());
    }
}

void AmplitudeStabilityAnalyzer::reset() {
    _angles.clear();
    _time.clear();
}

StabilityResult AmplitudeStabilityAnalyzer::analyzeAmplitudeStability(float sampling_rate) {
    
    StabilityResult result = {
        .is_stable = false,
        .amplitude_slope = 0.0f,
        .amplitude_cv = 0.0f,
        .peak_data = {.amplitudes = {}, .times = {}, .period = 0.0f}
    };
    
    // Input validation
    if (_angles.size() != _time.size() || _angles.size() < 20) {
        return result;
    }
    for(int i=0; i < _angles.size(); i++) {
        ESP_LOGE(TAG, "%.2f : %.2f", _time[i], _angles[i]);
    }
    
    // Find peaks in the signal
    _peak_data = findPeaks();
    
    // Need at least 3 peaks for meaningful analysis
    if (_peak_data.amplitudes.size() < 6) {
        // ESP_LOGE(TAG, "Not enought peaks detected %d, datapoints %d", _peak_data.amplitudes.size(), _angles.size());
        // for(int i=0; i < _angles.size(); i++) {
        //     ESP_LOGE(TAG, "%.2f", _angles[i]);
        // }
        return result;
    }
    
    // Calculate linear trend slope
    result.amplitude_slope = calculateSlope();
    
    // Calculate amplitude statistics
    float amp_mean = calculateMean();
    float amp_std = calculateStdDev();
    result.amplitude_cv = (amp_mean != 0.0f) ? (amp_std / amp_mean) : INFINITY;
    
    // Calculate average period
    result.peak_data = _peak_data;
    result.peak_data.period = calculateAveragePeriod();
    
    // Determine stability
    bool slope_stable = std::abs(result.amplitude_slope) < (STABILITY_SLOPE_THRESHOLD * amp_mean);
    bool cv_stable = result.amplitude_cv < STABILITY_CV_THRESHOLD;
    result.is_stable = slope_stable && cv_stable;
        
    return result;
}

PeakData AmplitudeStabilityAnalyzer::findPeaks() {
    
    PeakData peaks;
    int min_distance_samples = static_cast<int>(MIN_DISTANCE_S * sampling_rate);
    
    // Simple peak detection: local maxima with minimum distance
    for (size_t i = min_distance_samples; i < _angles.size() - min_distance_samples; i++) {
        bool is_peak = true;
        
        // Check if current point is greater than neighbors within the minimum distance
        for (int j = 1; j <= min_distance_samples; j++) {
            if (_angles[i] <= _angles[i - j] || _angles[i] <= _angles[i + j]) {
                is_peak = false;
                break;
            }
        }
        
        if (is_peak) {
            peaks.amplitudes.push_back(_angles[i]);
            peaks.times.push_back(_time[i]);
            
            // Skip ahead to avoid detecting multiple peaks in the same region
            i += min_distance_samples / 2;
        }
    }
    
    ESP_LOGD(TAG, "Found %d peaks", peaks.amplitudes.size());
    return peaks;
}

float AmplitudeStabilityAnalyzer::calculateSlope() {
    //peak_data._times, peak_data.amplitudes
    auto x = _peak_data.times;
    auto y = _peak_data.amplitudes;
    if (x.size() != y.size() || x.size() < 2) {
        return 0.0f;
    }
    
    // Simple linear regression: y = a + b*x
    float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_xx = 0.0f;
    size_t n = x.size();
    
    for (size_t i = 0; i < n; i++) {
        sum_x += x[i];
        sum_y += y[i];
        sum_xy += x[i] * y[i];
        sum_xx += x[i] * x[i];
    }
    
    float denominator = n * sum_xx - sum_x * sum_x;
    if (std::abs(denominator) < 1e-10f) {
        return 0.0f;
    }
    
    // Slope b = (n*Σxy - Σx*Σy) / (n*Σx² - (Σx)²)
    return (n * sum_xy - sum_x * sum_y) / denominator;
}

float AmplitudeStabilityAnalyzer::calculateMean() {
    auto data = _peak_data.amplitudes;
    if (data.empty()) return 0.0f;
    
    float sum = std::accumulate(data.begin(), data.end(), 0.0f);
    return sum / data.size();
}

float AmplitudeStabilityAnalyzer::calculateStdDev() {
    auto data = _peak_data.amplitudes;
    if (data.size() < 2) return 0.0f;
    
    float mean = calculateMean();
    float sum_sq_diff = 0.0f;
    
    for (float value : data) {
        float diff = value - mean;
        sum_sq_diff += diff * diff;
    }
    
    return std::sqrt(sum_sq_diff / (data.size() - 1));
}

float AmplitudeStabilityAnalyzer::calculateAveragePeriod() {
    auto peak_times = _peak_data.times;
    if (peak_times.size() < 2) return 0.0f;
    
    float total_period = 0.0f;
    int period_count = 0;
    
    for (size_t i = 1; i < peak_times.size(); i++) {
        total_period += (peak_times[i] - peak_times[i - 1]);
        period_count++;
    }
    
    return (period_count > 0) ? (total_period / period_count) : 0.0f;
}