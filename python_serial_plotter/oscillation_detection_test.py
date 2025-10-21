import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks, hilbert
from scipy.optimize import curve_fit

RE_MOTOR_DATA = re.compile("(\d+.\d+) : (\d+.\d+)")
REAL_MOTOR_DATA = """
E (43688) OSC_DETECT: 41.59 : 93.03
E (43688) OSC_DETECT: 41.60 : 91.63
E (43688) OSC_DETECT: 41.62 : 88.73
E (43688) OSC_DETECT: 41.63 : 87.30
E (43688) OSC_DETECT: 41.64 : 85.94
E (43688) OSC_DETECT: 41.65 : 84.59
E (43698) OSC_DETECT: 41.66 : 83.19
E (43698) OSC_DETECT: 41.67 : 81.85
E (43698) OSC_DETECT: 41.68 : 80.51
E (43698) OSC_DETECT: 41.69 : 79.10
E (43698) OSC_DETECT: 41.71 : 76.33
E (43698) OSC_DETECT: 41.72 : 74.97
E (43698) OSC_DETECT: 41.73 : 73.54
E (43698) OSC_DETECT: 41.74 : 72.16
E (43708) OSC_DETECT: 41.75 : 70.64
E (43708) OSC_DETECT: 41.76 : 69.15
E (43708) OSC_DETECT: 41.77 : 67.74
E (43708) OSC_DETECT: 41.79 : 65.19
E (43708) OSC_DETECT: 41.80 : 63.90
E (43708) OSC_DETECT: 41.81 : 62.62
E (43708) OSC_DETECT: 41.82 : 61.24
E (43708) OSC_DETECT: 41.83 : 59.83
E (43718) OSC_DETECT: 41.84 : 58.56
E (43718) OSC_DETECT: 41.85 : 57.33
E (43718) OSC_DETECT: 41.86 : 56.03
E (43718) OSC_DETECT: 41.88 : 53.39
E (43718) OSC_DETECT: 41.89 : 52.12
E (43718) OSC_DETECT: 41.90 : 50.89
E (43718) OSC_DETECT: 41.91 : 49.70
E (43728) OSC_DETECT: 41.92 : 48.58
E (43728) OSC_DETECT: 41.93 : 47.33
E (43728) OSC_DETECT: 41.94 : 46.05
E (43728) OSC_DETECT: 41.95 : 44.67
E (43728) OSC_DETECT: 41.97 : 41.84
E (43728) OSC_DETECT: 41.98 : 40.54
E (43728) OSC_DETECT: 41.99 : 39.31
E (43728) OSC_DETECT: 42.00 : 38.10
E (43738) OSC_DETECT: 42.01 : 36.87
E (43738) OSC_DETECT: 42.02 : 35.55
E (43738) OSC_DETECT: 42.03 : 34.10
E (43738) OSC_DETECT: 42.04 : 32.65
E (43738) OSC_DETECT: 42.06 : 29.99
E (43738) OSC_DETECT: 42.07 : 28.56
E (43738) OSC_DETECT: 42.08 : 27.00
E (43738) OSC_DETECT: 42.09 : 25.55
E (43748) OSC_DETECT: 42.10 : 24.17
E (43748) OSC_DETECT: 42.11 : 22.85
E (43748) OSC_DETECT: 42.12 : 21.60
E (43748) OSC_DETECT: 42.14 : 18.90
E (43748) OSC_DETECT: 42.15 : 17.31
E (43748) OSC_DETECT: 42.16 : 16.04
E (43748) OSC_DETECT: 42.17 : 15.31
E (43748) OSC_DETECT: 42.18 : 15.16
E (43758) OSC_DETECT: 42.19 : 15.27
E (43758) OSC_DETECT: 42.20 : 15.64
E (43758) OSC_DETECT: 42.21 : 16.08
E (43758) OSC_DETECT: 42.23 : 17.38
E (43758) OSC_DETECT: 42.24 : 18.22
E (43758) OSC_DETECT: 42.25 : 19.12
E (43758) OSC_DETECT: 42.26 : 20.08
E (43758) OSC_DETECT: 42.27 : 21.12
E (43768) OSC_DETECT: 42.28 : 21.95
E (43768) OSC_DETECT: 42.29 : 22.21
E (43768) OSC_DETECT: 42.30 : 22.24
E (43768) OSC_DETECT: 42.32 : 21.93
E (43768) OSC_DETECT: 42.33 : 21.58
E (43768) OSC_DETECT: 42.34 : 21.12
E (43768) OSC_DETECT: 42.35 : 20.54
E (43778) OSC_DETECT: 42.36 : 19.84
E (43778) OSC_DETECT: 42.37 : 18.96
E (43778) OSC_DETECT: 42.38 : 18.22
E (43778) OSC_DETECT: 42.39 : 17.97
E (43778) OSC_DETECT: 42.41 : 18.11
E (43778) OSC_DETECT: 42.42 : 18.46
E (43778) OSC_DETECT: 42.43 : 18.92
E (43778) OSC_DETECT: 42.44 : 19.47
E (43788) OSC_DETECT: 42.45 : 20.13
E (43788) OSC_DETECT: 42.46 : 20.87
E (43788) OSC_DETECT: 42.47 : 21.47
E (43788) OSC_DETECT: 42.48 : 21.62
E (43788) OSC_DETECT: 42.50 : 21.51
E (43788) OSC_DETECT: 42.51 : 21.25
E (43788) OSC_DETECT: 42.52 : 20.85
E (43788) OSC_DETECT: 42.53 : 20.32
E (43798) OSC_DETECT: 42.54 : 19.71
E (43798) OSC_DETECT: 42.55 : 18.94
E (43798) OSC_DETECT: 42.56 : 18.28
E (43798) OSC_DETECT: 42.57 : 18.11
E (43798) OSC_DETECT: 42.59 : 18.26
E (43798) OSC_DETECT: 42.60 : 18.57
E (43798) OSC_DETECT: 42.61 : 19.03
E (43798) OSC_DETECT: 42.62 : 19.56
E (43808) OSC_DETECT: 42.63 : 20.17
E (43808) OSC_DETECT: 42.64 : 20.90
E (43808) OSC_DETECT: 42.65 : 21.47
E (43808) OSC_DETECT: 42.67 : 21.60
E (43808) OSC_DETECT: 42.68 : 21.49
E (43808) OSC_DETECT: 42.69 : 21.23
E (43808) OSC_DETECT: 42.70 : 20.85
E (43808) OSC_DETECT: 42.71 : 20.32
E (43818) OSC_DETECT: 42.72 : 19.71
E (43818) OSC_DETECT: 42.73 : 18.96
E (43818) OSC_DETECT: 42.74 : 18.30
E (43818) OSC_DETECT: 42.76 : 18.13
E (43818) OSC_DETECT: 42.77 : 18.28
E (43818) OSC_DETECT: 42.78 : 18.63
E (43818) OSC_DETECT: 42.79 : 19.07
E (43828) OSC_DETECT: 42.80 : 19.62
E (43828) OSC_DETECT: 42.81 : 20.26
E (43828) OSC_DETECT: 42.82 : 21.01
E (43828) OSC_DETECT: 42.83 : 21.56
E (43828) OSC_DETECT: 42.85 : 21.67
E (43828) OSC_DETECT: 42.86 : 21.60
E (43828) OSC_DETECT: 42.87 : 21.36
E (43828) OSC_DETECT: 42.88 : 21.01
E (43838) OSC_DETECT: 42.89 : 20.52
E (43838) OSC_DETECT: 42.90 : 19.91
E (43838) OSC_DETECT: 42.91 : 19.14
E (43838) OSC_DETECT: 42.92 : 18.48
E (43838) OSC_DETECT: 42.94 : 18.30
E (43838) OSC_DETECT: 42.95 : 18.41
E (43838) OSC_DETECT: 42.96 : 18.74
E (43838) OSC_DETECT: 42.97 : 19.18
E (43848) OSC_DETECT: 42.98 : 19.75
E (43848) OSC_DETECT: 42.99 : 20.39
E (43848) OSC_DETECT: 43.00 : 21.12
E (43848) OSC_DETECT: 43.02 : 21.82
E (43848) OSC_DETECT: 43.03 : 21.82
E (43848) OSC_DETECT: 43.04 : 21.69
E (43848) OSC_DETECT: 43.05 : 21.38
E (43848) OSC_DETECT: 43.06 : 21.03
E (43858) OSC_DETECT: 43.07 : 20.52
E (43858) OSC_DETECT: 43.08 : 19.91
E (43858) OSC_DETECT: 43.09 : 19.14
E (43858) OSC_DETECT: 43.11 : 18.26
E (43858) OSC_DETECT: 43.12 : 18.26
E (43858) OSC_DETECT: 43.13 : 18.39
E (43858) OSC_DETECT: 43.14 : 18.72
E (43868) OSC_DETECT: 43.15 : 19.16
E (43868) OSC_DETECT: 43.16 : 19.71
E (43868) OSC_DETECT: 43.17 : 20.35
E (43868) OSC_DETECT: 43.18 : 21.09
E (43868) OSC_DETECT: 43.20 : 21.80
E (43868) OSC_DETECT: 43.21 : 21.80
E (43868) OSC_DETECT: 43.22 : 21.69
E (43868) OSC_DETECT: 43.23 : 21.40
E (43878) OSC_DETECT: 43.24 : 21.03
E (43878) OSC_DETECT: 43.25 : 20.52
E (43878) OSC_DETECT: 43.26 : 19.89
E (43878) OSC_DETECT: 43.27 : 19.14
E (43878) OSC_DETECT: 43.29 : 18.28
E (43878) OSC_DETECT: 43.30 : 18.26
E (43878) OSC_DETECT: 43.31 : 18.39
E (43878) OSC_DETECT: 43.32 : 18.74
E (43888) OSC_DETECT: 43.33 : 19.16
E (43888) OSC_DETECT: 43.34 : 19.73
E (43888) OSC_DETECT: 43.35 : 20.37
E (43888) OSC_DETECT: 43.37 : 21.69
E (43888) OSC_DETECT: 43.38 : 21.82
E (43888) OSC_DETECT: 43.39 : 21.82
E (43888) OSC_DETECT: 43.40 : 21.71
E (43888) OSC_DETECT: 43.41 : 21.42
E (43898) OSC_DETECT: 43.42 : 21.07
E (43898) OSC_DETECT: 43.43 : 20.57
E (43898) OSC_DETECT: 43.44 : 19.95
E (43898) OSC_DETECT: 43.46 : 18.50
E (43898) OSC_DETECT: 43.47 : 18.35
E (43898) OSC_DETECT: 43.48 : 18.33
E (43898) OSC_DETECT: 43.49 : 18.48
E (43908) OSC_DETECT: 43.50 : 18.83
E (43908) OSC_DETECT: 43.51 : 19.27
E (43908) OSC_DETECT: 43.52 : 19.82
E (43908) OSC_DETECT: 43.53 : 20.46
E (43908) OSC_DETECT: 43.55 : 21.77
E (43908) OSC_DETECT: 43.56 : 21.86
E (43908) OSC_DETECT: 43.57 : 21.86
E (43908) OSC_DETECT: 43.58 : 21.77
E (43918) OSC_DETECT: 43.59 : 21.51
E (43918) OSC_DETECT: 43.60 : 21.14
E (43918) OSC_DETECT: 43.61 : 20.63
E (43918) OSC_DETECT: 43.62 : 20.04
E (43918) OSC_DETECT: 43.64 : 18.39
E (43918) OSC_DETECT: 43.65 : 17.62
E (43918) OSC_DETECT: 43.66 : 17.36
E (43918) OSC_DETECT: 43.67 : 17.34
E (43928) OSC_DETECT: 43.68 : 17.56
E (43928) OSC_DETECT: 43.69 : 17.95
E (43928) OSC_DETECT: 43.70 : 18.41
E (43928) OSC_DETECT: 43.72 : 19.62
E (43928) OSC_DETECT: 43.73 : 20.32
E (43928) OSC_DETECT: 43.74 : 21.18
E (43928) OSC_DETECT: 43.75 : 21.84
E (43928) OSC_DETECT: 43.76 : 21.97
E (43938) OSC_DETECT: 43.77 : 21.97
E (43938) OSC_DETECT: 43.78 : 21.93
E (43938) OSC_DETECT: 43.79 : 21.67
E (43938) OSC_DETECT: 43.81 : 20.83
E (43938) OSC_DETECT: 43.82 : 20.21
E (43938) OSC_DETECT: 43.83 : 19.47
"""

def analyze_amplitude_stability(angle_data, time, sampling_rate=100):
    """
    Analyze amplitude stability by tracking peak amplitudes over time
    """
    # Find peaks (maxima)
    peaks, _ = find_peaks(angle_data, distance=sampling_rate//10)  # assuming min 0.1s between peaks
    peak_amplitudes = angle_data[peaks]
    peak_times = time[peaks]
    
    # Fit trend to peak amplitudes
    def linear_trend(x, a, b):
        return a * x + b
    
    if len(peak_amplitudes) > 2:
        popt, _ = curve_fit(linear_trend, peak_times, peak_amplitudes)
        slope = popt[0]
        
        # Calculate amplitude variation
        amp_std = np.std(peak_amplitudes)
        amp_mean = np.mean(peak_amplitudes)
        amplitude_coefficient_of_variation = amp_std / amp_mean if amp_mean != 0 else np.inf
        
        stability = "stable" if abs(slope) < 0.1 * amp_mean and amplitude_coefficient_of_variation < 0.2 else "unstable"
        
        if len(peak_times) % 2 == 1:
            period = np.mean([peak_times[i+1] - peak_times[i] for i in range(0, len(peak_times)-1, 2)])
        else:
            period = np.mean([peak_times[i+1] - peak_times[i] for i in range(0, len(peak_times), 2)])
        return {
            'stability': stability,
            'amplitude_slope': slope,
            'amplitude_cv': amplitude_coefficient_of_variation,
            'peak_amplitudes': peak_amplitudes,
            'peak_times': peak_times,
            'period': period
        }
    
    return None

def generate_test_data():
    # Test the analysis
    time = np.arange(0, 10, 0.01)  # 10 seconds, 0.01s sampling
    data = {}
    # Stable oscillation
    data['stable_data'] = 2 * np.sin(2 * np.pi * 1.0 * time)  # 1 Hz

    # Unstable oscillation (growing amplitude)
    data['growing_data'] = 2 * np.exp(0.1 * time) * np.sin(2 * np.pi * 1 * time)
    data['desceding_data'] = 2 * np.exp(-0.1 * time) * np.sin(2 * np.pi * 1 * time)
    return time, data


def parse_motor_data():
    pairs = RE_MOTOR_DATA.findall(REAL_MOTOR_DATA)
    time = np.array([float(p[0]) for p in pairs])
    data = np.array([float(p[1]) for p in pairs])
    print(', '.join([p[0] for p in pairs]))    
    print(', '.join([p[1] for p in pairs]))    
    return time, {'realmd': data}
    
time, data = generate_test_data()
time, data = parse_motor_data()

for data_key in data:
    print("Analyzing {} oscillation:".format(data_key))
    result = analyze_amplitude_stability(data[data_key], time)
    for key, val in result.items():
        print(key, ": ",val)

# Plot results
plt.figure(figsize=(12, 8))
for n, data_key in enumerate(data):
    plt.subplot(len(data), 1, n + 1)
    plt.plot(time, data[data_key])
    plt.title(data_key)
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')

plt.tight_layout()
plt.show()