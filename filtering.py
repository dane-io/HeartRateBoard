# %%
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

## Load dataset
raw_data = np.genfromtxt("DATASET_100Hz.txt")
raw_signal = raw_data[:,0]
raw_rates = raw_data[:,1]

# Remove NaNs
raw_rates = raw_rates[~np.isnan(raw_signal)]
raw_signal = raw_signal[~np.isnan(raw_signal)]
Fs = 100    # Sampling frequency

# Plot dataset
fig, axes = plt.subplots(2, 1, layout="constrained")
axes[0].plot(raw_signal)
axes[0].set_xlabel("Sample")
axes[0].set_ylabel("ADC value")
axes[0].set_ylim(0, 4096)
axes[0].set_title("Raw heart beat signal")
axes[0].grid(True)
axes[1].plot(raw_rates)
axes[1].set_xlabel("Sample")
axes[1].set_ylabel("Heart rate (BPM)")
axes[1].set_title("Reported heart rate")
axes[1].grid(True)
plt.show(block=False)

# %%
## Make FIR filter to filter raw signal
# Filter specifications (bandpass)
freq_vals_bpm = np.array([0, 30, 50, 150, 200, Fs/2*60])
freq_vals_normalized = freq_vals_bpm / (Fs/2) / 60

N_taps = 149 # Filter order
b = signal.firls(N_taps, freq_vals_normalized, np.array([0, 0, 1, 1, 0, 0]))
# Put into Q1.15 format
b_q15 = np.round(b*32767)

# Plot frequency response
[w, h] = signal.freqz(b_q15, 32767, 8192)
fig, ax = plt.subplots()
ax.plot(w/np.pi*Fs/2*60, np.abs(h))
ax.set_xlabel("Heart rate (BPM)")
ax.set_ylabel("Gain")
ax.set_title("Bandpass filter")
ax.grid(True)
plt.show(block=False)

# Make moving average FIR filter to find comparison value for zero crossings
N_zeros_taps = 512
b_zeros = 1/N_zeros_taps * np.ones(N_zeros_taps)
b_zeros_q15 = np.round(b_zeros*32767)

# Make moving average FIR filter to smooth estimated heart rates
N_bpm_taps = 8
b_bpm = 1/N_bpm_taps * np.ones(N_bpm_taps)
b_bpm_q15 = np.round(b_bpm*32767)

# %%
## Simulate extracting heart rate from zero crossings
filtered_signal = signal.lfilter(b_q15, 32767, raw_signal-2048)
filtered_zeros = signal.lfilter(b_zeros_q15, 32767, filtered_signal)
zero_cross_flag = filtered_signal[0] < 0   # Set on falling edge, reset on rising edge
heart_rates = np.zeros(len(filtered_signal))
start = 0
for sample in range(0, len(filtered_signal)):
    if filtered_signal[sample] < filtered_zeros[sample] and zero_cross_flag == 0:
        zero_cross_flag = 1
        # Record time between falling edges
        heart_rates[start:sample] = 1 / ( (sample-start) / Fs ) * 60
        start = sample; # Start timer
    elif filtered_signal[sample] > filtered_zeros[sample] and zero_cross_flag == 1:
        zero_cross_flag = 0

# Plot filtered signal and extracted heart rates
fig, axes = plt.subplots(2, 1, layout="constrained")
axes[0].plot(filtered_signal)
axes[0].plot(filtered_zeros)
axes[0].set_xlabel("Sample")
axes[0].set_ylabel("ADC value")
axes[0].set_title("Filtered heart beat signal")
axes[0].grid(True)
axes[1].plot(heart_rates)
axes[1].set_xlabel("Sample")
axes[1].set_ylabel("Heart rate (BPM)")
axes[1].set_title("Extracted heart rate")
axes[1].grid(True)
plt.show(block=False)

# %%
## Export to C format
# FIR filter for main signal
print()
print("// FIR filter for main signal")
print("#define SAMPLE_FREQ "+str(Fs))
print("#define NUM_TAPS "+str(N_taps+1))    # Add 1 to make even for ARM implementation
print("q15_t Filt_Coeffs[NUM_TAPS] = {", end="")
for coeff in b_q15:
    print(str(int(coeff))+", ", end="")
print("0};")    # Make last element 0 for even length filter
print("q15_t Filt_State[NUM_TAPS] = {0};")
print("arm_fir_instance_q15 S_FIR;")
print()

# FIR filter for zero crossing comparison
print("// FIR filter to find zero crossings (avg of current signal)")
print("#define NUM_ZEROS_TAPS "+str(N_zeros_taps))
print("q15_t Zeros_Filt_Coeffs[NUM_ZEROS_TAPS] = {", end="")
for coeff in b_zeros_q15[:-1]:
    print(str(int(coeff))+", ", end="")
print(str(int(b_zeros_q15[-1]))+"};")
print("q15_t Zeros_Filt_State[NUM_ZEROS_TAPS] = {0};")
print("arm_fir_instance_q15 S_ZEROS;")
print()

# FIR filter for heart rate estimation smoothing
print("// FIR filter to smooth heart rate estimations")
print("#define NUM_BPM_TAPS "+str(N_bpm_taps))
print("q15_t BPM_Filt_Coeffs[NUM_BPM_TAPS] = {", end="")
for coeff in b_bpm_q15[:-1]:
    print(str(int(coeff))+", ", end="")
print(str(int(b_bpm_q15[-1]))+"};")
print("q15_t BPM_Filt_State[NUM_BPM_TAPS] = {0};")
print("arm_fir_instance_q15 S_BPM;")
print()

# %%
## Keep plots open after code is done
plt.show()
