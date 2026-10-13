"""
VIGIL — Wireless Motor Health Monitoring System
Live FFT Bearing Fault Detection Demo

Simulates:
STM32F411 + MPU6050 + CMSIS-DSP FFT pipeline

Pipeline:
4000 Hz sampling
1024 vibration samples
Hanning window
1024 point FFT
Bearing fault frequency monitoring
BPFO / BPFI / BSF detection
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from scipy.signal import find_peaks


# ==========================================================
# SYSTEM PARAMETERS (same as embedded firmware)
# ==========================================================

FS = 4000                 # MPU6050 sampling frequency (Hz)
N = 1024                  # STM32 FFT size
RPM = 1500
SHAFT_FREQ = RPM / 60

t = np.arange(N) / FS


# ==========================================================
# 6205 BEARING PARAMETERS
# ==========================================================

n = 9                     # number of balls
Bd = 7.94                 # ball diameter (mm)
Pd = 38.5                 # pitch diameter (mm)
theta = np.deg2rad(0)


BPFO = (n / 2) * SHAFT_FREQ * (
    1 - (Bd / Pd) * np.cos(theta)
)

BPFI = (n / 2) * SHAFT_FREQ * (
    1 + (Bd / Pd) * np.cos(theta)
)

BSF = (Pd / (2 * Bd)) * SHAFT_FREQ * (
    1 - ((Bd / Pd) * np.cos(theta)) ** 2
)


print("\n====== VIGIL Bearing Frequencies ======")
print(f"BPFO : {BPFO:.2f} Hz")
print(f"BPFI : {BPFI:.2f} Hz")
print(f"BSF  : {BSF:.2f} Hz")
print("=======================================\n")


# ==========================================================
# SIGNAL GENERATION
# ==========================================================

def generate_signal(fault=False):

    # normal motor vibration
    signal = (
        1.0 * np.sin(2*np.pi*25*t) +
        0.45 * np.sin(2*np.pi*50*t) +
        0.25 * np.sin(2*np.pi*75*t)
    )

    # sensor/environment noise
    signal += np.random.normal(
        0,
        0.25,
        N
    )


    if fault:

        # BPFO impact events
        impulse_train = np.zeros(N)

        interval = int(FS / BPFO)

        for i in range(0, N, interval):
            impulse_train[i] = 1


        # bearing housing impact response
        decay = np.exp(
            -np.linspace(0, 6, 80)
        )

        impacts = np.convolve(
            impulse_train,
            decay,
            mode="same"
        )


        # add BPFO harmonics for clear FFT fault signature
        signal += (
            3.0 * impacts +
            0.8*np.sin(2*np.pi*BPFO*t) +
            0.5*np.sin(2*np.pi*2*BPFO*t) +
            0.3*np.sin(2*np.pi*3*BPFO*t)
        )


    return signal



# ==========================================================
# FFT PROCESSING (CMSIS-DSP equivalent)
# ==========================================================

def compute_fft(signal):

    # Hanning window exactly like firmware preprocessing
    window = np.hanning(N)

    windowed = signal * window


    fft_result = np.fft.rfft(windowed)

    magnitude = (
        2.0 / N *
        np.abs(fft_result)
    )

    freq = np.fft.rfftfreq(
        N,
        1 / FS
    )

    return freq, magnitude



# ==========================================================
# BASELINE CREATION
# ==========================================================

baseline_signal = generate_signal(False)

freq_axis, baseline_fft = compute_fft(
    baseline_signal
)

bpfo_bin = np.argmin(
    np.abs(freq_axis - BPFO)
)

baseline_bpfo_energy = baseline_fft[bpfo_bin]


# ==========================================================
# ALERT LOGIC
# ==========================================================

def get_alert(current):

    ratio = current / baseline_bpfo_energy


    if ratio >= 1.8:
        return (
            ratio,
            "Level 3 — Critical (>1.8×)",
            "red"
        )

    elif ratio >= 1.4:
        return (
            ratio,
            "Level 2 — Warning (>1.4×)",
            "orange"
        )

    elif ratio >= 1.2:
        return (
            ratio,
            "Level 1 — Watch (>1.2×)",
            "yellow"
        )

    else:
        return (
            ratio,
            "Level 0 — Normal",
            "lime"
        )



# ==========================================================
# MATPLOTLIB LIVE DEMO
# ==========================================================

plt.style.use("dark_background")


fig, (ax_time, ax_fft) = plt.subplots(
    2,
    1,
    figsize=(12, 8)
)

plt.subplots_adjust(
    bottom=0.18,
    hspace=0.35
)


fault_mode = False



# ------------------------------
# Initial plots
# ------------------------------

signal = generate_signal(False)

freq, spectrum = compute_fft(signal)


time_line, = ax_time.plot(
    t * 1000,
    signal,
    linewidth=1
)


fft_line, = ax_fft.plot(
    freq,
    spectrum,
    linewidth=1.5
)



# ==========================================================
# FORMAT AXES
# ==========================================================

fig.suptitle(
    "VIGIL — Live FFT Bearing Fault Detection Demo",
    fontsize=16
)


ax_time.set_title(
    "MPU-6050 Vibration Signal"
)

ax_time.set_xlabel(
    "Time (ms)"
)

ax_time.set_ylabel(
    "Acceleration amplitude"
)


ax_fft.set_title(
    "STM32 CMSIS-DSP 1024 Point FFT Spectrum"
)

ax_fft.set_xlabel(
    "Frequency (Hz)"
)

ax_fft.set_ylabel(
    "Amplitude"
)

ax_fft.set_xlim(
    0,
    500
)


ax_fft.set_ylim(
    0,
    2
)



# ==========================================================
# FAULT MARKERS
# ==========================================================

markers = [
    (BPFO, "BPFO", "red"),
    (BPFI, "BPFI", "orange"),
    (BSF, "BSF", "yellow")
]


for value, name, color in markers:

    ax_fft.axvline(
        value,
        linestyle="--",
        color=color,
        linewidth=1.4
    )

    ax_fft.text(
        value + 3,
        1.7,
        f"{name}\n{value:.1f}Hz",
        color=color
    )



alert_text = ax_fft.text(
    260,
    1.45,
    "",
    fontsize=13
)


mode_text = ax_time.text(
    20,
    2.5,
    "NORMAL MODE",
    fontsize=14,
    color="lime"
)



# ==========================================================
# UPDATE FUNCTION
# ==========================================================

def update_plot():

    global fault_mode


    sig = generate_signal(
        fault_mode
    )

    freq, mag = compute_fft(
        sig
    )


    time_line.set_ydata(
        sig
    )

    fft_line.set_ydata(
        mag
    )


    current_energy = mag[bpfo_bin]

    ratio, message, color = get_alert(
        current_energy
    )


    alert_text.set_text(
        f"{message}\nBPFO Ratio: {ratio:.2f}×"
    )

    alert_text.set_color(
        color
    )


    if fault_mode:

        mode_text.set_text(
            "FAULT MODE — Outer Race Defect"
        )

        mode_text.set_color(
            "red"
        )

    else:

        mode_text.set_text(
            "NORMAL MODE"
        )

        mode_text.set_color(
            "lime"
        )


    fig.canvas.draw_idle()



# ==========================================================
# BUTTON
# ==========================================================

button_axis = plt.axes(
    [0.40, 0.05, 0.22, 0.06]
)

button = Button(
    button_axis,
    "Toggle Fault Mode"
)


def toggle(event):

    global fault_mode

    fault_mode = not fault_mode

    update_plot()



button.on_clicked(
    toggle
)


update_plot()


plt.show()