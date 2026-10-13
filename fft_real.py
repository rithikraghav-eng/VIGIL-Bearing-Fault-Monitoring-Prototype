"""
VIGIL — Wireless Motor Health Monitoring System
Live FFT Bearing Fault Detection — REAL SENSOR VERSION

Reads live accelerometer data from an Arduino over a serial (USB) port
and runs it through the same Hanning-window + FFT + bearing-fault-ratio
pipeline as the original simulation.

--------------------------------------------------------------------
EXPECTED ARDUINO OUTPUT FORMAT
--------------------------------------------------------------------
This script expects the Arduino to print ONE LINE per sample, as
comma-separated floats:

    ax,ay,az

e.g.

    0.02,-0.01,0.98
    0.03,-0.02,0.99
    ...

Print as fast as your loop allows (no delay()) for the best sample
rate. A minimal Arduino sketch (MPU6050 example, adjust to your IMU):

    #include <Wire.h>
    #include <MPU6050.h>
    MPU6050 mpu;

    void setup() {
      Serial.begin(115200);
      Wire.begin();
      mpu.initialize();
    }

    void loop() {
      int16_t ax, ay, az;
      mpu.getAcceleration(&ax, &ay, &az);
      Serial.print(ax / 16384.0, 4); Serial.print(",");
      Serial.print(ay / 16384.0, 4); Serial.print(",");
      Serial.println(az / 16384.0, 4);
    }

--------------------------------------------------------------------
IMPORTANT NOTES
--------------------------------------------------------------------
1. FS (sampling frequency) is no longer a fixed constant. Real serial
   throughput jitters, so this script measures the actual time between
   samples and computes an effective FS from that. The FFT frequency
   axis is built from this measured FS, so BPFO/BPFI/BSF markers stay
   accurate even if your real rate is 300 Hz instead of 4000 Hz.

2. Over a normal USB-serial link (115200 baud, CSV text) you will
   realistically get on the order of a few hundred Hz to ~1-2 kHz,
   NOT 4000 Hz. Nyquist means you can only reliably see fault
   frequencies below FS/2. If BPFO/BPFI/BSF markers fall above your
   measured Nyquist limit, they will not be visible — that's a
   hardware/link limitation, not a bug. To go faster: raise the baud
   rate, send binary instead of CSV text, or stream from an onboard
   buffer.

3. There is no fake "fault mode" anymore. Instead there's a
   "Capture Baseline" button — click it once while the motor is
   running healthy, and all later readings are compared against it.

4. RPM is still a constant assumption (used to compute BPFO/BPFI/BSF
   target frequencies). Edit RPM below to match your actual motor
   speed, or wire up a tachometer/hall sensor later and replace the
   constant.
"""

import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import serial
import serial.tools.list_ports


# ==========================================================
# SERIAL PORT CONFIGURATION — EDIT THESE
# ==========================================================

SERIAL_PORT = "COM9"        # <-- change to your Arduino's COM port
BAUD_RATE = 115200


# ==========================================================
# SYSTEM PARAMETERS
# ==========================================================

N = 1024                    # FFT window size (samples)
RPM = 1500                 # assumed motor speed — edit to match reality
SHAFT_FREQ = RPM / 60


# ==========================================================
# 6205 BEARING PARAMETERS
# ==========================================================

n_balls = 9                 # number of balls
Bd = 7.94                   # ball diameter (mm)
Pd = 38.5                   # pitch diameter (mm)
theta = np.deg2rad(0)

BPFO = (n_balls / 2) * SHAFT_FREQ * (1 - (Bd / Pd) * np.cos(theta))
BPFI = (n_balls / 2) * SHAFT_FREQ * (1 + (Bd / Pd) * np.cos(theta))
BSF = (Pd / (2 * Bd)) * SHAFT_FREQ * (1 - ((Bd / Pd) * np.cos(theta)) ** 2)

print("\n====== VIGIL Bearing Frequencies ======")
print(f"BPFO : {BPFO:.2f} Hz")
print(f"BPFI : {BPFI:.2f} Hz")
print(f"BSF  : {BSF:.2f} Hz")
print("=======================================\n")


# ==========================================================
# SERIAL READER (background thread)
# ==========================================================
# Holds the last N (timestamp, vibration_magnitude) samples.
# A background thread keeps this buffer filled so the GUI never
# blocks waiting on serial I/O.

buffer_lock = threading.Lock()
sample_buffer = deque(maxlen=N)   # each item: (timestamp, magnitude)
serial_ok = True
serial_error_msg = ""


def open_serial():
    try:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # allow Arduino to reset after port opens
        return ser, None
    except Exception as e:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        msg = (
            f"Could not open {SERIAL_PORT}: {e}\n"
            f"Available ports: {ports if ports else 'none detected'}"
        )
        return None, msg


def serial_reader_thread():
    global serial_ok, serial_error_msg

    ser, err = open_serial()
    if ser is None:
        serial_ok = False
        serial_error_msg = err
        print("[SERIAL ERROR]", err)
        return

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != 3:
                continue  # ignore malformed / partial lines

            ax, ay, az = (float(p) for p in parts)

            # vibration magnitude (removes dependence on sensor orientation)
            magnitude = np.sqrt(ax ** 2 + ay ** 2 + az ** 2)

            with buffer_lock:
                sample_buffer.append((time.time(), magnitude))

        except (ValueError, UnicodeDecodeError):
            continue  # skip garbled line
        except Exception as e:
            serial_ok = False
            serial_error_msg = str(e)
            print("[SERIAL ERROR]", e)
            break


reader_thread = threading.Thread(target=serial_reader_thread, daemon=True)
reader_thread.start()


def get_snapshot():
    """Return (times, magnitudes, effective_fs) for whatever is currently buffered."""
    with buffer_lock:
        data = list(sample_buffer)

    if len(data) < 2:
        return None, None, None

    times = np.array([d[0] for d in data])
    values = np.array([d[1] for d in data])

    # effective sample rate from measured timestamps
    dt = np.mean(np.diff(times))
    fs_effective = 1.0 / dt if dt > 0 else None

    return times, values, fs_effective


# ==========================================================
# FFT PROCESSING (CMSIS-DSP equivalent)
# ==========================================================

def compute_fft(signal_vals, fs):
    n_samples = len(signal_vals)

    # remove DC / gravity offset so the window centers on the AC vibration
    ac_signal = signal_vals - np.mean(signal_vals)

    window = np.hanning(n_samples)
    windowed = ac_signal * window

    fft_result = np.fft.rfft(windowed)
    magnitude = (2.0 / n_samples) * np.abs(fft_result)
    freq = np.fft.rfftfreq(n_samples, 1 / fs)

    return freq, magnitude


# ==========================================================
# BASELINE / ALERT LOGIC
# ==========================================================

baseline_bpfo_energy = None  # set by "Capture Baseline" button


def get_alert(current):
    if baseline_bpfo_energy is None or baseline_bpfo_energy == 0:
        return None, "No baseline captured yet", "gray"

    ratio = current / baseline_bpfo_energy

    if ratio >= 1.8:
        return ratio, "Level 3 — Critical (>1.8x)", "red"
    elif ratio >= 1.4:
        return ratio, "Level 2 — Warning (>1.4x)", "orange"
    elif ratio >= 1.2:
        return ratio, "Level 1 — Watch (>1.2x)", "yellow"
    else:
        return ratio, "Level 0 — Normal", "lime"


# ==========================================================
# MATPLOTLIB LIVE DEMO
# ==========================================================

plt.style.use("dark_background")

fig, (ax_time, ax_fft) = plt.subplots(2, 1, figsize=(12, 8))
plt.subplots_adjust(bottom=0.18, hspace=0.35)

fig.suptitle("VIGIL — Live FFT Bearing Fault Detection (Real Sensor)", fontsize=16)

ax_time.set_title("Live Accelerometer Vibration Magnitude")
ax_time.set_xlabel("Time (ms, relative)")
ax_time.set_ylabel("Acceleration magnitude (g)")

ax_fft.set_title("Live FFT Spectrum")
ax_fft.set_xlabel("Frequency (Hz)")
ax_fft.set_ylabel("Amplitude")
ax_fft.set_xlim(0, 500)
ax_fft.set_ylim(0, 2)

time_line, = ax_time.plot([], [], linewidth=1)
fft_line, = ax_fft.plot([], [], linewidth=1.5)

markers = [(BPFO, "BPFO", "red"), (BPFI, "BPFI", "orange"), (BSF, "BSF", "yellow")]
for value, name, color in markers:
    ax_fft.axvline(value, linestyle="--", color=color, linewidth=1.4)
    ax_fft.text(value + 3, 1.7, f"{name}\n{value:.1f}Hz", color=color)

alert_text = ax_fft.text(200, 1.45, "", fontsize=13)

status_text = ax_time.text(
    0.02, 0.9, "Waiting for serial data...",
    transform=ax_time.transAxes, fontsize=12, color="yellow"
)


# ==========================================================
# LIVE UPDATE LOOP
# ==========================================================

def update_plot(frame=None):
    global baseline_bpfo_energy

    if not serial_ok:
        status_text.set_text(f"SERIAL ERROR:\n{serial_error_msg}")
        status_text.set_color("red")
        fig.canvas.draw_idle()
        return (time_line, fft_line)

    times, values, fs_eff = get_snapshot()

    if times is None or len(values) < 32 or fs_eff is None:
        status_text.set_text(
            f"Buffering... ({len(sample_buffer)}/{N} samples)"
        )
        status_text.set_color("yellow")
        fig.canvas.draw_idle()
        return (time_line, fft_line)

    status_text.set_text(f"Live — {len(values)} samples, ~{fs_eff:.0f} Hz measured")
    status_text.set_color("lime")

    # time-domain plot (relative ms)
    t_ms = (times - times[0]) * 1000
    time_line.set_data(t_ms, values)
    ax_time.set_xlim(t_ms[0], t_ms[-1])
    ax_time.relim()
    ax_time.autoscale_view(scalex=False)

    # FFT
    freq, mag = compute_fft(values, fs_eff)
    fft_line.set_data(freq, mag)
    ax_fft.set_xlim(0, min(500, fs_eff / 2))

    bpfo_bin = np.argmin(np.abs(freq - BPFO))
    current_energy = mag[bpfo_bin]

    ratio, message, color = get_alert(current_energy)
    if ratio is None:
        alert_text.set_text(message)
    else:
        alert_text.set_text(f"{message}\nBPFO Ratio: {ratio:.2f}x")
    alert_text.set_color(color)

    fig.canvas.draw_idle()
    return (time_line, fft_line)


# ==========================================================
# BUTTON — CAPTURE BASELINE
# ==========================================================

button_axis = plt.axes([0.38, 0.05, 0.26, 0.06])
button = Button(button_axis, "Capture Baseline")


def capture_baseline(event):
    global baseline_bpfo_energy

    times, values, fs_eff = get_snapshot()
    if times is None or len(values) < 32 or fs_eff is None:
        alert_text.set_text("Not enough data yet — wait a moment and try again")
        alert_text.set_color("yellow")
        fig.canvas.draw_idle()
        return

    freq, mag = compute_fft(values, fs_eff)
    bpfo_bin = np.argmin(np.abs(freq - BPFO))
    baseline_bpfo_energy = mag[bpfo_bin]

    alert_text.set_text(f"Baseline captured (BPFO energy = {baseline_bpfo_energy:.4f})")
    alert_text.set_color("cyan")
    fig.canvas.draw_idle()


button.on_clicked(capture_baseline)


# ==========================================================
# ANIMATION LOOP
# ==========================================================

from matplotlib.animation import FuncAnimation

ani = FuncAnimation(fig, update_plot, interval=200, cache_frame_data=False)

plt.show()