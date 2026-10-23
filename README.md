# VIGIL - Bearing Fault Monitoring Prototype

VIGIL is an experimental motor-health monitoring project exploring vibration analysis for rolling-element bearing faults. This repository contains two Python applications:

- **Simulation mode (`fft_sim.py`)** — generates synthetic healthy and outer-race fault vibration signals and demonstrates an FFT-based BPFO alert pipeline.
- **Live sensor mode (`fft_real.py`)** — reads three-axis accelerometer data over USB serial, computes a live FFT, and compares the amplitude near the calculated BPFO frequency against a baseline captured while the motor is running.

The broader VIGIL concept explores low-cost wireless motor condition monitoring. These scripts are development tools for evaluating the signal-processing approach; they are not a validated or certified diagnostic system.

## Repository contents

```text
VIGIL/
├── fft_sim.py       # Synthetic vibration simulation and fault-mode demo
├── fft_real.py      # Live serial accelerometer monitoring
├── README.md        # Project documentation
├── requirements.txt # Python dependencies
└── .gitignore       # Local files to exclude from Git
```

## Features

### Simulation mode

- Generates a synthetic healthy-motor signal with 25 Hz, 50 Hz, and 75 Hz components plus random noise.
- Adds synthetic outer-race impact events and BPFO harmonics in fault mode.
- Applies a Hann window and computes a real FFT.
- Calculates nominal BPFO, BPFI, and BSF frequencies from configured bearing geometry and shaft speed.
- Displays time-domain and frequency-domain plots.
- Provides a **Toggle Fault Mode** button to switch between the synthetic normal and fault scenarios.
- Shows threshold-based alert levels based on the amplitude at the FFT bin nearest BPFO relative to a synthetic baseline.

### Live sensor mode

- Reads `ax,ay,az` values from an Arduino-compatible board over USB serial.
- Calculates a scalar vibration magnitude from the three acceleration axes.
- Maintains a rolling buffer of up to 1,024 samples.
- Estimates an effective sample rate from incoming sample timestamps.
- Applies mean removal, a Hann window, and an FFT.
- Displays a live vibration plot and FFT spectrum.
- Provides a **Capture Baseline** button for recording the current BPFO-bin amplitude during a known healthy operating condition.

## Signal-processing workflow

Both scripts calculate the characteristic bearing frequencies from shaft speed and nominal 6205 bearing geometry:

- **BPFO** — Ball Pass Frequency Outer race
- **BPFI** — Ball Pass Frequency Inner race
- **BSF** — Ball Spin Frequency

The implementation uses the following nominal parameters, which should be verified against the actual bearing:

| Parameter | Value in code |
| --- | ---: |
| Number of rolling elements | 9 |
| Ball diameter | 7.94 mm |
| Pitch diameter | 38.5 mm |
| Contact angle | 0° |
| Assumed shaft speed | 1,500 RPM |

The characteristic frequencies change with shaft speed and bearing geometry. The live script currently uses a manually configured RPM constant rather than a measured tachometer input.

### Alert thresholds

Both applications use these prototype thresholds:

| BPFO amplitude / baseline amplitude | Displayed state |
| --- | --- |
| Below 1.2× | Level 0 — Normal |
| 1.2× to below 1.4× | Level 1 — Watch |
| 1.4× to below 1.8× | Level 2 — Warning |
| 1.8× or higher | Level 3 — Critical |

These thresholds are hard-coded demonstration values, not validated maintenance limits. In simulation mode, the injected fault signature is designed to make the demonstration visible. In live mode, an increase at a single FFT bin does not by itself establish that a bearing defect is present.

## Requirements

- Python 3
- NumPy
- SciPy
- Matplotlib
- pySerial (live sensor mode only)

## Installation

Clone the repository:

```bash
git clone https://github.com/<your-username>/<your-repository>.git
cd <your-repository>
```

Create a virtual environment (recommended).

**Windows PowerShell**
```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
```

**Linux / macOS**
```bash
python3 -m venv .venv
source .venv/bin/activate
```

Install the dependencies:

```bash
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Suggested `requirements.txt`:

```text
numpy
scipy
matplotlib
pyserial
```

## Run simulation mode

```bash
python fft_sim.py
```

A Matplotlib window displays the generated vibration signal and FFT spectrum.

1. Start in **NORMAL MODE**.
2. Click **Toggle Fault Mode** to switch to the synthetic outer-race fault signal.
3. Observe the time-domain signal, FFT spectrum, BPFO marker, and displayed alert ratio.
4. Toggle again to return to the normal signal.

**Interpretation:** this simulation is a controlled demonstration. The fault signal is generated in code using periodic impacts and BPFO-related sinusoidal components. It is not a recording of a physically damaged bearing and should not be used as evidence of real-world classification accuracy.

## Run live sensor mode

```bash
python fft_real.py
```

### Configure the serial port

Open `fft_real.py` and update the configuration to match your device:

```python
SERIAL_PORT = "COM9"
BAUD_RATE = 115200

N = 1024
RPM = 1500
```

Examples of serial-port names include `COM9` on Windows and `/dev/ttyUSB0` on Linux. The actual port depends on your operating system and hardware.

### Expected serial data

The live script expects one line per sample with three comma-separated floating-point values:

```text
0.02,-0.01,0.98
0.03,-0.02,0.99
0.01,-0.02,1.00
```

The example in the script assumes acceleration values expressed in `g`; ensure the firmware output uses consistent units and scaling. Close any serial monitor or other program that is using the same port.

### Capture a baseline

1. Start the motor in a known healthy condition.
2. Wait for the live buffer to collect enough samples.
3. Click **Capture Baseline**.
4. Observe subsequent BPFO-bin amplitude ratios and alert levels.

The baseline is kept in memory and is not persisted between runs.

## Sampling considerations

The simulation uses a nominal sample rate of **4,000 Hz** and a **1,024-sample** FFT window. Its nominal frequency-bin spacing is:

Δf = Fs / N

The live sensor script estimates the effective sample rate from serial timestamps. USB serial throughput, text parsing, buffering, and scheduling can introduce timing jitter. The Nyquist frequency is half the effective sample rate, so a target above that limit cannot be reliably represented. Timestamp averaging alone does not correct irregular sampling.

## Current limitations

- The simulation uses synthetic signals and intentionally adds BPFO-related impacts and harmonics in fault mode.
- The live implementation uses a manually configured RPM value; it does not currently measure shaft speed.
- The alert logic compares the amplitude at the nearest BPFO FFT bin rather than a validated frequency-band energy feature or trained classifier.
- The live implementation calculates vector magnitude before FFT processing; this nonlinear transformation can affect the spectrum.
- The live implementation does not currently perform calibrated sensor-health checks, independent fault confirmation, or sensor/wiring fault attribution.
- The baseline is not saved between runs.
- No remaining-useful-life estimate or validated prediction of time-to-failure is implemented.
- Results depend on sampling quality, sensor mounting, operating speed, load, noise, and bearing parameters.

## Roadmap

- Compare simulation output with reproducible, labelled vibration datasets.
- Add CSV logging for signals, FFT features, RPM, baseline, and alerts.
- Measure shaft speed using a tachometer or Hall sensor.
- Evaluate frequency-band features and harmonics against verified healthy and faulty bearing data.
- Test threshold stability across speeds, loads, sensors, and mounting positions.
- Add sampling-quality checks and explicit uncertain / sensor-fault states.
- Document experimental setup, measured results, false alarms, missed detections, and limitations.
- Explore deployment of validated signal-processing components to an embedded target.

## Contributing

Bug reports, reproducible tests, and evidence-backed improvements are welcome. When sharing results, document the bearing model and geometry, RPM, sensor and mounting, sampling configuration, load, and how the fault condition was independently verified.

## Safety and intended use

This repository is an experimental engineering prototype for learning and evaluation. It is not a certified safety system and must not be used as the sole basis for maintenance, shutdown, or safety decisions.

## License

No license has been selected yet. Until a license is added, others should not assume they have permission to reuse, modify, or distribute this project. Choose an appropriate open-source license before inviting public reuse.
