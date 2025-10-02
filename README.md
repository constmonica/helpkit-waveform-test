
# Arduino HelpKit Waveform Generator Test

Python script using pytest and libm2k to verify the waveform generator on Arduino HelpKit works properly.

## Requirements

- ADALM2000 connected at ``ip:192.168.2.1``
- Python 3.7+
- Arduino HelpKit waveform generator

## Installation

### 1. Install libm2k

Download and install libm2k from the [official releases](https://github.com/analogdevicesinc/libm2k/releases):
- Windows: Download the appropriate `.exe` installer
- Install both the library and Python bindings

### 2. Install Python dependencies
```bash
pip install -r requirements.txt
```
Note: The requirements.txt does not include libm2k as it must be installed separately from the official installer.
### Usage
Run the test suite:
```bash
python read_signals.py
```
The script will prompt you to connect three different waveforms:

Sine wave

Square wave

Triangle wave


Press ENTER after connecting each waveform. The script will analyze the signal and report if it detected the correct waveform type, frequency, and amplitude.
