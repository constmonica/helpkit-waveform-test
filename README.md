
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

### 3. Install Arduino Demo Code

1. Download and install [Arduino IDE](https://www.arduino.cc/en/software/)
2. Connect Arduino HelpKit via USB 
3. Select board and port in Tools menu
4. Open *HelpKit_AD5443_v1_9_3_DCOff_Calibrate_Flash.ino* and add new demos as needed
5. Click Upload to flash the code

## Usage
Run the test suite:
```bash
python waveform_test.py
```
The script will prompt you to test a sine wave.

Press ENTER after starting the basic demo on the Arduino. The script will analyze the signal and report if it detected the correct waveform type, frequency, and amplitude.
