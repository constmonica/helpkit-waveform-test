import libm2k
import sys
import numpy as np
from scipy.fft import fft, fftfreq
import pytest

uri = "ip:192.168.2.1"

ctx = None
ain = None
sample_rate = 1000000
buffer_size = 4096

def setup_module():
    global ctx, ain

    print("\nArduino Helpkit Waveform Detection Tests")
    print("=" * 70)
    
    ctx = libm2k.m2kOpen(uri)
    if ctx is None:
        pytest.fail(f"Can't connect to device at {uri}")
    
    print(f"Connected to device at {uri}")
    
    ain = ctx.getAnalogIn()
    ain.reset()
    ain.setSampleRate(sample_rate)
    ain.enableChannel(0, True)
    ain.enableChannel(1, False)
    ain.setRange(0, libm2k.PLUS_MINUS_25V)
    ain.setKernelBuffersCount(1)
    
    print("Oscilloscope ready\n")

def teardown_module():
    global ctx
    if ctx is not None:
        libm2k.contextClose(ctx)
        print("\nDisconnected")

def analyze_signal(data, fs):
    dc = np.mean(data)
    data_ac = data - dc
    vpp = np.max(data) - np.min(data)
    vpk = vpp / 2
    vrms = np.sqrt(np.mean(data_ac**2))
    
    # check if it's just DC
    if np.std(data) < 0.01:
        return "DC", 0, vpp, dc, 0, {"voltage": dc, "std_dev": np.std(data)}
    
    # FFT analysis
    n = len(data_ac)
    fft_vals = fft(data_ac)
    freqs = fftfreq(n, 1/fs)
    
    pos_mask = freqs > 0
    freqs_pos = freqs[pos_mask]
    fft_pos = np.abs(fft_vals[pos_mask])
    
    peak_idx = np.argmax(fft_pos)
    f0 = freqs_pos[peak_idx]
    mag0 = fft_pos[peak_idx]
    
    if f0 < 1:
        return "DC/NOISE", f0, vpp, dc, 0, {"voltage": dc, "std_dev": np.std(data)}
    
    # get harmonics
    harmonics = []
    for h in range(1, 11):
        fh = f0 * h
        window = f0 * 0.1
        mask = (freqs_pos >= fh - window) & (freqs_pos <= fh + window)
        if np.any(mask):
            h_mag = np.max(fft_pos[mask])
            harmonics.append(h_mag / mag0 if mag0 > 0 else 0)
        else:
            harmonics.append(0)
    
    # THD calculation
    thd = np.sqrt(sum([h**2 for h in harmonics[1:]])) * 100
    
    # form factor and crest factor
    ff = vrms / np.mean(np.abs(data_ac)) if np.mean(np.abs(data_ac)) > 0 else 0
    cf = vpk / vrms if vrms > 0 else 0
    
    # zero crossings
    zc = np.where(np.diff(np.sign(data_ac)))[0]
    if len(zc) > 2:
        periods = np.diff(zc[::2])
        if len(periods) > 0:
            f_zc = fs / (np.mean(periods) * 2)
        else:
            f_zc = f0
    else:
        f_zc = f0
    
    duty = (np.sum(data_ac > 0) / len(data_ac)) * 100 if len(zc) > 1 else 0
    
    # classify waveform
    wtype = "UNKNOWN"
    
    # sine: clean signal, low harmonics
    if thd < 15 and 1.05 < ff < 1.15 and 1.35 < cf < 1.50:
        wtype = "SINE"
    
    # square: strong odd harmonics
    elif len(harmonics) >= 3:
        h_ratio = harmonics[2] / harmonics[0] if harmonics[0] > 0 else 0
        if (0.8 < ff < 1.05 and thd > 30 and harmonics[2] > 0.15 and 0.2 < h_ratio < 0.5):
            wtype = "SQUARE"
    
    # triangle: weaker harmonics than square
    if len(harmonics) >= 3:
        h3_ratio = harmonics[2] / harmonics[0] if harmonics[0] > 0 else 0
        if (1.10 < ff < 1.20 and 1.50 < cf < 1.80 and 10 < thd < 25 and 0.05 < h3_ratio < 0.15):
            wtype = "TRIANGLE"
    
    # fallback classification
    if wtype == "UNKNOWN" and thd > 5:
        if harmonics[2] > 0.2:
            wtype = "SQUARE"
        elif 15 < thd < 30:
            wtype = "TRIANGLE"
    
    info = {
        "thd": thd,
        "form_factor": ff,
        "crest_factor": cf,
        "rms": vrms,
        "harmonics": harmonics[:5],
        "detected_freq": f_zc
    }
    
    return wtype, f0, vpp, dc, duty, info

def get_reading():
    global ain
    
    data = ain.getSamples(buffer_size)
    samples = np.array(data[0])
    
    wtype, freq, vpp, dc, duty, info = analyze_signal(samples, sample_rate)
    
    print(f"\nWaveform: {wtype}")
    print(f"Frequency: {freq:.2f} Hz")
    print(f"Amplitude: {vpp/2:.4f} V")
    
    return wtype, freq, vpp

def test_sine_wave():
    print("\n" + "=" * 70)
    print("Test 1: Sine Wave")
    print("=" * 70)
    print("Select Sine as waveform type using the encoder.")
    input("Press ENTER when ready...")
    
    wtype, freq, amp = get_reading()
    
    assert wtype == "SINE", f"Expected SINE, got {wtype}"
    print("\nPASSED")

def test_square_wave():
    print("\n" + "=" * 70)
    print("Test 2: Square Wave")
    print("=" * 70)
    print("Select Sqw as waveform type using the encoder.")
    input("Press ENTER when ready...")
    
    wtype, freq, amp = get_reading()
    
    assert wtype == "SQUARE", f"Expected SQUARE, got {wtype}"
    print("\nPASSED")

def test_triangle_wave():
    print("\n" + "=" * 70)
    print("Test 3: Triangle Wave")
    print("=" * 70)
    print("Select Tri as waveform type using the encoder.")
    input("Press ENTER when ready...")
    
    wtype, freq, amp = get_reading()
    
    assert wtype == "TRIANGLE", f"Expected TRIANGLE, got {wtype}"
    print("\nPASSED")

if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])