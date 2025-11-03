import serial
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ---------------- CONFIG ----------------
PORT = "COM22"
BAUD = 115200
OUTFILE = "anc_log.csv"
TARGET_FREQ = 440  # Hz — used for attenuation readout
# ----------------------------------------

def log_serial():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"{PORT} port, {BAUD}")
    print(f"Saving to {OUTFILE}")
    print("Press Ctrl+C to stop after calibration.\n")

    lines = []
    with open(OUTFILE, "w") as f:
        try:
            while True:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    print(line)
                    if any(line.startswith(tag) for tag in ["BEFORE","AFTER","SWEEP","RESULT"]):
                        f.write(line + "\n")
                        f.flush()
                        lines.append(line)
        except KeyboardInterrupt:
            print("\nLogging stopped.")
    return lines


def parse_and_plot():
    df = pd.read_csv(OUTFILE, header=None, names=["Tag","X","Y"], engine="python")

    before = df[df.Tag == "BEFORE"].copy()
    after  = df[df.Tag == "AFTER"].copy()

    if before.empty or after.empty:
        print("No BEFORE/AFTER data found in file.")
        return

    before["X"] = before["X"].astype(float)
    before["Y"] = before["Y"].astype(float)
    after["X"]  = after["X"].astype(float)
    after["Y"]  = after["Y"].astype(float)

    # ---------- LINEAR AMPLITUDE PLOT ----------
    plt.figure()
    plt.plot(before["X"], before["Y"], label="Before")
    plt.plot(after["X"], after["Y"], label="After")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude (Linear)")
    plt.title("FFT Spectrum Before vs After ANC (Linear Scale)")
    plt.legend()
    plt.grid(True, alpha=0.4)

    # ---------- DECIBEL (dB) PLOT ----------
    ref = before["Y"].max()
    before_db = 20 * np.log10(before["Y"] / ref)
    after_db  = 20 * np.log10(after["Y"] / ref)

    plt.figure()
    plt.plot(before["X"], before_db, label="Before (dB)")
    plt.plot(after["X"], after_db, label="After (dB)")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude (dB re Before peak)")
    plt.title("FFT Spectrum Before vs After ANC (Shared Reference dB Scale)")
    plt.legend()
    plt.grid(True, alpha=0.4)

    # ---------- COMPUTE DROP AT TARGET FREQ ----------
    idx_before = (before["X"] - TARGET_FREQ).abs().idxmin()
    idx_after  = (after["X"] - TARGET_FREQ).abs().idxmin()
    amp_before = before.loc[idx_before, "Y"]
    amp_after  = after.loc[idx_after, "Y"]
    drop_db = 20 * np.log10((amp_after + 1e-9) / (amp_before + 1e-9))
    print(f"\n=== ATTENUATION SUMMARY ===")
    print(f"Target frequency: {TARGET_FREQ:.1f} Hz")
    print(f"Before amplitude: {amp_before:.3e}")
    print(f"After amplitude : {amp_after:.3e}")
    print(f"Drop: {drop_db:.2f} dB")

    # ---------- PHASE SWEEP ----------
    sweep = df[df.Tag == "SWEEP"]
    if not sweep.empty:
        plt.figure()
        plt.plot(sweep["X"], sweep["Y"], marker="o")
        plt.xlabel("Phase (rad)")
        plt.ylabel("Amplitude at drill freq")
        plt.title("Phase Sweep Curve")
        plt.grid(True, alpha=0.4)

    # ---------- RESULTS ----------
    results = df[df.Tag == "RESULT"]
    if not results.empty:
        try:
            amp_before = results[results["X"]=="Before"]["Y"].astype(float).values[0]
            amp_after  = results[results["X"]=="After"]["Y"].astype(float).values[0]
            drop_db    = results[results["X"]=="Drop_dB"]["Y"].astype(float).values[0]
            print(f"\n=== ANC RESULTS ===")
            print(f"Before amplitude: {amp_before:.2f}")
            print(f"After amplitude : {amp_after:.2f}")
            print(f"Improvement     : {drop_db:.2f} dB")
        except:
            pass

    plt.show()


if __name__ == "__main__":
    log_serial()
    parse_and_plot()