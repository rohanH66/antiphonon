import serial
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats   # <-- added for statistical testing

# ---------------- CONFIG ----------------
PORT = "COM22"
BAUD = 115200
OUTFILE = "anc_log.csv"
TARGET_FREQ = None  # Will auto-detect from serial log
BANDWIDTH = 200     # ±Hz window for stats
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
                    # save only structured calibration data to CSV
                    if any(line.startswith(tag) for tag in ["BEFORE", "AFTER", "SWEEP", "RESULT"]):
                        f.write(line + "\n")
                        f.flush()
                        lines.append(line)
                    # also save frequency info for auto-detection
                    elif "Frequency:" in line:
                        f.write(line + "\n")
                        f.flush()
        except KeyboardInterrupt:
            print("\nLogging stopped.")
    return lines


def parse_and_plot():
    global TARGET_FREQ

    # Try to extract detected frequency from the serial log text
    try:
        with open(OUTFILE, "r") as f:
            for line in f:
                if "Frequency:" in line:
                    try:
                        TARGET_FREQ = float(line.strip().split(":")[1].split()[0])
                        print(f"\nAuto-detected target frequency: {TARGET_FREQ:.1f} Hz\n")
                        break
                    except:
                        pass
    except FileNotFoundError:
        print("No log file found. Run calibration first.")
        return

    # Fallback if not found
    if TARGET_FREQ is None:
        TARGET_FREQ = 7000.0
        print(f"No frequency detected in log. Defaulting to {TARGET_FREQ:.1f} Hz.\n")

    df = pd.read_csv(OUTFILE, header=None, names=["Tag", "X", "Y"], engine="python")

    before = df[df.Tag == "BEFORE"].copy()
    after = df[df.Tag == "AFTER"].copy()

    if before.empty or after.empty:
        print("No BEFORE/AFTER data found in file.")
        return

    before["X"] = before["X"].astype(float)
    before["Y"] = before["Y"].astype(float)
    after["X"] = after["X"].astype(float)
    after["Y"] = after["Y"].astype(float)

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
    before_db = 20 * np.log10((before["Y"] + 1e-9) / ref)
    after_db = 20 * np.log10((after["Y"] + 1e-9) / ref)

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
    idx_after = (after["X"] - TARGET_FREQ).abs().idxmin()
    amp_before = before.loc[idx_before, "Y"]
    amp_after = after.loc[idx_after, "Y"]
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
            amp_before = results[results["X"] == "Before"]["Y"].astype(float).values[0]
            amp_after = results[results["X"] == "After"]["Y"].astype(float).values[0]
            drop_db = results[results["X"] == "Drop_dB"]["Y"].astype(float).values[0]
            print(f"\n=== ANC RESULTS ===")
            print(f"Before amplitude: {amp_before:.2f}")
            print(f"After amplitude : {amp_after:.2f}")
            print(f"Improvement     : {drop_db:.2f} dB")
        except Exception as e:
            pass

    # ---------- STATISTICAL SIGNIFICANCE TEST ----------
    try:
        merged = before.merge(after, on="X", suffixes=("_before", "_after"))

        # Focus only on ±BANDWIDTH Hz around target
        window = (merged["X"] > TARGET_FREQ - BANDWIDTH) & (merged["X"] < TARGET_FREQ + BANDWIDTH)
        subset = merged[window]

        if subset.empty:
            print(f"\nNo data found near {TARGET_FREQ:.1f} Hz.")
        else:
            t_stat, p_val = stats.ttest_rel(subset["Y_before"], subset["Y_after"])
            mean_drop = 20 * np.log10((subset["Y_after"] + 1e-9) / (subset["Y_before"] + 1e-9)).mean()

            print(f"\n=== STATISTICAL ANALYSIS (±{BANDWIDTH} Hz around {TARGET_FREQ:.1f} Hz) ===")
            print(f"Samples analyzed   : {len(subset)}")
            print(f"Mean dB change     : {mean_drop:.2f} dB")
            print(f"T-statistic        : {t_stat:.3f}")
            print(f"P-value            : {p_val:.5f}")

            if p_val < 0.05:
                print("Result: Statistically significant attenuation (p < 0.05)")
            else:
                print("Result: Not statistically significant (p ≥ 0.05)")

    except Exception as e:
        print("\n[Warning] Statistical test failed:", e)

    plt.show()


if __name__ == "__main__":
    log_serial()
    parse_and_plot()
