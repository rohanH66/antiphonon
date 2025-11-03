import serial
import pandas as pd
import matplotlib.pyplot as plt

# ---------------- CONFIG ----------------
PORT = "COM22"
BAUD = 115200
OUTFILE = "anc_log.csv"
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

    before = df[df.Tag=="BEFORE"]
    after  = df[df.Tag=="AFTER"]

    plt.figure()
    plt.plot(before["X"], before["Y"], label="Before")
    plt.plot(after["X"], after["Y"], label="After")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude")
    plt.title("FFT Spectrum Before vs After ANC")
    plt.legend()
    plt.grid()

    sweep = df[df.Tag=="SWEEP"]
    if not sweep.empty:
        plt.figure()
        plt.plot(sweep["X"], sweep["Y"], marker="o")
        plt.xlabel("Phase (rad)")
        plt.ylabel("Amplitude at drill freq")
        plt.title("Phase Sweep Curve")
        plt.grid()

    results = df[df.Tag=="RESULT"]
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