
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from tqdm import tqdm


# ==========================================================
# PATH CONFIGURATION
# ==========================================================

BASE_DIR = r"G:\FI_ThreadAnalysis\20260520_140708"
IMAGE_DIR = os.path.join(BASE_DIR, "images")

RESULTS_DIR = os.path.join(BASE_DIR, "Results")
os.makedirs(RESULTS_DIR, exist_ok=True)

OUTPUT_VIDEO = os.path.join(RESULTS_DIR, "airsim_output.mp4")


# ==========================================================
# VIDEO CREATION
# ==========================================================

def create_video_from_images(image_dir, output_path, fps=60):
    images = [img for img in os.listdir(image_dir) if img.endswith(".png")]
    images.sort(key=lambda x: int(os.path.splitext(x)[0]))

    first_image_path = os.path.join(image_dir, images[0])
    frame = cv2.imread(first_image_path)
    height, width, _ = frame.shape

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    for image in tqdm(images, desc="Creating Video"):
        frame = cv2.imread(os.path.join(image_dir, image))
        video.write(frame)

    video.release()
    print(f"Video saved at: {output_path}")


# ==========================================================
# SAFE FREQUENCY COMPUTATION
# ==========================================================

def compute_frequency_from_txt(file_path):
    df = pd.read_csv(
        file_path,
        sep=r"\s+",
        comment="#",
        header=None,
        engine="python"
    )

    timestamps_ns = pd.to_numeric(df[0], errors="coerce").dropna().values

    if len(timestamps_ns) < 2:
        return np.array([])

    timestamps_s = timestamps_ns * 1e-9
    dt = np.diff(timestamps_s)

    # Remove zero and negative dt
    dt = dt[dt > 0]

    if len(dt) == 0:
        return np.array([])

    freq = 1.0 / dt
    return freq


# ==========================================================
# SAFE PLOT SAVING
# ==========================================================

def save_frequency_plot(freq_array, title, output_path):
    if len(freq_array) == 0:
        print(f"⚠ Skipping {title} (no valid data)")
        return

    plt.figure(figsize=(10, 5))
    plt.plot(freq_array)
    plt.axhline(np.mean(freq_array), linestyle='--')
    plt.xlabel("Sample Index")
    plt.ylabel("Frequency (Hz)")
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()

    print(f"Plot saved: {output_path}")
    print(f"   Avg: {np.mean(freq_array):.2f} Hz | "
          f"Min: {np.min(freq_array):.2f} | "
          f"Max: {np.max(freq_array):.2f}")


# ==========================================================
# PROCESS ALL SENSOR FILES
# ==========================================================

def process_all_sensors(base_dir):
    sensor_files = [
        "times.txt",
        "imu.txt",
        "gps.txt",
        "barometer.txt",
        "gt_imu.txt",
        "magnetometer.txt"
    ]

    all_freq = {}

    for sensor in sensor_files:
        file_path = os.path.join(base_dir, sensor)

        if not os.path.exists(file_path):
            continue

        freq = compute_frequency_from_txt(file_path)
        all_freq[sensor] = freq

        output_plot = os.path.join(
            RESULTS_DIR,
            os.path.splitext(sensor)[0] + "_frequency.png"
        )

        save_frequency_plot(freq, f"{sensor} Frequency", output_plot)

    return all_freq


# ==========================================================
# COMPARISON PLOT
# ==========================================================

def save_comparison_plot(freq_dict, output_path):
    plt.figure(figsize=(12, 6))

    valid_data = False

    for name, freq in freq_dict.items():
        if len(freq) > 0:
            plt.plot(freq, label=name)
            valid_data = True

    if not valid_data:
        print("⚠ No valid frequency data to compare.")
        return

    plt.xlabel("Sample Index")
    plt.ylabel("Frequency (Hz)")
    plt.title("Sensor Frequency Comparison")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()

    print(f"Comparison plot saved: {output_path}")


# ==========================================================
# MAIN
# ==========================================================

if __name__ == "__main__":

    create_video_from_images(IMAGE_DIR, OUTPUT_VIDEO, fps=45)

    frequencies = process_all_sensors(BASE_DIR) # processing sensors

    comparison_path = os.path.join(RESULTS_DIR, "all_sensors_frequency.png")
    save_comparison_plot(frequencies, comparison_path)

    print("\n*** All processing completed successfully. ***")