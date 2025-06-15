import subprocess
import os

def convert_webm_to_mp4(input_path, output_path=None):
    if not input_path.endswith(".webm"):
        raise ValueError("Input file must be a .webm file")

    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"File not found: {input_path}")

    if output_path is None:
        output_path = input_path.rsplit(".", 1)[0] + ".mp4"

    print(f"Converting {input_path} to {output_path}...")

    command = [
        "ffmpeg",
        "-i", input_path,
        "-c:v", "libx264",
        "-c:a", "aac",
        "-strict", "experimental",
        output_path
    ]

    try:
        subprocess.run(command, check=True)
        print("Conversion successful.")
    except subprocess.CalledProcessError as e:
        print("FFmpeg failed:", e)

# Example usage
if __name__ == "__main__":
    convert_webm_to_mp4("ros-llama.webm")
