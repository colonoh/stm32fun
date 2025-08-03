import os
import subprocess
import struct

INPUT_FOLDER = "source"
OUTPUT_BIN = "audio_data.bin"
OUTPUT_HEADER = "audio_data.h"
SAMPLE_RATE = 16000
CHANNELS = 2

def convert_mp3_to_raw(mp3_path, raw_path):
    command = [
        "ffmpeg",
        "-y",  # Overwrite
        "-i", mp3_path,
        "-ac", str(CHANNELS),
        "-ar", str(SAMPLE_RATE),
        "-f", "s16le",  # signed 16-bit little endian PCM
        raw_path
    ]
    subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def to_c_array(data: bytes, name: str) -> str:
    lines = [f"const uint8_t {name}[] = {{"]
    for i in range(0, len(data), 12):
        chunk = data[i:i+12]
        line = ", ".join(f"0x{b:02X}" for b in chunk)
        lines.append(f"    {line},")
    lines.append("};")
    return "\n".join(lines)

def main():
    os.makedirs("temp", exist_ok=True)
    clips = []
    combined_data = bytearray()

    print(f"Processing MP3s in '{INPUT_FOLDER}'...")
    for filename in sorted(os.listdir(INPUT_FOLDER)):
        if not filename.lower().endswith(".mp3"):
            continue

        mp3_path = os.path.join(INPUT_FOLDER, filename)
        raw_path = os.path.join("temp", filename + ".raw")

        convert_mp3_to_raw(mp3_path, raw_path)
        with open(raw_path, "rb") as f:
            raw_data = f.read()

        start = len(combined_data)
        length = len(raw_data)
        clips.append((filename, start, length))
        combined_data.extend(raw_data)

        print(f"✔ {filename} → {length} bytes at offset {start}")

    # Write binary file
    with open(OUTPUT_BIN, "wb") as f:
        f.write(combined_data)

    # Write header file
    with open(OUTPUT_HEADER, "w") as f:
        f.write("#pragma once\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write(to_c_array(combined_data, "audio_clip"))
        f.write(f"\n\n#define AUDIO_CLIP_COUNT {len(clips)}\n")
        f.write("typedef struct {\n    uint32_t start;\n    uint32_t length;\n} AudioClip;\n\n")
        f.write("const AudioClip audio_index[AUDIO_CLIP_COUNT] = {\n")
        for name, start, length in clips:
            f.write(f"    {{ {start}, {length} }}, // {name}\n")
        f.write("};\n")

    print(f"\n✅ Done. Wrote {OUTPUT_BIN} and {OUTPUT_HEADER}")

if __name__ == "__main__":
    main()
