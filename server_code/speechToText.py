import vosk
import wave
import json
import sys


if len(sys.argv) < 2:
    print("Usage: python speechToText.py <audio_file>")
    sys.exit(1)

# Get the input audio file from command-line arguments
audio_file = sys.argv[1]
# Load the Vosk model
model_path = "vosk_model/vosk-model-small-en-us-0.15"
model = vosk.Model(model_path)

# Open the audio file

wf = wave.open(audio_file, "rb")

# Initialize the recognizer
rec = vosk.KaldiRecognizer(model, wf.getframerate())

# Define the output transcription file
output_file = "transcription.txt"

# Process audio and get transcription
transcription = ""

with open(output_file, "w") as f:
    while True:
        data = wf.readframes(4000)  # Read 4000 frames at a time
        if len(data) == 0:
            break
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            text = result["text"]
            transcription += text + " "
            f.write(text + "\n")  # Write to file
            print(text)  # Print for capturing in `voice_control.py`

    # Get final result
    final_result = json.loads(rec.FinalResult())
    final_text = final_result["text"]
    transcription += final_text
    f.write(final_text + "\n")  # Write final result to file
    print(final_text)  # Print final text for piping

# Print full transcription for `voice_control.py`
print(transcription.strip())

sys.exit(0)  # Ensure script exits successfully
