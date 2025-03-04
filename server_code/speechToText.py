import vosk
import wave
import json

# Load the Vosk model from the extracted directory
model_path = "model/vosk-model-small-en-us-0.15"
model = vosk.Model(model_path)

# Open the audio file
audio_file = "output.wav"  # Change to your actual file
wf = wave.open(audio_file, "rb")

# Initialize the recognizer
rec = vosk.KaldiRecognizer(model, wf.getframerate())

# Open a text file to store the results
output_file = "transcription.txt"

with open(output_file, "w") as f:
    while True:
        data = wf.readframes(4000)  # Read 4000 frames at a time
        if len(data) == 0:
            break
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            f.write(result["text"] + "\n")  # Write to file
            print("Recognized Text:", result["text"])

    # Write the final result
    final_result = json.loads(rec.FinalResult())
    f.write(final_result["text"] + "\n")
    print("Final Transcript:", final_result["text"])

print(f"Transcription saved to {output_file}")
