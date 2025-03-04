import subprocess
import json
import wave
import vosk
import sys

def transcribe_audio(audio_file: str) -> str:
    """
    Transcribe the given audio file using the Vosk speech recognition model.

    :param audio_file: Path to the audio file.
    :return: Transcribed text.
    """
    model_path = "/home/cpsl/Documents/crazyflies_headset_codebase/server_code/vosk_model/vosk-model-small-en-us-0.15"
    model = vosk.Model(model_path)

    wf = wave.open(audio_file, "rb")
    rec = vosk.KaldiRecognizer(model, wf.getframerate())

    transcription = ""
    while True:
        data = wf.readframes(4000)
        if len(data) == 0:
            break
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            transcription += result["text"] + " "

    final_result = json.loads(rec.FinalResult())
    transcription += final_result["text"]

    return transcription.strip()

def execute_voice_command(command: str) -> str:
    """
    Execute the given voice command using `voice_control.py`.

    :param command: Transcribed command.
    :return: Output of the execution.
    """
    try:
        result = subprocess.run(
            ["python3", "voice_control.py", command],
            capture_output=True,
            text=True
        )
        return result.stdout
    except Exception as e:
        return f"Error executing voice control: {str(e)}"

def process_audio_command(audio_file: str):
    """
    Process an audio command: transcribe it and execute the corresponding command.

    :param audio_file: Path to the audio file.
    """
    transcribed_text = transcribe_audio(audio_file)
    print(f"Transcribed Text: {transcribed_text}")

    if transcribed_text:
        response = execute_voice_command(transcribed_text)
        print(f"Voice Control Response: {response}")
    else:
        print("No valid transcription found.")

# Example usage
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 process_audio_command.py <audio_file>")
        sys.exit(1)

    # Get the input audio file from command-line arguments
    audio_file = sys.argv[1]
    process_audio_command(audio_file)
