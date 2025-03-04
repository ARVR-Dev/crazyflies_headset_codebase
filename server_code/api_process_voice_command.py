import subprocess
import openai
import os
import sys
from pydub import AudioSegment

# Set your OpenAI API key
OPENAI_API_KEY = "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"

def convert_audio_to_wav(input_audio: str, output_wav: str):
    """
    Converts an M4A file to WAV using FFmpeg.
    Ensures mono (1 channel) and 16kHz sample rate for better transcription accuracy.
    """
    audio = AudioSegment.from_file(input_audio)
    audio = audio.set_channels(1).set_frame_rate(16000)  # Mono, 16kHz for speech recognition
    audio.export(output_wav, format="wav")
    print(f"Converted {input_audio} to {output_wav}")

def transcribe_audio_online(audio_file: str) -> str:
    """
    Uses OpenAI Whisper API to transcribe the given audio file.

    :param audio_file: Path to the WAV audio file.
    :return: Transcribed text.
    """
    try:
        openai.api_key = OPENAI_API_KEY  # Set API key
        with open(audio_file, "rb") as audio:
            response = openai.audio.transcriptions.create(
                model="whisper-1",
                file=audio,
                response_format="json"
            )
        return response.text.strip()
    except Exception as e:
        print(f"Error in transcription: {str(e)}")
        return ""

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
    Process an audio command: convert, transcribe, and execute the command.

    :param audio_file: Path to the audio file.
    """
    converted_wav = "converted_audio.wav"
    convert_audio_to_wav(audio_file, converted_wav)

    transcribed_text = transcribe_audio_online(converted_wav)
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
