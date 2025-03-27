import openai
import os
import sys

# Set your OpenAI API key
OPENAI_API_KEY = os.getenv("OPEN_AI_KEY")

def transcribe_audio_online(audio_file: str) -> str:
    """
    Uses OpenAI Whisper API to transcribe the given audio file directly.

    :param audio_file: Path to the audio file.
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

def process_audio_transcription(audio_file: str):
    """
    Directly transcribes the input audio file without conversion.

    :param audio_file: Path to the audio file.
    """
    transcribed_text = transcribe_audio_online(audio_file)
    print(f"Transcribed Text: {transcribed_text}")

# Example usage
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 process_audio_transcription.py <audio_file>")
        sys.exit(1)

    # Get the input audio file from command-line arguments
    audio_file = sys.argv[1]
    process_audio_transcription(audio_file)
