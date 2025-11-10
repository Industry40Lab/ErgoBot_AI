from TTS.api import TTS
import soundfile as sf
import sounddevice as sd

# Step 1: Load a pretrained TTS model
tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", progress_bar=False, gpu=False)

# Step 2: Synthesize speech to an array
text = "Hello! This is a test of the Coqui TTS engine speaking through your system speakers."
wav = tts.tts(text)

# Step 3: Play the audio using your system speakers
# Get the sampling rate of the model (usually 22050 Hz)
sample_rate = tts.synthesizer.output_sample_rate
sd.play(wav, samplerate=sample_rate)
sd.wait()  # Wait until audio is done playing
