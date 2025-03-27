import numpy as np
import sounddevice as sd
from piper.voice import PiperVoice

class PiperTTS:
    def __init__(self, model_path):
        self.voice = PiperVoice.load(model_path)

    def speak(self, text):
        stream = sd.OutputStream(samplerate=self.voice.config.sample_rate, channels=1, dtype='int16')
        stream.start()
        for audio_bytes in self.voice.synthesize_stream_raw(text):
            stream.write(np.frombuffer(audio_bytes, dtype=np.int16))
        stream.stop()
        stream.close()
