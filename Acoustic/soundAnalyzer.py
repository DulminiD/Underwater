import wave
import numpy as np
import matplotlib.pyplot as plt

# /home/mcpslab/Downloads/beep-01a
wav_obj = wave.open('recording1.wav', 'rb')

sample_freq = wav_obj.getframerate()
print(sample_freq)

n_samples = wav_obj.getnframes()
print(n_samples)

t_audio = n_samples/sample_freq
print(t_audio)

n_channels = wav_obj.getnchannels()
print(n_channels)

signal_wave = wav_obj.readframes(n_samples)

signal_array = np.frombuffer(signal_wave, dtype=np.int16)
print(len(signal_array))

l_channel = signal_array[0::2]
r_channel = signal_array[1::2]

times = np.linspace(0, n_samples/sample_freq, num=n_samples)

print(len(times))
print(len(signal_array))
plt.figure(figsize=(15, 5))
plt.plot(times, l_channel)
plt.title('Left Channel')
plt.ylabel('Signal Value')
plt.xlabel('Time (s)')
plt.xlim(0, t_audio)
plt.show()
