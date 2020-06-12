import numpy as np
import tensorflow as tf
from PIL import Image
import pyaudio
import wave
import matplotlib.pyplot as plt
import librosa.display
import librosa

# make recording
RESPEAKER_RATE = 16000
RESPEAKER_CHANNELS = 2
RESPEAKER_WIDTH = 2
# run getDeviceInfo.py to get index
RESPEAKER_INDEX = 2  # refer to input device id
CHUNK = 1024
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "output.wav"

p = pyaudio.PyAudio()

stream = p.open(
    rate=RESPEAKER_RATE,
    format=p.get_format_from_width(RESPEAKER_WIDTH),
    channels=RESPEAKER_CHANNELS,
    input=True,
    input_device_index=RESPEAKER_INDEX, )

print("* recording")

frames = []

for i in range(0, int(RESPEAKER_RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("* done recording")

stream.stop_stream()
stream.close()
p.terminate()

wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
wf.setnchannels(RESPEAKER_CHANNELS)
wf.setsampwidth(p.get_sample_size(p.get_format_from_width(RESPEAKER_WIDTH)))
wf.setframerate(RESPEAKER_RATE)
wf.writeframes(b''.join(frames))
wf.close()

y, sr = librosa.load(WAVE_OUTPUT_FILENAME)
# make melspectrogram image
mfccs = librosa.feature.melspectrogram(y=y, sr=sr, fmax=sr/2)
# mfccs = np.mean(mfccs.T, axis=0)
mfccs = librosa.power_to_db(mfccs, ref=np.max)

fig = plt.figure(figsize=(5, 4))
librosa.display.specshow(mfccs, x_axis='time')
plt.clim(-60, 0)
plt.tight_layout()
plt.gcf()
plt.savefig('melspectrogram_raw.png')
plt.close()

# Setting the points for cropped image
left = 19
top = 15
right = 484
bottom = 340

im = Image.open('melspectrogram_raw.png')
im1 = im.crop((left, top, right, bottom))
img = im1.resize((256, 256), Image.ANTIALIAS)
img.save('melspectrogram_cropped.png')
# load the image
image = Image.open('melspectrogram_cropped.png')

# convert image to numpy array
data = np.asarray(image, dtype=np.float32)
data = data[:,:,0:3]
data = tf.expand_dims(data, 0)
# Load TFLite model and allocate tensors.
interpreter = tf.lite.Interpreter(model_path="converted_model_5-23.tflite")
interpreter.allocate_tensors()

# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Test model on random input data.
input_shape = input_details[0]['shape']
input_data = data
interpreter.set_tensor(input_details[0]['index'], input_data)

interpreter.invoke()

# The function `get_tensor()` returns a copy of the tensor data.
# Use `tensor()` in order to get a pointer to the tensor.
output_data = interpreter.get_tensor(output_details[0]['index']).flatten()
if output_data[0] > output_data[1]:
    print('There\'s no queen in that bih')
else:
    print('Go off queen')
