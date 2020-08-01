# LongHive

*Check out our [project blog](https://www.hackster.io/354300/longhive-12d952), which was entered in the Helium #IoTforGood Contest on Hackster.io!*

Code for a smart beehive based on LongFi networking via the Helium Network. Integrates temperature, humidity, CO2, & pressure data, as well as a deep learning-based audio classifier that has been trained to detect whether or not a hive has a queen with 89% accuracy on a test set. The hardware consists of a Raspberry Pi 3/4 (for running the Tensorflow Lite interpreter based on dual microphone input), ST-LRWAN development board (from the [Helium Developer Kit](https://developer.helium.com/devices/devkit)), and various environmental sensors.

## ST-LRWAN Development Board Code
The [.cpp script](main.cpp) is uploaded to the board via the PlatformIO extension for VS Code. [Here](https://developer.helium.com/resources/getting-started-with-platformio) is a robust tutorial for getting started with PIO and the Helium Developer Kit. You should only need to change the AppKey, AppEUI, and DevEUI to match what you have in the Helium Console. For additional details and hardware schematics, please refer to the [project blog](https://www.hackster.io/354300/longhive-12d952).

## Raspberry Pi Setup
Setup was tested on a Raspberry Pi 3B/4B running the latest version of 32-bit Raspbian Buster. The ReSpeaker 2 Hat is installed on the GPIO pins and the Pi will communicate with the LoRaWAN board via the USB serial port. 

1. Update, install the required dependencies, and reboot

```
$ sudo apt-get install virtualenv python-pyaudio python3-scipy
$ sudo apt libatlas-base-dev
$ sudo apt-get update
$ sudo apt-get upgrade
$ git clone https://github.com/respeaker/seeed-voicecard.git
$ cd seeed-voicecard
$ sudo ./install.sh
$ reboot
```

2. Check that the ReSpeaker software was installed correctly and take note of the device index (this is the `RESPEAKER_INDEX` field in `classify_buzz_rpi.py`), taken from the [SeeedStudio docs](https://wiki.seeedstudio.com/ReSpeaker_2_Mics_Pi_HAT/)

```
~/seeed-voicecard $ aplay -l
**** List of PLAYBACK Hardware Devices ****
card 0: ALSA [bcm2835 ALSA], device 0: bcm2835 ALSA [bcm2835 ALSA]
  Subdevices: 8/8
  Subdevice #0: subdevice #0
  Subdevice #1: subdevice #1
  Subdevice #2: subdevice #2
  Subdevice #3: subdevice #3
  Subdevice #4: subdevice #4
  Subdevice #5: subdevice #5
  Subdevice #6: subdevice #6
  Subdevice #7: subdevice #7
card 0: ALSA [bcm2835 ALSA], device 1: bcm2835 ALSA [bcm2835 IEC958/HDMI]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: seeed2micvoicec [seeed-2mic-voicecard], device 0: bcm2835-i2s-wm8960-hifi wm8960-hifi-0 []
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```

3. Clone this repo, setup the virtualenv, and install dependencies from `requirements.txt`

```
$ git clone https://github.com/evandiewald/LongHive
$ cd LongHive

$ virtualenv venv --python=python3.7
source venv/bin/activate
```

:warning: Raspbian is not going to want to let you install scipy without root privileges. Before installing `requirements.txt`, I had to download and install the wheel directly by using

```
$ wget https://www.piwheels.org/simple/scipy/scipy-1.5.0rc2-cp37-cp37m-linux_armv7l.whl
$ pip3 install scipy-1.5.0rc2-cp37-cp37m-linux_armv7l.whl
```

Next, install the rest of the dependencies with

`$ pip3 install -r requirements.txt`

4. Finally, you may have to update `classify_buzz_rpi.py` to adjust the serial port (`dev/ttyUSB0`) and/or `RESPEAKER_INDEX` to match the port(s) your Pi is using. Run the classifier with

`$ python3 classify_buzz_rpi.py`

There are going to be a ton of error messages (it's a bit of a hack), but you'll know it's working when you see outputs like 

```
* recording
* done
...
No queen detected
```

## CNN Architecture
The Python code runs a TensorFlow Lite classifier on the RPi. It works with the Seeed ReSpeaker 2-Mics Pi HAT to classify audio signals (bees buzzing) based on a pre-trained CNN (dataset obtained from [Kaggle](https://www.kaggle.com/chrisfilo/to-bee-or-no-to-bee)). The program is pretty processing-intensive (generating the MFCC plots is a lot on the Pi) and definitely a point of future optimization. The CNN architecture from `train_spectral_cnn.py` is outlined below. 

CNN Architecture: 

```
Model: "sequential"
_________________________________________________________________
Layer (type)                 Output Shape              Param #   
=================================================================
conv2d (Conv2D)              (None, 254, 254, 16)      448       
_________________________________________________________________
leaky_re_lu (LeakyReLU)      (None, 254, 254, 16)      0         
_________________________________________________________________
max_pooling2d (MaxPooling2D) (None, 127, 127, 16)      0         
_________________________________________________________________
conv2d_1 (Conv2D)            (None, 125, 125, 32)      4640      
_________________________________________________________________
leaky_re_lu_1 (LeakyReLU)    (None, 125, 125, 32)      0         
_________________________________________________________________
max_pooling2d_1 (MaxPooling2 (None, 62, 62, 32)        0         
_________________________________________________________________
conv2d_2 (Conv2D)            (None, 60, 60, 32)        9248      
_________________________________________________________________
leaky_re_lu_2 (LeakyReLU)    (None, 60, 60, 32)        0         
_________________________________________________________________
max_pooling2d_2 (MaxPooling2 (None, 30, 30, 32)        0         
_________________________________________________________________
conv2d_3 (Conv2D)            (None, 28, 28, 64)        18496     
_________________________________________________________________
leaky_re_lu_3 (LeakyReLU)    (None, 28, 28, 64)        0         
_________________________________________________________________
max_pooling2d_3 (MaxPooling2 (None, 14, 14, 64)        0         
_________________________________________________________________
conv2d_4 (Conv2D)            (None, 12, 12, 64)        36928     
_________________________________________________________________
leaky_re_lu_4 (LeakyReLU)    (None, 12, 12, 64)        0         
_________________________________________________________________
max_pooling2d_4 (MaxPooling2 (None, 6, 6, 64)          0         
_________________________________________________________________
flatten (Flatten)            (None, 2304)              0         
_________________________________________________________________
dense (Dense)                (None, 32)                73760     
_________________________________________________________________
leaky_re_lu_5 (LeakyReLU)    (None, 32)                0         
_________________________________________________________________
dense_1 (Dense)              (None, 16)                528       
_________________________________________________________________
leaky_re_lu_6 (LeakyReLU)    (None, 16)                0         
_________________________________________________________________
dense_2 (Dense)              (None, 2)                 34        
=================================================================
Total params: 144,082
Trainable params: 144,082
Non-trainable params: 0
```
