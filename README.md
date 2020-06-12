# LongHive

Code for a smart beehive based on LongFi networking. Integrates temperature, humidity, & pressure data, as well as a deep learning-based audio classifier for automatically detecting whether or not a hive has a queen with 89% accuracy on a test set. The hardware consists of a Raspberry Pi (for running the tensorflow lite interpreter and dual microphone input) and ST-LRWAN development board.

## STM Board Code
The .ino script is uploaded to the board. You should only need to change the AppKey, AppEUI, and DevEUI at the top. It works with the X-NUCLEO MEMS shield and listens to the serial port for the classification, which is converted to an analog input in the CayenneLPP packet. 
**Still need to add weight sensors

## Python Code (for Raspberry Pi)
The python code runs a tensorflow lite classifier on the RPi. It works with the Seeed ReSpeaker 2-Mics Pi HAT to classify audio signals (bees buzzing) based on a pre-trained CNN. Code is pretty processing-intensive (generating the MFCC plots is a lot on the Pi) and definitely a point of future optimization. The CNN architecture is below. Keep in mind that installing dependencies on the RPi (ARM) is a serious pain...

I'll also upload a script for standard venvs and my code used to train the network. 

CNN Architecture: 

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
