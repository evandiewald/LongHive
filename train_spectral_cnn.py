import tensorflow as tf
import numpy as np
from tensorflow.keras import datasets, layers, models, optimizers
import matplotlib.pyplot as plt
import pathlib
from sklearn.model_selection import train_test_split


data_dir = 'Training Images2'
data_dir = pathlib.Path(data_dir)
image_count = len(list(data_dir.glob('*/*.png')))
CLASS_NAMES = np.array([item.name for item in data_dir.glob('*') if item.name != "LICENSE.txt"])
image_generator = tf.keras.preprocessing.image.ImageDataGenerator(rescale=1./255, validation_split=0.2)
BATCH_SIZE = 32
IMG_HEIGHT = 256
IMG_WIDTH = 256
STEPS_PER_EPOCH = np.ceil(image_count/BATCH_SIZE)

train_generator = image_generator.flow_from_directory(
    data_dir,
    target_size=(IMG_HEIGHT, IMG_WIDTH),
    batch_size=BATCH_SIZE,
    classes=list(CLASS_NAMES),
    shuffle=True,
    subset='training') # set as training data

validation_generator = image_generator.flow_from_directory(
    data_dir,
    target_size=(IMG_HEIGHT, IMG_WIDTH),
    batch_size=BATCH_SIZE,
    classes=list(CLASS_NAMES),
    shuffle=True,
    subset='validation') # set as training data

test_dir = 'Test Images2'
test_dir = pathlib.Path(test_dir)
test_generator = tf.keras.preprocessing.image.ImageDataGenerator(rescale=1./255)
test_generator = test_generator.flow_from_directory(test_dir, target_size=(IMG_HEIGHT, IMG_WIDTH))

# model architecture 
model = models.Sequential()
model.add(layers.Conv2D(16, (3, 3), input_shape=(256, 256, 3)))
model.add(layers.LeakyReLU())
model.add(layers.MaxPooling2D((2, 2)))

model.add(layers.Conv2D(32, (3, 3)))
model.add(layers.LeakyReLU())
model.add(layers.MaxPooling2D((2, 2)))

model.add(layers.Conv2D(32, (3, 3)))
model.add(layers.LeakyReLU())
model.add(layers.MaxPooling2D((2, 2)))

model.add(layers.Conv2D(64, (3, 3)))
model.add(layers.LeakyReLU())
model.add(layers.MaxPooling2D((2, 2)))

model.add(layers.Conv2D(64, (3, 3)))
model.add(layers.LeakyReLU())
model.add(layers.MaxPooling2D((2, 2)))

# model.add(layers.Conv2D(64, (3, 3)))
# model.add(layers.LeakyReLU())
# model.add(layers.MaxPooling2D((2, 2)))



model.add(layers.Flatten())
# model.add(layers.Dense(128))
# model.add(layers.LeakyReLU())
model.add(layers.Dense(32))
model.add(layers.LeakyReLU())
model.add(layers.Dense(16))
model.add(layers.LeakyReLU())
model.add(layers.Dense(2))
model.summary()

opt = optimizers.Adam(learning_rate=0.0001)
# opt = optimizers.Adagrad(
#     learning_rate=0.0001,
#     initial_accumulator_value=0.1,
#     epsilon=1e-07,
#     name="Adagrad"
# )

model.compile(
    # learning_rate=0.0001,
    optimizer=opt,
    loss=tf.keras.losses.BinaryCrossentropy(),
    metrics=['accuracy'])

callback = tf.keras.callbacks.EarlyStopping(monitor='val_accuracy', patience=5)
history = model.fit(
    train_generator,
    steps_per_epoch=train_generator.samples // BATCH_SIZE,
    validation_data=validation_generator,
    epochs=100,
    callbacks=callback)

plt.plot(history.history['accuracy'], label='accuracy')
plt.plot(history.history['val_accuracy'], label = 'val_accuracy')
plt.xlabel('Epoch')
plt.ylabel('Accuracy')
# plt.ylim([0.5, 1])
plt.legend(loc='lower right')
plt.show()

test_loss, test_acc = model.evaluate(test_generator, verbose=2)
print('Test Loss: ', test_loss, ' Test Accuracy: ', test_acc)

#model.save_weights('Trained Models/trained_model.ckpt')
model.save('Trained Model 5-23')
