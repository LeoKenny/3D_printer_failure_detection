import numpy as np
import tensorflow as tf
from tensorflow.keras.layers import (
    LSTM,
    Conv1D,
    Dense,
    Flatten,
    Input,
    MaxPooling1D,
    TimeDistributed,
)
from tensorflow.keras.models import Sequential

# Example data: (samples, features)
X_train = np.random.rand(100, 64, 64)  # 100 samples, 64x64 features

# Aggregate features into timesteps
timesteps = 10
X_train_reshaped = X_train.reshape((
    X_train.shape[0] // timesteps,
    timesteps,
    *X_train.shape[1:],
))

# Target -> for each sample // timestep group
y_train = np.random.rand(  # samples // timesteps, 1 output
    X_train_reshaped.shape[0],
    1,
)

# LTSM-CNN
model = Sequential([
    Input((None, 64, 64)),
    TimeDistributed(
        Sequential([
            Conv1D(filters=64, kernel_size=3, activation="relu"),
            MaxPooling1D(pool_size=2),
            Flatten(),
        ]),
    ),
    LSTM(100, activation="relu", return_sequences=True),
    LSTM(100, activation="relu"),
    Dense(1),
])
model.compile(optimizer="adam", loss="mse")
model.summary()

# Train
history = model.fit(
    X_train_reshaped,
    y_train,
    epochs=2,
    batch_size=32,
)

# Results
print(history.history)
