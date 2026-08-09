"""Train the five fault-classification models.

Follows lstm_training.ipynb and cnn_lstm_training.ipynb: a 70/20/10 stratified split
of the sequences, ReLU activations in the hidden layers, Glorot uniform initialisation,
softmax output, Adam, categorical cross-entropy, batch size 32, and a checkpoint that
keeps the epoch with the best validation accuracy.

Usage:
    python repro/train.py [--models vanilla,stacked,bidirectional]
    python repro/train.py --models cnn_lstm,cnn_bidirectional --images
"""

from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")

import tensorflow as tf                                            # noqa: E402
from keras import Sequential                                       # noqa: E402
from keras.callbacks import ModelCheckpoint                        # noqa: E402
from keras.initializers import GlorotUniform                       # noqa: E402
from keras.layers import (Bidirectional, Conv1D, Dense, Dropout,   # noqa: E402
                          Flatten, Input, LSTM, MaxPooling1D, TimeDistributed)
from keras.utils import to_categorical                             # noqa: E402
from sklearn.model_selection import train_test_split               # noqa: E402

N_CLASSES = 6
RANDOM_STATE = 42


def build_model(name, input_shape):
    init = GlorotUniform()
    if name == "vanilla":
        return Sequential([Input(input_shape),
                           LSTM(100, activation="relu", kernel_initializer=init),
                           Dropout(0.2), Dense(N_CLASSES, activation="softmax")])
    if name == "stacked":
        return Sequential([Input(input_shape),
                           LSTM(50, activation="relu", return_sequences=True,
                                kernel_initializer=init),
                           LSTM(50, activation="relu", kernel_initializer=init),
                           Dropout(0.2), Dense(N_CLASSES, activation="softmax")])
    if name == "bidirectional":
        return Sequential([Input(input_shape),
                           Bidirectional(LSTM(100, activation="relu",
                                              kernel_initializer=init)),
                           Dropout(0.2), Dense(N_CLASSES, activation="softmax")])
    if name in ("cnn_lstm", "cnn_bidirectional"):
        cnn = TimeDistributed(Sequential([Conv1D(32, 3, activation="relu"),
                                          MaxPooling1D(2), Flatten()]))
        if name == "cnn_lstm":
            tail = [LSTM(50, activation="relu", return_sequences=True,
                         kernel_initializer=init), Dropout(0.5),
                    LSTM(50, activation="relu", kernel_initializer=init), Dropout(0.5)]
        else:
            tail = [Bidirectional(LSTM(100, activation="relu",
                                       kernel_initializer=init)), Dropout(0.5)]
        return Sequential([Input(input_shape), cnn, *tail,
                           Dense(N_CLASSES, activation="softmax")])
    raise ValueError(f"unknown model {name!r}")


def load_split(features_dir, use_images):
    x = np.load(os.path.join(features_dir,
                             "x_image.npy" if use_images else "x_data.npy"))
    y = to_categorical(np.load(os.path.join(features_dir, "y_data.npy")),
                       num_classes=N_CLASSES)
    x_train, x_rest, y_train, y_rest = train_test_split(
        x, y, test_size=0.3, random_state=RANDOM_STATE, stratify=y)
    x_val, x_test, y_val, y_test = train_test_split(
        x_rest, y_rest, test_size=0.34, random_state=RANDOM_STATE, stratify=y_rest)
    return (x_train, y_train), (x_val, y_val), (x_test, y_test)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--features", default="TCC_data/repro")
    ap.add_argument("--out", default="TCC_data/repro/models")
    ap.add_argument("--models", default="vanilla,stacked,bidirectional")
    ap.add_argument("--images", action="store_true")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--batch-size", type=int, default=32)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    train, val, test = load_split(args.features, args.images)
    print(f"train {train[0].shape}  validation {val[0].shape}  test {test[0].shape}")

    for name in args.models.split(","):
        name = name.strip()
        tf.keras.utils.set_random_seed(RANDOM_STATE)
        model = build_model(name, train[0].shape[1:])
        model.compile(optimizer="adam", loss="categorical_crossentropy",
                      metrics=["accuracy"])
        checkpoint = os.path.join(args.out, f"best_{name}_model.keras")
        started = time.time()
        history = model.fit(
            *train, epochs=args.epochs, batch_size=args.batch_size,
            validation_data=val, shuffle=True, verbose=0,
            callbacks=[ModelCheckpoint(checkpoint, monitor="val_accuracy",
                                       save_best_only=True, mode="max", verbose=0)],
        )
        with open(os.path.join(args.out, f"training_history_{name}.json"), "w") as fh:
            json.dump({k: [float(v) for v in vals]
                       for k, vals in history.history.items()}, fh)
        best = int(np.argmax(history.history["val_accuracy"]))
        print(f"  {name:<18}{model.count_params():>9,} params  "
              f"{len(history.history['loss'])} epochs  {time.time() - started:>6.0f}s  "
              f"best epoch {best + 1} val acc {history.history['val_accuracy'][best]:.4f}")


if __name__ == "__main__":
    main()
