"""Evaluate the trained models and emit the results table and confusion matrices.

Follows model_evaluation.ipynb: reload the best checkpoint for each model, recreate the
same split, and report macro precision, macro recall and accuracy, together with a
confusion matrix.

Usage:
    python repro/evaluate.py [--models vanilla,stacked,bidirectional]
    python repro/evaluate.py --models cnn_lstm,cnn_bidirectional --images
"""

from __future__ import annotations

import argparse
import json
import os

import numpy as np

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")

import tensorflow as tf                                          # noqa: E402
from sklearn.metrics import (accuracy_score, confusion_matrix,   # noqa: E402
                             precision_score, recall_score)

from train import load_split                                     # noqa: E402

MODEL_LABEL = {"vanilla": "Standard LSTM", "stacked": "Stacked LSTM",
               "bidirectional": "Bidirectional LSTM", "cnn_lstm": "CNN-LSTM",
               "cnn_bidirectional": "Bidirectional CNN-LSTM"}


def metrics(model, x, y):
    predicted = model.predict(x, batch_size=512, verbose=0).argmax(1)
    true = y.argmax(1)
    return {
        "precision": float(precision_score(true, predicted, average="macro",
                                           zero_division=0)),
        "recall": float(recall_score(true, predicted, average="macro",
                                     zero_division=0)),
        "accuracy": float(accuracy_score(true, predicted)),
        "confusion": confusion_matrix(true, predicted).tolist(),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--features", default="TCC_data/repro")
    ap.add_argument("--models-dir", default="TCC_data/repro/models")
    ap.add_argument("--models", default="vanilla,stacked,bidirectional")
    ap.add_argument("--images", action="store_true")
    args = ap.parse_args()

    with open(os.path.join(args.features, "class_index.json")) as fh:
        class_index = json.load(fh)
    classes = sorted(class_index, key=class_index.get)

    splits = dict(zip(("train", "validation", "test"),
                      load_split(args.features, args.images)))

    results = {}
    for name in args.models.split(","):
        name = name.strip()
        path = os.path.join(args.models_dir, f"best_{name}_model.keras")
        if not os.path.exists(path):
            print(f"skipping {name}: {path} not found")
            continue
        model = tf.keras.models.load_model(path)
        results[name] = {split: metrics(model, *data) for split, data in splits.items()}

    with open(os.path.join(args.models_dir, "results.json"), "w") as fh:
        json.dump(results, fh, indent=1)

    for split in ("validation", "test"):
        print(f"\n### {split}\n")
        print("| Model | Precision | Recall | Accuracy |")
        print("|---|---|---|---|")
        for name, res in results.items():
            m = res[split]
            print(f"| {MODEL_LABEL.get(name, name)} | {m['precision']:.4f} "
                  f"| {m['recall']:.4f} | {m['accuracy']:.4f} |")

    for name, res in results.items():
        print(f"\nConfusion matrix ({MODEL_LABEL.get(name, name)}, validation):\n")
        header = "| true \\ pred | " + " | ".join(classes) + " |"
        print(header)
        print("|---" * (len(classes) + 1) + "|")
        for i, row in enumerate(res["validation"]["confusion"]):
            cells = " | ".join(f"**{v}**" if j == i else str(v)
                               for j, v in enumerate(row))
            print(f"| {classes[i]} | {cells} |")


if __name__ == "__main__":
    main()
