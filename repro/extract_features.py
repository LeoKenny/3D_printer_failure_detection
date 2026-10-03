"""Build the model inputs from the 200 Hz dataset.

Follows data_feature_image_extraction.ipynb.

Handcrafted features: the signal is normalised per axis, segmented into 44-sample
windows with 50% overlap, and each window reduced to its mean, standard deviation and
first principal component per axis -- a 9-dimensional vector. Those vectors are then
concatenated into sequences of 10 timesteps.

Image features: the three axes are stacked in the order 1-2-3-1 to give a 44x4 image,
and the log magnitude of its 2D Fourier transform is taken.

Usage:
    python repro/extract_features.py [--images]
"""

from __future__ import annotations

import argparse
import json
import os
import re

import numpy as np
import pyarrow.parquet as pq

AXES = ["accel_x", "accel_y", "accel_z"]
CLASS_INDEX = {"healthy": 0, "temp_220": 1, "temp_230": 2,
               "nozzle_03": 3, "nozzle_02": 4, "loose_head": 5}

WINDOW, STEP, TIMESTEPS = 44, 22, 10
CONVERSION_CONSTANT = (2.0 * 16.0) / 8192


def extract_class(filename):
    match = re.search(r"benchy_\d+_(.*?)\.parquet\.gzip", filename)
    return match.group(1) if match else None


def window_using_rolling(signal):
    """44-sample windows at 50% overlap.

    Mirrors `df.rolling(window=44, step=22)` followed by `list(...)[2:]`, which drops
    the two leading partial windows and so starts at row 1.
    """
    view = np.lib.stride_tricks.sliding_window_view(signal, WINDOW, axis=0)
    return np.ascontiguousarray(view[1::STEP].transpose(0, 2, 1))


def first_principal_component(windows):
    """First principal component of each window, one row per window.

    Equivalent to fitting `sklearn.decomposition.PCA(n_components=1)` per window and
    taking `components_[0]`. sklearn selects its `covariance_eigh` solver for input of
    this shape, which resolves the sign of the component by making its largest-magnitude
    entry positive; that convention is reproduced here. See --check-pca.
    """
    centred = windows - windows.mean(1)[:, None, :]
    covariance = centred.transpose(0, 2, 1) @ centred
    _, vectors = np.linalg.eigh(covariance)
    component = vectors[..., -1]
    lead = np.take_along_axis(component, np.abs(component).argmax(1)[:, None], 1)
    return component * np.sign(np.where(lead == 0, 1.0, lead))


def feature_extraction(windows):
    return np.concatenate([windows.mean(1), windows.std(1),
                           first_principal_component(windows)], axis=1)


def image_extraction(windows, chunk=20000):
    """44x4 images stacked 1-2-3-1, as log10(|FFT2| + 1)."""
    stacked = np.concatenate([windows, windows[:, :, :1]], axis=2)
    out = np.empty(stacked.shape, dtype=np.float32)
    for i in range(0, len(stacked), chunk):
        block = stacked[i:i + chunk]
        out[i:i + chunk] = np.log10(np.abs(np.fft.fft2(block, axes=(1, 2))) + 1)
    return out


def to_sequences(features):
    """Concatenate consecutive feature vectors into TIMESTEPS-long sequences."""
    keep = features[len(features) % TIMESTEPS:]
    return keep.reshape(len(keep) // TIMESTEPS, TIMESTEPS, *keep.shape[1:])


def read_signal(path):
    table = pq.read_table(path, columns=AXES + ["class"])
    signal = np.stack([table[a].to_numpy() for a in AXES], 1).astype(np.float64)
    return signal, table["class"].to_pylist()[0]


def check_pca(n=200):
    """Confirm the vectorised principal component matches sklearn window by window."""
    from sklearn.decomposition import PCA
    rng = np.random.default_rng(0)
    mismatches = 0
    for _ in range(n):
        window = rng.normal(size=(WINDOW, 3)) * rng.uniform(0.1, 5, 3)
        reference = PCA(n_components=1).fit(window).components_[0]
        ours = first_principal_component(window[None])[0]
        mismatches += not np.allclose(ours, reference, atol=1e-9)
    print(f"PCA check: {n} windows, {mismatches} mismatches against sklearn")
    return mismatches == 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default="TCC_data/labeled_dataset_downsampled_200_filter")
    ap.add_argument("--out", default="TCC_data/repro")
    ap.add_argument("--images", action="store_true")
    ap.add_argument("--check-pca", action="store_true")
    args = ap.parse_args()

    if args.check_pca and not check_pca():
        raise SystemExit("PCA convention does not match this sklearn build")

    os.makedirs(args.out, exist_ok=True)
    files = sorted(f for f in os.listdir(args.data) if f.endswith(".parquet.gzip"))
    print(f"{len(files)} files in {args.data}")

    # normalisation statistics over the whole dataset, as in the notebook
    totals = np.zeros(3)
    squares = np.zeros(3)
    count = 0
    for name in files:
        signal, _ = read_signal(os.path.join(args.data, name))
        signal *= CONVERSION_CONSTANT
        totals += signal.sum(0)
        squares += (signal ** 2).sum(0)
        count += len(signal)
    mean = totals / count
    std = np.sqrt(squares / count - mean ** 2)
    print(f"  normalisation mean={np.round(mean, 6)} std={np.round(std, 6)}")

    x_parts, y_parts, img_parts = [], [], []
    for name in files:
        signal, cls = read_signal(os.path.join(args.data, name))
        normalised = (signal * CONVERSION_CONSTANT - mean) / std
        # window_using_rolling applies the conversion constant to the frame it receives
        windows = window_using_rolling(normalised * CONVERSION_CONSTANT)

        sequences = to_sequences(feature_extraction(windows))
        x_parts.append(sequences.astype(np.float32))
        y_parts.append(np.full(len(sequences), CLASS_INDEX[cls], dtype=np.int64))

        if args.images:
            raw_windows = window_using_rolling(signal * CONVERSION_CONSTANT)
            img_parts.append(to_sequences(image_extraction(raw_windows)))
        print(f"  {name:<40}{len(windows):>8,} windows -> {len(sequences):>7,} sequences"
              f"  class={cls}")

    x_data = np.concatenate(x_parts)
    y_data = np.concatenate(y_parts)
    np.save(os.path.join(args.out, "x_data.npy"), x_data)
    np.save(os.path.join(args.out, "y_data.npy"), y_data)
    with open(os.path.join(args.out, "class_index.json"), "w") as fh:
        json.dump(CLASS_INDEX, fh)
    print(f"\n  handcrafted {x_data.shape} -> {args.out}/x_data.npy")
    print(f"  per-class sequences: {np.bincount(y_data, minlength=6).tolist()}")

    if args.images:
        img = np.concatenate(img_parts)
        np.save(os.path.join(args.out, "x_image.npy"), img)
        print(f"  image       {img.shape} -> {args.out}/x_image.npy")


if __name__ == "__main__":
    main()
