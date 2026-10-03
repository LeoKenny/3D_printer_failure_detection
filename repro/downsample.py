"""Filter and downsample the labelled acquisitions from 3200 Hz to 200 Hz.

Follows data_downsampling.ipynb: each axis is low-pass filtered with a Butterworth
filter designed for 1 dB attenuation at 80 Hz and 40 dB at 100 Hz, then decimated by
taking the median of every 16-sample window (`downsample_with_median` in the notebook;
verified to match the released 200 Hz parquet files exactly, sample for sample).

Usage:
    python repro/downsample.py
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import pandas as pd
from scipy import signal

ORIGINAL_FREQ = 3200
TARGET_FREQ = 200
FACTOR = ORIGINAL_FREQ // TARGET_FREQ
AXES = ["accel_x", "accel_y", "accel_z"]


def design_filter(pass_freq=80, stop_freq=100, attenuation=40, sample_f=ORIGINAL_FREQ):
    order, wn = signal.buttord(wp=pass_freq, ws=stop_freq, gpass=1, gstop=attenuation,
                               analog=False, fs=sample_f)
    return signal.butter(order, wn, fs=sample_f, btype="low", analog=False, output="sos")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default="TCC_data/labeled_dataset_typed")
    ap.add_argument("--out", default="TCC_data/labeled_dataset_downsampled_200_filter")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    sos = design_filter()

    files = sorted(f for f in os.listdir(args.data) if f.endswith(".parquet.gzip"))
    print(f"{len(files)} files in {args.data}")
    for name in files:
        df = pd.read_parquet(os.path.join(args.data, name))
        cls = df["class"].iloc[0]
        filtered = pd.DataFrame(
            {axis: signal.sosfilt(sos, df[axis]) for axis in AXES})
        downsampled = (filtered.rolling(window=FACTOR).median()
                       .iloc[FACTOR - 1::FACTOR].reset_index(drop=True))
        for axis in AXES:
            downsampled[axis] = downsampled[axis].astype(np.int16)
        downsampled["class"] = pd.Categorical([cls] * len(downsampled))
        downsampled.to_parquet(os.path.join(args.out, name), compression="gzip")
        print(f"  {name:<40}{len(df):>9,} -> {len(downsampled):>7,} samples  class={cls}")


if __name__ == "__main__":
    main()
