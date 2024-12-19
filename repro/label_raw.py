"""Slice and label the raw acquisitions.

Follows raw_data_manipulation.ipynb: each raw CSV covers more than the print itself, so
the print interval is cut out, the transport columns are dropped, the class is attached,
and the result is cast to int16 and written as parquet.

The interval and class for each print were originally read off a plot of `accel_x` by
hand, one raw file at a time; `acquisition_intervals.csv` records the result of that so
this step can run over the whole folder unattended. The notebook's integrity checks
(overrun/comm-failure bits, block continuity, NaNs) are reproduced as printed diagnostics
rather than a manual plot.

Usage:
    python repro/label_raw.py
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import pandas as pd

OVERRUN_BIT = 0
COMM_FAILURE_BIT = 1
AXES = ["accel_x", "accel_y", "accel_z"]


def check_integrity(df, name):
    overrun = (df["overrun"] & (1 << OVERRUN_BIT)).astype(bool).sum()
    comm_failure = (df["overrun"] & (1 << COMM_FAILURE_BIT)).astype(bool).sum()
    gaps = int((df["block"].diff() > 1).sum())
    backwards = int((df["block"].diff() < 0).sum())
    nans = {axis: int(df[axis].isna().sum()) for axis in AXES}
    print(f"  {name}: {len(df):>9,} rows  overrun={overrun}  comm_failure={comm_failure}"
          f"  block_gaps={gaps}  block_backwards={backwards}  nans={nans}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw", default="TCC_data/raw_dataset")
    ap.add_argument("--intervals", default="repro/acquisition_intervals.csv")
    ap.add_argument("--out", default="TCC_data/labeled_dataset_typed")
    args = ap.parse_args()

    intervals = pd.read_csv(args.intervals, comment="#").rename(columns={"class": "cls"})
    os.makedirs(args.out, exist_ok=True)
    print(f"{len(intervals)} labelled prints from {intervals['raw_file'].nunique()} "
          f"raw acquisitions in {args.raw}")

    raw_cache = {}
    for row in intervals.itertuples(index=False):
        if row.raw_file not in raw_cache:
            raw_cache.clear()  # one raw CSV resident at a time; each is ~500 MB
            df = pd.read_csv(os.path.join(args.raw, row.raw_file))
            check_integrity(df, row.raw_file)
            raw_cache[row.raw_file] = df
        df = raw_cache[row.raw_file]

        sliced = df.iloc[row.start:row.start + row.n_samples].copy()
        sliced["class"] = pd.Categorical([row.cls] * len(sliced))
        sliced = sliced.drop(columns=["block", "count", "overrun", "queue_state"])
        for axis in AXES:
            sliced[axis] = sliced[axis].astype(np.int16)

        out_path = os.path.join(args.out, f"{row.labeled_file}.parquet.gzip")
        sliced.to_parquet(out_path, compression="gzip")
        print(f"    -> {row.labeled_file:<20}{len(sliced):>9,} samples  class={row.cls}")


if __name__ == "__main__":
    main()
