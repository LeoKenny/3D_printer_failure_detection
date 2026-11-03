# Reproducing the published results

Steps to regenerate the dataset statistics and the model results from the raw
acquisitions. All five stages are scripts in `repro/` that carry out the same pipeline
as the original training notebooks (archived under [`notebooks/`](notebooks/)) in a
form that runs end to end and unattended.

```
raw acquisitions (CSV or Zenodo parquet) ──1──▶ labelled parquet ──2──▶ 200 Hz parquet ──3──▶ features ──4──▶ models ──5──▶ results
```

## Environment

The environment is managed by [pixi](https://pixi.sh) and pinned in `pixi.lock`, so
`pixi install` reproduces the exact dependency versions below on any machine.

```bash
curl -fsSL https://pixi.sh/install.sh | sh   # if pixi isn't already installed
pixi install
```

Stages 1-3 (labelling, filtering, feature extraction) and stages 4-5 (training,
evaluation) live in **two separate pixi environments**, because TensorFlow pins numpy,
scipy and scikit-learn tighter than the rest of the pipeline needs:

| Environment | Stages |
| ----------- | ------ |
| `default`   | 1-3    |
| `train`     | 4-5    |

Run a stage with `pixi run -e <environment> python repro/<script>.py`, or use the
predefined tasks (`pixi run label`, `pixi run downsample`, `pixi run features`,
`pixi run -e train train`, `pixi run -e train evaluate`) shown under each stage below.

The `train` environment pins TensorFlow 2.19.0 and Keras 3.9.2 (Python 3.9). The `default`
environment is the one used for stages 1-3.

Floating-point results can differ between TensorFlow builds, CPU and GPU hardware, and
library versions. The pinned environment makes the software match; it does not make
training bit-identical across machines.

## Data layout

Everything produced by the pipeline lives under `TCC_data/`, which is not tracked by git.
The raw acquisitions are published on Zenodo ([record 22716165](https://zenodo.org/records/22716165))
as 26 parquet files, one per raw acquisition. Download them into `TCC_data/parquet_raw_dataset/`
(or use CSV files from the acquisition system in `TCC_data/raw_dataset/`).

```
TCC_data/
  parquet_raw_dataset/                      26 parquet files from Zenodo (same rows as the CSVs)
  raw_dataset/                              26 CSV files from the acquisition system (alternative input)
  labeled_dataset_typed/                    stage 1 output, 3200 Hz
  labeled_dataset_downsampled_200_filter/   stage 2 output, 200 Hz
  repro/                                    stages 3-5 outputs
```

## Stage 1 — label the raw acquisitions

```bash
pixi run label                                   # reads TCC_data/raw_dataset (CSV)
```

From the Zenodo download instead:

```bash
pixi run python repro/label_raw.py --raw TCC_data/parquet_raw_dataset --raw-format parquet
```

Each raw acquisition covers more than the print itself: the acquisition is started before the
job and stopped after it. `raw_data_manipulation.ipynb` originally cut this down by
plotting `accel_x` and reading the print interval off by hand, one acquisition at a
time. `repro/label_raw.py` carries out the same slicing unattended, from the interval
and class recorded for each print in `repro/acquisition_intervals.csv`: it slices to that
interval, attaches the class, drops the transport columns (`block`, `count`, `overrun`,
`queue_state`), casts to `int16` and writes the parquet.

It also runs the notebook's acquisition-integrity checks before slicing — `overrun` and
communication-failure bit counts, `block` continuity via `df["block"].diff()`, and NaN
checks on each axis — printed per file rather than plotted.

Output: `TCC_data/labeled_dataset_typed/benchy_<n>_<class>.parquet.gzip`, 27 files from
the 26 raw acquisitions (one raw file holds two back-to-back prints: benchy_11_12).

## Stage 2 — filter and downsample to 200 Hz

```bash
pixi run downsample
```

The reference study sampled at 200 Hz, so the 3200 Hz signal is low-pass filtered and
decimated. The filter is a Butterworth designed by `scipy.signal.buttord` for 1 dB at
80 Hz and 40 dB at 100 Hz, which yields order 24:

```python
N, Wn = signal.buttord(wp=80, ws=100, gpass=1, gstop=40, analog=False, fs=3200)
sos = signal.butter(N, Wn, fs=3200, btype="low", output="sos")
```

Filtering is applied per axis. The 200 Hz samples are then taken from a 16-sample rolling
median of the filtered signal, keeping every 16th value (`df.rolling(16).median()` followed
by `iloc[15::16]`). `repro/downsample.py` runs this over the whole folder.

Output: `TCC_data/labeled_dataset_downsampled_200_filter/`, 27 files.

## Stage 3 — features

```bash
pixi run features
```

Normalises each axis using statistics pooled over the dataset, segments into 44-sample
windows with 50 % overlap, and reduces each window to mean, standard deviation and first
principal component per axis — a 9-dimensional vector. Vectors are then grouped into
sequences of 10 timesteps.

`--images` additionally builds the image representation: the three axes stacked in the
order 1-2-3-1 to form a 44×4 image, reduced to `log10(|FFT2| + 1)`.

Expected output:

```
27 files in TCC_data/labeled_dataset_downsampled_200_filter
  normalisation mean=[-0.022767 -0.000250  0.425919] std=[0.040802 0.026676 0.110972]
  ...
  handcrafted (124175, 10, 9) -> TCC_data/repro/x_data.npy
  per-class sequences: [27744, 23239, 26404, 9245, 13984, 23559]
  image       (124175, 10, 44, 4) -> TCC_data/repro/x_image.npy
```

Windowing is done with `sliding_window_view`.

## Stage 4 — train

```bash
pixi run -e train train
```

which runs both `pixi run -e train train-lstm` (the three LSTM variants) and
`pixi run -e train train-cnn` (the two CNN-LSTM hybrids, on the image features).

The sequences are split 70 / 20 / 10, stratified, with `random_state=42`:

```python
x_train, x_rest, y_train, y_rest = train_test_split(
    x, y, test_size=0.3, random_state=42, stratify=y)
x_val, x_test, y_val, y_test = train_test_split(
    x_rest, y_rest, test_size=0.34, random_state=42, stratify=y_rest)
```

giving 86,922 / 24,586 / 12,667 sequences (124,175 in total) — 2,717 training batches per
epoch at batch size 32.

All models use ReLU in the hidden layers, Glorot uniform initialisation and a softmax
output, trained with Adam and categorical cross-entropy for 100 epochs.
`ModelCheckpoint(monitor="val_accuracy", save_best_only=True)` keeps the best epoch.

| Model                  | Architecture                                                                  | Parameters |
| ---------------------- | ----------------------------------------------------------------------------- | ---------- |
| Standard LSTM          | LSTM(100) → Dropout(0.2) → Dense(6)                                           | 44,606     |
| Stacked LSTM           | LSTM(50, seq) → LSTM(50) → Dropout(0.2) → Dense(6)                            | 32,506     |
| Bidirectional LSTM     | Bi-LSTM(100) → Dropout(0.2) → Dense(6)                                        | 89,206     |
| CNN-LSTM               | TD(Conv1D(32,3) → MaxPool(2) → Flatten) → LSTM(50, seq) → Dropout(0.5) → LSTM(50) → Dropout(0.5) → Dense(6) | 165,522    |
| Bidirectional CNN-LSTM | TD(Conv1D(32,3) → MaxPool(2) → Flatten) → Bi-LSTM(100) → Dropout(0.5) → Dense(6)                             | 620,022    |

Outputs: `TCC_data/repro/models/best_<name>_model.keras` and
`training_history_<name>.json` per model.

## Stage 5 — evaluate

```bash
pixi run -e train evaluate
```

Reloads each best checkpoint, recreates the same split, and prints macro precision,
macro recall and accuracy with confusion matrices. Results are written to
`TCC_data/repro/models/results.json`.

## Results

> **Provenance of these numbers.** The tables below were produced from the stored checkpoints
> in `TCC_data/repro/models/`. Those checkpoints were trained on an earlier build of the
> labelled data (their files are dated 9 Aug, before the label fix in commit `a011a7c`), so
> rerunning stages 4-5 on the current data gives different numbers. Re-evaluating the stored
> checkpoints with `repro/evaluate.py` reproduces the tables exactly.
>
> Training uses `random_state=42` throughout and `tf.keras.utils.set_random_seed(42)` before
> each fit. The table below lists the best epoch of each stored training run.

Best epoch selected by validation accuracy over 100 epochs:

| Model                  | Parameters | Best epoch |
| ---------------------- | ---------- | ---------- |
| Standard LSTM          | 44,606     | 38         |
| Stacked LSTM           | 32,506     | 63         |
| Bidirectional LSTM     | 89,206     | 33         |
| CNN-LSTM               | 165,522    | 49         |
| Bidirectional CNN-LSTM | 620,022    | 17         |

### Validation

| Model                  | Precision  | Recall     | Accuracy   |
| ---------------------- | ---------- | ---------- | ---------- |
| Standard LSTM          | **0.7319** | 0.6949     | **0.6847** |
| Stacked LSTM           | 0.7082     | **0.7042** | 0.6800     |
| Bidirectional LSTM     | 0.6938     | 0.6906     | 0.6697     |
| CNN-LSTM               | 0.7081     | 0.6779     | 0.6667     |
| Bidirectional CNN-LSTM | 0.6965     | 0.6582     | 0.6543     |

### Test

| Model                  | Precision  | Recall     | Accuracy   |
| ---------------------- | ---------- | ---------- | ---------- |
| Standard LSTM          | **0.7358** | 0.6967     | **0.6869** |
| Stacked LSTM           | 0.7057     | **0.7023** | 0.6783     |
| Bidirectional LSTM     | 0.6958     | 0.6927     | 0.6713     |
| CNN-LSTM               | 0.7094     | 0.6778     | 0.6669     |
| Bidirectional CNN-LSTM | 0.6914     | 0.6544     | 0.6504     |

### Comparison with the published accuracy

The published accuracy is not reproduced exactly by any model. Differences are the
reproduced accuracy minus the published one:

| Model                  | Published accuracy | Validation (Δ) | Test (Δ)  |
| ---------------------- | ------------------ | -------------- | --------- |
| Standard LSTM          | 0.6900             | 0.6847 (−0.0053) | 0.6869 (−0.0031) |
| Stacked LSTM           | 0.7002             | 0.6800 (−0.0202) | 0.6783 (−0.0219) |
| Bidirectional LSTM     | 0.6933             | 0.6697 (−0.0236) | 0.6713 (−0.0220) |
| CNN-LSTM               | 0.6734             | 0.6667 (−0.0067) | 0.6669 (−0.0065) |
| Bidirectional CNN-LSTM | 0.6676             | 0.6543 (−0.0133) | 0.6504 (−0.0172) |

The reproduced results do not match the published ranking. The Standard LSTM has the
highest accuracy on both validation and test, while the published table has the Stacked
LSTM first. The Stacked LSTM has the highest recall on both splits. The two hybrid
CNN-LSTM architectures have the lowest accuracy on both splits: both use the image
representation and score below all three handcrafted-feature LSTM models on both splits.

### Confusion matrix — Stacked LSTM, validation

| true \ pred | healthy  | temp_220 | temp_230 | nozzle_03 | nozzle_02 | loose_head |
| ----------- | -------- | -------- | -------- | --------- | --------- | ---------- |
| healthy     | **2769** | 1752     | 822      | 78        | 32        | 40         |
| temp_220    | 30       | **3318** | 926      | 116       | 73        | 138        |
| temp_230    | 21       | 1685     | **2362** | 256       | 842       | 62         |
| nozzle_03   | 0        | 43       | 54       | **1218**  | 514       | 2          |
| nozzle_02   | 2        | 11       | 10       | 212       | **2529**  | 4          |
| loose_head  | 9        | 104      | 17       | 3         | 10        | **4522**   |

This reproduces the structure reported in the paper. The nominal, 220 °C and 230 °C
classes are heavily mixed — all three lie within the manufacturer's recommended
operating range and differ only in nozzle temperature. The two nozzle-obstruction levels
are also confused with each other, though the models separate the presence of a nozzle
fault from the temperature classes. The loose X-axis carriage is the best identified
state, at recall 0.97.

`repro/evaluate.py` prints the equivalent matrix for every model and writes all figures
to `TCC_data/repro/models/results.json`.
