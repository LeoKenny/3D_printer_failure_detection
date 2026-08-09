# Reproducing the published results

Steps to regenerate the dataset statistics and the model results from the raw
acquisitions. Stages 1 and 2 are the notebooks used to build the dataset; stages 3 to 5
are scripts in `repro/` that carry out the same pipeline as the training notebooks in a
form that runs end to end.

```
raw CSV  ──1──▶ labelled parquet ──2──▶ 200 Hz parquet ──3──▶ features ──4──▶ models ──5──▶ results
```

## Environment

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r repro/requirements.txt
```

Training needs TensorFlow, which pins NumPy tightly and is best installed in its own
environment:

```bash
pip install "tensorflow>=2.16"
```

Verified with Python 3.13, NumPy 2.5.1, pandas 3.0.5, pyarrow 25.0.0, SciPy 1.18.0,
scikit-learn 1.9.0, TensorFlow 2.19.0 / Keras 3.9.2. The published results were produced
on TensorFlow 2.17 / Keras 3.

A GPU is not required. On an RTX 3060 Laptop each LSTM model takes roughly 20 s per
epoch at batch size 32.

Two version-sensitive points are worth pinning if exact agreement matters:

- `pandas.DataFrame.rolling(step=)` requires pandas ≥ 1.5.
- scikit-learn changed the default PCA solver in 1.5. For 44×3 windows it now selects
  `covariance_eigh`, which resolves the sign of a component by making its
  largest-magnitude entry positive; earlier versions used `full` with a different
  convention. `repro/extract_features.py --check-pca` verifies its vectorised
  implementation against whichever scikit-learn is installed.

## Data layout

Everything lives under `TCC_data/`, which is not tracked by git:

```
TCC_data/
  raw_dataset/                              26 CSV files from the acquisition system
  labeled_dataset_typed/                    stage 1 output, 3200 Hz
  labeled_dataset_downsampled_200_filter/   stage 2 output, 200 Hz
  repro/                                    stages 3-5 outputs
```

## Stage 1 — label the raw acquisitions

`raw_data_manipulation.ipynb`, run once per acquisition.

Each raw CSV covers more than the print itself: the acquisition is started before the
job and stopped after it. The notebook plots `accel_x` so the print interval can be read
off, then slices to that interval, attaches the class, drops the transport columns
(`block`, `count`, `overrun`, `queue_state`), casts to `int16` and writes the parquet.

Per file, set the interval and the class:

```python
df_printing = df.iloc[70500:16754000]
df_printing["class"] = "nozzle_02"
```

The notebook also checks acquisition integrity before slicing — `overrun` and
communication-failure bit counts, `block` continuity via `df["block"].diff()`, and NaN
checks on each axis.

Output: `TCC_data/labeled_dataset_typed/benchy_<n>_<class>.parquet.gzip`.

## Stage 2 — filter and downsample to 200 Hz

`data_downsampling.ipynb`, run once over the whole folder.

The reference study sampled at 200 Hz, so the 3200 Hz signal is low-pass filtered and
decimated. The filter is a Butterworth designed by `scipy.signal.buttord` for 1 dB at
80 Hz and 40 dB at 100 Hz, which yields order 24:

```python
N, Wn = signal.buttord(wp=80, ws=100, gpass=1, gstop=40, analog=False, fs=3200)
sos = signal.butter(N, Wn, fs=3200, btype="low", output="sos")
```

Filtering is applied per axis, then one sample in every 16 is kept, giving 200 Hz.

Output: `TCC_data/labeled_dataset_downsampled_200_filter/`, 28 files.

## Stage 3 — features

```bash
python repro/extract_features.py --check-pca --images
```

Normalises each axis using statistics pooled over the dataset, segments into 44-sample
windows with 50 % overlap, and reduces each window to mean, standard deviation and first
principal component per axis — a 9-dimensional vector. Vectors are then grouped into
sequences of 10 timesteps.

`--images` additionally builds the image representation: the three axes stacked in the
order 1-2-3-1 to form a 44×4 image, reduced to `log10(|FFT2| + 1)`.

Expected output:

```
28 files in TCC_data/labeled_dataset_downsampled_200_filter
  normalisation mean=[-0.021804 -0.000209  0.319926] std=[0.03814  0.024902 0.142855]
  ...
  handcrafted (128061, 10, 9) -> TCC_data/repro/x_data.npy
  per-class sequences: [27744, 23239, 25839, 9202, 18478, 23559]
  image       (128061, 10, 44, 4) -> TCC_data/repro/x_image.npy
```

Windowing is done with `sliding_window_view` rather than `DataFrame.rolling`, which is
several orders of magnitude faster; the two are verified to produce identical arrays.

## Stage 4 — train

```bash
python repro/train.py --models vanilla,stacked,bidirectional --epochs 100
python repro/train.py --models cnn_lstm,cnn_bidirectional --images --epochs 100
```

The sequences are split 70 / 20 / 10, stratified, with `random_state=42`:

```python
x_train, x_rest, y_train, y_rest = train_test_split(
    x, y, test_size=0.3, random_state=42, stratify=y)
x_val, x_test, y_val, y_test = train_test_split(
    x_rest, y_rest, test_size=0.34, random_state=42, stratify=y_rest)
```

giving 89,642 / 25,356 / 13,063 sequences — 2,802 batches per epoch at batch size 32.

All models use ReLU in the hidden layers, Glorot uniform initialisation and a softmax
output, trained with Adam and categorical cross-entropy for 100 epochs.
`ModelCheckpoint(monitor="val_accuracy", save_best_only=True)` keeps the best epoch.

| Model | Architecture | Parameters |
|---|---|---|
| Standard LSTM | LSTM(100) → Dropout(0.2) → Dense(6) | 44,606 |
| Stacked LSTM | LSTM(50, seq) → LSTM(50) → Dropout(0.2) → Dense(6) | 32,506 |
| Bidirectional LSTM | Bi-LSTM(100) → Dropout(0.2) → Dense(6) | 89,206 |
| CNN-LSTM | TD(Conv1D(32,3) → MaxPool(2) → Flatten) → LSTM(50, seq) → LSTM(50) → Dense(6) | 165,522 |
| Bidirectional CNN-LSTM | TD(Conv1D(32,3) → MaxPool(2) → Flatten) → Bi-LSTM(100) → Dense(6) | 620,022 |

Outputs: `TCC_data/repro/models/best_<name>_model.keras` and
`training_history_<name>.json` per model.

## Stage 5 — evaluate

```bash
python repro/evaluate.py --models vanilla,stacked,bidirectional
python repro/evaluate.py --models cnn_lstm,cnn_bidirectional --images
```

Reloads each best checkpoint, recreates the same split, and prints macro precision,
macro recall and accuracy with confusion matrices. Results are written to
`TCC_data/repro/models/results.json`.

## Results

Produced by the commands above on the environment recorded at the top, with
`random_state=42` throughout and `tf.keras.utils.set_random_seed(42)` before each fit.

Best epoch selected by validation accuracy over 100 epochs:

| Model | Parameters | Best epoch | Training time |
|---|---|---|---|
| Standard LSTM | 44,606 | 38 | 2,119 s |
| Stacked LSTM | 32,506 | 63 | 2,857 s |
| Bidirectional LSTM | 89,206 | 33 | 2,804 s |

### Validation

| Model | Precision | Recall | Accuracy |
|---|---|---|---|
| Standard LSTM | 0.7142 | 0.6999 | 0.6891 |
| Stacked LSTM | **0.7118** | **0.7044** | **0.6920** |
| Bidirectional LSTM | 0.7065 | 0.6961 | 0.6871 |

### Test

| Model | Precision | Recall | Accuracy |
|---|---|---|---|
| Standard LSTM | 0.7107 | 0.6981 | 0.6887 |
| Stacked LSTM | **0.7090** | **0.7033** | **0.6903** |
| Bidirectional LSTM | 0.7058 | 0.6945 | 0.6862 |

### Agreement with the published table

| Model | Published accuracy | Reproduced (validation) | Difference |
|---|---|---|---|
| Standard LSTM | 0.69 | 0.6891 | −0.001 |
| Stacked LSTM | 0.7002 | 0.6920 | −0.008 |
| Bidirectional LSTM | 0.6933 | 0.6871 | −0.006 |

Accuracies land within 0.008 of the published values and the Stacked LSTM remains the
best of the three. Precision reproduces within 0.02 and recall within 0.02; the residual
differences are consistent with the TensorFlow and scikit-learn version changes noted
above, neither of which is pinned in the original notebooks.

### Confusion matrix — Stacked LSTM, validation

| true \ pred | healthy | temp_220 | temp_230 | nozzle_03 | nozzle_02 | loose_head |
|---|---|---|---|---|---|---|
| healthy | **2756** | 1797 | 789 | 67 | 23 | 61 |
| temp_220 | 47 | **3455** | 825 | 76 | 47 | 152 |
| temp_230 | 33 | 1760 | **2304** | 610 | 350 | 59 |
| nozzle_03 | 1 | 74 | 52 | **1170** | 520 | 5 |
| nozzle_02 | 1 | 24 | 17 | 284 | **3325** | 7 |
| loose_head | 14 | 91 | 13 | 2 | 8 | **4537** |

This reproduces the structure reported in the paper. The nominal, 220 °C and 230 °C
classes are heavily mixed — all three lie within the manufacturer's recommended
operating range and differ only in nozzle temperature. The two nozzle-obstruction levels
are also confused with each other, though the models separate the presence of a nozzle
fault from the temperature classes. The loose X-axis carriage is the best identified
state, at recall 0.97.

`repro/evaluate.py` prints the equivalent matrix for every model and writes all figures
to `TCC_data/repro/models/results.json`.

## Runtime

| Stage | Time |
|---|---|
| 1 — labelling | manual, one pass per acquisition |
| 2 — filter and downsample | ~20 min for 28 files |
| 3 — features | ~13 min including the image representation |
| 4 — training | ~35 min per LSTM model, longer for the CNN variants |
| 5 — evaluation | ~1 min |

Disk: 11 GB for `raw_dataset/`, 938 MB for `labeled_dataset_typed/`, 64 MB at 200 Hz,
and 1.0 GB for the extracted features including the image representation.
