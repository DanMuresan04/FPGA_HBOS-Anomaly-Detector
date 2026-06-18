"""
Off-chip comparison: Isolation Forest and One-Class SVM vs the FPGA HBOS result.

The two CPU algorithms are trained on the same training rows and evaluated on
the same test rows as the FPGA, so all three share identical ground-truth labels.
sklearn is required; if it is missing every result is returned as None and the
UI shows a "sklearn not installed" notice rather than crashing.
"""

from __future__ import annotations

import time
from typing import Optional

try:
    import numpy as np
    from sklearn.ensemble import IsolationForest
    from sklearn.preprocessing import StandardScaler
    from sklearn.svm import OneClassSVM

    _SKLEARN_OK = True
except ImportError:
    _SKLEARN_OK = False


# ── metric helpers ────────────────────────────────────────────────────────────

def compute_metrics(y_true: list[int], y_pred: list[int]) -> dict:
    """
    Compute classification metrics for binary anomaly detection.

    Parameters
    ----------
    y_true : list of 0/1  (1 = anomaly)
    y_pred : list of 0/1  (1 = anomaly)

    Returns
    -------
    dict with keys: tp, fp, tn, fn, precision, recall, f1, accuracy,
                    caught (str "TP/total_anomalies"), false_alarms (int),
                    total (int), total_anomalies (int)
    """
    n = min(len(y_true), len(y_pred))
    if n == 0:
        return {
            "tp": 0, "fp": 0, "tn": 0, "fn": 0,
            "precision": 0.0, "recall": 0.0, "f1": 0.0, "accuracy": 0.0,
            "caught": "0/0", "false_alarms": 0,
            "total": 0, "total_anomalies": 0,
        }

    tp = sum(1 for i in range(n) if y_true[i] == 1 and y_pred[i] == 1)
    fp = sum(1 for i in range(n) if y_true[i] == 0 and y_pred[i] == 1)
    tn = sum(1 for i in range(n) if y_true[i] == 0 and y_pred[i] == 0)
    fn = sum(1 for i in range(n) if y_true[i] == 1 and y_pred[i] == 0)

    total_anomalies = tp + fn

    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall    = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1        = (2 * precision * recall / (precision + recall)
                 if (precision + recall) > 0 else 0.0)
    accuracy  = (tp + tn) / n if n > 0 else 0.0

    return {
        "tp": tp, "fp": fp, "tn": tn, "fn": fn,
        "precision": precision,
        "recall":    recall,
        "f1":        f1,
        "accuracy":  accuracy,
        "caught":    f"{tp}/{total_anomalies}",
        "false_alarms": fp,
        "total":         n,
        "total_anomalies": total_anomalies,
    }


def _rows_to_xy(rows: list) -> tuple:
    """
    Parse CSV rows into (X, y_labels).

    Columns: s0 s1 s2 s3 label [extras…]
    Returns X as float array shape (n, 4), y as list of 0/1 ints.
    """
    X, y = [], []
    for row in rows:
        try:
            X.append([float(row[ch]) for ch in range(4)])
            y.append(1 if int(float(row[4])) != 0 else 0)
        except (IndexError, ValueError):
            pass
    if not X:
        return [], []
    try:
        import numpy as np
        return np.array(X, dtype=float), y
    except ImportError:
        return X, y


# ── main entry point ──────────────────────────────────────────────────────────

def run_comparison(
    train_rows:    list,
    test_rows:     list,
    hbos_preds:    list[int],
) -> dict:
    """
    Run the full three-way comparison.

    Parameters
    ----------
    train_rows  : CSV rows from the training phase (same format as test_rows)
    test_rows   : CSV rows from the test/detect phase
    hbos_preds  : per-sample 0/1 predictions already produced by the FPGA

    Returns
    -------
    dict with keys "hbos", "iforest", "ocsvm", "dataset", "sklearn_available"

      Each algorithm dict has all keys from compute_metrics() plus "fit_ms"
      and "predict_ms" (only for CPU algorithms).
      "dataset" has: total, total_anomalies, anomaly_rate_pct.
      "sklearn_available" is bool.
    """
    result: dict = {"sklearn_available": _SKLEARN_OK}

    # Extract ground-truth labels from test rows
    _, y_true = _rows_to_xy(test_rows)

    # Dataset summary (use the full y_true length)
    total          = len(y_true)
    total_anomalies = sum(y_true)
    result["dataset"] = {
        "total":            total,
        "total_anomalies":  total_anomalies,
        "anomaly_rate_pct": 100 * total_anomalies / total if total > 0 else 0.0,
    }

    # HBOS/FPGA metrics — align predictions to ground truth length
    aligned_preds = list(hbos_preds[:total]) + [0] * max(0, total - len(hbos_preds))
    result["hbos"] = compute_metrics(y_true, aligned_preds)
    result["hbos"]["fit_ms"]     = None
    result["hbos"]["predict_ms"] = None

    if not _SKLEARN_OK:
        result["iforest"] = None
        result["ocsvm"]   = None
        return result

    import numpy as np

    # Feature matrix for training (anomaly-free rows only → unsupervised)
    X_train_all, y_train = _rows_to_xy(train_rows)
    X_test,  _           = _rows_to_xy(test_rows)

    if len(X_train_all) == 0 or len(X_test) == 0:
        result["iforest"]  = None
        result["ocsvm"]    = None
        result["hbos_cpu"] = None
        return result

    # Train on clean-only samples (label == 0) to mirror real anomaly-detection usage
    clean_mask  = [i for i, lbl in enumerate(y_train) if lbl == 0]
    X_train_clean = X_train_all[clean_mask] if clean_mask else X_train_all

    scaler = StandardScaler()
    X_tr_s = scaler.fit_transform(X_train_clean)
    X_te_s = scaler.transform(X_test)

    # contamination: rough fraction of anomalies in the training set
    contamination = min(
        0.45,
        max(0.001, (len(y_train) - len(clean_mask)) / max(1, len(y_train)))
    )

    # ── Isolation Forest ──────────────────────────────────────────────────────
    t0 = time.perf_counter()
    iforest = IsolationForest(
        n_estimators=200,
        max_samples=min(512, len(X_tr_s)),
        contamination=contamination,
        random_state=42,
        n_jobs=-1,
    )
    iforest.fit(X_tr_s)
    fit_ms_if = int((time.perf_counter() - t0) * 1000)

    t0 = time.perf_counter()
    # sklearn: -1 = outlier (anomaly), 1 = inlier (normal)
    if_raw = iforest.predict(X_te_s)
    predict_ms_if = int((time.perf_counter() - t0) * 1000)

    if_preds = [1 if v == -1 else 0 for v in if_raw]
    m = compute_metrics(y_true, if_preds)
    m["fit_ms"]     = fit_ms_if
    m["predict_ms"] = predict_ms_if
    result["iforest"] = m

    # ── One-Class SVM ─────────────────────────────────────────────────────────
    # Use a subsample for OC-SVM since it scales O(n²) with training set size.
    max_train = 5000
    if len(X_tr_s) > max_train:
        rng = np.random.default_rng(42)
        idx = rng.choice(len(X_tr_s), max_train, replace=False)
        X_tr_svm = X_tr_s[idx]
    else:
        X_tr_svm = X_tr_s

    t0 = time.perf_counter()
    ocsvm = OneClassSVM(kernel="rbf", gamma="scale", nu=contamination)
    ocsvm.fit(X_tr_svm)
    fit_ms_svm = int((time.perf_counter() - t0) * 1000)

    t0 = time.perf_counter()
    svm_raw = ocsvm.predict(X_te_s)
    predict_ms_svm = int((time.perf_counter() - t0) * 1000)

    svm_preds = [1 if v == -1 else 0 for v in svm_raw]
    m = compute_metrics(y_true, svm_preds)
    m["fit_ms"]     = fit_ms_svm
    m["predict_ms"] = predict_ms_svm
    result["ocsvm"] = m

    # ── CPU HBOS ──────────────────────────────────────────────────────────────
    # Per-feature histogram density; score = sum(-log(density)) across features.
    # Threshold is calibrated from TRAINING scores (all normal), not test scores.
    # Using test scores with 20-36% anomaly rates inflates the percentile into
    # deep anomaly territory, making the threshold uselessly high.
    # 99th percentile of training scores mirrors the FPGA calibration phase.
    _HBOS_TRAIN_PERCENTILE = 99.0

    n_bins = min(50, max(5, int(np.sqrt(len(X_tr_s)))))
    n_features = X_tr_s.shape[1]

    t0 = time.perf_counter()
    hbos_hists  = []
    hbos_edges  = []
    for j in range(n_features):
        counts, edges = np.histogram(X_tr_s[:, j], bins=n_bins)
        width   = edges[1] - edges[0]
        density = counts / (counts.sum() * width) if width > 0 else counts / max(counts.sum(), 1)
        density = np.maximum(density, 1e-10)
        hbos_hists.append(density)
        hbos_edges.append(edges)

    # Score training samples to calibrate the threshold on known-normal data
    train_scores = np.zeros(len(X_tr_s))
    for j in range(n_features):
        edges   = hbos_edges[j]
        density = hbos_hists[j]
        bin_idx = np.clip(np.searchsorted(edges[1:-1], X_tr_s[:, j]), 0, n_bins - 1)
        train_scores += -np.log(density[bin_idx])
    threshold = np.percentile(train_scores, _HBOS_TRAIN_PERCENTILE)
    fit_ms_hbos = int((time.perf_counter() - t0) * 1000)

    t0 = time.perf_counter()
    scores = np.zeros(len(X_te_s))
    for j in range(n_features):
        edges   = hbos_edges[j]
        density = hbos_hists[j]
        bin_idx = np.clip(np.searchsorted(edges[1:-1], X_te_s[:, j]), 0, n_bins - 1)
        scores += -np.log(density[bin_idx])

    hbos_preds  = [1 if s >= threshold else 0 for s in scores]
    predict_ms_hbos = int((time.perf_counter() - t0) * 1000)

    m = compute_metrics(y_true, hbos_preds)
    m["fit_ms"]     = fit_ms_hbos
    m["predict_ms"] = predict_ms_hbos
    result["hbos_cpu"] = m

    return result
