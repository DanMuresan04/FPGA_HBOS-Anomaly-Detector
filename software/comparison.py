"""
Off-chip comparison of the FPGA HBOS result against CPU baselines:
  - CPU HBOS      — the classic textbook HBOS: per-feature histogram densities
    built over the whole test set, every sample scored. Fully unsupervised (no
    clean data), and its score cut is TUNED for best F1.
  - Isolation Forest — a true unsupervised outlier detector, fit directly on the
    test stream (no clean data); its score cut is TUNED for best F1.
  - One-Class SVM — a novelty detector trained on clean data (its intended
    regime); its RBF gamma and score cut are TUNED for best F1.
  - supervised k-NN  — trained on labelled anomalies and hyper-parameter-tuned
    by cross-validation; reported as N/A when the training set has no anomaly
    labels.

The three unsupervised/novelty CPU baselines have no spike-penalty term like the
FPGA HBOS+, so each is shown at its BEST-CASE (label-optimal) operating point:
the score cut is chosen to maximise F1 against the ground truth. That makes them
optimistic upper bounds — the fairest number each can reach — against the FPGA's
actual deployed result. (k-NN is likewise an upper bound, but supervised.)

Every model is scored against the same test rows as the FPGA, so they share
identical ground-truth labels. sklearn is required; if it is missing every
result is returned as None and the UI shows a "sklearn not installed" notice
rather than crashing.
"""

from __future__ import annotations

import time
from typing import Optional

try:
    import numpy as np
    from sklearn.ensemble import IsolationForest
    from sklearn.preprocessing import StandardScaler
    from sklearn.svm import OneClassSVM
    from sklearn.neighbors import KNeighborsClassifier
    from sklearn.model_selection import GridSearchCV

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


def _best_f1_cut(scores, y_true) -> tuple:
    """
    Sweep the anomaly-score cut that maximises F1 for the anomaly class.

    `scores` : array-like, higher = more anomalous.
    `y_true` : 0/1 labels (1 = anomaly).
    Returns (preds:list[int], best_f1:float, flagged_frac:float).

    This is a cheap O(n log n) sweep over every possible cut — equivalent to
    fine-tuning the threshold percentile. It picks the cut using the ground-truth
    labels, so the resulting F1 is a BEST-CASE (label-optimal) operating point:
    an upper bound on what the detector can achieve, reported as such. It gives
    each plain CPU baseline its fairest shot against the FPGA HBOS+, which enjoys
    a spike-penalty term the textbook models don't have — without hand-picking a
    magic threshold.
    """
    import numpy as np
    s = np.asarray(scores, dtype=float)
    y = np.asarray(y_true, dtype=int)
    n = len(y)
    P = int(y.sum())
    if n == 0 or P == 0 or P == n:
        return [0] * n, 0.0, 0.0
    order = np.argsort(-s, kind="mergesort")      # most-anomalous first, stable
    ys    = y[order]
    tp    = np.cumsum(ys)                          # TP when we flag the top-k
    k     = np.arange(1, n + 1)
    precision = tp / k
    recall    = tp / P
    denom     = precision + recall
    f1    = np.where(denom > 0, 2 * precision * recall / denom, 0.0)
    best  = int(np.argmax(f1))
    preds = np.zeros(n, dtype=int)
    preds[order[: best + 1]] = 1
    return preds.tolist(), float(f1[best]), (best + 1) / n


# ── main entry point ──────────────────────────────────────────────────────────

def run_comparison(
    train_rows:      list,
    test_rows:       list,
    hbos_preds:      list[int],
    calib_percentile: Optional[float] = None,
) -> dict:
    """
    Run the full comparison of the FPGA against the CPU baselines.

    Parameters
    ----------
    train_rows       : CSV rows from the training phase (same format as test_rows)
    test_rows        : CSV rows from the test/detect phase
    hbos_preds       : per-sample 0/1 predictions already produced by the FPGA
    calib_percentile : the percentile the FPGA calibrated its global threshold
                       at (100·(1-2**-shift) for the live calib knob) — i.e. the
                       fraction of data considered normal. Classic CPU HBOS sorts
                       the test-set scores and flags the top (100 - percentile)%,
                       using the SAME cut strictness as the FPGA. Defaults to 99.0
                       (the historical value) when not supplied.

    Returns
    -------
    dict with keys "hbos", "hbos_cpu", "iforest", "ocsvm", "knn",
    "dataset", "sklearn_available"

      Each algorithm dict has all keys from compute_metrics() plus "fit_ms"
      and "predict_ms" (only for CPU algorithms). "knn" additionally carries a
      "note" string when it is degenerate/unavailable (e.g. the training set
      has no labelled anomalies, so a supervised model has nothing to learn).
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
        result["iforest"]  = None
        result["ocsvm"]    = None
        result["hbos_cpu"] = None
        result["knn"]      = None
        return result

    import numpy as np

    # Feature matrix for training (anomaly-free rows only → unsupervised)
    X_train_all, y_train = _rows_to_xy(train_rows)
    X_test,  _           = _rows_to_xy(test_rows)

    if len(X_train_all) == 0 or len(X_test) == 0:
        result["iforest"]  = None
        result["ocsvm"]    = None
        result["hbos_cpu"] = None
        result["knn"]      = None
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

    # ── Isolation Forest (unsupervised, in-place on the test set) ─────────────
    # iForest is a genuine unsupervised outlier detector: it is fit on the very
    # data being analysed — the test stream — and finds the outliers within it.
    # It never sees clean data. We DON'T pin contamination: fixing it to the true
    # anomaly rate forces #flagged == #anomalies, which makes FP == FN identically
    # (both = flagged − TP) and collapses precision = recall = F1 — an artefact of
    # the cut, not the model. Instead we take the raw anomaly scores and sweep the
    # cut for best F1 (its fairest operating point).
    if_scaler = StandardScaler()
    X_if = if_scaler.fit_transform(X_test)

    t0 = time.perf_counter()
    iforest = IsolationForest(
        n_estimators=200,
        max_samples=min(512, len(X_if)),
        random_state=42,
        # n_jobs=1 (not -1): joblib's subprocess worker pool can stall when
        # spawned from the GUI's background thread in a detached session, which
        # left the comparison stuck on "Computing…". Single-threaded is still
        # sub-second here.
        n_jobs=1,
    )
    iforest.fit(X_if)
    # decision_function: higher = more normal, so negate for an anomaly score.
    if_scores = -iforest.decision_function(X_if)
    if_preds, _if_f1, if_frac = _best_f1_cut(if_scores, y_true)
    fit_ms_if = int((time.perf_counter() - t0) * 1000)

    m = compute_metrics(y_true, if_preds)
    m["fit_ms"]      = fit_ms_if
    m["predict_ms"]  = None
    m["tuned"]       = True
    m["flagged_pct"] = 100 * if_frac
    result["iforest"] = m

    # ── One-Class SVM (novelty detector, tuned) ───────────────────────────────
    # Trained on clean data (its intended regime). It IS tunable: the RBF width
    # `gamma` sets how tightly the boundary wraps the normal cloud, and the score
    # cut sets the operating point. We grid a few gammas, and for each sweep the
    # decision-function cut for best F1, keeping the best combination. Use a
    # subsample since OC-SVM scales O(n²) with training set size.
    max_train = 5000
    if len(X_tr_s) > max_train:
        rng = np.random.default_rng(42)
        idx = rng.choice(len(X_tr_s), max_train, replace=False)
        X_tr_svm = X_tr_s[idx]
    else:
        X_tr_svm = X_tr_s

    t0 = time.perf_counter()
    best_svm = None   # (preds, f1, frac, gamma)
    for gamma in ("scale", 0.1, 1.0):
        svm = OneClassSVM(kernel="rbf", gamma=gamma, nu=contamination)
        svm.fit(X_tr_svm)
        # decision_function: higher = more normal, so negate for an anomaly score.
        svm_scores = -svm.decision_function(X_te_s)
        preds, f1v, frac = _best_f1_cut(svm_scores, y_true)
        if best_svm is None or f1v > best_svm[1]:
            best_svm = (preds, f1v, frac, gamma)
    fit_ms_svm = int((time.perf_counter() - t0) * 1000)

    svm_preds, _svm_f1, svm_frac, svm_gamma = best_svm
    m = compute_metrics(y_true, svm_preds)
    m["fit_ms"]      = fit_ms_svm
    m["predict_ms"]  = None
    m["tuned"]       = True
    m["gamma"]       = svm_gamma
    m["flagged_pct"] = 100 * svm_frac
    result["ocsvm"] = m

    # ── CPU HBOS (classic textbook HBOS, unsupervised, in-place, tuned) ───────
    # Standard HBOS: build per-feature histogram densities over the whole set
    # being analysed (the test stream) and score every sample as sum(-log
    # density). The plain CPU HBOS has no spike-penalty term like the FPGA
    # HBOS+, so pinning it to the FPGA's exact percentile handicaps it. Instead
    # we sweep the score cut for best F1 — cheap here — giving it its fairest
    # operating point. (calib_percentile is kept only as a labelled reference of
    # what the FPGA cut would have been.)
    n_bins     = min(50, max(5, int(np.sqrt(len(X_test)))))
    n_features = X_test.shape[1]

    hbos_scaler = StandardScaler()
    X_hb = hbos_scaler.fit_transform(X_test)

    t0 = time.perf_counter()
    scores = np.zeros(len(X_hb))
    for j in range(n_features):
        counts, edges = np.histogram(X_hb[:, j], bins=n_bins)
        width   = edges[1] - edges[0]
        density = counts / (counts.sum() * width) if width > 0 else counts / max(counts.sum(), 1)
        density = np.maximum(density, 1e-10)
        bin_idx = np.clip(np.searchsorted(edges[1:-1], X_hb[:, j]), 0, n_bins - 1)
        scores += -np.log(density[bin_idx])

    hbos_preds, _hbos_f1, hbos_frac = _best_f1_cut(scores, y_true)
    fit_ms_hbos = int((time.perf_counter() - t0) * 1000)

    m = compute_metrics(y_true, hbos_preds)
    m["fit_ms"]      = fit_ms_hbos
    m["predict_ms"]  = None
    m["tuned"]       = True
    m["flagged_pct"] = 100 * hbos_frac
    if calib_percentile is not None:
        m["fpga_pct"] = float(calib_percentile)
    result["hbos_cpu"] = m

    # ── Supervised k-NN ───────────────────────────────────────────────────────
    # Unlike the four detectors above (all trained on normal data only), k-NN is
    # a SUPERVISED classifier: it needs labelled anomalies in the training set.
    # It's included as the "upper bound" reference — how well a model that has
    # actually seen anomalies does, versus the unsupervised HBOS the FPGA runs.
    # If the training set is single-class (no labelled anomalies — e.g. the
    # synthetic clean-only set) there is nothing to learn, so we report a note
    # instead of a misleading all-normal result.
    n_anom_train = len(y_train) - len(clean_mask)
    if n_anom_train == 0 or len(clean_mask) == 0:
        result["knn"] = {
            "note": "needs labelled anomalies in training data "
                    f"({n_anom_train} present)",
        }
    else:
        # Scale on the full (labelled) training set — a supervised model should
        # see the whole distribution, not just the normal class.
        knn_scaler = StandardScaler()
        Xk_tr = knn_scaler.fit_transform(X_train_all)
        Xk_te = knn_scaler.transform(X_test)

        # Fine-tune n_neighbors and the neighbour weighting by cross-validation
        # on the TRAINING set only — no test rows are involved, so the "upper
        # bound" stays honest. We score on F1 of the anomaly class (pos_label=1),
        # the metric the comparison actually cares about. Folds are capped by the
        # rarer class count so no fold can miss a class.
        max_k  = max(1, min(31, len(Xk_tr) - 1))
        k_grid = [k for k in (1, 3, 5, 7, 9, 11, 15, 21, 31) if k <= max_k] or [1]
        n_splits = int(min(5, n_anom_train, len(clean_mask)))

        t0 = time.perf_counter()
        if n_splits >= 2:
            grid = GridSearchCV(
                KNeighborsClassifier(n_jobs=1),
                {"n_neighbors": k_grid, "weights": ["uniform", "distance"]},
                scoring="f1", cv=n_splits, n_jobs=1,
            )
            grid.fit(Xk_tr, y_train)
            knn    = grid.best_estimator_
            best_k = grid.best_params_["n_neighbors"]
            best_w = grid.best_params_["weights"]
        else:
            # Too few anomalies to cross-validate — fall back to the √n heuristic.
            best_k = max(1, min(15, int(np.sqrt(len(Xk_tr)))))
            best_w = "distance"
            knn    = KNeighborsClassifier(n_neighbors=best_k, weights=best_w, n_jobs=1)
            knn.fit(Xk_tr, y_train)
        fit_ms_knn = int((time.perf_counter() - t0) * 1000)

        t0 = time.perf_counter()
        knn_raw = knn.predict(Xk_te)
        predict_ms_knn = int((time.perf_counter() - t0) * 1000)

        knn_preds = [1 if int(v) == 1 else 0 for v in knn_raw]
        m = compute_metrics(y_true, knn_preds)
        m["fit_ms"]     = fit_ms_knn
        m["predict_ms"] = predict_ms_knn
        m["k"]          = best_k
        m["weights"]    = best_w
        result["knn"] = m

    return result
