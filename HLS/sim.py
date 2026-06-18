import csv

NR_SENSORS = 4
NR_BINS = 2048
NR_DELTA_BINS = 256
SENSOR_WEIGHTS = [50, 93, 58, 55]
SPIKE_PENALTY = 5632

def clz(x):
    if x == 0: return 32
    return 31 - x.bit_length() + 1

def aprox_log2(x):
    if x == 0: return 0
    lz = clz(x)
    msb = 31 - lz
    if msb >= 4: frac_bits = (x >> (msb - 4)) & 0xF
    else: frac_bits = (x << (4 - msb)) & 0xF
    lut = [0, 22, 43, 63, 82, 101, 119, 136, 153, 170, 186, 202, 217, 232, 246, 260]
    return (msb << 8) + lut[frac_bits]

def log_linear_addr(v, center):
    diff = v - center if v >= center else center - v
    sign = 0 if v >= center else 1
    if diff == 0: diff = 1
    msb = 31 - clz(diff)
    exp = min(msb, 31)
    mantissa = (diff >> (msb - 5)) & 31 if msb >= 5 else (diff << (5 - msb)) & 31
    return (sign << 10) | (exp << 5) | mantissa

def delta_log_linear_addr(diff):
    if diff == 0: diff = 1
    msb = 31 - clz(diff)
    exp = min(msb, 31)
    mantissa = (diff >> (msb - 3)) & 7 if msb >= 3 else (diff << (3 - msb)) & 7
    return (exp << 3) | mantissa

class AddressEngine:
    def __init__(self):
        self.center = [0]*4
        self.history = [[0]*5 for _ in range(4)]
        self.initialized = [False]*4
    def process(self, data, opcode):
        out_addr = [0]*4
        out_d_addr = [0]*4
        for i in range(4):
            v = data[i]
            if not self.initialized[i]:
                self.center[i] = v
                for h in range(5): self.history[i][h] = v
                self.initialized[i] = True
            out_addr[i] = log_linear_addr(v, self.center[i])
            # Golden / address_engine.cpp: delta vs oldest in 5-tap window.
            delta = v - self.history[i][0] if v > self.history[i][0] else self.history[i][0] - v
            out_d_addr[i] = delta_log_linear_addr(delta)
        if opcode != 3:  # OP_DUMP
            for i in range(4):
                for h in range(4):
                    self.history[i][h] = self.history[i][h + 1]
                self.history[i][4] = data[i]
        return out_addr, out_d_addr

ae = AddressEngine()
hist = [[0]*NR_BINS for _ in range(4)]
d_hist = [[0]*NR_DELTA_BINS for _ in range(4)]
score_hist = [0]*2048

train_count = 0
calib_count = 0
phase = 0

rows = []
with open("hls_test_stream.csv") as f:
    for row in csv.reader(f): rows.append(row)

for idx, row in enumerate(rows):
    data = [int(x) for x in row[:4]]
    is_clean = (int(row[4]) == 0)
    addr, d_addr = ae.process(data, 0)
    if is_clean and phase == 0:
        train_count += 1
        for i in range(4):
            hist[i][addr[i]] += 1
            d_hist[i][d_addr[i]] += 1
    phase = (phase + 1) % 5

log2_denom = aprox_log2(train_count + 2048)
for i in range(4):
    for j in range(NR_BINS):
        log2_num = aprox_log2(hist[i][j] + 1)
        hist[i][j] = log2_denom - log2_num if log2_denom > log2_num else 0

target = train_count - (train_count >> 10)
delta_th = [255] * 4
for i in range(4):
    cum = 0
    for d in range(NR_DELTA_BINS):
        cum += d_hist[i][d]
        if cum >= target and delta_th[i] == 255:
            delta_th[i] = d

for idx, row in enumerate(rows):
    data = [int(x) for x in row[:4]]
    is_clean = (int(row[4]) == 0)
    addr, d_addr = ae.process(data, 1)
    if is_clean and phase == 1:
        calib_count += 1
        total = sum(
            ((hist[i][addr[i]] + (SPIKE_PENALTY if d_addr[i] > delta_th[i] else 0)) * SENSOR_WEIGHTS[i]) >> 8
            for i in range(4)
        )
        if (total >> 4) < 2048:
            score_hist[total >> 4] += 1
    phase = (phase + 1) % 5

ae.process([0, 0, 0, 0], 3)

# Golden verify_architecture_5_5.py / hbos_top.cpp: exclude ~top 1/512 (>> 9).
target_calib = calib_count - (calib_count >> 9)
global_threshold = 32767
cum = 0
for j in range(2048):
    cum += score_hist[j]
    if cum >= target_calib and global_threshold == 32767: global_threshold = j << 4
phase = (phase + 1) % 5

for _ in range(1):
    ae.process([0,0,0,0], 1)
    phase = (phase + 1) % 5

tp = fp = fn = tn = 0
for idx, row in enumerate(rows):
    data = [int(x) for x in row[:4]]
    actual = int(row[4]) != 0
    addr, d_addr = ae.process(data, 2)
    total = sum(
        ((hist[i][addr[i]] + (SPIKE_PENALTY if d_addr[i] > delta_th[i] else 0)) * SENSOR_WEIGHTS[i]) >> 8
        for i in range(4)
    )
    pred = total >= global_threshold
    if pred and actual:
        tp += 1
    elif pred and not actual:
        fp += 1
    elif not pred and actual:
        fn += 1
    else:
        tn += 1
    phase = (phase + 1) % 5

print(f"Anomalies flagged: {tp + fp}")
print(f"TP={tp}  FP={fp}  FN={fn}  TN={tn}")
if tp + fp:
    print(f"Precision: {tp / (tp + fp):.4f}  Recall: {tp / (tp + fn):.4f}")
print("\n==================================================")
print("  [EXPECTED MATHEMATICAL THRESHOLDS]")
print(f"  Train Count:      {train_count}")
print(f"  Calib Count:      {calib_count}")
print(f"  Global Threshold: {global_threshold}")
for i in range(4):
    print(f"  Sensor {i} Delta Threshold (delta_th[{i}]): {delta_th[i]}")
print("==================================================\n")
