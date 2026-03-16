import csv
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import argparse
import random
import sys
import time
import os

# ══════════════════════════════════════════════════════════════
#  CONSTANTS
# ══════════════════════════════════════════════════════════════
SAVE_DIR           = "green_circle_pictures"
CENTER_TOL         = 150    # px — centerline tolerance
NEAR_LINE_RANGE    = 120    # px
CAPTURE_COOLDOWN   = 3.0    # seconds — global cooldown after each capture

# ── Averaging window before capture ──
MEASURE_DELAY      = 0.5    # seconds — collect radius samples for this duration before capturing

# ── Distance and size parameters ──
FOCAL_LENGTH_PX    = 600    # px  — measured at REFERENCE_WIDTH (640px)
CAMERA_DISTANCE_CM = 28     # cm  — camera to object distance
MIN_RADIUS_CM      = 5      # cm  — minimum radius (diameter >= 20cm)

# ── Camera resolution ──
CAPTURE_WIDTH      = 160    # px
CAPTURE_HEIGHT     = 120    # px
REFERENCE_WIDTH    = 640    # px  — width used when measuring FOCAL_LENGTH_PX

FOCAL_LENGTH_SCALED = FOCAL_LENGTH_PX * (CAPTURE_WIDTH / REFERENCE_WIDTH)

# ── PX_PER_CM calibrated: old=29.27, real=15.75cm, reported=14.71cm
#    new = 29.27 × (14.71 / 15.75) = 27.34
PX_PER_CM = 27.34   # px/cm — calibrated from real measurement

# MIN_CAPTURE_RADIUS stored as float
MIN_CAPTURE_RADIUS = MIN_RADIUS_CM * PX_PER_CM  # minimum radius in px (float)

# ── Process every N frames ──
PROCESS_EVERY_N_FRAMES = 4

# ══════════════════════════════════════════════════════════════
#  DETECTION GATE PARAMETERS  [v4.4]
# ══════════════════════════════════════════════════════════════
PLANTING_WARMUP_SEC = 5.0   # seconds to wait after startc before detecting
FALLBACK_WARMUP_SEC = 10.0  # seconds to wait after DONE:cabbage fallback (no startc)

# ══════════════════════════════════════════════════════════════
#  CABBAGE VALIDATION PARAMETERS  [v4.1]
# ══════════════════════════════════════════════════════════════
CABBAGE_SOLIDITY_MIN  = 0.85   # minimum solidity to pass
CABBAGE_FILL_MIN      = 0.40   # minimum green fill ratio to pass (0.0–1.0)
CABBAGE_FILTER_ENABLED = True

# ══════════════════════════════════════════════════════════════
#  SESSION MEASUREMENT LOG  [v4.4]
# ══════════════════════════════════════════════════════════════
_measurement_log: list = []
DEFAULT_LOG_FILE = os.path.join(SAVE_DIR, "measurement_log.csv")


def log_measurement(diameter_cm, log_file=DEFAULT_LOG_FILE, ros2_publisher=None):
    """
    Append diameter_cm to the in-memory session log.
    Publishes "LOG:[d1 d2 ...]" to /msg via ros2_publisher.
    Also appends a row to CSV — timestamp is the primary key.
    """
    _measurement_log.append(round(diameter_cm, 2))

    values_str = " ".join(f"{v:.2f}" for v in _measurement_log)
    log_msg    = f"LOG:[{values_str}]"

    if ros2_publisher is not None:
        from std_msgs.msg import String as RosString
        out = RosString()
        out.data = log_msg
        ros2_publisher.publish(out)

    ensure_save_dir()
    file_exists = os.path.isfile(log_file)
    with open(log_file, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(["timestamp", "diameter_cm"])
        writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), round(diameter_cm, 2)])


# ══════════════════════════════════════════════════════════════
#  A. GREEN EDGE MASK
# ══════════════════════════════════════════════════════════════
def green_edge_mask(bgr_img, thickness=6):
    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([35, 35, 35]), np.array([85, 255, 255]))
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    k7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k7)
    k_erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                        (thickness * 2 + 1, thickness * 2 + 1))
    inner = cv2.erode(mask, k_erode)
    edge  = cv2.subtract(mask, inner)
    return mask, edge


# ══════════════════════════════════════════════════════════════
#  B. CANNY ON GREEN REGION
# ══════════════════════════════════════════════════════════════
def green_canny(bgr_img, green_mask):
    gray  = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
    blur  = cv2.GaussianBlur(gray, (5, 5), 1.5)
    canny = cv2.Canny(blur, 30, 90)
    dilated = cv2.dilate(green_mask,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    return cv2.bitwise_and(canny, dilated)


# ══════════════════════════════════════════════════════════════
#  C. HOG ARC SCORE  [v4.3 — OpenCV only, no skimage]
# ══════════════════════════════════════════════════════════════
def arc_hog_score(gray_roi):
    h, w = gray_roi.shape
    if h < 16 or w < 16:
        return 0.0

    size = max(h, w)
    # must be divisible by 8 and >= 16 for HOGDescriptor
    size = max((size // 8) * 8, 16)

    resized = cv2.resize(gray_roi, (size, size))

    win_size     = (size, size)
    block_size   = (16, 16)
    block_stride = (8, 8)
    cell_size    = (8, 8)
    nbins        = 9

    try:
        hog_desc = cv2.HOGDescriptor(win_size, block_size, block_stride, cell_size, nbins)
        fd = hog_desc.compute(resized).flatten()
    except cv2.error:
        return 0.0

    if fd.sum() == 0:
        return 0.0

    n_cells = (size // 8) ** 2
    bins = fd[:n_cells * nbins].reshape(-1, nbins).sum(axis=0)
    if bins.sum() == 0:
        return 0.0
    bins = bins / bins.sum()
    entropy = -np.sum(bins * np.log(bins + 1e-9))
    return float(entropy / np.log(9.0))


# ══════════════════════════════════════════════════════════════
#  D. RANSAC CIRCLE FITTING
# ══════════════════════════════════════════════════════════════
def _circle_from_3pts(p1, p2, p3):
    ax, ay = p1; bx, by = p2; cx, cy = p3
    d = 2 * (ax*(by-cy) + bx*(cy-ay) + cx*(ay-by))
    if abs(d) < 1e-6:
        return None
    ux = ((ax**2+ay**2)*(by-cy) + (bx**2+by**2)*(cy-ay) + (cx**2+cy**2)*(ay-by)) / d
    uy = ((ax**2+ay**2)*(cx-bx) + (bx**2+by**2)*(ax-cx) + (cx**2+cy**2)*(bx-ax)) / d
    return ux, uy, np.hypot(ax-ux, ay-uy)


def ransac_circle(xs, ys, n_iter=300, tol=2.5, min_inliers=8):
    if len(xs) < 3:
        return None
    pts = list(zip(xs.tolist(), ys.tolist()))
    best, best_n = None, 0
    for _ in range(n_iter):
        s = random.sample(pts, 3)
        r = _circle_from_3pts(*s)
        if r is None or r[2] < 5:
            continue
        ccx, ccy, cr = r
        dist = np.abs(np.hypot(xs - ccx, ys - ccy) - cr)
        n_in = int((dist < tol).sum())
        if n_in > best_n:
            best_n, best = n_in, r
    if best is None or best_n < min_inliers:
        return None
    ccx, ccy, cr = best
    mask = np.abs(np.hypot(xs-ccx, ys-ccy) - cr) < tol
    xi, yi = xs[mask], ys[mask]
    if len(xi) >= 3:
        A = np.column_stack([xi, yi, np.ones(len(xi))])
        b = -(xi**2 + yi**2)
        try:
            D, E, F = np.linalg.lstsq(A, b, rcond=None)[0]
            cx_r, cy_r = -D/2, -E/2
            r_r = np.sqrt(max(cx_r**2 + cy_r**2 - F, 0))
            if r_r > 5:
                return float(cx_r), float(cy_r), float(r_r), int(mask.sum())
        except Exception:
            pass
    return float(best[0]), float(best[1]), float(best[2]), best_n


# ══════════════════════════════════════════════════════════════
#  E. NON-MAXIMUM SUPPRESSION
# ══════════════════════════════════════════════════════════════
def nms_circles(circles):
    if not circles:
        return []
    circles = sorted(circles, key=lambda c: c[3], reverse=True)
    kept = []
    for c in circles:
        cx, cy, r, _ = c
        duplicate = any(
            np.hypot(cx - k[0], cy - k[1]) < (r + k[2]) * 0.5
            for k in kept
        )
        if not duplicate:
            kept.append(c)
    return kept


# ══════════════════════════════════════════════════════════════
#  F. MAIN DETECTOR
# ══════════════════════════════════════════════════════════════
def detect_green_circles(bgr_img,
                          hog_thresh=0.50,
                          min_radius=15,
                          max_radius=500,
                          edge_thickness=6):
    img_h, img_w = bgr_img.shape[:2]
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

    green_mask, edge_mask = green_edge_mask(bgr_img, thickness=edge_thickness)
    canny_green = green_canny(bgr_img, green_mask)
    combined = cv2.bitwise_or(edge_mask, canny_green)

    contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_NONE)
    candidates = []

    for cnt in contours:
        if cv2.arcLength(cnt, False) < 30:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        pad = 4
        x1 = max(0, x-pad); y1 = max(0, y-pad)
        x2 = min(img_w, x+w+pad); y2 = min(img_h, y+h+pad)
        roi_gray = gray[y1:y2, x1:x2]
        score = arc_hog_score(roi_gray)

        if score < hog_thresh:
            continue

        edge_roi = combined[y1:y2, x1:x2]
        ys_e, xs_e = np.where(edge_roi > 0)
        if len(xs_e) < 8:
            continue

        result = ransac_circle(xs_e + x1, ys_e + y1)
        if result is None:
            continue

        ccx, ccy, cr, n_in = result

        if not (min_radius <= cr <= max_radius):
            continue

        candidates.append((ccx, ccy, cr, n_in))

    results = nms_circles(candidates)
    return results, green_mask, combined


# ══════════════════════════════════════════════════════════════
#  F2. CABBAGE VALIDATION FILTERS  [v4.1]
# ══════════════════════════════════════════════════════════════
def compute_solidity(bgr_img, cx, cy, r):
    img_h, img_w = bgr_img.shape[:2]
    ir = int(round(r))
    icx, icy = int(round(cx)), int(round(cy))

    pad = 4
    x1 = max(0, icx - ir - pad)
    y1 = max(0, icy - ir - pad)
    x2 = min(img_w, icx + ir + pad)
    y2 = min(img_h, icy + ir + pad)
    roi = bgr_img[y1:y2, x1:x2]
    if roi.size == 0:
        return 0.0

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    green_roi = cv2.inRange(hsv, np.array([35, 35, 35]), np.array([85, 255, 255]))

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    green_roi = cv2.morphologyEx(green_roi, cv2.MORPH_CLOSE, k)

    contours, _ = cv2.findContours(green_roi, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0

    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    if area < 10:
        return 0.0

    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    if hull_area < 1:
        return 0.0

    return float(area / hull_area)


def compute_green_fill(bgr_img, cx, cy, r):
    img_h, img_w = bgr_img.shape[:2]
    ir = int(round(r))
    icx, icy = int(round(cx)), int(round(cy))

    mask_circle = np.zeros((img_h, img_w), dtype=np.uint8)
    cv2.circle(mask_circle, (icx, icy), ir, 255, -1)

    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, np.array([35, 35, 35]), np.array([85, 255, 255]))

    total_inside = int(np.count_nonzero(mask_circle))
    if total_inside == 0:
        return 0.0

    green_inside = int(np.count_nonzero(cv2.bitwise_and(green_mask, mask_circle)))
    return float(green_inside / total_inside)


def is_cabbage(bgr_img, cx, cy, r, debug_print=False):
    if not CABBAGE_FILTER_ENABLED:
        return True, 1.0, 1.0

    solidity   = compute_solidity(bgr_img, cx, cy, r)
    fill_ratio = compute_green_fill(bgr_img, cx, cy, r)

    passed = (solidity >= CABBAGE_SOLIDITY_MIN) and (fill_ratio >= CABBAGE_FILL_MIN)

    if debug_print:
        status = "PASS" if passed else "FAIL"
        print(f"  [CABBAGE] solidity={solidity:.3f}(min={CABBAGE_SOLIDITY_MIN})  "
              f"fill={fill_ratio:.3f}(min={CABBAGE_FILL_MIN})  → {status}")

    return passed, solidity, fill_ratio


# ══════════════════════════════════════════════════════════════
#  G. UTILITIES
# ══════════════════════════════════════════════════════════════
def px_to_cm(px):
    return px / PX_PER_CM


def ensure_save_dir():
    if not os.path.exists(SAVE_DIR):
        os.makedirs(SAVE_DIR)
        print(f"[INFO] Created directory '{SAVE_DIR}'")


def find_nearest_to_centerline(circles, center_x):
    best, best_dist = None, float("inf")
    for c in circles:
        cx, cy, r, n_in = c
        dist = abs(cx - center_x)
        if dist < best_dist and dist <= NEAR_LINE_RANGE:
            best_dist = dist
            best = c
    return best, best_dist


def draw_centerline_overlay(frame, circles, center_x):
    h, w = frame.shape[:2]
    nearest, dist = find_nearest_to_centerline(circles, center_x)

    if nearest is not None and dist <= CENTER_TOL:
        ncx_t, ncy_t, nr_t, _ = nearest
        line_color = (0, 255, 80) if nr_t >= MIN_CAPTURE_RADIUS else (0, 80, 255)
    else:
        line_color = (0, 255, 255)

    cv2.line(frame, (center_x, 0), (center_x, h), line_color, 2)

    if nearest is not None:
        ncx, ncy, nr, _ = nearest
        passes = nr >= MIN_CAPTURE_RADIUS
        ring_color = (0, 220, 60) if passes else (0, 60, 220)

        icx, icy, ir = int(round(ncx)), int(round(ncy)), int(round(nr))
        cv2.circle(frame, (icx, icy), ir, ring_color, 3)
        cv2.circle(frame, (icx, icy), 4, ring_color, -1)

        lx = max(0, icx - ir)
        ly = max(15, icy - ir - 32)
        status_icon = "OK" if passes else "FAIL"
        d_cm = px_to_cm(nr) * 2
        cv2.putText(frame, f"d = {nr*2:.1f}px ({d_cm:.2f}cm) [{status_icon}]",
                    (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.6, ring_color, 2)
        cv2.putText(frame, f"min={MIN_RADIUS_CM*2}cm ({MIN_CAPTURE_RADIUS*2:.1f}px)",
                    (lx, ly+20), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180,180,180), 1)

    return frame


def draw_capture_annotation(frame, cx, cy, cr, center_x,
                             avg_diameter_cm=None, sample_count=None, std_r=None):
    out = frame.copy()
    h, w = out.shape[:2]

    icx, icy, icr = int(round(cx)), int(round(cy)), int(round(cr))

    cv2.line(out, (center_x, 0), (center_x, h), (0, 80, 255), 2)
    cv2.circle(out, (icx, icy), icr+2, (255, 255, 255), 4)
    cv2.circle(out, (icx, icy), icr,   (255, 160, 0),   2)
    cv2.line(out, (icx-icr, icy), (icx+icr, icy), (0, 255, 255), 1)
    ch = 10
    cv2.line(out, (icx-ch, icy), (icx+ch, icy), (0,255,255), 2)
    cv2.line(out, (icx, icy-ch), (icx, icy+ch), (0,255,255), 2)
    cv2.circle(out, (icx, icy), 4, (0,255,255), -1)

    bx1 = max(0, icx-icr-6);  by1 = max(0, icy-icr-6)
    bx2 = min(w-1, icx+icr+6); by2 = min(h-1, icy+icr+6)
    cv2.rectangle(out, (bx1, by1), (bx2, by2), (200,200,200), 1)

    label_y = max(20, by1-8)

    if avg_diameter_cm is not None:
        d_cm = avg_diameter_cm
        label_main = f"AVG d = {d_cm:.2f} cm"
    else:
        d_cm = px_to_cm(cr) * 2
        label_main = f"d = {cr*2:.1f}px ({d_cm:.2f} cm)"

    for color, thickness in [((255,255,255),3), ((255,160,0),2)]:
        cv2.putText(out, label_main,
                    (bx1, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, thickness)

    cv2.putText(out, f"center ({cx:.1f}, {cy:.1f})",
                (bx1, label_y+22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)

    if sample_count is not None and std_r is not None:
        std_cm = px_to_cm(std_r) * 2
        cv2.putText(out, f"samples={sample_count}  std={std_cm:.2f}cm",
                    (bx1, label_y+42), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (150, 220, 255), 1)

    cv2.putText(out, time.strftime("%Y-%m-%d  %H:%M:%S"),
                (8, h-8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180,180,180), 1)
    return out


# ══════════════════════════════════════════════════════════════
#  CAPTURE STATE  [v4.4]
# ══════════════════════════════════════════════════════════════
class CaptureState:
    """
    One capture per startc session — hard locked until DONE:cabbage.

    State machine:
        IDLE      ──(object on centerline + cabbage pass)──► MEASURING
        MEASURING ──(MEASURE_DELAY elapsed)──────────────► fires capture → LOCKED
        MEASURING ──(object leaves centerline)───────────► IDLE (no capture)
        LOCKED    ──(DONE:cabbage received)──────────────► IDLE (ready for next startc)
    """
    def __init__(self, publish_callback=None, log_publisher=None,
                 log_file=DEFAULT_LOG_FILE):
        self.last_capture_t   = 0.0
        self.must_exit_first  = False
        self.capture_done     = False   # True = LOCKED
        self.publish_callback = publish_callback
        self.log_publisher    = log_publisher
        self.log_file         = log_file
        self._reset_measuring()

    def _reset_measuring(self):
        """Reset measuring sub-state only — lock flags untouched."""
        self.measuring       = False
        self.measure_start   = 0.0
        self.radius_samples  = []
        self.cx_samples      = []
        self.cy_samples      = []

    def full_reset(self):
        """Full reset on DONE:cabbage — clears lock and measuring state."""
        self.capture_done    = False
        self.must_exit_first = False
        self._reset_measuring()


_capture_state = CaptureState()


def check_and_capture(frame, circles, center_x, cap_counter, state=None):
    global _capture_state
    if state is None:
        state = _capture_state

    now = time.time()

    # ── LOCKED: already captured this session ──
    if state.capture_done:
        return False

    nearest = None
    for c in circles:
        cx, cy, cr, _ = c
        if abs(cx - center_x) <= CENTER_TOL and cr >= MIN_CAPTURE_RADIUS:
            nearest = c
            break

    if nearest is None:
        if state.measuring:
            print(f"  [ABORT] Object left centerline — discarding {len(state.radius_samples)} samples")
            state._reset_measuring()
        if state.must_exit_first:
            print("  [READY] Object left centerline — ready for next capture")
            state.must_exit_first = False
        return False

    cx, cy, cr, _ = nearest

    if state.must_exit_first:
        return False

    if now - state.last_capture_t < CAPTURE_COOLDOWN:
        return False

    if not state.measuring:
        passed, solidity, fill_ratio = is_cabbage(frame, cx, cy, cr, debug_print=True)
        if not passed:
            return False

        state.measuring      = True
        state.measure_start  = now
        state.radius_samples = []
        state.cx_samples     = []
        state.cy_samples     = []
        print(f"  [CABBAGE OK] solidity={solidity:.3f}  fill={fill_ratio:.3f}")
        print(f"  [MEASURE] Collecting samples for {MEASURE_DELAY}s...")

    state.radius_samples.append(cr)
    state.cx_samples.append(cx)
    state.cy_samples.append(cy)

    elapsed = now - state.measure_start
    progress = min(elapsed / MEASURE_DELAY, 1.0)
    bar = int(progress * 20)
    print(f"\r  [MEASURE] {'█'*bar}{'░'*(20-bar)} {elapsed:.2f}/{MEASURE_DELAY:.2f}s  "
          f"samples={len(state.radius_samples)}", end="", flush=True)

    if elapsed >= MEASURE_DELAY:
        print()

        r_arr  = np.array(state.radius_samples)
        cx_arr = np.array(state.cx_samples)
        cy_arr = np.array(state.cy_samples)

        median_r  = float(np.median(r_arr))
        std_r_all = float(np.std(r_arr))
        mask = np.abs(r_arr - median_r) <= 2 * std_r_all

        r_clean  = r_arr[mask]
        cx_clean = cx_arr[mask]
        cy_clean = cy_arr[mask]

        n_total    = len(r_arr)
        n_filtered = int((~mask).sum())

        if len(r_clean) == 0:
            r_clean, cx_clean, cy_clean = r_arr, cx_arr, cy_arr
            n_filtered = 0

        avg_r  = float(np.mean(r_clean))
        avg_cx = float(np.mean(cx_clean))
        avg_cy = float(np.mean(cy_clean))
        std_r  = float(np.std(r_clean))
        d_cm   = px_to_cm(avg_r) * 2

        log_measurement(d_cm, log_file=state.log_file,
                        ros2_publisher=state.log_publisher)

        annotated = draw_capture_annotation(frame, avg_cx, avg_cy, avg_r, center_x,
                                             avg_diameter_cm=d_cm,
                                             sample_count=len(r_clean),
                                             std_r=std_r)
        ensure_save_dir()
        cap_counter[0] += 1
        ts = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(SAVE_DIR, f"circle_{ts}_{cap_counter[0]:04d}.png")
        cv2.imwrite(filename, annotated)

        last_t = now
        state._reset_measuring()
        state.last_capture_t  = last_t
        state.must_exit_first = True
        state.capture_done    = True   # LOCK until DONE:cabbage

        print(f"\n{'='*55}")
        print(f"  [CAPTURE] Cabbage measurement captured!")
        print(f"  File      : {filename}")
        print(f"  Avg center: ({avg_cx:.1f}, {avg_cy:.1f})")
        print(f"  Avg radius: {avg_r:.1f}px  std={std_r:.1f}px")
        print(f"  Diameter  : {d_cm:.2f} cm")
        print(f"  Samples   : {len(r_clean)} used / {n_total} total  ({n_filtered} outliers removed)")
        print(f"  Session LOCKED — send DONE:cabbage to /msg to unlock.")
        print(f"{'='*55}")

        if state.publish_callback is not None:
            state.publish_callback()

        return True

    return False


def draw_overlay(frame, circles, frame_num):
    for cx, cy, r, _ in circles:
        d_cm = px_to_cm(r) * 2
        passes = r >= MIN_CAPTURE_RADIUS

        icx, icy, ir = int(round(cx)), int(round(cy)), int(round(r))

        if passes:
            cv2.circle(frame, (icx, icy), ir, (0, 255, 0), 2)
            cv2.circle(frame, (icx, icy), 6, (0, 0, 255), -1)
            ch = 12
            cv2.line(frame, (icx-ch, icy), (icx+ch, icy), (0, 255, 255), 1)
            cv2.line(frame, (icx, icy-ch), (icx, icy+ch), (0, 255, 255), 1)
            label = f"d={r*2:.1f}px ({d_cm:.2f}cm)"
            cv2.putText(frame, label,
                        (max(0, icx-ir), max(15, icy-ir-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 2)
        else:
            cv2.circle(frame, (icx, icy), ir, (120, 120, 120), 1)
            cv2.putText(frame, f"d={d_cm:.2f}cm<{MIN_RADIUS_CM*2}cm",
                        (max(0, icx-ir), max(15, icy-ir-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 120, 120), 1)

    valid_count = sum(1 for c in circles if c[2] >= MIN_CAPTURE_RADIUS)
    info = (f"Circles(>={MIN_RADIUS_CM*2}cm diam): {valid_count}/{len(circles)}"
            f"  Frame:{frame_num}  Q=quit  M=mask")
    cv2.putText(frame, info, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    return frame


# ══════════════════════════════════════════════════════════════
#  I. ROS2 SUBSCRIBER MODE
# ══════════════════════════════════════════════════════════════
def run_ros2(hog_thresh=0.50, edge_thickness=6,
             min_radius=15, max_radius=500, show_mask=False):
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String
        import threading
    except ImportError:
        print("[ERROR] rclpy not found. Please source your ROS2 workspace first.")
        sys.exit(1)

    class CircleDetectorNode(Node):
        def __init__(self):
            super().__init__('circle_detector')

            self.hog_thresh     = hog_thresh
            self.edge_thickness = edge_thickness
            self.min_radius     = min_radius
            self.max_radius     = max_radius
            self.show_m         = show_mask

            self.frame_count   = 0
            self.last_circles  = []
            self.last_combined = None
            self.cap_counter   = [0]
            self.fps_v         = 0.0
            self.fps_t         = time.time()
            self.center_x      = CAPTURE_WIDTH // 2

            # Publishers
            self.feedback_pub = self.create_publisher(String, '/capture_feedback', 10)
            self.log_pub = self.create_publisher(String, '/msg', 10)

            self.cam_state = CaptureState(
                publish_callback=self._publish_success,
                log_publisher=self.log_pub,
            )

            self.disp_frame = None
            self.frame_lock = threading.Lock()

            # ── Detection gate state [v4.4] ──────────────────────────
            # WAITING   → WARMUP  (on startc  OR  DONE:cabbage fallback)
            # WARMUP    → DETECTING (after warmup_sec elapsed)
            # DETECTING → WAITING (on DONE:cabbage after capture, ready for next startc)
            self.gate_state        = "WAITING"
            self.planting2_recv_t  = 0.0
            self.warmup_sec        = PLANTING_WARMUP_SEC  # active warmup duration
            self.last_msg_received = "—"
            # Track whether startc was received for this session
            self._startc_received  = False

            self.sub = self.create_subscription(
                CompressedImage,
                '/camera_side/image_raw/compressed',
                self.image_callback,
                10
            )

            self.cmd_sub = self.create_subscription(
                String, '/msg', self.cmd_callback, 10)

            self.plant_feedback_sub = self.create_subscription(
                String, '/plant_feedback', self.plant_feedback_callback, 10)

            self.get_logger().info("=" * 55)
            self.get_logger().info(" Circle Detector Node — ROS2 Mode v4.4")
            self.get_logger().info(f" Cabbage filter = {'ON' if CABBAGE_FILTER_ENABLED else 'OFF'}")
            self.get_logger().info(f"   solidity >= {CABBAGE_SOLIDITY_MIN}  fill >= {CABBAGE_FILL_MIN}")
            self.get_logger().info(f" Detection gate = LOCKED (waiting for startc on /plant_feedback)")
            self.get_logger().info(f"                  OR DONE:cabbage on /msg as fallback")
            self.get_logger().info(f" Warmup delay   = {PLANTING_WARMUP_SEC}s after trigger")
            self.get_logger().info(f" PX_PER_CM      = {PX_PER_CM} (calibrated)")
            self.get_logger().info(" Topic : /camera_side/image_raw/compressed")
            self.get_logger().info(" Output: /capture_feedback  (String: SUCCESS)")
            self.get_logger().info(" Log   : /msg  (String: LOG:[d1 d2 ...]  published each capture)")
            self.get_logger().info(" Reset : /msg  (String: DONE:cabbage)")
            self.get_logger().info("          → if after capture: unlock + gate → WAITING")
            self.get_logger().info("          → if startc not received: fallback warmup → DETECTING")
            self.get_logger().info(" Unlock: /plant_feedback  (String: startc)")
            self.get_logger().info(" HOG   : OpenCV only (no skimage)")
            self.get_logger().info("=" * 55)

        def _publish_success(self):
            from std_msgs.msg import String
            msg = String()
            msg.data = "SUCCESS"
            self.feedback_pub.publish(msg)
            self.get_logger().info("[FEEDBACK] Published: SUCCESS -> /capture_feedback")

        def plant_feedback_callback(self, msg):
            cmd = msg.data.strip()
            self.last_msg_received = f"/plant_feedback: {cmd}"
            if cmd == "startc":
                if self.gate_state == "WAITING":
                    self.gate_state        = "WARMUP"
                    self.planting2_recv_t  = time.time()
                    self.warmup_sec        = PLANTING_WARMUP_SEC
                    self._startc_received  = True
                    self.get_logger().info(
                        f"[GATE] Received startc — warmup {PLANTING_WARMUP_SEC}s starts now")
                else:
                    self.get_logger().info(
                        f"[GATE] Received startc (already {self.gate_state}, ignored)")

        def cmd_callback(self, msg):
            cmd = msg.data.strip()
            if cmd.startswith("LOG:"):
                return
            self.last_msg_received = f"/msg: {cmd}"
            if cmd == "DONE:cabbage":
                was_detecting = self.gate_state in ("DETECTING", "WARMUP")

                if was_detecting:
                    # Normal post-capture reset — go back to WAITING for next startc
                    # Guard: only reset if capture actually happened (avoid duplicate spam)
                    if self.gate_state == "WARMUP" and not self.cam_state.capture_done:
                        # Still warming up, no capture yet — ignore duplicate
                        self.get_logger().warn(
                            "[RESET] DONE:cabbage ignored — warmup in progress, no capture yet")
                        return
                    self.cam_state.full_reset()
                    self.gate_state       = "WAITING"
                    self._startc_received = False
                    self.get_logger().info(
                        "[RESET] Received DONE:cabbage — capture unlocked, gate → WAITING")
                else:
                    # gate is WAITING (startc never arrived) — use DONE:cabbage as fallback
                    # Guard: if already in WARMUP from a previous fallback, ignore duplicates
                    if self.gate_state == "WARMUP":
                        remaining = max(0.0, self.warmup_sec - (time.time() - self.planting2_recv_t))
                        self.get_logger().warn(
                            f"[GATE] DONE:cabbage ignored — fallback warmup already running "
                            f"({remaining:.1f}s left)")
                        return
                    self.cam_state.full_reset()
                    self.gate_state        = "WARMUP"
                    self.planting2_recv_t  = time.time()
                    self.warmup_sec        = FALLBACK_WARMUP_SEC
                    self._startc_received  = False
                    self.get_logger().info(
                        f"[GATE] DONE:cabbage fallback — startc not received, "
                        f"warmup {FALLBACK_WARMUP_SEC}s starts now")

        def image_callback(self, msg):
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn("Failed to decode image frame")
                return

            h, w = frame.shape[:2]
            self.center_x = w // 2

            now = time.time()
            self.fps_v = 0.9*self.fps_v + 0.1*(1.0/max(now-self.fps_t, 1e-6))
            self.fps_t = now

            self.frame_count += 1

            # ── Detection gate check ───────────────────────────────────
            if self.gate_state == "WARMUP":
                if now - self.planting2_recv_t >= self.warmup_sec:
                    self.gate_state = "DETECTING"
                    self.get_logger().info("[GATE] Warmup complete — detection ACTIVE")

            gate_active = (self.gate_state == "DETECTING")

            if gate_active and self.frame_count % PROCESS_EVERY_N_FRAMES == 0:
                self.last_circles, _, self.last_combined = detect_green_circles(
                    frame,
                    hog_thresh=self.hog_thresh,
                    edge_thickness=self.edge_thickness,
                    min_radius=self.min_radius,
                    max_radius=self.max_radius,
                )
                check_and_capture(frame, self.last_circles, self.center_x,
                                  self.cap_counter, state=self.cam_state)
            elif not gate_active:
                self.last_circles = []

            disp = draw_overlay(frame.copy(), self.last_circles, self.frame_count)
            draw_centerline_overlay(disp, self.last_circles, self.center_x)

            is_processed = gate_active and (self.frame_count % PROCESS_EVERY_N_FRAMES == 0)
            if not gate_active:
                proc_label, proc_color = "GATE-LOCKED", (0, 80, 255)
            elif self.cam_state.capture_done:
                proc_label, proc_color = "CAP-LOCKED", (0, 140, 255)
            elif is_processed:
                proc_label, proc_color = "DETECT", (0, 255, 150)
            else:
                proc_label = f"skip ({self.frame_count % PROCESS_EVERY_N_FRAMES}/{PROCESS_EVERY_N_FRAMES})"
                proc_color = (150, 150, 150)

            cv2.putText(disp, proc_label,
                        (disp.shape[1]-130, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, proc_color, 1)
            cv2.putText(disp, f"FPS:{self.fps_v:.1f}",
                        (disp.shape[1]-80, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
            cv2.putText(disp, f"Cap:{self.cap_counter[0]}",
                        (disp.shape[1]-80, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100,255,150), 1)

            if CABBAGE_FILTER_ENABLED:
                cv2.putText(disp, "CAB-FILTER:ON",
                            (8, disp.shape[0]-20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 100), 1)

            if self.show_m and self.last_combined is not None:
                mh, mw = self.last_combined.shape
                sm = cv2.resize(self.last_combined, (mw//4, mh//4))
                sm_bgr = cv2.cvtColor(sm, cv2.COLOR_GRAY2BGR)
                dh, dw = disp.shape[:2]
                sh, sw = sm_bgr.shape[:2]
                disp[dh-sh:dh, dw-sw:dw] = sm_bgr
                cv2.putText(disp, "Edge", (dw-sw+2, dh-sh+14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)

            # ── Status bar ────────────────────────────────────────────
            if self.gate_state == "WAITING":
                gate_label = "GATE: WAITING (startc or DONE:cabbage)"
                gate_color = (0, 80, 255)
            elif self.gate_state == "WARMUP":
                remaining  = max(0.0, self.warmup_sec - (now - self.planting2_recv_t))
                src_label  = "startc" if self._startc_received else "DONE:cabbage fallback"
                gate_label = f"GATE: WARMUP [{src_label}] {remaining:.1f}s"
                gate_color = (0, 200, 255)
            elif self.cam_state.capture_done:
                gate_label = "GATE: DETECTING  |  CAP: LOCKED (wait DONE:cabbage)"
                gate_color = (0, 140, 255)
            else:
                gate_label = "GATE: DETECTING"
                gate_color = (0, 220, 100)

            fb_label  = f"/capture_feedback: {'SUCCESS' if self.cam_state.capture_done else '—'}"
            msg_label = f"last msg: {self.last_msg_received}"

            cv2.rectangle(disp,
                          (0, disp.shape[0] - 36),
                          (disp.shape[1], disp.shape[0]),
                          (20, 20, 20), -1)
            cv2.putText(disp, gate_label,
                        (6, disp.shape[0] - 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, gate_color, 1)
            cv2.putText(disp, fb_label,
                        (disp.shape[1] - 210, disp.shape[0] - 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 200, 255), 1)
            cv2.putText(disp, msg_label,
                        (6, disp.shape[0] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)

            with self.frame_lock:
                self.disp_frame = disp

    rclpy.init()
    node = CircleDetectorNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    WIN_NAME = "Green Circle Detector v4.4 — ROS2"
    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)

    try:
        while rclpy.ok():
            with node.frame_lock:
                frame = node.disp_frame
            if frame is not None:
                cv2.imshow(WIN_NAME, frame)
            key = cv2.waitKey(16) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('m'), ord('M')):
                node.show_m = not node.show_m

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        cv2.destroyAllWindows()
        print(f"Node stopped. Total captures: {node.cap_counter[0]} -> ./{SAVE_DIR}/")


# ══════════════════════════════════════════════════════════════
#  J. ENTRY POINT
# ══════════════════════════════════════════════════════════════
def main():
    global FOCAL_LENGTH_PX, CAMERA_DISTANCE_CM, MIN_RADIUS_CM
    global PX_PER_CM, MIN_CAPTURE_RADIUS, PROCESS_EVERY_N_FRAMES
    global CAPTURE_WIDTH, CAPTURE_HEIGHT, FOCAL_LENGTH_SCALED, MEASURE_DELAY, CENTER_TOL
    global CABBAGE_SOLIDITY_MIN, CABBAGE_FILL_MIN, CABBAGE_FILTER_ENABLED

    parser = argparse.ArgumentParser(
        description="Green Circle Detector v4.4 — ROS2 mode only (OpenCV HOG, calibrated PX_PER_CM)"
    )
    parser.add_argument("--threshold",    type=float, default=0.50)
    parser.add_argument("--thickness",    type=int,   default=6)
    parser.add_argument("--min-r",        type=int,   default=15)
    parser.add_argument("--max-r",        type=int,   default=500)
    parser.add_argument("--mask",         action="store_true")
    parser.add_argument("--focal",        type=float, default=FOCAL_LENGTH_PX)
    parser.add_argument("--distance",     type=float, default=CAMERA_DISTANCE_CM)
    parser.add_argument("--min-cm",       type=float, default=MIN_RADIUS_CM)
    parser.add_argument("--every",        type=int,   default=PROCESS_EVERY_N_FRAMES)
    parser.add_argument("--width",        type=int,   default=CAPTURE_WIDTH)
    parser.add_argument("--height",       type=int,   default=CAPTURE_HEIGHT)
    parser.add_argument("--px-per-cm",    type=float, default=PX_PER_CM)
    parser.add_argument("--measure-delay",type=float, default=MEASURE_DELAY)
    parser.add_argument("--center-tol",   type=int,   default=CENTER_TOL)
    parser.add_argument("--no-cabbage-filter", action="store_true")
    parser.add_argument("--solidity-min", type=float, default=CABBAGE_SOLIDITY_MIN)
    parser.add_argument("--fill-min",     type=float, default=CABBAGE_FILL_MIN)

    args = parser.parse_args()

    random.seed(0)

    FOCAL_LENGTH_PX        = args.focal
    CAMERA_DISTANCE_CM     = args.distance
    MIN_RADIUS_CM          = args.min_cm
    CAPTURE_WIDTH          = args.width
    CAPTURE_HEIGHT         = args.height
    PROCESS_EVERY_N_FRAMES = args.every
    FOCAL_LENGTH_SCALED    = FOCAL_LENGTH_PX * (CAPTURE_WIDTH / REFERENCE_WIDTH)
    PX_PER_CM              = args.px_per_cm
    MIN_CAPTURE_RADIUS     = MIN_RADIUS_CM * PX_PER_CM
    MEASURE_DELAY          = args.measure_delay
    CENTER_TOL             = args.center_tol
    CABBAGE_FILTER_ENABLED = not args.no_cabbage_filter
    CABBAGE_SOLIDITY_MIN   = args.solidity_min
    CABBAGE_FILL_MIN       = args.fill_min

    run_ros2(
        hog_thresh=args.threshold,
        edge_thickness=args.thickness,
        min_radius=args.min_r,
        max_radius=args.max_r,
        show_mask=args.mask,
    )


if __name__ == "__main__":
    main()