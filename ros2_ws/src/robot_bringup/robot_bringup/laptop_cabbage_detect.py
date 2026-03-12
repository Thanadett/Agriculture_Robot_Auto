"""
Green Circle Detector v4 — Hollow circle (green edge only)
===========================================================
[Updated] Process every 5 frames + show only circles with diameter >= 20 cm
          Camera distance: 28 cm
          - PX_PER_CM = 21.62 (measured from real object diameter=15.91cm, radius=172px)
          - Display diameter (d) instead of radius (r)
          - Use float throughout all calculations
          - Capture image only when object diameter >= MIN_RADIUS_CM*2

[v4 Change] Added ROS2 subscriber mode (--ros2 flag)
            Receives compressed image from /camera_side/image_raw/compressed topic
            instead of opening camera directly — suitable for running detector on laptop
            while Pi5 handles camera publishing.
"""

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from skimage.feature import hog
import argparse
import random
import sys
import time
import os

# ══════════════════════════════════════════════════════════════
#  CONSTANTS
# ══════════════════════════════════════════════════════════════
SAVE_DIR           = "green_circle_pictures"
CENTER_TOL         = 50     # px — centerline tolerance
NEAR_LINE_RANGE    = 120    # px
CAPTURE_COOLDOWN   = 3.0    # seconds — global cooldown after each capture

# ── Averaging window before capture ──
MEASURE_DELAY      = 0.5    # seconds — collect radius samples for this duration before capturing
                             # once object enters centerline, wait MEASURE_DELAY seconds,
                             # then average all collected radius values and capture

# ── Distance and size parameters ──
FOCAL_LENGTH_PX    = 600    # px  — measured at REFERENCE_WIDTH (640px)
CAMERA_DISTANCE_CM = 28     # cm  — camera to object distance
MIN_RADIUS_CM      = 5      # cm  — minimum radius (diameter >= 20cm)

# ── Camera resolution ──
CAPTURE_WIDTH      = 160    # px
CAPTURE_HEIGHT     = 120    # px
REFERENCE_WIDTH    = 640    # px  — width used when measuring FOCAL_LENGTH_PX

FOCAL_LENGTH_SCALED = FOCAL_LENGTH_PX * (CAPTURE_WIDTH / REFERENCE_WIDTH)

# ── PX_PER_CM calculated from real measurement ──
# Real object diameter=15.91cm → radius=7.955cm, measured radius=172px
# PX_PER_CM = 172 / 7.955 = 21.62
PX_PER_CM = 29.27   # px/cm — hardcoded from real measurement

# MIN_CAPTURE_RADIUS stored as float
MIN_CAPTURE_RADIUS = MIN_RADIUS_CM * PX_PER_CM  # minimum radius in px (float)

# ── Process every N frames ──
PROCESS_EVERY_N_FRAMES = 4


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
#  C. HOG ARC SCORE
# ══════════════════════════════════════════════════════════════
def arc_hog_score(gray_roi):
    h, w = gray_roi.shape
    if h < 16 or w < 16:
        return 0.0
    size = max(h, w)
    size = size + (size % 2)
    resized = cv2.resize(gray_roi, (size, size))
    fd = hog(resized, orientations=9, pixels_per_cell=(8, 8),
             cells_per_block=(1, 1), visualize=False, channel_axis=None)
    n_cells = (size // 8) ** 2
    bins = fd[:n_cells * 9].reshape(-1, 9).sum(axis=0)
    if bins.sum() == 0:
        return 0.0
    bins /= bins.sum()
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
                # Return as float, no int rounding
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
                          edge_thickness=6,
                          debug=False):
    img_h, img_w = bgr_img.shape[:2]
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

    green_mask, edge_mask = green_edge_mask(bgr_img, thickness=edge_thickness)
    canny_green = green_canny(bgr_img, green_mask)
    combined = cv2.bitwise_or(edge_mask, canny_green)

    contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_NONE)
    candidates = []
    debug_info = []

    for cnt in contours:
        if cv2.arcLength(cnt, False) < 30:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        pad = 4
        x1 = max(0, x-pad); y1 = max(0, y-pad)
        x2 = min(img_w, x+w+pad); y2 = min(img_h, y+h+pad)
        roi_gray = gray[y1:y2, x1:x2]
        score = arc_hog_score(roi_gray)

        if debug:
            debug_info.append(dict(cnt=cnt, score=score, passed=score >= hog_thresh))
        if score < hog_thresh:
            continue

        edge_roi = combined[y1:y2, x1:x2]
        ys_e, xs_e = np.where(edge_roi > 0)
        if len(xs_e) < 8:
            continue

        result = ransac_circle(xs_e + x1, ys_e + y1)
        if result is None:
            continue

        # Keep as float throughout
        ccx, ccy, cr, n_in = result

        if not (min_radius <= cr <= max_radius):
            continue

        candidates.append((ccx, ccy, cr, n_in))

    results = nms_circles(candidates)
    return results, green_mask, combined, debug_info


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

        # Convert to int only when passing to OpenCV drawing functions
        icx, icy, ir = int(round(ncx)), int(round(ncy)), int(round(nr))
        cv2.circle(frame, (icx, icy), ir, ring_color, 3)
        cv2.circle(frame, (icx, icy), 4, ring_color, -1)

        lx = max(0, icx - ir)
        ly = max(15, icy - ir - 32)
        status_icon = "OK" if passes else "FAIL"
        d_cm = px_to_cm(nr) * 2  # diameter
        cv2.putText(frame, f"d = {nr*2:.1f}px ({d_cm:.2f}cm) [{status_icon}]",
                    (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.6, ring_color, 2)
        cv2.putText(frame, f"min={MIN_RADIUS_CM*2}cm ({MIN_CAPTURE_RADIUS*2:.1f}px)",
                    (lx, ly+20), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180,180,180), 1)

    return frame


def draw_capture_annotation(frame, cx, cy, cr, center_x,
                             avg_diameter_cm=None, sample_count=None, std_r=None):
    out = frame.copy()
    h, w = out.shape[:2]

    # Convert to int only when passing to OpenCV drawing functions
    icx, icy, icr = int(round(cx)), int(round(cy)), int(round(cr))

    cv2.line(out, (center_x, 0), (center_x, h), (0, 80, 255), 2)
    cv2.circle(out, (icx, icy), icr+2, (255, 255, 255), 4)
    cv2.circle(out, (icx, icy), icr,   (255, 160, 0),   2)
    cv2.line(out, (icx-icr, icy), (icx+icr, icy), (0, 255, 255), 1)  # diameter line
    ch = 10
    cv2.line(out, (icx-ch, icy), (icx+ch, icy), (0,255,255), 2)
    cv2.line(out, (icx, icy-ch), (icx, icy+ch), (0,255,255), 2)
    cv2.circle(out, (icx, icy), 4, (0,255,255), -1)

    bx1 = max(0, icx-icr-6);  by1 = max(0, icy-icr-6)
    bx2 = min(w-1, icx+icr+6); by2 = min(h-1, icy+icr+6)
    cv2.rectangle(out, (bx1, by1), (bx2, by2), (200,200,200), 1)

    label_y = max(20, by1-8)

    # Use averaged diameter if provided, otherwise fall back to direct measurement
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

    # Show sample count and std deviation if available
    if sample_count is not None and std_r is not None:
        std_cm = px_to_cm(std_r) * 2
        cv2.putText(out, f"samples={sample_count}  std={std_cm:.2f}cm",
                    (bx1, label_y+42), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (150, 220, 255), 1)

    cv2.putText(out, time.strftime("%Y-%m-%d  %H:%M:%S"),
                (8, h-8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180,180,180), 1)
    return out


class CaptureState:
    """
    Tracks the averaging window state for a single capture cycle.

    State machine:
        IDLE  ──(object enters centerline)──►  MEASURING
        MEASURING  ──(MEASURE_DELAY elapsed)──►  fires capture → publish feedback → locked forever
        MEASURING  ──(object leaves centerline)──►  IDLE (reset, no capture)

    After one successful capture, no further captures will occur (capture_done=True).
    publish_callback: optional callable() that will be called once after capture to publish ROS2 feedback.
    """
    def __init__(self, publish_callback=None):
        self.last_capture_t   = 0.0    # preserved across reset() calls
        self.must_exit_first  = False   # True after capture — object must leave centerline before next capture
        self.capture_done     = False   # True after first successful capture — no more captures allowed
        self.publish_callback = publish_callback  # called once after capture if provided
        self.reset()

    def reset(self):
        self.measuring       = False
        self.measure_start   = 0.0
        self.radius_samples  = []
        self.cx_samples      = []
        self.cy_samples      = []
        # NOTE: last_capture_t, must_exit_first, capture_done, publish_callback
        # are intentionally NOT reset here so they persist across capture cycles

# Global capture state instance (used by both run_camera and run_ros2)
_capture_state = CaptureState()


def check_and_capture(frame, circles, center_x, cap_counter, state: CaptureState = None):
    """
    Averaging-delay capture logic:
      1. When a valid circle enters the centerline, start measuring.
      2. Collect radius/position samples every frame during MEASURE_DELAY seconds.
      3. After MEASURE_DELAY, compute average radius and position, then capture.
      4. If the object leaves the centerline before delay is up, reset and discard.
      5. Global CAPTURE_COOLDOWN prevents re-triggering immediately after capture.
    """
    global _capture_state
    if state is None:
        state = _capture_state

    now = time.time()

    # ── One-shot lock — no more captures after first success ──
    if state.capture_done:
        return False

    # Find the nearest valid circle to the centerline
    nearest = None
    for c in circles:
        cx, cy, cr, _ = c
        if abs(cx - center_x) <= CENTER_TOL and cr >= MIN_CAPTURE_RADIUS:
            nearest = c
            break

    # ── Object NOT in centerline ──────────────────────────────
    if nearest is None:
        if state.measuring:
            # Object left before delay finished — discard samples
            print(f"  [ABORT] Object left centerline during measuring window — discarding {len(state.radius_samples)} samples")
            state.reset()
        # Object has left the centerline — unlock for next capture
        if state.must_exit_first:
            print("  [READY] Object left centerline — ready for next capture")
            state.must_exit_first = False
        return False

    cx, cy, cr, _ = nearest

    # ── Must exit lock ────────────────────────────────────────
    # Object stayed in centerline after last capture — block until it leaves
    if state.must_exit_first:
        return False

    # ── Global cooldown check ─────────────────────────────────
    if now - state.last_capture_t < CAPTURE_COOLDOWN:
        return False

    # ── Object IS in centerline ───────────────────────────────
    if not state.measuring:
        # First frame object enters centerline — start measuring window
        state.measuring     = True
        state.measure_start = now
        state.radius_samples = []
        state.cx_samples     = []
        state.cy_samples     = []
        print(f"  [MEASURE] Object entered centerline — collecting samples for {MEASURE_DELAY}s...")

    # Collect sample this frame
    state.radius_samples.append(cr)
    state.cx_samples.append(cx)
    state.cy_samples.append(cy)

    elapsed = now - state.measure_start

    # Show measuring progress on console
    progress = min(elapsed / MEASURE_DELAY, 1.0)
    bar = int(progress * 20)
    print(f"\r  [MEASURE] {'█'*bar}{'░'*(20-bar)} {elapsed:.2f}/{MEASURE_DELAY:.2f}s  samples={len(state.radius_samples)}", end="", flush=True)

    # ── Delay elapsed — compute average and capture ───────────
    if elapsed >= MEASURE_DELAY:
        print()  # newline after progress bar

        # Filter outliers — remove samples more than 2 std from median
        r_arr  = np.array(state.radius_samples)
        cx_arr = np.array(state.cx_samples)
        cy_arr = np.array(state.cy_samples)

        median_r = float(np.median(r_arr))
        std_r_all = float(np.std(r_arr))
        mask = np.abs(r_arr - median_r) <= 2 * std_r_all

        r_clean  = r_arr[mask]
        cx_clean = cx_arr[mask]
        cy_clean = cy_arr[mask]

        n_total    = len(r_arr)
        n_filtered = int((~mask).sum())

        # Fall back to all samples if filtering removes everything
        if len(r_clean) == 0:
            r_clean, cx_clean, cy_clean = r_arr, cx_arr, cy_arr
            n_filtered = 0

        avg_r  = float(np.mean(r_clean))
        avg_cx = float(np.mean(cx_clean))
        avg_cy = float(np.mean(cy_clean))
        std_r  = float(np.std(r_clean))
        d_cm   = px_to_cm(avg_r) * 2

        annotated = draw_capture_annotation(frame, avg_cx, avg_cy, avg_r, center_x,
                                             avg_diameter_cm=d_cm,
                                             sample_count=len(r_clean),
                                             std_r=std_r)
        ensure_save_dir()
        cap_counter[0] += 1
        ts = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(SAVE_DIR, f"circle_{ts}_{cap_counter[0]:04d}.png")
        cv2.imwrite(filename, annotated)

        # Set cooldown timestamp BEFORE reset so it is preserved
        last_t = now
        state.reset()
        state.last_capture_t  = last_t
        state.must_exit_first = True   # block re-trigger until object leaves centerline
        state.capture_done    = True   # one-shot: no more captures in this session

        print(f"\n{'='*55}")
        print(f"  [CAPTURE] Averaged measurement captured!")
        print(f"  File      : {filename}")
        print(f"  Avg center: ({avg_cx:.1f}, {avg_cy:.1f})")
        print(f"  Avg radius: {avg_r:.1f}px  std={std_r:.1f}px")
        print(f"  Diameter  : {d_cm:.2f} cm")
        print(f"  Samples   : {len(r_clean)} used / {n_total} total  ({n_filtered} outliers removed)")
        print(f"  Session locked — no further captures this run.")
        print(f"{'='*55}")

        # Publish feedback topic once after capture
        if state.publish_callback is not None:
            state.publish_callback()

        return True

    return False


def draw_overlay(frame, circles, frame_num):
    for cx, cy, r, _ in circles:
        d_cm = px_to_cm(r) * 2  # diameter
        passes = r >= MIN_CAPTURE_RADIUS

        # Convert to int only when passing to OpenCV drawing functions
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
            f"  Frame:{frame_num}  Q=quit  S=shot  M=mask")
    cv2.putText(frame, info, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    return frame


def visualize_static(bgr_img, circles, green_mask, combined,
                     debug_info=None, save_path=None):
    out = bgr_img.copy()
    for cx, cy, r, n_in in circles:
        d_cm = px_to_cm(r) * 2
        icx, icy, ir = int(round(cx)), int(round(cy)), int(round(r))
        cv2.circle(out, (icx, icy), ir, (0, 0, 255), 2)
        cv2.circle(out, (icx, icy), 4, (255, 0, 0), -1)
        cv2.putText(out, f"d={r*2:.1f}px ({d_cm:.2f}cm)",
                    (max(0, icx-ir), max(15, icy-ir-6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

    ncols = 4 if debug_info else 3
    fig, axes = plt.subplots(1, ncols, figsize=(5*ncols, 5))
    axes[0].imshow(cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB))
    axes[0].set_title("Input"); axes[0].axis("off")
    axes[1].imshow(cv2.cvtColor(out, cv2.COLOR_BGR2RGB))
    axes[1].set_title(f"Result ({len(circles)} circles)"); axes[1].axis("off")
    axes[2].imshow(combined, cmap="gray")
    axes[2].set_title("Green Edge + Canny"); axes[2].axis("off")
    if debug_info:
        dbg = bgr_img.copy()
        for d in debug_info:
            col = (0,255,0) if d["passed"] else (80,80,80)
            cv2.drawContours(dbg, [d["cnt"]], -1, col, 1)
        axes[3].imshow(cv2.cvtColor(dbg, cv2.COLOR_BGR2RGB))
        axes[3].set_title("HOG filter (green=pass)"); axes[3].axis("off")

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=130, bbox_inches="tight")
        print(f"Saved → {save_path}")
    else:
        plt.show()
    plt.close()


# ══════════════════════════════════════════════════════════════
#  H. DIRECT CAMERA MODE (local camera fallback)
# ══════════════════════════════════════════════════════════════
def run_camera(camera_index='0', hog_thresh=0.50, edge_thickness=6,
               min_radius=15, max_radius=500, show_mask=False):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera index={camera_index}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)

    frame_w  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    center_x = frame_w // 2

    ensure_save_dir()

    print("=" * 60)
    print(" Green Circle Detector v4 — Live Camera (Direct)")
    print(f" Resolution     = {CAPTURE_WIDTH}x{CAPTURE_HEIGHT} px")
    print(f" FOCAL_LENGTH   = {FOCAL_LENGTH_PX} px (ref@{REFERENCE_WIDTH}px -> scaled={FOCAL_LENGTH_SCALED:.1f}px)")
    print(f" DISTANCE       = {CAMERA_DISTANCE_CM} cm")
    print(f" PX_PER_CM      = {PX_PER_CM:.4f} px/cm")
    print(f" MIN_DIAMETER   = {MIN_RADIUS_CM*2} cm  ({MIN_CAPTURE_RADIUS*2:.2f} px)")
    print(f" Process every  = {PROCESS_EVERY_N_FRAMES} frames")
    print(f" Save images to : ./{SAVE_DIR}/")
    print(" Q / ESC : quit    S : screenshot    M : toggle mask")
    print("=" * 60)

    shot_n        = 0
    show_m        = show_mask
    fps_t         = time.time()
    fps_v         = 0.0
    cap_counter:   list = [0]
    cam_state = CaptureState()  # independent capture state for direct camera mode

    frame_count   = 0
    last_circles  = []
    last_combined = None

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        frame_count += 1
        now = time.time()
        fps_v = 0.9*fps_v + 0.1*(1.0/max(now-fps_t, 1e-6))
        fps_t = now

        if frame_count % PROCESS_EVERY_N_FRAMES == 0:
            last_circles, gmask, last_combined, _ = detect_green_circles(
                frame,
                hog_thresh=hog_thresh,
                edge_thickness=edge_thickness,
                min_radius=min_radius,
                max_radius=max_radius,
            )
            check_and_capture(frame, last_circles, center_x, cap_counter, state=cam_state)

        disp = draw_overlay(frame.copy(), last_circles, frame_count)
        draw_centerline_overlay(disp, last_circles, center_x)

        is_processed = (frame_count % PROCESS_EVERY_N_FRAMES == 0)
        proc_label   = "DETECT" if is_processed else f"skip ({frame_count % PROCESS_EVERY_N_FRAMES}/{PROCESS_EVERY_N_FRAMES})"
        proc_color   = (0, 255, 150) if is_processed else (150, 150, 150)
        cv2.putText(disp, proc_label,
                    (disp.shape[1]-120, 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, proc_color, 1)

        cv2.putText(disp, f"FPS:{fps_v:.1f}",
                    (disp.shape[1]-80, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
        cv2.putText(disp, f"Cap:{cap_counter[0]}",
                    (disp.shape[1]-80, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100,255,150), 1)

        if show_m and last_combined is not None:
            mh, mw = last_combined.shape
            sm = cv2.resize(last_combined, (mw//4, mh//4))
            sm_bgr = cv2.cvtColor(sm, cv2.COLOR_GRAY2BGR)
            dh, dw = disp.shape[:2]
            sh, sw = sm_bgr.shape[:2]
            disp[dh-sh:dh, dw-sw:dw] = sm_bgr
            cv2.putText(disp, "Edge", (dw-sw+2, dh-sh+14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)

        cv2.imshow("Green Circle Detector v4", disp)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            break
        elif key in (ord('s'), ord('S')):
            shot_n += 1
            fn = f"screenshot_{shot_n:03d}.png"
            cv2.imwrite(fn, disp)
            print(f"[Screenshot] -> {fn}")
        elif key in (ord('m'), ord('M')):
            show_m = not show_m

    cap.release()
    cv2.destroyAllWindows()
    print(f"Closed. Total captures: {cap_counter[0]} -> ./{SAVE_DIR}/")


# ══════════════════════════════════════════════════════════════
#  I. ROS2 SUBSCRIBER MODE
#     Receives compressed image from Pi5 via topic
#     /camera_side/image_raw/compressed
#
#     IMPORTANT: cv2.imshow must be called from the main thread.
#     The ROS2 callback only decodes and processes the frame,
#     then stores the result in self.disp_frame.
#     The main thread reads self.disp_frame and handles display.
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

            # Publisher — sends "SUCCESS" once after a successful capture
            self.feedback_pub = self.create_publisher(
                String, '/capture_feedback', 10)

            # Independent capture state with publish callback wired in
            self.cam_state = CaptureState(publish_callback=self._publish_success)

            # Shared frame buffer between callback thread and main display thread
            self.disp_frame = None
            self.frame_lock = threading.Lock()

            # Subscribe to compressed image topic published by Pi5
            self.sub = self.create_subscription(
                CompressedImage,
                '/camera_side/image_raw/compressed',
                self.image_callback,
                10
            )

            # Subscribe to /msg for reset command (DONE:cabbage)
            self.cmd_sub = self.create_subscription(
                String,
                '/msg',
                self.cmd_callback,
                10
            )

            self.get_logger().info("=" * 50)
            self.get_logger().info(" Circle Detector Node — ROS2 Mode")
            self.get_logger().info(" Waiting for images from Pi5...")
            self.get_logger().info(" Topic : /camera_side/image_raw/compressed")
            self.get_logger().info(" Output: /capture_feedback  (String: SUCCESS)")
            self.get_logger().info(" Reset : /msg  (String: DONE:cabbage)")
            self.get_logger().info(f" PX_PER_CM     = {PX_PER_CM:.4f}")
            self.get_logger().info(f" MIN_DIAMETER  = {MIN_RADIUS_CM*2} cm")
            self.get_logger().info(f" Process every = {PROCESS_EVERY_N_FRAMES} frames")
            self.get_logger().info(" Q / ESC : quit    M : toggle mask")
            self.get_logger().info("=" * 50)

        def _publish_success(self):
            """Called once after capture — publishes SUCCESS to /capture_feedback."""
            msg = String()
            msg.data = "SUCCESS"
            self.feedback_pub.publish(msg)
            self.get_logger().info("[FEEDBACK] Published: SUCCESS -> /capture_feedback")

        def cmd_callback(self, msg: String):
            """Listens to /msg topic — resets capture state when 'DONE:cabbage' is received."""
            cmd = msg.data.strip()
            if cmd == "DONE:cabbage":
                self.cam_state.capture_done    = False
                self.cam_state.must_exit_first = False
                self.cam_state.reset()
                self.get_logger().info("[RESET] Received DONE:cabbage — capture state reset, ready for next capture")

        def image_callback(self, msg: CompressedImage):
            # Decode JPEG compressed image from topic
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn("Failed to decode image frame")
                return

            # Update center_x from actual received frame size
            h, w = frame.shape[:2]
            self.center_x = w // 2

            # FPS calculation
            now = time.time()
            self.fps_v = 0.9*self.fps_v + 0.1*(1.0/max(now-self.fps_t, 1e-6))
            self.fps_t = now

            self.frame_count += 1

            # Run detection every N frames to reduce CPU load
            if self.frame_count % PROCESS_EVERY_N_FRAMES == 0:
                self.last_circles, _, self.last_combined, _ = detect_green_circles(
                    frame,
                    hog_thresh=self.hog_thresh,
                    edge_thickness=self.edge_thickness,
                    min_radius=self.min_radius,
                    max_radius=self.max_radius,
                )
                check_and_capture(frame, self.last_circles, self.center_x,
                                  self.cap_counter, state=self.cam_state)

            # Build display frame with overlays
            disp = draw_overlay(frame.copy(), self.last_circles, self.frame_count)
            draw_centerline_overlay(disp, self.last_circles, self.center_x)

            # Frame process status label
            is_processed = (self.frame_count % PROCESS_EVERY_N_FRAMES == 0)
            proc_label   = "DETECT" if is_processed else f"skip ({self.frame_count % PROCESS_EVERY_N_FRAMES}/{PROCESS_EVERY_N_FRAMES})"
            proc_color   = (0, 255, 150) if is_processed else (150, 150, 150)
            cv2.putText(disp, proc_label,
                        (disp.shape[1]-120, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, proc_color, 1)

            # FPS and capture count labels
            cv2.putText(disp, f"FPS:{self.fps_v:.1f}",
                        (disp.shape[1]-80, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
            cv2.putText(disp, f"Cap:{self.cap_counter[0]}",
                        (disp.shape[1]-80, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100,255,150), 1)

            # Optional mask overlay in bottom-right corner
            if self.show_m and self.last_combined is not None:
                mh, mw = self.last_combined.shape
                sm = cv2.resize(self.last_combined, (mw//4, mh//4))
                sm_bgr = cv2.cvtColor(sm, cv2.COLOR_GRAY2BGR)
                dh, dw = disp.shape[:2]
                sh, sw = sm_bgr.shape[:2]
                disp[dh-sh:dh, dw-sw:dw] = sm_bgr
                cv2.putText(disp, "Edge", (dw-sw+2, dh-sh+14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)

            # Source label to distinguish from direct camera mode
            cv2.putText(disp, "SRC: ROS2 Pi5", (8, disp.shape[0]-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 200, 255), 1)

            # Store frame for main thread to display (thread-safe)
            with self.frame_lock:
                self.disp_frame = disp

    # Initialize ROS2 node
    rclpy.init()
    node = CircleDetectorNode()

    # Spin ROS2 in a background thread so main thread stays free for cv2.imshow
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    WIN_NAME = "Green Circle Detector v4 — ROS2"
    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)

    try:
        while rclpy.ok():
            # Read latest frame prepared by callback (main thread only)
            with node.frame_lock:
                frame = node.disp_frame

            if frame is not None:
                cv2.imshow(WIN_NAME, frame)

            key = cv2.waitKey(16) & 0xFF  # ~60 Hz display poll
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

    parser = argparse.ArgumentParser(
        description="Green Circle Detector v4 — Hollow green-edge circles"
    )

    def camera_input(value):
        """Accept both int index (0,1,2) and string device path (/dev/webcam_...)"""
        try:
            return int(value)
        except ValueError:
            return value

    parser.add_argument("--camera",    type=camera_input, default=None,
                        metavar="PATH_OR_INDEX",
                        help="Camera device path or index (e.g. 0 or /dev/webcam_EYD_1080p)")
    parser.add_argument("--ros2",      action="store_true",
                        help="Subscribe to ROS2 topic from Pi5 instead of opening camera directly")
    parser.add_argument("--image",     default=None,
                        help="Path to static image file for single-frame detection")
    parser.add_argument("--save",      default=None,
                        help="Save visualization result to file (image mode only)")
    parser.add_argument("--threshold", type=float, default=0.50,
                        help="HOG arc score threshold (default=0.50)")
    parser.add_argument("--thickness", type=int,   default=6,
                        help="Green edge thickness in px (default=6)")
    parser.add_argument("--min-r",     type=int,   default=15,
                        help="Minimum circle radius in px (default=15)")
    parser.add_argument("--max-r",     type=int,   default=500,
                        help="Maximum circle radius in px (default=500)")
    parser.add_argument("--mask",      action="store_true",
                        help="Show edge mask overlay in corner")
    parser.add_argument("--debug",     action="store_true",
                        help="Show HOG filter debug visualization")
    parser.add_argument("--focal",     type=float, default=FOCAL_LENGTH_PX,
                        help=f"Focal length in px at reference width (default={FOCAL_LENGTH_PX})")
    parser.add_argument("--distance",  type=float, default=CAMERA_DISTANCE_CM,
                        help=f"Camera to object distance in cm (default={CAMERA_DISTANCE_CM})")
    parser.add_argument("--min-cm",    type=float, default=MIN_RADIUS_CM,
                        help=f"Minimum circle radius in cm (default={MIN_RADIUS_CM})")
    parser.add_argument("--every",     type=int,   default=PROCESS_EVERY_N_FRAMES,
                        help=f"Run detection every N frames (default={PROCESS_EVERY_N_FRAMES})")
    parser.add_argument("--width",     type=int,   default=CAPTURE_WIDTH,
                        help=f"Frame width in px (default={CAPTURE_WIDTH})")
    parser.add_argument("--height",    type=int,   default=CAPTURE_HEIGHT,
                        help=f"Frame height in px (default={CAPTURE_HEIGHT})")
    parser.add_argument("--px-per-cm", type=float, default=PX_PER_CM,
                        help=f"Calibrated px/cm value (default={PX_PER_CM})")
    parser.add_argument("--measure-delay", type=float, default=MEASURE_DELAY,
                        help=f"Seconds to collect samples before capturing averaged result (default={MEASURE_DELAY})")
    parser.add_argument("--center-tol", type=int, default=CENTER_TOL,
                        help=f"Centerline tolerance in px (default={CENTER_TOL})")
    args = parser.parse_args()

    random.seed(0)

    # Apply global config overrides from arguments
    FOCAL_LENGTH_PX        = args.focal
    CAMERA_DISTANCE_CM     = args.distance
    MIN_RADIUS_CM          = args.min_cm
    CAPTURE_WIDTH          = args.width
    CAPTURE_HEIGHT         = args.height
    PROCESS_EVERY_N_FRAMES = args.every
    FOCAL_LENGTH_SCALED    = FOCAL_LENGTH_PX * (CAPTURE_WIDTH / REFERENCE_WIDTH)
    PX_PER_CM              = args.px_per_cm
    MIN_CAPTURE_RADIUS     = MIN_RADIUS_CM * PX_PER_CM  # float, no int()
    MEASURE_DELAY          = args.measure_delay
    CENTER_TOL             = args.center_tol

    # ── Mode selection ──────────────────────────────────────────
    # Priority: --ros2 > --camera > --image
    if args.ros2:
        run_ros2(
            hog_thresh=args.threshold,
            edge_thickness=args.thickness,
            min_radius=args.min_r,
            max_radius=args.max_r,
            show_mask=args.mask,
        )
        return

    if args.camera is not None:
        run_camera(
            camera_index=args.camera,
            hog_thresh=args.threshold,
            edge_thickness=args.thickness,
            min_radius=args.min_r,
            max_radius=args.max_r,
            show_mask=args.mask,
        )
        return

    if args.image is None:
        print("[ERROR] Specify one of: --ros2 | --camera <index> | --image <path>")
        parser.print_help()
        return

    # Static image mode
    bgr = cv2.imread(args.image)
    if bgr is None:
        print(f"[ERROR] File not found: {args.image}")
        return

    circles, gmask, combined, dbg = detect_green_circles(
        bgr,
        hog_thresh=args.threshold,
        edge_thickness=args.thickness,
        min_radius=args.min_r,
        max_radius=args.max_r,
        debug=args.debug,
    )

    print(f"\nDetected {len(circles)} circle(s):")
    for i, (cx, cy, r, n_in) in enumerate(circles, 1):
        d_cm = px_to_cm(r) * 2
        print(f"  [{i}] center=({cx:.1f},{cy:.1f})  diameter={r*2:.1f}px ({d_cm:.2f}cm)  inliers={n_in}")

    visualize_static(bgr, circles, gmask, combined,
                     debug_info=dbg if args.debug else None,
                     save_path=args.save)


if __name__ == "__main__":
    main()