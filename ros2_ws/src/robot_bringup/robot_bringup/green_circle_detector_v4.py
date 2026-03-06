"""
Green Circle Detector v3 — วงกลมกลวง (ขอบสีเขียวเท่านั้น)
===========================================================
[แก้ไข] ประมวลผลทุก 5 เฟรม + แสดงเฉพาะวงกลมที่มีรัศมี >= 10 cm
        โดยกล้องอยู่ห่าง 39 cm (ใช้ focal length ประมาณ 600 px สำหรับ webcam ทั่วไป)

วิธีคำนวณ px → cm:
  px_per_cm = FOCAL_LENGTH_PX / CAMERA_DISTANCE_CM
  radius_cm = radius_px / px_per_cm
"""

# Use the 'by-path' link instead of a number
# cam_path = "/dev/v4l/by-path/platform-xhci-hcd.0-usbv2-0:2.3:1.0-video-index0"
# cap = cv2.VideoCapture(cam_path)

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
CENTER_TOL         = 6      # px — tolerance ของเส้นกลาง
NEAR_LINE_RANGE    = 120    # px
CAPTURE_COOLDOWN   = 1.0    # วินาที

# ── พารามิเตอร์ระยะทางและขนาด ──
FOCAL_LENGTH_PX    = 600    # px  ← วัดที่ความละเอียด REFERENCE_WIDTH (640px)
CAMERA_DISTANCE_CM = 39     # cm  ← ระยะจากกล้องถึงวัตถุ
MIN_RADIUS_CM      = 5      # cm  ← รัศมีขั้นต่ำที่จะแสดง/บันทึก

# ── [แก้ไข] ความละเอียดของกล้อง ──
CAPTURE_WIDTH      = 160    # px  ← ความกว้างที่ใช้จริง
CAPTURE_HEIGHT     = 120    # px  ← ความสูงที่ใช้จริง
REFERENCE_WIDTH    = 640    # px  ← ความกว้างที่วัด FOCAL_LENGTH_PX มา

# ── [แก้ไข] scale focal length ตามความละเอียดจริง แล้วคำนวณ px_per_cm ──
# เมื่อภาพเล็กลง 1/4 วัตถุมีขนาด px น้อยลงด้วย → focal length ต้อง scale ตาม
FOCAL_LENGTH_SCALED = FOCAL_LENGTH_PX * (CAPTURE_WIDTH / REFERENCE_WIDTH) # ควรเป็น 45.24
PX_PER_CM = 1.16
# PX_PER_CM           = FOCAL_LENGTH_SCALED / CAMERA_DISTANCE_CM 
# ควรเป็น 1.16
MIN_CAPTURE_RADIUS  = int(MIN_RADIUS_CM * PX_PER_CM / 2)  # รัศมีขั้นต่ำหน่วย px

# ── ประมวลผลทุก N เฟรม ──
PROCESS_EVERY_N_FRAMES = 5  # ← ตรวจจับทุก 5 เฟรม


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
#  B. CANNY บน GREEN REGION
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
                return cx_r, cy_r, r_r, int(mask.sum())
        except Exception:
            pass
    return best[0], best[1], best[2], best_n


# ══════════════════════════════════════════════════════════════
#  E. NMS
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

        ccx, ccy, cr, n_in = result
        ccx, ccy, cr = int(round(ccx)), int(round(ccy)), int(round(cr))

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
        print(f"[INFO] สร้างโฟลเดอร์ '{SAVE_DIR}' เรียบร้อย")


def find_nearest_to_centerline(circles, center_x):
    best, best_dist = None, float("inf")
    for c in circles:
        cx, cy, r, n_in = c
        dist = abs(cx - center_x)
        if dist < best_dist and dist <= NEAR_LINE_RANGE:
            best_dist = dist
            best = c
    return best, best_dist


def draw_centerline_overlay(frame, circles, center_x, last_captured):
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
        cv2.circle(frame, (ncx, ncy), nr, ring_color, 3)
        cv2.circle(frame, (ncx, ncy), 4, ring_color, -1)

        lx = max(0, ncx - nr)
        ly = max(15, ncy - nr - 32)
        status_icon = "✓" if passes else "✗"
        r_cm = px_to_cm(nr)
        cv2.putText(frame, f"r = {nr}px ({r_cm:.1f}cm) {status_icon}",
                    (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.6, ring_color, 2)
        cv2.putText(frame, f"min={MIN_RADIUS_CM}cm ({MIN_CAPTURE_RADIUS}px)",
                    (lx, ly+20), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180,180,180), 1)

    return frame


def draw_capture_annotation(frame, cx, cy, cr, center_x):
    out = frame.copy()
    h, w = out.shape[:2]
    cv2.line(out, (center_x, 0), (center_x, h), (0, 80, 255), 2)
    cv2.circle(out, (cx, cy), cr+2, (255, 255, 255), 4)
    cv2.circle(out, (cx, cy), cr,   (255, 160, 0),   2)
    cv2.line(out, (cx, cy), (cx+cr, cy), (0, 255, 255), 1)
    ch = 10
    cv2.line(out, (cx-ch, cy), (cx+ch, cy), (0,255,255), 2)
    cv2.line(out, (cx, cy-ch), (cx, cy+ch), (0,255,255), 2)
    cv2.circle(out, (cx, cy), 4, (0,255,255), -1)

    bx1, by1 = max(0, cx-cr-6), max(0, cy-cr-6)
    bx2, by2 = min(w-1, cx+cr+6), min(h-1, cy+cr+6)
    cv2.rectangle(out, (bx1, by1), (bx2, by2), (200,200,200), 1)

    label_y = max(20, by1-8)
    r_cm = px_to_cm(cr)
    for color, thickness in [((255,255,255),3), ((255,160,0),2)]:
        cv2.putText(out, f"r = {cr}px ({r_cm:.1f} cm)",
                    (bx1, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, thickness)
    cv2.putText(out, f"center ({cx}, {cy})",
                (bx1, label_y+22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)
    cv2.putText(out, time.strftime("%Y-%m-%d  %H:%M:%S"),
                (8, h-8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180,180,180), 1)
    return out


def check_and_capture(frame, circles, center_x, last_captured, cap_counter):
    now = time.time()
    captured = False
    for c in circles:
        cx, cy, cr, _ = c
        if abs(cx - center_x) <= CENTER_TOL:
            if cr < MIN_CAPTURE_RADIUS:
                r_cm = px_to_cm(cr)
                print(f"  [SKIP] r={cr}px ({r_cm:.1f}cm) < {MIN_RADIUS_CM}cm ไม่บันทึก")
                continue
            key = f"{cx}_{cy}_{cr}"
            if now - last_captured.get(key, 0) >= CAPTURE_COOLDOWN:
                annotated = draw_capture_annotation(frame, cx, cy, cr, center_x)
                ensure_save_dir()
                cap_counter[0] += 1
                ts = time.strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(SAVE_DIR, f"circle_{ts}_{cap_counter[0]:04d}.png")
                cv2.imwrite(filename, annotated)
                last_captured[key] = now
                r_cm = px_to_cm(cr)
                print(f"\n{'='*50}")
                print(f"  [CAPTURE] วงกลมสัมผัสเส้นกลาง!")
                print(f"  ไฟล์    : {filename}")
                print(f"  ตำแหน่ง : center=({cx}, {cy})")
                print(f"  รัศมี   : {cr}px = {r_cm:.1f} cm ✓")
                print(f"{'='*50}")
                captured = True
    return captured


def draw_overlay(frame, circles, frame_num):
    for cx, cy, r, _ in circles:
        r_cm = px_to_cm(r)
        passes = r >= MIN_CAPTURE_RADIUS

        if passes:
            cv2.circle(frame, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
            ch = 12
            cv2.line(frame, (cx-ch, cy), (cx+ch, cy), (0, 255, 255), 1)
            cv2.line(frame, (cx, cy-ch), (cx, cy+ch), (0, 255, 255), 1)
            label = f"r={r}px ({r_cm:.1f}cm)"
            cv2.putText(frame, label,
                        (max(0, cx-r), max(15, cy-r-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 2)
        else:
            cv2.circle(frame, (cx, cy), r, (120, 120, 120), 1)
            cv2.putText(frame, f"r={r_cm:.1f}cm<{MIN_RADIUS_CM}cm",
                        (max(0, cx-r), max(15, cy-r-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 120, 120), 1)

    valid_count = sum(1 for c in circles if c[2] >= MIN_CAPTURE_RADIUS)
    info = (f"Circles(>={MIN_RADIUS_CM}cm): {valid_count}/{len(circles)}"
            f"  Frame:{frame_num}  Q=quit  S=shot  M=mask")
    cv2.putText(frame, info, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    return frame


def visualize_static(bgr_img, circles, green_mask, combined,
                     debug_info=None, save_path=None):
    out = bgr_img.copy()
    for cx, cy, r, n_in in circles:
        r_cm = px_to_cm(r)
        cv2.circle(out, (cx, cy), r, (0, 0, 255), 2)
        cv2.circle(out, (cx, cy), 4, (255, 0, 0), -1)
        cv2.putText(out, f"r={r}px ({r_cm:.1f}cm)",
                    (max(0, cx-r), max(15, cy-r-6)),
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
#  H. CAMERA MODE
# ══════════════════════════════════════════════════════════════
def run_camera(camera_index=0, hog_thresh=0.50, edge_thickness=6,
               min_radius=15, max_radius=500, show_mask=False):
    # cap = cv2.VideoCapture(camera_index)
    cam_path = "/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:2.3:1.0-video-index0"
    cap = cv2.VideoCapture(cam_path)
    if not cap.isOpened():
        print(f"[ERROR] ไม่สามารถเปิดกล้อง index={camera_index}")
        sys.exit(1)

    # ── [แก้ไข] ลดความละเอียดเป็น 160×120 ──
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)

    frame_w  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    center_x = frame_w // 2

    ensure_save_dir()

    print("=" * 60)
    print(" Green Circle Detector v3 — Live Camera")
    print(f" ความละเอียด    = {CAPTURE_WIDTH}×{CAPTURE_HEIGHT} px")
    print(f" FOCAL_LENGTH   = {FOCAL_LENGTH_PX} px (ref@{REFERENCE_WIDTH}px → scaled={FOCAL_LENGTH_SCALED:.1f}px)")
    print(f" DISTANCE       = {CAMERA_DISTANCE_CM} cm")
    print(f" PX_PER_CM      = {PX_PER_CM:.2f} px/cm")
    print(f" MIN_RADIUS     = {MIN_RADIUS_CM} cm  ({MIN_CAPTURE_RADIUS} px)")
    print(f" ประมวลผลทุก    = {PROCESS_EVERY_N_FRAMES} เฟรม")
    print(f" บันทึกภาพไปที่ : ./{SAVE_DIR}/")
    print(" Q / ESC : ออก    S : screenshot    M : toggle mask")
    print("=" * 60)

    shot_n        = 0
    show_m        = show_mask
    fps_t         = time.time()
    fps_v         = 0.0
    last_captured: dict = {}
    cap_counter:   list = [0]

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
            check_and_capture(frame, last_circles, center_x,
                              last_captured, cap_counter)

        disp = draw_overlay(frame.copy(), last_circles, frame_count)
        draw_centerline_overlay(disp, last_circles, center_x, last_captured)

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

        cv2.imshow("Green Circle Detector v3", disp)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            break
        elif key in (ord('s'), ord('S')):
            shot_n += 1
            fn = f"screenshot_{shot_n:03d}.png"
            cv2.imwrite(fn, disp)
            print(f"[Screenshot] → {fn}")
        elif key in (ord('m'), ord('M')):
            show_m = not show_m

    cap.release()
    cv2.destroyAllWindows()
    print(f"ปิดโปรแกรมเรียบร้อย  (แคปภาพทั้งหมด {cap_counter[0]} ใบ → ./{SAVE_DIR}/)")


# ══════════════════════════════════════════════════════════════
#  J. ENTRY POINT
# ══════════════════════════════════════════════════════════════
def main():
    global FOCAL_LENGTH_PX, CAMERA_DISTANCE_CM, MIN_RADIUS_CM
    global PX_PER_CM, MIN_CAPTURE_RADIUS, PROCESS_EVERY_N_FRAMES
    global CAPTURE_WIDTH, CAPTURE_HEIGHT, FOCAL_LENGTH_SCALED

    parser = argparse.ArgumentParser(
        description="Green Circle Detector v3 — วงกลมกลวงขอบสีเขียว"
    )
    parser.add_argument("--camera",    type=int,   default=None, metavar="INDEX")
    parser.add_argument("--image",     default=None)
    parser.add_argument("--save",      default=None)
    parser.add_argument("--threshold", type=float, default=0.50)
    parser.add_argument("--thickness", type=int,   default=6)
    parser.add_argument("--min-r",     type=int,   default=15)
    parser.add_argument("--max-r",     type=int,   default=500)
    parser.add_argument("--mask",      action="store_true")
    parser.add_argument("--debug",     action="store_true")
    parser.add_argument("--focal",     type=float, default=FOCAL_LENGTH_PX)
    parser.add_argument("--distance",  type=float, default=CAMERA_DISTANCE_CM)
    parser.add_argument("--min-cm",    type=float, default=MIN_RADIUS_CM)
    parser.add_argument("--every",     type=int,   default=PROCESS_EVERY_N_FRAMES)
    parser.add_argument("--width",     type=int,   default=CAPTURE_WIDTH,
                        help=f"ความกว้างภาพ px (default={CAPTURE_WIDTH})")
    parser.add_argument("--height",    type=int,   default=CAPTURE_HEIGHT,
                        help=f"ความสูงภาพ px (default={CAPTURE_HEIGHT})")
    args = parser.parse_args()

    random.seed(0)

    FOCAL_LENGTH_PX        = args.focal
    CAMERA_DISTANCE_CM     = args.distance
    MIN_RADIUS_CM          = args.min_cm
    CAPTURE_WIDTH          = args.width
    CAPTURE_HEIGHT         = args.height
    PROCESS_EVERY_N_FRAMES = args.every

    # ── [แก้ไข] scale focal length ตามความละเอียดจริงก่อนคำนวณ px_per_cm ──
    FOCAL_LENGTH_SCALED = FOCAL_LENGTH_PX * (CAPTURE_WIDTH / REFERENCE_WIDTH)
    PX_PER_CM           = FOCAL_LENGTH_SCALED / CAMERA_DISTANCE_CM
    MIN_CAPTURE_RADIUS  = int(MIN_RADIUS_CM * PX_PER_CM / 2)

    if args.camera is not None:
        run_camera(camera_index=args.camera,
                   hog_thresh=args.threshold,
                   edge_thickness=args.thickness,
                   min_radius=args.min_r,
                   max_radius=args.max_r,
                   show_mask=args.mask)
        return

    if args.image is None:
        print("[ERROR] ระบุ --image <path> หรือ --camera <index>")
        return

    bgr = cv2.imread(args.image)
    if bgr is None:
        print(f"[ERROR] ไม่พบไฟล์: {args.image}")
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
        r_cm = px_to_cm(r)
        print(f"  [{i}] center=({cx},{cy})  radius={r}px ({r_cm:.1f}cm)  inliers={n_in}")

    visualize_static(bgr, circles, gmask, combined,
                     debug_info=dbg if args.debug else None,
                     save_path=args.save)


if __name__ == "__main__":
    main()