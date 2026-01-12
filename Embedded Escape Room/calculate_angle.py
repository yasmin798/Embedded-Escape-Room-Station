import cv2
import numpy as np
import serial
import time

# ---- SERIAL COMM ----
try:
    pico = serial.Serial("COM3", 115200, timeout=0.1, write_timeout=0.1)
    print("✓ Connected to COM3")
except Exception as e:
    print(f"⚠ Serial port COM3 not available: {e}")
    pico = None

# ---------------- SIMPLE HSV COLOR RANGES ----------------
# Red (BASE): Two ranges for hue wraparound, lower saturation for clear reds
hsv_ranges = {
    'base': (
        np.array([0, 50, 100], dtype=np.uint8),       # lower (S≥50 for less saturated)
        np.array([10, 255, 255], dtype=np.uint8),     # upper
        np.array([170, 50, 100], dtype=np.uint8),     # extra_lower (red wraparound, S≥50)
        np.array([180, 255, 255], dtype=np.uint8)     # extra_upper
    ),
    # Blue (JOINT)
    'joint': (
        np.array([100, 100, 100], dtype=np.uint8),    # lower
        np.array([130, 255, 255], dtype=np.uint8),    # upper
        None, None                                     # no extra range
    ),
    # Green (EFFECTOR) - adjusted for light/lime green
    'effector': (
        np.array([25, 40, 80], dtype=np.uint8),      # lower (reduced S≥40, V≥80 for lighter green)
        np.array([85, 255, 255], dtype=np.uint8),    # upper (extended hue range)
        None, None                                    # no extra range
    )
}

print("✓ Using simple HSV color ranges:")
print("  RED (base):      H=0-10,170-180 S≥50 V≥80")
print("  BLUE (joint):    H=100-130 S≥100 V≥100")
print("  GREEN (effector): H=25-85 S≥40 V≥80 (light/lime green)")

# ------------- DETECT ALL COLOR CANDIDATES -------------
def detect_all_color_candidates(frame, lower, upper, extra_lower=None, extra_upper=None, min_area=150):
    """Returns list of all candidate markers sorted by area (largest first)"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array(lower) if not isinstance(lower, np.ndarray) else lower
    upper = np.array(upper) if not isinstance(upper, np.ndarray) else upper
    mask = cv2.inRange(hsv, lower, upper)

    if extra_lower is not None:   # for red dual-range
        extra_lower = np.array(extra_lower) if not isinstance(extra_lower, np.ndarray) else extra_lower
        extra_upper = np.array(extra_upper) if not isinstance(extra_upper, np.ndarray) else extra_upper
        mask2 = cv2.inRange(hsv, extra_lower, extra_upper)
        mask = mask | mask2

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    
    for c in cnts:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        candidates.append((cx, cy, area))
    
    # Sort by area (largest first - likely closest/most prominent)
    candidates.sort(key=lambda x: x[2], reverse=True)
    return candidates

# ------------- FIND BEST DEPTH-MATCHED SET -------------
def find_best_depth_matched_set(base_candidates, joint_candidates, eff_candidates, tolerance=0.4):
    """
    Find the best combination of markers with similar depths (areas).
    Returns: (base, joint, effector, debug_info) or (None, None, None, error_msg)
    """
    if not (base_candidates and joint_candidates and eff_candidates):
        missing = []
        if not base_candidates: missing.append("RED")
        if not joint_candidates: missing.append("BLUE")
        if not eff_candidates: missing.append("GREEN")
        return None, None, None, f"Missing: {','.join(missing)}"
    
    best_combo = None
    best_deviation = float('inf')
    
    # Try all combinations
    for base in base_candidates[:3]:  # Limit to top 3 of each color
        for joint in joint_candidates[:3]:
            for eff in eff_candidates[:3]:
                areas = [base[2], joint[2], eff[2]]
                avg_area = np.mean(areas)
                max_deviation = max(abs(a - avg_area) / avg_area for a in areas)
                
                if max_deviation < best_deviation:
                    best_deviation = max_deviation
                    best_combo = (base, joint, eff)
    
    if best_deviation <= tolerance:
        base, joint, eff = best_combo
        debug_info = f"Areas: R={base[2]:.0f} B={joint[2]:.0f} G={eff[2]:.0f} Dev={best_deviation:.1%}"
        return base, joint, eff, debug_info
    else:
        return None, None, None, f"No depth match (best dev={best_deviation:.1%})"

# ---------------- ANGLE ----------------
def angle_between(p1, p2, p3):
    try:
        v1 = np.array(p1) - np.array(p2)
        v2 = np.array(p3) - np.array(p2)
        
        mag1 = np.linalg.norm(v1)
        mag2 = np.linalg.norm(v2)
        
        if mag1 == 0 or mag2 == 0:
            return 0.0
        
        dot_product = np.dot(v1, v2) / (mag1 * mag2)
        # Clamp to [-1, 1] to avoid numerical errors
        dot_product = np.clip(dot_product, -1.0, 1.0)
        
        ang = np.degrees(np.arccos(dot_product))
        return ang
    except Exception as e:
        print(f"Angle calculation error: {e}")
        return 0.0

# ---- CAMERA ----
print("Opening camera...")
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("❌ ERROR: Cannot open camera 0!")
    exit(1)

print("✓ Camera opened successfully")
print("=" * 60)
print("TRACKING MODE - Press ESC to quit")
print("=" * 60)

cv2.namedWindow("angles")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # ---- DETECT ALL COLOR CANDIDATES ----
    base_candidates  = detect_all_color_candidates(frame, *hsv_ranges['base'])
    joint_candidates = detect_all_color_candidates(frame, *hsv_ranges['joint'])
    eff_candidates   = detect_all_color_candidates(frame, *hsv_ranges['effector'])

    # Debug: show detection status
    status = f"R:{len(base_candidates)} B:{len(joint_candidates)} G:{len(eff_candidates)}"
    cv2.putText(frame, status, (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Find best depth-matched combination
    base, joint, eff, depth_info = find_best_depth_matched_set(
        base_candidates, joint_candidates, eff_candidates, tolerance=0.5
    )
    
    # Display depth match info
    if base and joint and eff:
        cv2.putText(frame, depth_info, (10, frame.shape[0] - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    else:
        cv2.putText(frame, depth_info, (10, frame.shape[0] - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow("angles", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        continue

    # Extract positions (now tuples include area)
    bx, by, _ = base
    jx, jy, _ = joint
    ex, ey, _ = eff

    # compute angles
    C1 = angle_between((bx, by), (jx, jy), (jx+100, jy))
    C2 = angle_between((jx, jy), (ex, ey), (jx-100, jy))

    # Send Pico-friendly ANGLE format
    if pico:
        try:
            pico.write(f"ANGLE:{C1:.2f},{C2:.2f}\n".encode())
        except Exception as e:
            print(f"Serial write error: {e}")
            pico = None  # Disable serial if it fails

    # Throttled console logging every ~200 ms
    if 'last_print_ts' not in globals():
        last_print_ts = 0.0
    now_ts = time.time()
    if now_ts - last_print_ts >= 0.2:
        # Mirror serial format for easy debugging
        print(f"ANGLE:{C1:.2f},{C2:.2f}")
        last_print_ts = now_ts

    # debug draw (BGR format: Red, Blue, Green)
    cv2.circle(frame, (bx, by), 10, (0,0,255), -1)    # RED circle for base
    cv2.circle(frame, (jx, jy), 10, (255,0,0), -1)    # BLUE circle for joint
    cv2.circle(frame, (ex, ey), 10, (0,255,0), -1)    # GREEN circle for effector

    cv2.putText(frame, f"C1: {C1:.1f}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
    cv2.putText(frame, f"C2: {C2:.1f}", (20, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    cv2.imshow("angles", frame)
    key = cv2.waitKey(1) & 0xFF
    
    if key == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
