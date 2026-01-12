import cv2
import mediapipe as mp
import time
import sys
try:
    import serial
except ImportError:
    serial = None

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

HOLD_TIME = 5

# Per-hand state
hands_state = {
    "Left": {"count": -1, "prev": -1, "start": 0, "stable": False},
    "Right": {"count": -1, "prev": -1, "start": 0, "stable": False}
}

global_hold_start = 0
global_hold_active = False

# Optional: open COM3 to send the confirmed count to the Pico LCD
ser = None
if serial is not None:
    try:
        ser = serial.Serial("COM3", 115200, timeout=1)
        print("Serial connected on COM3 (Pico)")
    except Exception as e:
        print(f"WARN: Could not open COM3: {e}\n      LCD won't update from console unless COM3 is available.")
else:
    print("WARN: pyserial not installed. Run: pip install pyserial")


def count_fingers(hand_landmarks, hand_label):
    tips = [8, 12, 16, 20]
    count = 0

    # Correct thumb logic AFTER cv2.flip(frame,1)
    if hand_label == "Right":
        thumb_open = hand_landmarks.landmark[4].x < hand_landmarks.landmark[3].x
    else:  # Left
        thumb_open = hand_landmarks.landmark[4].x > hand_landmarks.landmark[3].x

    if thumb_open:
        count += 1

    # Other 4 fingers
    for tip in tips:
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
            count += 1

    return count


with mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7) as hands:
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        detected = {"Left": False, "Right": False}
        total_fingers = 0

        # TRACK IF ANY HAND CHANGED COUNT → RESET GLOBAL TIMER
        any_hand_changed = False

        if results.multi_hand_landmarks and results.multi_handedness:
            for i, hand_landmarks in enumerate(results.multi_hand_landmarks):

                label = results.multi_handedness[i].classification[0].label
                detected[label] = True

                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                finger_count = count_fingers(hand_landmarks, label)
                st = hands_state[label]

                if finger_count != st["prev"]:
                    any_hand_changed = True

                # Per-hand stability
                if finger_count == st["prev"]:
                    if st["start"] == 0:
                        st["start"] = time.time()

                    elapsed = time.time() - st["start"]
                    st["stable"] = elapsed >= HOLD_TIME

                else:
                    st["start"] = 0
                    st["stable"] = False

                st["prev"] = finger_count
                st["count"] = finger_count
                total_fingers += finger_count

        left_detected = detected["Left"]
        right_detected = detected["Right"]
        hands_detected = left_detected + right_detected  # 0,1,2 hands

        # Reset global hold if:
        # - counts changed
        # - hands disappear
        if any_hand_changed or hands_detected == 0:
            global_hold_active = False
            global_hold_start = 0

        # Start global hold ONLY when hands_detected > 0
        if hands_detected > 0:
            if not global_hold_active:
                global_hold_active = True
                global_hold_start = time.time()

            global_elapsed = time.time() - global_hold_start

            # UI
            cv2.putText(frame, f"Hands: {hands_detected}", (50, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 3)

            cv2.putText(frame, f"Total Fingers: {total_fingers}", (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 4)

            cv2.putText(frame, f"Hold: {global_elapsed:.1f}/{HOLD_TIME}s", (50, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

            if global_elapsed >= HOLD_TIME:
                if hands_detected == 1:
                    # One-hand mode
                    active = "Left" if left_detected else "Right"
                    if hands_state[active]["stable"]:
                        print(f"CONFIRMED: {total_fingers} fingers (1 hand)")
                        # Send to Pico LCD over COM3
                        if ser:
                            try:
                                ser.write(f"FINGER:{total_fingers}\n".encode())
                                ser.flush()
                            except Exception as e:
                                print(f"WARN: Failed to write to COM3: {e}")
                        global_hold_active = False
                        global_hold_start = 0
                        hands_state[active]["start"] = 0
                        hands_state[active]["stable"] = False

                else:
                    # Two-hand mode → BOTH must be stable
                    if hands_state["Left"]["stable"] and hands_state["Right"]["stable"]:
                        print(f"CONFIRMED: {total_fingers} fingers (2 hands)")
                        # Send to Pico LCD over COM3
                        if ser:
                            try:
                                ser.write(f"FINGER:{total_fingers}\n".encode())
                                ser.flush()
                            except Exception as e:
                                print(f"WARN: Failed to write to COM3: {e}")
                        global_hold_active = False
                        global_hold_start = 0
                        hands_state["Left"]["start"] = 0
                        hands_state["Right"]["start"] = 0
                        hands_state["Left"]["stable"] = False
                        hands_state["Right"]["stable"] = False

        cv2.imshow("Finger Counter", frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
