import serial
import subprocess

# Open the Pico USB serial port
ser = serial.Serial("COM3", 115200, timeout=1)

print("Supervisor ready. Waiting for Pico...")

while True:
    line = ser.readline().decode().strip()
    if not line:
        continue

    print("Pico:", line)

    if line == "START_FINGER_SCAN":
        print("Running finger_counter.py...")
        # Run script and capture its output
        result = subprocess.run(
            ["python", "finger_counter.py"],
            capture_output=True,
            text=True
        )
        
        # finger_counter.py prints "FINGER:x"
        for output_line in result.stdout.splitlines():
            if output_line.startswith("FINGER:"):
                print("Sending result to Pico:", output_line)
                ser.write((output_line + "\n").encode())
