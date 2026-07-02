# ============================================================
# lora_publisher.py
# Reads FREQ and RPM from STM32 via USB-UART (FTDI)
# Publishes to MQTT broker
# ============================================================

import serial
import paho.mqtt.client as mqtt
import json
import time

# ── Configure your serial port ──────────────────────────────
SERIAL_PORT = "COM11"         # Windows: COM3, COM4 etc.
                              # Linux:   /dev/ttyUSB0
                              # Mac:     /dev/tty.usbserial-xxxx
BAUD_RATE   = 9600            # Must match STM32 UART1 baud rate

# ── MQTT settings ────────────────────────────────────────────
BROKER  = "localhost"
PORT    = 1883
TOPIC   = "iot/sensor/data"

TIMER_CLOCK_HZ = 100000
def compute_arr(freq: int) -> int:
    """
    Mirrors the STM32 formula:
        arr = (100000 / freq) - 1
    Returns -1 if freq is invalid.
    """
    if freq <= 0 or freq > 50000:
        return -1
    arr = (TIMER_CLOCK_HZ // freq) - 1
    return min(arr, 99999)   # clamp to 16-bit max


# ── Parse the STM32 line ─────────────────────────────────────
def parse_lora_line(line: str):
    """
    Parses lines like: 'FREQ:63 RPM:3780'
    Handles corrupted RPM prefixes: RP: RM: RR: R:
    Returns dict or None if parse fails.
    """
    line = line.strip()
    if not line:
        return None

    data = {}

    # Parse FREQ
    if "FREQ:" in line:
        try:
            freq_part = line.split("FREQ:")[1].split()[0]
            data["frequency"] = int(freq_part)
        except (IndexError, ValueError):
            data["frequency"] = 0

    # Parse RPM — tolerant of corrupted prefix
    rpm_value = None
    for prefix in ["RPM:", "RP:", "RM:", "RR:", "R:"]:
        if prefix in line:
            idx = line.find(prefix)
            if idx == 0 or line[idx - 1] in (' ', '\t'):
                try:
                    rpm_part = line[idx + len(prefix):].split()[0]
                    rpm_value = int(rpm_part)
                    break
                except (IndexError, ValueError):
                    pass

    data["rpm"] = rpm_value if rpm_value is not None else 0
     # ── Compute ARR from FREQ ─────────────────────────────────
    if "frequency" in data:
        data["arr"]        = compute_arr(data["frequency"])
        data["pwm_freq_hz"] = data["frequency"]   # alias for clarity
        data["duty_cycle_pct"] = 50               # always 50% in your STM32 code
    # Add timestamp
    data["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")

    return data if "frequency" in data else None


def main():
    # ── Connect to MQTT broker ────────────────────────────────
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)  # FIXED
    mqtt_client.connect(BROKER, PORT)
    mqtt_client.loop_start()
    print(f"Connected to MQTT broker at {BROKER}:{PORT}")

    # ── Open serial port ──────────────────────────────────────
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud")
    except serial.SerialException as e:
        print(f"ERROR: Could not open serial port — {e}")
        print("Check: Is FTDI plugged in? Is the port correct?")
        return

    print(f"Publishing to topic '{TOPIC}'... Press Ctrl+C to stop.\n")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore")
            print(f"RAW  ← {line.strip()}")

            data = parse_lora_line(line)
            if data:
                payload = json.dumps(data)
                mqtt_client.publish(TOPIC, payload)
                print(f"PUB  → {payload}\n")
            else:
                print("SKIP — could not parse line\n")

    except KeyboardInterrupt:
        print("\nStopped publisher.")
    finally:
        ser.close()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()