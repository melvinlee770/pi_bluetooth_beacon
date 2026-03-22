#!/usr/bin/env python3
"""
RSE3204 Wireless Localisation - SLAVE (Pi B)
=============================================
Run this on Pi B.
Waits for a request from master over UART, then scans for
the Bluetooth device via RSSI (using Kalman Filter), and sends dXB back.

Usage:
    sudo python3 slave.py

Requirements:
    pip install pyserial
    sudo apt-get install python3-pip libglib2.0-dev
    (bluepy installed from source - see project README)
"""

import serial
import json
import math
from bluepy.btle import Scanner, DefaultDelegate

# -- UART Configuration -------------------------------------------------------
SERIAL_DEVICE  = "/dev/serial0"
SERIAL_SPEED   = 9600
SERIAL_WAIT    = 30        # seconds to wait for a message

# -- Bluetooth Configuration --------------------------------------------------
BEACON_ADDR      = "46:8C:00:00:FE:4D"  # your beacon MAC address
SIGNAL_REF       = -70                   # RSSI at 1 metre (calibrate for your device)
ATTENUATION_EXP  = 2.0                   # Path loss exponent (2.0 = free space)
SAMPLE_COUNT     = 100                   # Number of RSSI readings to collect


# -- Kalman Filter ------------------------------------------------------------

def smooth_signal(data_points: list) -> float:
    """
    Apply a 1D Kalman Filter to a list of RSSI readings.

    Key variables:
      prediction        : current best guess of the true RSSI
      uncertainty       : how uncertain we are about our estimate
      drift_noise       : how much the true RSSI might drift between readings
                          (small = assume RSSI is stable)
      sensor_noise      : how noisy/unreliable the sensor is
                          (larger = trust new readings less)

    Each iteration:
      1. Kalman Gain = uncertainty / (uncertainty + sensor_noise)
         how much to trust the new reading vs current estimate
      2. prediction = prediction + weight * (new_reading - prediction)
         blend estimate toward new reading weighted by gain
      3. uncertainty = (1 - weight) * uncertainty
         shrinks as we become more confident
    """
    prediction    = data_points[0]
    uncertainty   = 1.0
    drift_noise   = 0.01
    sensor_noise  = 2.0

    for sample in data_points[1:]:
        # Prediction step: uncertainty grows slightly each step
        uncertainty = uncertainty + drift_noise

        # Update step
        weight      = uncertainty / (uncertainty + sensor_noise)
        prediction  = prediction + weight * (sample - prediction)
        uncertainty = (1 - weight) * uncertainty

    return prediction


# -- Outlier Removal ----------------------------------------------------------

def filter_anomalies(measurements: list) -> list:
    """
    Remove RSSI readings that are more than 2 standard deviations
    from the mean. This eliminates sudden spikes before Kalman Filter.
    """
    import statistics
    mean_val  = sum(measurements) / len(measurements)
    std_dev   = statistics.stdev(measurements)
    filtered  = [r for r in measurements if abs(r - mean_val) < 2 * std_dev]
    discarded = len(measurements) - len(filtered)
    if discarded > 0:
        print(f"  Removed {discarded} outliers from {len(measurements)} readings")
    return filtered if len(filtered) > 1 else measurements   # fallback if too many removed


# -- Bluetooth RSSI to Distance -----------------------------------------------

class BleListener(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        pass


def signal_to_range(rssi_val: float) -> float:
    """
    Convert RSSI (dBm) to estimated distance (metres) using the log-distance
    path loss model:

        distance = 10 ^ ((SIGNAL_REF - RSSI) / (10 * ATTENUATION_EXP))

    SIGNAL_REF     : RSSI measured at exactly 1 metre (calibrate this!)
    ATTENUATION_EXP: environment constant - 2.0 free space, 2.7-3.5 indoors
    """
    return 10 ** ((SIGNAL_REF - rssi_val) / (10 * ATTENUATION_EXP))


def scan_beacon_range(total_samples: int = SAMPLE_COUNT) -> float:
    """
    Scan for BEACON_ADDR, collect RSSI readings, apply Kalman Filter,
    and return estimated distance in metres.
    """
    print(f"[Slave] Scanning for Bluetooth device ({BEACON_ADDR}) to measure dXB...")
    print(f"[Slave] Collecting {total_samples} samples - keep devices still!")

    ble_scanner = Scanner().withDelegate(BleListener())
    measurements = []

    while len(measurements) < total_samples:
        try:
            found_devices = ble_scanner.scan(1.0)
        except Exception as exc:
            print(f"[Slave] Scan error: {exc}. Retrying...")
            continue

        for dev in found_devices:
            if dev.addr == BEACON_ADDR.lower():
                measurements.append(dev.rssi)
                print(f"  Sample {len(measurements):>3}/{total_samples}  RSSI = {dev.rssi} dBm", end="\r")
                break

    print()

    # Step 1: Remove outliers
    cleaned_data = filter_anomalies(measurements)

    # Step 2: Apply Kalman Filter on clean readings
    smoothed_rssi = smooth_signal(cleaned_data)
    raw_mean      = sum(measurements) / len(measurements)
    range_est     = signal_to_range(smoothed_rssi)

    print(f"[Slave] Simple average RSSI  = {raw_mean:.2f} dBm")
    print(f"[Slave] Kalman filtered RSSI = {smoothed_rssi:.2f} dBm")
    print(f"[Slave] dXB = {range_est:.3f} m")
    return range_est


# -- Main slave loop ----------------------------------------------------------

def execute_slave():
    print("=" * 50)
    print("  RSE3204 Wireless Localisation - SLAVE (Pi B)")
    print("=" * 50)
    print(f"[Slave] Opening UART on {SERIAL_DEVICE} at {SERIAL_SPEED} baud ...")

    with serial.Serial(SERIAL_DEVICE, SERIAL_SPEED, timeout=SERIAL_WAIT) as conn:
        import time
        time.sleep(2)
        conn.reset_input_buffer()
        print("[Slave] UART ready. Waiting for master request ...\n")

        while True:
            # Wait for a line from master
            incoming = conn.readline()
            if not incoming:
                print("[Slave] Timeout waiting for master. Still waiting ...")
                continue

            decoded_msg = incoming.decode().strip()
            if not decoded_msg:
                continue

            try:
                parsed_cmd = json.loads(decoded_msg)
            except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                print(f"[Slave] Bad data received: {exc}. Ignoring.")
                continue

            print(f"[Slave] Request received: {parsed_cmd}")

            if parsed_cmd.get("cmd") == "GET_DISTANCE":
                # Measure dXB via Bluetooth RSSI with Kalman Filter
                try:
                    peer_range = scan_beacon_range()
                except Exception as exc:
                    err_msg = json.dumps({"error": f"Bluetooth scan failed: {exc}"}) + "\n"
                    conn.write(err_msg.encode())
                    conn.flush()
                    print(f"[Slave] Bluetooth error: {exc}")
                    continue

                # Send response back to master
                reply = json.dumps({"dxb": peer_range}) + "\n"
                conn.write(reply.encode())
                conn.flush()
                print(f"[Slave] Sent dXB = {peer_range:.3f} m to master.\n")
                print("[Slave] Waiting for next request ...\n")

            else:
                err_msg = json.dumps({"error": "unknown command"}) + "\n"
                conn.write(err_msg.encode())
                conn.flush()
                print("[Slave] Unknown command. Sent error to master.")


# -- Entry point --------------------------------------------------------------

if __name__ == "__main__":
    try:
        execute_slave()
    except KeyboardInterrupt:
        print("\n[Slave] Stopped.")
    except serial.SerialException as exc:
        print(f"\n[Slave] UART error: {exc}")
        print("  Make sure UART is enabled (raspi-config) and you are running with sudo.")
