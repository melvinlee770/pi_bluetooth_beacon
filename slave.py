#!/usr/bin/env python3
"""
RSE3204 Wireless Localisation - SLAVE (Pi B)
=============================================
PURPOSE:
    This script runs on Raspberry Pi B (the "slave" node) in a two-Pi
    wireless localisation system. It sits idle until the master (Pi A)
    sends a command over UART, then measures the distance to a BLE beacon
    and returns the result.

SYSTEM OVERVIEW:
    The localisation system forms a triangle between three points:
      - Pi A (master)  — runs master.py, measures dXA
      - Pi B (slave)   — runs this script, measures dXB
      - Device X       — BLE beacon whose position we want to estimate

    Pi A and Pi B are connected via UART (serial cable). The master
    coordinates the measurement sequence and computes the final angle.

COMMUNICATION PROTOCOL (UART, JSON-over-serial):
    Request from master:  {"cmd": "GET_DISTANCE"}
    Response from slave:  {"dxb": <float>}        (distance in metres)
    Error response:       {"error": "<message>"}
    All messages are newline-terminated (\n) for readline() framing.

DISTANCE MEASUREMENT PIPELINE:
    1. Perform repeated BLE scans to collect RSSI samples from the beacon.
    2. Remove statistical outliers (readings > 2 SD from mean).
    3. Apply a 1D Kalman filter to smooth the remaining readings.
    4. Convert the filtered RSSI to distance using the log-distance
       path loss model.

DEPENDENCIES:
    pip install pyserial
    bluepy (BLE library, requires libglib2.0-dev, must run as root)

USAGE:
    sudo python3 slave.py
"""

import serial
import json
import math
from bluepy.btle import Scanner, DefaultDelegate

# --- UART Configuration ---
# /dev/serial0 is the default UART device on Raspberry Pi (GPIO 14/15).
# Both master and slave must use the same baud rate.
SERIAL_DEVICE  = "/dev/serial0"
SERIAL_SPEED   = 9600
SERIAL_WAIT    = 30   # Timeout in seconds for waiting for a UART message.
                       # If no message arrives within this window, readline()
                       # returns empty bytes and the loop retries.

# --- Bluetooth Configuration ---
# BEACON_ADDR: MAC address of the target BLE beacon. Must match across all scripts.
BEACON_ADDR      = "46:8C:00:00:FE:4D"

# SIGNAL_REF: The RSSI value (in dBm) measured when the beacon is exactly
# 1 metre away. Obtained by running calibrate.py. This is the anchor point
# for the path loss distance formula — an incorrect value shifts all
# distance estimates proportionally.
SIGNAL_REF       = -70

# ATTENUATION_EXP: Path loss exponent for the environment.
# 2.0 = ideal free space, 2.7–3.5 = typical indoor with walls/furniture.
# Higher values mean signal drops faster with distance.
ATTENUATION_EXP  = 2.0

# SAMPLE_COUNT: Number of RSSI readings to collect per measurement.
# More samples = more stable estimate, but each scan takes ~1 second.
SAMPLE_COUNT     = 100


def smooth_signal(data_points: list) -> float:
    """
    Applies a 1D Kalman filter to a sequence of RSSI readings and returns
    the final filtered estimate.

    The Kalman filter incrementally refines an estimate of the true RSSI by
    balancing two sources of information:
      - The previous estimate (prediction)
      - The new sensor reading (measurement)

    Parameters tuned for BLE RSSI:
      - drift_noise  = 0.01 : Low, because the true RSSI shouldn't change
                               much between consecutive 1-second scans if
                               both devices are stationary.
      - sensor_noise = 2.0  : Moderately high, reflecting the inherent
                               jitter in BLE RSSI readings.

    Algorithm per iteration:
      1. Predict: uncertainty grows by drift_noise (models possible change).
      2. Compute Kalman gain = uncertainty / (uncertainty + sensor_noise).
         This is a value between 0 and 1. High gain = trust the new reading
         more; low gain = trust the existing estimate more.
      3. Update: blend the new reading into the estimate, weighted by gain.
      4. Shrink uncertainty by factor (1 - gain), reflecting increased
         confidence after incorporating the new data.
    """
    prediction    = data_points[0]
    uncertainty   = 1.0
    drift_noise   = 0.01
    sensor_noise  = 2.0

    for sample in data_points[1:]:
        uncertainty = uncertainty + drift_noise
        weight      = uncertainty / (uncertainty + sensor_noise)
        prediction  = prediction + weight * (sample - prediction)
        uncertainty = (1 - weight) * uncertainty

    return prediction


def filter_anomalies(measurements: list) -> list:
    """
    Removes RSSI readings that deviate more than 2 standard deviations
    from the mean. This pre-processing step protects the Kalman filter
    from extreme spikes caused by multipath interference, temporary
    obstructions, or BLE advertising collisions.

    Returns the original list if filtering would leave fewer than 2 samples,
    to avoid breaking downstream calculations.
    """
    import statistics
    mean_val  = sum(measurements) / len(measurements)
    std_dev   = statistics.stdev(measurements)
    filtered  = [r for r in measurements if abs(r - mean_val) < 2 * std_dev]
    discarded = len(measurements) - len(filtered)
    if discarded > 0:
        print(f"  Removed {discarded} outliers from {len(measurements)} readings")
    return filtered if len(filtered) > 1 else measurements


class BleListener(DefaultDelegate):
    """
    Callback handler required by bluepy's Scanner. handleDiscovery() is
    invoked for each BLE advertisement received during a scan window.
    We leave it empty because we process results after the scan completes,
    not during discovery.
    """
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        pass


def signal_to_range(rssi_val: float) -> float:
    """
    Converts an RSSI value (dBm) to an estimated distance (metres) using
    the log-distance path loss model:

        distance = 10 ^ ((SIGNAL_REF - rssi_val) / (10 * ATTENUATION_EXP))

    Derivation:
      The model states that RSSI decreases logarithmically with distance:
        RSSI = SIGNAL_REF - 10 * n * log10(d)
      Solving for d gives the formula above.

    Accuracy depends heavily on correct SIGNAL_REF (from calibrate.py)
    and an appropriate ATTENUATION_EXP for the environment.
    """
    return 10 ** ((SIGNAL_REF - rssi_val) / (10 * ATTENUATION_EXP))


def scan_beacon_range(total_samples: int = SAMPLE_COUNT) -> float:
    """
    Performs the complete distance measurement pipeline:
      1. Runs BLE scans in a loop, collecting one RSSI reading per scan
         cycle from the target beacon (identified by BEACON_ADDR).
         Each scan() call listens for 1 second. If the beacon isn't
         detected in a cycle (it advertises periodically), that cycle
         is simply skipped.
      2. Removes outlier readings via filter_anomalies().
      3. Smooths the cleaned data with the Kalman filter.
      4. Converts the smoothed RSSI to distance via the path loss model.

    Returns the estimated distance in metres (this is dXB — the distance
    from Pi B to the BLE beacon).
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

    cleaned_data  = filter_anomalies(measurements)
    smoothed_rssi = smooth_signal(cleaned_data)
    raw_mean      = sum(measurements) / len(measurements)
    range_est     = signal_to_range(smoothed_rssi)

    print(f"[Slave] Simple average RSSI  = {raw_mean:.2f} dBm")
    print(f"[Slave] Kalman filtered RSSI = {smoothed_rssi:.2f} dBm")
    print(f"[Slave] dXB = {range_est:.3f} m")
    return range_est


def execute_slave():
    """
    Main event loop for the slave node.

    Startup sequence:
      1. Opens the UART serial port.
      2. Waits 2 seconds for the UART hardware to stabilise, then flushes
         any garbage bytes that may have been in the input buffer.
      3. Enters an infinite loop, waiting for JSON commands from the master.

    Command handling:
      - "GET_DISTANCE": Triggers a full BLE scan + distance calculation,
        then sends {"dxb": <distance>} back over UART.
      - Any other command: Responds with {"error": "unknown command"}.
      - If the BLE scan fails, an error JSON is sent back so the master
        can handle it gracefully instead of hanging.

    All UART messages are newline-delimited JSON, encoded as UTF-8.
    conn.flush() is called after every write to ensure the data is
    pushed out immediately rather than sitting in the OS write buffer.
    """
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
                try:
                    peer_range = scan_beacon_range()
                except Exception as exc:
                    err_msg = json.dumps({"error": f"Bluetooth scan failed: {exc}"}) + "\n"
                    conn.write(err_msg.encode())
                    conn.flush()
                    print(f"[Slave] Bluetooth error: {exc}")
                    continue

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


if __name__ == "__main__":
    try:
        execute_slave()
    except KeyboardInterrupt:
        print("\n[Slave] Stopped.")
    except serial.SerialException as exc:
        print(f"\n[Slave] UART error: {exc}")
        print("  Make sure UART is enabled (raspi-config) and you are running with sudo.")
