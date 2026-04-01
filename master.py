#!/usr/bin/env python3
"""
RSE3204 Wireless Localisation - MASTER (Pi A)
=============================================
PURPOSE:
    This script runs on Raspberry Pi A (the "master" node) and orchestrates
    a wireless distance-and-angle measurement using two Pis and one BLE beacon.

SYSTEM OVERVIEW:
    Three points form a triangle:
      - Pi A (master)  — this device, measures dXA (distance to beacon)
      - Pi B (slave)   — runs slave.py, measures dXB (distance to beacon)
      - Device X       — BLE beacon whose angular position we want to estimate

    The master collects all three side lengths of the triangle:
      dXA — measured locally via BLE RSSI
      dAB — entered manually by the operator (physical tape measure)
      dXB — requested from the slave over UART

EXECUTION FLOW:
    1. Scan for the BLE beacon and measure dXA (same pipeline as slave:
       collect RSSI samples -> remove outliers -> Kalman filter -> path loss).
    2. Prompt the operator to enter dAB (Pi A to Pi B baseline distance).
    3. Open UART, send GET_DISTANCE to slave, receive dXB.
    4. Print a formatted results table.

DEPENDENCIES:
    pip install pyserial
    bluepy (BLE library, requires libglib2.0-dev, must run as root)

USAGE:
    sudo python3 master.py
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
SERIAL_WAIT    = None   # No timeout — the master blocks indefinitely waiting
                        # for the slave's response, because the slave's BLE scan
                        # (100 samples) can take over a minute.

# --- Bluetooth Configuration ---
# BEACON_ADDR: MAC address of the target BLE beacon. Must match across all scripts.
BEACON_ADDR      = "46:8C:00:00:FE:4D"

# SIGNAL_REF: The RSSI value (in dBm) measured when the beacon is exactly
# take note: another pi may get different value 
SIGNAL_REF       = -63

# ATTENUATION_EXP: Path loss exponent for the environment.
ATTENUATION_EXP  = 2.0 # do not change (tested)

# SAMPLE_COUNT: Number of RSSI readings to collect per measurement.
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
        weight     = uncertainty / (uncertainty + sensor_noise)
        prediction = prediction + weight * (sample - prediction)
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


def scan_beacon_range(tag: str, total_samples: int = SAMPLE_COUNT) -> float:
    """
    Performs the complete distance measurement pipeline:
      1. Runs BLE scans in a loop, collecting one RSSI reading per scan
         cycle from the target beacon (identified by BEACON_ADDR).
         Each scan() call listens for 1 second. If the beacon isn't
         detected in a cycle, that cycle is simply skipped.
      2. Removes outlier readings via filter_anomalies().
      3. Smooths the cleaned data with the Kalman filter.
      4. Converts the smoothed RSSI to distance via the path loss model.

    The 'tag' parameter is a label string (e.g. "dXA") used in log output
    to identify which distance is being measured.

    Returns the estimated distance in metres.
    """
    print(f"[Master] Scanning for Bluetooth device ({BEACON_ADDR}) to measure {tag}...")
    print(f"[Master] Collecting {total_samples} samples - keep devices still!")

    ble_scanner = Scanner().withDelegate(BleListener())
    measurements = []

    while len(measurements) < total_samples:
        try:
            found_devices = ble_scanner.scan(1.0)
        except Exception as exc:
            print(f"[Master] Scan error: {exc}. Retrying...")
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

    print(f"[Master] Simple average RSSI  = {raw_mean:.2f} dBm")
    print(f"[Master] Kalman filtered RSSI = {smoothed_rssi:.2f} dBm")
    print(f"[Master] {tag} = {range_est:.3f} m")
    return range_est ## RAW rssi -> metres


def request_measurement(tag: str) -> float:
    """
    Prompts the operator to manually enter a distance value via the terminal.
    Used to input dAB — the physical baseline distance between Pi A and Pi B,
    which must be measured with a tape measure since it cannot be derived
    from BLE signals between the two Pis.

    Loops until a valid non-negative number is entered.
    """
    while True:
        try:
            entered = float(input(f"[Master] Enter {tag} (metres): "))
            if entered < 0:
                print("  Distance cannot be negative. Try again.")
                continue
            return entered
        except ValueError:
            print("  Invalid input. Please enter a number.")


def retrieve_range_from_peer(conn: serial.Serial) -> float:
    """
    Sends a GET_DISTANCE command to the slave over UART and blocks until
    the slave responds with its measured distance (dXB).

    Protocol:
      - Sends:    {"cmd": "GET_DISTANCE"}\n
      - Expects:  {"dxb": <float>}\n       on success
                  {"error": "<message>"}\n  on failure

    The slave takes a long time to respond (~100+ seconds) because it
    performs its own full BLE scan. The UART timeout is set to None
    (infinite) on the master side to accommodate this.

    Raises RuntimeError if the slave reports an error.
    """
    payload = json.dumps({"cmd": "GET_DISTANCE"}) + "\n"
    print("[Master] Sending GET_DISTANCE request to slave over UART ...")
    conn.write(payload.encode())
    conn.flush() # send out the data

    print("[Master] Waiting for slave response (this may take a while - slave is scanning 100 samples)...")
    incoming = conn.readline()

    reply = json.loads(incoming.decode().strip()) # received then convert from bytes to dict

    if "error" in reply:
        raise RuntimeError(f"Slave returned error: {reply['error']}")

    peer_range = float(reply["dxb"])
    print(f"[Master] Received dXB = {peer_range:.3f} m from slave.")
    return peer_range


def calculate_bearing(dist_xa: float, dist_xb: float, dist_ab: float):
    """
    Computes the angle at vertex X (the BLE beacon) in the triangle
    formed by Pi A, Pi B, and Device X, using the Law of Cosines:

        cos(theta) = (dXA^2 + dXB^2 - dAB^2) / (2 * dXA * dXB)

    Where:
      dXA = distance from Device X to Pi A (measured by master)
      dXB = distance from Device X to Pi B (measured by slave)
      dAB = distance from Pi A to Pi B     (entered by operator)

    The angle theta_AB tells us how "spread apart" the two Pis appear
    from the beacon's perspective.

    The cos_val is clamped to [-1, 1] to handle floating-point rounding
    errors that could make acos() fail (e.g. cos_val = 1.0000000002).

    Returns the angle in degrees, or None if either distance is zero
    (degenerate triangle).
    """
    if dist_xa == 0 or dist_xb == 0:
        return None

    cos_val = (dist_xa**2 + dist_xb**2 - dist_ab**2) / (2 * dist_xa * dist_xb)
    cos_val = max(-1.0, min(1.0, cos_val))
    return math.degrees(math.acos(cos_val))


def check_triangle_valid(dist_xa: float, dist_xb: float, dist_ab: float) -> bool:
    """
    Verifies that the three distances satisfy the triangle inequality:
    the sum of any two sides must be greater than the third side.
    If this fails, the distances are geometrically impossible (likely due
    to RSSI measurement error) and the angle cannot be computed.
    """
    return (dist_xa + dist_xb > dist_ab) and (dist_xa + dist_ab > dist_xb) and (dist_xb + dist_ab > dist_xa)


def display_summary(dist_xa: float, dist_xb: float, dist_ab: float, bearing):
    """
    Prints a formatted results table showing all three distances and the
    computed angle, followed by an ASCII diagram of the triangle layout
    to help the operator visualise the geometry.
    """
    border = "=" * 46
    print(f"\n  +{border}+")
    print(f"  |{'RSE3204 - Localisation Results':^46}|")
    print(f"  +{border}+")
    print(f"  |  {'dXA  (Pi A -> Bluetooth device)':<38} {dist_xa:>5.3f} m  |")
    print(f"  |  {'dXB  (Pi B -> Bluetooth device)':<38} {dist_xb:>5.3f} m  |")
    print(f"  |  {'dAB  (Pi A -> Pi B baseline)':<38} {dist_ab:>5.3f} m  |")
    print(f"  +{border}+")
    if bearing is not None:
        print(f"  |  {'Angle at device X (theta_AB)':<38} {bearing:>5.2f} deg  |")
    else:
        print(f"  |  {'Angle at device X (theta_AB)':<38} {'N/A':>7}  |")
    print(f"  +{border}+")

    print("\n  Triangle layout (not to scale):\n")
    print("        Pi A")
    print("         |\\")
    print(f"   dXA={dist_xa:.2f}m |  \\ dAB={dist_ab:.2f}m")
    print("         |    \\")
    print("    device X----Pi B")
    print(f"        dXB={dist_xb:.2f}m")
    # if bearing is not None:
    #     print(f"\n  Angle at device X = {bearing:.2f} degrees")
    print()


def execute_master():
    """
    Main orchestration routine for the master node.

    Sequence:
      1. Measure dXA — BLE scan from this Pi to the beacon.
      2. Get dAB    — operator types the physical Pi-to-Pi distance.
      3. Get dXB    — send UART request to slave, wait for its BLE result.
      4. Validate   — check triangle inequality before computing the angle.
      5. Display    — print summary table and ASCII triangle diagram.
    """
    print("=" * 50)
    print("  RSE3204 Wireless Localisation - MASTER (Pi A)")
    print("=" * 50)

    dist_xa = scan_beacon_range("dXA")

    dist_ab = request_measurement("dAB  [Pi A -> Pi B baseline]")

    print(f"\n[Master] Opening UART on {SERIAL_DEVICE} at {SERIAL_SPEED} baud ...")
    with serial.Serial(SERIAL_DEVICE, SERIAL_SPEED, timeout=SERIAL_WAIT) as conn:
        dist_xb = retrieve_range_from_peer(conn)

    if not check_triangle_valid(dist_xa, dist_xb, dist_ab):
        print("\n  WARNING: Distances do not form a valid triangle.")
        print(" Check your measurements.\n")
        # bearing = None
    # else:
    #     bearing = calculate_bearing(dist_xa, dist_xb, dist_ab)

    display_summary(dist_xa, dist_xb, dist_ab)


if __name__ == "__main__":
    try:
        execute_master()
    except KeyboardInterrupt:
        print("\n[Master] Stopped.")
    except serial.SerialException as exc:
        print(f"\n[Master] UART error: {exc}")
        print("  Make sure UART is enabled (raspi-config) and you are running with sudo.")
    except TimeoutError as exc:
        print(f"\n[Master] Timeout: {exc}")
    except Exception as exc:
        print(f"\n[Master] Error: {exc}")
