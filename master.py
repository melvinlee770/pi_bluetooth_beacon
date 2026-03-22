#!/usr/bin/env python3
"""
RSE3204 Wireless Localisation - MASTER (Pi A)
=============================================
Run this on Pi A.
It:
  1. Scans for the target Bluetooth device and measures dXA via RSSI
     (using a Kalman Filter to reduce noise)
  2. Asks the operator to enter dAB (Pi A → Pi B baseline distance)
  3. Sends a GET_DISTANCE request to Slave (Pi B) over UART
  4. Receives dXB from Pi B over UART
  5. Computes θAB using the Law of Cosines
  6. Prints a formatted summary table

Usage:
    sudo python3 master.py

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
SERIAL_WAIT    = None      # no timeout - wait as long as slave needs (100 samples takes a while)

# -- Bluetooth Configuration --------------------------------------------------
BEACON_ADDR      = "46:8C:00:00:FE:4D"  # your beacon MAC address
SIGNAL_REF       = -63                   # RSSI at 1 metre (calibrate for your device)
ATTENUATION_EXP  = 2.0                  # Path loss exponent (2.0 = free space)
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
      2. prediction = prediction + gain * (new_reading - prediction)
         blend estimate toward new reading weighted by gain
      3. uncertainty = (1 - gain) * uncertainty
         shrinks as we become more confident
    """
    prediction    = data_points[0]   # start with first reading
    uncertainty   = 1.0
    drift_noise   = 0.01          # assume RSSI is fairly stable
    sensor_noise  = 2.0           # RSSI is noisy, moderately high

    for sample in data_points[1:]:
        # Prediction step: uncertainty grows slightly each step
        uncertainty = uncertainty + drift_noise

        # Update step
        weight     = uncertainty / (uncertainty + sensor_noise)
        prediction = prediction + weight * (sample - prediction)
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


def scan_beacon_range(tag: str, total_samples: int = SAMPLE_COUNT) -> float:
    """
    Scan for BEACON_ADDR, collect RSSI readings, apply Kalman Filter,
    and return estimated distance in metres.
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

    # Step 1: Remove outliers
    cleaned_data = filter_anomalies(measurements)

    # Step 2: Apply Kalman Filter on clean readings
    smoothed_rssi = smooth_signal(cleaned_data)
    raw_mean      = sum(measurements) / len(measurements)
    range_est     = signal_to_range(smoothed_rssi)

    print(f"[Master] Simple average RSSI  = {raw_mean:.2f} dBm")
    print(f"[Master] Kalman filtered RSSI = {smoothed_rssi:.2f} dBm")
    print(f"[Master] {tag} = {range_est:.3f} m")
    return range_est


# -- Manual distance input ----------------------------------------------------

def request_measurement(tag: str) -> float:
    """Prompt operator for a distance value (used for dAB baseline)."""
    while True:
        try:
            entered = float(input(f"[Master] Enter {tag} (metres): "))
            if entered < 0:
                print("  Distance cannot be negative. Try again.")
                continue
            return entered
        except ValueError:
            print("  Invalid input. Please enter a number.")


# -- UART communication -------------------------------------------------------

def retrieve_range_from_peer(conn: serial.Serial) -> float:
    """Send GET_DISTANCE request over UART and wait for dXB response."""
    payload = json.dumps({"cmd": "GET_DISTANCE"}) + "\n"
    print("[Master] Sending GET_DISTANCE request to slave over UART ...")
    conn.write(payload.encode())
    conn.flush()

    print("[Master] Waiting for slave response (this may take a while - slave is scanning 100 samples)...")
    incoming = conn.readline()

    reply = json.loads(incoming.decode().strip())

    if "error" in reply:
        raise RuntimeError(f"Slave returned error: {reply['error']}")

    peer_range = float(reply["dxb"])
    print(f"[Master] Received dXB = {peer_range:.3f} m from slave.")
    return peer_range


# -- Geometry -----------------------------------------------------------------

def calculate_bearing(dist_xa: float, dist_xb: float, dist_ab: float):
    """
    Compute theta_AB (angle at vertex X) using the Law of Cosines:

        cos(theta) = (dXA^2 + dXB^2 - dAB^2) / (2 * dXA * dXB)

    Returns angle in degrees, or None if degenerate.
    """
    if dist_xa == 0 or dist_xb == 0:
        return None

    cos_val = (dist_xa**2 + dist_xb**2 - dist_ab**2) / (2 * dist_xa * dist_xb)
    cos_val = max(-1.0, min(1.0, cos_val))
    return math.degrees(math.acos(cos_val))


def check_triangle_valid(dist_xa: float, dist_xb: float, dist_ab: float) -> bool:
    """Triangle inequality check."""
    return (dist_xa + dist_xb > dist_ab) and (dist_xa + dist_ab > dist_xb) and (dist_xb + dist_ab > dist_xa)


# -- Display ------------------------------------------------------------------

def display_summary(dist_xa: float, dist_xb: float, dist_ab: float, bearing):
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
    if bearing is not None:
        print(f"\n  Angle at device X = {bearing:.2f} degrees")
    print()


# -- Main ---------------------------------------------------------------------

def execute_master():
    print("=" * 50)
    print("  RSE3204 Wireless Localisation - MASTER (Pi A)")
    print("=" * 50)

    # Step 1: Measure dXA via Bluetooth RSSI with Kalman Filter
    dist_xa = scan_beacon_range("dXA")

    # Step 2: Get baseline dAB from operator (tape measure)
    dist_ab = request_measurement("dAB  [Pi A -> Pi B baseline]")

    # Step 3: Open UART and fetch dXB from slave
    print(f"\n[Master] Opening UART on {SERIAL_DEVICE} at {SERIAL_SPEED} baud ...")
    with serial.Serial(SERIAL_DEVICE, SERIAL_SPEED, timeout=SERIAL_WAIT) as conn:
        dist_xb = retrieve_range_from_peer(conn)

    # Step 4: Validate triangle
    if not check_triangle_valid(dist_xa, dist_xb, dist_ab):
        print("\n  WARNING: Distances do not form a valid triangle.")
        print("     Angle cannot be computed. Check your measurements.\n")
        bearing = None
    else:
        bearing = calculate_bearing(dist_xa, dist_xb, dist_ab)

    # Step 5: Display results
    display_summary(dist_xa, dist_xb, dist_ab, bearing)


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
