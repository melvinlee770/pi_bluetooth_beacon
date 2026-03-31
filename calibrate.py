#!/usr/bin/env python3
"""
RSE3204 Wireless Localisation - Calibration Script
===================================================
PURPOSE:
    Determines the SIGNAL_REF value (also called TX_POWER) — the RSSI reading
    your Pi's Bluetooth radio reports when the beacon is exactly 1 metre away.
    This value is needed by both master.py and slave.py to convert RSSI into
    distance using the log-distance path loss model.

HOW IT WORKS:
    1. The operator places the beacon exactly 1 m from the Pi.
    2. The script performs 100 BLE scans, collecting one RSSI sample per scan
       from the target beacon.
    3. Outliers beyond 1 standard deviation are discarded to remove
       environmental spikes (reflections, interference).
    4. The cleaned mean RSSI is the recommended SIGNAL_REF value.

HOW TO USE:
    sudo python3 calibrate.py
    Then copy the output value into the SIGNAL_REF constant in both
    master.py and slave.py.

DEPENDENCIES:
    - bluepy (BLE library for Linux / Raspberry Pi)
    - Must run as root (sudo) because BLE scanning requires elevated privileges.
"""

import statistics
from bluepy.btle import Scanner, DefaultDelegate

# MAC address of the BLE beacon to calibrate against.
# Change this to match your specific beacon hardware.
BEACON_ADDR   = "46:8C:00:00:FE:4D"

# Number of RSSI samples to collect. More samples = more stable average,
# but takes longer (each BLE scan cycle is ~1 second).
SAMPLE_COUNT  = 100


class BleListener(DefaultDelegate):
    """
    Required callback handler for bluepy's Scanner. The Scanner calls
    handleDiscovery() each time it finds a BLE advertisement. We don't need
    to do anything here — we just iterate over results after each scan cycle.
    """
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleDiscovery(self, dev, isNewDev, isNewData):
        pass


def filter_anomalies(measurements: list) -> list:
    """
    Removes RSSI readings that fall more than 1 standard deviation from the
    mean. BLE RSSI can spike due to multipath reflections or temporary
    obstructions, and these spikes would skew the calibration value.

    Uses 1 SD (stricter than the 2 SD used at runtime in master/slave)
    because calibration accuracy is critical — it directly affects all
    future distance calculations.

    Returns the original list as a fallback if filtering would leave fewer
    than 2 samples (avoids division-by-zero downstream).
    """
    ## calculate mean and standard deviation
    mean_val  = sum(measurements) / len(measurements)  
    std_dev   = statistics.stdev(measurements)
    filtered  = [r for r in measurements if abs(r - mean_val) < 1 * std_dev] 
    discarded = len(measurements) - len(filtered)
    if discarded > 0:
        print(f"  Removed {discarded} outliers from {len(measurements)} readings")
    return filtered if len(filtered) > 1 else measurements


def execute_calibration():
    """
    Main calibration routine.

    Flow:
      1. Wait for operator confirmation that beacon is at 1 m.
      2. Run BLE scans in a loop until SAMPLE_COUNT readings are collected.
         Each scan() call runs for 1 second and may or may not detect the
         beacon (BLE advertising is periodic, not continuous).
      3. Compute raw mean, then filter outliers and compute cleaned mean.
      4. Print the recommended SIGNAL_REF value to use in master/slave.
    """
    print("=" * 50)
    print("  RSE3204 - TX_POWER Calibration")
    print("=" * 50)
    print(f"\nTarget MAC : {BEACON_ADDR}")
    print(f"Samples    : {SAMPLE_COUNT}")
    print("\nPlace your beacon EXACTLY 1 metre from the Pi.")
    input("Press Enter when ready...")

    ble_scanner  = Scanner().withDelegate(BleListener())
    measurements = []

    print(f"\nCollecting {SAMPLE_COUNT} samples - keep devices still!")
    while len(measurements) < SAMPLE_COUNT:
        try:
            found_devices = ble_scanner.scan(1.0)
        except Exception as exc:
            print(f"Scan error: {exc}. Retrying...")
            continue

        # Each scan returns a list of discovered devices. We only care about
        # our target beacon, identified by its MAC address.
        for dev in found_devices:
            if dev.addr == BEACON_ADDR.lower():
                measurements.append(dev.rssi)
                print(f"  Sample {len(measurements):>3}/{SAMPLE_COUNT}  RSSI = {dev.rssi} dBm", end="\r")
                break

    print("\n")

    ## second calculation pass: compute mean 
    raw_mean = sum(measurements) / len(measurements)
    print(f"Raw simple average     = {raw_mean:.2f} dBm")

    filtered_data = filter_anomalies(measurements)
    filtered_mean = sum(filtered_data) / len(filtered_data)
    print(f"Cleaned simple average = {filtered_mean:.2f} dBm")
    print(f"Samples kept           = {len(filtered_data)}/{SAMPLE_COUNT}")

    # The rounded mean is the SIGNAL_REF value: the expected RSSI at 1 metre.
    # This becomes the reference point for all distance calculations.
    print(f"\n{'=' * 50}")
    print(f"  Recommended TX_POWER = {round(filtered_mean)}")
    print(f"{'=' * 50}")
    print(f"\nSet this in both master.py and slave.py:")
    print(f"  TX_POWER = {round(filtered_mean)}")


if __name__ == "__main__":
    try:
        execute_calibration()
    except KeyboardInterrupt:
        print("\nCalibration stopped.")
    except Exception as exc:
        print(f"\nError: {exc}")
