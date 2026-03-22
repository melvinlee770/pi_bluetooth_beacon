#!/usr/bin/env python3
"""
RSE3204 Wireless Localisation - Calibration Script
===================================================
Place your Bluetooth beacon exactly 1 metre from the Pi and run this.
It collects 100 RSSI samples, removes outliers, and gives you the
TX_POWER value to paste into master.py and slave.py.

Usage:
    sudo python3 calibrate.py
"""

import statistics
from bluepy.btle import Scanner, DefaultDelegate

# -- Configuration ------------------------------------------------------------
BEACON_ADDR   = "46:8C:00:00:FE:4D"   # your beacon MAC address
SAMPLE_COUNT  = 100


# -- Scan Delegate ------------------------------------------------------------

class BleListener(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleDiscovery(self, dev, isNewDev, isNewData):
        pass


# -- Outlier Removal ----------------------------------------------------------

def filter_anomalies(measurements: list) -> list:
    """
    Remove RSSI readings more than 2 standard deviations from the mean.
    Eliminates sudden spikes for a cleaner TX_POWER estimate.
    """
    mean_val  = sum(measurements) / len(measurements)
    std_dev   = statistics.stdev(measurements)
    filtered  = [r for r in measurements if abs(r - mean_val) < 1 * std_dev]
    discarded = len(measurements) - len(filtered)
    if discarded > 0:
        print(f"  Removed {discarded} outliers from {len(measurements)} readings")
    return filtered if len(filtered) > 1 else measurements


# -- Main ---------------------------------------------------------------------

def execute_calibration():
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

        for dev in found_devices:
            if dev.addr == BEACON_ADDR.lower():
                measurements.append(dev.rssi)
                print(f"  Sample {len(measurements):>3}/{SAMPLE_COUNT}  RSSI = {dev.rssi} dBm", end="\r")
                break

    print("\n")

    # Stats before cleaning
    raw_mean = sum(measurements) / len(measurements)
    print(f"Raw simple average     = {raw_mean:.2f} dBm")

    # Remove outliers
    filtered_data = filter_anomalies(measurements)
    filtered_mean = sum(filtered_data) / len(filtered_data)
    print(f"Cleaned simple average = {filtered_mean:.2f} dBm")
    print(f"Samples kept           = {len(filtered_data)}/{SAMPLE_COUNT}")

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
