# RSE3204 Wireless Localisation

This project contains three Python scripts used to estimate the position of a Bluetooth beacon with two Raspberry Pis:

- `calibrate.py` measures the beacon RSSI at exactly 1 metre and recommends a `TX_POWER` value.
- `master.py` runs on Pi A, measures its own distance to the beacon, requests Pi B's distance over UART, and computes the angle at the beacon.
- `slave.py` runs on Pi B, waits for the master request, measures its own distance to the beacon, and sends that result back over UART.

The scripts are intended for a setup where:

- Pi A is the master node.
- Pi B is the slave node.
- Both Pis scan for the same Bluetooth beacon.
- Pi A and Pi B are connected over UART.

## Files

### `calibrate.py`

Purpose:

- Collects 100 RSSI samples from the target Bluetooth beacon.
- Removes RSSI outliers.
- Computes an average RSSI at 1 metre.
- Prints a recommended `TX_POWER` value.

Why it matters:

The distance estimate in both `master.py` and `slave.py` depends heavily on `TX_POWER`. If this value is wrong, the calculated distances will be wrong.

How it works:

1. The script scans for the beacon MAC address stored in `BEACON_ADDR`.
2. It records `SAMPLE_COUNT` RSSI readings.
3. It removes readings outside one standard deviation from the mean.
4. It computes the cleaned average RSSI.
5. It prints the rounded result as the value to copy into the other scripts.

Run:

```bash
sudo python3 calibrate.py
```

Important settings:

- `BEACON_ADDR`: MAC address of the Bluetooth beacon.
- `SAMPLE_COUNT`: number of RSSI samples to collect.

Expected output:

- Raw average RSSI
- Cleaned average RSSI
- Recommended `TX_POWER`

After calibration:

Copy the recommended `TX_POWER` value into both `master.py` and `slave.py` so both devices use the same calibration.

### `master.py`

Purpose:

- Runs on Pi A.
- Measures Pi A to beacon distance, `dXA`.
- Prompts the operator for the baseline distance between the two Pis, `dAB`.
- Requests Pi B to measure its own distance to the beacon, `dXB`.
- Uses the Law of Cosines to compute the angle at the beacon.
- Prints a formatted summary table.

How it works:

1. Scans for the beacon and collects RSSI samples.
2. Removes outliers from the collected RSSI values.
3. Applies a 1D Kalman filter to smooth the RSSI sequence.
4. Converts filtered RSSI to a distance estimate using the log-distance path loss model.
5. Asks the user to enter `dAB`, the measured distance from Pi A to Pi B.
6. Sends a JSON UART request to the slave:

```json
{"cmd": "GET_DISTANCE"}
```

7. Waits for the slave to return:

```json
{"dxb": <distance_in_metres>}
```

8. Checks whether `dXA`, `dXB`, and `dAB` can form a valid triangle.
9. If valid, computes the beacon angle with the Law of Cosines.
10. Prints the final results.

Key functions:

- `smooth_signal(data_points)`: smooths RSSI noise.
- `filter_anomalies(measurements)`: removes abnormal RSSI spikes.
- `signal_to_range(rssi_val)`: converts RSSI to metres.
- `scan_beacon_range(tag)`: performs the local Bluetooth measurement.
- `retrieve_range_from_peer(conn)`: sends the UART request and receives the slave result.
- `calculate_bearing(dist_xa, dist_xb, dist_ab)`: computes the angle using triangle geometry.
- `check_triangle_valid(dist_xa, dist_xb, dist_ab)`: checks triangle inequality.
- `display_summary(...)`: prints the formatted result block.

Run:

```bash
sudo python3 master.py
```

Important settings:

- `SERIAL_DEVICE`: UART device path, currently `/dev/serial0`.
- `SERIAL_SPEED`: UART speed, currently `9600`.
- `BEACON_ADDR`: beacon MAC address.
- `SIGNAL_REF`: calibrated RSSI at 1 metre.
- `ATTENUATION_EXP`: propagation constant used in distance estimation.
- `SAMPLE_COUNT`: number of Bluetooth RSSI samples.

Inputs required from the operator:

- `dAB`, the measured baseline distance between Pi A and Pi B.

Outputs:

- `dXA`: Pi A to beacon distance.
- `dXB`: Pi B to beacon distance, received from the slave.
- `dAB`: manually entered baseline distance.
- `theta_AB`: angle at the beacon.

### `slave.py`

Purpose:

- Runs on Pi B.
- Waits for a UART request from the master.
- Measures Pi B to beacon distance, `dXB`, when requested.
- Sends the result back to the master as JSON.

How it works:

1. Opens the UART connection and waits for input.
2. Reads a line from the master.
3. Parses the line as JSON.
4. If the command is `GET_DISTANCE`, it scans for the beacon.
5. It removes outliers from the RSSI values.
6. It applies the Kalman filter.
7. It converts the filtered RSSI to a distance estimate.
8. It sends the measured distance back to the master.

Response format:

```json
{"dxb": <distance_in_metres>}
```

If something fails, it sends an error message such as:

```json
{"error": "Bluetooth scan failed: ..."}
```

Key functions:

- `smooth_signal(data_points)`: smooths RSSI noise.
- `filter_anomalies(measurements)`: removes abnormal RSSI spikes.
- `signal_to_range(rssi_val)`: converts RSSI to metres.
- `scan_beacon_range()`: performs the Bluetooth scan and distance estimation.
- `execute_slave()`: main UART loop.

Run:

```bash
sudo python3 slave.py
```

Important settings:

- `SERIAL_DEVICE`: UART device path, currently `/dev/serial0`.
- `SERIAL_SPEED`: UART speed, currently `9600`.
- `SERIAL_WAIT`: how long the slave waits for UART input.
- `BEACON_ADDR`: beacon MAC address.
- `SIGNAL_REF`: calibrated RSSI at 1 metre.
- `ATTENUATION_EXP`: propagation constant used in distance estimation.
- `SAMPLE_COUNT`: number of Bluetooth RSSI samples.

## Recommended workflow

1. Place the beacon exactly 1 metre from a Raspberry Pi.
2. Run `calibrate.py`.
3. Copy the recommended `TX_POWER` into both `master.py` and `slave.py`.
4. Set the same `BEACON_ADDR` in all three files.
5. Connect Pi A and Pi B over UART.
6. Start `slave.py` on Pi B.
7. Start `master.py` on Pi A.
8. Enter the measured `dAB` baseline distance when prompted.
9. Read the final `dXA`, `dXB`, and angle output from the master.

## Dependencies

Python packages:

- `pyserial`
- `bluepy`

System packages mentioned in the scripts:

```bash
sudo apt-get install python3-pip libglib2.0-dev
pip install pyserial
```

`bluepy` is expected to be installed separately as described in your course or project setup.

## Configuration notes

- Keep `BEACON_ADDR` identical in all scripts.
- Keep `SIGNAL_REF` identical in `master.py` and `slave.py` after calibration.
- `ATTENUATION_EXP = 2.0` assumes near free-space conditions. Indoors, a higher value may be more realistic.
- Each script currently uses `SAMPLE_COUNT = 100`, so scans may take some time.
- `master.py` waits indefinitely for the slave response because its UART timeout is set to `None`.

## Limitations

- RSSI-based distance estimation is sensitive to obstacles, reflections, antenna orientation, and interference.
- The result is an estimate, not an exact physical position.
- A bad `SIGNAL_REF` value or incorrect `ATTENUATION_EXP` value can significantly distort the geometry.
- The triangle calculation only works when the three distances satisfy the triangle inequality.

## Typical data flow

```text
Beacon RSSI -> filtering -> distance estimate

Pi A:
  Beacon RSSI -> dXA
  User input  -> dAB
  UART request to Pi B

Pi B:
  Beacon RSSI -> dXB
  UART response to Pi A

Pi A:
  dXA + dXB + dAB -> angle calculation -> printed results
```