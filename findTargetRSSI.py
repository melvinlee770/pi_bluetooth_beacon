import subprocess
import sys
import time
from bluepy.btle import Scanner, DefaultDelegate

# --- HELPER: System Service Manager ---
def set_bluetooth_service(state):
    """
    Toggles the system bluetooth service to prevent conflicts.
    state='on'  -> Required for fetching paired devices
    state='off' -> Required for bluepy scanning
    """
    if state == "on":
        print("[System] Starting Bluetooth service to check paired list...")
        subprocess.run(["sudo", "systemctl", "start", "bluetooth"], check=False)
        time.sleep(2) # Give it time to wake up
    elif state == "off":
        print("[System] Stopping Bluetooth service to allow Scanner access...")
        subprocess.run(["sudo", "systemctl", "stop", "bluetooth"], check=False)
        subprocess.run(["sudo", "hciconfig", "hci0", "up"], check=False)
        time.sleep(1)

# --- STEP 1: Find the Paired MAC ---
def get_first_paired_device():
    set_bluetooth_service("on")
    
    try:
        # Run the command to list devices
        output = subprocess.check_output(["bt-device", "-l"]).decode("utf-8")
        lines = output.strip().split('\n')
        
        for line in lines:
            if "Added devices:" in line: 
                continue
            
            # Format is: "DeviceName (MAC)"
            if "(" in line and ")" in line:
                parts = line.split("(")
                name = parts[0].strip()
                mac = parts[1].replace(")", "").strip()
                
                print(f"[Success] Found Paired Device: {name} ({mac})")
                return mac # Return the first one found
                
        print("[Error] No paired devices found!")
        return None

    except Exception as e:
        print(f"[Error] Could not get paired devices: {e}")
        return None

# --- STEP 2: The Scanner (Bluepy) ---
class TargetScanDelegate(DefaultDelegate):
    def __init__(self, target_mac):
        DefaultDelegate.__init__(self)
        self.target_mac = target_mac.lower()

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if dev.addr.lower() == self.target_mac:
            if isNewDev:
                print(f"\n!!! FOUND TARGET: {dev.addr} | RSSI: {dev.rssi} dB !!!")
            elif isNewData:
                print(f" -> Signal Update: RSSI={dev.rssi} dB")

def start_scan(target_mac):
    # Switch mode: Kill system bluetooth so bluepy can work
    set_bluetooth_service("off")
    
    print(f"\n--- Scanning for TARGET: {target_mac} ---")
    print("Press Ctrl+C to stop...")
    
    scanner = Scanner().withDelegate(TargetScanDelegate(target_mac))
    
    while True:
        scanner.scan(2.0, passive=False)

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    try:
        # 1. Auto-fetch the MAC
        my_mac = get_first_paired_device()
        
        if my_mac:
            # 2. Start Scanning automatically
            start_scan(my_mac)
        else:
            print("Exiting because no MAC address was found.")
            
    except KeyboardInterrupt:
        print("\n\n[User] Stopped by user.")
        sys.exit(0)