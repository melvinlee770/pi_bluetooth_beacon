# RSE3204 project: wireless localisation and tracking

## Setup

### Packages
python packages download:- pyserial, bluepy

command: pip3 install pyserial bluepy

### UART
enable the UART transmit for raspberry pi4

1. run the command "sudo raspi-config"
2. choose the options of "Interface Options"
3. choose the options of "Serial Port" and options of "no" for the login shell access and "yes" for serial port hardware enable
4. reboot the device
5. run the command of "sudo nano /boot/firmware/config.txt
6. check the from bottom of the file to find the line "enable_uart=1"
7. connect the jumper wire across the 2 raspberry pi4 by follow the pinout

### Steps to run the program
1. put the bluetooth device 1 meter away from both raspberry pi4
2. run the command of "sudo python calibrate.py" in both devices seperately, it will return the value of "TX_POWER =" (normally both devices will return different value)
3. change the variable value "SIGNAL_REF" in the file "master.py" and "slave.py" for both devices
4. run the "slave.py" by using the command of "sudo python3 slave.py", this device will be piB
5. run the "master.py" by using the command of "sudo python3 master.py", this device will be piA
6. once both pi done for the 100 sampling , the end result (dimension of the triangle) will output at the terminal in piA

### others
- the file "findTargetRSSI.py" is the testing code that created by the team to check the MAC address of the device that paired with the pi, and it will run a for loop to check the RSSI value of the device
- Physical environment can affect the end result of the program, recommend to run in the environment that do not have too many electronics device and metal
