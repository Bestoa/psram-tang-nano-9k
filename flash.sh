#!/bin/bash
# Flash bitstream to Tang Nano 9K embedded flash and recover the serial port.
# openFPGALoader wedges macOS's AppleUSBFTDI dext (ports stuck EBUSY);
# killing the dext makes it respawn cleanly.
set -e
cd "$(dirname "$0")"
openFPGALoader -b tangnano9k -f impl/pnr/project.fs
sudo killall com.apple.DriverKit-AppleUSBFTDI 2>/dev/null || true
sleep 2
stty -f /dev/tty.usbserial-1101 115200 cs8 -cstopb -parenb -ixon -ixoff raw
echo "Flashed. UART: /dev/cu.usbserial-1101 @115200"
