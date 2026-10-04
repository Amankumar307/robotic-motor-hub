#!/usr/bin/env bash
# Load the Linux Kernel Module and create device node with proper permissions
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRIVER_KO="$DIR/driver/motor_hub_driver.ko"

if [ ! -f "$DRIVER_KO" ]; then
    echo "[!] Driver binary not found. Compiling kernel module..."
    make -C "$DIR/driver"
fi

echo "[+] Loading kernel module: $DRIVER_KO"
sudo insmod "$DRIVER_KO"

echo "[+] Adjusting device node permissions for /dev/motor_hub"
sudo chmod 666 /dev/motor_hub

echo "[+] Verifying driver in dmesg..."
dmesg | tail -n 10 | grep motor_hub

echo "[SUCCESS] Robotic Motor Hub Driver loaded successfully!"
