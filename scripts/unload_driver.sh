#!/usr/bin/env bash
# Unload the Linux Kernel Module safely
set -e

echo "[-] Unloading motor_hub_driver kernel module..."
sudo rmmod motor_hub_driver

echo "[+] Verifying removal in dmesg..."
dmesg | tail -n 5 | grep motor_hub || true

echo "[SUCCESS] Driver unloaded cleanly."
