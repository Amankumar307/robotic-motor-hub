#!/usr/bin/env bash
# Execute the full Pick-and-Place simulation and verification test
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=========================================================="
echo " Running Robotic Motor Hub Simulation Verification Suite  "
echo "=========================================================="

python3 "$DIR/scripts/simulate_and_verify.py"
