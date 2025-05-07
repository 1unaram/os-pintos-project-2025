#!/bin/bash
set -e

if [ $# -lt 2 ]; then
  echo "Usage: $0 <num> <layout>"
  echo "Example: $0 5 2A:4C:2B:2C:3A"
  exit 1
fi

NUM=$1
LAYOUT=$2

echo "[+] Cleaning and building..."
cd threads/build/
make clean > ../../make_clean_result 2>&1
cd ../
make > ../make_result 2>&1

echo "[+] Running Pintos with automated_warehouse $NUM $LAYOUT..."
cd build/
../../utils/pintos automated_warehouse "$NUM" "$LAYOUT" 2>&1 | tee ../../output.txt

