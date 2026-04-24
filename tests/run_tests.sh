#!/bin/bash

# Test suite runner for MOCO jib firmware
# Catches regressions: trigger polarity, direction mappings, motor control

set -e

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$TEST_DIR/build"

echo "=== MOCO Jib Regression Test Suite ==="
echo ""

mkdir -p "$BUILD_DIR"

# Compile and run each test
for test_file in "$TEST_DIR"/*.cpp; do
  if [ -f "$test_file" ]; then
    test_name=$(basename "$test_file" .cpp)
    echo "Running: $test_name"
    
    g++ -std=c++11 -Wall -Wextra "$test_file" -o "$BUILD_DIR/$test_name"
    "$BUILD_DIR/$test_name"
    
    echo "✓ $test_name passed"
    echo ""
  fi
done

echo "=== All Regression Tests Passed ==="
echo ""
echo "Safe to deploy firmware."
