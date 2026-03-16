#!/bin/bash
# test.sh — run full test suite (no hardware required)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
source venv/bin/activate 2>/dev/null || true
echo "[ROV] Running test suite..."
python -m pytest tests/test_all.py -v --tb=short "$@"
