#!/bin/bash
# run.sh — launch full autonomous mission
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source venv/bin/activate 2>/dev/null || true

echo "[ROV] Starting autonomous mission..."
python main.py "$@"
