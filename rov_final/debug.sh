#!/bin/bash
# debug.sh — dry-run with camera display
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
source venv/bin/activate 2>/dev/null || true
echo "[ROV] Debug mode (dry-run + display)..."
python main.py --dry-run --debug "$@"
