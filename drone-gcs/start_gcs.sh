#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [ -f .venv/bin/activate ]; then source .venv/bin/activate; fi
exec uvicorn launcher:app --host 0.0.0.0 --port 8000
