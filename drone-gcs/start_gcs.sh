#!/usr/bin/env bash
set -euo pipefail
cd "$HOME/drone-gcs"
source .venv/bin/activate
exec uvicorn launcher:app --host 0.0.0.0 --port 8000
