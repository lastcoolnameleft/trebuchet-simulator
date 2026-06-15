#!/usr/bin/env bash
set -euo pipefail

PORT="${PORT:-8080}"
DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "Building trebuchet-simulator..."
cd "$DIR"
npm run build

echo "Starting trebuchet-simulator on http://localhost:${PORT}"
PORT="$PORT" node serve.mjs
