#!/bin/bash
set -eu
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec uv run --offline --frozen --extra live python main.py "$@"
