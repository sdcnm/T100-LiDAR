#!/bin/zsh
cd -- "$(dirname -- "$0")" || exit 1
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m t100_lidar demo --port 18766 --open
