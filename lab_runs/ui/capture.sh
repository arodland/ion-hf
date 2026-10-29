#!/bin/sh
# Screenshots of both lamp styles in each state, at 1280x720 and
# 1920x1080: ionhf --loopback, driven through its host ports by drive.py.
# Run from the worktree root after a release build; the portable
# target/release/ionhf.toml sets the call and host port 8400.
set -e
out=$(pwd)/lab_runs/ui
cd target/release
run() { # name snr at [driver mode] [extra flags]
    name=$1 snr=$2 at=$3 mode=$4; shift 4
    if [ "$mode" != none ]; then
        python "$out/drive.py" "$mode" --port 8400 --secs $((at + 12)) > "$out/drive_$name.log" 2>&1 &
    fi
    ./ionhf.exe --loopback --loop-snr "$snr" --size 1280x720 --shots "$out" \
        --shot-set "$name" --shot-at "$at" --quit $((at + 10)) "$@"
    wait
}
run idle 10 6 none
run mfsk -7 75 arq
run ofdm 25 70 arq
run bcast 10 22 bcast
run log 25 60 arq --log
