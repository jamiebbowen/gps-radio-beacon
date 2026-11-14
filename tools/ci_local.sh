#!/usr/bin/env bash
# Run what CI runs (.github/workflows/ci.yml), locally, before pushing.
#
#   make ci-local            (or tools/ci_local.sh from the repo root)
#
# Stops at the first failing step. The receiver firmware build needs
# arm-none-eabi-gcc and receiver/firmware/lib/STM32CubeF4; the transmitter
# build needs arduino-cli with the pinned core and libraries. A missing
# toolchain skips that build with a note instead of failing.
set -euo pipefail

cd "$(dirname "$0")/.."
LOGS=$(mktemp -d /tmp/ci-local.XXXXXX)
TX_BUILD="$LOGS/tx-build"
trap 'echo; echo "ci-local: FAILED at: $step (logs in $LOGS)"' ERR

run() {   # run <label> <command...>
    step="$1"; shift
    echo "== $step"
    "$@"
}

run "receiver unit tests"    bash -c "make -C receiver/tests test 2>&1 | tee '$LOGS/build-rx-tests.log' > /dev/null"
run "transmitter unit tests" bash -c "make -C transmitter/tests test 2>&1 | tee '$LOGS/build-tx-tests.log' > /dev/null"
run "sanitizer run"          bash -c "make sanitize 2>&1 | tee '$LOGS/build-sanitize.log' > /dev/null"
run "lfs_extract tests"      bash -c "make -C tools/lfs_extract test 2>&1 | tee '$LOGS/build-lfsx.log' > /dev/null"
run "no compiler warnings in host builds" \
    bash -c "! grep -h 'warning:' '$LOGS'/build-*.log"
run "tools tests + packet-format drift guard" bash -c "make tools-test > '$LOGS/tools-test.log' 2>&1"
run "receiver coverage gate" \
    bash -c "make -C receiver/tests coverage 2>&1 | python3 tools/check_coverage.py rx > '$LOGS/cov-rx.log' || { cat '$LOGS/cov-rx.log'; exit 1; }"
run "transmitter coverage gate" \
    bash -c "make -C transmitter/tests coverage 2>&1 | python3 tools/check_coverage.py tx > '$LOGS/cov-tx.log' || { cat '$LOGS/cov-tx.log'; exit 1; }"

if command -v arm-none-eabi-gcc > /dev/null && [ -d receiver/firmware/lib/STM32CubeF4/Drivers ]; then
    run "receiver firmware (-Werror)" bash -c "make -C receiver/firmware compile WERROR=1 > '$LOGS/rx-fw.log' 2>&1 || { tail -20 '$LOGS/rx-fw.log'; exit 1; }"
    run "receiver reachability scan" python3 tools/reachability.py --tx-map /nonexistent
else
    echo "== receiver firmware: SKIPPED (arm-none-eabi-gcc or STM32CubeF4 missing)"
fi

if command -v arduino-cli > /dev/null; then
    run "transmitter firmware (--warnings all)" bash -c "arduino-cli compile --fqbn adafruit:samd:adafruit_itsybitsy_m4 --warnings all --build-path '$TX_BUILD' transmitter/firmware > '$LOGS/tx-fw.log' 2>&1 || { tail -20 '$LOGS/tx-fw.log'; exit 1; }"
    run "no warnings in transmitter sources" \
        bash -c "! grep -E 'transmitter/firmware/[^:]+:[0-9]+:[0-9]+: (warning|error):' '$LOGS/tx-fw.log'"
    run "transmitter flash-size guard" python3 tools/check_tx_flash.py "$TX_BUILD/firmware.ino.elf"
    run "transmitter reachability scan" python3 tools/reachability.py --rx-map /nonexistent --tx-map "$TX_BUILD/firmware.ino.map"
else
    echo "== transmitter firmware: SKIPPED (arduino-cli missing)"
fi

trap - ERR
echo
echo "ci-local: all steps passed (logs in $LOGS)"
