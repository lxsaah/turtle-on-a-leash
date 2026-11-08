#!/bin/sh
# The way to start the gateway. Each start resets the board first, so the
# gateway always meets a fresh session (aimdb-dev/aimdb#443), and exit status 3
# (board lost) restarts it, because mirrors do not survive a board reset.
#
# Usage: SERIAL_PORT=/dev/cu.usbmodemXXXX gateway/run.sh
CHIP=STM32H563ZITx
BOOT_SECONDS=1

cd "$(dirname "$0")/.." || exit 1
cargo build -p gateway || exit 1

while :; do
    if ! probe-rs reset --chip "$CHIP"; then
        echo "gateway: could not reset the board. Is it connected, and is another probe-rs session holding the probe?" >&2
        exit 1
    fi
    sleep "$BOOT_SECONDS"

    ./target/debug/gateway
    status=$?
    [ "$status" -eq 3 ] || exit "$status"
    echo "gateway: board lost, restarting" >&2
    sleep 1
done
