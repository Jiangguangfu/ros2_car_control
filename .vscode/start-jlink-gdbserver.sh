#!/usr/bin/env bash
set -euo pipefail

workspace_dir="${1:?workspace directory is required}"
jlink_server="${2:?J-Link GDB Server path is required}"
device="${3:?J-Link device is required}"
gdb_port="${4:?GDB port is required}"
swo_port="${5:?SWO port is required}"
telnet_port="${6:?Telnet port is required}"
rtos_plugin="${7:-}"

log="$workspace_dir/build/jlink-gdbserver.log"
mkdir -p "$(dirname "$log")"

is_windows_exe() {
    local path="$1"
    [[ "$path" == /mnt/* ]] && [[ "$path" == *.exe || "$path" == *.EXE ]]
}

port_listening() {
    local port="$1"
    pgrep -f "JLinkGDBServerCL.*-port ${port}" >/dev/null 2>&1
}

if port_listening "$gdb_port"; then
    echo "J-Link GDB Server port $gdb_port is already listening; reusing existing server."
    exit 0
fi

: > "$log"

jlink_args=(
    -singlerun
    -if swd
    -device "$device"
    -speed 1000
    -port "$gdb_port"
    -swoport "$swo_port"
    -telnetport "$telnet_port"
)

if [[ -n "$rtos_plugin" ]]; then
    jlink_args+=(-rtos "$rtos_plugin")
fi

if is_windows_exe "$jlink_server"; then
    # WSL cannot exec PE binaries directly; /init launches Windows processes.
    # /init consumes the first argument after the executable path; "." is a no-op placeholder.
    nohup /init "$jlink_server" . "${jlink_args[@]}" > "$log" 2>&1 &
else
    nohup "$jlink_server" "${jlink_args[@]}" > "$log" 2>&1 &
fi

server_pid=$!
disown "$server_pid" 2>/dev/null || true

echo "Started J-Link GDB Server pid=$server_pid, log=$log"

for _ in $(seq 1 80); do
    if awk 'index($0, "Waiting for GDB connection") { found = 1 } END { exit found ? 0 : 1 }' "$log" 2>/dev/null; then
        echo "J-Link GDB Server is ready on port $gdb_port"
        exit 0
    fi

    if ! kill -0 "$server_pid" 2>/dev/null; then
        echo "J-Link GDB Server exited before it became ready. See $log"
        exit 1
    fi

    sleep 0.25
done

echo "Timed out waiting for J-Link GDB Server. See $log"
exit 1
