#!/usr/bin/env bash
set -euo pipefail

workspace_dir="${1:?workspace directory is required}"
jlink_exe="${2:?JLink.exe path is required}"
device="${3:?J-Link device is required}"
elf_path="${4:?ELF path is required}"

if [[ ! -f "$elf_path" ]]; then
    echo "ELF not found: $elf_path"
    exit 1
fi

if [[ ! -x "$jlink_exe" && ! -f "$jlink_exe" ]]; then
    echo "JLink.exe not found: $jlink_exe"
    exit 1
fi

to_win_path() {
    local p="$1"
    if command -v wslpath >/dev/null 2>&1; then
        wslpath -w "$p"
    else
        # Fallback: /mnt/c/foo -> C:\foo
        if [[ "$p" =~ ^/mnt/([a-zA-Z])/(.*)$ ]]; then
            local drive="${BASH_REMATCH[1]}"
            local rest="${BASH_REMATCH[2]//\//\\}"
            echo "${drive^^}:\\${rest}"
        else
            echo "Cannot convert path to Windows: $p" >&2
            exit 1
        fi
    fi
}

script_dir="$workspace_dir/build"
mkdir -p "$script_dir"
cmd_script="$script_dir/jlink_flash.jlink"
log="$script_dir/jlink_flash.log"
elf_win="$(to_win_path "$elf_path")"

cat > "$cmd_script" <<EOF
device ${device}
si SWD
speed 4000
connect
h
loadfile ${elf_win}
r
g
q
EOF

: > "$log"
echo "Flashing $elf_path via $jlink_exe"
echo "Commander script: $cmd_script"

# /init launches Windows PE from WSL; "." is a required no-op placeholder.
/init "$jlink_exe" . -CommanderScript "$(to_win_path "$cmd_script")" > "$log" 2>&1
status=$?

cat "$log"

if [[ $status -ne 0 ]]; then
    echo "J-Link flash failed (exit=$status). See $log"
    exit "$status"
fi

if ! grep -Eqi 'O\.K\.|Script processing completed' "$log"; then
    echo "J-Link flash may have failed. See $log"
    exit 1
fi

echo "Flash completed."
