#!/usr/bin/env bash
# Flash BMS_Project.elf via J-Link (1000 kHz — more reliable on BMS board).
# Usage: flash_current_elf.sh [workspace_dir] [JLink.exe path]
set -euo pipefail

workspace_dir="${1:-/home/alan/projects/bms_project_remote}"
jlink_exe="${2:-/mnt/c/Program Files/SEGGER/JLink_V910/JLink.exe}"
device="STM32U385CG"
elf_path="${workspace_dir}/build/Debug/BMS_Project.elf"

if [[ ! -f "$jlink_exe" ]]; then
  jlink_exe="/mnt/c/Program Files/SEGGER/JLink_V964/JLink.exe"
fi

if [[ ! -f "$elf_path" ]]; then
  echo "ELF not found: $elf_path" >&2
  echo "Build first: cmake --build ${workspace_dir}/build/Debug" >&2
  exit 1
fi

to_win_path() {
  local p="$1"
  if command -v wslpath >/dev/null 2>&1; then
    wslpath -w "$p"
  else
    echo "$p"
  fi
}

script_dir="${workspace_dir}/build"
mkdir -p "$script_dir"
cmd_script="${script_dir}/jlink_flash.jlink"
log="${script_dir}/jlink_flash.log"
elf_win="$(to_win_path "$elf_path")"

cat > "$cmd_script" <<EOF
device ${device}
si SWD
speed 1000
connect
r
h
loadfile ${elf_win}
verifybin ${elf_win}, 0x08000000
r
g
q
EOF

: > "$log"
echo "Flashing $elf_path @ 1000 kHz"
/init "$jlink_exe" . -CommanderScript "$(to_win_path "$cmd_script")" > "$log" 2>&1
status=$?

cat "$log"

if [[ $status -ne 0 ]]; then
  echo "J-Link flash failed (exit=$status). See $log" >&2
  exit "$status"
fi

if ! grep -Eqi 'Downloading file|O\.K\.' "$log"; then
  echo "Flash download may have failed — check log for 'O.K.'" >&2
  exit 1
fi

echo "Flash completed. Power-cycle BMS 24V, wait 5s, then run watch_lin_status.sh bms"
