#!/bin/sh
# Pull the newest match log from the Control Hub and print the exposure report.
#   tools/vision/pull_and_report.sh            newest log of any OpMode
#   tools/vision/pull_and_report.sh Main_log   newest TeleOp log (ScoringAuto for the auto)
# The hub's clock resets each boot, so "newest" is by inode, not file name. Logs land in tools/vision/logs/.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
HUB="${ROBOT_IP:-192.168.43.1}:${ROBOT_ADB_PORT:-5555}"
adb connect "$HUB" >/dev/null 2>&1 || true
f=$(adb -s "$HUB" shell "ls -1i /sdcard/FIRST/PsiKit/" | tr -d '\r' | grep "${1:-rlog}" | sort -n | tail -1 | awk '{print $2}')
[ -n "$f" ] || { echo "no log matching '${1:-rlog}' on the hub" >&2; exit 1; }
mkdir -p "$HERE/logs"
adb -s "$HUB" pull "/sdcard/FIRST/PsiKit/$f" "$HERE/logs/" >/dev/null
echo "pulled $HERE/logs/$f"
python3 "$HERE/exposure_report.py" "$HERE/logs/$f"
