#!/usr/bin/env bash
# Hot-deploy TeamCode to the Control Hub with Sloth (README "Deploy with Sloth"), failing within seconds when the
# hub can't be reached. Sloth's own adb handling waits about two minutes in `adb connect` for a hub that isn't
# there, and while a stale "offline" entry from before a hub reboot is listed it skips connecting and fails.
#
#   tools/deploy.sh           build and push TeamCode; refuses while an OpMode is writing its log
#   tools/deploy.sh --force   push even while an OpMode runs
#
# Overrides: ROBOT_IP, ROBOT_ADB_PORT, ADB. Other arguments go to Gradle.
set -euo pipefail
cd "$(dirname "$0")/.."
SECONDS=0

ROBOT_IP="${ROBOT_IP:-192.168.43.1}"
SERIAL="$ROBOT_IP:${ROBOT_ADB_PORT:-5555}"
force=0
if [ "${1:-}" = "--force" ]; then force=1; shift; fi

# The JDK verify.sh uses, so both reuse one warm Gradle daemon
if [ -z "${JAVA_HOME:-}" ] && [ -d "$HOME/.local/jdk17-dl/full/jdk-17.0.21+9" ]; then
  export JAVA_HOME="$HOME/.local/jdk17-dl/full/jdk-17.0.21+9"
fi

# The SDK's adb, as Sloth uses: a second adb version restarts the adb server on every switch
if [ -z "${ADB:-}" ]; then
  sdk="$(sed -n 's/^sdk\.dir=//p' local.properties 2>/dev/null || true)"
  sdk="${sdk:-${ANDROID_HOME:-${ANDROID_SDK_ROOT:-}}}"
  if [ -n "$sdk" ] && [ -x "$sdk/platform-tools/adb" ]; then ADB="$sdk/platform-tools/adb"; else ADB=adb; fi
fi

if ! ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1; then
  echo "Can't reach the robot at $ROBOT_IP. Join the 29550-RC Wi-Fi (tools/start_robot.sh does) and try again."
  exit 1
fi

state="$("$ADB" -s "$SERIAL" get-state 2>/dev/null || true)"
if [ "$state" != device ]; then
  "$ADB" disconnect "$SERIAL" >/dev/null 2>&1 || true
  case "$(timeout 10 "$ADB" connect "$SERIAL" 2>&1 || true)" in
    *"connected to"*) timeout 5 "$ADB" -s "$SERIAL" wait-for-device >/dev/null 2>&1 || true ;;
  esac
  state="$("$ADB" -s "$SERIAL" get-state 2>/dev/null || true)"
fi
if [ "$state" != device ]; then
  echo "adb can't use the hub at $SERIAL (state: ${state:-not connected}). Check the hub has finished booting."
  exit 1
fi

if [ "$force" = 0 ]; then
  # PsiKit appends to the newest .rlog while an OpMode runs; only a file touched in the last 10 s needs the 1 s check
  # shellcheck disable=SC2016 # expands on the hub
  running="$("$ADB" -s "$SERIAL" shell 'f=$(ls -t /sdcard/FIRST/PsiKit/*.rlog 2>/dev/null | head -n 1);
    [ -n "$f" ] || exit 0; [ $(( $(date +%s) - $(stat -c %Y "$f") )) -gt 10 ] && exit 0;
    a=$(stat -c %s "$f"); sleep 1; [ "$a" != "$(stat -c %s "$f")" ] && echo "$f"; exit 0' | tr -d '\r' || true)"
  if [ -n "$running" ]; then
    echo "An OpMode is running ($running is growing). Stop it on the Driver Station, or use --force."
    exit 1
  fi
fi

# Pins Sloth's adb calls to the hub when another device is plugged in
export ANDROID_SERIAL="$SERIAL"
./gradlew :TeamCode:deploySloth --offline "$@"
echo "Deployed in ${SECONDS} s."
