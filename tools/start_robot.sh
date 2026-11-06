#!/usr/bin/env bash
# Bring up everything for working with the robot from this PC, safe to run repeatedly:
#   1. robot Wi-Fi (NetworkManager profile "29550-RC", on its own adapter, home internet untouched)
#   2. adb to the Control Hub
#   3. Panels (web UI on :8001)
#   4. the controller bridge, which drives the robot through Panels from the DualSense on this PC
#
#   tools/start_robot.sh          bring everything up
#   tools/start_robot.sh --stop   stop the controller bridge
#
# Overrides: ROBOT_IP, ROBOT_PROFILE, ROBOT_ADB_PORT.
set -u

ROBOT_IP="${ROBOT_IP:-192.168.43.1}"
ROBOT_PROFILE="${ROBOT_PROFILE:-29550-RC}"
ADB_PORT="${ROBOT_ADB_PORT:-5555}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE="$HERE/panels_gamepad_bridge.py"
BRIDGE_LOG=/tmp/panels_bridge.log
BRIDGE_PID=/tmp/panels_bridge.pid

ok() { printf '  [ok]   %s\n' "$*"; }
bad() { printf '  [FAIL] %s\n' "$*"; }
info() { printf '         %s\n' "$*"; }

stop_bridge() {
  if [ -f "$BRIDGE_PID" ] && kill -0 "$(cat "$BRIDGE_PID")" 2>/dev/null; then
    kill "$(cat "$BRIDGE_PID")" 2>/dev/null
    sleep 0.3
  fi
  pkill -f "$BRIDGE" 2>/dev/null
  rm -f "$BRIDGE_PID"
}

if [ "${1:-}" = "--stop" ]; then
  stop_bridge
  echo "controller bridge stopped"
  exit 0
fi

reachable() { ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1; }

echo "Robot Wi-Fi"
if reachable; then
  ok "$ROBOT_IP answers"
else
  iface="$(nmcli -g connection.interface-name connection show "$ROBOT_PROFILE" 2>/dev/null)"
  info "joining '$ROBOT_PROFILE'${iface:+ on $iface}"
  nmcli --wait 30 connection up "$ROBOT_PROFILE" ${iface:+ifname "$iface"} >/dev/null 2>&1
  for _ in 1 2 3 4 5; do reachable && break; sleep 1; done
  if reachable; then
    ok "connected, $ROBOT_IP answers"
  else
    bad "cannot reach the robot at $ROBOT_IP"
    if ! nmcli -f SSID device wifi list --rescan yes 2>/dev/null | grep -q "$ROBOT_PROFILE"; then
      info "'$ROBOT_PROFILE' is not visible to any adapter: is the robot powered on?"
    elif [ -n "$iface" ]; then
      info "'$ROBOT_PROFILE' is visible but $iface could not join it."
      info "A 2.4 GHz-only adapter cannot see a hub running on 5 GHz."
    fi
    exit 1
  fi
fi

echo "adb"
adb connect "$ROBOT_IP:$ADB_PORT" >/dev/null 2>&1
if adb devices | grep -q "^$ROBOT_IP:$ADB_PORT[[:space:]]*device"; then
  ok "adb connected to $ROBOT_IP:$ADB_PORT"
  if [ -n "$(adb -s "$ROBOT_IP:$ADB_PORT" shell pidof com.qualcomm.ftcrobotcontroller 2>/dev/null | tr -d '\r')" ]; then
    ok "Robot Controller app is running"
  else
    bad "Robot Controller app is not running on the hub"
  fi
else
  bad "adb could not connect to $ROBOT_IP:$ADB_PORT"
fi

echo "Panels"
if curl -s -m 5 -o /dev/null "http://$ROBOT_IP:8001/"; then
  ok "web UI at http://$ROBOT_IP:8001"
else
  bad "Panels web UI not answering on :8001 (is the Robot Controller app running?)"
  exit 1
fi

echo "Controller bridge"
pad=""
for dev in /sys/class/input/js*; do
  [ -e "$dev/device/name" ] || continue
  case "$(cat "$dev/device/name")" in
    *"Wireless Controller") pad="/dev/input/$(basename "$dev")"; break ;;
  esac
done
if [ -z "$pad" ]; then
  bad "no DualSense found: connect it to this PC, then run this again"
  exit 1
fi
stop_bridge
: > "$BRIDGE_LOG"
setsid nohup python3 "$BRIDGE" --device "$pad" --robot "$ROBOT_IP" >"$BRIDGE_LOG" 2>&1 &
echo $! > "$BRIDGE_PID"
for _ in $(seq 1 20); do
  grep -q "Connected to" "$BRIDGE_LOG" && break
  kill -0 "$(cat "$BRIDGE_PID")" 2>/dev/null || break
  sleep 0.5
done
if grep -q "Connected to" "$BRIDGE_LOG"; then
  ok "driving from $pad (pid $(cat "$BRIDGE_PID"), log $BRIDGE_LOG)"
  info "turn off the Panels gamepad widget in the browser, or the two will fight"
else
  bad "bridge did not connect; last log lines:"
  tail -5 "$BRIDGE_LOG" | sed 's/^/         /'
  exit 1
fi

echo
echo "Everything is up."
