#!/usr/bin/env bash
# Every check a refactor commit must pass: the debug APK builds, all unit tests pass (including the constants
# snapshot), and no log key changed. Run from anywhere in the repo.
set -euo pipefail
cd "$(dirname "$0")/.."
if [ -z "${JAVA_HOME:-}" ] && [ -d "$HOME/.local/jdk17-dl/full/jdk-17.0.21+9" ]; then
  export JAVA_HOME="$HOME/.local/jdk17-dl/full/jdk-17.0.21+9"
fi
./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest --offline -q
python3 - <<'PY'
import glob, re
t = f = 0
for p in glob.glob("TeamCode/build/test-results/testDebugUnitTest/*.xml"):
    m = re.search(r'tests="(\d+)".*?failures="(\d+)".*?errors="(\d+)"', open(p).read())
    t += int(m[1]); f += int(m[2]) + int(m[3])
print(f"unit tests: {t} run, {f} failed")
PY
python3 tools/check_log_keys.py
