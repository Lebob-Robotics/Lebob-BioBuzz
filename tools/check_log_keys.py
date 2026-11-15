#!/usr/bin/env python3
"""List every PsiKit log key the TeamCode source writes, and compare against tools/log-keys.txt.

Log keys feed AdvantageScope layouts and the analysis scripts, so a refactor must not rename or drop any.
Keys built from a variable prefix are listed as <expr>/rest: the prefix's value isn't checked.

    python3 tools/check_log_keys.py           # compare, exit 1 on any difference
    python3 tools/check_log_keys.py --write   # rewrite tools/log-keys.txt from the source
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = ROOT / "TeamCode/src/main/java"
SAVED = ROOT / "tools/log-keys.txt"

CALL = re.compile(r'Logger\.(recordOutput|recordMetadata|processInputs)\(\s*(.*?)\s*,', re.S)
LITERAL = re.compile(r'^"((?:[^"\\]|\\.)*)"$')
PREFIXED = re.compile(r'^([A-Za-z_][\w.()\[\]]*)\s*\+\s*"((?:[^"\\]|\\.)*)"$')


def keys():
    found = set()
    for path in sorted(SRC.rglob("*.java")):
        text = path.read_text(encoding="utf-8")
        for kind, arg in CALL.findall(text):
            arg = " ".join(arg.split())
            m = LITERAL.match(arg)
            if m:
                key = m.group(1)
            else:
                m = PREFIXED.match(arg)
                key = "<expr>" + m.group(2) if m else "<expr>"
            found.add(("meta " if kind == "recordMetadata" else "input " if kind == "processInputs" else "") + key)
    return sorted(found)


def main():
    now = keys()
    if "--write" in sys.argv:
        SAVED.write_text("\n".join(now) + "\n", encoding="utf-8")
        print(f"wrote {len(now)} keys to {SAVED.relative_to(ROOT)}")
        return 0
    saved = SAVED.read_text(encoding="utf-8").split()
    saved = [line for line in SAVED.read_text(encoding="utf-8").splitlines() if line]
    gone = sorted(set(saved) - set(now))
    new = sorted(set(now) - set(saved))
    for k in gone:
        print("missing:", k)
    for k in new:
        print("new:    ", k)
    print(f"{len(now)} keys, {len(gone)} missing, {len(new)} new")
    return 1 if gone or new else 0


if __name__ == "__main__":
    sys.exit(main())
