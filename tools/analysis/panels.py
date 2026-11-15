#!/usr/bin/env python3
"""panels.py: Panels websocket helpers for scripted robot runs.
  panels.py cfg [Class[.FIELD]]           print configurables (current values)
  panels.py set Class.FIELD=value ...     set configurables
  panels.py init NAME | start | stop      OpMode control
  panels.py listen SECONDS [plugin]       print messages (default: everything but opModesList)
"""
import asyncio, json, sys, time
import websockets
URL = "ws://192.168.43.1:8002"

async def connect():
    return await websockets.connect(URL, max_size=None, open_timeout=5, ping_interval=None)

async def read_cfg(ws, wait_s=4.0):
    cfg = None; t0 = time.monotonic()
    while time.monotonic() - t0 < wait_s:
        try: m = json.loads(await asyncio.wait_for(ws.recv(), 1))
        except Exception: continue
        if m.get('messageID') == 'configurables' and isinstance(m.get('data'), dict):
            cfg = m['data']; break
    return cfg

async def set_cfg(pairs):
    ws = await connect()
    try:
        cfg = await read_cfg(ws)
        ids = {(c, x['fieldName']): x['id'] for c, xs in cfg.items() for x in xs}
        data = []
        for k, v in pairs.items():
            c, f = k.split('.')
            data.append({"id": ids[(c, f)], "newValueString": str(v)})
        await ws.send(json.dumps({"pluginID": "com.bylazar.configurables", "messageID": "updatedConfigurable", "data": data}))
        await asyncio.sleep(0.8)
    finally:
        await ws.close()
    ws = await connect()
    try:
        cfg = await read_cfg(ws)
        vals = {(c, x['fieldName']): x['value'] for c, xs in cfg.items() for x in xs}
        return {k: vals[tuple(k.split('.'))] for k in pairs}
    finally:
        await ws.close()

async def opmode(mid, data=None):
    ws = await connect()
    try:
        await ws.send(json.dumps({"pluginID": "com.bylazar.opmodecontrol", "messageID": mid, "data": data}))
        await asyncio.sleep(0.5)
    finally:
        await ws.close()

async def listen(seconds, plugin=None):
    ws = await connect(); t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            try: m = json.loads(await asyncio.wait_for(ws.recv(), 1))
            except asyncio.TimeoutError: continue
            if m.get('messageID') == 'opModesList': continue
            if plugin and plugin not in str(m.get('pluginID')): continue
            print(f"{time.monotonic()-t0:6.1f} {m.get('pluginID')} {m.get('messageID')} {json.dumps(m.get('data'))[:1500]}", flush=True)
    finally:
        await ws.close()

def main():
    a = sys.argv[1:]
    if a[0] == 'cfg':
        async def go():
            ws = await connect(); cfg = await read_cfg(ws); await ws.close()
            want = a[1] if len(a) > 1 else None
            for c, xs in sorted(cfg.items()):
                for x in xs:
                    k = f"{c}.{x['fieldName']}"
                    if not want or k.startswith(want): print(k, '=', x['value'])
        asyncio.run(go())
    elif a[0] == 'set':
        print(asyncio.run(set_cfg(dict(p.split('=', 1) for p in a[1:]))))
    elif a[0] == 'init': asyncio.run(opmode('initOpMode', a[1]))
    elif a[0] == 'start': asyncio.run(opmode('startActiveOpMode'))
    elif a[0] == 'stop': asyncio.run(opmode('stopActiveOpMode'))
    elif a[0] == 'listen': asyncio.run(listen(float(a[1]), a[2] if len(a) > 2 else None))

if __name__ == '__main__':
    main()
