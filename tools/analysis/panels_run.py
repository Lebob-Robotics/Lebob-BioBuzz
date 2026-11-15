import asyncio, json, sys, time
import websockets
NAME = sys.argv[1]; RUN_S = float(sys.argv[2])
def msg(mid, data=None):
    return json.dumps({"pluginID": "com.bylazar.opmodecontrol", "messageID": mid, "data": data})
async def main():
    async with websockets.connect("ws://192.168.43.1:8002", max_size=None, open_timeout=5, ping_interval=None) as ws:
        async def listen():
            try:
                async for m in ws:
                    try: j = json.loads(m)
                    except Exception: continue
                    if j.get("pluginID") == "com.bylazar.opmodecontrol" and j.get("messageID") != "opModesList":
                        print(f"{time.monotonic()-t0:6.1f} rx {j.get('messageID')} {str(j.get('data'))[:200]}", flush=True)
            except Exception as e:
                print("listen end", e)
        t0 = time.monotonic()
        task = asyncio.create_task(listen())
        await asyncio.sleep(1)
        await ws.send(msg("initOpMode", NAME)); print(f"{time.monotonic()-t0:6.1f} sent init", flush=True)
        await asyncio.sleep(6)
        await ws.send(msg("startActiveOpMode")); print(f"{time.monotonic()-t0:6.1f} sent start", flush=True)
        await asyncio.sleep(RUN_S)
        await ws.send(msg("stopActiveOpMode")); print(f"{time.monotonic()-t0:6.1f} sent stop", flush=True)
        await asyncio.sleep(2)
        task.cancel()
asyncio.run(main())
