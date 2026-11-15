import asyncio, json, websockets
async def main():
    async with websockets.connect("ws://192.168.43.1:8002", max_size=None, open_timeout=5, ping_interval=None) as ws:
        await ws.send(json.dumps({"pluginID":"com.bylazar.opmodecontrol","messageID":"stopActiveOpMode","data":None}))
        print("sent stop")
        try:
            for _ in range(20):
                m = json.loads(await asyncio.wait_for(ws.recv(), 3))
                if m.get("messageID")=="activeOpMode": print(str(m.get("data"))[:200])
        except Exception as e: print("end", type(e).__name__)
asyncio.run(main())
