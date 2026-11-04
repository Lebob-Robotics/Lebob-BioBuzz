#!/usr/bin/env python3
"""Drive the robot from a controller plugged into this PC, through Panels, without the browser.

The Panels gamepad widget only sends when its dropdown is set to a gamepad and the browser can see
the controller. This script reads the controller from /dev/input/js0 (Linux joystick API) and sends
the same "gamepad0" messages the widget sends, 25 times a second.

  python3 tools/panels_gamepad_bridge.py [--device /dev/input/js0] [--robot 192.168.43.1]

Needs `pip install websockets`, a TeleOp built with Main.PANELS_ENABLED, and the PC on the robot's
Wi-Fi. Stop with Ctrl+C: the robot drops Panels input it has not heard for 60 ms, so it stops too.
Mapping is for a DualSense on the hid-playstation driver.
"""
import argparse
import asyncio
import json
import os
import struct
import time

import websockets

JS_EVENT = struct.Struct("<IhBB")  # time ms, value, type, number
JS_BUTTON, JS_AXIS, JS_INIT = 0x01, 0x02, 0x80
SEND_PERIOD_S = 0.04  # Under the robot's 60 ms timeout, with room for Wi-Fi jitter
STICK_REST = 0.06  # Treat smaller readings as centred. This DualSense rests at up to 0.04

# hid-playstation button numbers on the joystick API
BUTTONS = {0: "cross", 1: "circle", 2: "triangle", 3: "square", 4: "l1", 5: "r1",
           8: "share", 9: "options", 10: "ps"}


class Pad:
    def __init__(self):
        self.axes = [0.0] * 8
        self.axes[2] = self.axes[5] = -1.0  # Triggers rest at -1
        self.buttons = {}

    def apply(self, value, kind, number):
        kind &= ~JS_INIT
        if kind == JS_AXIS and number < len(self.axes):
            self.axes[number] = value / 32767.0
        elif kind == JS_BUTTON:
            self.buttons[number] = value != 0

    def message(self):
        def stick(v):
            return 0.0 if abs(v) < STICK_REST else round(v, 3)

        def trigger(v):
            return round(max(0.0, (v + 1.0) / 2.0), 3)

        a = self.axes
        data = {
            "l2": trigger(a[2]), "r2": trigger(a[5]),
            # Same sign as the browser Gamepad API (down and right positive), which the plugin expects
            "leftStick": {"x": stick(a[0]), "y": stick(a[1]), "value": self.buttons.get(11, False)},
            "rightStick": {"x": stick(a[3]), "y": stick(a[4]), "value": self.buttons.get(12, False)},
            "dpad_left": a[6] < -0.5, "dpad_right": a[6] > 0.5,
            "dpad_up": a[7] < -0.5, "dpad_down": a[7] > 0.5,
            "touchpad": False,
        }
        for number, name in BUTTONS.items():
            data[name] = self.buttons.get(number, False)
        return json.dumps({"pluginID": "com.bylazar.gamepad", "messageID": "gamepad0", "data": data})


async def read_controller(device, pad):
    loop = asyncio.get_running_loop()
    fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
    ready = asyncio.Event()
    loop.add_reader(fd, ready.set)
    try:
        while True:
            await ready.wait()
            ready.clear()
            while True:
                try:
                    chunk = os.read(fd, JS_EVENT.size * 64)
                except BlockingIOError:
                    break
                if not chunk:
                    raise OSError("controller unplugged")
                for i in range(0, len(chunk) - JS_EVENT.size + 1, JS_EVENT.size):
                    _, value, kind, number = JS_EVENT.unpack_from(chunk, i)
                    pad.apply(value, kind, number)
    finally:
        loop.remove_reader(fd)
        os.close(fd)


async def send(robot, pad):
    url = f"ws://{robot}:8002"
    while True:
        try:
            async with websockets.connect(url, max_size=None, open_timeout=3, ping_interval=None)  # Panels never answers pings as ws:
                print(f"Connected to {url}. Driving.")
                last_print = 0.0
                while True:
                    await ws.send(pad.message())
                    now = time.monotonic()
                    if now - last_print > 1.0:
                        a = pad.axes
                        print(f"  left ({a[0]:+.2f}, {a[1]:+.2f})  right x {a[3]:+.2f}  "
                              f"LT {max(0, (a[2] + 1) / 2):.2f}  RT {max(0, (a[5] + 1) / 2):.2f}", flush=True)
                        last_print = now
                    await asyncio.sleep(SEND_PERIOD_S)
        except (OSError, websockets.WebSocketException, asyncio.TimeoutError) as e:
            print(f"Robot not reachable ({e}); retrying in 1 s")
            await asyncio.sleep(1.0)


async def main():
    parser = argparse.ArgumentParser(description="Drive the robot from a PC controller through Panels")
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--robot", default="192.168.43.1")
    args = parser.parse_args()
    pad = Pad()
    reader = asyncio.create_task(read_controller(args.device, pad))
    sender = asyncio.create_task(send(args.robot, pad))
    done, _ = await asyncio.wait({reader, sender}, return_when=asyncio.FIRST_EXCEPTION)
    for task in done:
        task.result()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
