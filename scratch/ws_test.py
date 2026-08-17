import asyncio
import websockets
import json

async def test_ws():
    uri = "ws://127.0.0.1:8000/ws/stream/aero"
    async with websockets.connect(uri) as websocket:
        print("Connected!")
        payload = {
            "command": "calculate",
            "data": {
                "mach": 0.8,
                "aoa": 2.0,
                "altitude": 10000.0,
                "cfl": 1.5,
                "span": 10.0,
                "rootChord": 1.0,
                "tipScale": 0.5,
                "wingSweep": 15.0
            }
        }
        await websocket.send(json.dumps(payload))
        print("Sent payload, waiting for response...")
        try:
            response = await websocket.recv()
            print("Received response!")
        except Exception as e:
            print("Error receiving:", e)

if __name__ == "__main__":
    asyncio.run(test_ws())
