import asyncio
import websockets
import json
import random


async def send_bearing(websocket):
    while True:
        # Simule un rel_bearing aléatoire entre 0 et 360°
        bearing = rel_bearing
        data = json.dumps({"rel_bearing": bearing})
        await websocket.send(data)
        await asyncio.sleep(0.5)

async def handler(websocket):
    print("[WS SERVER] Client connecté ✅")
    try:
        await send_bearing(websocket)
    except websockets.ConnectionClosed:
        print("[WS SERVER] Client déconnecté ❌")

async def main():
    async with websockets.serve(handler, "localhost", 8765):
        print("[WS SERVER] Serveur WebSocket en écoute sur ws://localhost:8765")
        await asyncio.Future()  # bloque pour toujours

if __name__ == "__main__":
    asyncio.run(main())