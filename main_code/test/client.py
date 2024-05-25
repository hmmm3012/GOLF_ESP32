import websocket

ws = websocket.WebSocket()
ws.connect("ws://192.168.2.144:80/automatic")
ws.send("Hello, Server")
