"""
Smartphone teleoperation web server.

Serves a slider-based control page to the phone browser and forwards
joint position commands to /leader/joint_commands via ROS2.

Usage:
    python3 phone_teleop_server.py --port 8765 --num-joints 6
"""
from __future__ import annotations

import argparse
import asyncio
import json
import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Inline HTML served to the phone (no external CDN dependencies)
_HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<title>Arm Teleop</title>
<style>
  body {{ font-family: system-ui, sans-serif; background: #111; color: #eee;
          padding: 12px; max-width: 480px; margin: auto; }}
  h2 {{ font-size: 1.1rem; margin-bottom: 4px; }}
  #status {{ font-size: 0.8rem; color: #aaa; margin-bottom: 16px; }}
  .joint {{ margin-bottom: 14px; }}
  .joint label {{ display: flex; justify-content: space-between; font-size: 0.85rem; }}
  .joint input[type=range] {{ width: 100%; accent-color: #4af; }}
  #send-btn {{ width: 100%; padding: 14px; font-size: 1rem; background: #4af;
               border: none; border-radius: 8px; color: #000; cursor: pointer;
               margin-top: 8px; }}
  #send-btn:active {{ opacity: 0.7; }}
  #hold-label {{ font-size: 0.8rem; color: #aaa; margin-top: 8px; text-align: center; }}
</style>
</head>
<body>
<h2>Isaac Sim Arm Teleop</h2>
<div id="status">Connecting...</div>
<div id="joints"></div>
<button id="send-btn" onclick="sendOnce()">Send Now</button>
<div id="hold-label">Or move sliders — auto-sends at 10 Hz</div>
<script>
const NUM_JOINTS = {num_joints};
const JOINT_NAMES = {joint_names_json};
const WS_URL = "ws://" + location.hostname + ":{port}/ws";
// Foxglove bridge for visualization: ws://&lt;host&gt;:8765

let ws = null;
let sliders = [];
let autoSendTimer = null;

function buildUI() {{
  const container = document.getElementById("joints");
  for (let i = 0; i < NUM_JOINTS; i++) {{
    const div = document.createElement("div");
    div.className = "joint";
    const name = JOINT_NAMES[i] || ("joint_" + i);
    div.innerHTML = `
      <label><span>${{name}}</span><span id="val${{i}}">0.00 rad</span></label>
      <input type="range" id="sl${{i}}" min="-3.14" max="3.14" step="0.01" value="0"
             oninput="onSlider(${{i}}, this.value)">`;
    container.appendChild(div);
    sliders.push(div.querySelector("input"));
  }}
}}

function onSlider(i, val) {{
  document.getElementById("val" + i).textContent = parseFloat(val).toFixed(2) + " rad";
}}

function getPositions() {{
  return sliders.map(s => parseFloat(s.value));
}}

function sendOnce() {{
  if (ws && ws.readyState === WebSocket.OPEN) {{
    ws.send(JSON.stringify({{ positions: getPositions() }}));
  }}
}}

function connect() {{
  ws = new WebSocket(WS_URL);
  ws.onopen = () => {{
    document.getElementById("status").textContent = "Connected ✓";
    autoSendTimer = setInterval(sendOnce, 100);  // 10 Hz
  }};
  ws.onclose = () => {{
    document.getElementById("status").textContent = "Disconnected — retrying...";
    clearInterval(autoSendTimer);
    setTimeout(connect, 2000);
  }};
  ws.onerror = () => ws.close();
}}

buildUI();
connect();
</script>
</body>
</html>
"""


class TeleopPublisherNode(Node):
    def __init__(self, topic: str):
        super().__init__("phone_teleop_server")
        self._pub = self.create_publisher(Float64MultiArray, topic, 10)

    def publish_positions(self, positions: list[float]):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in positions]
        self._pub.publish(msg)


def get_local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


async def handle_ws(request, node: TeleopPublisherNode):
    from aiohttp import web
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    async for msg in ws:
        if msg.type == 1:  # WSMsgType.TEXT
            try:
                data = json.loads(msg.data)
                positions = data.get("positions", [])
                if positions:
                    node.publish_positions(positions)
            except (json.JSONDecodeError, KeyError):
                pass
    return ws


async def main_async(args, node: TeleopPublisherNode):
    from aiohttp import web

    joint_names = [f"joint_{i}" for i in range(args.num_joints)]
    html = _HTML_TEMPLATE.format(
        num_joints=args.num_joints,
        joint_names_json=json.dumps(joint_names),
        port=args.port,
    )

    async def handle_index(_request):
        return web.Response(text=html, content_type="text/html")

    app = web.Application()
    app.router.add_get("/", handle_index)
    app.router.add_get("/ws", lambda r: handle_ws(r, node))

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", args.port)
    await site.start()

    local_ip = get_local_ip()
    print(f"[phone_teleop_server] Serving at http://{local_ip}:{args.port}")
    print(f"[phone_teleop_server] Open this URL on your phone (same WiFi network).")
    print(f"[phone_teleop_server] Publishing to: {args.topic}")

    try:
        await asyncio.get_event_loop().run_in_executor(None, lambda: threading.Event().wait())
    except asyncio.CancelledError:
        pass
    finally:
        await runner.cleanup()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--num-joints", type=int, default=6)
    parser.add_argument("--topic", default="/leader/joint_commands")
    parser.add_argument("--ros-domain-id", type=int, default=42)
    args = parser.parse_args()

    import os
    os.environ.setdefault("ROS_DOMAIN_ID", str(args.ros_domain_id))

    try:
        rclpy.init()
    except RuntimeError:
        pass

    node = TeleopPublisherNode(args.topic)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        asyncio.run(main_async(args, node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
