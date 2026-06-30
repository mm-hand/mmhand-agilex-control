from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from .config import FrameTransformConfig, RosConfig, VisualizationConfig
from .frames import build_coordinate_frames
from .ros_io import PicoPoseSubscriber

AXIS_COLORS = ("#ff0000", "#00aa00", "#0066ff")

_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>Control Frames</title>
<style>body{margin:0;background:#111;color:#eee;font:14px monospace}
canvas{display:block;width:100vw;height:100vh}</style></head>
<body><canvas id="c"></canvas><script>
const c=document.getElementById('c'),x=c.getContext('2d');
function rs(){c.width=innerWidth;c.height=innerHeight}rs();addEventListener('resize',rs);
const COL=['#ff0000','#00aa00','#0066ff'],SCALE=400,CX=innerWidth/2,CY=innerHeight/2;
function proj(p){return [CX+(p[0]*0.866-p[2]*0.866)*SCALE, CY+(-p[1]+(p[0]+p[2])*0.5)*SCALE]}
function frame(t,name){
  const o=t[0];
  for(let i=0;i<3;i++){const e=[o[0]+t[1][i][0],o[1]+t[1][i][1],o[2]+t[1][i][2]];
    const [sx,sy]=proj(o),[ex,ey]=proj(e);
    x.strokeStyle=COL[i];x.lineWidth=2;x.beginPath();x.moveTo(sx,sy);x.lineTo(ex,ey);x.stroke()}
  const [lx,ly]=proj(o);x.fillStyle='#eee';x.font='13px monospace';
  x.fillText(name,lx+5,ly-5)}
let latest=null;
const es=new EventSource('/stream');
es.onmessage=e=>{latest=JSON.parse(e.data)};
function draw(){
  x.clearRect(0,0,c.width,c.height);
  if(latest){for(const f of latest) frame(f.t,f.n)}
  requestAnimationFrame(draw)}
draw();
</script></body></html>"""


class _State:
    def __init__(self) -> None:
        self.latest: list[dict] = []
        self.lock = threading.Lock()

    def set(self, payload: list[dict]) -> None:
        with self.lock:
            self.latest = payload


def _frames_to_payload(frames, vis_config: VisualizationConfig) -> list[dict]:
    payload: list[dict] = []
    L = vis_config.axis_length_m

    def add(transform: np.ndarray, name: str) -> None:
        origin = transform[:3, 3].tolist()
        axes = (transform[:3, 0].tolist(), transform[:3, 1].tolist(), transform[:3, 2].tolist())
        scaled_axes = ([[v * L for v in ax] for ax in axes])
        payload.append({"n": name, "t": [origin, scaled_axes]})

    add(np.eye(4, dtype=float), "PICO_HORIZON")
    if frames.left is not None:
        add(frames.left.transform_pico_horizon_from_arm_base, "ROBOT_BASE_L")
        add(frames.left.transform_pico_horizon_from_pico_controller, "PICO_CONTROLLER_L")
        add(frames.left.transform_pico_horizon_from_flange, "FLANGE_L")
    if frames.right is not None:
        add(frames.right.transform_pico_horizon_from_arm_base, "ROBOT_BASE_R")
        add(frames.right.transform_pico_horizon_from_pico_controller, "PICO_CONTROLLER_R")
        add(frames.right.transform_pico_horizon_from_flange, "FLANGE_R")
    return payload


def run_frames(
    ros_config: RosConfig,
    frame_config: FrameTransformConfig,
    vis_config: VisualizationConfig,
) -> int:
    state = _State()

    sub = PicoPoseSubscriber(ros_config)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args) -> None:
            pass

        def do_GET(self) -> None:
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(_HTML.encode("utf-8"))
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                while True:
                    with state.lock:
                        payload = state.latest
                    if payload:
                        self.wfile.write(f"data: {json.dumps(payload)}\n\n".encode("utf-8"))
                        self.wfile.flush()
                    time.sleep(vis_config.refresh_interval_s)
            else:
                self.send_response(404)
                self.end_headers()

    def _ros_loop() -> None:
        while True:
            sub.spin()
            frames = build_coordinate_frames(sub.get_left(), sub.get_right(), frame_config)
            if frames.left is not None or frames.right is not None:
                state.set(_frames_to_payload(frames, vis_config))
            time.sleep(vis_config.refresh_interval_s)

    threading.Thread(target=_ros_loop, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", 8080), Handler)
    print(f"http://127.0.0.1:8080", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
        server.shutdown()
    return 0
