#!/usr/bin/env python3
"""
Object detection on the robot's camera: reads the Pi's MJPEG stream, runs YOLO on the notebook and
serves the annotated video plus the detections:
  http://localhost:8001/            annotated video (MJPEG, also opens on a phone via the notebook's IP)
  http://localhost:8001/detections  latest detections as JSON (label, confidence, box in pixels)

  ~/.venvs/yolo/bin/python scripts/yolo_cam.py [--source http://picam.local:8000/stream.mjpg] [--model yolo11n.pt]
"""
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
from ultralytics import YOLO

p = argparse.ArgumentParser()
p.add_argument('--source', default='http://picam.local:8000/stream.mjpg')
p.add_argument('--model', default='yolo11n.pt')
p.add_argument('--conf', type=float, default=0.4)
p.add_argument('--port', type=int, default=8001)
args = p.parse_args()

model = YOLO(args.model)
state = {'jpeg': None, 'detections': [], 'fps': 0.0, 'size': [0, 0]}
cond = threading.Condition()


latest = {'frame': None, 'seq': 0}
latest_cond = threading.Condition()


def reader_loop():
    """Keeps only the newest frame, so a detector slower than the camera never falls behind."""
    while True:
        cap = cv2.VideoCapture(args.source)
        if not cap.isOpened():
            print('no stream at', args.source, '- retrying')
            time.sleep(2)
            continue
        while True:
            ok, frame = cap.read()
            if not ok:
                print('stream lost - reconnecting')
                break
            with latest_cond:
                latest['frame'] = frame
                latest['seq'] += 1
                latest_cond.notify_all()
        cap.release()


def detect_loop():
    seen = 0
    last = time.time()
    while True:
        with latest_cond:
            while latest['seq'] == seen:
                latest_cond.wait()
            frame = latest['frame']
            seen = latest['seq']
        r = model(frame, conf=args.conf, verbose=False)[0]
        dets = [
            {'label': r.names[int(b.cls)], 'conf': round(float(b.conf), 2), 'box': [round(float(v)) for v in b.xyxy[0]]}
            for b in r.boxes
        ]
        ok, jpg = cv2.imencode('.jpg', r.plot(), [cv2.IMWRITE_JPEG_QUALITY, 75])
        now = time.time()
        with cond:
            state['jpeg'] = jpg.tobytes()
            state['detections'] = dets
            state['fps'] = 0.8 * state['fps'] + 0.2 / max(now - last, 1e-3)
            state['size'] = [frame.shape[1], frame.shape[0]]
            cond.notify_all()
        last = now


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/detections':
            with cond:
                body = json.dumps({'fps': round(state['fps'], 1), 'size': state['size'], 'detections': state['detections']}).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)
        elif self.path in ('/', '/stream.mjpg'):
            self.send_response(200)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with cond:
                        cond.wait()
                        jpg = state['jpeg']
                    self.wfile.write(b'--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def log_message(self, *a):
        pass


threading.Thread(target=reader_loop, daemon=True).start()
threading.Thread(target=detect_loop, daemon=True).start()
print(f'YOLO ({args.model}) on {args.source} → http://localhost:{args.port}/  (detections: /detections)')
ThreadingHTTPServer(('', args.port), Handler).serve_forever()
