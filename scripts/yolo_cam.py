#!/usr/bin/env python3
"""
Object detection on the robot's camera: reads the Pi's MJPEG stream, runs YOLO on the notebook and
serves the annotated video plus the detections:
  http://localhost:8001/            annotated video (MJPEG, also opens on a phone via the notebook's IP)
  http://localhost:8001/detections  latest detections as JSON (label, confidence, box in pixels)

  ~/.venvs/yolo/bin/python scripts/yolo_cam.py [--source http://picam.local:8000/stream.mjpg] [--model yolo11n.pt] [--device auto|cpu|intel:gpu]
(pip install ultralytics opencv-python-headless openvino; OpenVINO is optional)
"""
import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# low-latency reading of the MJPEG stream (no ffmpeg input buffering)
os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'fflags;nobuffer|flags;low_delay')
import cv2  # noqa: E402
import torch  # noqa: E402
from ultralytics import YOLO  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('--source', default='http://picam.local:8000/stream.mjpg')
p.add_argument('--model', default='yolo11n.pt')
p.add_argument('--conf', type=float, default=0.4)
p.add_argument('--port', type=int, default=8001)
# 416 px: ~33 ms per frame on an i7-11390H (640: ~58 ms), little loss for furniture-sized objects
p.add_argument('--imgsz', type=int, default=416)
# leave cores for the browser (3D view, SLAM): with all of them YOLO and the page slow each other down
p.add_argument('--threads', type=int, default=4)
# 'auto': OpenVINO on the Intel GPU when available (frees the CPU for the page; on an Iris Xe ~70 ms vs
# ~111 ms for PyTorch on 4 threads with the notebook busy), else PyTorch on the CPU
p.add_argument('--device', default='auto')
args = p.parse_args()
torch.set_num_threads(args.threads)


def pick_model():
    if args.device != 'auto' or not args.model.endswith('.pt'):
        return YOLO(args.model), (None if args.device == 'auto' else args.device)
    try:
        import openvino as ov

        if 'GPU' not in ov.Core().available_devices:
            raise RuntimeError('no Intel GPU')
        path = args.model[:-3] + '_openvino_model'
        if not os.path.isdir(path):
            YOLO(args.model).export(format='openvino', imgsz=args.imgsz)
        print('YOLO on the Intel GPU (OpenVINO)')
        return YOLO(path, task='detect'), 'intel:gpu'
    except Exception as e:
        print('YOLO on the CPU (PyTorch):', e)
        return YOLO(args.model), None


model, device = pick_model()

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
        t0 = time.time()
        r = model(frame, conf=args.conf, imgsz=args.imgsz, verbose=False, **({'device': device} if device else {}))[0]
        infer_ms = (time.time() - t0) * 1000
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
            state['ms'] = 0.8 * state.get('ms', infer_ms) + 0.2 * infer_ms
            state['at'] = now
            cond.notify_all()
        last = now


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/detections':
            with cond:
                body = json.dumps({
                    'fps': round(state['fps'], 1),
                    'ms': round(state.get('ms', 0)),
                    # seconds since these detections were made
                    'age': round(time.time() - state.get('at', time.time()), 2),
                    'size': state['size'],
                    'detections': state['detections'],
                }).encode()
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
