#!/usr/bin/env python3
# MJPEG camera server for the Raspberry Pi on the robot (legacy camera stack: picamera v1, Python 3.5+,
# so no f-strings). Open http://picam.local:8000/ in a browser, or read /stream.mjpg from the YOLO script.
#   python3 camera_server.py [width height fps]
import io
import json
import logging
import threading
import time
import socketserver
import sys
from http import server
from threading import Condition
from urllib.parse import parse_qs, urlparse

import picamera

WIDTH, HEIGHT, FPS = (int(a) for a in sys.argv[1:4]) if len(sys.argv) >= 4 else (640, 480, 24)

PAGE = """<html><head><title>picam</title><meta name="viewport" content="width=device-width"></head>
<body style="margin:0;background:#000"><img src="stream.mjpg" style="width:100%"></body></html>"""


class Frames(object):
    """Keeps the latest JPEG frame; each client waits for the next one."""

    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # a new frame starts: publish the previous one
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)


class Handler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        url = urlparse(self.path)
        if url.path == '/mode':
            # /mode?set=auto|day|night → {"mode": ..., "night": bool, "brightness": ...}
            want = parse_qs(url.query).get('set', [None])[0]
            if want in ('auto', 'day', 'night'):
                night_mode['mode'] = want
            body = json.dumps({'mode': night_mode['mode'], 'night': night_mode['night'], 'brightness': night_mode['brightness']}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path in ('/', '/index.html'):
            body = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            # the robot page (another origin) draws the frames on a canvas
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with frames.condition:
                        frames.condition.wait()
                        frame = frames.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.info('client %s left: %s', self.client_address, e)
        else:
            self.send_error(404)
            self.end_headers()

    def log_message(self, *args):
        pass


class Server(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


# night mode: in the dark (an IR illuminator for the NoIR camera, or just dim light) longer exposures,
# higher gain, fewer frames per second and black and white (in infrared there are no colors anyway)
night_mode = {'mode': 'auto', 'night': False, 'brightness': 128}


def set_night(camera, on):
    if on == night_mode['night']:
        return
    night_mode['night'] = on
    camera.stop_recording()
    if on:
        camera.framerate = 10
        camera.iso = 800
        camera.exposure_mode = 'night'
        camera.color_effects = (128, 128)
    else:
        camera.framerate = FPS
        camera.iso = 0
        camera.exposure_mode = 'auto'
        camera.color_effects = None
    camera.start_recording(frames, format='mjpeg', quality=15)
    logging.info('night mode %s', 'on' if on else 'off')


def watch_light(camera):
    """Measures the scene's brightness from a small still and switches modes (with hysteresis)."""
    small = io.BytesIO()
    while True:
        time.sleep(3)
        try:
            small.seek(0)
            small.truncate()
            camera.capture(small, format='yuv', resize=(64, 48), use_video_port=True)
            y = small.getvalue()[: 64 * 48]
            night_mode['brightness'] = sum(y) // len(y)
        except Exception as e:
            logging.info('brightness: %s', e)
            continue
        m = night_mode['mode']
        b = night_mode['brightness']
        if m == 'night' or (m == 'auto' and not night_mode['night'] and b < 35):
            set_night(camera, True)
        elif m == 'day' or (m == 'auto' and night_mode['night'] and b > 90):
            set_night(camera, False)


logging.basicConfig(level=logging.INFO)
with picamera.PiCamera(resolution=(WIDTH, HEIGHT), framerate=FPS) as camera:
    frames = Frames()
    camera.start_recording(frames, format='mjpeg', quality=15)
    t = threading.Thread(target=watch_light, args=(camera,))
    t.daemon = True
    t.start()
    try:
        logging.info('streaming %dx%d at %d fps on http://picam.local:8000/', WIDTH, HEIGHT, FPS)
        Server(('', 8000), Handler).serve_forever()
    finally:
        camera.stop_recording()
