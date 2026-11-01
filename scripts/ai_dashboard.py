#!/usr/bin/env python3
"""ROS 2 telemetry subscriber and read-only browser dashboard.

Runs within the existing Docker container; only the Docker-mapped loopback port
is visible to the Mac. Does not alter robot control or fake classification data.
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TelemetryStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = None
        self.received = None

    def update(self, message, now=None):
        value = json.loads(message)
        if not isinstance(value, dict) or 'phase' not in value:
            return
        with self.lock:
            self.data = value
            self.received = time.monotonic() if now is None else now

    def snapshot(self):
        with self.lock:
            if self.data is None:
                return {'connected': False, 'age_s': None, 'message': 'Waiting for ROS 2 telemetry'}
            delta = max(0., time.monotonic() - self.received)
            # A stale message should never masquerade as current live telemetry.
            return {'connected': delta < 3.0, 'age_s': round(delta, 1), 'data': self.data}


class DashboardNode(Node):
    def __init__(self, store):
        super().__init__('sense_before_touch_dashboard')
        self.store = store
        self.create_subscription(String, '/rescue_ai/telemetry', self.callback, 10)

    def callback(self, msg):
        try:
            self.store.update(msg.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning('Invalid telemetry ignored: %s' % exc)


def make_handler(store, html_path):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/telemetry':
                body = json.dumps(store.snapshot(), ensure_ascii=True).encode('utf-8')
                mime = 'application/json; charset=utf-8'
            elif path in ('/', '/index.html'):
                body = html_path.read_bytes()
                mime = 'text/html; charset=utf-8'
            elif path == '/health':
                body = b'ok'
                mime = 'text/plain; charset=utf-8'
            else:
                self.send_error(404, 'Not Found')
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, fmt, *args):
            # Avoid flooding the docker compose logs on each dashboard poll.
            return
    return Handler


def main():
    store = TelemetryStore()
    rclpy.init()
    node = DashboardNode(store)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    html_path = Path(__file__).resolve().parent / 'dashboard' / 'index.html'
    server = ThreadingHTTPServer(('0.0.0.0', 6081), make_handler(store, html_path))
    print('[PAI] AI telemetry dashboard ready on :6081', flush=True)
    try:
        server.serve_forever(poll_interval=.5)
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
