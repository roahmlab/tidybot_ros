import logging
import socket
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
from queue import Queue
import threading
import rclpy
from rclpy.node import Node
from tidybot_utils.msg import TeleopMsg
from std_msgs.msg import String
import json

import os
from ament_index_python.packages import get_package_share_directory
from urllib.request import urlopen, Request
import time

GREEN = "\x1b[32m"
RED = "\x1b[31m"
RESET = "\x1b[0m"
BOLD = "\x1b[1m"

# Ensure the template and static directories are set correctly
pkg_share = get_package_share_directory("tidybot_teleop")
config_path = os.path.join(pkg_share, "config")

class WebServer:
    def __init__(self, queue, record_enabled: bool):
        self.app = Flask(
            __name__,
            template_folder=config_path,
            static_folder=config_path,
            static_url_path="/static",
        )
        # Use threading async mode with Werkzeug dev server; requires simple-websocket installed for WS
        self.socketio = SocketIO(self.app, async_mode="threading", cors_allowed_origins="*")
        self.queue = queue
        self.address = None
        self.port = 5000
        self.server_thread = None
        self.record_enabled = record_enabled

        @self.app.route("/")
        def index():
            return render_template("index.html", record_enabled=self.record_enabled)

        @self.app.route("/__shutdown__")
        def shutdown():
            func = request.environ.get('werkzeug.server.shutdown')
            if func is None:
                return "Server shutdown not available", 500
            func()
            return "OK", 200

        @self.socketio.on("message")
        def handle_message(data):
            # Send the timestamp back for RTT calculation (expected RTT on 5 GHz Wi-Fi is 7 ms)
            emit("echo", data["timestamp"])

            # Add data to queue for processing
            self.queue.put(data)

        # Explicit handlers for save/discard events
        @self.socketio.on("save_episode")
        def handle_save():
            if self.record_enabled:
                self.queue.put({"state_update": "save_episode", "timestamp": 0})

        @self.socketio.on("discard_episode")
        def handle_discard():
            if self.record_enabled:
                self.queue.put({"state_update": "discard_episode", "timestamp": 0})

        # Reduce verbose Flask log output
        logging.getLogger("werkzeug").setLevel(logging.WARNING)
        self.run()
        # Start the Flask server in a separate thread
        self.server_thread = threading.Thread(
            target=lambda: self.socketio.run(
                self.app,
                host="0.0.0.0",
                port=self.port,
                allow_unsafe_werkzeug=True,
                use_reloader=False,
            ),
            daemon=True,
        )
        self.server_thread.start()
        
    def run(self):
        # Get IP address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        try:
            s.connect(("8.8.8.8", 1))
            self.address = s.getsockname()[0]
        except Exception:
            self.address = "127.0.0.1"
        finally:
            s.close()
        print(f"Starting server at {self.address}:{self.port}")

    def stop(self):
        # Request Werkzeug shutdown endpoint
        try:
            urlopen(Request(f"http://127.0.0.1:{self.port}/__shutdown__"), timeout=1)
        except Exception:
            pass
        # Give the server a moment to stop
        time.sleep(0.2)
        # Join thread briefly; it's daemon so process can exit regardless
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=1.0)

class WebServerPublisher(Node):
    def __init__(self, queue):
        super().__init__("teleop_bridge_publisher")
        self.pub = self.create_publisher(
            TeleopMsg, "/teleop_commands", 10
        )
        self.state_pub = self.create_publisher(String, "/teleop_state", 10)

        self.queue = queue
        self.create_timer(0.0001, self._publish_loop)

    def _publish_loop(self):
            if not self.queue.empty():
                data = self.queue.get()
            else:
                return
            msg = TeleopMsg()
            msg.timestamp = data["timestamp"]
            if "state_update" in data:
                msg.state_update = data["state_update"]
                state_command = String()
                state_command.data = msg.state_update
                self.state_pub.publish(state_command)
            else:
                msg.device_id = data["device_id"]
                if "teleop_mode" in data:
                    msg.teleop_mode = data["teleop_mode"]
                    msg.pos_x = float(data["position"]["x"])
                    msg.pos_y = float(data["position"]["y"])
                    msg.pos_z = float(data["position"]["z"])
                    msg.or_x = float(data["orientation"]["x"])
                    msg.or_y = float(data["orientation"]["y"])
                    msg.or_z = float(data["orientation"]["z"])
                    msg.or_w = float(data["orientation"]["w"])
                if "gripper_delta" in data:
                    msg.gripper_delta = float(data["gripper_delta"])
            self.pub.publish(msg)

def main():
    rclpy.init()

    queue = Queue()

    web_server_publisher = WebServerPublisher(queue)

    # Get record flag from ROS param server if declared on this process via launch
    record_enabled = False
    try:
        node_for_params = rclpy.create_node('teleop_server_params')
        node_for_params.declare_parameter('record', False)
        record_enabled = node_for_params.get_parameter('record').get_parameter_value().bool_value
        node_for_params.destroy_node()
    except Exception:
        record_enabled = False

    webserver = WebServer(queue, record_enabled)
    web_server_publisher.get_logger().info(f"{GREEN}{BOLD}Web server publisher node started at {webserver.address}:5000{RESET}")

    try:
        rclpy.spin(web_server_publisher)
    finally:
        # Cleanly stop the web server on shutdown
        webserver.stop()
    web_server_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
