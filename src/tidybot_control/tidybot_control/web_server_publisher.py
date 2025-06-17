import logging
import socket
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from queue import Queue
import threading
import rclpy
from rclpy.node import Node
from tidybot_msgs.msg import WSMsg
import json

import os
from ament_index_python.packages import get_package_share_directory

# Ensure the template and static directories are set correctly
pkg_share = get_package_share_directory("tidybot_control")
config_path = os.path.join(pkg_share, "config")

class WebServer:
    def __init__(self, queue):
        self.app = Flask(
            __name__,
            template_folder=config_path,
            static_folder=config_path,
            static_url_path="/static",
        )
        self.socketio = SocketIO(self.app)
        self.queue = queue
        self.address = None

        @self.app.route("/")
        def index():
            return render_template("index.html")

        @self.socketio.on("message")
        def handle_message(data):
            # Send the timestamp back for RTT calculation (expected RTT on 5 GHz Wi-Fi is 7 ms)
            emit("echo", data["timestamp"])

            # Add data to queue for processing
            self.queue.put(data)

        # Reduce verbose Flask log output
        logging.getLogger("werkzeug").setLevel(logging.WARNING)
        self.run()
        # Start the Flask server in a separate thread
        threading.Thread(
            target=lambda: self.socketio.run(self.app, host="0.0.0.0", allow_unsafe_werkzeug=True),
            daemon=True
        ).start()
        
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
        print(f"Starting server at {self.address}:5000")

class WebServerPublisher(Node):
    def __init__(self, queue):
        super().__init__("ws_bridge_publisher")
        self.pub = self.create_publisher(
            WSMsg, "/ws_commands", 10
        )

        self.queue = queue
        self.create_timer(0.0001, self._publish_loop)

    def _publish_loop(self):
            if not self.queue.empty():
                data = self.queue.get()
            else:
                return
            msg = WSMsg()
            msg.timestamp = data["timestamp"]
            if "state_update" in data:
                msg.state_update = data["state_update"]
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

    webserver = WebServer(queue)
    web_server_publisher.get_logger().info(f"Web server publisher node started at {webserver.address}:5000")

    rclpy.spin(web_server_publisher)
    web_server_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
