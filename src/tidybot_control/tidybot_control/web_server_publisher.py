import logging
import socket
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from queue import Queue
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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

        @self.app.route("/")
        def index():
            return render_template("index.html")

        @self.socketio.on("message")
        def handle_message(data):
            # Send the timestamp back for RTT calculation (expected RTT on 5 GHz Wi-Fi is 7 ms)
            emit("echo", data["timestamp"])

            # Add data to queue for processing
            self.queue.put(data)
            print(f"Received data: {data}")

        # Reduce verbose Flask log output
        logging.getLogger("werkzeug").setLevel(logging.WARNING)

    def run(self):
        # Get IP address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        try:
            s.connect(("8.8.8.8", 1))
            address = s.getsockname()[0]
        except Exception:
            address = "127.0.0.1"
        finally:
            s.close()
        print(f"Starting server at {address}:5000")
        self.socketio.run(self.app, host="0.0.0.0")

class WebServerPublisher(Node):
    def __init__(self, queue):
        super().__init__("ws_bridge_publisher")
        self.pub = self.create_publisher(
            String, "/ws_commands", 10
        )
        self.queue = queue

        self._stop_event = threading.Event()
        self.thread = threading.Thread(target=self._publish_loop)
        self.thread.start()

    def _publish_loop(self):
        while rclpy.ok() and not self._stop_event.is_set():
            if not self.queue.empty():
                data = self.queue.get()
            else:
                continue
            msg = String()
            msg.data = json.dumps(data)
            self.pub.publish(msg)
            self.get_logger().info(f"Published message: {msg.data}")


    def stop(self):
        self._stop_event.set()
        self.thread.join()
            

    def destroy_node(self):
        self.thread.join()
        super().destroy_node()

def main():
    rclpy.init()

    queue = Queue()

    web_server_publisher = WebServerPublisher(queue)

    webserver = WebServer(queue)
    webserver.run()

    try:
        webserver.run()
    except KeyboardInterrupt:
        pass
    finally:
        web_server_publisher.stop()
        web_server_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
