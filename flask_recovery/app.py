from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import rclpy
from rosbridge import RosBridge


app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*')

# ROS 2 Global Initialization
rclpy.init()

# Create RosBridge Node
ros_node = RosBridge()
ros_node.socketio = socketio

# Run ROS 2 spin in a separate thread to handle callbacks
ros_thread = threading.Thread(
    target=rclpy.spin,
    args=(ros_node,),
    daemon=True
)
ros_thread.start()

@app.route('/')
def index():
    """Renders the main dashboard page."""
    return render_template("index.html")

# ==========================================
# Web to ROS 2 (Socket.io Event Handlers)
# ==========================================

@socketio.on('start_signal')
def handle_start_signal(msg):
    """Handles START signal from the web dashboard."""
    ros_node.publish_start(True)
    print("[Web -> ROS] START signal published")

@socketio.on('mode_select')
def handle_mode_select(data):
    """Handles cooking mode selection (1, 2, or 3)."""
    mode = data["mode"]
    ros_node.publish_mode(mode)
    print(f"[Web -> ROS] Mode selected: {mode}")
    
@socketio.on('stop_signal')
def handle_stop_signal(msg):
    """Handles STOP (Emergency) signal from the web dashboard."""
    ros_node.publish_stop() 
    print("[Web -> ROS] STOP signal published")

@socketio.on('recovery_signal')
def handle_recovery_signal(msg):
    """Handles RECOVERY signal to reset the system state."""
    ros_node.publish_recovery()
    print("[Web -> ROS] RECOVERY signal published")

if __name__ == '__main__':
    # Start the Flask-SocketIO server
    # The 'eventlet' or 'gevent' server is recommended for production
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
