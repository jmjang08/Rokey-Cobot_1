from rclpy.node import Node
from std_msgs.msg import Bool, Int32

# Robot Namespace Configuration
ROBOT_ID = "dsr01"


class RosBridge(Node):
    """
    Bridge node that connects Web (Socket.io) and ROS 2.
    Translates web signals into ROS 2 topics and vice-versa.
    """
    def __init__(self):
        super().__init__("ros_web_bridge", namespace=ROBOT_ID)

        self.socketio = None

        # ==========================================
        # Web -> ROS 2 (Publishers)
        # ==========================================
        self.start_pub = self.create_publisher(Bool, "/start_signal", 10)
        self.mode_pub  = self.create_publisher(Int32, "/mode_select", 10)
        self.stop_pub  = self.create_publisher(Bool, "/stop_signal", 10)
        self.recovery_pub = self.create_publisher(Bool, "/recovery_signal", 10)
        
        # ==========================================
        # ROS 2 -> Web (Subscribers)
        # ==========================================
        self.create_subscription(
            Int32,
            "/progress_state",
            self.cb_progress,
            10
        )

        self.get_logger().info("RosBridge Node Initialized and Ready.")

    # ------------------------------------------
    # Publish Methods (Called by Flask-SocketIO)
    # ------------------------------------------

    def publish_start(self, flag: bool):
        """Publishes the start signal to ROS 2."""
        msg = Bool()
        msg.data = flag
        self.start_pub.publish(msg)
        self.get_logger().info("Published: /start_signal (True)")

    def publish_mode(self, mode: int):
        """Publishes the selected cooking mode to ROS 2."""
        msg = Int32()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"Published: /mode_select ({mode})")

    def publish_stop(self):
        """Publishes the stop (emergency) signal to ROS 2."""
        msg = Bool()
        msg.data = True 
        self.stop_pub.publish(msg)
        self.get_logger().info("Published: /stop_signal (True)")

    def publish_recovery(self):
        """Publishes the recovery signal to reset the ROS 2 state machine."""
        msg = Bool()
        msg.data = True 
        self.recovery_pub.publish(msg)
        self.get_logger().info("Published: /recovery_signal (True)")

    # ------------------------------------------
    # Callback Methods (ROS 2 -> Web)
    # ------------------------------------------

    def cb_progress(self, msg):
        """
        Receives progress state from ROS 2 
        and sends it to the web dashboard via Socket.io.
        """
        if self.socketio is not None:
            # Mapping state integer to human-readable text
            status_map = {
                0: "Waiting for Order",
                1: "Moving to Kitchen",
                2: "Cooking in Progress",
                3: "Delivering Food",
                4: "Mission Completed",
                9: "Emergency Stopped"
            }
            status_text = status_map.get(msg.data, "Unknown State")
            
            self.socketio.emit("progress_update", {
                "step": msg.data,
                "text": status_text
            })
            self.get_logger().info(f"Forwarded to Web: State {msg.data} ({status_text})")
