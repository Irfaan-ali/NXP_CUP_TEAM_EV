# Copyright 2024-2026 NXP
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import time
import math
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from synapse_msgs.msg import EdgeVectors, ServerCommunication

QOS_PROFILE_DEFAULT = 10
PI = math.pi

# Control bounds
SPEED_MIN = 0.0
SPEED_MAX = 1.0
TURN_MIN = -1.0
TURN_MAX = 1.0

# CONFIGURATION:
# The buggy is driven in manual mode by publishing standard controller Joy messages to /cerebri/in/joy.
# The layout is: msg.axes = [0.0, speed, 0.0, turn]
# - speed: positive for forward, negative for reverse. Range: [-1.0, 1.0]
# - turn: positive for left steer, negative for right steer. Range: [-1.0, 1.0]
# msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1] (Keep buttons set to this pattern for manual override mode)

class LineFollower(Node):
    """
    Core controller Node for the B3RB buggy.
    By default, it publishes a safe drive-straight command on a timer loop.
    Implement logic inside the callbacks to steer, dodge obstacles, detect destinations,
    communicate with the server, and park.
    """
    def __init__(self):
        super().__init__('line_follower')

        # ------------------ Subscriptions ------------------
        
        # 1. Lane Edge Vectors (from edge_vectors_publisher)
        self.subscription_vectors = self.create_subscription(
            EdgeVectors,
            '/edge_vectors',
            self.edge_vectors_callback,
            QOS_PROFILE_DEFAULT)

        # 2. LIDAR Obstacle Scanner
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QOS_PROFILE_DEFAULT)

        # 3. Server Communication Feedback Loop
        self.subscription_server = self.create_subscription(
            ServerCommunication,
            '/ServerCommunication',
            self.server_communication_callback,
            QOS_PROFILE_DEFAULT)

        # 4. QR Code Detections (from qr_detector)
        self.subscription_qr = self.create_subscription(
            String,
            '/qr_detection',
            self.qr_detection_callback,
            QOS_PROFILE_DEFAULT)

        # 5. Sign Board Detections (from object_recognizer)
        self.subscription_signs = self.create_subscription(
            String,
            '/sign_board_detection',
            self.sign_board_callback,
            QOS_PROFILE_DEFAULT)

        # ------------------ Publishers ------------------
        
        # Publisher to drive/steer the buggy
        self.publisher_joy = self.create_publisher(
            Joy,
            '/cerebri/in/joy',
            QOS_PROFILE_DEFAULT)

        # Publisher to send messages to the Server
        self.publisher_server = self.create_publisher(
            ServerCommunication,
            '/ServerCommunication',
            QOS_PROFILE_DEFAULT)

        # ------------------ State Variables & Timer ------------------
        
        # ---------------- Lane Following parameters1 ----------------


        self.center_offset = -5

        # Lane commands
        self.lane_speed = 0.3
        self.lane_turn = 0.0

        # Obstacle commands
        self.avoid_speed = 0.0
        self.avoid_turn = 0.0

        # Obstacle state
        self.obstacle_in_front = False

        # Distances
        self.front_distance = 10.0
        self.left_distance = 10.0
        self.right_distance = 10.0
        self.back_distance = 10.0

        # ---------- Lane Following Parameters ----------
        self.kp = 0.006         # Steering gain
        self.max_speed = 0.45     # Maximum speed
        self.min_speed = 0.25     # Minimum speed while turning

        self.last_turn = 0.0

        # Previous steering (used for smoothing)
        self.previous_turn = 0.0

        # Estimated lane width in pixels (used if only one lane is detected)
        self.lane_width_pixels = 220

        # Steering smoothing factor
        self.alpha = 0.82

        # State variables (You can add your own state flags / state machines here)
        self.obstacle_in_front = False
        self.patient_id = None
        self.hospital_id = None
        self.current_destination = None
        self.mission_completed = False
        self.target_speed = 0.0
        self.target_turn = 0.0

        # Timer to publish drive commands at 10Hz
        self.control_timer = self.create_timer(0.1, self.publish_drive_commands)

        self.get_logger().info("Line Follower controller initialized. Safe Drive-Straight Mode active.")

    def publish_drive_commands(self):
        """Timer callback that periodically publishes the current speed and steer command."""
        if self.obstacle_in_front:

            speed = self.avoid_speed
            turn = self.avoid_turn

        else:

            speed = self.lane_speed
            turn = self.lane_turn

        self.rover_move_manual_mode(speed, turn)
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]  # Manual override button configuration
        msg.axes = [0.0, self.target_speed, 0.0, self.target_turn]
        self.publisher_joy.publish(msg)

    def rover_move_manual_mode(self, speed, turn):
        """Helper to immediately set control speed and steering angle."""
        self.target_speed = float(max(min(speed, SPEED_MAX), -SPEED_MAX))
        self.target_turn = float(max(min(turn, TURN_MAX), -TURN_MAX))

    # ------------------ Callback Implementations ------------------

    def edge_vectors_callback(self, message):

        self.get_logger().info(f"Vectors detected : {message.vector_count}")

        if self.obstacle_in_front:
            return

        image_center = message.image_width / 2.0

        # ----------------------------------------------------
        # NO LANE DETECTED
        # ----------------------------------------------------
        if message.vector_count == 0:

            # Continue slowly using previous steering
            self.lane_speed = 0.12
            self.lane_turn = self.previous_turn

            self.get_logger().warn("Lane Lost")
            return

        # ----------------------------------------------------
        # TWO LANES DETECTED
        # ----------------------------------------------------
        if message.vector_count == 2:

            # Bottom points
            left_bottom = message.vector_1[1]
            right_bottom = message.vector_2[1]

            # Top points
            left_top = message.vector_1[0]
            right_top = message.vector_2[0]

            bottom_center = (left_bottom.x + right_bottom.x) / 2.0
            top_center = (left_top.x + right_top.x) / 2.0

            # Bottom is more important than top
            lane_center = 0.9 * bottom_center + 0.1 * top_center


        # ----------------------------------------------------
        # ONE LANE DETECTED
        # ----------------------------------------------------
        else:

            v1_length = math.hypot(message.vector_1[1].x - message.vector_1[0].x,message.vector_1[1].y - message.vector_1[0].y)

            v2_length = math.hypot(message.vector_2[1].x - message.vector_2[0].x,message.vector_2[1].y - message.vector_2[0].y)

            if v1_length > v2_length:
                lane = message.vector_1
            else:
                lane = message.vector_2

            if lane[1].x < image_center:

                # Left lane
                lane_center = lane[1].x + self.lane_width_pixels / 2

            else:

                # Right lane
                lane_center = lane[1].x - self.lane_width_pixels / 2


        # ----------------------------------------------------
        # Calculate Error
        # ----------------------------------------------------
        error = (image_center + self.center_offset) - lane_center


        # ----------------------------------------------------
        # Proportional Steering
        # ----------------------------------------------------
        turn = self.kp * error


        # ----------------------------------------------------
        # Clamp
        # ----------------------------------------------------
        turn = max(TURN_MIN, min(turn, TURN_MAX))


        # ----------------------------------------------------
        # Steering Smoothing
        # ----------------------------------------------------
        turn = (self.alpha * turn + (1 - self.alpha) * self.previous_turn)

        self.previous_turn = turn


        # ----------------------------------------------------
        # Adaptive Speed
        # ----------------------------------------------------
        speed = self.max_speed - abs(turn) * 0.20

        speed = max(self.min_speed,min(speed, self.max_speed))


        # Extra slowdown for single lane
        if message.vector_count == 1:
            speed *= 0.90


        # ----------------------------------------------------
        # Send command
        # ----------------------------------------------------
        self.lane_speed = speed
        self.lane_turn = turn

        self.get_logger().info(f"Error={error:.1f}  Turn={turn:.2f}  Speed={speed:.2f}")


    def lidar_callback(self, message):
        """LIDAR obstacle detection and avoidance."""

        ranges = list(message.ranges)

        if len(ranges) == 0:
            return

        num_readings = len(ranges)

        # -------------------------------------------------------
        # Remove invalid readings
        # -------------------------------------------------------

        def valid(data):
            return [
                r for r in data
                if math.isfinite(r) and 0.05 < r < 10.0
            ]

        # -------------------------------------------------------
        # FRONT SECTOR
        # Used for obstacle detection
        # -------------------------------------------------------

        front = valid(ranges[int(num_readings * 7 / 18):int(num_readings * 11 / 18)])

        # -------------------------------------------------------
        # LEFT SECTOR
        # Used to choose avoidance direction
        # -------------------------------------------------------

        left = valid(ranges[int(num_readings * 11 / 18):int(num_readings * 14 / 18)])

        # -------------------------------------------------------
        # RIGHT SECTOR
        # Used to choose avoidance direction
        # -------------------------------------------------------

        right = valid(ranges[int(num_readings * 4 / 18):int(num_readings * 7 / 18)])

        # -------------------------------------------------------
        # BACK SECTOR
        # Useful if reverse is ever needed
        # -------------------------------------------------------

        back = valid(ranges[int(num_readings * 16 / 18):]+ranges[:int(num_readings * 2 / 18)])

        # -------------------------------------------------------
        # Minimum distances
        # -------------------------------------------------------

        front_dist = min(front) if front else 10.0
        left_dist = min(left) if left else 10.0
        right_dist = min(right) if right else 10.0
        back_dist = min(back) if back else 10.0

        self.front_distance = front_dist
        self.left_distance = left_dist
        self.right_distance = right_dist
        self.back_distance = back_dist

        # -------------------------------------------------------
        # Hysteresis
        # Prevents obstacle flag from rapidly toggling
        # -------------------------------------------------------

        ENTER_DISTANCE = 0.80
        EXIT_DISTANCE = 1.00

        if self.obstacle_in_front:

            if front_dist > EXIT_DISTANCE:
                self.obstacle_in_front = False

        else:

            if front_dist < ENTER_DISTANCE:
                self.obstacle_in_front = True

        # -------------------------------------------------------
        # Obstacle Avoidance
        # -------------------------------------------------------

        if self.obstacle_in_front:

            # Slow down
            self.avoid_speed = 0.18

            # Turn toward the side with more free space
            if left_dist > right_dist:

                desired_turn = 0.85

            else:

                desired_turn = -0.85

            # Smooth steering
            self.avoid_turn = (0.35 * desired_turn + 0.65 * self.avoid_turn)

        else:

            # Lane follower takes control
            self.avoid_speed = 0.0
            self.avoid_turn = 0.0

        # -------------------------------------------------------
        # Debug Information
        # -------------------------------------------------------

        self.get_logger().debug(
            f"Front={front_dist:.2f}m | "
            f"Left={left_dist:.2f}m | "
            f"Right={right_dist:.2f}m | "
            f"Back={back_dist:.2f}m | "
            f"Obstacle={self.obstacle_in_front}")
            
    def server_communication_callback(self, message):
        """
        Receives coordination commands from the server.
        
        GUIDELINE (Server Communication):
        - Check if the message is destined for the Buggy (`message.dest == 1`).
		- Do not forget to check for ACK messages from server
        - The server communicates mission info in the `message.msg` payload string.
        - Parse server instructions (e.g., patient pickup, target hospitals).
        - Call `self.send_server_update` to report your status when you reach a checkpoint.
        """
        if message.dest == 1:
            self.get_logger().info(f"Received Server Message: {message.msg}")
            # Parse payload and update state machine destination/objectives here
            pass

    def send_server_update(self, text_msg):
        """Sends status messages to the server. (Do not forget to send ACK messages to server)"""
        server_msg = ServerCommunication()
        server_msg.src = 1       # Source component: Buggy-1
        server_msg.dest = 2      # Destination component: Server-2
        server_msg.uid = 100     # Replace with a rolling message ID/counter
        server_msg.ack = 0
        server_msg.msg = text_msg
        self.publisher_server.publish(server_msg)

    def qr_detection_callback(self, message):
        """
        Receives QR codes scanned from the buildings.
        
        GUIDELINE (Patient/Hospital Identification):
        - Parse the decoded string payload in `message.data` (e.g. "PATIENT_A", "HOSPITAL_B").
        - If it matches your target destination, stop the vehicle close to the building (verify range using LIDAR),
          perform the action (pick patient / drop patient), and communicate the arrival to the server.
        """
        self.get_logger().info(f"Heard QR code: {message.data}")
        pass

    def sign_board_callback(self, message):
        """
        Receives traffic sign boards.
        
        GUIDELINE (Sign Board Routing):
        - Use the detected signs to choose the quickest route at intersections.
        """
        self.get_logger().info(f"Heard Sign Board: {message.data}")
        pass

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


