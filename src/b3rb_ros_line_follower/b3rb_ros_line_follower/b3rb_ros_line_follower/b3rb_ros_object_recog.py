# Copyright 2024-2026 NXP
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
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import os

from collections import deque, Counter

# HINT: TensorFlow/Keras can be heavy and might not be installed by default.
# We wrap the import in a try-except block so the node runs even if TensorFlow is missing.
# Install it using: pip install tensorflow

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

class ObjectRecognizer(Node):
    """
    ROS 2 Node that processes raw camera images to recognize traffic sign boards.
    It publishes the detected sign type/labels on the `/sign_board_detection` topic.
    """
    def __init__(self):
        super().__init__('object_recognizer')
        self.confirmed_pairs = {

            "A":"NotDetected",
            "B":"NotDetected",
            "C":"NotDetected",
            "X":"NotDetected",
            "Y":"NotDetected",
            "Z":"NotDetected"

        }

        # Store the last 5 observations for each location
        self.history = {

            "A": deque(maxlen=5),
            "B": deque(maxlen=5),
            "C": deque(maxlen=5),
            "X": deque(maxlen=5),
            "Y": deque(maxlen=5),
            "Z": deque(maxlen=5)

        }

        self.locked_locations = set()
        self.CONFIRMATION_THRESHOLD = 3
        MIN_OVERLAP = 20
        self.VERTICAL_THRESHOLD = 200

        # Subscription for camera images.
        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            10)

        # Publisher for sign board detection results.
        self.publisher_sign = self.create_publisher(
            String,
            '/sign_board_detection',
            10)

        # Attempt to load the pre-trained YOLO model (best.pt) located in the same directory.
        self.model = None

        if YOLO is not None:
            try:
                dir_path = os.path.dirname(os.path.abspath(__file__))
                model_path = os.path.join(dir_path, "best.pt")

                if os.path.exists(model_path):

                    self.model = YOLO(model_path)

                    self.get_logger().info(
                        f"Loaded YOLO model from {model_path}"
                    )

                else:
                    self.get_logger().warn(
                        f"best.pt not found at {model_path}"
                    )

            except Exception as e:

                self.get_logger().error(
                    f"Failed to load YOLO model : {e}"
                )

        else:

            self.get_logger().warn(
                "Ultralytics not installed."
            )

        self.get_logger().info("Object Recognizer Node started. Waiting for images...")

    def camera_image_callback(self, message):
        """Processes incoming camera frames to classify traffic signs."""
        # Convert compressed image message to OpenCV format
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        detections = self.classify_sign(image)

        if detections is None:
            return

        # Separate locations and arrows

        locations = []
        arrows = []

        for obj in detections:

            if obj["label"] in ["A","B","C","X","Y","Z"]:
                locations.append(obj)

            elif obj["label"] in ["Left","Right","Straight"]:
                arrows.append(obj)

        if len(locations) == 0 or len(arrows) == 0:
            return


        # Initialise result

        detected_pairs = {

            "A":"NotDetected",
            "B":"NotDetected",
            "C":"NotDetected",
            "X":"NotDetected",
            "Y":"NotDetected",
            "Z":"NotDetected"

        }

        used_arrows = set()

        for location in locations:

            best_arrow = None

            best_score = 1e9

            for i, arrow in enumerate(arrows):

                if i in used_arrows:
                    continue

                # Arrow MUST be below

                dy = arrow["centre_y"] - location["centre_y"]

                if dy <= 0 or dy > self.VERTICAL_THRESHOLD:
                    continue
                
                # Horizontal overlap
                
                overlap = min(location["x2"], arrow["x2"]) - max(location["x1"], arrow["x1"])

                if overlap < MIN_OVERLAP:
                    continue

                score = dy + 0.5 * dx
                
                if score < best_score:

                    best_score = score

                    best_arrow = i

            if best_arrow is not None:

                detected_pairs[
                    location["label"]
                ] = arrows[
                    best_arrow
                ]["label"]

                used_arrows.add(best_arrow)

        for location, direction in detected_pairs.items():

            # Already learned -> skip forever
            if location in self.locked_locations:
                continue

            # Ignore if nothing detected this frame
            if direction == "NotDetected":
                continue

            # Add current observation to history
            self.history[location].append(direction)

            # Count occurrences in the last 5 frames
            votes = Counter(self.history[location])

            most_common_direction, count = votes.most_common(1)[0]

            if count >= self.CONFIRMATION_THRESHOLD:

                self.confirmed_pairs[location] = most_common_direction

                self.locked_locations.add(location)

                self.history[location].clear()

                self.get_logger().info(
                    f"{location} locked as {most_common_direction}"
                )

        msg = String()

        msg.data = str(self.confirmed_pairs)

        self.publisher_sign.publish(msg)

        self.get_logger().info(msg.data)

    def classify_sign(self, image):
        """
        Classify traffic sign boards.
        
        OPTIMIZATION HINTS:
        - If TensorFlow is installed, you can pre-process the image (e.g. crop the sign region, 
          resize to 150x150, normalize, expand dimensions) and feed it into `self.model.predict()`.
        - Alternatively, you can use classic Computer Vision techniques:
          1. Color Segmentation: Convert to HSV and threshold for specific sign colors.
          2. Shape Detection: Find contours and approximate polygons.
          3. Template Matching: Match regions of interest against template images of sign boards.
        """
        

        if self.model is None:
            return None

        try:

            results = self.model(
            image,
            imgsz=640,
            conf=0.25,
            iou=0.45,
            verbose=False
        )

            detections = []

            for result in results:

                for box in result.boxes:

                    confidence = float(box.conf)

                    class_id = int(box.cls)

                    label = self.model.names[class_id]

                    if label in ["Left", "Right", "Straight"]:
                        if confidence < 0.75:
                            continue
                    else:
                        if confidence < 0.55:
                            continue

                    x1, y1, x2, y2 = box.xyxy[0]

                    x1 = float(x1)
                    y1 = float(y1)
                    x2 = float(x2)
                    y2 = float(y2)

                    detections.append({

                        "label": label,

                        "confidence": confidence,

                        "centre_x": (x1 + x2) / 2,

                        "centre_y": (y1 + y2) / 2,

                        "x1": x1,
                        "y1": y1,
                        "x2": x2,
                        "y2": y2

                    })
                        

            return detections

        except Exception as e:

            self.get_logger().error(
                f"Inference failed : {e}"
            )

            return None

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

