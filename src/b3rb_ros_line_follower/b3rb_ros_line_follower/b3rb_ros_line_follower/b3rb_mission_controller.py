"""
mission_controller.py

Brain of the robot.

Responsibilities
----------------
Mission State Machine
Mission Variables
UID Management
Retry Management
Timers
Logging
Decision Making

NO ROS Subscribers
NO ROS Publishers

All ROS interaction happens through LineFollower.
"""

from enum import Enum, auto
import time

TOTAL_PATIENTS = 3
SERVER_TIMEOUT = 3.0
MAX_RETRIES = 5

# ==========================================================
# Mission States
# ==========================================================

class MissionState(Enum):

    START = auto()
    FOLLOW_LANE = auto()
    PATIENT_FOUND = auto()
    WAIT_PATIENT_ACK = auto()
    WAIT_HOSPITAL_ASSIGNMENT = auto()
    GO_TO_HOSPITAL = auto()
    HOSPITAL_FOUND = auto()
    WAIT_HOSPITAL_ACK = auto()
    NEXT_PATIENT = auto()
    PARK = auto()
    WAIT_PARK_ACK = auto()
    FINISHED = auto()
    ERROR = auto()

# ==========================================================
# Mission Controller
# ==========================================================

class MissionController:

    def __init__(self, robot):

        """
        robot

        Reference to LineFollower object.

        Allows MissionController to access

            robot.send_server_update()

            robot.rover_move_manual_mode()

            robot.get_logger()

        without becoming a ROS node.
        """

        self.robot = robot
        self.logger = robot.get_logger()
        self.logger.info("Mission Controller Initializing...")

        # ==========================================
        # FSM
        # ==========================================

        self.state = MissionState.START
        self.previous_state = None
        self.state_entered = True

        # ==========================================
        # Mission Information
        # ==========================================

        self.current_patient = None
        self.assigned_hospital = None
        self.current_sign = None
        self.latest_qr = None
        self.last_qr = None
        self.qr_detected = False
        self.sign_detected = False
        self.server_message = None
        self.server_message_received = False
        self.obstacle_detected = False
        self.parking_complete = False
        self.mission_finished = False
        self.last_processed_qr = None
        self.last_payload = None

        # ==========================================
        # Delivery Tracking
        # ==========================================

        self.patients_delivered = 0
        self.total_patients = TOTAL_PATIENTS

        # ==========================================
        # Communication
        # ==========================================

        self.uid_counter = 0
        self.waiting_for_ack = False
        self.waiting_for_assignment = False
        self.waiting_for_parking_confirmation = False
        self.last_uid_sent = None
        self.retry_count = 0
        self.max_retry = MAX_RETRIES
        self.assignment_received = False
        self.parking_response = None
        self.invalid_response = False

        # ==========================================
        # Timing
        # ==========================================

        self.state_start_time = time.time()
        self.last_message_time = None
        self.server_timeout = SERVER_TIMEOUT
        self.max_patient_time = 60.0

        # ==========================================
        # Debug
        # ==========================================

        self.debug = True
        self.logger.info("Mission Controller Ready")

    # ==========================================================
    # Utility Functions
    # ==========================================================

    def next_uid(self):

        """
        Returns next rolling UID.

        0

        1

        2

        ...

        255

        0
        """

        uid = self.uid_counter

        self.uid_counter = (self.uid_counter + 1) % 256

        return uid

    # ----------------------------------------------------------

    def reset_retry_counter(self):

        self.retry_count = 0

    # ----------------------------------------------------------

    def increment_retry(self):

        self.retry_count += 1

    # ----------------------------------------------------------

    def retry_exceeded(self):

        return self.retry_count >= self.max_retry

    # ----------------------------------------------------------

    def state_elapsed(self):

        return time.time() - self.state_start_time

    # ----------------------------------------------------------

    def reset_state_timer(self):

        self.state_start_time = time.time()

    # ----------------------------------------------------------

    def timeout(self):

        return self.state_elapsed() >= self.server_timeout

    # ==========================================================
    # State Management
    # ==========================================================

    def change_state(self, new_state):

        if self.debug:

            self.logger.info(
                f"[FSM] {self.state.name} --> {new_state.name}"
            )

        self.previous_state = self.state
        self.state = new_state
        self.reset_state_timer()

    # ==========================================================
    # Update Loop
    # ==========================================================

    def update(self):
        """
        Called every timer cycle from LineFollower.
        This is the central intelligence that coordinates every subsystem 
        of the autonomous buggy throughout the competition.
        """
        # 1. Process Networking & Timeouts
        # Always check for incoming server data or dropped packets first
        self.process_server_packet()
        self.communication_timeout()
        if self.invalid_response:
            self.invalid_response = False
            self.change_state(MissionState.ERROR)
            return

        # 2. Global Safety Override: Obstacle Detection
        # If an obstacle is detected and we aren't already finished or in error, halt.
        if self.obstacle_detected and self.state not in [MissionState.FINISHED, MissionState.ERROR]:
            self.stop_robot()
            self.log("[WARNING] Obstacle detected! Halting buggy until path is clear.")
            return  # Pause FSM progression until the obstacle is removed

        # 3. Finite State Machine Logic
        if self.state == MissionState.START:
            self.log("[MISSION] Starting mission. Initializing search...")
            self.change_state(MissionState.FOLLOW_LANE)

        elif self.state == MissionState.FOLLOW_LANE:
            self.follow_lane()
            if self.qr_detected:
                self.current_patient = self.latest_qr
                self.qr_detected = False  # Consume the detection flag
                self.stop_robot()
                self.log(f"[MISSION] QR Detected. Patient identified: {self.current_patient}")
                self.change_state(MissionState.PATIENT_FOUND)

        elif self.state == MissionState.PATIENT_FOUND:
            self.stop_robot()
            self.send_server_message(self.current_patient)
            self.change_state(MissionState.WAIT_PATIENT_ACK)

        elif self.state == MissionState.WAIT_PATIENT_ACK:
            self.stop_robot()
            if not self.waiting_for_ack:
                self.log("[MISSION] Patient ACK received. Waiting for hospital assignment...")
                self.change_state(MissionState.WAIT_HOSPITAL_ASSIGNMENT)

        elif self.state == MissionState.WAIT_HOSPITAL_ASSIGNMENT:
            self.stop_robot()
            if self.assignment_received:
                self.assignment_received = False
                self.change_state(
                    MissionState.GO_TO_HOSPITAL
                )

        elif self.state == MissionState.GO_TO_HOSPITAL:

            # Continue normal navigation
            self.follow_lane()

            # -------------------------------
            # Sign Detection
            # -------------------------------
            if self.sign_detected:
                self.sign_detected = False
                self.log(
                 f"[MISSION] Hospital Sign Detected: {self.current_sign}"
                )
        # Placeholder:
        # Person 2's navigation logic can use self.current_sign
        # to decide whether to turn left, right or continue.
        #
        # Example:
        #
        # self.navigation.handle_hospital_sign(self.current_sign)

            # -------------------------------
            # Hospital QR Verification
            # -------------------------------
            if self.qr_detected:
                self.qr_detected = False
                if self.latest_qr == self.assigned_hospital:
                    self.stop_robot()
                    self.log(
                        f"[MISSION] Correct Hospital Reached: {self.assigned_hospital}"
                    )
                    self.change_state(
                        MissionState.HOSPITAL_FOUND
                    )
                else:
                    self.log(
                        f"[MISSION] Wrong Hospital ({self.latest_qr}) "
                        f"Expected ({self.assigned_hospital})"
                    )
            # Continue following the lane

        elif self.state == MissionState.HOSPITAL_FOUND:
            self.stop_robot()
            self.send_server_message(self.assigned_hospital)
            self.change_state(MissionState.WAIT_HOSPITAL_ACK)

        elif self.state == MissionState.WAIT_HOSPITAL_ACK:
            self.stop_robot()
            if not self.waiting_for_ack:
                self.log("[MISSION] Hospital drop-off ACK received.")
                self.change_state(MissionState.NEXT_PATIENT)

        elif self.state == MissionState.NEXT_PATIENT:
            self.stop_robot()
            self.patients_delivered += 1
            self.log(f"[MISSION] Delivery success. {self.patients_delivered}/{self.total_patients} patients delivered.")
            
            # Reset mission variables for the next run
            self.current_patient = None
            self.assigned_hospital = None
            self.current_sign = None
            self.latest_qr = None
            self.reset_server_flags()

            # Check if mission is complete
            if self.patients_delivered >= self.total_patients:
                self.change_state(MissionState.PARK)
            else:
                self.change_state(MissionState.FOLLOW_LANE)

        elif self.state == MissionState.PARK:
            self.state_entered = False
            self.log("[MISSION] All patients delivered! Initiating parking sequence.")
            self.park_robot()
            self.send_server_message("PARKED")
            self.change_state(MissionState.WAIT_PARK_ACK)

        elif self.state == MissionState.WAIT_PARK_ACK:
            self.stop_robot()
            # State transition to FINISHED is handled by process_server_packet()
            # when the server responds with "OK".
            if self.parking_response == "OK":
               self.change_state(MissionState.FINISHED)
            elif self.parking_response == "INVALID":
                self.parking_response = None
                self.change_state(MissionState.PARK)

        elif self.state == MissionState.FINISHED:
            if not self.mission_finished:
                self.stop_robot()
                self.log("=========================================")
                self.log("[SUCCESS] Mission Complete! Buggy is parked.")
                self.log("=========================================")
                self.mission_finished = True

        elif self.state == MissionState.ERROR:
            self.stop_robot()
            self.log("[CRITICAL] Mission Controller entered ERROR state. Halting operations.")

    # ==========================================================
    # Interfaces from ROS Callbacks
    # ==========================================================

    def update_qr(self, qr):

        if qr == self.last_processed_qr:
            return

        self.latest_qr = qr
        self.last_processed_qr = qr
        self.qr_detected = True

    # ----------------------------------------------------------

    def update_sign(self, sign):

        self.current_sign = sign
        self.sign_detected = True

    # ----------------------------------------------------------

    def update_server(self, msg):

        self.server_message = msg
        self.server_message_received = True

    # ----------------------------------------------------------

    def update_obstacle(self, detected):

        self.obstacle_detected = detected

    # ==========================================================
    # Communication
    # ==========================================================

    def send_server_message(self, payload):

        """
        Send normal message to Municipality Server.
        payload examples
        PATIENT_1
        HOSPITAL_2
        PARKED
        """
        self.reset_retry_counter()
        uid = self.next_uid()
        self.last_uid_sent = uid
        self.waiting_for_ack = True
        self.last_message_time = time.time()
        self.last_payload = payload
        self.robot.send_server_packet(
            uid=uid,
            ack=0,
            payload=payload
        )
        self.log(f"[SERVER] Sent -> {payload}  UID={uid}")

        # ----------------------------------------------------------

    def send_ack(self, uid):

        """
        ACK a packet received
        from server.
        """

        self.robot.send_server_packet(
            uid=uid,
            ack=1,
            payload=""
        )
        self.log(f"[SERVER] ACK -> UID={uid}")

    # ---------------------------------------------------------

    def process_server_packet(self):

        """
        Called every update cycle.
        Reads newest packet received
        from LineFollower.
        """

        if not self.server_message_received:
            return
        packet = self.server_message
        self.server_message_received = False

        # ----------------------------------
        # Ignore packets not for Buggy
        # ----------------------------------

        if packet.dest != 1:
            return

        # ----------------------------------
        # ACK packet
        # ----------------------------------

        if packet.ack == 1:
            self.waiting_for_ack = False
            self.reset_retry_counter()
            self.log(
                f"[SERVER] ACK received UID={packet.uid}"
            )
            return

        # ----------------------------------
        # New Server Message
        # ----------------------------------

        self.send_ack(packet.uid)
        text = packet.msg.strip()
        self.log(
            f"[SERVER] Message = {text}"
        )

        # ----------------------------------
        # INVALID
        # ----------------------------------

        if text == "INVALID":
            if self.state == MissionState.WAIT_PARK_ACK:
                self.parking_response = "INVALID"
            else:
                self.invalid_response = True
            return

        # ----------------------------------
        # PARKING SUCCESS
        # ----------------------------------

        if text == "OK":
            if self.state == MissionState.WAIT_PARK_ACK:
                self.parking_complete = True
                self.parking_response = "OK"
            return

        # ----------------------------------
        # Hospital Assignment
        # ----------------------------------

        self.assigned_hospital = text
        self.waiting_for_assignment = False
        self.assignment_received = True

    # ---------------------------------------------------------

    def communication_timeout(self):

        """
        Retry if ACK never comes.
        """

        if not self.waiting_for_ack:
            return

        if not self.timeout():
            return

        self.increment_retry()
        self.log(
            f"[SERVER] Timeout {self.retry_count}"
        )

        if self.retry_exceeded():
            self.invalid_response = True
            return

        self.robot.send_server_packet(

            uid=self.last_uid_sent,
            ack=0,
            payload=self.last_payload
            )
        self.last_message_time = time.time()
        self.log(
            f"[SERVER] Retrying ({self.retry_count}/{self.max_retry})"
        )

    # ---------------------------------------------------------

    def reset_server_flags(self):
        self.waiting_for_ack = False
        self.waiting_for_assignment = False
        self.server_message_received = False

    # ==========================================================
    # Navigation Hooks
    # ==========================================================

    def follow_lane(self):

        """
        Placeholder.

        Person 1's lane follower
        naturally controls the buggy.

        Mission Controller only decides
        WHEN lane following should happen.
        """

        pass

    # ----------------------------------------------------------

    def stop_robot(self):
        self.robot.rover_move_manual_mode(0.0, 0.0)

    # ----------------------------------------------------------

    def park_robot(self):

        """
        Placeholder for parking logic.
        """
        pass

    # ==========================================================
    # Logging Helpers
    # ==========================================================

    def log(self, text):
        if self.debug:
            self.logger.info(text)