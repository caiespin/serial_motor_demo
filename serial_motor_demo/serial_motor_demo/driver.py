#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals
import time
import math
import serial
from threading import Lock

##############################################################################
# Helper function to handle absolute encoder wrap-around
##############################################################################
def compute_delta_abs_enc(new_val: int, old_val: int, encoder_max: int) -> int:
    """
    Computes the signed delta between two absolute encoder readings, handling wrap-around.
    
    For a 14-bit encoder:
      encoder_max = 16383
    If the difference is larger than half the range, we assume we've wrapped around.
    """
    delta = new_val - old_val
    wrap_half = (encoder_max + 1) // 2  # e.g. 8192 for 14-bit (16384 counts)

    if delta > wrap_half:
        # e.g. jumped from near max to near 0
        delta -= (encoder_max + 1)
    elif delta < -wrap_half:
        # e.g. jumped from near 0 to near max
        delta += (encoder_max + 1)

    return delta

class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        # --- Setup parameters ---
        self.declare_parameter('encoder_cpr', value=16384)  # For 14-bit absolute (0..16383)
        self.declare_parameter('loop_rate', value=30)       # Example default loop rate
        self.declare_parameter('serial_port', value="/dev/ttyArduino")
        self.declare_parameter('baud_rate', value=57600)
        self.declare_parameter('serial_debug', value=False)

        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        if self.encoder_cpr == 0:
            self.get_logger().warn("WARNING! ENCODER CPR SET TO 0!!")

        self.loop_rate = self.get_parameter('loop_rate').value
        if self.loop_rate == 0:
            self.get_logger().warn("WARNING! LOOP RATE SET TO 0!!")

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if self.debug_serial_cmds:
            self.get_logger().info("Serial debug enabled")

        # --- Setup topics ---
        self.subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10
        )
        self.speed_pub = self.create_publisher(MotorVels, 'motor_vels', 10)
        self.encoder_pub = self.create_publisher(EncoderVals, 'encoder_vals', 10)
        
        # --- Internal State ---
        self.last_enc_read_time = time.time()

        # Store the "absolute" encoder readings from the last cycle
        self.last_m1_abs = 0
        self.last_m2_abs = 0

        self.m1_spd = 0.0
        self.m2_spd = 0.0

        # If physically forward on the right motor yields decreasing raw counts,
        # set this to True to invert the right encoder reading.
        self.invert_right_encoder = True  # <-- Toggle this if needed

        self.mutex = Lock()

        # --- Open serial connection ---
        self.get_logger().info(f"Connecting to port {self.serial_port} at {self.baud_rate}")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        self.get_logger().info(f"Connected to {self.conn}")

    ##########################################################################
    # Motor command callbacks
    ##########################################################################
    def motor_command_callback(self, motor_command):
        """
        When a MotorCommand is received:
         - If is_pwm=True, we send raw PWM
         - Otherwise, we convert rad/sec -> 'ticks per loop' and send that
        """
        if motor_command.is_pwm:
            self.send_pwm_motor_command(
                motor_command.mot_1_req_rad_sec,
                motor_command.mot_2_req_rad_sec
            )
        else:
            # counts per loop = rad/sec * (counts / (2*pi)) * (seconds per loop)
            scaler = (1.0 / (2.0 * math.pi)) * self.encoder_cpr * (1.0 / float(self.loop_rate))
            mot1_ct_per_loop = motor_command.mot_1_req_rad_sec * scaler
            mot2_ct_per_loop = motor_command.mot_2_req_rad_sec * scaler
            self.send_feedback_motor_command(mot1_ct_per_loop, mot2_ct_per_loop)

    ##########################################################################
    # Helper commands
    ##########################################################################
    def send_pwm_motor_command(self, mot_1_pwm, mot_2_pwm):
        """
        Example command: "o <mot_1_pwm> <mot_2_pwm>"
        If your firmware uses a different command for MOTOR_RAW_PWM, adapt accordingly.
        """
        cmd = f"o {int(mot_1_pwm)} {int(mot_2_pwm)}"
        self.send_command(cmd)

    def send_feedback_motor_command(self, mot_1_ct_per_loop, mot_2_ct_per_loop):
        """
        Example command: "m <mot_1_ct_per_loop> <mot_2_ct_per_loop>"
        If your firmware uses the "MOTOR_SPEEDS" approach with speed in ticks/frame,
        this matches that usage.
        """
        cmd = f"m {int(mot_1_ct_per_loop)} {int(mot_2_ct_per_loop)}"
        self.send_command(cmd)

    def send_encoder_read_command(self):
        """
        Sends "e" command: The firmware returns something like "12345 2345" (absolute).
        """
        resp = self.send_command("e")
        if resp:
            try:
                return [int(raw_enc) for raw_enc in resp.split()]
            except ValueError:
                self.get_logger().warn("Could not parse encoder data")
                return []
        return []

    ##########################################################################
    # Reading Encoders & Computing Speed
    ##########################################################################
    def check_encoders(self):
        resp_vals = self.send_encoder_read_command()
        if resp_vals and len(resp_vals) >= 2:
            new_time = time.time()
            time_diff = new_time - self.last_enc_read_time
            self.last_enc_read_time = new_time

            encoder_max = self.encoder_cpr - 1  # e.g., 16383 for 14-bit
            left_abs_raw = resp_vals[0]
            right_abs_raw = resp_vals[1]

            # Optionally invert the right encoder reading
            if self.invert_right_encoder:
                # If physically forward yields a decreasing raw value,
                # we can unify it by flipping it here:
                right_abs_raw = encoder_max - right_abs_raw

            # Compute deltas with wrap-around
            m1_diff = compute_delta_abs_enc(left_abs_raw, self.last_m1_abs, encoder_max)
            m2_diff = compute_delta_abs_enc(right_abs_raw, self.last_m2_abs, encoder_max)

            # Store new absolute for next time
            self.last_m1_abs = left_abs_raw
            self.last_m2_abs = right_abs_raw

            # Convert to rad/sec
            rads_per_ct = (2.0 * math.pi) / float(self.encoder_cpr)
            self.m1_spd = (m1_diff * rads_per_ct) / time_diff
            self.m2_spd = (m2_diff * rads_per_ct) / time_diff

            # Publish the speeds
            spd_msg = MotorVels()
            spd_msg.mot_1_rad_sec = self.m1_spd
            spd_msg.mot_2_rad_sec = self.m2_spd
            self.speed_pub.publish(spd_msg)

            # Publish the "current" absolute encoder values
            enc_msg = EncoderVals()
            enc_msg.mot_1_enc_val = left_abs_raw
            enc_msg.mot_2_enc_val = right_abs_raw
            self.encoder_pub.publish(enc_msg)


    ##########################################################################
    # Low-level serial
    ##########################################################################
    def send_command(self, cmd_string):
        """
        Sends 'cmd_string' plus '\r' to the firmware, reads until '\r' from firmware.
        Returns the stripped response (minus the carriage return).
        """
        self.mutex.acquire()
        try:
            cmd = cmd_string + "\r"
            self.conn.write(cmd.encode("utf-8"))
            if self.debug_serial_cmds:
                print(f"Sent: {cmd.strip()}")

            c = ''
            value = ''
            while c != '\r':
                c = self.conn.read(1).decode("utf-8", errors='ignore')
                if c == '':
                    print(f"Error: Serial timeout on command: {cmd_string}")
                    return ''
                value += c

            value = value.strip('\r')
            if self.debug_serial_cmds:
                print(f"Received: {value}")
            return value
        finally:
            self.mutex.release()

    def close_conn(self):
        self.conn.close()

def main(args=None):
    rclpy.init(args=args)

    motor_driver = MotorDriver()

    # We'll mimic the original approach of a 2 Hz loop:
    rate = motor_driver.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(motor_driver)
        motor_driver.check_encoders()

    motor_driver.close_conn()
    motor_driver.destroy_node()
    rclpy.shutdown()
