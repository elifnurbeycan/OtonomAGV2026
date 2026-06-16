import math
import serial

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Twist

from tf2_ros import TransformBroadcaster


class EncoderImuOdomPublisher(Node):
    def __init__(self):
        super().__init__('encoder_imu_odom_publisher')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_base', 0.40)
        self.declare_parameter('pwm_multiplier', 185.0)
        self.declare_parameter('pulses_per_rev', 3000.0)

        self.serial_port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baud').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.pwm_multiplier = self.get_parameter('pwm_multiplier').get_parameter_value().double_value
        self.pulses_per_rev = self.get_parameter('pulses_per_rev').get_parameter_value().double_value

        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.005)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.imu_yaw = 0.0
        self.last_imu_time = None

        self.prev_left = None
        self.prev_right = None

        # Motor target state variables
        self.left_pwm = 0
        self.right_pwm = 0
        self.last_cmd_vel_time = self.get_clock().now()

        # Timers: 50 Hz serial reader, 10 Hz motor writer (watchdog feeder)
        self.timer = self.create_timer(0.02, self.read_serial)
        self.motor_timer = self.create_timer(0.1, self.send_motor_commands)

        self.get_logger().info(f'Serial baglandi: {self.serial_port} @ {self.baudrate}')
        self.get_logger().info('Publishing: /odom, /imu/data_raw, /tf')

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive kinematics
        v_left = linear - angular * (self.wheel_base / 2.0)
        v_right = linear + angular * (self.wheel_base / 2.0)

        # Update target PWM values
        self.left_pwm = int(v_left * self.pwm_multiplier)
        self.right_pwm = int(v_right * self.pwm_multiplier)
        
        # Reset python watchdog timer
        self.last_cmd_vel_time = self.get_clock().now()

    def send_motor_commands(self):
        import json
        # Python watchdog: if no /cmd_vel message received for 2.0 seconds, safety stop
        now = self.get_clock().now()
        dt_cmd = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if dt_cmd > 2.0:
            self.left_pwm = 0
            self.right_pwm = 0

        cmd_dict = {
            "motors": {
                "left": self.left_pwm,
                "right": self.right_pwm
            }
        }

        try:
            cmd_json = json.dumps(cmd_dict) + "\n"
            self.ser.write(cmd_json.encode())
        except Exception as e:
            self.get_logger().warn(f'Komut gonderilemedi: {e}')

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def read_serial(self):
        import json
        try:
            latest_line = None
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('{') and line.endswith('}'):
                    latest_line = line

            if not latest_line:
                return

            try:
                data = json.loads(latest_line)
            except json.JSONDecodeError:
                return

            # Detect Arduino Reset
            if 'status' in data and data['status'] == 'ready':
                self.get_logger().error('!!! ARDUINO MEGA YENİDEN BAŞLATILDI (RESET/BROWNOUT DETECTED) !!!')
                return

            # Extract encoder ticks (with key check and type casting)
            if 'encoders' not in data or 'imu' not in data:
                return

            left_ticks = int(data['encoders']['left'])
            right_ticks = int(data['encoders']['right'])

            # Extract IMU accel (already in m/s^2 from Arduino, cast to float)
            ax_m_s2 = float(data['imu']['accel']['x'])
            ay_m_s2 = float(data['imu']['accel']['y'])
            az_m_s2 = float(data['imu']['accel']['z'])

            # Extract IMU gyro (already in rad/s from Arduino, cast to float)
            gx_rad_s = float(data['imu']['gyro']['x'])
            gy_rad_s = float(data['imu']['gyro']['y'])
            gz_rad_s = float(data['imu']['gyro']['z'])

            # Calculate distance in meters from encoder ticks
            # wheel diameter: 0.12 m
            wheel_diameter = 0.12
            meters_per_pulse = (math.pi * wheel_diameter) / self.pulses_per_rev

            left_m = left_ticks * meters_per_pulse
            right_m = right_ticks * meters_per_pulse

            now = self.get_clock().now()

            if self.prev_left is None:
                self.prev_left = left_m
                self.prev_right = right_m
                self.last_imu_time = now
                return

            dt = (now - self.last_imu_time).nanoseconds / 1e9
            if dt > 0:
                self.imu_yaw += gz_rad_s * dt
            self.last_imu_time = now

            d_left = left_m - self.prev_left
            d_right = right_m - self.prev_right

            self.prev_left = left_m
            self.prev_right = right_m

            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / self.wheel_base

            self.theta += d_theta
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)

            odom_qz, odom_qw = self.yaw_to_quaternion(self.theta)
            imu_qz, imu_qw = self.yaw_to_quaternion(self.imu_yaw)

            self.publish_odom(now, odom_qz, odom_qw)
            self.publish_imu(now, imu_qz, imu_qw, ax_m_s2, ay_m_s2, az_m_s2, gx_rad_s, gy_rad_s, gz_rad_s)

        except Exception as e:
            self.get_logger().warn(f'Okuma hatasi: {e}')

    def publish_odom(self, now, qz, qw):
        odom = Odometry()

        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom)

        t = TransformStamped()

        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def publish_imu(self, now, qz, qw, ax_m_s2, ay_m_s2, az_m_s2, gx_rad_s, gy_rad_s, gz_rad_s):
        imu = Imu()

        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'imu_link'

        imu.orientation.z = qz
        imu.orientation.w = qw

        imu.orientation_covariance[0] = 0.05
        imu.orientation_covariance[4] = 0.05
        imu.orientation_covariance[8] = 0.1

        # Acceleration is already in m/s^2 from Arduino
        imu.linear_acceleration.x = ax_m_s2
        imu.linear_acceleration.y = ay_m_s2
        imu.linear_acceleration.z = az_m_s2

        # Gyro is already in rad/s from Arduino
        imu.angular_velocity.x = gx_rad_s
        imu.angular_velocity.y = gy_rad_s
        imu.angular_velocity.z = gz_rad_s

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)

    node = EncoderImuOdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
