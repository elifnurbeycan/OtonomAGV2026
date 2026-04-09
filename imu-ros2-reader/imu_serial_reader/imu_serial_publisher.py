import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuSerialPublisher(Node):
    def __init__(self):
        super().__init__('imu_serial_publisher')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)

        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Serial baglandi: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Serial acilamadi: {e}')
            raise

        self.timer = self.create_timer(0.02, self.read_and_publish)

    def read_and_publish(self):
        try:
            line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            if (
                'READY' in line
                or 'Basliyor' in line
                or 'kalibrasyonu' in line
                or 'DEVID' in line
                or 'WHO_AM_I' in line
            ):
                return

            parts = line.split(',')
            if len(parts) != 6:
                return

            ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps = map(float, parts)

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # Linear acceleration: g -> m/s^2
            msg.linear_acceleration.x = ax_g * 9.80665
            msg.linear_acceleration.y = ay_g * 9.80665
            msg.linear_acceleration.z = az_g * 9.80665

            # Angular velocity: deg/s -> rad/s
            msg.angular_velocity.x = math.radians(gx_dps)
            msg.angular_velocity.y = math.radians(gy_dps)
            msg.angular_velocity.z = math.radians(gz_dps)

            # Accelerometer'dan roll ve pitch hesapla
            roll = math.atan2(ay_g, az_g)
            pitch = math.atan2(-ax_g, math.sqrt(ay_g ** 2 + az_g ** 2))
            yaw = 0.0

            # Euler -> Quaternion
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            msg.orientation.w = cr * cp * cy + sr * sp * sy
            msg.orientation.x = sr * cp * cy - cr * sp * sy
            msg.orientation.y = cr * sp * cy + sr * cp * sy
            msg.orientation.z = cr * cp * sy - sr * sp * cy

            # Covariance değerleri
            msg.orientation_covariance[0] = 0.01
            msg.orientation_covariance[4] = 0.01
            msg.orientation_covariance[8] = 99999.0  # yaw güvenilmez

            msg.angular_velocity_covariance[0] = 0.01
            msg.angular_velocity_covariance[4] = 0.01
            msg.angular_velocity_covariance[8] = 0.01

            msg.linear_acceleration_covariance[0] = 0.01
            msg.linear_acceleration_covariance[4] = 0.01
            msg.linear_acceleration_covariance[8] = 0.01

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Okuma hatasi: {e}')

    def destroy(self):
        try:
            if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
                self.serial_conn.close()
        except Exception:
            pass
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ImuSerialPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()