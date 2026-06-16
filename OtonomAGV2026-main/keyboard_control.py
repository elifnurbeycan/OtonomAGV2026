import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardControl(Node):

    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        return key

    def run(self):
        print("WASD Durum Kontrollü Sürüş Aktif")
        print("w = ileri git (geri gidiyorsa dur)")
        print("s = geri git (ileri gidiyorsa dur)")
        print("a = sola dön (sağa dönüyorsa dur)")
        print("d = sağa dön (sola dönüyorsa dur)")
        print("x = acil durdur")
        print("q = çıkış")
        print("-" * 35)

        linear_x = 0.0
        angular_z = 0.0

        while True:
            key = self.get_key()

            if key == 'w':
                if linear_x < 0:
                    linear_x = 0.0
                else:
                    linear_x = 0.3
            elif key == 's':
                if linear_x > 0:
                    linear_x = 0.0
                else:
                    linear_x = -0.3
            elif key == 'a':
                if angular_z < 0:
                    angular_z = 0.0
                else:
                    angular_z = 1.0
            elif key == 'd':
                if angular_z > 0:
                    angular_z = 0.0
                else:
                    angular_z = -1.0
            elif key == 'x':
                linear_x = 0.0
                angular_z = 0.0
            elif key == 'q':
                # Stop robot on exit
                msg = Twist()
                self.publisher_.publish(msg)
                print("\nÇıkış yapılıyor...")
                break
            else:
                continue

            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.publisher_.publish(msg)

            # Clear line and print current speeds
            sys.stdout.write(f"\rŞu anki Hız -> Doğrusal: {linear_x:.1f} m/s | Açısal: {angular_z:.1f} rad/s   ")
            sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)

    node = KeyboardControl()
    node.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
