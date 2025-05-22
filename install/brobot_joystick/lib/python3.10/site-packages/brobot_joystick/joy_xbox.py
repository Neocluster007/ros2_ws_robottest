import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import time


class JoyCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('joy_cmdvel_node')

        # Publisher ไปยัง /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # เริ่มต้น pygame และจอยสติ๊ก
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("❌ ไม่พบจอยสติ๊ก!")
            exit()

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.get_logger().info(f"🎮 จอย: {self.joy.get_name()}")

        # ตั้งค่า scale ความเร็ว
        self.max_linear = 1.0   # m/s
        self.max_angular = 1.0  # rad/s

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def timer_callback(self):
        pygame.event.pump()

        # แกนที่ใช้: แกนขวาแนวตั้ง -> linear x | แกนซ้ายแนวนอน -> angular z
        linear_input = -self.joy.get_axis(4)  # ขวาแนวตั้ง
        angular_input = self.joy.get_axis(0)  # ซ้ายแนวนอน

        twist = Twist()
        twist.linear.x = linear_input * self.max_linear
        twist.angular.z = (angular_input*(-1)) * self.max_angular
        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"📤 cmd_vel - linear: {twist.linear.x:.2f} m/s | angular: {twist.angular.z:.2f} rad/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = JoyCmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
