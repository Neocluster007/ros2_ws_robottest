#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from builtin_interfaces.msg import Time
import time
from rclpy.qos import QoSProfile

class LaserTFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf')
        print("[Brobot][TF] TF initial BRobot......!")

        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(Time, 'ros_time', QoSProfile(depth=10))
        self.timer = self.create_timer(0.1, self.broadcast_tf)

        print("[Brobot][TF] TF Ready")

    def broadcast_tf(self):

        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        #t.header.stamp = Time()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'

        # ตำแหน่งของ LIDAR
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2

        # คำนวณ quaternion จาก euler(roll, pitch, yaw)
        roll = 0.0
        pitch = 0.0
        yaw = math.pi
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)
        self.pub.publish(now)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # ใช้สูตรจาก ROS Wiki
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)

def main():
    rclpy.init()
    node = LaserTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
