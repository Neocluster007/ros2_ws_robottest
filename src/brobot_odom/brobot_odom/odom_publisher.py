#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
import math 

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom') 

        print("[Brobot][Odom] Odom initial BRobot......!")

        # Subscribe encoder
        self.sub_left = self.create_subscription(Int32, '/encoder_L', self.left_callback, 10)
        self.sub_right = self.create_subscription(Int32, '/encoder_R', self.right_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.br = TransformBroadcaster(self) 

        # Robot state
        self.enc_L = 0
        self.enc_R = 0
        self.prev_L = 0
        self.prev_R = 0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # ✅ ปรับตามที่คุณระบุ
        self.wheel_radius = 0.022     # 44mm / 2
        self.wheel_base = 0.128       # 128mm
        self.ticks_per_rev = 407

        self.prev_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update_odom)

        print("[Brobot][Odom] Odom Ready")

    def left_callback(self, msg):
        self.enc_L = msg.data

    def right_callback(self, msg):
        self.enc_R = msg.data

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        delta_L = self.enc_L - self.prev_L
        delta_R = self.enc_R - self.prev_R
        self.prev_L = self.enc_L
        self.prev_R = self.enc_R

        dist_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        dL = delta_L * dist_per_tick
        dR = delta_R * dist_per_tick

        d_center = (dR + dL) / 2.0
        d_theta = (dR - dL) / self.wheel_base

        self.th += d_theta
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)

        q = self.yaw_to_quaternion(self.th)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)

        # Send TF
        t = TransformStamped() 
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.br.sendTransform(t)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

def main():
    rclpy.init()
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
