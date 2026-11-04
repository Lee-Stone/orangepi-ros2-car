#!/usr/bin/env python3
import math
import select
import threading

import gpiod
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# 底盘实测参数：轮半径、轮距、轮子转一圈的编码器计数
WHEEL_RADIUS = 0.024
WHEEL_BASE = 0.122
COUNTS_PER_REV = 1040
GPIO_CHIP = 'gpiochip0'
# gpiochip0偏移：左A/B、右A/B
LEFT_A, LEFT_B = 259, 260
RIGHT_A, RIGHT_B = 258, 272


# 读取一路编码器的A/B相，用状态跳变做4倍频计数
class WheelEncoder:
    def __init__(self, a_offset, b_offset, name, invert=False):
        self.count = 0
        self.lock = threading.Lock()
        self.running = True
        self.invert = invert
        self.chip = gpiod.Chip(GPIO_CHIP)
        self.a = self.chip.get_line(a_offset)
        self.b = self.chip.get_line(b_offset)
        # 双边沿触发，A、B相各触发一次，一个周期得到4个计数
        for line, label in ((self.a, name + '_a'), (self.b, name + '_b')):
            line.request(consumer=label, type=gpiod.LINE_REQ_EV_BOTH_EDGES,
                         flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
        self.a_fd, self.b_fd = self.a.event_get_fd(), self.b.event_get_fd()
        self.state = (self.a.get_value() << 1) | self.b.get_value()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        quad = (0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0)
        while self.running:
            ready, _, _ = select.select([self.a_fd, self.b_fd], [], [], 0.1)
            for fd in ready:
                line = self.a if fd == self.a_fd else self.b
                event = line.event_read()
                level = 1 if event.type == 1 else 0
                if line is self.a:
                    cur = (level << 1) | self.b.get_value()
                else:
                    cur = (self.a.get_value() << 1) | level
                delta = quad[(self.state << 2) | cur]
                if self.invert:
                    delta = -delta
                self.state = cur
                with self.lock:
                    self.count += delta

    def read(self):
        with self.lock:
            return self.count

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.a.release()
        self.b.release()
        self.chip.close()


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        # 融合前的轮式odom只发布/wheel_odom，TF由EKF统一发布
        self.pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        # 左轮A/B接线方向相反，计数取反后前进为正
        self.left = WheelEncoder(LEFT_A, LEFT_B, 'left_enc', invert=True)
        self.right = WheelEncoder(RIGHT_A, RIGHT_B, 'right_enc')
        self.x = self.y = self.theta = 0.0
        self.last_left = self.left.read()
        self.last_right = self.right.read()
        self.last_time = self.get_clock().now()
        # 50Hz读取一次编码器并发布/wheel_odom
        self.create_timer(0.02, self.update)
        self.get_logger().info('轮式里程计节点已启动，正在发布/wheel_odom')

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            return
        l, r = self.left.read(), self.right.read()
        # 计数差换算成左右轮走过的弧长
        dl = (l - self.last_left) / COUNTS_PER_REV * 2.0 * math.pi * WHEEL_RADIUS
        dr = (r - self.last_right) / COUNTS_PER_REV * 2.0 * math.pi * WHEEL_RADIUS
        self.last_left, self.last_right = l, r
        self.last_time = now

        # 差速运动学：平均位移给线速度，左右轮差给角速度
        v = (dl + dr) / (2.0 * dt)
        w = (dr - dl) / (WHEEL_BASE * dt)
        self.theta += w * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        # 位姿在odom坐标系，twist相对base_link
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        # 给EKF使用的twist协方差：线速度和角速度不要被当成绝对正确
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[35] = 0.02
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.left.close()
        node.right.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
