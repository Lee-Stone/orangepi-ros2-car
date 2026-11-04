#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

import spidev


class ImuNode(Node):
    # MPU6500寄存器地址
    WHO_AM_I = 0x75
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B

    def __init__(self):
        super().__init__('imu_node')
        # 标准IMU消息供EKF使用，另发PoseStamped供RViz显示姿态
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'imu_pose', 10)
        self.spi = spidev.SpiDev()
        # SPI1片选0对应/dev/spidev1.0，MPU6500使用模式3
        self.spi.open(1, 0)
        self.spi.max_speed_hz = 500000
        self.spi.mode = 3

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = time.monotonic()
        self.gx_bias = 0.0
        self.gy_bias = 0.0
        self.gz_bias = 0.0

        if not self.init_mpu():
            raise RuntimeError('MPU6500初始化失败')

        # 上电后先静止校准零偏，再开始50Hz定时读取
        self.calibrate_gyro()
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('IMU节点已启动，正在发布话题/imu')

    def read_reg(self, reg):
        data = self.spi.xfer2([reg | 0x80, 0x00])
        return data[1]

    def read_bytes(self, reg, length):
        data = self.spi.xfer2([reg | 0x80] + [0x00] * length)
        return data[1:]

    def write_reg(self, reg, value):
        self.spi.xfer2([reg & 0x7F, value & 0xFF])

    # 复位、唤醒并配置量程和低通滤波
    def init_mpu(self):
        who = self.read_reg(self.WHO_AM_I)
        self.get_logger().info(f'MPU6500 WHO_AM_I=0x{who:02X}')
        if who != 0x70:
            self.get_logger().error('WHO_AM_I不是0x70，请检查SPI接线和CS')
            return False

        self.write_reg(self.PWR_MGMT_1, 0x80)   # 复位
        time.sleep(0.1)
        self.write_reg(self.PWR_MGMT_1, 0x01)   # 唤醒，选择PLL时钟
        self.write_reg(self.CONFIG, 0x03)        # DLPF 44Hz
        self.write_reg(self.SMPLRT_DIV, 0x09)    # 1kHz / 10 = 100Hz
        self.write_reg(self.GYRO_CONFIG, 0x00)   # ±250 dps
        self.write_reg(self.ACCEL_CONFIG, 0x00)  # ±2g
        time.sleep(0.1)
        return True

    # 静止采样求陀螺仪零偏，EKF融合前先去掉固定偏置
    def calibrate_gyro(self, samples=200):
        sx = sy = sz = 0.0
        for _ in range(samples):
            data = self.read_bytes(self.ACCEL_XOUT_H, 14)
            sx += self.get_word(data, 8)
            sy += self.get_word(data, 10)
            sz += self.get_word(data, 12)
            time.sleep(0.002)
        self.gx_bias = sx / samples
        self.gy_bias = sy / samples
        self.gz_bias = sz / samples
        self.get_logger().info(
            f'陀螺仪零偏gx={self.gx_bias:.1f} gy={self.gy_bias:.1f} gz={self.gz_bias:.1f}')

    def get_word(self, data, index):
        value = (data[index] << 8) | data[index + 1]
        if value >= 0x8000:
            value -= 0x10000
        return value

    def timer_callback(self):
        data = self.read_bytes(self.ACCEL_XOUT_H, 14)
        ax_raw = self.get_word(data, 0)
        ay_raw = self.get_word(data, 2)
        az_raw = self.get_word(data, 4)
        gx_raw = self.get_word(data, 8)
        gy_raw = self.get_word(data, 10)
        gz_raw = self.get_word(data, 12)

        ax = ax_raw / 16384.0 * 9.80665
        ay = ay_raw / 16384.0 * 9.80665
        az = az_raw / 16384.0 * 9.80665
        # 减去零偏后按±250dps量程换算成rad/s
        gx = (gx_raw - self.gx_bias) / 131.0 * math.pi / 180.0
        gy = (gy_raw - self.gy_bias) / 131.0 * math.pi / 180.0
        gz = (gz_raw - self.gz_bias) / 131.0 * math.pi / 180.0

        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        # 加速度计估计横滚/俯仰，陀螺仪积分做短期补偿
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        self.roll = 0.98 * (self.roll + gx * dt) + 0.02 * roll_acc
        self.pitch = 0.98 * (self.pitch + gy * dt) + 0.02 * pitch_acc
        self.yaw += gz * dt

        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        # IMU消息直接使用base_link坐标系
        msg.header.frame_id = 'base_link'
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        # 协方差表示数据可信程度，EKF会参考这些值分配权重
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01
        self.publisher_.publish(msg)

        # 同一姿态发布为PoseStamped，方便RViz用Pose显示
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'base_link'
        pose.pose.orientation = msg.orientation
        self.pose_publisher_.publish(pose)

    def close(self):
        self.spi.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
