#!/usr/bin/env python3
import os
import time

import gpiod
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


PWM_ROOT = '/sys/class/pwm/pwmchip0'
PWM_PERIOD_NS = 100000
MAX_WHEEL_SPEED = 0.5
# gpiochip0偏移：左轮AIN1/AIN2，右轮BIN1/BIN2
DIR_OFFSETS = (233, 76, 256, 271)


class MotorDriver:
    def __init__(self):
        self.chip = gpiod.Chip('gpiochip0')
        self.lines = []
        for offset in DIR_OFFSETS:
            line = self.chip.get_line(offset)
            # TB6612方向脚先全部拉低，上电时电机不会乱转
            line.request(consumer='car_motor', type=gpiod.LINE_REQ_DIR_OUT,
                         default_vals=[0])
            self.lines.append(line)
        self.last_duty = {1: 0, 2: 0}
        self.setup_pwm()

    @staticmethod
    def write(path, value):
        with open(path, 'w') as f:
            f.write(str(value))

    def setup_pwm(self):
        # 使用10kHz PWM，占空比按0~100000ns写
        for ch in (1, 2):
            pwm_dir = f'{PWM_ROOT}/pwm{ch}'
            if not os.path.exists(pwm_dir):
                self.write(f'{PWM_ROOT}/export', ch)
                time.sleep(0.1)
            self.write(f'{pwm_dir}/period', PWM_PERIOD_NS)
            self.write(f'{pwm_dir}/duty_cycle', 0)
            self.write(f'{pwm_dir}/enable', 1)

    def car_move(self, left, right):
        # 公式里的car_move：正速度前进，负速度后退
        self.set_side(0, 1, 1, left)
        self.set_side(2, 3, 2, right)

    def set_side(self, in1_index, in2_index, channel, speed):
        speed = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, speed))
        if speed > 0.002:
            self.lines[in1_index].set_value(1)
            self.lines[in2_index].set_value(0)
        elif speed < -0.002:
            self.lines[in1_index].set_value(0)
            self.lines[in2_index].set_value(1)
        else:
            self.lines[in1_index].set_value(0)
            self.lines[in2_index].set_value(0)
            speed = 0.0
        duty = int(abs(speed) / MAX_WHEEL_SPEED * PWM_PERIOD_NS)
        if duty != self.last_duty[channel]:
            self.write(f'{PWM_ROOT}/pwm{channel}/duty_cycle', duty)
            self.last_duty[channel] = duty

    def stop(self):
        for ch in (1, 2):
            self.write(f'{PWM_ROOT}/pwm{ch}/duty_cycle', 0)
            self.last_duty[ch] = 0
        for line in self.lines:
            line.set_value(0)

    def close(self):
        self.stop()
        for line in self.lines:
            line.release()
        self.chip.close()


class VelNode(Node):
    def __init__(self):
        super().__init__('vel_node')
        self.declare_parameter('angle_kp', 0.11)
        self.declare_parameter('angle_ki', 0.0)
        self.declare_parameter('speed_kp', 0.60)
        self.declare_parameter('speed_ki', 0.20)
        self.angle_kp = self.get_parameter('angle_kp').value
        self.angle_ki = self.get_parameter('angle_ki').value
        self.speed_kp = self.get_parameter('speed_kp').value
        self.speed_ki = self.get_parameter('speed_ki').value
        self.add_on_set_parameters_callback(self.on_parameters)

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.actual_linear = 0.0
        self.actual_angular = 0.0
        self.speed_integral = 0.0
        self.angle_integral = 0.0
        self.pid_log_pending = False
        self.last_time = time.monotonic()

        # car_move的两路输出最终作用到TB6612的PWMA和PWMB
        self.motor = MotorDriver()
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_timer(0.02, self.control_loop)
        self.get_logger().info(
            f'小车控制节点已启动：角度环Kp={self.angle_kp:.3f} Ki={self.angle_ki:.3f}，'
            f'速度环Kp={self.speed_kp:.3f} Ki={self.speed_ki:.3f}')

    def on_parameters(self, params):
        # key_node退出前通过参数服务把调好的PID写回这里
        changed = False
        for p in params:
            if p.name == 'angle_kp' and abs(self.angle_kp - float(p.value)) > 1e-9:
                self.angle_kp = float(p.value); changed = True
            elif p.name == 'angle_ki' and abs(self.angle_ki - float(p.value)) > 1e-9:
                self.angle_ki = float(p.value); changed = True
            elif p.name == 'speed_kp' and abs(self.speed_kp - float(p.value)) > 1e-9:
                self.speed_kp = float(p.value); changed = True
            elif p.name == 'speed_ki' and abs(self.speed_ki - float(p.value)) > 1e-9:
                self.speed_ki = float(p.value); changed = True
        if changed:
            self.pid_log_pending = True
        return SetParametersResult(successful=True)

    def odom_callback(self, msg):
        self.actual_linear = msg.twist.twist.linear.x
        self.actual_angular = msg.twist.twist.angular.z

    def cmd_callback(self, msg):
        self.target_linear = max(-0.5, min(0.5, msg.linear.x))
        self.target_angular = max(-2.0, min(2.0, msg.angular.z))

    def control_loop(self):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0:
            return

        if self.pid_log_pending:
            self.pid_log_pending = False
            self.get_logger().info(
                f'PID参数更新：角度环Kp={self.angle_kp:.3f} Ki={self.angle_ki:.3f}，'
                f'速度环Kp={self.speed_kp:.3f} Ki={self.speed_ki:.3f}')

        # 目标清零时直接停车，并清掉积分，避免松键后继续爬行
        if abs(self.target_linear) < 1e-6 and abs(self.target_angular) < 1e-6:
            self.speed_integral = 0.0
            self.angle_integral = 0.0
            self.motor.stop()
            return

        speed_error = self.target_linear - self.actual_linear
        angle_error = self.target_angular - self.actual_angular

        # 速度环PI：输出左右轮共同的前进速度b
        self.speed_integral += speed_error * dt
        self.speed_integral = max(-0.5, min(0.5, self.speed_integral))
        b = self.speed_kp * speed_error + self.speed_ki * self.speed_integral
        b = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, b))

        # 角度环P：输出左右轮反向的修正量a
        # 正角速度目标对应左轮后退、右轮前进，所以这里取负号
        self.angle_integral += angle_error * dt
        self.angle_integral = max(-0.5, min(0.5, self.angle_integral))
        a = -(self.angle_kp * angle_error + self.angle_ki * self.angle_integral)
        a = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, a))

        # 并级PID：左轮b+a，右轮b-a
        self.motor.car_move(b + a, b - a)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VelNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.motor.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
