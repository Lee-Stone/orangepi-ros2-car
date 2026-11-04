#!/usr/bin/env python3
import json
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters, SetParameters

PID_NAMES = ('angle_kp', 'angle_ki', 'speed_kp', 'speed_ki')
# WASD目标在key_node里做斜坡，vel_node对导航下发的cmd_vel保持直通
CMD_LINEAR_ACCEL = 1.5
CMD_ANGULAR_ACCEL = 4.0
TARGET_FILE = os.path.expanduser('~/.config/car_pkg/key_targets.json')


class KeyNode(Node):
    def __init__(self):
        super().__init__('key_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.02, self.step_command)

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.desired_linear = 0.0
        self.desired_angular = 0.0
        self.ramp_linear = 0.0
        self.ramp_angular = 0.0
        self.actual_linear = 0.0
        self.actual_angular = 0.0
        self.loop_name = '角度环'
        self.w_linear = 0.15
        self.w_angular = 1.50
        self.pid = {
            'angle_kp': 0.15, 'angle_ki': 0.0,
            'speed_kp': 0.60, 'speed_ki': 0.20,
        }
        self.last_key_time = 0.0
        self.last_draw_time = 0.0
        self.last_publish_time = time.monotonic()
        self.input_mode = False
        self.old_term = None
        self.fd = sys.stdin.fileno()

        self.load_targets()
        self.get_param_client = self.create_client(
            GetParameters, '/vel_node/get_parameters')
        self.set_param_client = self.create_client(
            SetParameters, '/vel_node/set_parameters')

    def load_targets(self):
        try:
            with open(TARGET_FILE, 'r') as f:
                data = json.load(f)
            self.w_linear = max(0.0, min(0.5, float(data.get('linear', self.w_linear))))
            self.w_angular = max(0.0, min(2.0, float(data.get('angular', self.w_angular))))
        except (OSError, ValueError, TypeError):
            pass

    def save_targets(self):
        try:
            os.makedirs(os.path.dirname(TARGET_FILE), exist_ok=True)
            with open(TARGET_FILE, 'w') as f:
                json.dump({'linear': self.w_linear, 'angular': self.w_angular}, f)
        except OSError:
            pass

    def wait_for_vel_node(self):
        # key_node可以比vel_node先启动，找不到时一直等待
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            names = [n.strip('/') for n in self.get_node_names()]
            if 'vel_node' in names:
                print('检测到vel_node，进入PID调试界面。', flush=True)
                return True
            print('等待vel_node启动...', end='\r', flush=True)
            time.sleep(0.3)
        return False

    def get_pid_from_vel_node(self):
        if not self.get_param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('没有找到vel_node参数服务，使用默认PID')
            return
        req = GetParameters.Request(names=list(PID_NAMES))
        future = self.get_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            self.get_logger().warn('读取vel_node参数失败，使用默认PID')
            return
        for name, value in zip(PID_NAMES, future.result().values):
            self.pid[name] = value.double_value

    def send_pid(self):
        # 通过ROS2参数服务把当前PID写回vel_node
        if not self.set_param_client.wait_for_service(timeout_sec=1.0):
            return False
        params = [
            ParameterMsg(
                name=name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=value,
                ),
            )
            for name, value in self.pid.items()
        ]
        req = SetParameters.Request(parameters=params)
        future = self.set_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.done() and future.result() is not None and all(
            r.successful for r in future.result().results)

    def cmd_callback(self, msg):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z

    def odom_callback(self, msg):
        self.actual_linear = msg.twist.twist.linear.x
        self.actual_angular = msg.twist.twist.angular.z

    def publish_cmd(self, linear, angular):
        # 按键只设置期望目标，step_command按20ms周期平滑发布
        if abs(linear) < 1e-6 and abs(angular) < 1e-6:
            self.zero_cmd()
            return
        self.desired_linear = linear
        self.desired_angular = angular
        self.last_key_time = time.monotonic()

    def zero_cmd(self):
        self.desired_linear = 0.0
        self.desired_angular = 0.0
        self.ramp_linear = 0.0
        self.ramp_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        msg = Twist()
        self.cmd_pub.publish(msg)

    def step_command(self):
        now = time.monotonic()
        dt = min(max(now - self.last_publish_time, 0.0), 0.1)
        self.last_publish_time = now
        if abs(self.desired_linear) < 1e-6 and abs(self.desired_angular) < 1e-6:
            return
        linear_step = CMD_LINEAR_ACCEL * dt
        angular_step = CMD_ANGULAR_ACCEL * dt
        # ramp_*只保存自己生成的目标，避免/cmd_vel回调和斜坡状态互相覆盖
        self.ramp_linear += max(-linear_step, min(linear_step, self.desired_linear - self.ramp_linear))
        self.ramp_angular += max(-angular_step, min(angular_step, self.desired_angular - self.ramp_angular))
        self.target_linear = self.ramp_linear
        self.target_angular = self.ramp_angular
        msg = Twist()
        msg.linear.x = self.ramp_linear
        msg.angular.z = self.ramp_angular
        self.cmd_pub.publish(msg)

    def draw(self):
        if self.loop_name == '角度环':
            loop_mark = '角度环 <=='
            speed_mark = '速度环'
        else:
            loop_mark = '角度环'
            speed_mark = '速度环 <=='
        angle_err = self.target_angular - self.actual_angular
        speed_err = self.target_linear - self.actual_linear
        lines = [
            '================ 小车PID调试 ================',
            f'{loop_mark}    当前环按 ←/→ 切换    {speed_mark}',
            f'角度环  Kp={self.pid["angle_kp"]:.3f}  Ki={self.pid["angle_ki"]:.3f}',
            f'        目标ω={self.target_angular:+.2f} rad/s  实际ω={self.actual_angular:+.2f} rad/s  误差={angle_err:+.2f}',
            f'速度环  Kp={self.pid["speed_kp"]:.3f}  Ki={self.pid["speed_ki"]:.3f}',
            f'        目标v={self.target_linear:+.2f} m/s  实际v={self.actual_linear:+.2f} m/s  误差={speed_err:+.2f}',
            '',
            f'W/S：线速度 ±{self.w_linear:.2f} m/s    A/D：角速度 ±{self.w_angular:.2f} rad/s',
            'Ctrl+P：输入P    Ctrl+I：输入I    Ctrl+T：改目标    ←/→：切换环    Ctrl+C：保存并退出',
        ]
        sys.stdout.write('\033[2J\033[H' + '\r\n'.join(lines) + '\r\n')
        sys.stdout.flush()
        self.last_draw_time = time.monotonic()

    def read_key(self, timeout=0.05):
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return None
        return os.read(self.fd, 1)

    def handle_key(self, ch):
        if ch == b'\x10':          # Ctrl+P
            self.enter_pid('P')
        elif ch == b'\x09':        # Ctrl+I
            self.enter_pid('I')
        elif ch == b'\x14':        # Ctrl+T
            self.enter_target()
        elif ch == b'\x1b':        # 方向键
            self.read_arrow()
        elif ch in (b'w', b'W'):
            self.publish_cmd(self.w_linear, 0.0)
        elif ch in (b's', b'S'):
            self.publish_cmd(-self.w_linear, 0.0)
        elif ch in (b'a', b'A'):
            self.publish_cmd(0.0, self.w_angular)
        elif ch in (b'd', b'D'):
            self.publish_cmd(0.0, -self.w_angular)

    def read_arrow(self):
        first = self.read_key(0.05)
        if first != b'[':
            return
        second = self.read_key(0.05)
        if second == b'D':
            self.loop_name = '角度环'
        elif second == b'C':
            self.loop_name = '速度环'

    def enter_target(self):
        # Ctrl+T修改当前环的WASD目标值
        self.input_mode = True
        self.zero_cmd()
        is_angle = self.loop_name == '角度环'
        current = self.w_angular if is_angle else self.w_linear
        unit = 'rad/s' if is_angle else 'm/s'
        limit = 2.0 if is_angle else 0.5
        sys.stdout.write(f'\r\n输入{self.loop_name}目标值(当前{current:.2f}{unit}，Enter确认，Esc取消):')
        sys.stdout.flush()
        buf = ''
        while True:
            ch = self.read_key(0.2)
            if ch is None:
                continue
            if ch in (b'\r', b'\n'):
                if buf:
                    try:
                        value = abs(float(buf))
                    except ValueError:
                        value = current
                    value = max(0.0, min(limit, value))
                    if is_angle:
                        self.w_angular = value
                    else:
                        self.w_linear = value
                    self.save_targets()
                    sys.stdout.write(f'\r\n目标值已改为{value:.2f}{unit}')
                    sys.stdout.flush()
                    time.sleep(0.3)
                break
            if ch == b'\x03':
                raise KeyboardInterrupt
            if ch == b'\x1b':
                break
            if ch in (b'\x7f', b'\x08'):
                buf = buf[:-1]
            elif ch in b'0123456789.-':
                buf += ch.decode()
                sys.stdout.write(ch.decode())
                sys.stdout.flush()
        self.input_mode = False
        self.last_draw_time = 0.0

    def enter_pid(self, kind):
        # 输入期间先停车，避免改参数时小车继续跑
        self.input_mode = True
        self.zero_cmd()
        prefix = 'angle' if self.loop_name == '角度环' else 'speed'
        name = f'{prefix}_k{kind.lower()}'
        current = self.pid[name]
        sys.stdout.write(f'\r\n输入{self.loop_name}{kind}值(当前{current:.3f}，Enter确认，Esc取消):')
        sys.stdout.flush()
        buf = ''
        while True:
            ch = self.read_key(0.2)
            if ch is None:
                continue
            if ch in (b'\r', b'\n'):
                if buf:
                    try:
                        value = float(buf)
                    except ValueError:
                        value = current
                    self.pid[name] = value
                    ok = self.send_pid()
                    sys.stdout.write('\r\n参数已写入vel_node' if ok else '\r\n参数写入失败')
                    sys.stdout.flush()
                    time.sleep(0.3)
                break
            if ch == b'\x03':
                raise KeyboardInterrupt
            if ch == b'\x1b':
                break
            if ch in (b'\x7f', b'\x08'):
                buf = buf[:-1]
            elif ch in b'0123456789.-':
                buf += ch.decode()
                sys.stdout.write(ch.decode())
                sys.stdout.flush()
        self.input_mode = False
        self.last_draw_time = 0.0

    def run(self):
        try:
            if not self.wait_for_vel_node():
                return 1
        except KeyboardInterrupt:
            print('已取消等待。')
            return 1
        self.get_pid_from_vel_node()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.monotonic()
                # 终端收不到真实松开事件，超时200ms就把目标清零
                if (not self.input_mode and now - self.last_key_time > 0.2
                        and (abs(self.desired_linear) > 1e-6
                             or abs(self.desired_angular) > 1e-6)):
                    self.zero_cmd()
                if now - self.last_draw_time > 0.15:
                    self.draw()
                ch = self.read_key(0.02)
                if ch == b'\x03':
                    break
                if ch is not None:
                    self.handle_key(ch)

        except KeyboardInterrupt:
            pass
        finally:
            self.save_targets()
            self.send_pid()
            try:
                self.zero_cmd()
            except Exception:
                pass
            if self.old_term is not None:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
            print('\r\nPID已保存，目标速度已清零，key_node退出。')
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = KeyNode()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
