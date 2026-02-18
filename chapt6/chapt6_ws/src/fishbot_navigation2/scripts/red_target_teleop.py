#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

# 键盘映射（键值→速度）
KEY_MAPPING = {
    'w': (0.3, 0.0),   # 前进
    's': (-0.3, 0.0),  # 后退
    'a': (0.0, 0.6),   # 左转
    'd': (0.0, -0.6),  # 右转
    ' ': (0.0, 0.0)    # 停止
}

class RedTargetTeleop(Node):
    def __init__(self):
        super().__init__('red_target_teleop')
        
        # 发布器：控制红色物体运动
        self.cmd_vel_pub = self.create_publisher(Twist, '/red_target/cmd_vel', 10)
        
        # 保存终端属性（恢复用）
        self.terminal_settings = termios.tcgetattr(sys.stdin)
        
        # 提示信息
        self.get_logger().info("=== 红色物体键盘控制 ===")
        self.get_logger().info("W:前进 | S:后退 | A:左转 | D:右转 | 空格:停止 | Q:退出")

    def get_key(self):
        """非阻塞获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
        return key

    def run(self):
        """主循环：监听键盘并发布控制指令"""
        while rclpy.ok():
            key = self.get_key()
            if key in KEY_MAPPING:
                # 构造速度指令
                twist = Twist()
                twist.linear.x = KEY_MAPPING[key][0]
                twist.angular.z = KEY_MAPPING[key][1]
                self.cmd_vel_pub.publish(twist)
            elif key == 'q':
                # 退出时停止红色物体
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("退出红色物体控制")
                break
            # 自旋一次，处理ROS回调
            rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    node = RedTargetTeleop()
    try:
        node.run()
    finally:
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.terminal_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
