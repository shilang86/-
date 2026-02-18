#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import math

class RedObjectFollower(Node):
    def __init__(self):
        super().__init__('red_object_follower')

        # 跟随参数配置（可根据需求调整）
        self.follow_distance = 0.6  # 小车与红色物体的目标距离(m)
        self.max_linear_vel = 0.3   # 最大线速度(m/s)
        self.max_angular_vel = 0.6  # 最大角速度(rad/s)
        self.angle_tolerance = 0.1  # 角度误差容忍(rad)
        self.distance_tolerance = 0.1  # 距离误差容忍(m)

        # 发布器：控制小车运动
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 订阅器：里程计（备用，可扩展）
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # TF2：坐标转换（world→map→base_footprint）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Gazebo服务：获取红色物体位姿
        self.gazebo_client = self.create_client(GetModelState, '/gazebo/get_model_state')
        while not self.gazebo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待/gazebo/get_model_state服务...")

        # 定时器：10Hz执行跟随逻辑
        self.timer = self.create_timer(0.1, self.follow_loop)

    def odom_callback(self, msg):
        """里程计回调（备用）"""
        pass

    def get_target_pose_in_base(self):
        """获取红色物体在机器人坐标系（base_footprint）下的位姿"""
        # 1. 从Gazebo获取红色物体在world坐标系的位姿
        req = GetModelState.Request()
        req.model_name = "red_target"  # 必须与Gazebo中红色物体名称一致
        req.relative_entity_name = "my_world"
        future = self.gazebo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result() or not future.result().success:
            self.get_logger().warn("无法获取红色物体位姿")
            return None

        # 2. 构造world坐标系下的位姿消息
        pose_world = PoseStamped()
        pose_world.header.frame_id = "my_world"
        pose_world.header.stamp = self.get_clock().now().to_msg()
        pose_world.pose = future.result().pose

        try:
            # 3. world → map（已保存的SLAM地图坐标系）
            tf_world_to_map = self.tf_buffer.lookup_transform(
                "map", "my_world", rclpy.time.Time()
            )
            pose_map = do_transform_pose(pose_world, tf_world_to_map)

            # 4. map → base_footprint（机器人本体坐标系）
            tf_map_to_base = self.tf_buffer.lookup_transform(
                "base_footprint", "map", rclpy.time.Time()
            )
            pose_base = do_transform_pose(pose_map, tf_map_to_base)

            return pose_base.pose
        except Exception as e:
            self.get_logger().warn(f"坐标转换失败: {str(e)[:50]}")
            return None

    def follow_loop(self):
        """核心跟随逻辑"""
        # 获取红色物体在机器人坐标系下的位姿
        target_pose = self.get_target_pose_in_base()
        if not target_pose:
            self.stop_robot()  # 无目标时停止
            return

        # 计算相对距离和角度
        dx = target_pose.position.x
        dy = target_pose.position.y
        relative_distance = math.hypot(dx, dy)
        relative_angle = math.atan2(dy, dx)
        # 角度归一化到[-π, π]
        relative_angle = math.atan2(math.sin(relative_angle), math.cos(relative_angle))

        # 构造控制指令
        twist = Twist()

        # 1. 优先调整角度（朝向红色物体）
        if abs(relative_angle) > self.angle_tolerance:
            twist.angular.z = self.max_angular_vel * (relative_angle / abs(relative_angle))
        # 2. 角度对齐后，调整距离
        else:
            distance_error = relative_distance - self.follow_distance
            if abs(distance_error) > self.distance_tolerance:
                # 线速度与距离误差成正比，且限幅
                twist.linear.x = max(
                    min(0.3 * distance_error, self.max_linear_vel), 
                    -self.max_linear_vel
                )

        # 发布控制指令
        self.cmd_vel_pub.publish(twist)
        # 日志输出（节流，避免刷屏）
        self.get_logger().info(
            f"相对距离: {relative_distance:.2f}m | 角度差: {relative_angle:.2f}rad | 线速度: {twist.linear.x:.2f}m/s",
            throttle_duration_sec=0.5
        )

    def stop_robot(self):
        """停止小车运动"""
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = RedObjectFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()  # 手动中断时停止小车
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
