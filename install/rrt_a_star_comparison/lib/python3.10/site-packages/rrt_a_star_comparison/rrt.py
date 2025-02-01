import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
import random
import math
import time

class RRT(Node):
    def __init__(self):
        super().__init__('rrt_node')
        # 发布路径
        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)
        # 发布起点和终点的Marker
        self.marker_pub = self.create_publisher(Marker, '/rrt_markers', 10)
        # 发布性能指标
        self.performance_pub = self.create_publisher(Float32MultiArray, '/rrt_performance', 10)
        # 降低发布频率，每1秒发布一次路径
        self.timer = self.create_timer(1.0, self.plan)

        # 地图范围和目标点
        self.map_width = 100
        self.map_height = 100
        self.start = (10, 10)  # 起点
        self.goal = (90, 90)   # 终点
        self.step_size = 5.0   # 步长
        self.max_iterations = 1000  # 最大迭代次数
        self.goal_threshold = 5.0  # 目标点阈值

        # 性能指标
        self.start_time = time.time()
        self.node_expansions = 0

        # 存储路径节点的成员变量
        self.nodes = []

    def plan(self):
        self.nodes = [self.start]  # 重置路径节点
        for _ in range(self.max_iterations):
            self.node_expansions += 1  # 记录节点扩展数
            random_point = (random.uniform(0, self.map_width), random.uniform(0, self.map_height))
            nearest_node = self.find_nearest_node(random_point, self.nodes)
            new_node = self.steer(nearest_node, random_point)

            if self.check_collision(nearest_node, new_node):
                self.nodes.append(new_node)
                if self.distance(new_node, self.goal) < self.goal_threshold:
                    self.publish_path(self.nodes)
                    self.publish_markers()
                    self.log_performance()  # 记录性能指标
                    return

        self.get_logger().info("RRT Path Not Found!")

    def find_nearest_node(self, point, nodes):
        return min(nodes, key=lambda n: self.distance(n, point))

    def steer(self, from_node, to_point):
        angle = math.atan2(to_point[1] - from_node[1], to_point[0] - from_node[0])
        new_node = (
            from_node[0] + self.step_size * math.cos(angle),
            from_node[1] + self.step_size * math.sin(angle)
        )
        return new_node

    def check_collision(self, from_node, to_node):
        # 这里可以添加碰撞检测逻辑
        return True

    def distance(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def publish_path(self, nodes):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for node in nodes:
            pose = PoseStamped()
            pose.pose.position = Point(x=float(node[0]), y=float(node[1]), z=0.0)
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.get_logger().info(f"RRT Path Published with {len(nodes)} nodes")

    def publish_markers(self):
        # 发布起点Marker
        start_marker = Marker()
        start_marker.header.frame_id = "map"
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = "start"
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position = Point(x=float(self.start[0]), y=float(self.start[1]), z=0.0)
        start_marker.scale.x = 2.0
        start_marker.scale.y = 2.0
        start_marker.scale.z = 2.0
        start_marker.color.r = 0.0  # 绿色
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        self.marker_pub.publish(start_marker)

        # 发布终点Marker
        goal_marker = Marker()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = "goal"
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position = Point(x=float(self.goal[0]), y=float(self.goal[1]), z=0.0)
        goal_marker.scale.x = 2.0
        goal_marker.scale.y = 2.0
        goal_marker.scale.z = 2.0
        goal_marker.color.r = 1.0  # 红色
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        self.marker_pub.publish(goal_marker)

        self.get_logger().info("Start and Goal Markers Published!")

    def log_performance(self):
        elapsed_time = time.time() - self.start_time
        path_length = self.calculate_path_length(self.nodes)
        
        # 确保值在 float 范围内
        if not (-340282346600000016151267322115014000640.000000 <= elapsed_time <= 340282346600000016151267322115014000640.000000):
            elapsed_time = 0.0  # 如果超出范围，设置为默认值
        if not (-340282346600000016151267322115014000640.000000 <= path_length <= 340282346600000016151267322115014000640.000000):
            path_length = 0.0  # 如果超出范围，设置为默认值
        
        # 将 node_expansions 转换为 float
        node_expansions = float(self.node_expansions)
        
        # 发布性能指标
        performance_msg = Float32MultiArray()
        performance_msg.data = [elapsed_time, path_length, node_expansions]
        self.performance_pub.publish(performance_msg)
        
        self.get_logger().info(
            f"RRT Performance: "
            f"Time = {elapsed_time:.4f} s, "
            f"Path Length = {path_length:.4f}, "
            f"Node Expansions = {self.node_expansions}"
        )

    def calculate_path_length(self, nodes):
        length = 0.0
        for i in range(1, len(nodes)):
            length += self.distance(nodes[i-1], nodes[i])
        return length

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRT()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()