import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
import heapq
import math
import time

class AStar(Node):
    def __init__(self):
        super().__init__('a_star_node')
        # 发布路径
        self.path_pub = self.create_publisher(Path, '/a_star_path', 10)
        # 发布起点和终点的Marker
        self.marker_pub = self.create_publisher(Marker, '/a_star_markers', 10)
        # 发布性能指标
        self.performance_pub = self.create_publisher(Float32MultiArray, '/a_star_performance', 10)
        # 降低发布频率，每1秒发布一次路径
        self.timer = self.create_timer(1.0, self.plan)

        # 地图范围和目标点
        self.map_width = 100
        self.map_height = 100
        self.start = (10, 10)  # 起点
        self.goal = (90, 90)   # 终点

        # 性能指标
        self.start_time = time.time()
        self.node_expansions = 0

        # 存储路径信息的成员变量
        self.came_from = {}

    def plan(self):
        open_list = []
        heapq.heappush(open_list, (0, self.start))
        self.came_from = {}  # 重置路径信息
        cost_so_far = {self.start: 0}

        while open_list:
            current_cost, current_node = heapq.heappop(open_list)
            self.node_expansions += 1  # 记录节点扩展数

            if current_node == self.goal:
                path_nodes = self.reconstruct_path(self.came_from)
                self.publish_path(path_nodes)
                self.publish_markers()
                self.log_performance()  # 记录性能指标
                return

            for next_node in self.get_neighbors(current_node):
                new_cost = cost_so_far[current_node] + self.distance(current_node, next_node)
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, self.goal)
                    heapq.heappush(open_list, (priority, next_node))
                    self.came_from[next_node] = current_node

        self.get_logger().info("A* Path Not Found!")

    def get_neighbors(self, node):
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_node = (node[0] + dx, node[1] + dy)
            if 0 <= new_node[0] < self.map_width and 0 <= new_node[1] < self.map_height:
                neighbors.append(new_node)
        return neighbors

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def distance(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def reconstruct_path(self, came_from):
        current_node = self.goal
        path = [current_node]
        while current_node in came_from:
            current_node = came_from[current_node]
            path.append(current_node)
        path.reverse()
        return path

    def publish_path(self, nodes):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for node in nodes:
            pose = PoseStamped()
            pose.pose.position = Point(x=float(node[0]), y=float(node[1]), z=0.0)
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.get_logger().info(f"A* Path Published with {len(nodes)} nodes")

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
        path_length = self.calculate_path_length(self.reconstruct_path(self.came_from))
        
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
            f"A* Performance: "
            f"Time = {elapsed_time:.4f} s, "
            f"Path Length = {path_length:.4f}, "
            f"Node Expansions = {self.node_expansions}"
        )

    def calculate_path_length(self, path_nodes):
        length = 0.0
        for i in range(1, len(path_nodes)):
            length += self.distance(path_nodes[i-1], path_nodes[i])
        return length

def main(args=None):
    rclpy.init(args=args)
    a_star_node = AStar()
    rclpy.spin(a_star_node)
    a_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()