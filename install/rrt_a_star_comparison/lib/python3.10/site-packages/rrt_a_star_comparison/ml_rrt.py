import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import random
import math
import torch
import os
from ament_index_python.packages import get_package_share_directory

class SamplingPolicy(torch.nn.Module):
    def __init__(self):
        super(SamplingPolicy, self).__init__()
        self.fc1 = torch.nn.Linear(4, 64)
        self.fc2 = torch.nn.Linear(64, 64)
        self.fc3 = torch.nn.Linear(64, 2)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x

class ML_RRT(Node):
    def __init__(self):
        super().__init__('ml_rrt_node')
        # 获取模型文件的完整路径
        model_path = os.path.join(
            get_package_share_directory('rrt_a_star_comparison'),
            'resource',
            'sampling_policy.pth'
        )

        # 检查模型文件是否存在
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found: {model_path}")
            raise FileNotFoundError(f"Model file not found: {model_path}")

        # 加载预训练的机器学习模型
        self.model = SamplingPolicy()
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()  # 设置为评估模式

        # 发布路径
        self.path_pub = self.create_publisher(Path, '/ml_rrt_path', 10)
        # 发布起点和终点的Marker
        self.marker_pub = self.create_publisher(Marker, '/ml_rrt_markers', 10)
        # 降低发布频率，每1秒发布一次路径
        self.timer = self.create_timer(1.0, self.plan)

        # 地图范围和目标点
        self.map_width = 100
        self.map_height = 100
        self.start = (10, 10)  # 起点
        self.goal = (90, 90)   # 终点
        self.step_size = 5.0   # 步长
        self.max_iterations = 10000  # 最大迭代次数
        self.goal_threshold = 5.0  # 目标点阈值

    def plan(self):
        nodes = [self.start]
        for i in range(self.max_iterations):
            # 使用机器学习模型预测采样方向
            random_point = self.predict_sampling_direction(nodes[-1])
            nearest_node = self.find_nearest_node(random_point, nodes)
            new_node = self.steer(nearest_node, random_point)

            if self.check_collision(nearest_node, new_node):
                nodes.append(new_node)
                self.get_logger().info(f"Iteration {i}: New node added at ({new_node[0]}, {new_node[1]})")
                if self.distance(new_node, self.goal) < self.goal_threshold:
                    self.publish_path(nodes)
                    self.publish_markers()
                    self.get_logger().info("ML-RRT Path Found and Published!")
                    return

        self.get_logger().info("ML-RRT Path Not Found!")

    def predict_sampling_direction(self, current_node):
        # 将当前节点和目标点输入模型，预测采样方向
        input_tensor = torch.tensor([current_node[0], current_node[1], self.goal[0], self.goal[1]], dtype=torch.float32)
        with torch.no_grad():
            output_tensor = self.model(input_tensor)
        predicted_point = (output_tensor[0].item(), output_tensor[1].item())
        self.get_logger().info(f"Predicted point: {predicted_point}")
        return predicted_point

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
        # 暂时禁用碰撞检测
        return True

    def distance(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def publish_path(self, nodes):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for node in nodes:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path.header.stamp
            pose.pose.position = Point(x=float(node[0]), y=float(node[1]), z=0.0)
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.get_logger().info(f"ML-RRT Path Published with {len(nodes)} nodes")

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

def main(args=None):
    rclpy.init(args=args)
    ml_rrt_node = None
    try:
        ml_rrt_node = ML_RRT()
        rclpy.spin(ml_rrt_node)
    except Exception as e:
        rclpy.logging.get_logger('ml_rrt_node').error(f"Error: {str(e)}")
    finally:
        if ml_rrt_node:
            ml_rrt_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()