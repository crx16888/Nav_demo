import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PerformanceVisualizer(Node):
    def __init__(self):
        super().__init__('performance_visualizer')
        self.a_star_data = None
        self.rrt_data = None
        self.a_star_sub = self.create_subscription(Float32MultiArray, '/a_star_performance', self.a_star_callback, 10)
        self.rrt_sub = self.create_subscription(Float32MultiArray, '/rrt_performance', self.rrt_callback, 10)

    def a_star_callback(self, msg):
        self.a_star_data = msg.data
        self.compare_performance()

    def rrt_callback(self, msg):
        self.rrt_data = msg.data
        self.compare_performance()

    def compare_performance(self):
        if self.a_star_data is not None and self.rrt_data is not None:
            labels = ['Time (s)', 'Path Length', 'Node Expansions']
            a_star_values = np.array(self.a_star_data)
            rrt_values = np.array(self.rrt_data)

            x = np.arange(len(labels))  # X 轴的刻度位置
            width = 0.3  # 柱子宽度

            fig, ax = plt.subplots(figsize=(10, 6))  # 调整图表大小
            bars1 = ax.bar(x - width / 2, a_star_values, width, label='A*', color='#4C72B0')  # 蓝色
            bars2 = ax.bar(x + width / 2, rrt_values, width, label='RRT', color='#C44E52')  # 红色

            # 添加数据标签
            for bar in bars1:
                ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f'{bar.get_height():.2f}',
                        ha='center', va='bottom', fontsize=10, color='black')
            for bar in bars2:
                ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f'{bar.get_height():.2f}',
                        ha='center', va='bottom', fontsize=10, color='black')

            ax.set_xticks(x)
            ax.set_xticklabels(labels, fontsize=12)
            ax.set_xlabel('Metrics', fontsize=14)
            ax.set_ylabel('Values', fontsize=14)
            ax.set_title('A* vs RRT Performance Comparison', fontsize=16)
            ax.legend(fontsize=12)
            ax.grid(axis='y', linestyle='--', alpha=0.7)  # 添加横向网格线

            plt.show()

def main(args=None):
    rclpy.init(args=args)
    visualizer = PerformanceVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
