import matplotlib.pyplot as plt
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
            a_star_values = self.a_star_data
            rrt_values = self.rrt_data

            x = range(len(labels))
            plt.bar(x, a_star_values, width=0.4, label='A*', color='b', align='center')
            plt.bar(x, rrt_values, width=0.4, label='RRT', color='r', align='edge')
            plt.xticks(x, labels)
            plt.xlabel('Metrics')
            plt.ylabel('Values')
            plt.title('A* vs RRT Performance Comparison')
            plt.legend()
            plt.show()

def main(args=None):
    rclpy.init(args=args)
    visualizer = PerformanceVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()