import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/crx/Vscode/ros2_nav_ws/install/rrt_a_star_comparison'
