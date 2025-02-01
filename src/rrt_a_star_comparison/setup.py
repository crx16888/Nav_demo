from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rrt_a_star_comparison'

# 获取资源文件路径
model_file = 'resource/sampling_policy.pth'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# 如果模型文件存在，将其添加到数据文件中
if os.path.exists(model_file):
    data_files.append(('share/' + package_name + '/resource', [model_file]))
else:
    print(f"Warning: Model file '{model_file}' not found. Please ensure it exists in the 'resource' directory.")

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crx',
    maintainer_email='crx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_node = rrt_a_star_comparison.rrt:main',
            'a_star_node = rrt_a_star_comparison.a_star:main',
            'ml_rrt_node = rrt_a_star_comparison.ml_rrt:main',  # 添加 ml_rrt_node 的入口点
            'performance_visualizer = rrt_a_star_comparison.performance_visualizer:main',  # 添加 PerformanceVisualizer 的入口点
        ],
    },
)