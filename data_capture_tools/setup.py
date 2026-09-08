from glob import glob
from setuptools import find_packages, setup

package_name = 'data_capture_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='kazuki.takahashi@akg.t.u-tokyo.ac.jp',
    description='Record rosbag2 sessions and export datasets for teleoperation experiments.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'data_capture_manager = data_capture_tools.data_capture_node:main',
            'bag_to_dataset = data_capture_tools.bag_converter:cli_main',
            'data_capture_manager_10 = data_capture_tools.data_capture_node_10times:main',
            'extract_batch_bag = data_capture_tools.extract_batch_bag:main',
            'extract_all = data_capture_tools.extract_all:run_as_node', # ここを追加
            'analyze_gripper_status = data_capture_tools.analyze_gripper_status:main',
        ],
    },
)
