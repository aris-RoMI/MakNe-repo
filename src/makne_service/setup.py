from setuptools import find_packages, setup
import os, glob

package_name = 'makne_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=["makne_service", "makne_db", "library", "makne_ui"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/config/",  glob.glob("makne_service/config/slack_config.json")),
    ],
    install_requires=['setuptools', "flask", "makne_db", "library", "makne_ui"],
    zip_safe=True,
    maintainer='joe',
    maintainer_email='dlwlgh0106@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "slack_message_handler = makne_service.slack_message_handler:main",
            "checkpoint_calculator = makne_service.checkpoint_calculator:main",
            "robot_manager = makne_service.robot_manager:main",
            "send_goal_client = makne_service.send_goal_client:main",
            "map_editor = makne_service.map_editor:main",
            "dummy_robot_manager = makne_service.dummy_robot_manager:main",
            "dummy_action_server = makne_service.dummy_action_server:main",
            "dummy_action_client = makne_service.dummy_action_client:main",
        ],
    },
)
