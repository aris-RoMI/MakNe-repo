from setuptools import find_packages, setup
import glob, os

package_name = 'makne_ui'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=["library", "makne_ui", "makne_service"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/ui/",  glob.glob("makne_ui/ui/*.ui")),
        ('share/' + package_name + "/data/",  glob.glob("makne_ui/data/room_103.pgm")),
        ('share/' + package_name + "/data/",  glob.glob("makne_ui/data/room_103.yaml")),
    ],
    install_requires=['setuptools', "library", "makne_service"],
    zip_safe=True,
    maintainer='joe',
    maintainer_email='dlwlgh0106@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "slack_ui = makne_ui.slack_test:main",
            "admin_ui = makne_ui.admin_ui:main",
            "robot_state_subscription = makne_service.robot_state_subscription_node:main",
        ],
    },
)
