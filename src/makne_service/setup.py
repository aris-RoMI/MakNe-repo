from setuptools import find_packages, setup
import os, glob

package_name = 'makne_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=["makne_service"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/config/",  glob.glob("makne_service/config/slack_config.json")),
    ],
    install_requires=['setuptools', "flask"],
    zip_safe=True,
    maintainer='joe',
    maintainer_email='dlwlgh0106@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "slack_message_handler = makne_service.slack_message_handler:main"
        ],
    },
)
