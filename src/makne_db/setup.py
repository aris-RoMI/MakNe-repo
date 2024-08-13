from setuptools import find_packages, setup
import glob

package_name = 'makne_db'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=["makne_db"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/db/",  glob.glob("makne_db/db/makne_db")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joe',
    maintainer_email='dlwlgh0106@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "db_manager = makne_db.db_manager:main"
        ],
    },
)
