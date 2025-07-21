from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tutorial_ardusub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.yaml')),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='eugene',
    maintainer_email='eugenewang920@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "bluerov2_sensors = tutorial_ardusub.bluerov2_sensors:main",
            "ArmClient = tutorial_ardusub.ArmClient:main",
            "forward_move = tutorial_ardusub.forward_move:main",
            "Dance = tutorial_ardusub.Dance:main",
            "depthhold = tutorial_ardusub.depthhold:main",
            "headingcontrol = tutorial_ardusub.headingcontrol:main",
            "calcdepth = tutorial_ardusub.calcdepth:main",
            "controlMsgPub = tutorial_ardusub.controlMsgPub:main",
        ],
    },
)
