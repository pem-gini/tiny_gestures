from setuptools import setup
from glob import glob
import os

package_name = 'tiny_gestures'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name, 
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	    ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name + '/params/', glob('params/*.yml')),
        ('share/' + package_name + '/params/', glob('params/*.yaml')),
        ('share/' + package_name + '/params/', glob('params/*.json')),
        ('share/' + package_name + '/params/', glob('params/*.rviz')),
        ### package relevant source files as well, so that we can import them locally
        # (os.path.join('lib', package_name, 'utils'), glob(os.path.join(package_name, 'utils', '*.py'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.blob'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Nick Fiege',
    author_email='n.fiege@pem.rwth-aachen.de',
    maintainer='Nick Fiege',
    maintainer_email='n.fiege@pem.rwth-aachen.de',
    keywords=['ROS'],
    classifiers=["DNN", "3DCDC"],
    description='Gesture Recognition Node',
    license='PEM',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_detection_inference_on_oakd ='
            'tiny_gestures.Main:main',
        ],
    },
)
