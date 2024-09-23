from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'dtefm_middle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource', 'pntk'), glob('resource/pntk/*.py')),
        (os.path.join('share', package_name, 'resource', 'rltk'), glob('resource/rltk/*.py')),
        (os.path.join('share', package_name, 'resource', 'srtk'), glob('resource/srtk/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='879032479@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'identity = dtefm_middle.dtefm_identity:main',
            'intension_generator = dtefm_middle.dtefm_intension_generator:main',
            'sr_robot_ik_server = dtefm_middle.dtefm_sr_IK_server:main',
        ],
    },
)
