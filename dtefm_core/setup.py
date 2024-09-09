from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'dtefm_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource', 'initial_file'), glob('resource/initial_file/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'eap_command_publisher = dtefm_core.dtefm_command_publisher:main',
            'eap_command_decoder = dtefm_core.dtefm_command_decoder:main',
            'eap_command_gate = dtefm_core.dtefm_command_gate:main',
            'eap_command_server = dtefm_core.dtefm_command_server:main',
            'identity_initializer = dtefm_core.dtefm_identity_initializer:main',
            # 'sr_command_sender = dtefm_core.dtefm_sr_command_sender:main',
            'test_server = dtefm_core.test_server:main',
        ],
    },
)
