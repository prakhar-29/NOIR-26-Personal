from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add templates directory
        (os.path.join('share', package_name, 'templates'), 
            glob('py_pubsub/templates/*')),
        # If you have subdirectories in templates, use:
        # (os.path.join('share', package_name, 'templates'), 
        #     glob('py_pubsub/templates/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover',
    maintainer_email='rover@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'esp32rst = py_pubsub.esp32_rst:main' , 
            'esp32rst1 = py_pubsub.esp32_rst1:main' , 
            'esp32rst2 = py_pubsub.esp32_rst2:main' ,
            'camfeed_idmo = py_pubsub.camfeed_idmo:main',
            'camfeed_auto=py_pubsub.camfeed_auto:main',
            'camfeed_recon = py_pubsub.camfeed_recon:main',
            'camfeed_abex = py_pubsub.camfeed_abex:main',
            'esp32rst3 = py_pubsub.esp32_rst3:main' , 
            'esp32rst4 = py_pubsub.esp32_rst4:main' ,

        ],
    },
)
