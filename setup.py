from setuptools import find_packages, setup
import os
import glob


package_name = 'butler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'description'), glob.glob(os.path.join('description', '*.xacro'))),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keerthi',
    maintainer_email='keerthi003keerthi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = butler.control:main',
            'move = butler.move2location:main',
            'queue = butler.queue:main',
            'multi_order = butler.multi_order:main',
        ],
    },
)
