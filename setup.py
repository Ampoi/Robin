from setuptools import find_packages, setup

package_name = 'robin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pylot',
    maintainer_email='johndoe@example.com',
    description='Mobile Robot Controller for PyLoT Robotics',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = robin.client:main',
            'video_publisher = robin.video_publisher:main'
        ],
    },
)
