from setuptools import find_packages, setup

package_name = 'limo_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='novalab0401',
    maintainer_email='novalab0401@users.noreply.github.com',
    description='Heavily-commented starter examples for students (cmd_vel publisher, /scan subscriber, obstacle-stop).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_publisher_example = limo_examples.cmd_vel_publisher_example:main',
            'scan_subscriber_example = limo_examples.scan_subscriber_example:main',
            'obstacle_stop_example = limo_examples.obstacle_stop_example:main',
        ],
    },
)
