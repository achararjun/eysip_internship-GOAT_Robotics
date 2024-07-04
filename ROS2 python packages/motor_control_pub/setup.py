from setuptools import find_packages, setup

package_name = 'motor_control_pub'

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
    maintainer='arjun22',
    maintainer_email='arjun.achar03@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = motor_control_pub.publisher:main',
            'basic_publisher = motor_control_pub.basic_publisher:main'
        ],
    },
)
