from setuptools import find_packages, setup

package_name = 'block_detection'

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
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Robot block edge and height detection using inverted Mid-360',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 这里的格式是：终端命令名 = 包名.文件名:入口函数
            'block_detector = block_detection.block_detector:main'
        ],
    },
)