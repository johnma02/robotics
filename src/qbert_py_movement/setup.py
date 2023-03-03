from setuptools import setup

package_name = 'qbert_py_movement'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='johnma',
    maintainer_email='ma.jonathan02@gmail.com',
    description='CISC367: Homework 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = qbert_py_movement.publisher_member_function:main',
            'listener = qbert_py_movement.subscriber_member_function:main',
            'wanderer = qbert_py_movement.wanderer:main',
        ],
    },
)
