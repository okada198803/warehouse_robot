from glob import glob

from setuptools import setup

package_name = 'ai_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/params', glob('params/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pai-final',
    maintainer_email='noreply@example.com',
    description='LLM / gesture driven navigation on Gazebo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plan_executor = ai_nav.plan_executor:main',
            'obstacle = ai_nav.obstacle:main',
            'cargo_sim = ai_nav.cargo_sim:main',
        ],
    },
)
