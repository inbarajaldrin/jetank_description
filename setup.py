from setuptools import setup
import os
from glob import glob

package_name = 'jetank_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Jetank robot simulation and launch setup',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth = scripts.depth:main',
        ],
    },
)
