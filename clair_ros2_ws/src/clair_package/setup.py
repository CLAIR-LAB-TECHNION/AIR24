from setuptools import find_packages, setup
import glob


package_name = 'clair_package'


data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/ur5_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/ur_scratch.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/ur5e.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))
# add all protos:
data_files.append(('share/' + package_name + '/protos', glob.glob('protos/*.proto')))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuvalgos',
    maintainer_email='yuvalgos@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur5_driver = clair_package.ur5_driver:main'
        ],
    },
)
