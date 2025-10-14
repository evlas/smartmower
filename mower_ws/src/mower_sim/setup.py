from setuptools import setup

package_name = 'mower_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gz.launch.py', 'launch/rsp.launch.py']),
        ('share/' + package_name + '/config', ['config/gz_bridge.yaml']),
        ('share/' + package_name + '/worlds', ['worlds/empty.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vito',
    maintainer_email='vito@example.com',
    description='Launch e configurazioni per simulazione gz del robot tosaerba',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    },
)
