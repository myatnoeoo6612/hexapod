from setuptools import setup

package_name = 'hexapod_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/hexapod.urdf']),
        ('share/' + package_name + '/world', ['world/default.sdf']),
        ('share/' + package_name + '/config', ['config/controllers.yaml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='myat',
    maintainer_email='myatnoeoo772985799@gmail.com',
    description='Hexapod robot description package',
    license='TODO',
    entry_points={
        'console_scripts': [],
    },
)
