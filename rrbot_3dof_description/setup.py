from setuptools import find_packages, setup

package_name = 'rrbot_3dof_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install URDF file
        ('share/' + package_name + '/urdf', ['urdf/rrbot_3dof.urdf.xacro']),
        # Install config file
        ('share/' + package_name + '/config', ['config/rrbot_controllers.yaml']),
        ('share/' + package_name + '/config', ['config/rrbot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evloev.sayfuddin',
    maintainer_email='evloev.sayfuddin@wb.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
