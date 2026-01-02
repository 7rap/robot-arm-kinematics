from setuptools import find_packages, setup

package_name = 'rrbot_3dof_inverse_kinematics'

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
    maintainer='evloev.sayfuddin',
    maintainer_email='evloev.sayfuddin@wb.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rrbot_3dof_inverse_kinematics = rrbot_3dof_inverse_kinematics.rrbot_3dof_inverse_kinematics:main',
        ],
    },
)
