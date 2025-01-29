from setuptools import find_packages, setup

package_name = 'wcr_trajectory_generator'

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
    maintainer='branimir',
    maintainer_email='branimir.caran@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_wave_node = wcr_trajectory_generator.square_wave_node:main',
            'linear_trajectory_node = wcr_trajectory_generator.linear_node:main',
        ],
    },
)
