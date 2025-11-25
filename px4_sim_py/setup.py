from setuptools import find_packages, setup

package_name = 'px4_sim_py'

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
    maintainer='lim',
    maintainer_email='lim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        "test": ["pytest"]
    },
    entry_points={
        'console_scripts': [
          "px4_instance_manager = px4_sim_py.px4_instance_manager:main",
          "px4_status_aggregator = px4_sim_py.px4_status_aggregator:main",
          "gcs_mission_gateway = px4_sim_py.gcs_mission_gateway:main"
        ],
    },
)
