from setuptools import find_packages, setup

package_name = 'robot_mobile_evaluation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='essomanam',
    maintainer_email='manganmanasamuel@gmail.com',
    description="Outils d'expérimentation et d'évaluation du projet AS52.",
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'reference_trajectory = robot_mobile_evaluation.reference_trajectory:main',
            'sensor_check = robot_mobile_evaluation.sensor_check:main',
            'analyze_dataset = robot_mobile_evaluation.analyze_dataset:main',
            'trajectory_logger = robot_mobile_evaluation.trajectory_logger:main',
            'evaluate_slam = robot_mobile_evaluation.evaluate_slam:main',
        ],
    },
)
