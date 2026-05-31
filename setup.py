from setuptools import find_packages, setup

package_name = 'r2drip2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='C-43PO',
    maintainer_email='C-43PO@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_to_cell = r2drip2.robot_mover:main', # create command "move to cell". 1) packet, 2) file, 3) function main
            'farm_manager = r2drip2.farm_manager:main',
            'test = r2drip2.base:main',
            'decision_system = r2drip2.decision_system:main',
        ],
    },
)
