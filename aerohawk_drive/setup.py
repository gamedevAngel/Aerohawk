from setuptools import setup

package_name = 'aerohawk_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AeroHawk Team',
    maintainer_email='todo@example.com',
    description='AeroHawk drive: /cmd_vel to steering servo and IBT-2 motor.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drive_node = aerohawk_drive.drive_node:main',
            'teleop_keyboard = aerohawk_drive.teleop_keyboard:main',  # MANUAL MODE ONLY - delete with the file
        ],
    },
)
