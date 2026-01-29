from setuptools import find_packages, setup

package_name = 'dsr_rokey2'

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
    maintainer='wook',
    maintainer_email='wook@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_basic = dsr_rokey2.move_basic:main',
            'mini_jog = dsr_rokey2.mini_jog:main',
            'move_periodic = dsr_rokey2.move_periodic:main',
            'grip_test = dsr_rokey2.grip_test:main',
            'force_test = dsr_rokey2.force_test:main',
            'jeyoon_test = dsr_rokey2.jeyoon_test:main',
            'get_current_pos = dsr_rokey2.get_current_pos:main',
            'move_spiral = dsr_rokey2.move_spiral:main',
            'firebase_publisher = dsr_rokey2.firebase_publisher:main',
            'move_spiral_listener = dsr_rokey2.move_spiral_listener:main',
            'pick_drip_spiral = dsr_rokey2.pick_drip_spiral:main',
            'pick_filter = dsr_rokey2.pick_filer:main',
            'pick_drip_spiral_listener = dsr_rokey2.pick_drip_spiral_listener:main',
            'pick_filter_listener = dsr_rokey2.pick_filter_listener:main',
            'total_update_listener = dsr_rokey2.total_update_listener:main',
            'total_final_listener = dsr_rokey2.total_final_listener:main',
            'king_listener = dsr_rokey2.king_listener:main',
            'stop_test_listener = dsr_rokey2.stop_test_listener:main',
            'real_king_listener = dsr_rokey2.real_king_listener:main',
            'god_listener = dsr_rokey2.god_listener:main',
            'real_god_listener = dsr_rokey2.real_god_listener:main',
        
        ],
    },
)
