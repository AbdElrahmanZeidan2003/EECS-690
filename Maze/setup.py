from setuptools import find_packages, setup

package_name = 'class_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav.py']),
        ('share/' + package_name + '/config', ['config/slam_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='11306260+Liangfuyuan@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = class_pkg.test:main',
            'spin = class_pkg.spin:main',
            'talker = class_pkg.talker_node:main',
            'listener = class_pkg.listener_node:main',
            'beep = class_pkg.beep:main',
            'nav = class_pkg.nav:main',
            'demo = class_pkg.demo_derby:main',
            'localization = class_pkg.localization:main',
            'safety = class_pkg.safety:main',
            'orange = class_pkg.orange:main',
            'maze_explorer = class_pkg.maze_explorer:main',
        ],
    },
)
