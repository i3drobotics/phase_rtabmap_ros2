from setuptools import setup

package_name = 'phase_rtabmap_foxy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='i3dr',
    maintainer_email='kinyip@i3drobotics.com',
    description='Phase image publish',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'phase_camera = phase_rtabmap_foxy.phase_pub:main'
        ],
    },
)
