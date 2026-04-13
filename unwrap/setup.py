from setuptools import find_packages, setup

package_name = 'pipe_vision'

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
    maintainer='koceti',
    maintainer_email='koceti@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipe_unroller_node = pipe_vision.pipe_unroller_node:main',
            'inspect_tf = pipe_vision.inspect_tf:main'
        ],
    },
)
