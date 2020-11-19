from setuptools import setup

package_name = 'bulletroboy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboy',
    maintainer_email='roboy@todo.todo',
    description='The bulletroboy package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roboy_sim      = bulletroboy.nodes.roboy_node:main',
            'cage_sim       = bulletroboy.nodes.cage_node:main',
            'state_mapper   = bulletroboy.nodes.state_mapper_node:main',
            'operator       = bulletroboy.nodes.operator_node:main',
            'exoforce       = bulletroboy.nodes.exoforce_node:main',
            'test_publisher = bulletroboy.tests.test_publisher:main',
            'test_client    = bulletroboy.tests.test_client:main'
        ],
    },
)
