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
            'roboy_simulation = bulletroboy.roboy_simulation:main',
            'cage_simulation = bulletroboy.cage_simulation:main',
            'test_publisher = bulletroboy.test_publisher:main'
        ],
    },
)
