from setuptools import setup

package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['constants'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henrik',
    maintainer_email='henrik@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bottle_recog_node = camera_pkg.bottle_recog_node:main',
            'pouring_accuracy_node = camera_pkg.pouring_accuracy_node:main',
            'test_node = camera_pkg.test_node:main', 
        ],
    },
)
