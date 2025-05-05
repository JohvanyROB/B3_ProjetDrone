from setuptools import setup

package_name = 'tello_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johvany Gustave',
    maintainer_email='johvany.gustave@gmail.com',
    description='This package contains the navigation strategy of the UAV',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "nav.py = tello_nav.nav:main"
        ],
    },
)
