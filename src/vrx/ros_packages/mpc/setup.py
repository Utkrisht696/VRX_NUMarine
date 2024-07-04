# setup.py
from setuptools import setup

package_name = 'mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='rileychron111',
    maintainer_email='c3374355@uon.edu.au',
    description='Control node package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = mpc.control_node:main'
        ],
    },
)

