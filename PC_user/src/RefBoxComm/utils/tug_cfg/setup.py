#!/usr/bin/env python
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tug_cfg'],
    package_dir={'': 'python_src'}
)

setup(**d)
"""


from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tug_cfg'],
    package_dir={'': 'python_src'}
)

setup(**d)
