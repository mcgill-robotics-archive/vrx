#!/usr/bin/env python3
"""MR VRX Control - Python Setup"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['control'],
    package_dir={'': 'src'}
)

setup(**d)
