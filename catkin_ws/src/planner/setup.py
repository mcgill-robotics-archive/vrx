#!/usr/bin/env python

"""MR VRX Planner - Python Setup"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=['planner'], package_dir={'': 'src'})

setup(**d)
