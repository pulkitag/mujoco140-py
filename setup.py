#!/usr/bin/env python

from setuptools import setup

setup(
    name='mujoco140-py',
    version='0.5.7',
    description='Python wrapper for Mujoco',
    author='OpenAI',
    packages=['mujoco140_py'],
    install_requires=[
        'PyOpenGL>=3.1.0',
        'numpy>=1.10.4',
        'six',
    ],
    tests_requires=[
        'nose2'
    ]
)
