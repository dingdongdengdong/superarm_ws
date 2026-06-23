from setuptools import find_packages
from setuptools import setup

setup(
    name='nlink_message',
    version='0.0.0',
    packages=find_packages(
        include=('nlink_message', 'nlink_message.*')),
)
