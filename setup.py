from setuptools import setup, find_packages
import codecs

import os

def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), 'r') as fp:
        return fp.read()

def get_version(rel_path):
    for line in read(rel_path).splitlines():
        if line.startswith('__version__'):
            delim = '"' if '"' in line else "'"
            return line.split(delim)[1]
    else:
        raise RuntimeError("Unable to find version string.")

setup(
    name="fanuc_state_ros2",
    install_requires=["colorlog"
            ],
    version=get_version("fanuc_state_ros2/__init__.py"),
    packages=find_packages(),
    include_package_data=True,
)