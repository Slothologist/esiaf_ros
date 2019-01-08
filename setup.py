from setuptools import find_packages
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name='esiaf_ros',
    version='0.0.1',
    description='esiaf (Enhanced Sound Integration And Framework, pronounced easy a f) is a library to exchange audio signals between several applications over ros',
    url='---none---',
    author='rfeldhans',
    author_email='rfeldhans@techfak.uni-bielefeld.de',
    license='---none---',
    install_requires=[
    ],
    packages=find_packages(),
    package_dir={'orchestrator':'esiaf_ros/src/nodes/orchestrator'}
)

setup(**setup_args)