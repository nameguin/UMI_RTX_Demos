from setuptools import find_packages
from setuptools import setup

setup(
    name='umi_rtx_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('umi_rtx_interfaces', 'umi_rtx_interfaces.*')),
)
