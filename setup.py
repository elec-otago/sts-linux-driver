# python setup.py develop
from setuptools import setup, find_packages

setup(name='OceanOptics',
      version='0.1',
      description='Driver for STS OceanOptics spectrometers',
      url='http://github.com/elec-otago/sts-linux-driver',
      author='Matt West, Tim Molteno',
      author_email='tim@elec.ac.nz',
      install_requires=['numpy'],
      license='Copyright',
      packages=['OceanOptics'],
    )