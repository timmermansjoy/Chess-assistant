#!/usr/bin/env python3
# https://godatadriven.com/blog/a-practical-guide-to-using-setup-py/
import os
from setuptools import setup

directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(name='PXL chess',
      version='0.0.1',
      description='A chess package made for Research project.',
      author='Joy Timmermans',
      license='MIT',
      long_description=long_description,
      long_description_content_type='text/markdown',
      package_dir={"": "src"},
      #   packages=setup.find_packages(where="src"),
      platforms=['any'],
      classifiers=[
          "Programming Language :: Python :: 3",
          "License :: OSI Approved :: MIT License",
          "Operating System :: OS Independent",
      ],
      install_requires=['numpy'],
      python_requires='>=3.6',
      extras_require={
          'development': ["pytest", "pytest-xdist", "autopep8"]
      },
      include_package_data=True)
