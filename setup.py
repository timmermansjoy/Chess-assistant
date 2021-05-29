#!/usr/bin/env python3
# https://godatadriven.com/blog/a-practical-guide-to-using-setup
import os
import setuptools

directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setuptools.setup(name='PXL_Chess',
                 version='0.0.1',
                 description='A chess package made for Research project.',
                 author='Joy Timmermans, Nikki Bruls, Stijn Jacobs, Karel Sajdak, Muhammed Senturk',
                 license='MIT',
                 long_description=long_description,
                 long_description_content_type='text/markdown',
                 package_dir={"": "src/abstraction/scripts"},
                 packages=setuptools.find_packages(where="src/abstraction/scripts"),
                 platforms=['any'],
                 classifiers=[
                     "Programming Language :: Python :: 3",
                     "License :: OSI Approved :: MIT License",
                     "Operating System :: OS Independent",
                 ],
                 install_requires=['numpy'],
                 python_requires='>=3.6',
                 extras_require={
                     'development': ["pytest", "pytest-xdist", "pytest-cov", "autopep8", "cython"],
                     'docker': ["PyQt5", "pyyaml", "rospkg", "opencv-python"]
                 },
                 include_package_data=True)
