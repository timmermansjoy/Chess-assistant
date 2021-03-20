import setuptools

setuptools.setup(
    name="PXL chess",  # Replace with your own username
    version="0.0.1",
    author="Joy Timmermans",
    author_email="joy.timmermans@student.pxl.be",
    description="A chess package made for Research project",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    package_dir={"": "src"},
    packages=setuptools.find_packages(where="src"),
    python_requires=">=3.6",
)
