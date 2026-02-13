from setuptools import setup, find_packages

setup(
    name='EVRP-Module',
    version='0.1.0',
    author='clorenz0',
    author_email='youremail@example.com',
    description='A module for solving the EVRP problem.',
    packages=find_packages(),
    install_requires=[
        # List your dependencies here
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)