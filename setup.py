from setuptools import setup, find_packages
import sys

if sys.version_info.major != 3:
    print("This Python is only compatible with Python 3, but you are running "
          "Python {}. The installation will likely fail.".format(sys.version_info.major))


setup(name='FOR',
      packages=[package for package in find_packages()
                if package.startswith('FOR')],
      install_requires=[
          'numpy',
          'matplotlib',
          'sympy',
          'pybullet'
      ],
      description="Kinematical and Dynamical Analysis of Custom Built  Robotic Arm",
      author="Aadhithya Iyer,  Ajay Jagannathan, Yosef, Sakshi Yadav",
      url='https://github.com/aadhithya14/FOR_project.git',
      author_email="ai2324@nyu.edu",
      version="0.1.4")
