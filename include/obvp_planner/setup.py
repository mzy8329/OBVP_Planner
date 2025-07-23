from setuptools import setup, Extension
import pybind11
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))

include_dirs = [
    current_dir,
    os.path.join(current_dir, "obvp_solver"),  
    pybind11.get_include(),
    pybind11.get_include(True)  
]

ext = Extension(
    "obvp_planner",
    sources=["obvp_wrapper.cpp"],
    include_dirs=include_dirs,
    language="c++",
    extra_compile_args=["-std=c++11", "-O3", "-Wall", "-Wextra", "-fPIC"],
    define_macros=[],
)

setup(
    name="obvp_planner",
    version="0.1.0",
    author="zongyuma",
    author_email="zongyuma@tencent.com",
    description="OBVP Planner with Python bindings",
    ext_modules=[ext],
    install_requires=["pybind11", "numpy"],
    setup_requires=["pybind11>=2.6.0"],
    python_requires=">=3.6",
    packages=["obvp_solver"],
    package_dir={"obvp_solver": "obvp_solver"},
    package_data={"obvp_solver": ["*.py"]},
)