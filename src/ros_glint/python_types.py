from enum import IntEnum


class PythonUsage(IntEnum):
    NONE = 0
    PURE_PYTHON = 1
    MIXED_CMAKE_PYTHON = 2


def get_python_usage(package):
    # If there is no Python library, return NONE
    if not package.get_source_by_tags('pylib') and package.build_type != 'ament_python':
        return PythonUsage.NONE

    # If build type is ament_cmake or ament_cmake_python
    if 'ament_cmake' in package.build_type:
        return PythonUsage.MIXED_CMAKE_PYTHON

    return PythonUsage.PURE_PYTHON
