import collections
from enum import IntEnum
import fnmatch
import os.path

from stylish_cmake_parser import Command

from ..core import clean_ros
from ..cmake_ordering import insert_in_order
from .cmake import check_complex_section, get_multiword_section


class InstallType(IntEnum):
    EXECUTABLE = 1
    LIBRARY = 2
    HEADERS = 3
    SHARE = 4
    PYTHON = 5


# The destination map is dictionary from destinations to CMake section names
InstallConfig = collections.namedtuple('InstallConfig', ['section_name', 'destination_map'])


INSTALL_CONFIGS = {
    # ROS 1
    1: {
        InstallType.EXECUTABLE: InstallConfig('TARGETS',
                                              {'${CATKIN_PACKAGE_BIN_DESTINATION}': ['RUNTIME DESTINATION']}),
        InstallType.LIBRARY: InstallConfig('TARGETS',
                                           {'${CATKIN_PACKAGE_LIB_DESTINATION}': ['ARCHIVE DESTINATION',
                                                                                  'LIBRARY DESTINATION'],
                                            '${CATKIN_GLOBAL_BIN_DESTINATION}': ['RUNTIME DESTINATION']}),
        InstallType.HEADERS: InstallConfig('FILES', {'${CATKIN_PACKAGE_INCLUDE_DESTINATION}': ['DESTINATION']}),
        InstallType.SHARE: InstallConfig('FILES', {'${CATKIN_PACKAGE_SHARE_DESTINATION}': ['DESTINATION']}),
    },
    2: {
        InstallType.EXECUTABLE: InstallConfig('TARGETS', {'lib/${PROJECT_NAME}': ['DESTINATION']}),
        InstallType.LIBRARY: InstallConfig('TARGETS',
                                           {'lib': ['ARCHIVE DESTINATION', 'LIBRARY DESTINATION'],
                                            'bin': ['RUNTIME DESTINATION'],
                                            }),
        InstallType.HEADERS: InstallConfig('FILES', {'include': ['DESTINATION']}),
        InstallType.SHARE: InstallConfig('FILES', {'share/${PROJECT_NAME}': ['DESTINATION']}),
        InstallType.PYTHON: InstallConfig('PROGRAMS', {'lib/${PROJECT_NAME}': ['DESTINATION']}),
    }
}


def get_install_type(destination, section_name=None):
    """For a given catkin destination, return the matching install type."""
    for config_map in INSTALL_CONFIGS.values():
        for name, (kw, destination_map) in config_map.items():
            if destination not in destination_map:
                continue
            if kw == section_name or (kw == 'FILES' and section_name == 'DIRECTORY'):
                return name


def directory_matches(cmd, subfolder=''):
    the_dir = cmd.get_section('DIRECTORY')
    return the_dir and subfolder and subfolder in the_dir.values


def get_install_types(cmd, subfolder=''):
    """For a given CMake command, determine the install type(s) that this command uses.

    If there is a non-empty subfolder, we only return the install types if the command
    installs into the catkin_destination with the given subfolder
    """
    types = set()
    check_subfolder = not directory_matches(cmd, subfolder)
    token = cmd.get_tokens(True)[0]

    for section in cmd.get_sections('DESTINATION'):
        the_folder = section.values[0]
        if the_folder.endswith('/'):
            the_folder = the_folder[:-1]
        if len(subfolder) > 0 and check_subfolder:
            if subfolder not in the_folder:
                continue
            the_folder = the_folder.replace('/' + subfolder, '')
        type_ = get_install_type(the_folder, token)
        if type_:
            types.add(type_)
    return types


def install_sections(cmd, destination_map, subfolder=''):
    """Ensure that the command has all the appropriate CMake sections with the matching catkin destinations.

    If the subfolder is defined, the subfolder is appended to the catkin destination.
    """
    for destination, section_names in destination_map.items():
        for section_name in section_names:
            if len(subfolder) > 0:
                destination = os.path.join(destination, subfolder)
            elif destination == '${CATKIN_PACKAGE_SHARE_DESTINATION}':
                # ensure unslashed version is not present
                section = cmd.get_section(section_name)
                if section and destination in section.values:
                    section.values.remove(destination)
                destination += '/'
            check_complex_section(cmd, section_name, destination)


def remove_install_section(cmd, destination_map):
    empty_sections_to_remove = {}
    for destination, section_names in destination_map.items():
        for section_name in section_names:
            parts = section_name.split()
            if len(parts) == 2:
                empty_sections_to_remove[parts[0]] = destination
    sections = cmd.get_real_sections()
    to_remove = []
    for i, section in enumerate(sections):
        if section.name not in empty_sections_to_remove or len(section.values) != 0:
            continue
        next_section = sections[i + 1]
        dest = empty_sections_to_remove[section.name]
        if next_section.name == 'DESTINATION' and len(next_section.values) == 1 and next_section.values[0] == dest:
            to_remove.append(section)
            to_remove.append(next_section)
    if len(to_remove) > 0:
        for section in to_remove:
            cmd.sections.remove(section)
        cmd.changed = True


def get_commands_by_type(cmake, name, subfolder=''):
    matches = []
    for cmd in cmake.content_map['install']:
        if name in get_install_types(cmd, subfolder):
            matches.append(cmd)
    return matches


def matches_patterns(item, patterns):
    for pattern in patterns:
        if pattern[0] == pattern[-1] and pattern[0] == '"':
            pattern = pattern[1:-1]
        if fnmatch.fnmatch(item, pattern):
            return True


def install_section_check(cmake, items, install_type, ros_version, directory=False, subfolder=''):
    install_config = INSTALL_CONFIGS[ros_version][install_type]

    if directory and install_config.section_name == 'FILES':
        section_name = 'DIRECTORY'
    else:
        section_name = install_config.section_name

    cmds = get_commands_by_type(cmake, install_type, subfolder)
    if len(items) == 0:
        for cmd in cmds:
            if len(get_install_types(cmd)) == 1:
                cmake.remove_command(cmd)
            else:
                remove_install_section(cmd, install_config.destination_map)
        return True

    cmd = None
    items = [os.path.join(subfolder, item) for item in items]
    for cmd in cmds:
        if directory_matches(cmd, subfolder):
            return False
        install_sections(cmd, install_config.destination_map, subfolder)
        section = cmd.get_section(section_name)
        if not section:
            if section_name != 'FILES':
                continue
            section = cmd.get_section('DIRECTORY')
            if not section:
                continue
            pattern = get_multiword_section(cmd, ['FILES_MATCHING', 'PATTERN'])
            nonmatching_items = []
            for item in items:
                if pattern and not matches_patterns(item, pattern.values):
                    nonmatching_items.append(item)
            items = nonmatching_items
        else:
            # We match the section
            section.values = [value for value in section.values if value in items]
            items = [item for item in items if item not in section.values]

    if len(items) == 0:
        return False

    if cmd is None:
        cmd = Command('install')
        cmd.add_section(section_name, items)
        insert_in_order(cmake, cmd)
        install_sections(cmd, install_config.destination_map, subfolder)
    elif section:
        # section = cmd.get_section(section_name)
        section.values += items
        cmd.changed = True

    if subfolder and len(str(cmd)) > 120 and section_name == 'FILES':
        section = cmd.get_section(section_name)
        section.name = 'DIRECTORY'
        section.values = [subfolder + '/']
        cmd.changed = True

    return True


@clean_ros
def update_cplusplus_installs(package):
    if not package.cmake:
        return
    cmake = package.cmake.contents
    changed = install_section_check(cmake, package.cmake.get_executables(), InstallType.EXECUTABLE, package.ros_version)
    changed |= install_section_check(cmake, package.cmake.get_libraries(), InstallType.LIBRARY, package.ros_version)

    has_header_files = any('header' in source_file.tags for source_file in package.source_code)
    if package.name and has_header_files:
        if package.ros_version == 1:
            changed |= install_section_check(
                cmake, ['include/${PROJECT_NAME}/'], InstallType.HEADERS, package.ros_version, directory=True)
        else:
            if not cmake.content_map['ament_export_include_directories']:
                cmd = Command('ament_export_include_directories')
                cmd.add_section('', ['include'])
                insert_in_order(cmake, cmd)
            changed |= install_section_check(
                cmake, ['include/'], InstallType.HEADERS, package.ros_version, directory=True)

    package.cmake.changed |= changed
