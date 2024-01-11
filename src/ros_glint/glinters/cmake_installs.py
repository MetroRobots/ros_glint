import collections
from enum import IntEnum
import fnmatch
import os.path

from stylish_cmake_parser import Command

from ros_introspect.components.setup_py import create_setup_py

from ..core import glinter
from ..cmake_ordering import insert_in_order
from ..python_types import get_python_usage, PythonUsage
from .cmake import check_complex_section, section_check, get_multiword_section, install_cmake_dependencies


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


def get_destinations(cmd):
    for section in cmd.get_real_sections():
        if isinstance(section.name, str) and section.name.endswith('DESTINATION'):
            destination = section.values[0]
            if '}' in destination:
                ci = destination.rindex('}') + 1
                root = destination[:ci]
                subfolder = destination[ci:]
                yield root, subfolder
            else:
                yield destination, ''


def get_install_types(cmd, subfolder=''):
    """For a given CMake command, determine the install type(s) that this command uses.

    If there is a non-empty subfolder, we only return the install types if the command
    installs into the catkin_destination with the given subfolder
    """
    types = set()
    check_subfolder = subfolder is not None and not directory_matches(cmd, subfolder)
    token = cmd.get_tokens(True)[0]

    for root_destination, cmd_subfolder in get_destinations(cmd):
        if check_subfolder:
            cmd_subfolder = cmd_subfolder.lstrip('/')
            if subfolder != cmd_subfolder:
                continue
        type_ = get_install_type(root_destination, token)
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
                    section.mark_changed()
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
        cmd.mark_changed()


def get_commands_by_type(cmake, install_type, subfolder=''):
    matches = []
    for cmd in cmake.content_map['install']:
        if install_type in get_install_types(cmd, subfolder):
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
            if len(get_install_types(cmd, subfolder)) == 1:
                cmake.remove_command(cmd)
            else:
                remove_install_section(cmd, install_config.destination_map)

    cmd = None
    items = [os.path.join(subfolder, item) for item in items]
    for cmd in cmds:
        if directory_matches(cmd, subfolder):
            return
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
            if set(section.values) != set(items):
                section.values = [value for value in section.values if value in items]
                section.mark_changed()
            items = [item for item in items if item not in section.values]

    if len(items) == 0:
        return

    if cmd is None:
        cmd = Command('install', parent=cmake)
        cmd.add_section(section_name, items)
        insert_in_order(cmake, cmd)
        install_sections(cmd, install_config.destination_map, subfolder)
    elif section:
        # section = cmd.get_section(section_name)
        section.add_values(items, alpha_order=False)

    if subfolder and len(str(cmd)) > 120 and section_name == 'FILES':
        section = cmd.get_section(section_name)
        section.name = 'DIRECTORY'
        section.values = [subfolder + '/']
        section.mark_changed()


@glinter
def update_cplusplus_installs(package):
    if not package.cmake:
        return
    cmake = package.cmake
    install_section_check(cmake, package.cmake.get_executables(), InstallType.EXECUTABLE, package.ros_version)
    install_section_check(cmake, package.cmake.get_libraries(), InstallType.LIBRARY, package.ros_version)

    has_header_files = any(package.get_source_by_tags('header'))
    if package.name and has_header_files:
        if package.ros_version == 1:
            install_section_check(cmake, ['include/${PROJECT_NAME}/'], InstallType.HEADERS, package.ros_version,
                                  directory=True)
        else:
            if not cmake.content_map['ament_export_include_directories']:
                cmd = Command('ament_export_include_directories', parent=cmake)
                cmd.add_section('', ['include'])
                insert_in_order(cmake, cmd)
            install_section_check(cmake, ['include/'], InstallType.HEADERS, package.ros_version, directory=True)


@glinter
def export_cplusplus_libraries(package):
    if package.ros_version == 1:
        return
    libraries = package.cmake.get_libraries()
    if not libraries:
        return

    export_targets = []
    cmds = get_commands_by_type(package.cmake, InstallType.LIBRARY)

    for library in libraries:
        exporting_cmd = None
        lib_cmds = []

        my_export_name = None
        for cmd in cmds:
            targets = cmd.get_section('TARGETS')
            if not targets or library not in targets.values:
                continue
            lib_cmds.append(cmd)
            section = cmd.get_section('EXPORT')
            if section:
                my_export_name = section.values[0]
                exporting_cmd = cmd

        if package.cmake.content_map['ament_export_targets']:
            my_export_name = package.cmake.content_map['ament_export_targets'][0].first_token()

        if my_export_name is None:
            my_export_name = 'export_' + library

        export_targets.append(my_export_name)

        if not exporting_cmd:
            if lib_cmds:
                exporting_cmd = lib_cmds[0]
            else:
                exporting_cmd = Command('install', parent=package.cmake)
                exporting_cmd.add_section('TARGETS', [library])
                insert_in_order(package.cmake, exporting_cmd)

        check_complex_section(exporting_cmd, 'EXPORT', my_export_name)

        # Sort the sections to put EXPORT right after TARGETS
        sections_by_name = {s.name: s for s in exporting_cmd.get_real_sections()}
        assert exporting_cmd.sections.index(sections_by_name['TARGETS']) == 0
        ex_index = exporting_cmd.sections.index(sections_by_name['EXPORT'])

        # Reorder sections
        initial = exporting_cmd.sections[:1]
        export = exporting_cmd.sections[ex_index:ex_index + 1]
        intermediate = exporting_cmd.sections[1:ex_index]
        tail = exporting_cmd.sections[ex_index+1:]
        exporting_cmd.sections = initial + export + intermediate + tail

    section_check(package.cmake, export_targets, 'ament_export_targets')


def check_cmake_python_buildtype(package):
    if package.build_type == 'ament_cmake':
        acp = 'ament_cmake_python'
        build_tools = package.package_xml.get_packages_by_tag('buildtool_depend')
        if acp not in build_tools:
            package.package_xml.insert_new_packages('buildtool_depend', [acp])
            package.build_type = acp

        install_cmake_dependencies(package, {acp})

    if package.setup_py is None:
        create_setup_py(package)


@glinter
def update_misc_installs(package):
    extra_files_by_folder = collections.defaultdict(list)

    for rel_fn, package_file in sorted(package.components_by_name.items()):
        if package_file.__class__.needs_share_installation():
            extra_files_by_folder[rel_fn.parent].append(str(rel_fn.name))

    if package.cmake:
        existing_install_folders = set()
        for cmd in get_commands_by_type(package.cmake, InstallType.SHARE, None):
            for destination, subfolder in get_destinations(cmd):
                existing_install_folders.add(subfolder.lstrip('/'))

        for folder, files in sorted(extra_files_by_folder.items()):
            subfolder = str(folder)
            if subfolder == '.':
                subfolder = ''

            install_section_check(
                package.cmake, files, InstallType.SHARE, package.ros_version, subfolder=subfolder)

            if subfolder in existing_install_folders:
                existing_install_folders.remove(subfolder)

        for subfolder in existing_install_folders:
            install_section_check(package.cmake, [], InstallType.SHARE, package.ros_version, subfolder=subfolder)

    if get_python_usage(package) != PythonUsage.NONE:
        check_cmake_python_buildtype(package)
        if package.ros_version == 2:
            for folder, files in sorted(extra_files_by_folder.items()):
                package.setup_py.include_data_files(files, folder)


@glinter
def fix_double_directory_installs(package):
    if not package.cmake:
        return
    for cmd in package.cmake.content_map['install']:
        dir_section = cmd.get_section('DIRECTORY')
        dest_sections = cmd.get_sections('DESTINATION')

        if not dir_section or not dest_sections:
            continue
        directory = dir_section.values[0]
        final_slash = directory[-1] == '/'

        for section in dest_sections:
            destination = section.values[0]
            if not final_slash and destination.endswith(directory):
                # Remove double directory and final slash
                section.values[0] = destination[:-len(directory) - 1]
                cmd.mark_changed()
