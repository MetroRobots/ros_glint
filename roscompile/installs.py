import collections
import fnmatch
import os

from ros_introspection.cmake import Command
from ros_introspection.setup_py import create_setup_py
from clean_ros.cleaners.cmake import check_complex_section, get_multiword_section

from .util import roscompile

FILES_TO_NOT_INSTALL = ['CHANGELOG.rst', 'README.md', '.travis.yml', 'bitbucket-pipelines.yml', 'setup.cfg',
                        '.pre-commit-config.yaml', 'LICENSE', 'LICENSE.txt']

# We define four install types, using a unique string to identify each (the keys in this dict)
# The values define a tuple, where the first element is a CMake keyword.
# The second value is a dictionary mapping catkin destinations to some CMake section names.
INSTALL_CONFIGS = {
    'exec': ('TARGETS', {'${CATKIN_PACKAGE_BIN_DESTINATION}': ['RUNTIME DESTINATION']}),
    'library': ('TARGETS', {'${CATKIN_PACKAGE_LIB_DESTINATION}': ['ARCHIVE DESTINATION', 'LIBRARY DESTINATION'],
                            '${CATKIN_GLOBAL_BIN_DESTINATION}': ['RUNTIME DESTINATION']}),
    'headers': ('FILES', {'${CATKIN_PACKAGE_INCLUDE_DESTINATION}': ['DESTINATION']}),
    'misc': ('FILES', {'${CATKIN_PACKAGE_SHARE_DESTINATION}': ['DESTINATION']})
}


COLCON_INSTALL_CONFIGS = {
    'exec': ('TARGETS', {'lib/${PROJECT_NAME}': ['DESTINATION']}),
    'headers': ('FILES', {'include': ['DESTINATION']}),
    'library': ('TARGETS', {'lib': ['ARCHIVE DESTINATION', 'LIBRARY DESTINATION'],
                            'bin': ['RUNTIME DESTINATION'],
                            # 'export_my_library': ['EXPORT'],
                            # 'include': ['INCLUDES DESTINATION'],
                            }),
    'misc': ('FILES', {'share/${PROJECT_NAME}': ['DESTINATION']}),
    'python': ('PROGRAMS', {'lib/${PROJECT_NAME}': ['DESTINATION']}),
}


def get_install_type(destination, section_name=None):
    """For a given catkin destination, return the matching install type."""
    for config_map in [INSTALL_CONFIGS, COLCON_INSTALL_CONFIGS]:
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


def matches_patterns(item, patterns):
    for pattern in patterns:
        if pattern[0] == pattern[-1] and pattern[0] == '"':
            pattern = pattern[1:-1]
        if fnmatch.fnmatch(item, pattern):
            return True


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


def install_section_check(cmake, items, install_type, directory=False, subfolder='', catkin=True):
    if catkin:
        section_name, destination_map = INSTALL_CONFIGS[install_type]
    else:
        section_name, destination_map = COLCON_INSTALL_CONFIGS[install_type]
    if directory and section_name == 'FILES':
        section_name = 'DIRECTORY'
    cmds = get_commands_by_type(cmake, install_type, subfolder)
    if len(items) == 0:
        for cmd in cmds:
            if len(get_install_types(cmd)) == 1:
                cmake.remove_command(cmd)
            else:
                remove_install_section(cmd, destination_map)
        return

    cmd = None
    items = [os.path.join(subfolder, item) for item in items]
    for cmd in cmds:
        if directory_matches(cmd, subfolder):
            return
        install_sections(cmd, destination_map, subfolder)
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
        return

    print('\tInstalling %s' % ', '.join(items))
    if cmd is None:
        cmd = Command('install')
        cmd.add_section(section_name, items)
        cmake.add_command(cmd)
        install_sections(cmd, destination_map, subfolder)
    elif section:
        # section = cmd.get_section(section_name)
        section.values += items
        cmd.changed = True

    if subfolder and len(str(cmd)) > 120 and section_name == 'FILES':
        section = cmd.get_section(section_name)
        section.name = 'DIRECTORY'
        section.values = [subfolder + '/']
        cmd.changed = True


@roscompile
def update_cplusplus_installs(package):
    if not package.cmake:
        return
    catkin = package.build_type == 'catkin'
    install_section_check(package.cmake, package.cmake.get_executables(), 'exec', catkin=catkin)
    install_section_check(package.cmake, package.cmake.get_libraries(), 'library', catkin=catkin)
    if package.name and package.source_code.has_header_files():
        if catkin:
            install_section_check(package.cmake, ['include/${PROJECT_NAME}/'], 'headers', directory=True)
        else:
            if not package.cmake.content_map['ament_export_include_directories']:
                cmd = Command('ament_export_include_directories')
                cmd.add_section('', ['include'])
                package.cmake.add_command(cmd)
            install_section_check(package.cmake, ['include/'], 'headers', directory=True, catkin=False)


@roscompile
def update_misc_installs(package):
    extra_files_by_folder = collections.defaultdict(list)
    rel_paths = [obj.rel_fn for obj in package.launches + package.plugin_configs + package.urdf_files]
    rel_paths += package.misc_files
    for rel_path in sorted(rel_paths):
        if rel_path in FILES_TO_NOT_INSTALL:
            continue
        path, base = os.path.split(rel_path)
        extra_files_by_folder[path].append(base)
    if package.cmake:
        for folder, files in sorted(extra_files_by_folder.items()):
            install_section_check(package.cmake, files, 'misc', catkin=package.build_type == 'catkin', subfolder=folder)
    else:
        if package.setup_py is None:
            create_setup_py(package)

        for folder, files in sorted(extra_files_by_folder.items()):
            package.setup_py.include_data_files(files, folder)


@roscompile
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
                cmd.changed = True
