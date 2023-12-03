from ros_introspect.components.cmake import ROS_TESTING_FLAGS, is_testing_group
from stylish_cmake_parser import Command, CommandGroup
from enum import IntEnum

BUILD_TARGET_COMMANDS = ['qt5_wrap_cpp', 'add_library', 'add_executable', 'add_rostest',
                         'target_include_directories', 'add_dependencies', 'target_link_libraries',
                         'set_target_properties', 'ament_target_dependencies']
TEST_COMMANDS = [('group', variable) for variable in ROS_TESTING_FLAGS] + \
                ['catkin_download_test_data',
                 'roslint_cpp', 'roslint_python', 'roslint_add_test',
                 'catkin_add_nosetests', 'catkin_add_gtest', 'ament_add_gtest', 'add_rostest_gtest',
                 'ament_lint_auto_find_test_dependencies']
INSTALL_COMMANDS = ['ament_export_targets', 'install', 'catkin_install_python', 'ament_python_install_module',
                    'ament_python_install_package', 'ament_export_include_directories', 'ament_export_libraries',
                    'ament_export_dependencies', 'pluginlib_export_plugin_description_file']

BASE_ORDERING = ['cmake_minimum_required', 'project',
                 ('group', 'CMAKE_C_STANDARD'), ('group', 'CMAKE_CXX_STANDARD'), ('group', 'CMAKE_COMPILER_IS_GNUCXX'),
                 ('set', 'CMAKE'),
                 'set_directory_properties', 'add_compile_options', 'find_package', 'pkg_check_modules',
                 'moveit_build_options',
                 ('set', 'OTHER'),
                 'catkin_generate_virtualenv', 'catkin_python_setup', 'add_definitions',
                 'add_message_files', 'add_service_files', 'add_action_files', 'rosidl_generate_interfaces',
                 'generate_dynamic_reconfigure_options', 'generate_messages', 'catkin_package', 'catkin_metapackage',
                 BUILD_TARGET_COMMANDS + ['include_directories'],
                 ]


class CMakeOrderStyle(IntEnum):
    TEST_FIRST = 1  # Test commands come strictly before install commands
    INSTALL_FIRST = 2  # Test commands come strictly after install commands
    MIXED = 3  # Test and install commands are intermingled
    UNKNOWN = 4  # There are not install commands and test commands


def classify_content_as_test_or_install(content):
    if isinstance(content, CommandGroup) and is_testing_group(content):
        return 'test'
    elif not isinstance(content, Command):
        return
    elif content.command_name in TEST_COMMANDS:
        return 'test'
    elif content.command_name in INSTALL_COMMANDS:
        return 'install'


def get_style(cmake):
    """Examine the contents of the cmake and determine the style."""
    cats = []
    for content in cmake.contents:
        cat = classify_content_as_test_or_install(content)
        if not cat:
            continue
        elif len(cats) == 0 or cats[-1] != cat:
            cats.append(cat)

            if len(cats) > 2:
                return CMakeOrderStyle.MIXED

    if len(cats) == 2:
        if cats[0] == 'install':
            return CMakeOrderStyle.INSTALL_FIRST
        else:
            return CMakeOrderStyle.TEST_FIRST
    return CMakeOrderStyle.UNKNOWN


def get_ordering(style):
    """Given the style, return the correct ordering."""
    if style == CMakeOrderStyle.INSTALL_FIRST:
        return BASE_ORDERING + INSTALL_COMMANDS + TEST_COMMANDS + ['ament_package']
    else:
        return BASE_ORDERING + TEST_COMMANDS + INSTALL_COMMANDS + ['ament_package']


def get_ordering_index(command_name, ordering):
    """
    Given a command name, determine the integer index into the ordering.

    The ordering is a list of strings and arrays of strings.

    If the command name matches one of the strings in the inner arrays,
    the index of the inner array is returned.

    If the command name matches one of the other strings, its index is returned.

     Otherwise, the length of the ordering is returned (putting non-matches at the end)
    """
    for i, o in enumerate(ordering):
        if isinstance(o, list):
            if command_name in o:
                return i
        elif command_name == o:
            return i
    if command_name:
        # TODO: Raise Warning
        print(f'\tUnsure of ordering for {command_name}')
    return len(ordering)


def get_sort_key(content, anchors, ordering):
    """
    Given a piece of cmake content, return a tuple representing its sort_key.

    The first element of the tuple is the ordering_index of the content.
    The second element is an additional variable used for sorting among elements with the same ordering_index

    Most notably, we want all build commands with a particular library/executable to be grouped together.
    In that case, we use the anchors parameter, which is an ordered list of all the library/executables in the file.
    Then, the second variable is a tuple itself, with the first element being the index of library/executable in the
    anchors list, and the second is an integer representing the canonical order of the build commands.
    """
    if content is None:
        return len(ordering) + 1, None
    index = None
    key = ()
    if content.__class__ == CommandGroup:
        key_token = ()
        for token in content.initial_cmd.get_tokens(include_name=True):
            if token == 'NOT':
                continue
            key_token = token
            break
        index = get_ordering_index(('group', key_token), ordering)
    elif content.command_name == 'set':
        token = content.get_tokens(include_name=True)[0]
        if token.startswith('CMAKE_'):
            set_type = 'CMAKE'
        else:
            set_type = 'OTHER'
        index = get_ordering_index(('set', set_type), ordering)
    else:  # Command
        index = get_ordering_index(content.command_name, ordering)
        if content.command_name in BUILD_TARGET_COMMANDS:
            token = content.first_token()
            key = anchors.index(token), BUILD_TARGET_COMMANDS.index(content.command_name)
        elif content.command_name == 'include_directories' and 'include_directories' in anchors:
            key = -1, anchors.index('include_directories')
        elif content.command_name == 'find_package':
            token = content.get_tokens()[0]
            if token == 'catkin' or token.startswith('ament_cmake'):
                key = -1, token  # Force ament_cmake/catkin to come first
            else:
                key = 0, token
    return index, key


def get_ordered_build_targets(cmake):
    targets = []
    for content in cmake.contents:
        if content.__class__ != Command:
            continue
        if content.command_name == 'include_directories':
            targets.append('include_directories')
            continue
        elif content.command_name not in BUILD_TARGET_COMMANDS:
            continue
        token = content.first_token()
        if token not in targets:
            targets.append(token)
    return targets


def get_insertion_index(cmake, cmd):
    anchors = get_ordered_build_targets(cmake)
    ordering = get_ordering(CMakeOrderStyle.TEST_FIRST)

    new_key = get_sort_key(cmd, anchors, ordering)
    i_index = 0

    for i, content in enumerate(cmake.contents):
        if isinstance(content, str):
            continue
        key = get_sort_key(content, anchors, ordering)
        if key <= new_key:
            i_index = i + 1
        elif key[0] != len(ordering):
            return i_index
    return len(cmake.contents)


def insert_in_order(cmake, cmd):
    cmake.insert(cmd, get_insertion_index(cmake, cmd))
