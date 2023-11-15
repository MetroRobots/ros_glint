import re

from ..core import clean_ros
from ..util import get_ignore_data

from ros_introspect.package import DependencyType
from stylish_cmake_parser import Command, CommandGroup
ALL_SPACES = re.compile(r' +')


def remove_empty_strings(a):
    return list(filter(lambda x: x != '', a))


def remove_cmake_command_comments_helper(command, ignorables, replacement=''):
    for i, section in enumerate(command.sections):
        if not isinstance(section, str):
            continue
        for ignorable in ignorables:
            while ignorable in command.sections[i]:
                command.changed = True
                command.sections[i] = command.sections[i].replace(ignorable, replacement)
    if command.changed:
        command.sections = remove_empty_strings(command.sections)
        if command.sections == ['\n']:
            command.sections = []


def remove_cmake_comments_helper(cmake, ignorables, replacement=''):
    changed = False
    # Remove Raw Strings
    for ignorable in ignorables:
        while ignorable in cmake.contents:
            i = cmake.contents.index(ignorable)
            if i > 1:
                one_before = cmake.contents[i - 1]
                two_before = cmake.contents[i - 2]
                if isinstance(one_before, str) and isinstance(two_before, str):
                    if ALL_SPACES.match(one_before) and two_before[-1] == '\n':
                        cmake.contents = cmake.contents[:i - 1] + cmake.contents[i + 1:]
                        changed = True
                        continue

            cmake.contents = cmake.contents[:i] + cmake.contents[i + 1:]
            changed = True

    # Remove from Commands/CommandGroups
    for content in cmake.contents:
        if isinstance(content, Command):
            remove_cmake_command_comments_helper(content, ignorables, replacement)
        elif isinstance(content, CommandGroup):
            remove_cmake_comments_helper(content.contents, ignorables, replacement)
    cmake.contents = remove_empty_strings(cmake.contents)
    return changed


@clean_ros
def remove_empty_cmake_lines(package):
    cmake = package.cmake.contents
    for i, content in enumerate(cmake.contents[:-2]):
        if str(content)[-1] == '\n' and cmake.contents[i + 1] == '\n' and cmake.contents[i + 2] == '\n':
            package.cmake.changed = True
            cmake.contents[i + 1] = ''
    cmake.contents = remove_empty_strings(cmake.contents)


@clean_ros
def remove_boilerplate_cmake_comments(package):
    if not package.cmake:
        return

    ignorables = get_ignore_data('cmake', {'package': package.name})
    if remove_cmake_comments_helper(package.cmake.contents, ignorables):
        package.cmake.changed = True
    remove_empty_cmake_lines(package)


def get_command_section(cmake, cmd_name, section_name):
    """Return the first matching section from the properly named command"""
    for cmd in cmake.content_map[cmd_name]:
        s = cmd.get_section(section_name)
        if s:
            return s


def section_check(cmake, items, cmd_name, section_name='', zero_okay=False, alpha_order=True, ignore_quoting=False):
    """Ensure there's a CMake command of the given type with the given section name and items."""
    changed = False

    if len(items) == 0 and not zero_okay:
        return changed

    section = get_command_section(cmake, cmd_name, section_name)

    if not section:
        if cmake.content_map[cmd_name]:
            cmd = cmake.content_map[cmd_name][0]
        else:
            cmd = Command(cmd_name)
            cmake.insert(cmd)
            changed = True

    if section is None:
        cmd.add_section(section_name, sorted(items))
        changed = True


def install_cmake_dependencies(package, dependencies, check_catkin_pkg=True):
    if not dependencies:
        return

    cmake = package.cmake.contents
    if package.ros_version == 1:
        if len(cmake.content_map['find_package']) == 0:
            cmd = Command('find_package')
            # TODO: Standard indentation
            cmd.add_section('', ['catkin'])
            cmd.add_section('REQUIRED')
            cmake.add_command(cmd)
            package.cmake.changed = True

        for cmd in cmake.content_map['find_package']:
            tokens = cmd.get_tokens()
            if not tokens or tokens[0] != 'catkin':
                continue

            req_sec = cmd.get_section('REQUIRED')
            if not req_sec:
                continue
            section = cmd.get_section('COMPONENTS')
            if section is None and req_sec.values:
                section = req_sec  # Allow packages to be listed without COMPONENTS keyword
            if section is None:
                cmd.add_section('COMPONENTS', sorted(dependencies))
                package.cmake.changed = True
            else:
                existing = cmake.resolve_variables(section.values)
                needed_items = dependencies - set(existing)
                if needed_items:
                    section.add_values(needed_items)
                    cmd.changed = True
                    package.cmake.changed = True
        if check_catkin_pkg:
            ret = section_check(cmake, dependencies, 'catkin_package', 'CATKIN_DEPENDS')
            if ret:
                package.cmake.changed = True
    else:
        existing = set()
        for cmd in cmake.content_map['find_package']:
            existing.update(cmd.get_tokens())

        for pkg in sorted(dependencies - existing):
            cmd = Command('find_package')
            cmd.add_section('', [pkg])
            cmd.add_section('REQUIRED')
            cmake.insert(cmd)


@clean_ros
def check_cmake_dependencies(package):
    if not package.cmake:
        return
    dependencies = package.get_dependencies(DependencyType.BUILD)
    install_cmake_dependencies(package, dependencies)
