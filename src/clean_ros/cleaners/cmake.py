import re

from ..core import clean_ros
from ..cmake_ordering import insert_in_order
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


def ensure_section_values(cmake, section, items, alpha_order=True, ignore_quoting=False):
    """Ensure the CMake command with the given section has all of the values in items."""
    existing = cmake.resolve_variables(section.values)

    needed_items = []
    for item in items:
        if item in existing or item in section.values:
            continue

        if ignore_quoting:
            quoted = f'"{item}"'
            if quoted in existing or quoted in section.values:
                continue

        # TODO: Should maybe follow quote style
        needed_items.append(item)

    if needed_items:
        section.add_values(needed_items, alpha_order)
        section.parent.changed = True
        return True
    return False


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
            insert_in_order(cmake, cmd)
            changed = True

    if section is None:
        cmd.add_section(section_name, sorted(items))
        changed = True
    else:
        changed |= ensure_section_values(cmake, section, items, alpha_order, ignore_quoting)
    return changed


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
            insert_in_order(cmake, cmd)


@clean_ros
def check_cmake_dependencies(package):
    if not package.cmake:
        return
    dependencies = package.get_dependencies(DependencyType.BUILD)
    install_cmake_dependencies(package, dependencies)


@clean_ros
def check_generators(package):
    all_interfaces = package.get_ros_interfaces()
    if not package.cmake or not all_interfaces:
        return

    cmake = package.cmake.contents
    if package.ros_version == 1:
        changed = section_check(cmake, [m.fn for m in package.messages],
                                'add_message_files', 'FILES')
        changed |= section_check(cmake, [s.fn for s in package.services],
                                 'add_service_files', 'FILES')
        changed |= section_check(cmake, [a.fn for a in package.actions],
                                 'add_action_files', 'FILES')
        changed |= section_check(cmake, ['message_generation'], 'find_package', 'COMPONENTS')
        changed |= section_check(cmake, ['message_runtime'], 'catkin_package', 'CATKIN_DEPENDS')

        generate_cmd = 'generate_messages'
    else:
        generate_cmd = 'rosidl_generate_interfaces'

        if len(package.cmake.content_map[generate_cmd]) == 0:
            cmd = Command('rosidl_generate_interfaces')
            cmd.add_section('${PROJECT_NAME}')
            cmake.add_command(cmd)
            package.cmake.changed = True

        rel_fns = [ros_interface.rel_fn for ros_interface in all_interfaces]
        changed |= section_check(cmake, rel_fns, generate_cmd, '', ignore_quoting=True)

        install_cmake_dependencies(package, {'rosidl_default_generators'})
        changed |= section_check(cmake, ['rosidl_default_runtime'], 'ament_export_dependencies', '')

    msg_deps = set()
    for ros_interface in all_interfaces:
        msg_deps |= ros_interface.get_dependencies(DependencyType.BUILD)

    if msg_deps:
        changed |= section_check(cmake, msg_deps, generate_cmd, 'DEPENDENCIES', zero_okay=True)
    else:
        changed |= section_check(cmake, msg_deps, generate_cmd, zero_okay=True)

    if changed:
        package.cmake.changed = True
