import re

from ..core import glinter
from ..cmake_ordering import insert_in_order
from ..util import get_ignore_data

from ros_introspect.package import DependencyType
from ros_introspect.components.source_code import CPP_INCLUDE1, CPP_INCLUDE2
from ros_introspect.ros_resources import ROSResources
from stylish_cmake_parser import Command, CommandGroup, SectionStyle
ALL_SPACES = re.compile(r' +')
NEWLINE_PLUS_2 = '\n  '
NEWLINE_PLUS_4 = '\n    '
NEWLINE_PLUS_8 = '\n        '


def remove_empty_strings(a):
    return list(filter(lambda x: x != '', a))


def remove_cmake_command_comments_helper(command, ignorables, replacement=''):
    for i, section in enumerate(command.sections):
        if not isinstance(section, str):
            continue
        for ignorable in ignorables:
            while ignorable in command.sections[i]:
                command.mark_changed()
                command.sections[i] = command.sections[i].replace(ignorable, replacement)
    if command.changed:
        command.sections = remove_empty_strings(command.sections)
        if command.sections == ['\n']:
            command.sections = []


def remove_cmake_comments_helper(cmake, ignorables, replacement=''):
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
                        cmake.mark_changed()
                        continue

            cmake.contents = cmake.contents[:i] + cmake.contents[i + 1:]
            cmake.mark_changed()

    # Remove from Commands/CommandGroups
    for content in cmake.contents:
        if isinstance(content, Command):
            remove_cmake_command_comments_helper(content, ignorables, replacement)
        elif isinstance(content, CommandGroup):
            remove_cmake_comments_helper(content.contents, ignorables, replacement)
    cmake.contents = remove_empty_strings(cmake.contents)


@glinter
def remove_empty_cmake_lines(package):
    cmake = package.cmake
    for i, content in enumerate(cmake.contents[:-2]):
        if str(content)[-1] == '\n' and cmake.contents[i + 1] == '\n' and cmake.contents[i + 2] == '\n':
            cmake.contents[i + 1] = ''
            cmake.mark_changed()
    cmake.contents = remove_empty_strings(cmake.contents)


@glinter
def remove_boilerplate_cmake_comments(package):
    if not package.cmake:
        return

    ignorables = get_ignore_data('cmake', {'package': package.name})
    remove_cmake_comments_helper(package.cmake, ignorables)
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
        return True
    return False


def section_check(cmake, items, cmd_name, section_name='', zero_okay=False, alpha_order=True, ignore_quoting=False):
    """Ensure there's a CMake command of the given type with the given section name and items."""

    if len(items) == 0 and not zero_okay:
        return

    section = get_command_section(cmake, cmd_name, section_name)

    if not section:
        if cmake.content_map[cmd_name]:
            cmd = cmake.content_map[cmd_name][0]
        else:
            cmd = Command(cmd_name, parent=cmake)
            insert_in_order(cmake, cmd)

    if section is None:
        cmd.add_section(section_name, sorted(items))
    else:
        ensure_section_values(cmake, section, items, alpha_order, ignore_quoting)
    return


def targeted_section_check(cmake, cmd_name, section_name, items, style=None):
    def get_targeted_command(target_name):
        for cmd in cmake.content_map[cmd_name]:
            tokens = cmd.get_tokens()
            if tokens and target_name == tokens[0]:
                return cmd

    for target in cmake.get_libraries() + cmake.get_executables():
        cmd = get_targeted_command(target)
        if cmd is None:
            cmd = Command(cmd_name, parent=cmake)
            if section_name:
                cmd.add_section('', [target])
                cmd.add_section(section_name, items, style)
            else:
                cmd.add_section(section_name, [target] + items, style)
            insert_in_order(cmake, cmd)
        else:
            section = cmd.get_section(section_name)
            if not section:
                continue
            if style:
                section.style = style
            ensure_section_values(cmake, section, items, alpha_order=False)


def get_multiword_section(cmd, words):
    """Find a section that matches the last word, assuming all the previous sections matched the other words.

    Our definition of a CMake command section is ONE all-caps word followed by tokens.
    Installing stuff requires these weird TWO word sections (i.e. ARCHIVE DESTINATION).

    Ergo, we need to find the section that matches the second word, presuming the section
    before matched the first word.
    """
    i = 0
    for section in cmd.get_real_sections():
        if section.name == words[i]:
            # One word matches
            if i < len(words) - 1:
                # If there are more words, increment the counter
                i += 1
            else:
                # Otherwise, we matched all the words. Return this section
                return section
        else:
            # If the word doesn't match, we need to start searching from the first word again
            i = 0


def check_complex_section(cmd, key, value):
    """Find the section matching the key and ensure the value is in it.

    Key could be multiple words, see get_multiword_section.
    If the appropriate section is not found, it adds it.
    """
    words = key.split()
    if len(words) == 1:
        section = cmd.get_section(key)
    else:
        section = get_multiword_section(cmd, words)

    if section:
        if value not in section.values:
            section.add(value)
    else:
        cmd.add_section(key, [value], SectionStyle(NEWLINE_PLUS_8))


def install_cmake_dependencies(package, dependencies, check_catkin_pkg=True):
    if not dependencies:
        return

    cmake = package.cmake
    if package.ros_version == 1:
        if len(cmake.content_map['find_package']) == 0:
            cmd = Command('find_package', parent=cmake)
            # TODO: Standard indentation
            cmd.add_section('', ['catkin'])
            cmd.add_section('REQUIRED')
            cmake.add_command(cmd)

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
            else:
                existing = cmake.resolve_variables(section.values)
                needed_items = dependencies - set(existing)
                if needed_items:
                    section.add_values(needed_items)
        if check_catkin_pkg:
            section_check(cmake, dependencies, 'catkin_package', 'CATKIN_DEPENDS')
    else:
        existing = set()
        for cmd in cmake.content_map['find_package']:
            existing.update(cmd.get_tokens())

        for pkg in sorted(dependencies - existing):
            cmd = Command('find_package', parent=cmake)
            cmd.add_section('', [pkg])
            cmd.add_section('REQUIRED')
            insert_in_order(cmake, cmd)


@glinter
def check_cmake_dependencies(package):
    if not package.cmake:
        return
    dependencies = package.get_dependencies(DependencyType.BUILD)
    install_cmake_dependencies(package, dependencies)


@glinter
def check_generators(package):
    all_interfaces = package.get_ros_interfaces()
    if not package.cmake or not all_interfaces:
        return

    cmake = package.cmake
    if package.ros_version == 1:
        section_check(cmake, [m.fn for m in package.messages], 'add_message_files', 'FILES')
        section_check(cmake, [s.fn for s in package.services], 'add_service_files', 'FILES')
        section_check(cmake, [a.fn for a in package.actions], 'add_action_files', 'FILES')
        section_check(cmake, ['message_generation'], 'find_package', 'COMPONENTS')
        section_check(cmake, ['message_runtime'], 'catkin_package', 'CATKIN_DEPENDS')

        generate_cmd = 'generate_messages'
    else:
        generate_cmd = 'rosidl_generate_interfaces'

        if not cmake.content_map[generate_cmd]:
            cmd = Command('rosidl_generate_interfaces', parent=cmake)
            cmd.add_section('${PROJECT_NAME}')
            insert_in_order(cmake, cmd)

        rel_fns = [str(ros_interface.rel_fn) for ros_interface in all_interfaces]
        section_check(cmake, rel_fns, generate_cmd, '', ignore_quoting=True)

        install_cmake_dependencies(package, {'rosidl_default_generators'})
        section_check(cmake, ['rosidl_default_runtime'], 'ament_export_dependencies', '')

    msg_deps = set()
    for ros_interface in all_interfaces:
        msg_deps |= ros_interface.get_dependencies(DependencyType.BUILD)

    if msg_deps:
        section_check(cmake, msg_deps, generate_cmd, 'DEPENDENCIES', zero_okay=True)
    else:
        section_check(cmake, msg_deps, generate_cmd, zero_okay=True)


@glinter
def check_includes(package):
    if not package.cmake:
        return

    has_cpp = any(source_file.language == 'c++' for source_file in package.source_code)
    has_header_files = any(package.get_source_by_tags('header'))

    if not has_cpp:
        return

    cmake = package.cmake

    if package.ros_version == 1:
        if has_header_files:
            section_check(cmake, ['include'], 'catkin_package', 'INCLUDE_DIRS')
            section_check(cmake, ['include'], 'include_directories', alpha_order=False)

        section_check(cmake, ['${catkin_INCLUDE_DIRS}'], 'include_directories', alpha_order=False)
    else:
        include_directories = []
        if has_header_files:
            include_directories.append('$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>')

        include_directories.append('$<INSTALL_INTERFACE:include>')

        targeted_section_check(package.cmake, 'target_include_directories', 'PUBLIC', include_directories,
                               SectionStyle(name_val_sep='\n  ', val_sep='\n  '))


@glinter
def target_catkin_libraries(package):
    if not package.cmake:
        return

    if package.ros_version == 1:
        deps = ['${catkin_LIBRARIES}']
        command = 'target_link_libraries'
        style = None
    else:
        deps = sorted(package.get_dependencies(DependencyType.BUILD))
        command = 'ament_target_dependencies'
        style = SectionStyle(prename='', val_sep='\n  ')
    targeted_section_check(package.cmake, command, '', deps, style)


@glinter
def check_library_setup(package):
    if not package.cmake:
        return
    if package.ros_version != 1:
        return

    section_check(package.cmake, package.cmake.get_libraries(),
                  'catkin_package', 'LIBRARIES')


def get_target_build_rules(cmake):
    targets = {}
    targets.update(cmake.get_build_rules('add_library'))
    targets.update(cmake.get_build_rules('add_executable'))
    return targets


def get_msg_dependencies_from_source(package, sources):
    resources = ROSResources.get()
    deps = set()
    for rel_fn in sources:
        srcs = [sc for sc in package.source_code if str(sc.rel_fn) == rel_fn]
        if not srcs:
            continue
        src = srcs[0]
        for pkg, name in src.search_lines_for_pattern(CPP_INCLUDE1):
            if len(name) == 0 or name[-2:] != '.h':
                continue
            name = name.replace('.h', '')
            if resources.is_message(pkg, name) or resources.is_service(pkg, name):
                deps.add(pkg)
            elif pkg == package.name and any(name == ros_int.name for ros_int in package.get_ros_interfaces()):
                deps.add(pkg)
        for pkg, gen_type, name in src.search_lines_for_pattern(CPP_INCLUDE2):
            if gen_type == 'msg' or gen_type == 'srv':
                deps.add(pkg)
    if package.dynamic_reconfig:
        deps.add(package.name)
    return sorted(deps)


def get_matching_add_depends(cmake, search_target):
    valid_targets = {search_target}
    alt_target = cmake.resolve_variables(search_target)
    if alt_target != search_target:
        valid_targets.add(alt_target)

    for cmd in cmake.content_map['add_dependencies']:
        target = cmd.first_token()
        if target in valid_targets:
            return cmd
        resolved_target = cmake.resolve_variables(target)
        if resolved_target in valid_targets:
            return cmd


@glinter
def check_exported_dependencies(package):
    if not package.cmake or package.ros_version == 2:
        return
    cmake = package.cmake
    targets = get_target_build_rules(cmake)

    for target, sources in targets.items():
        deps = get_msg_dependencies_from_source(package, sources)
        if len(deps) == 0:
            continue

        if package.name in deps:
            self_depend = True
            if len(deps) == 1:
                cat_depend = False
            else:
                cat_depend = True
        else:
            self_depend = False
            cat_depend = True

        add_deps = get_matching_add_depends(package.cmake, target)
        add_add_deps = False

        if add_deps is None:
            add_deps = Command('add_dependencies', parent=cmake)
            add_add_deps = True  # Need to wait to add the command for proper sorting

        if len(add_deps.sections) == 0:
            add_deps.add_section('', [target])

        section = add_deps.sections[0]
        if cat_depend and '${catkin_EXPORTED_TARGETS}' not in section.values:
            section.add('${catkin_EXPORTED_TARGETS}')
        if self_depend:
            tokens = [cmake.resolve_variables(s) for s in section.values]
            key = '${%s_EXPORTED_TARGETS}' % package.name
            if key not in tokens:
                section.add(key)

        if add_add_deps:
            insert_in_order(cmake, add_deps)


@glinter
def remove_old_style_cpp_dependencies(package):
    def remove_pattern(section, pattern):
        prev_len = len(section.values)
        section.values = [v for v in section.values if pattern not in v]
        return prev_len != len(section.values)

    if not package.cmake:
        return
    global_changed = False
    targets = get_target_build_rules(package.cmake)
    for target in targets:
        add_deps = get_matching_add_depends(package.cmake, target)
        if add_deps is None or len(add_deps.sections) == 0:
            continue

        section = add_deps.sections[0]
        changed = remove_pattern(section, '_generate_messages_cpp')
        changed = remove_pattern(section, '_gencpp') or changed
        changed = remove_pattern(section, '_gencfg') or changed
        if changed:
            add_deps.changed = True
            global_changed = True
    if global_changed:
        check_exported_dependencies(package)
