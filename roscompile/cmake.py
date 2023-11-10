import re

from ros_introspection.cmake import Command, CommandGroup, SectionStyle
from ros_introspection.resource_list import is_message, is_service
from ros_introspection.source_code_file import CPLUS, CPLUS2

from .util import get_config, get_ignore_data, roscompile

SHOULD_ALPHABETIZE = ['COMPONENTS', 'DEPENDENCIES', 'FILES', 'CATKIN_DEPENDS']
NEWLINE_PLUS_4 = '\n    '
NEWLINE_PLUS_8 = '\n        '
CATKIN_INSTALL_PYTHON_PRENAME = '\n                      '  # newline plus len('catkin_install_python(')

ALL_SPACES = re.compile(r' +')


def check_cmake_dependencies_helper1(cmake, dependencies, check_catkin_pkg=True):
    if len(dependencies) == 0:
        return
    if len(cmake.content_map['find_package']) == 0:
        cmd = Command('find_package')
        cmd.add_section('', ['catkin'])
        cmd.add_section('REQUIRED')
        cmake.add_command(cmd)

    for cmd in cmake.content_map['find_package']:
        tokens = cmd.get_tokens()
        if tokens and tokens[0] == 'catkin' and cmd.get_section('REQUIRED'):
            req_sec = cmd.get_section('REQUIRED')
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
                    cmd.changed = True
    if check_catkin_pkg:
        cmake.section_check(dependencies, 'catkin_package', 'CATKIN_DEPENDS')


def check_cmake_dependencies_helper2(cmake, dependencies):
    if len(dependencies) == 0:
        return

    existing = set()
    for cmd in cmake.content_map['find_package']:
        existing.update(cmd.get_tokens())

    for pkg in sorted(dependencies - existing):
        cmd = Command('find_package')
        cmd.add_section('', [pkg])
        cmd.add_section('REQUIRED')
        cmake.add_command(cmd)


def check_cmake_dependencies_helper(package, dependencies, check_catkin_pkg=True):
    if not dependencies:
        return

    if package.build_type == 'ament_cmake':
        check_cmake_dependencies_helper2(package.cmake, dependencies)
    else:
        check_cmake_dependencies_helper1(package.cmake, dependencies)


@roscompile
def check_cmake_dependencies(package):
    if not package.cmake:
        return
    dependencies = package.get_dependencies_from_msgs()
    dependencies.update(package.get_build_dependencies())
    check_cmake_dependencies_helper(package, dependencies)


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


def match_generator_name(package, name):
    for gen in package.get_all_generators():
        if name == gen.base_name:
            return gen


def get_msg_dependencies_from_source(package, sources):
    deps = set()
    for rel_fn in sources:
        if rel_fn not in package.source_code.sources:
            continue
        src = package.source_code.sources[rel_fn]
        for pkg, name in src.search_lines_for_pattern(CPLUS):
            if len(name) == 0 or name[-2:] != '.h':
                continue
            name = name.replace('.h', '')
            if is_message(pkg, name) or is_service(pkg, name):
                deps.add(pkg)
            elif pkg == package.name and match_generator_name(package, name):
                deps.add(pkg)
        for pkg, gen_type, name in src.search_lines_for_pattern(CPLUS2):
            if gen_type == 'msg' or gen_type == 'srv':
                deps.add(pkg)
    if package.dynamic_reconfigs:
        deps.add(package.name)
    return sorted(deps)


@roscompile
def check_exported_dependencies(package):
    if not package.cmake:
        return
    targets = package.cmake.get_target_build_rules()
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
            add_deps = Command('add_dependencies')
            add_add_deps = True  # Need to wait to add the command for proper sorting

        if len(add_deps.sections) == 0:
            add_deps.add_section('', [target])
            add_deps.changed = True

        section = add_deps.sections[0]
        if cat_depend and '${catkin_EXPORTED_TARGETS}' not in section.values:
            section.add('${catkin_EXPORTED_TARGETS}')
            add_deps.changed = True
        if self_depend:
            tokens = [package.cmake.resolve_variables(s) for s in section.values]
            key = '${%s_EXPORTED_TARGETS}' % package.name
            if key not in tokens:
                section.add(key)
                add_deps.changed = True

        if add_add_deps:
            package.cmake.add_command(add_deps)


def remove_pattern(section, pattern):
    prev_len = len(section.values)
    section.values = [v for v in section.values if pattern not in v]
    return prev_len != len(section.values)


@roscompile
def remove_old_style_cpp_dependencies(package):
    if not package.cmake:
        return
    global_changed = False
    targets = package.cmake.get_target_build_rules()
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


def get_targeted_command(cmake, cmd_name, target_name):
    for cmd in cmake.content_map[cmd_name]:
        tokens = cmd.get_tokens()
        if tokens and target_name == tokens[0]:
            return cmd


def targeted_section_check(cmake, cmd_name, section_name, items, style=None):
    for target in cmake.get_libraries() + cmake.get_executables():
        cmd = get_targeted_command(cmake, cmd_name, target)
        if cmd is None:
            print(f'\tAdding {cmd_name} for {target}')
            cmd = Command(cmd_name)
            if section_name:
                cmd.add_section('', [target])
                cmd.add_section(section_name, items, style)
            else:
                cmd.add_section(section_name, [target] + items, style)
            cmake.add_command(cmd)
        else:
            section = cmd.get_section(section_name)
            if style:
                section.style = style
            cmake.ensure_section_values(cmd, section, items, alpha_order=False)


@roscompile
def target_catkin_libraries(package):
    if not package.cmake:
        return

    if package.ros_version == 1:
        deps = ['${catkin_LIBRARIES}']
        command = 'target_link_libraries'
        style = None
    else:
        deps = sorted(package.get_build_dependencies())
        command = 'ament_target_dependencies'
        style = SectionStyle(prename='', val_sep='\n  ')
    targeted_section_check(package.cmake, command, '', deps, style)


@roscompile
def check_generators(package):
    if len(package.generators) == 0 or not package.cmake:
        return

    if package.ros_version == 1:
        for gen_type, cmake_cmd in [('msg', 'add_message_files'),
                                    ('srv', 'add_service_files'),
                                    ('action', 'add_action_files')]:
            names = [gen.name for gen in package.generators[gen_type]]
            package.cmake.section_check(names, cmake_cmd, 'FILES')

        package.cmake.section_check(['message_generation'], 'find_package', 'COMPONENTS')
        package.cmake.section_check(['message_runtime'], 'catkin_package', 'CATKIN_DEPENDS')
        for cmd in package.cmake.content_map['catkin_package']:
            section = cmd.get_section('CATKIN_DEPENDS')
            if 'message_generation' in section.values:
                section.values.remove('message_generation')
                cmd.changed = True
        generate_cmd = 'generate_messages'
    else:
        generate_cmd = 'rosidl_generate_interfaces'
        if len(package.cmake.content_map[generate_cmd]) == 0:
            cmd = Command('rosidl_generate_interfaces')
            cmd.add_section('${PROJECT_NAME}')
            package.cmake.add_command(cmd)

        rel_fns = []
        for gen_type in ['msg', 'srv', 'action']:
            for gen in package.generators[gen_type]:
                rel_fns.append(gen.rel_fn)
        package.cmake.section_check(rel_fns, generate_cmd, '', ignore_quoting=True)

        check_cmake_dependencies_helper(package, {'rosidl_default_generators'})

        package.cmake.section_check(['rosidl_default_runtime'], 'ament_export_dependencies', '')

    msg_deps = package.get_dependencies_from_msgs()
    if msg_deps:
        package.cmake.section_check(msg_deps, generate_cmd,
                                    'DEPENDENCIES', zero_okay=True)
    else:
        package.cmake.section_check(msg_deps, generate_cmd,
                                    zero_okay=True)


@roscompile
def check_includes(package):
    if not package.cmake or not package.source_code.get_source_by_language('c++'):
        return

    if package.ros_version == 1:
        has_includes = False
        if package.source_code.has_header_files():
            package.cmake.section_check(['include'], 'catkin_package', 'INCLUDE_DIRS')
            package.cmake.section_check(['include'], 'include_directories', alpha_order=False)
            has_includes = True

        if len(package.source_code.get_source_by_language('c++')) > 0:
            package.cmake.section_check(['${catkin_INCLUDE_DIRS}'], 'include_directories', alpha_order=False)
            has_includes = True

        if not has_includes and 'include_directories' in package.cmake.content_map:
            for cmd in package.cmake.content_map['include_directories']:
                package.cmake.remove_command(cmd)
    else:
        include_directories = []
        if package.source_code.has_header_files():
            include_directories.append('$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>')

        include_directories.append('$<INSTALL_INTERFACE:include>')

        targeted_section_check(package.cmake, 'target_include_directories', 'PUBLIC', include_directories,
                               SectionStyle(name_val_sep='\n  ', val_sep='\n  '))


@roscompile
def check_library_setup(package):
    if not package.cmake:
        return
    if package.build_type == 'catkin':
        package.cmake.section_check(package.cmake.get_libraries(), 'catkin_package', 'LIBRARIES')


def alphabetize_sections_helper(cmake):
    for content in cmake.contents:
        if content.__class__ == Command:
            for section in content.get_real_sections():
                if section.name in SHOULD_ALPHABETIZE:
                    sorted_values = sorted(section.values)
                    if sorted_values != section.values:
                        section.values = sorted_values
                        content.changed = True
        elif content.__class__ == CommandGroup:
            alphabetize_sections_helper(content.sub)


@roscompile
def alphabetize_sections(package):
    if not package.cmake:
        return
    alphabetize_sections_helper(package.cmake)


@roscompile
def prettify_catkin_package_cmd(package):
    if not package.cmake:
        return
    for cmd in package.cmake.content_map['catkin_package']:
        for section in cmd.get_real_sections():
            section.style.prename = NEWLINE_PLUS_4
        cmd.changed = True


@roscompile
def prettify_package_lists(package):
    if not package.cmake:
        return
    acceptable_styles = [(NEWLINE_PLUS_8, NEWLINE_PLUS_8), (NEWLINE_PLUS_4, NEWLINE_PLUS_8)]

    for cmd_name, section_name in [('find_package', 'COMPONENTS'), ('catkin_package', 'CATKIN_DEPENDS')]:
        for cmd in package.cmake.content_map[cmd_name]:
            for section in cmd.get_real_sections():
                if section.name != section_name:
                    continue
                n = len(str(section))
                if n > 120:
                    key = section.style.name_val_sep, section.style.val_sep
                    if key not in acceptable_styles:
                        section.style.name_val_sep = NEWLINE_PLUS_8
                        section.style.val_sep = NEWLINE_PLUS_8
                        cmd.changed = True


@roscompile
def prettify_msgs_srvs(package):
    if not package.cmake:
        return
    for cmd in package.cmake.content_map['add_message_files'] + package.cmake.content_map['add_service_files']:
        for section in cmd.get_real_sections():
            if len(section.values) > 1:
                section.style.name_val_sep = NEWLINE_PLUS_4
                section.style.val_sep = NEWLINE_PLUS_4
            cmd.changed = True


@roscompile
def prettify_installs(package):
    if not package.cmake:
        return
    for cmd in package.cmake.content_map['install']:
        cmd.sections = [s for s in cmd.sections if not isinstance(s, str)]
        if not cmd.sections:
            continue
        cmd.changed = True
        if '\n' in cmd.sections[0].style.prename:
            new_style = cmd.sections[0].style.prename
        else:
            new_style = NEWLINE_PLUS_8

        zeroed = False
        for section in cmd.sections[1:]:
            if len(section.values) == 0:
                section.style.prename = new_style
                zeroed = True
            elif not zeroed:
                section.style.prename = new_style
            else:
                section.style.prename = ''

    for cmd in package.cmake.content_map['catkin_install_python']:
        section = cmd.sections[1]
        if section.style.prename != CATKIN_INSTALL_PYTHON_PRENAME:
            section.style.prename = CATKIN_INSTALL_PYTHON_PRENAME
            cmd.changed = True


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
                        continue

            cmake.contents = cmake.contents[:i] + cmake.contents[i + 1:]

    # Remove from Commands/CommandGroups
    for content in cmake.contents:
        if content.__class__ == Command:
            remove_cmake_command_comments_helper(content, ignorables, replacement)
        elif content.__class__ == CommandGroup:
            remove_cmake_comments_helper(content.sub, ignorables, replacement)
    cmake.contents = remove_empty_strings(cmake.contents)


@roscompile
def remove_boilerplate_cmake_comments(package):
    if not package.cmake:
        return
    ignorables = get_ignore_data('cmake', {'package': package.name})
    remove_cmake_comments_helper(package.cmake, ignorables)
    remove_empty_cmake_lines(package)


@roscompile
def remove_empty_cmake_lines(package):
    if not package.cmake:
        return
    for i, content in enumerate(package.cmake.contents[:-2]):
        if str(content)[-1] == '\n' and package.cmake.contents[i + 1] == '\n' and package.cmake.contents[i + 2] == '\n':
            package.cmake.contents[i + 1] = ''
    package.cmake.contents = remove_empty_strings(package.cmake.contents)


@roscompile
def enforce_cmake_ordering(package, config=None):
    if not package.cmake:
        return
    if config is None:
        config = get_config()
    default_style = config.get('cmake_style')
    package.cmake.enforce_ordering(default_style)
