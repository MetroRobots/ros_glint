from ros_introspection.cmake import Command, CommandGroup, SectionStyle
from ros_introspection.resource_list import is_message, is_service
from ros_introspection.source_code_file import CPLUS, CPLUS2
from clean_ros.cleaners.cmake import targeted_section_check


from .util import get_config, roscompile

SHOULD_ALPHABETIZE = ['COMPONENTS', 'DEPENDENCIES', 'FILES', 'CATKIN_DEPENDS']
NEWLINE_PLUS_4 = '\n    '
NEWLINE_PLUS_8 = '\n        '
CATKIN_INSTALL_PYTHON_PRENAME = '\n                      '  # newline plus len('catkin_install_python(')


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
    if package.ros_version == 1:
        for cmd in package.cmake.content_map['catkin_package']:
            section = cmd.get_section('CATKIN_DEPENDS')
            if 'message_generation' in section.values:
                section.values.remove('message_generation')
                cmd.changed = True


@roscompile
def check_library_setup(package):
    if not package.cmake:
        return
    if package.build_type == 'catkin':
        package.cmake.section_check(package.cmake.get_libraries(), 'catkin_package', 'LIBRARIES')


def alphabetize_sections_helper(cmake):
    for content in cmake.contents:
        if isinstance(content, Command):
            for section in content.get_real_sections():
                if section.name in SHOULD_ALPHABETIZE:
                    sorted_values = sorted(section.values)
                    if sorted_values != section.values:
                        section.values = sorted_values
                        content.changed = True
        elif isinstance(content, CommandGroup):
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


@roscompile
def enforce_cmake_ordering(package, config=None):
    if not package.cmake:
        return
    if config is None:
        config = get_config()
    default_style = config.get('cmake_style')
    package.cmake.enforce_ordering(default_style)
