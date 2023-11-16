from ros_introspection.cmake import Command, CommandGroup
from clean_ros.cleaners.cmake import NEWLINE_PLUS_4, NEWLINE_PLUS_8
from clean_ros.cleaners.cmake import get_matching_add_depends, check_exported_dependencies
from clean_ros.cleaners.python_setup import CATKIN_INSTALL_PYTHON_PRENAME

from .util import get_config, roscompile

SHOULD_ALPHABETIZE = ['COMPONENTS', 'DEPENDENCIES', 'FILES', 'CATKIN_DEPENDS']


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
def check_generators(package):
    if package.ros_version == 1:
        for cmd in package.cmake.content_map['catkin_package']:
            section = cmd.get_section('CATKIN_DEPENDS')
            if 'message_generation' in section.values:
                section.values.remove('message_generation')
                cmd.changed = True


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
