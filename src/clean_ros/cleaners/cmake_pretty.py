from stylish_cmake_parser import Command, CommandGroup
from ..core import clean_ros
from .cmake import NEWLINE_PLUS_2, NEWLINE_PLUS_4, NEWLINE_PLUS_8
from .python_setup import CATKIN_INSTALL_PYTHON_PRENAME

SHOULD_ALPHABETIZE = ['COMPONENTS', 'DEPENDENCIES', 'FILES', 'CATKIN_DEPENDS']


@clean_ros
def alphabetize_cmake_sections(package):
    if not package.cmake:
        return

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

    alphabetize_sections_helper(package.cmake)


@clean_ros
def prettify_catkin_package_cmd(package):
    if not package.cmake:
        return
    for cmd in package.cmake.content_map['catkin_package']:
        for section in cmd.get_real_sections():
            section.style.prename = NEWLINE_PLUS_4
        cmd.changed = True


@clean_ros
def prettify_package_lists(package):
    if not package.cmake:
        return
    acceptable_styles = [(NEWLINE_PLUS_8, NEWLINE_PLUS_8), (NEWLINE_PLUS_4, NEWLINE_PLUS_8)]

    for cmd_name, section_name in [('find_package', 'COMPONENTS'),
                                   ('catkin_package', 'CATKIN_DEPENDS')]:
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


@clean_ros
def prettify_msgs_srvs(package):
    if not package.cmake:
        return
    # ROS 1 version
    for cmd in package.cmake.content_map['add_message_files'] + package.cmake.content_map['add_service_files']:
        for section in cmd.get_real_sections():
            if len(section.values) > 1:
                section.style.name_val_sep = NEWLINE_PLUS_4
                section.style.val_sep = NEWLINE_PLUS_4
            cmd.changed = True
    # ROS 2 version
    for cmd in package.cmake.content_map['rosidl_generate_interfaces']:
        for section in cmd.get_real_sections():
            if section.name == '' and section == cmd.sections[0]:
                if section.style.val_sep != NEWLINE_PLUS_4:
                    section.style.val_sep = NEWLINE_PLUS_4
                    cmd.changed = True
            elif section.name == 'DEPENDENCIES':
                if section.style.prename != NEWLINE_PLUS_2:
                    section.style.prename = NEWLINE_PLUS_2
                    cmd.changed = True
                if section.style.val_sep != NEWLINE_PLUS_4:
                    section.style.val_sep = NEWLINE_PLUS_4
                    cmd.changed = True
                if section.style.name_val_sep != NEWLINE_PLUS_4:
                    section.style.name_val_sep = NEWLINE_PLUS_4
                    cmd.changed = True


@clean_ros
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
