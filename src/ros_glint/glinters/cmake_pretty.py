from stylish_cmake_parser import Command, CommandGroup
from ..core import glinter
from .cmake import NEWLINE_PLUS_2, NEWLINE_PLUS_4, NEWLINE_PLUS_8
from .python_setup import CATKIN_INSTALL_PYTHON_PRENAME

SHOULD_ALPHABETIZE = ['COMPONENTS', 'DEPENDENCIES', 'FILES', 'CATKIN_DEPENDS']


def set_style_attribute(section, attribute, new_value):
    existing_value = getattr(section.style, attribute)
    if existing_value == new_value:
        return
    setattr(section.style, attribute, new_value)
    section.mark_changed()


@glinter
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
                            content.mark_changed()
            elif isinstance(content, CommandGroup):
                alphabetize_sections_helper(content.contents)

    alphabetize_sections_helper(package.cmake)


@glinter
def prettify_catkin_package_cmd(package):
    if not package.cmake:
        return
    for cmd in package.cmake.content_map['catkin_package']:
        for section in cmd.get_real_sections():
            set_style_attribute(section, 'prename', NEWLINE_PLUS_4)


@glinter
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
                        section.mark_changed()


@glinter
def prettify_msgs_srvs(package):
    if not package.cmake:
        return
    # ROS 1 version
    for cmd in package.cmake.content_map['add_message_files'] + package.cmake.content_map['add_service_files']:
        for section in cmd.get_real_sections():
            if len(section.values) > 1:
                set_style_attribute(section, 'name_val_sep', NEWLINE_PLUS_4)
                set_style_attribute(section, 'val_sep', NEWLINE_PLUS_4)
    # ROS 2 version
    for cmd in package.cmake.content_map['rosidl_generate_interfaces']:
        for section in cmd.get_real_sections():
            if section.name == '' and section == cmd.sections[0]:
                set_style_attribute(section, 'val_sep', NEWLINE_PLUS_4)
            elif section.name == 'DEPENDENCIES':
                set_style_attribute(section, 'prename', NEWLINE_PLUS_2)
                set_style_attribute(section, 'val_sep', NEWLINE_PLUS_4)
                set_style_attribute(section, 'name_val_sep', NEWLINE_PLUS_4)


@glinter
def prettify_installs(package):
    if not package.cmake:
        return
    for cmd in package.cmake.content_map['install']:
        cmd.sections = [s for s in cmd.sections if not isinstance(s, str)]
        if not cmd.sections:
            continue

        if '\n' in cmd.sections[0].style.prename:
            new_style = cmd.sections[0].style.prename
        else:
            new_style = NEWLINE_PLUS_8

        zeroed = False
        for section in cmd.sections[1:]:
            if len(section.values) == 0:
                set_style_attribute(section, 'prename', new_style)
                zeroed = True
            elif not zeroed:
                set_style_attribute(section, 'prename', new_style)
            else:
                set_style_attribute(section, 'prename', '')

    for cmd in package.cmake.content_map['catkin_install_python']:
        section = cmd.sections[1]
        set_style_attribute(section, 'prename', CATKIN_INSTALL_PYTHON_PRENAME)


@glinter
def prettify_command_groups(package):
    if not package.cmake:
        return

    def pretty_groups_helper(cmake, indent=0):
        for group in cmake.content_map['group']:
            pretty_groups_helper(group.contents, indent + 2)

        if indent == 0:
            return
        for i, content in enumerate(cmake.contents):
            if isinstance(content, Command) and i > 0 and isinstance(cmake.contents[i-1], str):
                prev = cmake.contents[i-1]
                while '\n ' in prev:
                    prev = prev.replace('\n ', '\n')
                revised = prev.replace('\n', '\n' + (' ' * indent))
                if prev != revised:
                    cmake.contents[i - 1] = revised
                    cmake.mark_changed()

    pretty_groups_helper(package.cmake)
