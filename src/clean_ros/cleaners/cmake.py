import re

from ..core import clean_ros
from ..util import get_ignore_data

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


def remove_empty_cmake_lines(cmake):
    for i, content in enumerate(cmake.contents[:-2]):
        if str(content)[-1] == '\n' and cmake.contents[i + 1] == '\n' and cmake.contents[i + 2] == '\n':
            cmake.contents[i + 1] = ''
    cmake.contents = remove_empty_strings(cmake.contents)


@clean_ros
def remove_boilerplate_cmake_comments(package):
    if not package.cmake:
        return

    ignorables = get_ignore_data('cmake', {'package': package.name})
    if remove_cmake_comments_helper(package.cmake.contents, ignorables):
        package.cmake.changed = True
    remove_empty_cmake_lines(package.cmake.contents)
