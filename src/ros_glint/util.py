import os
import importlib_resources as resources
import stat


def set_executable(fn, state):
    existing_permissions = stat.S_IMODE(os.lstat(fn).st_mode)
    if state:
        flags = stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH
    else:
        flags = ~stat.S_IXUSR | ~stat.S_IXGRP | ~stat.S_IXOTH
    os.chmod(fn, existing_permissions | flags)


def get_data_file(name):
    return (resources.files('ros_glint.data') / name).read_text()


def get_ignore_data(name, variables, add_newline=True):
    def get_ignore_data_helper(basename, add_newline=True):
        lines = []
        for s in get_data_file(basename + '.ignore').split('\n'):
            if s == '':
                continue
            if add_newline:
                lines.append(s + '\n')
            else:
                lines.append(s)
        return lines

    ignore_lines = get_ignore_data_helper(name, add_newline)
    for pattern in get_ignore_data_helper(name + '_patterns', add_newline):
        ignore_lines.append(pattern % variables)
    return ignore_lines
