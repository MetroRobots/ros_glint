import os
import re
import stat
from .core import root

TRAILING_PATTERN = re.compile(r'^(.*[^\w])\w+\n$')


def set_executable(fn, state):
    existing_permissions = stat.S_IMODE(os.lstat(fn).st_mode)
    if state:
        flags = stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH
    else:
        flags = ~stat.S_IXUSR | ~stat.S_IXGRP | ~stat.S_IXOTH
    os.chmod(fn, existing_permissions | flags)


def get_ignore_data(name, variables, add_newline=True):
    def get_ignore_data_helper(basename, add_newline=True):
        fn = root / 'data' / (basename + '.ignore')
        lines = []
        for s in open(fn):
            if s == '\n':
                continue
            if add_newline:
                lines.append(s)
            else:
                lines.append(s[:-1])
        return lines

    ignore_lines = get_ignore_data_helper(name, add_newline)
    for pattern in get_ignore_data_helper(name + '_patterns', add_newline):
        ignore_lines.append(pattern % variables)
    return ignore_lines
