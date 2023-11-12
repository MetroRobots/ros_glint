import os
import stat
from .core import root


def make_executable(fn):
    existing_permissions = stat.S_IMODE(os.lstat(fn).st_mode)
    os.chmod(fn, existing_permissions | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def make_not_executable(fn):
    existing_permissions = stat.S_IMODE(os.lstat(fn).st_mode)
    os.chmod(fn, existing_permissions | ~stat.S_IXUSR | ~stat.S_IXGRP | ~stat.S_IXOTH)


def get_ignore_data(name, variables=None, add_newline=True):
    def get_ignore_data_helper(basename, add_newline=True):
        fn = root / 'data' / (basename + '.ignore')
        lines = []
        if not fn.exists():
            print(f"can't find {fn}")
            return lines
        for s in open(fn):
            if s == '\n':
                continue
            if add_newline:
                lines.append(s)
            else:
                lines.append(s[:-1])
        return lines

    ignore_lines = get_ignore_data_helper(name, add_newline)
    if not variables:
        return ignore_lines
    for pattern in get_ignore_data_helper(name + '_patterns', add_newline):
        ignore_lines.append(pattern % variables)
    return ignore_lines
