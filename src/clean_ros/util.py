import os
import stat


def make_executable(fn):
    existing_permissions = stat.S_IMODE(os.lstat(fn).st_mode)
    os.chmod(fn, existing_permissions | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def make_not_executable(fn):
    existing_permissions = stat.S_IMODE(os.lstat(fn).st_mode)
    os.chmod(fn, existing_permissions | ~stat.S_IXUSR | ~stat.S_IXGRP | ~stat.S_IXOTH)
