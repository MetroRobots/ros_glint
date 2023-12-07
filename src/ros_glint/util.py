import os
import stat


def set_executable(fn, state):
    existing_permissions = stat.S_IMODE(os.lstat(fn).st_mode)
    if state:
        flags = stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH
    else:
        flags = ~stat.S_IXUSR | ~stat.S_IXGRP | ~stat.S_IXOTH
    os.chmod(fn, existing_permissions | flags)
