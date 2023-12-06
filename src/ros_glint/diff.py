import difflib
from .terminal import color_diff


def get_diff_string(contents0, contents1, filename):
    s = '=' * 5 + str(filename) + '=' * 45 + '\n'
    d = difflib.Differ()
    result = d.compare(contents0.split('\n'), contents1.split('\n'))
    s += '\n'.join(color_diff(result))
    return s


def check_diff(root0, root1, filename):
    path0 = root0 / filename
    path1 = root1 / filename
    if path0.exists():
        contents0 = open(path0).read()
    else:
        contents0 = ''

    if path1.exists():
        contents1 = open(path1).read()
    else:
        contents1 = ''

    if contents0 == contents1:
        return
    return get_diff_string(contents0, contents1, filename)
