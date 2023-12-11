import difflib
from .terminal import color_diff


def get_diff_string(contents0, contents1, filename):
    s = '=' * 5 + str(filename) + '=' * 45 + '\n'
    d = difflib.Differ()
    result = d.compare(contents0.split('\n'), contents1.split('\n'))
    s += '\n'.join(color_diff(result))
    return s
