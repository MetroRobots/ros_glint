import collections
import pathlib


_linter_functions = collections.OrderedDict()

root = pathlib.Path(__file__).parent.parent.parent


# Decorator function for gathering all the functions
def glinter(f):
    _linter_functions[f.__name__] = f
    return f


def get_functions():
    return _linter_functions
