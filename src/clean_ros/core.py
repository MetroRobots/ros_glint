import collections
import pathlib


_cleaner_functions = collections.OrderedDict()

root = pathlib.Path(__file__).parent.parent.parent


# Decorator function for gathering all the functions
def clean_ros(f):
    _cleaner_functions[f.__name__] = f
    return f


def get_functions():
    return _cleaner_functions
