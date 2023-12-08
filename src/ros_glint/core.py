import collections


_glinter_functions = collections.OrderedDict()


# Decorator function for gathering all the functions
def glinter(f):
    _glinter_functions[f.__name__] = f
    return f


def get_linters():
    return _glinter_functions
