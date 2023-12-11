import argparse
import pathlib
from . import get_linters
from ros_introspect import find_packages


def main():
    linters = get_linters()

    class ValidateCleaner(argparse.Action):
        def __call__(self, parser, args, values, option_string=None):
            for value in values:
                if value not in linters:
                    raise ValueError(f'{value} not a valid linter!')
            args.linters = values

    parser = argparse.ArgumentParser()
    parser.add_argument('linters', nargs='*', action=ValidateCleaner, metavar='linter', default=[])
    parser.add_argument('-f', '--folder', type=pathlib.Path, default='.')

    args = parser.parse_args()

    pkgs = list(find_packages(args.folder))

    ret = 0

    for package in pkgs:
        for name, fne in linters.items():
            if args.linters and name not in args.linters:
                continue
            fne(package)
        if package.has_changes():
            package.save()
            ret = -1

    return ret
