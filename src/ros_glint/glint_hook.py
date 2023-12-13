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
    parser.add_argument('-f', '--related_files', type=pathlib.Path, nargs='*')

    args = parser.parse_args()
    pkgs = {}

    for related_file in args.related_files:
        for pkg in find_packages(related_file.parent):
            if pkg.root not in pkgs:
                pkgs[pkg.root] = pkg

    ret = 0

    for _, package in sorted(pkgs.items()):
        for name, fne in linters.items():
            if args.linters and name not in args.linters:
                continue
            fne(package)
        if package.has_changes():
            package.save()
            ret = -1

    return ret
