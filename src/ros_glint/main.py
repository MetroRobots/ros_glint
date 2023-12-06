import argparse
import click
import pathlib
import shutil
import tempfile

from . import get_linters
from .terminal import color_header
from .diff import check_diff
from ros_introspect import find_packages, ROSResources, Package
from ros_introspect.finder import walk


# Check if a path is a textfile
# via https://stackoverflow.com/a/7392391/999581
TEXT_CHARS = bytearray({7, 8, 9, 10, 12, 13, 27} | set(range(0x20, 0x100)) - {0x7f})


def is_binary_file(path):
    first_bytes = open(path, 'rb').read(1024)
    return bool(first_bytes.translate(None, TEXT_CHARS))


def copy_text_files(src_folder, dst_folder):
    for subpath in walk(src_folder):
        src_path = src_folder / subpath
        dst_path = dst_folder / subpath
        dst_path.parent.mkdir(exist_ok=True, parents=True)

        if not is_binary_file(src_path):
            shutil.copyfile(src_path, dst_path)
        else:
            # Write stub file
            with open(dst_path, 'wb'):
                pass


def get_diff(pkg0, pkg1):
    keys0 = set(pkg0.components_by_name.keys())
    keys1 = set(pkg1.components_by_name.keys())
    diff = []
    for key in keys0.union(keys1):
        try:
            ret = check_diff(pkg0.root, pkg1.root, key)
        except UnicodeDecodeError:
            continue
        if ret:
            diff.append(ret)
    return diff


def preview_changes(package, fn_name, fne, use_package_name=False):
    """
    Given a package and a single function, run the function on a copy of the package in a temp dir and display the diff.
    """

    try:
        temp_dir = pathlib.Path(tempfile.mkdtemp())
        new_package_root = temp_dir / package.name

        copy_text_files(package.root, new_package_root)
        new_pkg = Package(new_package_root)

        fne(new_pkg)

        if not new_pkg.has_changes():
            return False

        new_pkg.save()

        the_diff = get_diff(package, new_pkg)
        if len(the_diff) == 0:
            return False

        if use_package_name:
            print(color_header(fn_name + ' (' + package.name + ')'))
        else:
            print(color_header(fn_name))

        for diff_string in the_diff:
            print(diff_string)

    finally:
        shutil.rmtree(temp_dir)
    return True


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
    parser.add_argument('-y', '--yes-to-all', action='store_true')
    parser.add_argument('-f', '--folder', type=pathlib.Path, default='.')
    parser.add_argument('-r', '--raise-errors', action='store_true', help='Devel only')

    args = parser.parse_args()

    pkgs = list(find_packages(args.folder))

    resources = ROSResources.get()
    resources.load_from_ros()

    for package in pkgs:
        for name, fne in linters.items():
            if args.linters and name not in args.linters:
                continue
            try:
                if args.yes_to_all:
                    fne(package)
                    package.save()
                    continue

                if preview_changes(package, name, fne, len(pkgs) > 1):
                    if click.confirm('Would you like to make this change?'):
                        fne(package)
                        package.write()
                    print('')

            except click.Abort:
                print('')
                exit(0)
            except Exception as e:
                click.secho(f'Exception occurred while running {name}: {e}', fg='red')
                if args.raise_errors:
                    raise
