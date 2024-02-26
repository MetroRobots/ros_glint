import argparse
import click
import pathlib
import shutil
import tempfile

from . import get_linters
from .terminal import color_header
from .diff import get_diff_string
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


def read_or_empty(filepath):
    """Read an existing file or return an empty string"""
    if filepath.exists():
        try:
            return open(filepath).read()
        except UnicodeDecodeError:
            return ''
    else:
        return ''


def get_diff(pkg0, pkg1):
    names0 = set(pkg0.components_by_name.keys())
    names1 = set(pkg1.components_by_name.keys())
    diff = []
    for rel_filename in names0.union(names1):
        contents0 = read_or_empty(pkg0.root / rel_filename)
        contents1 = read_or_empty(pkg1.root / rel_filename)

        if contents0 == contents1:
            continue

        diff.append(get_diff_string(contents0, contents1, rel_filename))

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
    parser.add_argument('linters', nargs='*', action=ValidateCleaner, metavar='linter', default=[],
                        help='By default, run all linters. If any are specified here, only those specified are run.')
    parser.add_argument('-f', '--folder', type=pathlib.Path, default='.',
                        help='The folder to search for ROS packages in. Defaults to the current directory.')
    parser.add_argument('-y', '--yes-to-all', action='store_true',
                        help='Non-interactive mode that accepts all suggestions.')
    parser.add_argument('-s', '--skip-ros-load', action='store_true',
                        help='Avoid loading ROS resources, useful in scripting environments.')

    args = parser.parse_args()

    pkgs = list(find_packages(args.folder))

    resources = ROSResources.get()

    if not args.skip_ros_load:
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
                raise
