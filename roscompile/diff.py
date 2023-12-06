import collections
import difflib
import os
import filecmp

from ros_introspection.package_structure import get_repo_root
from ros_glint.terminal import color_diff
from .util import REPO_FUNCTIONS


def listdir_maybe(path):
    """"Listdir if the path exists, otherwise empty list"""
    if path.exists():
        return sorted(path.iterdir())
    else:
        return []


def get_diff(original_folder, new_folder, subpath=''):
    """Finds the difference between two folders, starting at the subpath

    Annoyingly, filecmp/dircmp doesn't list the individual files in newly added folders,
    so it ended up being cleaner to just recurse on our own
    """
    D = collections.defaultdict(list)
    original_files = listdir_maybe(original_folder / subpath)
    new_files = listdir_maybe(new_folder / subpath)
    all_files = sorted(set(original_files).union(set(new_files)))
    for fn in all_files:
        o_fn = original_folder / subpath / fn
        n_fn = new_folder / subpath / fn
        rel_fn = subpath / fn
        if o_fn.is_dir() or n_fn.is_dir():
            if fn in filecmp.DEFAULT_IGNORES:
                continue
            for key, values in get_diff(original_folder, new_folder, rel_fn).items():
                D[key] += values
        elif fn in original_files:
            if fn in new_files:
                try:
                    if open(o_fn).read() != open(n_fn).read():
                        D['diff'].append(rel_fn)
                except UnicodeDecodeError:
                    # Probably a binary file, ignore
                    pass
            else:
                D['deleted'].append(rel_fn)
        else:
            D['added'].append(rel_fn)
    return dict(D)


def get_lines(path):
    return open(path).readlines()


def print_diff(filename, left_folder=None, right_folder=None):
    """Print a colored diff for the given file between two different folders."""
    if left_folder is None:
        left = None
    else:
        left = get_lines(left_folder / filename)

    if right_folder is None:
        right = None
    else:
        right = get_lines(right_folder / filename)

    if not left and not right:
        if left_folder is None:
            diff = [f'+++ b/{filename} (new empty file)']
        else:
            diff = [f'--- a/{filename} (removed empty file)']
    else:
        diff = difflib.unified_diff(left or [], right or [], fromfile=str(filename), tofile=f'{filename} (modified)')
    print(''.join(color_diff(diff)))


def preview_changes(package, fn_name):
    temp_dir = ''  # Fake
    if fn_name in REPO_FUNCTIONS:
        original_folder = get_repo_root(package)
        new_folder = temp_dir
        new_package_root = os.path.join(new_folder, os.path.relpath(package.root, original_folder))
        print(new_package_root)
        # TODO: Ideally the displayed filename would be relative to new_package_root
