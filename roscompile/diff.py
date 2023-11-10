import collections
import difflib
import os
import shutil
import tempfile
import filecmp

from ros_introspection.package import Package
from ros_introspection.package_structure import get_repo_root

from .terminal import color_diff, color_header
from .util import copy_text_files, REPO_FUNCTIONS


def listdir_maybe(path):
    """"Listdir if the path exists, otherwise empty list"""
    if os.path.exists(path):
        return os.listdir(path)
    else:
        return []


def get_diff_helper(original_folder, new_folder, subpath=''):
    """Finds the difference between two folders, starting at the subpath

    Annoyingly, filecmp/dircmp doesn't list the individual files in newly added folders,
    so it ended up being cleaner to just recurse on our own
    """
    D = collections.defaultdict(list)
    original_files = listdir_maybe(os.path.join(original_folder, subpath))
    new_files = listdir_maybe(os.path.join(new_folder, subpath))
    all_files = set(original_files).union(set(new_files))
    for fn in sorted(all_files):
        o_fn = os.path.join(original_folder, subpath, fn)
        n_fn = os.path.join(new_folder, subpath, fn)
        rel_fn = os.path.join(subpath, fn)
        if os.path.isdir(o_fn) or os.path.isdir(n_fn):
            if fn in filecmp.DEFAULT_IGNORES:
                continue
            for key, values in get_diff_helper(original_folder, new_folder, rel_fn).items():
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


def get_diff(original_folder, new_folder):
    return get_diff_helper(original_folder, new_folder)


def get_lines(folder, filename):
    return open(os.path.join(folder, filename)).readlines()


def print_diff(filename, left_folder=None, right_folder=None):
    """Print a colored diff for the given file between two different folders."""
    if left_folder is None:
        left = None
    else:
        left = get_lines(left_folder, filename)

    if right_folder is None:
        right = None
    else:
        right = get_lines(right_folder, filename)

    if not left and not right:
        if left_folder is None:
            diff = ['+++ b/' + filename + ' (new empty file)']
        else:
            diff = ['--- a/' + filename + ' (removed empty file)']
    else:
        diff = difflib.unified_diff(left or [], right or [], fromfile=filename, tofile='%s (modified)' % filename)
    print(''.join(color_diff(diff)))


def preview_changes(package, fn_name, fne, use_package_name=False):
    """
    Given a package and a single function, run the function on a copy of the package in a temp dir and display the diff.
    """
    try:
        temp_dir = tempfile.mkdtemp()
        new_package_root = os.path.join(temp_dir, package.name)
        if fn_name in REPO_FUNCTIONS:
            original_folder = get_repo_root(package)
            new_folder = temp_dir
            new_package_root = os.path.join(new_folder, os.path.relpath(package.root, original_folder))
            # TODO: Ideally the displayed filename would be relative to new_package_root
        else:
            original_folder = package.root
            new_folder = os.path.join(temp_dir, package.name)
            new_package_root = new_folder

        copy_text_files(original_folder, new_folder)
        new_pkg = Package(new_package_root)

        fne(new_pkg)
        new_pkg.write()
        the_diff = get_diff(original_folder, new_folder)
        if len(the_diff) == 0:
            return False

        if use_package_name:
            print(color_header(fn_name + ' (' + package.name + ')'))
        else:
            print(color_header(fn_name))

        for filename in the_diff.get('diff', []):
            print_diff(filename, original_folder, new_folder)
        for filename in the_diff.get('deleted', []):
            print_diff(filename, left_folder=original_folder)
        for filename in the_diff.get('added', []):
            print_diff(filename, right_folder=new_folder)
    finally:
        shutil.rmtree(temp_dir)
    return True


def files_match(pkg_in, pkg_out, filename, show_diff=True):
    """Return true if the contents of the given file are the same in each package. Otherwise maybe show the diff."""
    generated_contents = pkg_in.get_contents(filename).rstrip()
    canonical_contents = pkg_out.get_contents(filename).rstrip()
    ret = generated_contents == canonical_contents
    if show_diff and not ret:
        d = difflib.Differ()
        print('=' * 50 + filename)
        result = d.compare(generated_contents.split('\n'), canonical_contents.split('\n'))
        print('\n'.join(color_diff(result)))
    return ret
