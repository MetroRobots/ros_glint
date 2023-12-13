import collections
import os
import pathlib
import tempfile
import yaml
import zipfile

from ros_introspect.finder import walk
from ros_glint.util import set_executable


class ROSCompilePackageFiles:
    def __init__(self, package_name, pkg_files, executables):
        self.package_name = package_name
        self.is_written = False
        self.tempdir = None
        self.root = None
        self.pkg_files = pkg_files
        self.executables = executables

    def copy(self):
        return ROSCompilePackageFiles(self.package_name, self.pkg_files, self.executables)

    def __enter__(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.tempdir.name)
        self.write()
        self.is_written = True
        return self

    def __exit__(self, a_type, value, traceback):
        self.is_written = False
        self.root = None
        self.tempdir = None

    def get_filenames(self):
        if self.is_written:
            the_files = []
            for subpath in walk(self.root):
                the_files.append(subpath)
            return set(the_files)
        else:
            return set(self.pkg_files.keys())

    def get_contents(self, filename):
        if self.is_written:
            full_path = self.root / filename
            if full_path.exists():
                return open(full_path).read().replace('\r\n', '\n')
        elif filename in self.pkg_files:
            return self.pkg_files[filename].replace('\r\n', '\n')

    def write(self):
        for fn, contents in self.pkg_files.items():
            outfile = self.root / fn
            outfile.parent.mkdir(exist_ok=True, parents=True)
            with open(outfile, 'w') as f:
                f.write(contents)
            if fn in self.executables:
                set_executable(outfile, True)

    def __repr__(self):
        return self.package_name


def get_test_cases(zip_filename):
    file_data = collections.defaultdict(dict)
    zf = zipfile.ZipFile(zip_filename)
    config = None
    executables = set()
    for file in zf.filelist:
        if file.filename[-1] == '/':
            continue
        if file.filename == 'list_o_tests.yaml':
            config = yaml.safe_load(zf.read(file))
            continue
        parts = file.filename.split(os.path.sep)
        package = parts[0]
        path = pathlib.Path(os.path.join(*parts[1:]))
        file_data[package][path] = zf.read(file).decode()
        if (file.external_attr >> 16) & 0o111:
            executables.add(path)

    test_data = {}
    for package, pkg_data in file_data.items():
        test_data[package] = ROSCompilePackageFiles(package, pkg_data, executables)
    for pkg_data in config:
        if 'function' in pkg_data:
            pkg_data['functions'] = [pkg_data.pop('function')]

    return config, test_data
