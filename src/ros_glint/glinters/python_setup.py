from stylish_cmake_parser import Command, SectionStyle
from ros_introspect.package import MiscPackageFile
from ros_introspect.components.setup_py import create_setup_py, contains_quoted_string, quote_string, unquote_string
from ros_introspect.components.setup_cfg import SetupCFG
from ros_introspect.components.source_code import SourceCode
from ..core import glinter
from ..cmake_ordering import insert_in_order
from ..python_types import get_python_usage, PythonUsage
from .cmake import section_check
from .cmake_installs import install_section_check, InstallType, check_cmake_python_buildtype
from ..util import set_executable
import re

CATKIN_INSTALL_PYTHON_PRENAME = '\n                      '  # newline plus len('catkin_install_python(')
MAIN_IMPORT_PATTERN = re.compile(r'from ([\w\.]+) import.*main.*')
ENTRY_POINT_SCRIPT_TEMPLATE = """#!/usr/bin/env python3

from {pkg}.{lib_name} import main
main()
"""


@glinter
def check_python_marker(package):
    if get_python_usage(package) == PythonUsage.NONE:
        return

    resource_folder = package.root / 'resource'
    marker_path = resource_folder / package.name
    if marker_path.exists():
        return
    resource_folder.mkdir(exist_ok=True)
    with open(marker_path, 'w') as f:
        f.write('')

    package.add_file(MiscPackageFile(marker_path, package))


@glinter
def generate_ament_cmake_python_entry_points(package):
    if package.build_type != 'ament_cmake':
        return
    entry_dict = get_entry_points(package)

    if not entry_dict:
        return
    execs = package.get_source_by_tags('pyscript', 'python')
    for exec in execs:
        ret = exec.search_lines_for_pattern(MAIN_IMPORT_PATTERN)
        if not ret:
            continue
        package_parts = ret[0][0].split('.')
        if package_parts[0] == package.name and package_parts[-1] in entry_dict:
            del entry_dict[package_parts[-1]]

    scripts_dir = package.root / 'scripts'
    scripts_dir.mkdir(exist_ok=True)

    for name, source in entry_dict.items():
        script_fn = scripts_dir / name
        if not script_fn.exists():
            with open(script_fn, 'w') as f:
                f.write(ENTRY_POINT_SCRIPT_TEMPLATE.format(
                    pkg=package.name,
                    lib_name=name,
                ))
            set_executable(script_fn, True)
            package.add_file(SourceCode(script_fn, package))


@glinter
def sync_package_xml_and_setup_py(package):
    if package.setup_py is None and package.build_type != 'ament_python':
        return

    if package.setup_py is None:
        create_setup_py(package)

    for tag in ['version', 'description', 'license']:
        tags = package.manifest.get_elements_by_tags([tag])
        if not tags:
            continue
        value = tags[0].childNodes[0].nodeValue
        package.setup_py.args[tag] = repr(value)

    for maintainer in package.manifest.get_elements_by_tags(['maintainer']):
        # TODO: Expand for author
        # TODO: Joint multiple people?
        package.setup_py.args['maintainer'] = repr(maintainer.childNodes[0].nodeValue)
        package.setup_py.args['maintainer_email'] = repr(maintainer.getAttribute('email'))

    package.setup_py.changed = True


def has_python_library(package_name, py_src):
    for source in py_src:
        parts = source.rel_fn.parts
        if len(parts) > 1 and parts[0] == 'src' and parts[1] == package_name:
            return True
    return False


@glinter
def check_setup_py(package):
    py_src = package.get_source_by_tags(set(), 'python')
    if not py_src and package.build_type not in ['ament_python', 'catkin']:
        return
    if package.setup_py is None:
        if not has_python_library(package.name, py_src) and package.build_type == 'catkin':
            # No library, and no existing setup_py means nothing to write
            return
        create_setup_py(package)

    if package.build_type == 'catkin':
        if 'catkin_python_setup' not in package.cmake.content_map:
            insert_in_order(package.cmake, Command('catkin_python_setup', package.cmake))
    else:
        package.setup_py.include_data_files(['package.xml'])

        # Determine packages/package_dir
        pkg_paths = [src.rel_fn.parent for src in package.get_source_by_tags('init.py')]
        if not pkg_paths:
            return
        package_dir = None
        if all(path.parts[0] == 'src' for path in pkg_paths):
            packages = [path.parts[1] for path in pkg_paths]
            package_dir = 'src'
        else:
            packages = [path.parts[0] for path in pkg_paths]

        if 'packages' not in package.setup_py.args:
            package.setup_py.args['packages'] = []

        for pkg in packages:
            if contains_quoted_string(package.setup_py.args['packages'], pkg):
                continue
            if package.setup_py.declare_package_name and pkg == package.name:
                value = 'package_name'
                if value in package.setup_py.args['packages']:
                    continue
            else:
                value = quote_string(pkg)
            package.setup_py.args['packages'].append(value)
            package.setup_py.changed = True

        if package_dir:
            # Do this after packages just for standard ordering
            if 'package_dir' not in package.setup_py.args:
                package.setup_py.args['package_dir'] = {}

            if not contains_quoted_string(package.setup_py.args['package_dir'].values(), package_dir):
                package.setup_py.args['package_dir'][quote_string('')] = quote_string(package_dir)
                package.setup_py.changed = True


def get_entry_points(package):
    if package.ros_version == 1:
        return {}

    entries = {}
    for source in package.get_source_by_tags({'entry_pt', 'pylib'}):
        entries[source.rel_fn.stem] = source
    return entries


@glinter
def update_python_installs(package):
    # Part 1: Library Setup for ament_cmake
    python_usage = get_python_usage(package)
    if python_usage != PythonUsage.NONE:
        check_cmake_python_buildtype(package)

        if python_usage == PythonUsage.MIXED_CMAKE_PYTHON:
            section_check(package.cmake, ['${PROJECT_NAME}'], 'ament_python_install_package')

    # Part 2: Executables
    execs = package.get_source_by_tags('pyscript', 'python')
    entry_dict = get_entry_points(package)

    if len(execs) == 0 and len(entry_dict) == 0:
        return

    exec_fns = sorted(str(exec.rel_fn) for exec in execs)

    if package.setup_py:
        package_dir = package.setup_py.args.get('package_dir')
        if package_dir:
            package_dir = unquote_string(list(package_dir.values())[0])
    else:
        package_dir = None

    entries = []
    for name, source in entry_dict.items():
        parts = list(source.rel_fn.parent.parts)
        if parts and parts[0] == package_dir:
            parts = parts[1:]
        parts.append(source.rel_fn.stem)
        py_namespace = '.'.join(parts)
        script = f'{name} = {py_namespace}:main'
        entries.append(script)

    if package.build_type == 'catkin':
        cmd = 'catkin_install_python'
        if cmd not in package.cmake.content_map:
            cmake_cmd = Command(cmd, parent=package.cmake)
            cmake_cmd.add_section('PROGRAMS', exec_fns)
            cmake_cmd.add_section('DESTINATION', ['${CATKIN_PACKAGE_BIN_DESTINATION}'],
                                  SectionStyle(CATKIN_INSTALL_PYTHON_PRENAME))
            insert_in_order(package.cmake, cmake_cmd)
        else:
            section_check(package.cmake, exec_fns, cmd, 'PROGRAMS')

    elif package.build_type == 'ament_python':
        if package.setup_cfg is None:
            package.add_file(SetupCFG(package.root / 'setup.cfg', package))

        scripts_folder = '$base/lib/' + package.name + '/scripts'
        package.setup_cfg.ensure('develop', 'script_dir', scripts_folder)
        package.setup_cfg.ensure('install', 'install_scripts', scripts_folder)

        if exec_fns:
            if 'scripts' not in package.setup_py.args:
                package.setup_py.args['scripts'] = []
                package.setup_py.changed = True
            for exec in exec_fns:
                if not contains_quoted_string(package.setup_py.args['scripts'], exec):
                    package.setup_py.args['scripts'].append(quote_string(exec))
                    package.setup_py.changed = True

        if entries:
            if 'entry_points' not in package.setup_py.args:
                package.setup_py.args['entry_points'] = {}

            entry_points = package.setup_py.args['entry_points']

            console_key = contains_quoted_string(entry_points, 'console_scripts')
            if console_key:
                console_scripts = entry_points[console_key]
            else:
                console_scripts = []
                entry_points[quote_string('console_scripts')] = console_scripts

            for entry in entries:
                if not contains_quoted_string(console_scripts, entry):
                    console_scripts.append(quote_string(entry))
                    package.setup_py.changed = True
    elif python_usage == PythonUsage.MIXED_CMAKE_PYTHON:
        install_section_check(package.cmake, exec_fns, InstallType.PYTHON, package.ros_version)
