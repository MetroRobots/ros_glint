import os
import os.path
import re

from ros_introspection.cmake import Command, SectionStyle
from ros_introspection.setup_cfg import SetupCFG
from ros_introspection.setup_py import create_setup_py, contains_quoted_string, quote_string, unquote_string
from ros_introspection.source_code_file import INIT_PY

from .cmake import CATKIN_INSTALL_PYTHON_PRENAME, check_cmake_dependencies_helper
from .installs import install_section_check
from .util import roscompile, make_executable

MAIN_IMPORT_PATTERN = re.compile(r'from ([\w\.]+) import.*main.*')


def has_python(package):
    return len(package.source_code.get_source_by_language('python')) > 0


def has_python_library(package):
    key = 'src/%s' % package.name
    for source in package.source_code.get_source_by_language('python'):
        if key in source.rel_fn:
            return True
    return False


@roscompile
def check_setup_py(package):
    if package.build_type == 'ament_cmake':
        return
    if not has_python(package) and package.build_type not in ['ament_python', 'catkin']:
        return
    if package.setup_py is None:
        if not has_python_library(package) and package.build_type == 'catkin':
            # No library, and no existing setup_py means nothing to write
            return
        create_setup_py(package)

    if package.build_type == 'catkin':
        if 'catkin_python_setup' not in package.cmake.content_map:
            package.cmake.add_command(Command('catkin_python_setup'))
    elif package.build_type == 'ament_python':
        package.setup_py.include_data_files(['package.xml'])

        # Determine packages/package_dir
        packages = [os.path.dirname(rel_fn) for rel_fn in package.source_code.sources.keys() if INIT_PY in rel_fn]
        if not packages:
            return
        package_dir = None
        if all(pkg.startswith('src') for pkg in packages):
            packages = [pkg.replace('src/', '') for pkg in packages]
            package_dir = 'src'

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


@roscompile
def check_python_marker(package):
    if package.build_type != 'ament_python':
        return
    marker = 'resource/' + package.name
    if marker not in package.misc_files:
        resource_folder = os.path.join(package.root, 'resource')
        if not os.path.exists(resource_folder):
            os.mkdir(resource_folder)
        with open(os.path.join(resource_folder, package.name), 'w') as f:
            f.write('')
        package.misc_files.append(marker)


def get_entry_points(package):
    if package.ros_version == 1:
        return {}

    entries = {}
    for source in package.source_code.get_source_by_tags({'entry_pt', 'pylib'}):
        # Equivalent of pathlib.Path.stem
        stem = os.path.splitext(os.path.basename(source.rel_fn))[0]
        entries[stem] = source
    return entries


ENTRY_POINT_SCRIPT_TEMPLATE = """#!/usr/bin/env python3

from {pkg}.{lib_name} import main
main()
"""


@roscompile
def generate_ament_cmake_python_entry_points(package):
    if package.build_type != 'ament_cmake':
        return
    entry_dict = get_entry_points(package)

    if not entry_dict:
        return
    execs = package.source_code.get_source_by_tags('pyscript', 'python')
    for exec in execs:
        ret = exec.search_lines_for_pattern(MAIN_IMPORT_PATTERN)
        if not ret:
            continue
        package_parts = ret[0][0].split('.')
        if package_parts[0] == package.name and package_parts[-1] in entry_dict:
            del entry_dict[package_parts[-1]]

    scripts_dir = os.path.join(package.root, 'scripts')
    if not os.path.exists(scripts_dir):
        os.mkdir(scripts_dir)

    for name, source in entry_dict.items():
        script_fn = os.path.join(scripts_dir, name)
        if not os.path.exists(script_fn):
            rel_fn = os.path.join('scripts', name)
            contents = ENTRY_POINT_SCRIPT_TEMPLATE.format(
                pkg=package.name,
                lib_name=name,
            )
            scf = package.source_code.add_source_code(package.root, rel_fn, contents)
            make_executable(script_fn)
            scf.tags.add('pyscript')


@roscompile
def update_python_installs(package):
    # Part 1: Library Setup for ament_cmake
    if package.build_type == 'ament_cmake' and package.source_code.get_source_by_tags('pylib'):
        acp = 'ament_cmake_python'
        build_tools = package.manifest.get_packages_by_tag('buildtool_depend')
        if acp not in build_tools:
            package.manifest.insert_new_packages('buildtool_depend', [acp])

        check_cmake_dependencies_helper(package, {acp})

        package.cmake.section_check(['${PROJECT_NAME}'], 'ament_python_install_package')

    # Part 2: Executables
    execs = package.source_code.get_source_by_tags('pyscript', 'python')
    entry_dict = get_entry_points(package)

    if len(execs) == 0 and len(entry_dict) == 0:
        return

    exec_fns = sorted(exec.rel_fn for exec in execs)

    if package.setup_py:
        package_dir = package.setup_py.args.get('package_dir')
        if package_dir:
            package_dir = unquote_string(list(package_dir.values())[0])
    else:
        package_dir = None

    entries = []
    for name, source in entry_dict.items():
        path = os.path.splitext(source.rel_fn[:-3])[0]
        parts = path.split('/')
        if parts and parts[0] == package_dir:
            parts = parts[1:]
        py_namespace = '.'.join(parts)
        script = f'{name} = {py_namespace}:main'
        entries.append(script)

    if package.build_type == 'catkin':
        cmd = 'catkin_install_python'
        if cmd not in package.cmake.content_map:
            cmake_cmd = Command(cmd)
            cmake_cmd.add_section('PROGRAMS', exec_fns)
            cmake_cmd.add_section('DESTINATION', ['${CATKIN_PACKAGE_BIN_DESTINATION}'],
                                  SectionStyle(CATKIN_INSTALL_PYTHON_PRENAME))
            package.cmake.add_command(cmake_cmd)
        else:
            package.cmake.section_check(exec_fns, cmd, 'PROGRAMS')

    elif package.build_type == 'ament_python':
        if package.setup_py is None:
            create_setup_py(package)

        if package.setup_cfg is None:
            package.setup_cfg = SetupCFG(package.root + '/setup.cfg')

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
    elif package.build_type == 'ament_cmake':
        install_section_check(package.cmake, exec_fns, 'python', catkin=package.build_type == 'catkin')


@roscompile
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
