import os
import os.path
import re

from ros_introspection.setup_py import create_setup_py
from clean_ros.cleaners.python_setup import get_entry_points
from .util import roscompile, make_executable

MAIN_IMPORT_PATTERN = re.compile(r'from ([\w\.]+) import.*main.*')
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
