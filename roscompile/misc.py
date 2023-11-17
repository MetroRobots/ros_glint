import datetime
import os

from ros_introspection.package_structure import get_repo_root
from ros_introspection.resource_list import get_license_info
from ros_introspection.util import get_sibling_packages, get_packages

from .util import roscompile_repo
from .util import get_config, get_license_key

LICENSE_FILES = ['LICENSE', 'LICENSE.txt', 'UNLICENSE', 'UNLICENSE.txt', 'LICENSE.md']


@roscompile_repo
def update_metapackage(package, require_matching_name=False):
    # Check if there is indication in package.xml or CMake of being a metapackage
    if not package.is_metapackage():
        return False

    if require_matching_name:
        parent_path = os.path.abspath(os.path.join(package.root, '..'))

        if os.path.split(parent_path)[1] != package.name:
            return False

    sibling_packages = get_sibling_packages(package)

    existing_sub_packages = package.manifest.get_packages('run')
    package.manifest.add_packages(set(), sibling_packages, prefer_depend_tag=False)

    if package.manifest.format == 1:
        pkg_type = 'run_depend'
    else:
        pkg_type = 'exec_depend'

    package.manifest.remove_dependencies(pkg_type, existing_sub_packages - sibling_packages)

    # Ensure proper commands in CMake
    package.cmake.section_check([], 'catkin_metapackage', zero_okay=True)

    # Ensure proper tags in package.xml
    if not package.manifest.is_metapackage():
        export_tag = package.manifest.get_export_tag()
        meta_tag = package.manifest.tree.createElement('metapackage')
        package.manifest.insert_new_tag_inside_another(export_tag, meta_tag)


def get_license_path(folder):
    for file in LICENSE_FILES:
        full = os.path.join(folder, file)
        if os.path.exists(full):
            return full
    return os.path.join(folder, LICENSE_FILES[0])


@roscompile_repo
def check_license_file(package, config=None):
    if config is None:
        config = get_config()

    repo_root = get_repo_root(package)
    all_pkg_licenses = {pkg.manifest.get_license() for pkg in get_packages(repo_root)}

    license_path = get_license_path(repo_root)
    if os.path.exists(license_path):
        license_text = open(license_path).read()
    else:
        license_text = ''

    for license_name in all_pkg_licenses:
        license_key = get_license_key(license_name)
        license_info = get_license_info(license_key, get_body=True)
        template = license_info['body']
        first_line = template[:template.index('\n')]
        if first_line in license_text:
            # License text already present
            continue

        if license_text:
            # If already contains something, add some newlines
            license_text += '\n\n'

        contents = template
        variables = {}
        if 'year' in config:
            variables['year'] = config['year']
        else:
            variables['year'] = datetime.datetime.now().year

        if 'copyright_holder' in config:
            variables['fullname'] = config['copyright_holder']

        for key, value in variables.items():
            contents = contents.replace('[' + key + ']', str(value))
        license_text += contents

    with open(license_path, 'w') as f:
        f.write(license_text)
