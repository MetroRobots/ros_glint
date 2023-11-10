import datetime
import os
import re
import yaml

from ros_introspection.package_structure import get_repo_root
from ros_introspection.rviz_config import dictionary_subtract
from ros_introspection.ros_util import get_package_file
from ros_introspection.resource_list import get_license_info
from ros_introspection.util import get_sibling_packages, get_packages

from .util import roscompile, roscompile_repo
from .util import get_config, make_executable, get_license_key

MAINPAGE_S = r'/\*\*\s+\\mainpage\s+\\htmlinclude manifest.html\s+\\b %s\s+<!--\s+' + \
             r'Provide an overview of your package.\s+-->\s+-->\s+[^\*]*\*/'

RVIZ_CLASS_DEFAULTS = yaml.safe_load(open(get_package_file('roscompile', 'data/rviz_class_defaults.yaml')))
RVIZ_GENERIC_DEFAULTS = yaml.safe_load(open(get_package_file('roscompile', 'data/rviz_generic_defaults.yaml')))
RVIZ_GLOBAL_DEFAULTS = yaml.safe_load(open(get_package_file('roscompile', 'data/rviz_global_defaults.yaml')))
ROBOT_MODEL_LINK_DEFAULTS = {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True}
LICENSE_FILES = ['LICENSE', 'LICENSE.txt', 'UNLICENSE', 'UNLICENSE.txt', 'LICENSE.md']


@roscompile
def check_dynamic_reconfigure(package):
    cfgs = package.dynamic_reconfigs
    if len(cfgs) == 0:
        return
    pkg_list = {'dynamic_reconfigure'}
    package.manifest.add_packages(pkg_list, pkg_list)
    package.cmake.section_check(cfgs, 'generate_dynamic_reconfigure_options', '')
    package.cmake.section_check(pkg_list, 'find_package', 'COMPONENTS')

    for fn in cfgs:
        make_executable(os.path.join(package.root, fn))


@roscompile
def remove_useless_files(package):
    mainpage_pattern = re.compile(MAINPAGE_S % package.name)
    for fn in package.misc_files:
        if 'mainpage.dox' in fn:
            full_path = os.path.join(package.root, fn)
            s = open(full_path).read()
            if mainpage_pattern.match(s):
                os.remove(full_path)


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


@roscompile
def misc_xml_formatting(package):
    package.manifest.changed = True
    for config in package.plugin_configs:
        config.changed = True


@roscompile
def clean_up_rviz_configs(package):
    for rviz_config in package.rviz_configs:
        # print("\tCleaning up " + str(rviz_config))
        for config in rviz_config.get_class_dicts():
            if dictionary_subtract(config, RVIZ_GENERIC_DEFAULTS):
                rviz_config.changed = True

            full_class = config['Class']
            class_name = full_class.split('/')[-1]

            the_defaults = RVIZ_CLASS_DEFAULTS.get(full_class, {})
            the_defaults.update(RVIZ_CLASS_DEFAULTS.get(class_name, {}))
            if dictionary_subtract(config, the_defaults):
                rviz_config.changed = True

            # Special Case(s)
            if config.get('Topic') == '':
                del config['Topic']
                rviz_config.changed = True

            if full_class == 'rviz_default_plugins/RobotModel':
                for k, v in list(config.get('Links', {}).items()):
                    if not isinstance(v, dict):
                        continue
                    if dictionary_subtract(config['Links'][k], ROBOT_MODEL_LINK_DEFAULTS):
                        rviz_config.changed = True
                        if not config['Links'][k]:
                            del config['Links'][k]

            if full_class in ['rviz/Camera', 'rviz_default_plugins/Camera'] and 'Visibility' in config:
                visibility = config['Visibility']
                for key in list(visibility.keys()):
                    if visibility[key]:
                        rviz_config.changed = True
                        del visibility[key]
                if not visibility:
                    del config['Visibility']
        if dictionary_subtract(rviz_config.contents, RVIZ_GLOBAL_DEFAULTS):
            rviz_config.changed = True


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
