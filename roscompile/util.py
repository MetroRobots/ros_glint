from ros_introspection.ros_util import get_package_path, get_package_file
from ros_introspection.resource_list import get_available_licenses, get_license_info

import yaml

PKG_PATH = get_package_path('roscompile')
LICENSE_TRANSLATION = yaml.safe_load(open(get_package_file('roscompile', 'data/license_translation.yaml')))

REPO_FUNCTIONS = set()


# Decorator function for gathering subset of functions that require the whole repo
def roscompile_repo(f):
    REPO_FUNCTIONS.add(f.__name__)
    return f


def get_license_key(license_name):
    if license_name in LICENSE_TRANSLATION:
        license_name = LICENSE_TRANSLATION[license_name]

    if license_name is None:
        return None

    search_key = license_name.lower()

    for license_key in get_available_licenses():
        info = get_license_info(license_key)
        if not info:
            continue
        found_name = info['name'].lower()
        if search_key in found_name or found_name in search_key:
            return license_key

    return license_name.lower().replace(' ', '-')
