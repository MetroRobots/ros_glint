import os
import shutil

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


# Check if a path is a textfile
# via https://stackoverflow.com/a/7392391/999581
TEXT_CHARS = bytearray({7, 8, 9, 10, 12, 13, 27} | set(range(0x20, 0x100)) - {0x7f})


def is_binary_file(path):
    first_bytes = open(path, 'rb').read(1024)
    return bool(first_bytes.translate(None, TEXT_CHARS))


def copy_text_files(src_folder, dst_folder):
    if not os.path.exists(dst_folder):
        os.mkdir(dst_folder)
    for root, dirs, files in os.walk(src_folder):
        relative = os.path.relpath(root, src_folder)
        dst_sub = os.path.join(dst_folder, relative)
        if relative != '.':
            os.mkdir(dst_sub)
        for fn in files:
            original = os.path.join(root, fn)
            full_path = os.path.join(dst_sub, fn)
            if not is_binary_file(original):
                shutil.copyfile(original, full_path)
            else:
                # Write stub file
                with open(full_path, 'wb'):
                    pass
