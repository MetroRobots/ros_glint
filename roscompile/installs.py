import collections

from ros_introspection.setup_py import create_setup_py

from .util import roscompile

FILES_TO_NOT_INSTALL = ['CHANGELOG.rst', 'README.md', '.travis.yml', 'bitbucket-pipelines.yml', 'setup.cfg',
                        '.pre-commit-config.yaml', 'LICENSE', 'LICENSE.txt']


@roscompile
def update_misc_installs(package):
    extra_files_by_folder = collections.defaultdict(list)
    if not package.cmake:
        if package.setup_py is None:
            create_setup_py(package)

        for folder, files in sorted(extra_files_by_folder.items()):
            package.setup_py.include_data_files(files, folder)


@roscompile
def fix_double_directory_installs(package):
    if not package.cmake:
        return
    for cmd in package.cmake.content_map['install']:
        dir_section = cmd.get_section('DIRECTORY')
        dest_sections = cmd.get_sections('DESTINATION')

        if not dir_section or not dest_sections:
            continue
        directory = dir_section.values[0]
        final_slash = directory[-1] == '/'

        for section in dest_sections:
            destination = section.values[0]
            if not final_slash and destination.endswith(directory):
                # Remove double directory and final slash
                section.values[0] = destination[:-len(directory) - 1]
                cmd.changed = True
