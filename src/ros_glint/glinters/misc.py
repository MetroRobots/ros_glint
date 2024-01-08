from ..core import glinter
from ..util import set_executable
from .cmake import section_check
from ros_introspect.package import DependencyType
import re

MAINPAGE_S = r'/\*\*\s+\\mainpage\s+\\htmlinclude manifest.html\s+\\b %s\s+<!--\s+' + \
             r'Provide an overview of your package.\s+-->\s+-->\s+[^\*]*\*/'


@glinter
def check_dynamic_reconfigure(package):
    if not package.dynamic_reconfig:
        return
    pkg_list = {'dynamic_reconfigure'}

    package.package_xml.add_dependencies({DependencyType.BUILD: pkg_list, DependencyType.RUN: pkg_list})

    cmake = package.cmake
    section_check(cmake, [str(cfg.rel_fn) for cfg in package.dynamic_reconfig],
                  'generate_dynamic_reconfigure_options', '')
    section_check(cmake, pkg_list, 'find_package', 'COMPONENTS')

    for cfg in package.dynamic_reconfig:
        set_executable(cfg.full_path, True)


@glinter
def remove_useless_files(package):
    mainpage_pattern = re.compile(MAINPAGE_S % package.name)
    for doc in package.documentation[:]:
        if doc.full_path.name != 'mainpage.dox':
            continue

        s = open(doc.full_path).read()
        if mainpage_pattern.match(s):
            doc.full_path.unlink()
            package.documentation.remove(doc)


@glinter
def misc_xml_formatting(package):
    package.package_xml.force_regeneration()
    for config in package.plugin_xml:
        config.force_regeneration()
