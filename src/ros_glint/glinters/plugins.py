import collections
import re

from ros_introspect.components.plugin_xml import PluginXML

from ..core import glinter
from .cmake import install_cmake_dependencies, section_check

PLUGIN_PATTERN = r'PLUGINLIB_EXPORT_CLASS\(([^:]+)::([^,]+),\s*([^:]+)::([^,]+)\)'
PLUGIN_RE = re.compile(PLUGIN_PATTERN)


def get_plugin_xmls(package_xml):
    """Return a mapping from the package name to a list of the relative path(s) for the plugin xml(s)."""
    xmls = collections.defaultdict(list)
    export = package_xml.root.getElementsByTagName('export')
    for export_tag in export:
        for node in export_tag.childNodes:
            if node.nodeType == package_xml.root.ELEMENT_NODE:
                plugin = node.getAttribute('plugin').replace('${prefix}/', '')
                xmls[node.nodeName].append(plugin)
    return xmls


def add_to_package_xml(package_xml, plugin_xml, package_name):
    export_tags = package_xml.root.getElementsByTagName('export')
    if not export_tags:
        # Create export tag
        export_tag = package_xml.create_new_tag('export')
    else:
        export_tag = export_tags[0]

    attr = '${prefix}/' + str(plugin_xml.rel_fn)
    for tag in export_tag.childNodes:
        if tag.nodeName != package_name:
            continue
        plugin = tag.attributes.get('plugin')
        if plugin and plugin.value == attr:
            # No change needed
            return

    # Plugin not found
    pe = package_xml.tree.createElement(package_name)
    pe.setAttribute('plugin', attr)

    if not export_tag.childNodes:
        export_tag.appendChild(package_xml.tree.createTextNode('\n    '))
        export_tag.appendChild(pe)
        export_tag.appendChild(package_xml.tree.createTextNode('\n  '))
    else:
        all_elements = []
        all_elements.append(package_xml.create_new_tab_element(2))
        all_elements.append(pe)
        export_tag.childNodes = export_tag.childNodes[:-1] + all_elements + export_tag.childNodes[-1:]
    package_xml.changed = True


@glinter
def check_plugins(package):
    """Check that all the plugins are properly defined.

    We have three dictionaries
      * The plugins that are defined by macros (defined_macros)
      * The plugins that have associated configuration files (existing_plugins)
      * The plugins that are linked by the manifest. (plugin_xml_by_package)
    First, we reconcile the macros with the files.
    Then we handle the manifest.
    Then we make sure that the specific classes are in the configurations
    """
    if not package.cmake:
        return

    existing_plugins = collections.defaultdict(list)
    for xml in package.plugin_xml:
        for parent_pkg in xml.parent_pkgs:
            existing_plugins[parent_pkg].append(xml)

    defined_plugins = get_plugin_xmls(package.package_xml)
    build_rules = package.cmake.get_build_rules('add_library', resolve_target_name=True)

    def contains_library(xmls, library, pkg, name):
        for xml in xmls:
            if xml.contains_library(library, pkg, name):
                return True
        return False

    def lookup_library(rel_fn):
        for library, deps in build_rules.items():
            if rel_fn in deps:
                return library

    for source_file in package.source_code:
        plugin_info = source_file.search_lines_for_pattern(PLUGIN_RE)
        if not plugin_info:
            continue
        library = lookup_library(str(source_file.rel_fn))
        if library is None:
            continue
        for pkg1, name1, pkg2, name2 in plugin_info:
            # Create file if needed
            if pkg2 not in existing_plugins:
                xml_filename = f'{pkg2}_plugins.xml'
                full_path = package.root / xml_filename
                p_xml = PluginXML(full_path, package)
                package.plugin_xml.append(p_xml)
                existing_plugins[pkg2] = [p_xml]

            # Make sure plugins are properly exported
            for plugin_xml in existing_plugins[pkg2]:
                if package.ros_version == 1:
                    if plugin_xml.rel_fn not in defined_plugins[pkg2]:
                        add_to_package_xml(package.package_xml, plugin_xml, pkg2)
                else:
                    section_check(package.cmake, [pkg2, str(plugin_xml.rel_fn)],
                                  'pluginlib_export_plugin_description_file',
                                  '')

            # Make sure the class is in the files
            if not contains_library(existing_plugins[pkg2], library, pkg1, name1):
                # insert into first
                xml = existing_plugins[pkg2][0]
                xml.insert_new_class(library, pkg1, name1, pkg2, name2)

        if package.ros_version > 1:
            install_cmake_dependencies(package, {'ament_cmake_ros'})
