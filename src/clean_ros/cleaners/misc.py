from ..core import clean_ros


@clean_ros
def misc_xml_formatting(package):
    package.package_xml.changed = True
    for config in package.plugin_xml:
        config.changed = True
