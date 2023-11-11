from ..core import clean_ros


@clean_ros
def remove_empty_export_tag(package):

    def has_element_child(node):
        for child in node.childNodes:
            if child.nodeType == child.ELEMENT_NODE:
                return True
        return False

    manifest = package.package_xml
    exports = manifest.root.getElementsByTagName('export')
    if len(exports) == 0:
        return False
    for export in exports:
        if not has_element_child(export):
            manifest.remove_element(export)
            return True
