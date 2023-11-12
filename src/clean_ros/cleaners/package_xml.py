from ..core import clean_ros
from ..util import get_ignore_data
from ros_introspect.components.package_xml import count_trailing_spaces


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


@clean_ros
def enforce_manifest_tabbing(package):
    def enforce_tabbing_helper(manifest, node, tabs=1):
        ideal_length = manifest.std_tab * tabs
        prev_was_node = True
        insert_before_list = []
        if not node:
            return
        changed = False
        for c in node.childNodes:
            if c.nodeType == c.TEXT_NODE:
                prev_was_node = False
                if c == node.childNodes[-1]:
                    continue
                spaces = count_trailing_spaces(c.data)
                if spaces > ideal_length:
                    c.data = c.data[: ideal_length - spaces]
                    changed = True
                elif spaces < ideal_length:
                    c.data = c.data + ' ' * (ideal_length - spaces)
                    changed = True
                if '\n' not in c.data:
                    c.data = '\n' + c.data
                    changed = True
            else:
                if prev_was_node:
                    changed = True
                    insert_before_list.append(c)

                prev_was_node = True
                enforce_tabbing_helper(manifest, c, tabs + 1)

        for c in insert_before_list:
            node.insertBefore(manifest.get_tab_element(tabs), c)

        manifest.changed = manifest.changed or changed

        if len(node.childNodes) == 0:
            return
        last = node.childNodes[-1]
        if last.nodeType != last.TEXT_NODE:
            node.appendChild(manifest.get_tab_element(tabs - 1))
            manifest.changed = True

    enforce_tabbing_helper(package.package_xml, package.package_xml.root)


@clean_ros
def remove_empty_manifest_lines(package):
    def remove_empty_lines_helper(node):
        changed = False
        for child in node.childNodes:
            if child.nodeType == child.TEXT_NODE:
                while '\n\n\n' in child.data:
                    child.data = child.data.replace('\n\n\n', '\n\n')
                    changed = True
            else:
                changed = remove_empty_lines_helper(child) or changed
        return changed

    if remove_empty_lines_helper(package.package_xml.root):
        package.package_xml.changed = True


def cleanup_text_elements(node):
    new_children = []
    changed = False

    for child in node.childNodes:
        if child.nodeType == child.TEXT_NODE and len(new_children) and new_children[-1].nodeType == child.TEXT_NODE:
            changed = True
            new_children[-1].data += child.data
        elif child.nodeType == child.TEXT_NODE and child.data == '':
            continue
        else:
            new_children.append(child)

    node.childNodes = new_children
    return changed


def replace_text_node_contents(node, ignorables):
    changed = False
    removable = []
    for i, c in enumerate(node.childNodes):
        if c.nodeType == c.TEXT_NODE:
            continue
        elif c.nodeType == c.COMMENT_NODE:
            short = c.data.strip()
            if short in ignorables:
                removable.append(i)
                changed = True
                continue
        else:
            changed = replace_text_node_contents(c, ignorables) or changed
    for node_index in reversed(removable):  # backwards not to affect earlier indices
        if node_index > 0:
            before = node.childNodes[node_index - 1]
            if before.nodeType == c.TEXT_NODE:
                trailing = count_trailing_spaces(before.data)
                before.data = before.data[:-trailing]

        if node_index < len(node.childNodes) - 1:
            after = node.childNodes[node_index + 1]
            if after.nodeType == c.TEXT_NODE:
                while len(after.data) and after.data[0] == ' ':
                    after.data = after.data[1:]
                if len(after.data) and after.data[0] == '\n':
                    after.data = after.data[1:]

        node.childNodes.remove(node.childNodes[node_index])
    changed = cleanup_text_elements(node) or changed
    return changed


@clean_ros
def remove_boilerplate_manifest_comments(package):
    ignorables = get_ignore_data('package', {'package': package.name}, add_newline=False)
    changed = replace_text_node_contents(package.package_xml.root, ignorables)
    if changed:
        package.package_xml.changed = changed
        remove_empty_manifest_lines(package)
