from ..core import glinter
from ..util import get_ignore_data
from ..config import get_config
from ros_introspect.package import DependencyType
from ros_introspect.components.package_xml import count_trailing_spaces, get_chunks, get_sort_key, PEOPLE_TAGS


@glinter
def check_manifest_dependencies(package, config=None):
    if config is None:
        config = get_config()  # pragma: no cover
    prefer_depend_tag = config.get('prefer_depend_tag', False)

    dep_dict = {}
    for dt in DependencyType:
        dep_dict[dt] = package.get_dependencies(dt)

    package.package_xml.add_dependencies(dep_dict, prefer_depend_tag)

    # Special handling for interface dependencies
    # because not all run deps are build deps
    if not package.get_ros_interfaces():
        return
    if package.ros_version == 1:
        build_dep = 'message_generation'
        run_dep = 'message_runtime'
        export = 'message_runtime'
        export_tag = 'build_export_depend'
    else:
        build_dep = 'rosidl_default_generators'
        run_dep = 'rosidl_default_runtime'
        export = 'rosidl_interface_packages'
        export_tag = 'member_of_group'

    if package.package_xml.xml_format == 1:
        pairs = [('build_depend', build_dep),
                 ('run_depend', run_dep)]
    else:
        pairs = [('build_depend', build_dep),
                 (export_tag, export),
                 ('exec_depend', run_dep)]
        package.package_xml.remove_dependencies('depend', [build_dep, run_dep])
    for tag, msg_pkg in pairs:
        existing = package.package_xml.get_packages_by_tag(tag)
        if msg_pkg not in existing:
            package.package_xml.insert_new_packages(tag, [msg_pkg])


@glinter
def enforce_manifest_ordering(package, alphabetize=True):
    root = package.package_xml.root
    chunks = get_chunks(root.childNodes)

    new_children = []

    for a, b in sorted(chunks, key=lambda d: get_sort_key(d[0], alphabetize, package.package_xml.xml_format)):
        new_children += b

    if root.childNodes != new_children:
        package.package_xml.changed = True
        root.childNodes = new_children


@glinter
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


@glinter
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

                if '\n' not in c.data:
                    c.data = '\n' + c.data
                    changed = True
                spaces = count_trailing_spaces(c.data)
                if spaces != ideal_length:
                    last_nl = c.data.rindex('\n')
                    c.data = c.data[: last_nl + 1] + (' ' * ideal_length)
                    changed = True
            else:
                if prev_was_node:
                    changed = True
                    insert_before_list.append(c)

                prev_was_node = True
                enforce_tabbing_helper(manifest, c, tabs + 1)

        for c in insert_before_list:
            node.insertBefore(manifest.create_new_tab_element(tabs), c)

        manifest.changed = manifest.changed or changed

        if len(node.childNodes) == 0:
            return
        last = node.childNodes[-1]
        if last.nodeType != last.TEXT_NODE:
            node.appendChild(manifest.create_new_tab_element(tabs - 1))
            manifest.changed = True

    enforce_tabbing_helper(package.package_xml, package.package_xml.root)


@glinter
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


@glinter
def remove_boilerplate_manifest_comments(package):
    ignorables = get_ignore_data('package', {'package': package.name}, add_newline=False)
    changed = replace_text_node_contents(package.package_xml.root, ignorables)
    if changed:
        package.package_xml.changed = changed
        remove_empty_manifest_lines(package)


def replace_package_set(package_xml, source_tags, new_tag):
    """Replace all the elements with tags in source_tags with new elements with new_tag."""
    intersection = None
    for tag in source_tags:
        pkgs = set(package_xml.get_packages_by_tag(tag))
        if intersection is None:
            intersection = pkgs
        else:
            intersection = intersection.intersection(pkgs)
    for tag in source_tags:
        package_xml.remove_dependencies(tag, intersection)
    package_xml.insert_new_packages(new_tag, intersection)


@glinter
def greedy_depend_tag(package):
    if package.package_xml.xml_format == 1:
        return
    replace_package_set(package.package_xml, ['build_depend', 'build_export_depend', 'exec_depend'], 'depend')


@glinter
def update_people(package, config=None):
    if config is None:
        config = get_config()  # pragma: no cover
    for d in config.get('replace_rules', []):
        target_name = d['to']['name']
        target_email = d['to']['email']
        search_name = d['from'].get('name')
        search_email = d['from'].get('email')

        for el in package.package_xml.get_elements_by_tags(PEOPLE_TAGS):
            name = el.childNodes[0].nodeValue
            email = el.getAttribute('email') if el.hasAttribute('email') else ''
            if (search_name is None or name == search_name) and (search_email is None or email == search_email):
                el.childNodes[0].nodeValue = target_name
                if target_email:
                    el.setAttribute('email', target_email)
                package.package_xml.changed = True


@glinter
def update_license(package, config=None):
    if config is None:
        config = get_config()  # pragma: no cover
    if 'default_license' not in config or 'TODO' not in package.package_xml.get_license():
        return

    package.package_xml.set_license(config['default_license'])
