from ros_introspection.package_xml import get_ordering_index, replace_package_set

from .util import get_config, roscompile


@roscompile
def check_manifest_dependencies(package):
    build_depends = package.get_build_dependencies()
    run_depends = package.get_run_dependencies()
    test_depends = package.get_test_dependencies()
    package.package_xml.add_packages(build_depends, run_depends, test_depends)

    if package.generators:
        md = package.get_dependencies_from_msgs()
        package.package_xml.add_packages(md, md)

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

        if package.package_xml.format == 1:
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


@roscompile
def check_python_dependencies(package):
    run_depends = package.source_code.get_external_python_dependencies()
    package.package_xml.add_packages(set(), run_depends, prefer_depend_tag=False)


@roscompile
def greedy_depend_tag(package):
    if package.package_xml.format == 1:
        return
    replace_package_set(package.package_xml, ['build_depend', 'build_export_depend', 'exec_depend'], 'depend')


def get_sort_key(node, alphabetize_depends=True):
    if node:
        name = node.nodeName
    else:
        name = None

    index = get_ordering_index(name)

    if not alphabetize_depends:
        return index
    if name and 'depend' in name:
        return index, node.firstChild.data
    else:
        return index, None


def get_chunks(children):
    """Given the children, group the elements into tuples.

    Tuple format: (an element node, [(some number of text nodes), that element node again])
    """
    chunks = []
    current = []
    for child_node in children:
        current.append(child_node)
        if child_node.nodeType == child_node.ELEMENT_NODE:
            chunks.append((child_node, current))
            current = []
    if len(current) > 0:
        chunks.append((None, current))
    return chunks


@roscompile
def enforce_manifest_ordering(package, alphabetize=True):
    root = package.package_xml.root
    chunks = get_chunks(root.childNodes)

    new_children = []

    for a, b in sorted(chunks, key=lambda d: get_sort_key(d[0], alphabetize)):
        new_children += b

    if root.childNodes != new_children:
        package.package_xml.changed = True
        root.childNodes = new_children


@roscompile
def update_people(package, config=None):
    if config is None:
        config = get_config()
    for d in config.get('replace_rules', []):
        package.package_xml.update_people(d['to']['name'], d['to']['email'],
                                          d['from'].get('name', None), d['from'].get('email', None))


@roscompile
def update_license(package, config=None):
    if config is None:
        config = get_config()
    if 'default_license' not in config or 'TODO' not in package.package_xml.get_license():
        return

    package.package_xml.set_license(config['default_license'])
