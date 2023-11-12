from ..core import clean_ros

from ros_introspect.components.package_xml import INITIAL_TAGS, MBLOCK_TAGS, DEPEND_TAGS, FINAL_TAGS

# In most manifests, the ordering of the mblock doesn't matter, but we sort the depends
ORDERING = INITIAL_TAGS + [MBLOCK_TAGS] + DEPEND_TAGS + FINAL_TAGS
# In V3 manifests, we ensure the mblock is sorted, but not the depends
ORDERING_V3 = INITIAL_TAGS + MBLOCK_TAGS + DEPEND_TAGS + FINAL_TAGS


def get_ordering_index(name, whiny=True, manifest_version=None):
    if manifest_version and manifest_version >= 3:
        ordering = ORDERING_V3
    else:
        ordering = ORDERING

    for i, o in enumerate(ordering):
        if isinstance(o, list):
            if name in o:
                return i
        elif name == o:
            return i
    if name and whiny:
        print('\tUnsure of ordering for ' + name)
    return len(ordering)


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


@clean_ros
def enforce_manifest_ordering(package, alphabetize=True):
    root = package.package_xml.root
    chunks = get_chunks(root.childNodes)

    new_children = []

    for a, b in sorted(chunks, key=lambda d: get_sort_key(d[0], alphabetize)):
        new_children += b

    if root.childNodes != new_children:
        package.package_xml.changed = True
        root.childNodes = new_children
