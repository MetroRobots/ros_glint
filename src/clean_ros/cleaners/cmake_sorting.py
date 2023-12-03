from ..core import clean_ros
from ..config import get_config
from ..cmake_ordering import CMakeOrderStyle, get_ordered_build_targets, get_ordering, get_sort_key, get_style


def get_clusters(cmake, desired_style):
    """Return a list of clusters where each cluster is an array of strings with a Command/CommandGroup at the end.

    The clusters are sorted according to the desired style.
    The strings are grouped at the beginning to maintain the newlines and indenting before each Command.
    """
    anchors = get_ordered_build_targets(cmake)
    ordering = get_ordering(desired_style)
    clusters = []
    current = []
    for content in cmake.contents:
        current.append(content)
        if isinstance(content, str):
            continue
        key = get_sort_key(content, anchors, ordering)
        clusters.append((key, current))
        current = []
    if len(current) > 0:
        clusters.append((get_sort_key(None, anchors, ordering), current))

    return [kv[1] for kv in sorted(clusters, key=lambda kv: kv[0])]


def enforce_ordering(cmake, default_style=None):
    existing_style = get_style(cmake)
    if default_style in [CMakeOrderStyle.INSTALL_FIRST, CMakeOrderStyle.TEST_FIRST]:
        desired_style = default_style
    elif existing_style in [CMakeOrderStyle.INSTALL_FIRST, CMakeOrderStyle.TEST_FIRST]:
        desired_style = existing_style
    else:
        desired_style = CMakeOrderStyle.TEST_FIRST

    clusters = get_clusters(cmake, desired_style)

    cmake.contents = []
    for contents in clusters:
        cmake.contents += contents

    for group in cmake.content_map['group']:
        enforce_ordering(group.contents, default_style)
    cmake.mark_changed()


@clean_ros
def enforce_cmake_ordering(package, config=None):
    if not package.cmake:
        return
    if config is None:
        config = get_config()
    default_style_s = config.get('cmake_style', 'UNKNOWN')
    default_style = CMakeOrderStyle[default_style_s.upper()]

    enforce_ordering(package.cmake, default_style)
