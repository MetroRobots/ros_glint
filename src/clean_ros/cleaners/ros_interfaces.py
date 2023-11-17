from ..core import clean_ros
from ..util import TRAILING_PATTERN
from ros_introspect.components.ros_interface import PRIMITIVES

STANDARD = {
    'Header': 'std_msgs'
}


@clean_ros
def fill_in_msg_package_names(package):
    interfaces = package.get_ros_interfaces()
    all_names = {ros_i.name for ros_i in interfaces}

    for interface in interfaces:
        for section in interface.sections:
            for field in section.fields:
                if '/' in field.type or field.type in PRIMITIVES:
                    continue

                if field.type in STANDARD:
                    field.type = STANDARD[field.type] + '/' + field.type
                    interface.changed = True
                elif field.type in all_names:
                    field.type = package.name + '/' + field.type
                    interface.changed = True


@clean_ros
def remove_trailing_whitespace_from_generators(package):
    for interface in package.get_ros_interfaces():
        for i, content in enumerate(interface.contents):
            if not isinstance(content, str):
                continue
            m = TRAILING_PATTERN.match(content)
            if m:
                interface.contents[i] = m.group(1) + '\n'
                interface.changed = True
