from ..core import glinter
from ros_introspect.components.ros_interface import PRIMITIVES

STANDARD = {
    'Header': 'std_msgs'
}


@glinter
def clean_whitespace_from_interface_definition(package):
    # Formerly remove_trailing_whitespace_from_generators
    for interface in package.get_ros_interfaces():
        interface.force_regeneration()


@glinter
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
