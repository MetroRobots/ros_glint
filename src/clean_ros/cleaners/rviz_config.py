from ..core import clean_ros, root
from ros_introspect.components.rviz_config import dictionary_subtract
import yaml

DATA_FOLDER = root / 'data'

CLASS_DEFAULTS = yaml.safe_load(open(DATA_FOLDER / 'rviz_class_defaults.yaml'))
GENERIC_DEFAULTS = yaml.safe_load(open(DATA_FOLDER / 'rviz_generic_defaults.yaml'))
GLOBAL_DEFAULTS = yaml.safe_load(open(DATA_FOLDER / 'rviz_global_defaults.yaml'))

ROBOT_MODEL_LINK_DEFAULTS = {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True}


@clean_ros
def clean_up_rviz_configs(package):
    for rviz_config in package.rviz_config:
        # print("\tCleaning up " + str(rviz_config))
        for config in rviz_config.get_class_dicts():
            if dictionary_subtract(config, GENERIC_DEFAULTS):
                rviz_config.changed = True

            full_class = config['Class']
            class_name = full_class.split('/')[-1]

            the_defaults = CLASS_DEFAULTS.get(full_class, {})
            the_defaults.update(CLASS_DEFAULTS.get(class_name, {}))
            if dictionary_subtract(config, the_defaults):
                rviz_config.changed = True

            # Special Case(s)
            if config.get('Topic') == '':
                del config['Topic']
                rviz_config.changed = True

            if full_class == 'rviz_default_plugins/RobotModel':
                for k, v in list(config.get('Links', {}).items()):
                    if not isinstance(v, dict):
                        continue
                    if dictionary_subtract(config['Links'][k], ROBOT_MODEL_LINK_DEFAULTS):
                        rviz_config.changed = True
                        if not config['Links'][k]:
                            del config['Links'][k]

            if full_class in ['rviz/Camera', 'rviz_default_plugins/Camera'] and 'Visibility' in config:
                visibility = config['Visibility']
                for key in list(visibility.keys()):
                    if visibility[key]:
                        rviz_config.changed = True
                        del visibility[key]
                if not visibility:
                    del config['Visibility']
        if dictionary_subtract(rviz_config.contents, GLOBAL_DEFAULTS):
            rviz_config.changed = True
