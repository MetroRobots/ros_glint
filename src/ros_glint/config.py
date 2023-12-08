import pathlib
import yaml

DOT_ROS_FOLDER = pathlib.Path('~/.ros').expanduser()
POSSIBLE_CONFIG_FILES = ['roscompile.yaml', 'glint.yaml']
CONFIG = None


def get_config():
    global CONFIG
    if CONFIG is None:
        for possible_config_file in POSSIBLE_CONFIG_FILES:
            path = DOT_ROS_FOLDER / possible_config_file
            if path.exists():
                CONFIG = yaml.safe_load(open(path))
                return CONFIG
        CONFIG = {}
    return CONFIG
