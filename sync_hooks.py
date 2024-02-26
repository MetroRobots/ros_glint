from ros_glint import get_linters
from ros_introspect.util import identifier_split
import ruamel.yaml
yaml = ruamel.yaml.YAML()

path = '.pre-commit-hooks.yaml'

REPLACE_WORDS = {
    'Cmake': 'CMake',
    'Cpp': 'C++',
    'Cplusplus': 'C++',
    'Misc': 'Miscellaneous',
    'Rviz': 'RViz',
    'Xml': 'XML',
}
GROUPS = {
    'prettify_cmake': ['prettify_catkin_package_cmd',
                       'prettify_package_lists',
                       'prettify_msgs_srvs',
                       'prettify_installs',
                       'prettify_command_groups'],
    'remove_boilerplate': ['remove_empty_export_tag',
                           'remove_boilerplate_manifest_comments',
                           'remove_boilerplate_cmake_comments'],
}


def to_english(s):
    words = []
    for word in identifier_split(s):
        word = word.title()
        word = REPLACE_WORDS.get(word, word)
        words.append(word)

    return ' '.join(words)


data = []
for function_id in get_linters().keys():
    row = {'id': function_id}
    row['name'] = to_english(function_id)
    row['entry'] = f'glint_hook {function_id} -f'
    row['language'] = 'python'
    # TODO: Limit to certain files
    data.append(row)

for group_name, functions in GROUPS.items():
    row = {'id': group_name}
    row['name'] = to_english(group_name)
    row['entry'] = f'glint_hook {" ".join(functions)} -f'
    row['language'] = 'python'
    data.append(row)

yaml.dump(data, open(path, 'w'))
