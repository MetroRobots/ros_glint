from ros_glint import get_linters
from ros_introspect.util import identifier_split
import ruamel.yaml
yaml = ruamel.yaml.YAML()

path = '.pre-commit-hooks.yaml'

REPLACE_WORDS = {
    'Cmake': 'CMake',
    'Cpp': 'C++',
    'Cplusplus': 'C++',
    'Misc': 'Miscelaneous',
    'Rviz': 'RViz',
    'Xml': 'XML',
}


def get_words(s):
    words = []
    for word in identifier_split(s):
        word = word.title()
        word = REPLACE_WORDS.get(word, word)
        words.append(word)

    return words


data = []
for function_id, fne in get_linters().items():
    row = {'id': function_id}
    row['name'] = ' '.join(get_words(function_id))
    row['entry'] = f'glint_hook {function_id} -f'
    row['language'] = 'python'
    # TODO: Limit to certain files
    data.append(row)

yaml.dump(data, open(path, 'w'))
