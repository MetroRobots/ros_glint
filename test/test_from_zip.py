import inspect
import pooch
import pytest
from ros_glint import get_linters
from ros_glint.diff import get_diff_string
from ros_glint.terminal import Fore, Style
from zip_interface import get_test_cases
from betsy_ros import ROSInterface

from ros_introspect import Package, ROSResources

URL_TEMPLATE = 'https://github.com/DLu/roscompile_test_data/raw/{}/test_data.zip'
TEST_DATA = [
    # (branch, known_hash)
    ('ros1', '2100c19c912c6044194e7a77dad3d002e3b22f5206b171c3f127069b87dbc662'),
    ('ros2', '97fa59fdc742a60e57cf17643b70eb21aa26a6967cc64d364371480665fb5635'),
]

linters = get_linters()


def files_match(pkg_in, pkg_out, filename, show_diff=True):
    """Return true if the contents of the given file are the same in each package. Otherwise maybe show the diff."""
    generated_contents = pkg_in.get_contents(filename).rstrip()
    canonical_contents = pkg_out.get_contents(filename).rstrip()
    ret = generated_contents == canonical_contents
    if show_diff and not ret:
        print(get_diff_string(generated_contents, canonical_contents, filename))
    return ret


def run_case(test_config, cases):
    resources = ROSResources.get()

    with cases[test_config['in']] as pkg_in:
        pkg_out = cases[test_config['out']]
        root = pkg_in.root
        if 'subpkg' in test_config:
            root = root / test_config['subpkg']
        pp = Package(root)
        local_config = test_config.get('config', {})

        # Initialize ROS Resources
        resources.packages = set(test_config.get('pkgs', []))
        resources.messages = set()
        for msg in test_config.get('msgs', []):
            parts = msg.split('/')
            resources.messages.add(ROSInterface(parts[0], 'msg', parts[1]))

        # Run Functions
        for function_name in test_config['functions']:
            assert function_name in linters, f'Missing rule: {function_name}'
            if function_name not in linters:
                return
            fne = linters[function_name]
            if 'config' in inspect.getfullargspec(fne).args:
                fne(pp, config=local_config)
            else:
                fne(pp)
        pp.save()

        folder_diff = pkg_in.compare_filesets(pkg_out)

        def jp(paths):
            return ', '.join(map(str, paths))

        print(Fore.BLUE + Style.BRIGHT, end='')
        print('{:25} >> {:25} {}'.format(test_config['in'], test_config['out'],
                                         ','.join(test_config['functions'])), end='')
        print(Style.RESET_ALL)

        assert len(folder_diff['deleted']) == 0, \
            f'These files should have been deleted but weren\'t: {jp(folder_diff["deleted"])}'
        assert len(folder_diff['added']) == 0, \
            f'These files should have been generated but weren\'t: {jp(folder_diff["added"])}'
        for filename in folder_diff['matches']:
            assert files_match(pkg_in, pkg_out, filename), 'The contents of {} do not match!'.format(filename)


parameters = []
test_ids = []

for branch, known_hash in TEST_DATA:
    file_path = pooch.retrieve(URL_TEMPLATE.format(branch), known_hash=known_hash)
    config, test_data = get_test_cases(file_path)

    for i, test_config in enumerate(config):
        parameters.append((test_config, test_data))
        test_ids.append(f'{branch}_test_{i:03d}')


@pytest.mark.parametrize('test_config, test_data', parameters, ids=test_ids)
def test_from_zip(test_config, test_data):
    run_case(test_config, test_data)
