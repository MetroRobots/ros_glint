import inspect
import pooch
import pytest
from ros_glint import get_linters
from ros_glint.diff import get_diff_string
from ros_glint.terminal import color_text
from zip_interface import get_test_cases
from betsy_ros import ROSInterface

from ros_introspect import Package, ROSResources

URL_TEMPLATE = 'https://github.com/DLu/roscompile_test_data/raw/{}/test_data.zip'
TEST_DATA = [
    # (branch, known_hash)
    ('ros1', '84bf97b92d2ceb8d4369ed0d6fd8a01fabaf3006034fb7710d448b8cfe9885d9'),
    ('ros2', 'cf23d832e0cf56c8e160c0178af81e865c615fb330fb26604ddf031fc51a280a'),
]

linters = get_linters()


def run_case(test_config, cases):
    resources = ROSResources.get()

    with cases[test_config['in']] as pkg_in:
        if test_config['in'] == test_config['out']:
            pkg_out = pkg_in.copy()
        else:
            pkg_out = cases[test_config['out']]

        root = pkg_in.root
        pkg_obj = Package(root)
        local_config = test_config.get('config', {})

        # Initialize ROS Resources
        resources.packages = set(test_config.get('pkgs', []))
        resources.messages = set()
        for msg in test_config.get('msgs', []):
            parts = msg.split('/')
            resources.messages.add(ROSInterface(parts[0], 'msg', parts[1]))

        # Run Functions
        for function_name in test_config['functions']:
            if function_name not in linters:
                pytest.skip(f'Missing linter: {function_name}')

            fne = linters[function_name]
            if 'config' in inspect.getfullargspec(fne).args:
                fne(pkg_obj, config=local_config)
            else:
                fne(pkg_obj)

        components_with_changes = [comp.rel_fn for comp in pkg_obj if comp.changed]
        pkg_obj.save()

        s = '{:25} >> {:25} {}'.format(test_config['in'], test_config['out'],
                                       ','.join(test_config['functions']))
        print(color_text(s, 'BLUE', bright=True))

        # Compute the differences
        filenames_in = pkg_in.get_filenames()
        filenames_out = pkg_out.get_filenames()
        problems = []

        for filename in sorted(filenames_in - filenames_out):
            print(get_diff_string(pkg_in.get_contents(filename).rstrip(), '', filename))
            problems.append(f"File {filename} should have been deleted but wasn't")
        for filename in sorted(filenames_out - filenames_in):
            print(get_diff_string('', pkg_out.get_contents(filename).rstrip(), filename))
            problems.append(f"File {filename} should have been generated but wasn't")
        for filename in sorted(filenames_in.intersection(filenames_out)):
            contents_in = pkg_in.get_contents(filename).rstrip()
            contents_out = pkg_out.get_contents(filename).rstrip()

            if contents_in == contents_out:
                if filename in components_with_changes and test_config['in'] == test_config['out']:
                    problems.append(f'File {filename} has invisible changes')
            else:
                print(get_diff_string(contents_in, contents_out, filename))
                problems.append('The contents of {} do not match!'.format(filename))

        assert not problems, ', '.join(problems)


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


@pytest.mark.parametrize('test_config, test_data', parameters, ids=test_ids)
def test_idempotency_from_zip(test_config, test_data):
    test_config_x = dict(test_config)
    test_config_x['in'] = test_config_x['out']
    run_case(test_config_x, test_data)
