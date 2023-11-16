import inspect
import pathlib
import pytest
from clean_ros import get_functions
from clean_ros.diff import files_match
from zip_testing import get_test_cases
from betsy_ros import ROSInterface

from ros_introspect import Package, ROSResources

TEST_CASE_FILENAMES = ['../ros1_test_data.zip', '../ros2_test_data.zip']

ros1_config = []
ros1_ids = []
ros1_cases = None

ros2_config = []
ros2_ids = []
ros2_cases = None

cleaner_functions = get_functions()

for fn in TEST_CASE_FILENAMES:
    p = pathlib.Path(fn)
    v = p.name.split('_')[0]
    config, test_data = get_test_cases(p)

    configs_to_test = []
    test_ids = []

    c = 0
    for test_config in config:
        configs_to_test.append(test_config)
        test_ids.append(f'{v}_test_{c:03d}')
        c += 1

    if v == 'ros1':
        ros1_config = configs_to_test
        ros1_cases = test_data
        ros1_ids = test_ids
    else:
        ros2_config = configs_to_test
        ros2_cases = test_data
        ros2_ids = test_ids


def run_case(test_config, cases):
    resources = ROSResources()

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
            assert function_name in cleaner_functions, f'Missing rule: {function_name}'
            if function_name not in cleaner_functions:
                return
            fne = cleaner_functions[function_name]
            if 'config' in inspect.getfullargspec(fne).args:
                fne(pp, config=local_config)
            else:
                fne(pp)
        pp.save()

        folder_diff = pkg_in.compare_filesets(pkg_out)

        def jp(paths):
            return ', '.join(map(str, paths))

        assert len(folder_diff['deleted']) == 0, \
            f'These files should have been deleted but weren\'t: {jp(folder_diff["deleted"])}'
        assert len(folder_diff['added']) == 0, \
            f'These files should have been generated but weren\'t: {jp(folder_diff["added"])}'
        for filename in folder_diff['matches']:
            assert files_match(pkg_in, pkg_out, filename), 'The contents of {} do not match!'.format(filename)


@pytest.mark.parametrize('test_config', ros1_config, ids=ros1_ids)
def test_ros1_from_zip(test_config):
    run_case(test_config, ros1_cases)


@pytest.mark.parametrize('test_config', ros2_config, ids=ros2_ids)
def test_ros2_from_zip(test_config):
    run_case(test_config, ros2_cases)
