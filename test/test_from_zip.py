import inspect
import pathlib
import pytest
from clean_ros.diff import files_match
from zip_testing import get_test_cases

from ros_introspect import Package, ROSResources

TEST_CASE_FILENAMES = ['../ros1_test_data.zip', '../ros2_test_data.zip']
FILE_ERROR_MESSAGE = 'These files should have been {} but weren\'t: {}'

ros1_config = []
ros1_ids = []
ros1_cases = None

ros2_config = []
ros2_ids = []
ros2_cases = None

roscompile_functions = {}

for fn in TEST_CASE_FILENAMES:
    p = pathlib.Path(fn).resolve()
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

        manual_pkgs = []
        for pkg in test_config.get('pkgs', []):
            if pkg not in resources.packages:
                manual_pkgs.append(pkg)
                resources.packages.add(pkg)

        for function_name in test_config['functions']:
            if function_name not in roscompile_functions:
                continue
            fne = roscompile_functions[function_name]
            if 'config' in inspect.getfullargspec(fne).args:
                fne(pp, config=local_config)
            else:
                fne(pp)
        pp.save()

        for pkg in manual_pkgs:
            resources.packages.remove(pkg)

        folder_diff = pkg_in.compare_filesets(pkg_out)
        assert len(folder_diff['deleted']) == 0, FILE_ERROR_MESSAGE.format('deleted', folder_diff['deleted'])
        assert len(folder_diff['added']) == 0, FILE_ERROR_MESSAGE.format('generated', folder_diff['added'])
        for filename in folder_diff['matches']:
            assert files_match(pkg_in, pkg_out, filename), 'The contents of {} do not match!'.format(filename)


@pytest.mark.parametrize('test_config', ros1_config, ids=ros1_ids)
def test_ros1_from_zip(test_config):
    run_case(test_config, ros1_cases)


@pytest.mark.parametrize('test_config', ros2_config, ids=ros2_ids)
def test_ros2_from_zip(test_config):
    run_case(test_config, ros2_cases)
