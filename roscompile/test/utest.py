#!/usr/bin/env python
from roscompile import get_functions
from roscompile.diff import files_match
from roscompile.zipfile_interface import get_use_cases_from_zip, locate_zip_file, run_case

FILE_ERROR_MESSAGE = 'These files should have been {} but weren\'t: {}'

zipfile = locate_zip_file()
config, cases = get_use_cases_from_zip(zipfile)
roscompile_functions = get_functions()


def test_generator():
    for test_config in config:
        yield roscompile_check, test_config


def roscompile_check(test_config):
    with cases[test_config['in']] as pkg_in:
        pkg_out = cases[test_config['out']]
        run_case(pkg_in, pkg_out, test_config, roscompile_functions)

        folder_diff = pkg_in.compare_filesets(pkg_out)
        assert len(folder_diff['deleted']) == 0, FILE_ERROR_MESSAGE.format('deleted', folder_diff['deleted'])
        assert len(folder_diff['added']) == 0, FILE_ERROR_MESSAGE.format('generated', folder_diff['added'])
        for filename in folder_diff['matches']:
            assert files_match(pkg_in, pkg_out, filename), 'The contents of {} do not match!'.format(filename)
