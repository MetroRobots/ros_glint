#!/usr/bin/env python
from roscompile.zipfile_interface import locate_zip_file, get_test_cases
from roscompile import get_functions
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('zipfile', nargs='?')
    args = parser.parse_args()

    if not args.zipfile:
        args.zipfile = locate_zip_file()

    config, cases = get_test_cases(args.zipfile)
    roscompile_functions = get_functions()
    coverage_counts = {}
    max_len = 0
    for name in roscompile_functions:
        coverage_counts[name] = 0
        max_len = max(max_len, len(name))

    for test_config in config:
        for fne_name in test_config['functions']:
            coverage_counts[fne_name] += 1

    z_count = 0
    for name, count in sorted(coverage_counts.items(), key=lambda kv: kv[1], reverse=True):
        print('{:{}} {:=3d}'.format(name, max_len, count))
        if count == 0:
            z_count += 1

    if z_count > 0:
        print('Zero tests written for {} functions'.format(z_count))
