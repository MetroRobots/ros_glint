#!/usr/bin/python3
import argparse
import click

from roscompile import get_functions
from roscompile.diff import files_match
from roscompile.zipfile_interface import get_test_cases, run_case


def compare(pkg_in, pkg_out, debug=True):
    folder_diff = pkg_in.compare_filesets(pkg_out)

    success = True

    for fn in folder_diff['deleted']:
        if debug:
            click.secho('Failed to generate %s' % fn, fg='yellow')
        success = False
    for fn in folder_diff['added']:
        if debug:
            click.secho('Should have deleted %s' % fn, fg='yellow')
        success = False
    for filename in folder_diff['matches']:
        if not files_match(pkg_in, pkg_out, filename, debug):
            success = False
    return success


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('zipfile')
    parser.add_argument('-f', '--fail_once', action='store_true')
    parser.add_argument('-l', '--last', action='store_true')
    args = parser.parse_args()
    config, cases = get_test_cases(args.zipfile)
    roscompile_functions = get_functions()
    successes = 0
    total = 0

    try:
        for test_config in config:
            if args.last and test_config != config[-1]:
                continue

            with cases[test_config['in']] as pkg_in:
                try:
                    total += 1
                    if test_config['in'] == test_config['out']:
                        pkg_out = pkg_in.copy()
                    else:
                        pkg_out = cases[test_config['out']]

                    click.secho('{:25} >> {:25} {}'.format(test_config['in'], test_config['out'],
                                                           ','.join(test_config['functions'])),
                                bold=True, fg='white')
                    run_case(pkg_in, pkg_out, test_config, roscompile_functions)
                    if compare(pkg_in, pkg_out):
                        click.secho('  SUCCESS', fg='green')
                        successes += 1
                    else:
                        click.secho('  FAIL', fg='red')
                        if args.fail_once:
                            break
                except Exception as e:
                    click.secho('  EXCEPTION ' + str(e), fg='red')
                    if args.last or args.fail_once:
                        raise
    finally:
        if not args.last:
            click.secho('{}/{}'.format(successes, total), bold=True, fg='white')
    if successes != total:
        exit(-1)
