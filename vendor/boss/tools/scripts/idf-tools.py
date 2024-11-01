#!/usr/bin/env python
# coding=utf-8
# Created Info: 2024/10/31, Xi'an
#

import os
import sys
import argparse
from typing import List

# custom package
from pycmd.utils import fatal, warn, info
from pycmd.idf_tools_python_env import install_python_env, uninstall_python_env

# custom directory.
IDF_TOOLS_PATH_DEFAULT = os.path.join('~', '.boss')

# action's function
def action_install(args: argparse.Namespace) -> None:
    if args.python_env:
        install_python_env(args)
    pass

def action_uninstall(args: argparse.Namespace) -> None:
    if args.python_env:
        uninstall_python_env(args)
    pass

def main(argv: List[str]) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--quiet', help='Don\'t output diagnostic messages to stdout/stderr', action='store_true')
    parser.add_argument('--non-interactive', help='Don\'t output interactive messages and questions', action='store_true')
    parser.add_argument('--idf-path', help='FREE-IDF path to use')

    subparsers = parser.add_subparsers(dest='action')

    install = subparsers.add_parser('install', help='Download and install tools into the tools directory')
    install.add_argument('--python-env', help=('Create Python virtual environment and install the '
                                               'required Python packages'), action='store_true')
    
    uninstall = subparsers.add_parser('uninstall', help='Uninstall tools from the tools directory')
    uninstall.add_argument('--python-env', help=('Uninstall Python virtual environment and required Python packages'),
                           action='store_true')

    # let's start parser cmd's args
    args = parser.parse_args(argv)

    # code workspace
    idf_path = args.idf_path or os.environ.get('IDF_PATH') or os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
    os.environ['IDF_PATH'] = idf_path
    info(f'IDF_PATH: {idf_path}')

    # tools workspace
    idf_tools_path = os.environ.get('IDF_TOOLS_PATH') or os.path.expanduser(IDF_TOOLS_PATH_DEFAULT)
    os.environ['IDF_TOOLS_PATH'] = idf_tools_path
    info(f'IDF_TOOLS_PATH: {idf_tools_path}')

    # zips directory
    idf_dist_path = os.environ.get('IDF_DIST_PATH') or os.path.realpath(os.path.join(os.path.dirname(__file__), '../dist'))
    os.environ['IDF_DIST_PATH'] = idf_dist_path
    info(f'IDF_DIST_PATH: {idf_dist_path}')

    # call action's function
    if args.action:
        action_func_name = f'action_{args.action.replace("-", "_")}'
        action_func = globals()[action_func_name]
        action_func(args)
    else:
        parser.print_help()

if __name__ == '__main__':
    if 'MSYSTEM' in os.environ:
        fatal('MSys/Mingw is not supported. Please follow the getting started guide of the documentation to set up '
              'a supported environment')
        raise SystemExit(1)
    main(sys.argv[1:])