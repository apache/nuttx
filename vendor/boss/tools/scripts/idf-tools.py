#!/usr/bin/env python
# coding=utf-8
# Created Info: 2024/10/31, Xi'an
#

import os
import sys
import argparse
from typing import Any, List, Optional, Union

# custom package
from pycmd.utils import fatal, warn, info, CURRENT_PLATFORM
from pycmd.idf_tools_python_env import install_python_env, uninstall_python_env
from pycmd.idf_tools_unpack import unpack
from pycmd.idf_tools_parser import do_export, do_download, EXPORT_SHELL, EXPORT_KEY_VALUE

# custom directory.
IDF_TOOLS_PATH_DEFAULT = os.path.join('~', '.boss')
TOOLS_FILE_DEFAULT = 'scripts/tools.json'

IDF_NET_ASSETS_DEFAULT = 'dl.espressif.com'

class GlobalVarsStore:
    """
    Pythonic way how to handle global variables.
    One global instance of this class is initialized and used as an entrypoint (store)
    It handles string and boolean properties.
    """
    _instance: Optional['GlobalVarsStore'] = None
    _bool_properties = ['quiet', 'non_interactive']
    _string_properties = ['idf_path', 'idf_tools_path', 'tools_json', 'idf_dist_path']

    def __new__(cls, *args: Any, **kwargs: Any) -> 'GlobalVarsStore':
        if not cls._instance:
            cls._instance = super(GlobalVarsStore, cls).__new__(cls, *args, **kwargs)
            cls._instance._initialize_properties()
        return cls._instance

    def _initialize_properties(self) -> None:
        # Initialize boolean properties to False
        for prop in self._bool_properties:
            setattr(self, f'_{prop}', False)
        # Initialize string properties to None
        for prop in self._string_properties:
            setattr(self, f'_{prop}', None)

    def __getattr__(self, name: str) -> Any:
        if name in self._bool_properties + self._string_properties:
            value: Union[str, bool] = getattr(self, f'_{name}')
            if value is None and name in self._string_properties:
                raise ReferenceError(f'Variable {name} accessed before initialization.')
            return value
        raise AttributeError(f'{name} is not a valid attribute')

    def __setattr__(self, name: str, value: Any) -> None:
        if name in self._bool_properties + self._string_properties:
            super().__setattr__(f'_{name}', value)
        else:
            super().__setattr__(name, value)

g = GlobalVarsStore()

# action's function
def action_install(args: argparse.Namespace) -> None:
    info(f'Current system platform: {CURRENT_PLATFORM}')
    if args.python_env:
        install_python_env(args)
    pass

def action_uninstall(args: argparse.Namespace) -> None:
    if args.python_env:
        uninstall_python_env(args)
    pass

def action_extract(args: argparse.Namespace) -> None:
    location = args.files.pop()
    for file in args.files:
        unpack(file, location)

def action_export(args: argparse.Namespace) -> None:
    targets: List[str] = []
    if ',' in args.targets:
        targets = [ t.strip() for t in args.targets.split(',')]
    else:
        targets.extend([args.targets])

    if os.path.isfile(g.tools_json):
        do_export(args, g.tools_json, targets)
    else:
        fatal(f"{g.tools_json} is bad file.")
        raise SystemExit(1)

def action_download(args: argparse.Namespace) -> None:
    platform: str = ""
    targets: List[str] = []
    tools: List[str] = []

    if ',' in args.targets:
        targets = [ t.strip() for t in args.targets.split(',')]
    else:
        targets.extend([args.targets])

    assert not ('all' in args.tools and 'required' in args.tools)
    tools = args.tools

    platform = args.platform

    if os.path.isfile(g.tools_json):
        do_download(platform, tools, targets, g.tools_json)
    else:
        fatal(f"{g.tools_json} is bad file.")
        raise SystemExit(1)

def main(argv: List[str]) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--quiet', help='Don\'t output diagnostic messages to stdout/stderr', action='store_true')
    parser.add_argument('--non-interactive', help='Don\'t output interactive messages and questions', action='store_true')
    parser.add_argument('--tools-json', help='Path to the tools.json file to use')
    parser.add_argument('--idf-path', help='FREE-IDF path to use')

    subparsers = parser.add_subparsers(dest='action')

    download = subparsers.add_parser('download', help='Download the tools into the dist directory')
    download.add_argument('--platform', default=CURRENT_PLATFORM, help='Platform to download the tools for')
    download.add_argument('--targets', default='all', help=('A comma separated list of desired chip targets for installing. '
                                                            ' It defaults to installing all supported targets.'))
    download.add_argument('tools', metavar='TOOL', nargs='*', default=['required'],
                          help=('Tools to download. '
                                'Use empty or \'required\' to download required tools, not optional ones. '
                                'Use \'all\' to download all tools, including the optional ones.'))

    install = subparsers.add_parser('install', help='Download and install tools into the tools directory')
    install.add_argument('--python-env', help=('Create Python virtual environment and install the '
                                               'required Python packages'), action='store_true')
    
    uninstall = subparsers.add_parser('uninstall', help='Uninstall tools from the tools directory')
    uninstall.add_argument('--python-env', help=('Uninstall Python virtual environment and required Python packages'),
                           action='store_true')

    extract = subparsers.add_parser('extract', help='Extract the specified target.')
    extract.add_argument('--files', help=('Extract the specified target for Zip files. '
                                             'as: test1.zip test2.zip location.'), nargs='+', metavar='FILES')

    export = subparsers.add_parser('export', help='Output command for setting tool paths, suitable for shell')
    export.add_argument('--format', choices=[EXPORT_SHELL, EXPORT_KEY_VALUE],
                        help=('Format of the output: shell (suitable for printing into shell), '
                              'or key-value (suitable for parsing by other tools'))
    export.add_argument('--targets', default='all', help=('A comma separated list of desired chip targets for installing. '
                                                        ' It defaults to export all supported targets info.'))

    # let's start parser cmd's args
    args = parser.parse_args(argv)

    # code workspace
    g.idf_path = args.idf_path or os.environ.get('IDF_PATH') or os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
    os.environ['IDF_PATH'] = g.idf_path
    info(f'IDF_PATH: {g.idf_path}')

    # tools workspace
    g.idf_tools_path = os.environ.get('IDF_TOOLS_PATH') or os.path.expanduser(IDF_TOOLS_PATH_DEFAULT)
    os.environ['IDF_TOOLS_PATH'] = g.idf_tools_path
    info(f'IDF_TOOLS_PATH: {g.idf_tools_path}')

    # zips directory
    g.idf_dist_path = os.environ.get('IDF_DIST_PATH') or os.path.realpath(os.path.join(os.path.dirname(__file__), '../dist'))
    os.environ['IDF_DIST_PATH'] = g.idf_dist_path
    info(f'IDF_DIST_PATH: {g.idf_dist_path}')

    # net assets server
    net_assets = os.environ.get('IDF_NET_ASSETS', '')
    if not net_assets:
        #os.environ['IDF_NET_ASSETS'] = IDF_NET_ASSETS_DEFAULT
        info('not resource. let code master to resolve it.')

    if args.tools_json:
        g.tools_json = args.tools_json
    else:
        g.tools_json = os.path.realpath(os.path.join(g.idf_path, TOOLS_FILE_DEFAULT))

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