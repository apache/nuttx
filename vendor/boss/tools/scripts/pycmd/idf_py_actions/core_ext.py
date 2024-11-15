import fnmatch
import json
import os
import shutil
import sys
from typing import Any, Dict, List, Optional

from click import Context
import click

from pycmd.errors import FatalError
from pycmd.idf_py_actions.global_options import global_options
from pycmd.idf_py_actions.tools import PropertyDict, ensure_build_directory, merge_action_lists


def action_extensions(base_actions: Dict, project_path: str) -> Any:
    def help_and_exit(action: str, ctx: Context, param: List, json_option: bool, add_options: bool) -> None:
        if json_option:
            output_dict = {}
            # output_dict['target'] = get_target(param.project_dir)  # type: ignore
            output_dict['actions'] = []
            actions = ctx.to_info_dict().get('command').get('commands')
            for a in actions:
                action_info = {}
                action_info['name'] = a
                action_info['description'] = actions[a].get('help')
                if add_options:
                    action_info['options'] = actions[a].get('params')
                output_dict['actions'].append(action_info)
            print(json.dumps(output_dict, sort_keys=True, indent=4))
        else:
            print(ctx.get_help())
        ctx.exit()


    def python_clean(action: str, ctx: Context, args: PropertyDict) -> None:
        for root, dirnames, filenames in os.walk(os.path.join(os.environ['IDF_PATH'], 'scripts')):
            for d in dirnames:
                if d == '__pycache__':
                    dir_to_delete = os.path.join(root, d)
                    if args.verbose:
                        print('Removing: %s' % dir_to_delete)
                    shutil.rmtree(dir_to_delete)
            for filename in fnmatch.filter(filenames, '*.py[co]'):
                file_to_delete = os.path.join(root, filename)
                if args.verbose:
                    print('Removing: %s' % file_to_delete)
                os.remove(file_to_delete)


    def verbose_callback(ctx: Context, param: List, value: str) -> Optional[str]:
        if not value or ctx.resilient_parsing:
            return None

        for line in ctx.command.verbose_output:
            print(line)

        return value


    def validate_root_options(ctx: Context, args: PropertyDict, tasks: List) -> None:
        args.project_dir = os.path.realpath(args.project_dir)
        if args.build_dir is not None and args.project_dir == os.path.realpath(args.build_dir):
            raise FatalError(
                'Setting the build directory to the project directory is not supported. Suggest dropping '
                "--build-dir option, the default is a 'build' subdirectory inside the project directory.")
        if args.build_dir is None:
            args.build_dir = os.path.join(args.project_dir, 'build')
        args.build_dir = os.path.realpath(args.build_dir)


    def build_target(target_name: str, ctx: Context, args: PropertyDict) -> None:
        """
        Execute the target build system to build target 'target_name'

        Calls ensure_build_directory() which will run cmake to generate a build
        directory (with the specified generator) as needed.
        """
        ensure_build_directory(args, ctx.info_name)


    def menuconfig(target_name: str, ctx: Context, args: PropertyDict, style: str) -> None:
        """
        Menuconfig target is build_target extended with the style argument for setting the value for the environment
        variable.
        """
        if sys.platform != 'win32':
            try:
                import curses  # noqa: F401
            except ImportError:
                raise FatalError('\n'.join(
                    ['', "menuconfig failed to import the standard Python 'curses' library.",
                     'Please re-run the install script which might be able to fix the issue.']))
        if sys.version_info[0] < 3:
            # The subprocess lib cannot accept environment variables as "unicode".
            # This encoding step is required only in Python 2.
            style = style.encode(sys.getfilesystemencoding() or 'utf-8')
        os.environ['MENUCONFIG_STYLE'] = style
        args.no_hints = True
        build_target(target_name, ctx, args)


    root_options = {
        'global_options': [
            {
                'names': ['--dry-run'],
                'help': "Only process arguments, but don't execute actions.",
                'is_flag': True,
                'hidden': True,
                'default': False,
            },
            {
                'names': ['-v', '--verbose'],
                'help': 'Verbose build output.',
                'is_flag': True,
                'is_eager': True,
                'default': False,
                'callback': verbose_callback,
            },
            {
                'names': ['-C', '--project-dir'],
                'scope': 'shared',
                'help': 'Project directory.',
                'type': click.Path(),
                'default': os.getcwd(),
            },
            {
                'names': ['-B', '--build-dir'],
                'help': 'Build directory.',
                'type': click.Path(),
                'default': None,
            },
        ],
        'global_action_callbacks': [validate_root_options],
    }
    build_actions = {
        'actions': {
            'menuconfig': {
                'callback': menuconfig,
                'help': 'Run "menuconfig" project configuration tool.',
                'options': global_options + [
                    {
                        'names': ['--style', '--color-scheme', 'style'],
                        'help': (
                            'Menuconfig style.\n'
                            'The built-in styles include:\n\n'
                            '- default - a yellowish theme,\n\n'
                            '- monochrome -  a black and white theme, or\n\n'
                            '- aquatic - a blue theme.\n\n'
                            'It is possible to customize these themes further'
                            ' as it is described in the Color schemes section of the kconfiglib documentation.\n'
                            'The default value is \"aquatic\".'),
                        'envvar': 'MENUCONFIG_STYLE',
                        'default': 'aquatic',
                    }
                ],
            },
        }
    }
    clean_actions = {
        'actions': {
            'python-clean': {
                'callback': python_clean,
                'short_help': 'Delete generated Python byte code from the IDF directory',
                'help': (
                    'Delete generated Python byte code from the IDF directory '
                    'which may cause issues when switching between IDF and Python versions. '
                    'It is advised to run this target after switching versions.'),
            },
        }
    }
    help_action = {
        'actions': {
            'help': {
                'callback': help_and_exit,
                'help': 'Show help message and exit.',
                'hidden': True,
                'options': [
                    {
                        'names': ['--json', 'json_option'],
                        'is_flag': True,
                        'help': 'Print out actions in machine-readable format for selected target.'
                    },
                    {
                        'names': ['--add-options'],
                        'is_flag': True,
                        'help': 'Add options about actions to machine-readable format.'
                    }
                ],
            }
        }
    }

    return merge_action_lists(root_options, build_actions, clean_actions, help_action)
