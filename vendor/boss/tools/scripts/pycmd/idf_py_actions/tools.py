import os
import re
from typing import Any, Dict, Optional

from pycmd.errors import FatalError
from pycmd.utils import executable_exists


# Name of the program, normally 'idf.py'.
# Can be overridden from idf.bat using IDF_PY_PROGRAM_NAME
PROG = os.getenv('IDF_PY_PROGRAM_NAME', 'idf.py')


def ensure_build_directory(args: 'PropertyDict', prog_name: str, always_run_cmake: bool=False,
                           env: Optional[Dict]=None) -> None:
    """Check the build directory exists and that cmake has been run there.

    If this isn't the case, create the build directory (if necessary) and
    do an initial cmake run to configure it.

    This function will also check args.generator parameter. If the parameter is incompatible with
    the build directory, an error is raised. If the parameter is None, this function will set it to
    an auto-detected default generator or to the value already configured in the build directory.
    """

    if not executable_exists(['cmake', '--version']):
        raise FatalError(f'"cmake" must be available on the PATH to use {PROG}')

    project_dir = args.project_dir
    # Verify the project directory
    if not os.path.isdir(project_dir):
        if not os.path.exists(project_dir):
            raise FatalError('Project directory %s does not exist' % project_dir)
        else:
            raise FatalError('%s must be a project directory' % project_dir)
    if not os.path.exists(os.path.join(project_dir, 'CMakeLists.txt')):
        raise FatalError('CMakeLists.txt not found in project directory %s' % project_dir)

    pass


def merge_action_lists(*action_lists: Dict) -> Dict:
    merged_actions: Dict = {
        'global_options': [],
        'actions': {},
        'global_action_callbacks': [],
    }
    for action_list in action_lists:
        merged_actions['global_options'].extend(action_list.get('global_options', []))
        merged_actions['actions'].update(action_list.get('actions', {}))
        merged_actions['global_action_callbacks'].extend(action_list.get('global_action_callbacks', []))
    return merged_actions


def get_sdkconfig_value(sdkconfig_file: str, key: str) -> Optional[str]:
    """
    Return the value of given key from sdkconfig_file.
    If sdkconfig_file does not exist or the option is not present, returns None.
    """
    assert key.startswith('CONFIG_')
    if not os.path.exists(sdkconfig_file):
        return None
    # keep track of the last seen value for the given key
    value = None
    # if the value is quoted, this excludes the quotes from the value
    pattern = re.compile(r"^{}=\"?([^\"]*)\"?$".format(key))
    with open(sdkconfig_file, 'r') as f:
        for line in f:
            match = re.match(pattern, line)
            if match:
                value = match.group(1)
    return value


def get_target(path: str, sdkconfig_filename: str='sdkconfig') -> Optional[str]:
    path = os.path.join(path, sdkconfig_filename)
    return get_sdkconfig_value(path, 'CONFIG_IDF_TARGET')


class PropertyDict(dict):
    def __getattr__(self, name: str) -> Any:
        if name in self:
            return self[name]
        else:
            raise AttributeError("'PropertyDict' object has no attribute '%s'" % name)

    def __setattr__(self, name: str, value: Any) -> None:
        self[name] = value

    def __delattr__(self, name: str) -> None:
        if name in self:
            del self[name]
        else:
            raise AttributeError("'PropertyDict' object has no attribute '%s'" % name)
