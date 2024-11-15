#!/usr/bin/env python
# coding=utf-8
# Created Info: 2024/11/04, Xi'an
#

import argparse
from collections import namedtuple
import copy
import functools
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Callable, Dict, List, Optional, OrderedDict, Set, Tuple, Union

from pycmd.idf_tools_unpack import unpack

from .utils import (PYTHON_PLATFORM, Platforms, fatal, warn, info, CURRENT_PLATFORM, run_cmd_check_output,
                    mkdir_p, get_file_size_sha256, download, rename_with_retry, print_hints_on_download_error,
                    to_shell_specific_paths)

UNKNOWN_VERSION = 'unknown'
IDF_ENV_FILE = 'idf-env.json'
DOWNLOAD_RETRY_COUNT = 3

SUBST_TOOL_PATH_REGEX = re.compile(r'\${TOOL_PATH}')

# refs: do_export
EXPORT_SHELL = 'shell'
EXPORT_KEY_VALUE = 'key-value'

# use for environ variable prompt string
OS_ENV_IDF_PATH_PROMPT = 'IDF_PATH'
OS_ENV_IDF_TOOLS_PROMPT = 'IDF_TOOLS_PATH'
OS_ENV_IDF_NET_ASSETS_PROMPT = 'IDF_NET_ASSETS'

class ToolNotFoundError(RuntimeError):
    """
    Raise when the tool is not found (not present in the paths etc.).
    """
    pass


class ToolExecError(RuntimeError):
    """
    Raise when  the tool returns with a non-zero exit code.
    """
    pass


class ToolBinaryError(RuntimeError):
    """"
    Raise when an error occurred when running any version of the tool.
    """
    pass


class _ENVState:
    """
    ENVState is used to handle IDF global variables that are set in environment and need to be removed when switching between ESP-IDF versions in opened shell.
    Every opened shell/terminal has it's own temporary file to store these variables.
    The temporary file's name is generated automatically with suffix 'idf_ + opened shell ID'. Path to this tmp file is stored as env global variable (env_key).
    The shell ID is crucial, since in one terminal can be opened more shells.
    * env_key - global variable name/key
    * deactivate_file_path - global variable value (generated tmp file name)
    * idf_variables - loaded IDF variables from file
    """
    env_key = 'IDF_DEACTIVATE_FILE_PATH'
    deactivate_file_path = os.environ.get(env_key, '')

    def __init__(self) -> None:
        self.idf_variables: Dict[str, Any] = {}

    @classmethod
    def get_env_state(cls) -> '_ENVState':
        env_state_obj = cls()

        if cls.deactivate_file_path:
            try:
                with open(cls.deactivate_file_path, 'r') as fp:
                    env_state_obj.idf_variables = json.load(fp)
            except (IOError, OSError, ValueError):
                pass
        return env_state_obj

    def save(self) -> str:
        try:
            if self.deactivate_file_path and os.path.basename(self.deactivate_file_path).endswith(f'idf_{str(os.getppid())}'):
                # If exported file path/name exists and belongs to actual opened shell
                with open(self.deactivate_file_path, 'w') as w:
                    json.dump(self.idf_variables, w, ensure_ascii=False, indent=4)  # type: ignore
            else:
                with tempfile.NamedTemporaryFile(delete=False, suffix=f'idf_{str(os.getppid())}') as fp:
                    self.deactivate_file_path = fp.name
                    fp.write(json.dumps(self.idf_variables, ensure_ascii=False, indent=4).encode('utf-8'))
        except (IOError, OSError):
            warn(f'File storing IDF env variables {self.deactivate_file_path} is not accessible to write. '
                 'Potentional switching ESP-IDF versions may cause problems')
        return self.deactivate_file_path


class _IDFToolDownload(object):
    """
    Structure to store all the relevant information about particular download.
    """
    def __init__(self, platform_name: str, url: str, size: int, sha256: str, rename_dist: str) -> None:
        self.platform_name = platform_name
        self.url = url
        self.size = size
        self.sha256 = sha256
        self.rename_dist = rename_dist


@functools.total_ordering
class _IDFToolVersion(object):
    """
    Used for storing information about version; status (recommended, supported, deprecated)
    and easy way of comparing different versions. Also allows platform compatibility check
    and getting right download for given platform, if available.
    """
    STATUS_RECOMMENDED = 'recommended'
    STATUS_SUPPORTED = 'supported'
    STATUS_DEPRECATED = 'deprecated'

    STATUS_VALUES = [STATUS_RECOMMENDED, STATUS_SUPPORTED, STATUS_DEPRECATED]

    def __init__(self, version: str, status: str) -> None:
        self.version = version
        self.status = status
        self.downloads: OrderedDict[str, _IDFToolDownload] = OrderedDict()
        self.latest = False


    def __lt__(self, other: '_IDFToolVersion') -> bool:
        if self.status != other.status:
            return self.status > other.status
        else:
            assert not (self.status == _IDFToolVersion.STATUS_RECOMMENDED
                        and other.status == _IDFToolVersion.STATUS_RECOMMENDED)
            return self.version < other.version


    def __eq__(self, other: object) -> bool:
        if not isinstance(other, _IDFToolVersion):
            return NotImplemented
        return self.status == other.status and self.version == other.version


    def add_download(self, platform_name: str, url: str, size: int, sha256: str, rename_dist: str = '') -> None:
        """
        Add download entry of type IDFToolDownload into self.downloads.
        """
        self.downloads[platform_name] = _IDFToolDownload(platform_name, url, size, sha256, rename_dist)


    def get_download_for_platform(self, platform_name: Optional[str]) -> Optional[_IDFToolDownload]:
        """
        Get download for given platform if usable download already exists.
        """
        try:
            platform_name = Platforms.get(platform_name)
            if platform_name in self.downloads.keys():
                return self.downloads[platform_name]
        # exception can be omitted, as not detected platform is handled without err message
        except ValueError:
            pass
        if 'any' in self.downloads.keys():
            return self.downloads['any']
        return None


    def compatible_with_platform(self, platform_name: Optional[str] = PYTHON_PLATFORM) -> bool:
        """
        Check whether this version is compatible with given platform name.
        """
        return self.get_download_for_platform(platform_name) is not None


    def get_supported_platforms(self) -> Set[str]:
        """
        Get all platforms for which this version has a valid download record.
        """
        return set(self.downloads.keys())


_IDFToolOptions = namedtuple('IDFToolOptions', [
    'version_cmd',
    'version_regex',
    'version_regex_replace',
    'is_executable',
    'export_paths',
    'export_vars',
    'install',
    'info_url',
    'license',
    'strip_container_dirs',
    'supported_targets'])


class _IDFTool(object):
    """
    Used to store info about IDF tools from tools.json file in a Python-accesible form.
    The term "IDF tool" is used for e.g. CMake, ninja, QUEMU and toolchains.
    """
    # possible values of 'install' field
    INSTALL_ALWAYS = 'always'
    INSTALL_ON_REQUEST = 'on_request'
    INSTALL_NEVER = 'never'


    def __init__(self, name: str,
                 description: str,
                 install: str,
                 info_url: str,
                 license: str,
                 version_cmd: List[str],
                 version_regex: str,
                 supported_targets: List[str],
                 version_regex_replace: Optional[str] = None,
                 strip_container_dirs: int = 0,
                 is_executable: bool = True) -> None:
        self.name = name
        self.description = description
        self.drop_versions()
        self.version_in_path: Optional[str] = None
        self.versions_installed: List[str] = []
        self.options = _IDFToolOptions(version_cmd, version_regex, version_regex_replace, is_executable,
                                            [], OrderedDict(), install, info_url, license, strip_container_dirs, supported_targets)  # type: ignore
        self.platform_overrides: List[Dict[str, str]] = []
        self._platform = CURRENT_PLATFORM
        self._update_current_options()
        self.is_executable = is_executable


    def copy_for_platform(self, platform: str) -> '_IDFTool':
        result = copy.deepcopy(self)
        result._platform = platform
        result._update_current_options()
        return result


    def _update_current_options(self) -> None:
        """
        Update current options by platform overrides, if applicable for current platform.
        """
        self._current_options = _IDFToolOptions(*self.options)
        for override in self.platform_overrides:
            if self._platform and self._platform not in override['platforms']:
                continue
            override_dict = override.copy()
            del override_dict['platforms']
            self._current_options = self._current_options._replace(**override_dict)  # type: ignore


    def drop_versions(self) -> None:
        """
        Clear self.versions dictionary.
        """
        self.versions: Dict[str, _IDFToolVersion] = OrderedDict()


    def add_version(self, version: _IDFToolVersion) -> None:
        """
        Add new IDFVersion to self.versions.
        """
        assert type(version) is _IDFToolVersion
        self.versions[version.version] = version

    def get_version(self, extra_paths: Optional[List[str]] = None, executable_path: Optional[str] = None) -> str:
        assert self._platform == CURRENT_PLATFORM

        cmd = self._current_options.version_cmd
        if executable_path:
            cmd[0] = executable_path
        
        if not cmd[0]:
            raise ToolNotFoundError(f'Tool {self.name} not found')
        
        try:
            version_cmd_result = run_cmd_check_output(cmd, None, extra_paths)
        except OSError as e:
            # tool is not on the path
            raise ToolNotFoundError(f'Tool {self.name} not found with error: {e}')
        except subprocess.CalledProcessError as e:
            raise ToolExecError(f'non-zero exit code ({e.returncode}) with message: {e.stderr.decode("utf-8",errors="ignore")}')  # type: ignore

        in_str = version_cmd_result.decode('utf-8')
        match = re.search(self._current_options.version_regex, in_str)  # type: ignore
        if not match:
            return UNKNOWN_VERSION
        if not self._current_options.version_regex_replace:
            return match.group(0)
        else:
            return re.sub(self._current_options.version_regex, self._current_options.version_regex_replace, match.group(0))  # type: ignore

    def get_install_type(self) -> Callable[[str], None]:
        return self._current_options.install


    def find_installed_versions(self) -> None:
        assert self._platform == CURRENT_PLATFORM
        tool_error = False

        try:
            ver_str = self.get_version()
        except ToolNotFoundError:
            # not in PATH
            pass
        except ToolExecError as e:
            fatal(f'tool {self.name} is found in PATH, but has failed: {e}')
            tool_error = True
        else:
            # get version saved in version_in_path.
            self.version_in_path = ver_str

        self.versions_installed = []
        for version, version_obj in self.versions.items():
            if not version_obj.compatible_with_platform():
                continue
            tool_path = self.get_path_for_version(version)
            if not os.path.exists(tool_path):
                # version not installed
                continue
            if not self.is_executable:
                self.versions_installed.append(version)
                continue
            try:
                ver_str = self.get_version(self.get_export_paths(version))
            except ToolNotFoundError as e:
                warn(f'directory for tool {self.name} version {version} is present, but the tool has not been found: {e}')
            except ToolExecError as e:
                fatal(f'tool {self.name} version {version} is installed, but cannot be run: {e}')
                tool_error = True
            else:
                if ver_str != version:
                    warn(f'tool {self.name} version {version} is installed, but has reported version {ver_str}')
                self.versions_installed.append(version)
        if tool_error:
            raise ToolBinaryError


    def get_preferred_installed_version(self) -> Optional[str]:
        """
        Get the preferred installed version of the tool. If more versions installed, return the highest.
        """
        recommended_versions = [k for k in self.versions_installed
                                if self.versions[k].status == _IDFToolVersion.STATUS_RECOMMENDED
                                and self.versions[k].compatible_with_platform(self._platform)]
        assert len(recommended_versions) <= 1
        if recommended_versions:
            return recommended_versions[0]
        return None


    def get_path(self) -> str:
        """
        Returns path where the tool is installed.
        """
        idf_tools_path = os.environ.get(OS_ENV_IDF_TOOLS_PROMPT)
        return os.path.join(idf_tools_path, 'tools', self.name)


    def get_path_for_version(self, version: str) -> str:
        """
        Returns path for the tool of given version.
        """
        assert version in self.versions
        return os.path.join(self.get_path(), version)


    def get_supported_targets(self) -> List[str]:
        """
        Returns list of supported targets with current options.
        """
        return self._current_options.supported_targets  # type: ignore


    def is_supported_for_any_of_targets(self, targets: List[str]) -> bool:
        """
        Checks whether the tool is suitable for at least one of the specified targets.
        """
        supported_targets = self.get_supported_targets()
        return (any(item in targets for item in supported_targets) or supported_targets == ['all'])


    def get_recommended_version(self) -> Optional[str]:
        """
        Get all recommended versions of the tool. If more versions are recommended, highest version is returned.
        """
        recommended_versions = [k for k, v in self.versions.items()
                                if v.status == _IDFToolVersion.STATUS_RECOMMENDED
                                and v.compatible_with_platform(self._platform)]
        assert len(recommended_versions) <= 1
        if recommended_versions:
            return recommended_versions[0]
        return None


    def get_export_paths(self, version: str) -> List[str]:
        """
        Returns a list of paths that need to be exported.
        """
        tool_path = self.get_path_for_version(version)
        return [os.path.join(tool_path, *p) for p in self._current_options.export_paths]  # type: ignore


    def get_export_vars(self, version: str) -> Dict[str, str]:
        """
        Get the dictionary of environment variables to be exported, for the given version.
        Expands:
        - ${TOOL_PATH} => the actual path where the version is installed.
        """
        result = {}
        for k, v in self._current_options.export_vars.items():  # type: ignore
            replace_path = self.get_path_for_version(version).replace('\\', '\\\\')
            v_repl = re.sub(SUBST_TOOL_PATH_REGEX, replace_path, v)
            if v_repl != v:
                v_repl = to_shell_specific_paths([v_repl])[0]
            old_v = os.environ.get(k)
            if old_v is None or old_v != v_repl:
                result[k] = v_repl
        return result


    def check_binary_valid(self, version: str) -> bool:
        if not self.is_executable:
            return True
        try:
            ver_str = self.get_version(self.get_export_paths(version))
        except (ToolNotFoundError, ToolExecError) as e:
            fatal(f'tool {self.name} version {version} is installed, but getting error: {e}')
            return False
        if ver_str != version:
            # just print, state is still valid
            warn(f'tool {self.name} version {version} is installed, but reporting version {ver_str}')
        return True


    def download(self, version: str) -> None:
        """
        Download archive of the tool for platform given by self._platform.
        """
        idf_tools_path = os.environ.get(OS_ENV_IDF_TOOLS_PROMPT)
        assert version in self.versions
        download_obj = self.versions[version].get_download_for_platform(self._platform)
        if not download_obj:
            fatal(f'No packages for tool {self.name} platform {self._platform}!')
            raise SystemExit(1)
        
        url = download_obj.url
        archive_name = download_obj.rename_dist if download_obj.rename_dist else os.path.basename(url)
        local_path = os.path.join(idf_tools_path, 'dist', archive_name)
        mkdir_p(os.path.dirname(local_path))

        if os.path.isfile(local_path):
            if not self.check_download_file(download_obj, local_path):
                warn(f'removing downloaded file {archive_name} and downloading again')
                os.unlink(local_path)
            else:
                info(f'file {archive_name} is already downloaded')
                return
            
        downloaded = False
        local_temp_path = f'{local_path}.tmp'
        for retry in range(DOWNLOAD_RETRY_COUNT):
            err = download(url, local_temp_path)
            if not os.path.isfile(local_temp_path) or not self.check_download_file(download_obj, local_temp_path):
                warn(f'Download failure: {err}')
                warn(f'Failed to download {url} to {local_temp_path}')
                continue
            rename_with_retry(local_temp_path, local_path)
            downloaded = True
            break
        if not downloaded:
            fatal('Failed to download, and retry count has expired')
            print_hints_on_download_error(str(err))
            info(f'\nIf exsist Offline Package, then extract to {local_path}.\n')
            raise SystemExit(1)


    def _do_strip_container_dirs(self, path: str, levels: int) -> None:
        """
        The number of top directory levels specified by levels argument will be removed when extracting.
        E.g. if levels=2, archive path a/b/c/d.txt will be extracted as c/d.txt.
        """
        assert levels > 0
        # move the original directory out of the way (add a .tmp suffix)
        tmp_path = f'{path}.tmp'
        if os.path.exists(tmp_path):
            shutil.rmtree(tmp_path)
        rename_with_retry(path, tmp_path)
        os.mkdir(path)
        base_path = tmp_path
        # walk given number of levels down
        for level in range(levels):
            contents = os.listdir(base_path)
            if len(contents) > 1:
                raise RuntimeError(f'at level {level}, expected 1 entry, got {contents}')
            base_path = os.path.join(base_path, contents[0])
            if not os.path.isdir(base_path):
                raise RuntimeError(f'at level {level}, {contents[0]} is not a directory')
        # get the list of directories/files to move
        contents = os.listdir(base_path)
        for name in contents:
            move_from = os.path.join(base_path, name)
            move_to = os.path.join(path, name)
            rename_with_retry(move_from, move_to)
        shutil.rmtree(tmp_path)


    def install(self, version: str) -> None:
        """
        Unpack archive to destination directory and remove given number of top-level folder if specified.
        """
        idf_tools_path = os.environ.get(OS_ENV_IDF_TOOLS_PROMPT)
        assert version in self.versions
        download_obj = self.versions[version].get_download_for_platform(self._platform)
        assert download_obj is not None
        archive_name = download_obj.rename_dist if download_obj.rename_dist else os.path.basename(download_obj.url)
        archive_path = os.path.join(idf_tools_path, 'dist', archive_name)
        assert os.path.isfile(archive_path)
        dest_dir = self.get_path_for_version(version)
        if os.path.exists(dest_dir):
            warn('destination path already exists, removing')
            shutil.rmtree(dest_dir)
        mkdir_p(dest_dir)
        unpack(archive_path, dest_dir)
        if self._current_options.strip_container_dirs:  # type: ignore
            self._do_strip_container_dirs(dest_dir, self._current_options.strip_container_dirs)  # type: ignore
        if not self.check_binary_valid(version):
            fatal(f'Failed to check the tool while installed. Removing directory {dest_dir}')
            shutil.rmtree(dest_dir)
            raise SystemExit(1)


    @staticmethod
    def check_download_file(download_obj: _IDFToolDownload, local_path: str) -> bool:
        """
        Compare the computed sha256 to sha256 specified by downloaded archive.
        """
        expected_sha256 = download_obj.sha256
        expected_size = download_obj.size
        file_size, file_sha256 = get_file_size_sha256(local_path)
        if file_size != expected_size:
            warn(f'file size mismatch for {local_path}, expected {expected_size}, got {file_size}')
            return False
        if file_sha256 != expected_sha256:
            warn(f'hash mismatch for {local_path}, expected {expected_sha256}, got {file_sha256}')
            return False
        return True


    @classmethod
    def from_json(cls, tool_dict: Dict[str, Union[str, List[str], Dict[str, str]]]) -> '_IDFTool':
        """
        Create _IDFTool class instance form its JSON dump.
        """
        tool_name = tool_dict.get('name')  # type: ignore
        if not isinstance(tool_name, str):
            raise RuntimeError('tool_name is not a string')

        description = tool_dict.get('description')  # type: ignore
        if not isinstance(description, str):
            raise RuntimeError('description is not a string')

        install = tool_dict.get('install', False)  # type: ignore
        if not isinstance(install, str):
            raise RuntimeError(f'install for tool {tool_name} is not a string')

        info_url = tool_dict.get('info_url', False)  # type: ignore
        if not isinstance(info_url, str):
            raise RuntimeError(f'info_url for tool {tool_name} is not a string')

        license = tool_dict.get('license', False)  # type: ignore
        if not isinstance(license, str):
            raise RuntimeError(f'license for tool {tool_name} is not a string')

        version_cmd = tool_dict.get('version_cmd')
        if type(version_cmd) is not list:
            raise RuntimeError(f'version_cmd for tool {tool_name} is not a list of strings')

        is_executable = tool_dict.get('is_executable', True)  # type: ignore
        if not isinstance(is_executable, bool):
            raise RuntimeError(f'is_executable for tool {tool_name} is not a bool')

        version_regex = tool_dict.get('version_regex')
        if not isinstance(version_regex, str) or (not version_regex and is_executable):
            raise RuntimeError(f'version_regex for tool {tool_name} is not a non-empty string')

        supported_targets = tool_dict.get('supported_targets')
        if not isinstance(supported_targets, list):
            raise RuntimeError(f'supported_targets for tool {tool_name} is not a list of strings')

        version_regex_replace = tool_dict.get('version_regex_replace')
        if version_regex_replace and not isinstance(version_regex_replace, str):
            raise RuntimeError(f'version_regex_replace for tool {tool_name} is not a string')

        strip_container_dirs = tool_dict.get('strip_container_dirs', 0)
        if strip_container_dirs and type(strip_container_dirs) is not int:
            raise RuntimeError(f'strip_container_dirs for tool {tool_name} is not an int')

        # other arguements
        export_paths = tool_dict.get('export_paths')
        if type(export_paths) is not list:
            raise RuntimeError(f'export_paths for tool {tool_name} is not a list')

        export_vars = tool_dict.get('export_vars', {})  # type: ignore
        if type(export_vars) is not dict:
            raise RuntimeError(f'export_vars for tool {tool_name} is not a mapping')

        versions = tool_dict.get('versions')
        if type(versions) is not list:
            raise RuntimeError(f'versions for tool {tool_name} is not an array')

        overrides_list = tool_dict.get('platform_overrides', [])  # type: ignore
        if type(overrides_list) is not list:
            raise RuntimeError(f'platform_overrides for tool {tool_name} is not a list')

        # Create the object
        tool_obj: 'IDFTool' = cls(tool_name, description, install, info_url, license,  # type: ignore
                                  version_cmd, version_regex, supported_targets, version_regex_replace,  # type: ignore
                                  strip_container_dirs, is_executable)  # type: ignore

        for path in export_paths:  # type: ignore
            tool_obj.options.export_paths.append(path)  # type: ignore

        for name, value in export_vars.items():  # type: ignore
            tool_obj.options.export_vars[name] = value  # type: ignore

        for index, override in enumerate(overrides_list):
            tool_obj._check_overrides_list(index, override, tool_name)
            tool_obj.platform_overrides.append(override)  # type: ignore

        recommended_versions:Dict[str, List[str]] = {}
        for version_dict in versions:  # type: ignore
            version = version_dict.get('name')  # type: ignore
            if not isinstance(version, str):
                raise RuntimeError(f'version name for tool {tool_name} is not a string')

            version_status = version_dict.get('status')  # type: ignore
            if not isinstance(version_status, str) and version_status not in _IDFToolVersion.STATUS_VALUES:
                raise RuntimeError('tool {} version {} status is not one of {}', tool_name, version,
                                   _IDFToolVersion.STATUS_VALUES)

            version_obj = _IDFToolVersion(version, version_status)
            for platform_id, platform_dict in version_dict.items():  # type: ignore
                if platform_id in ['name', 'status']:
                    continue
                try:
                    Platforms.get(platform_id)
                except ValueError:
                    raise RuntimeError('invalid platform %s for tool %s version %s' %
                                       (platform_id, tool_name, version))

                version_obj.add_download(platform_id,
                                         platform_dict['url'], platform_dict['size'],
                                         platform_dict['sha256'], platform_dict.get('rename_dist', ''))

                if version_status == _IDFToolVersion.STATUS_RECOMMENDED:
                    if platform_id not in recommended_versions:
                        recommended_versions[platform_id] = []
                    recommended_versions[platform_id].append(version)

            tool_obj.add_version(version_obj)
        for platform_id, version_list in recommended_versions.items():
            if len(version_list) > 1:
                raise RuntimeError(f'tool {tool_name} for platform {platform_id} has {len(recommended_versions)} recommended versions')
            if install != _IDFTool.INSTALL_NEVER and len(recommended_versions) == 0:
                raise RuntimeError(f'required/optional tool {tool_name} for platform {platform_id} has no recommended versions')

        tool_obj._update_current_options()
        return tool_obj


    def _check_overrides_list(self, index: int, override: Dict[str, Any], tool_name: str) -> None:
        platforms_list = override.get('platforms')  # type: ignore
        if type(platforms_list) is not list:
            raise RuntimeError('platforms for override %d of tool %s is not a list' % (index, tool_name))

        install = override.get('install')  # type: ignore
        if install is not None and not isinstance(install, str):
            raise RuntimeError('install for override %d of tool %s is not a string' % (index, tool_name))

        version_cmd = override.get('version_cmd')  # type: ignore
        if version_cmd is not None and type(version_cmd) is not list:
            raise RuntimeError('version_cmd for override %d of tool %s is not a list of strings' %
                                (index, tool_name))

        version_regex = override.get('version_regex')  # type: ignore
        if version_regex is not None and (not isinstance(version_regex, str) or not version_regex):
            raise RuntimeError('version_regex for override %d of tool %s is not a non-empty string' %
                                (index, tool_name))

        version_regex_replace = override.get('version_regex_replace')  # type: ignore
        if version_regex_replace is not None and not isinstance(version_regex_replace, str):
            raise RuntimeError('version_regex_replace for override %d of tool %s is not a string' %
                                (index, tool_name))

        export_paths = override.get('export_paths')  # type: ignore
        if export_paths is not None and type(export_paths) is not list:
            raise RuntimeError('export_paths for override %d of tool %s is not a list' % (index, tool_name))

        export_vars = override.get('export_vars')  # type: ignore
        if export_vars is not None and type(export_vars) is not dict:
            raise RuntimeError('export_vars for override %d of tool %s is not a mapping' % (index, tool_name))


def _parse_tools_info_json(tools_info): # type: ignore
    tools_dict = OrderedDict()
    tools_array = tools_info.get('tools')
    if type(tools_array) is not list:
        raise RuntimeError('tools property is missing or not an array')
    
    for tool_dict in tools_array:
         tool = _IDFTool.from_json(tool_dict)
         tools_dict[tool.name] = tool

    return tools_dict


def _load_tools_info(file: str) -> Dict[str, _IDFTool]:
    with open(file, 'r') as f:
        tools_info = json.load(f)
    return _parse_tools_info_json(tools_info) # type: ignore


def _filer_tools_info(targets: List[str], tools_info: Dict[str, _IDFTool]) -> Dict[str, _IDFTool]:
    if not targets:
        return tools_info
    else:
        filtered_tools_spec = {k:v for k, v in tools_info.items() if
                            #    (v.get_install_type() == _IDFTool.INSTALL_ALWAYS or v.get_install_type() == _IDFTool.INSTALL_ON_REQUEST) and
                               (v.get_install_type() == _IDFTool.INSTALL_ALWAYS) and
                               (any(item in targets for item in v.get_supported_targets()) or v.get_supported_targets() == ['all'])}
        return OrderedDict(filtered_tools_spec)


def _get_need_tools_by_agrs(tool_list: List[str], overall_tools: OrderedDict, targets: List[str]) -> List[str]:
    tools = []
    if 'required' in tool_list:
        tools.extend([k for k, v in overall_tools.items() if v.get_install_type() == _IDFTool.INSTALL_ALWAYS and k not in tools])

    elif 'all' in tool_list:
        tools.extend([k for k, v in overall_tools.items() if v.get_install_type() != _IDFTool.INSTALL_NEVER and k not in tools])

    tools = [k for k in tools if overall_tools[k].is_supported_for_any_of_targets(targets)]
    return tools


def _handle_recommended_version_to_use(
    tool: _IDFTool,
    tool_name: str,
    version_to_use: str,
    prefer_system_hint: str,
) -> Tuple[list, dict]:
    """
    If there is unsupported tools version in PATH, prints info about that.
    """
    tool_export_paths = tool.get_export_paths(version_to_use)
    tool_export_vars = tool.get_export_vars(version_to_use)
    if tool.version_in_path and tool.version_in_path not in tool.versions:
        info(f'Not using an unsupported version of tool {tool.name} found in PATH: {tool.version_in_path}.' + prefer_system_hint, f=sys.stderr)
    return tool_export_paths, tool_export_vars


def _process_tool(
    tool: _IDFTool,
    tool_name: str,
    args: argparse.Namespace,
    install_cmd: str,
    prefer_system_hint: str
) -> Tuple[list, dict, bool]:
    """
    Helper function used only in action export.
    Returns:
        * Paths that need to be exported.
        * Dictionary of environment variables that need to be exported for the tool.
        * Flag if any tool was found.
    """
    tool_found: bool = True
    tool_export_paths: List[str] = []
    tool_export_vars: Dict[str, str] = {}

    try:
        tool.find_installed_versions()
    except ToolBinaryError:
        pass
    recommended_version_to_use = tool.get_preferred_installed_version()

    if not tool.is_executable and recommended_version_to_use:
        tool_export_vars = tool.get_export_vars(recommended_version_to_use)
        return tool_export_paths, tool_export_vars, tool_found

    if recommended_version_to_use:
        tool_export_paths, tool_export_vars = _handle_recommended_version_to_use(
            tool, tool_name, recommended_version_to_use, prefer_system_hint
        )
        return tool_export_paths, tool_export_vars, tool_found
    
    info(f'兄弟，异常了，咳血看代码吧！！！')
    return tool_export_paths, tool_export_vars, False


def _get_export_format_and_separator(args: List[str]) -> Tuple[str, str]:
    """
    Returns export pattern (formatted string) either for exporting in shell or as a key-value pair.
    """
    return {EXPORT_SHELL: ('export {}="{}"', ';'), EXPORT_KEY_VALUE: ('{}={}', '\n')}[args.format]  # type: ignore


def _add_variables_to_deactivate_file(args: List[str], new_idf_vars:Dict[str, Any]) -> str:
    """
    Add IDF global variables that need to be removed when the active esp-idf environment is deactivated.
    """
    if 'PATH' in new_idf_vars:
        if sys.platform == 'win32':
            new_idf_vars['PATH'] = new_idf_vars['PATH'].split(';')[:-1]  # PATH is stored as list of sub-paths without '$PATH'
        else:
            new_idf_vars['PATH'] = new_idf_vars['PATH'].split(':')[:-1]  # PATH is stored as list of sub-paths without '$PATH'

    new_idf_vars['PATH'] = new_idf_vars.get('PATH', [])
    args_add_paths_extras = vars(args).get('add_paths_extras')  # remove mypy error with args
    new_idf_vars['PATH'] = new_idf_vars['PATH'] + args_add_paths_extras.split(':') if args_add_paths_extras else new_idf_vars['PATH']

    env_state_obj = _ENVState.get_env_state()

    if env_state_obj.idf_variables:
        exported_idf_vars = env_state_obj.idf_variables
    else:
        env_state_obj.idf_variables = new_idf_vars
    deactivate_file_path = env_state_obj.save()

    return deactivate_file_path


# export command function
def do_export(args: argparse.Namespace, tools_file: str, targets: List[str]) -> None:
    tools_info = _load_tools_info(tools_file)
    tools_info = _filer_tools_info(targets, tools_info)
    all_tools_found = True
    export_vars: Dict[str, str] = {}
    paths_to_export = []

    self_restart_cmd = f'{sys.executable} {__file__}{(f" --tools-json {tools_file}") if tools_file else ""}'
    self_restart_cmd = to_shell_specific_paths([self_restart_cmd])[0]
    prefer_system_hint = f' To use it, run \'{self_restart_cmd} export --prefer-system\''
    install_cmd = f'{self_restart_cmd} install'

    for name, tool in tools_info.items():
        if tool.get_install_type() == _IDFTool.INSTALL_NEVER:
            continue

        tool_export_paths, tool_export_vars, tool_found = _process_tool(tool, name, args, install_cmd, prefer_system_hint)
        if not tool_found:
            all_tools_found = False
        paths_to_export += tool_export_paths
        export_vars = {**export_vars, **tool_export_vars}

    if not all_tools_found:
        # 到这里为止了吧，兄弟！！！
        raise SystemExit(1)

    current_path = os.getenv('PATH')

    idf_path = os.environ.get(OS_ENV_IDF_PATH_PROMPT)
    idf_tools_dir = os.path.join(idf_path, 'scripts')  # type: ignore
    idf_tools_dir = to_shell_specific_paths([idf_tools_dir])[0]
    if current_path and idf_tools_dir not in current_path:
        paths_to_export.append(idf_tools_dir)

    if sys.platform == 'win32':
        old_path = '%PATH%'
        path_sep = ';'
    else:
        old_path = '$PATH'
        path_sep = ':'

    export_format, export_sep = _get_export_format_and_separator(args)

    if paths_to_export:
        export_vars['PATH'] = path_sep.join(to_shell_specific_paths(paths_to_export) + [old_path])
        # Correct PATH order check for Windows platform
        # idf-exe has to be before \tools in PATH
        if sys.platform == 'win32':
            paths_to_check = rf'{export_vars["PATH"]}{os.environ["PATH"]}'
            try:
                if paths_to_check.index(r'\tools;') < paths_to_check.index(r'\idf-exe'):
                    warn('The PATH is not in correct order (idf-exe should be before esp-idf\\tools)')
            except ValueError:
                fatal(f'Both of the directories (..\\idf-exe\\.. and ..\\tools) has to be in the PATH:\n\n{paths_to_check}\n')

    if export_vars:
        # if not copy of export_vars is given to function, it brekas the formatting string for 'export_statements'
        deactivate_file_path = _add_variables_to_deactivate_file(args, export_vars.copy())
        export_vars[_ENVState.env_key] = deactivate_file_path
        export_statements = export_sep.join([export_format.format(k, v) for k, v in export_vars.items()])
        print(export_statements)


def _get_idf_download_url_apply_mirrors(download_url: str) -> str:
    """
    Returns URL for FREE-IDF download with applied mirrors if available.
    If original URL pointed to Github and IDF_GITHUB_ASSETS is set, change the source to Espressif's download servers.
    """
    new_url = download_url
    net_assets = os.environ.get(OS_ENV_IDF_NET_ASSETS_PROMPT, '').strip()
    if not net_assets:
        # no IDF_GITHUB_ASSETS or variable exists but is empty
        return new_url
    
    # check no URL qualifier in the mirror URL
    if '://' in net_assets:
        fatal("IDF_GITHUB_ASSETS shouldn't include any URL qualifier, https:// is assumed")
        raise SystemExit(1)

    # Strip any trailing / from the mirror URL
    net_assets = net_assets.rstrip('/')

    new_url = re.sub(r'^https://github.com/.*?/', f'https://{net_assets}/', download_url)
    if new_url != download_url:
        info(f'Using GitHub assets mirror for URL: {download_url} => {new_url}')
    return new_url


# export command function
def do_download(platform: str, tools: List[str], targets: List[str], tools_file: str) -> None:
    info(f"platform:{platform}, tools:{tools}, targets:{targets}, tools_file:${tools_file}")

    try:
        platform = Platforms.get(platform)
    except ValueError as e:
        fatal(str(e))
        raise SystemExit(1)
    
    tools_info = _load_tools_info(tools_file)

    # 获取指定平台的所有tool_obj
    tools_info_for_platform = OrderedDict()
    for name, obj in tools_info.items():
        tool_for_platform = obj.copy_for_platform(platform)
        tools_info_for_platform[name] = tool_for_platform

    tools_list = _get_need_tools_by_agrs(tools, tools_info_for_platform, targets)
    info(f'Downloading tools for {platform}: {", ".join(tools_list)}')
    for tool_name in tools_list:
        if tool_name not in tools_info_for_platform:
            fatal(f'unknown tool name: {tool_name}')
            raise SystemExit(1)

        tool_obj = tools_info_for_platform[tool_name]
        tool_version = tool_obj.get_recommended_version()
        if tool_version is None:
            fatal(f'tool {tool_name} not found for {platform} platform')
            raise SystemExit(1)

        info(f'Downloading {tool_name}@{tool_version}')
        _idf_tool_obj = tool_obj.versions[tool_version].get_download_for_platform(platform)
        _idf_tool_obj.url = _get_idf_download_url_apply_mirrors(_idf_tool_obj.url)

        tool_obj.download(tool_version)
        tool_obj.install(tool_version)
