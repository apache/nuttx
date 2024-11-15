#!/usr/bin/env python
# coding=utf-8
# Created Info: 2024/10/31, Xi'an
#
import os
import subprocess
import sys
import shutil
import argparse
from typing import List, Tuple

from .utils import fatal, warn, info

IDF_PYTHON_TOOLS_NAME = 'idf-python-3.11.2-embed-win64.zip'
IDF_PYTHON_VENV_PATH = 'python_env'
IDF_PYTHON_CONSTRAINT_FILE = 'idf.constraints.txt'
IDF_PYTHON_REQUIREMENTS_PATH = 'requirements'

def get_python_exe_and_subdir() -> Tuple[str, str]:
    """
    Returns:
    * python executable name (python on Unix, python.exe on Win)
    * subdir in which the executable is (bin for Unix, Scripts for Win)
    """
    if sys.platform == 'win32':
        subdir = 'Scripts'
        python_exe = 'python.exe'
    else:
        subdir = 'bin'
        python_exe = 'python'
    return python_exe, subdir

def get_constraints(online: bool = True) -> str:
    idf_path = os.environ['IDF_PATH']
    constraint_path = os.path.join(idf_path, IDF_PYTHON_CONSTRAINT_FILE)

    if not online:
        if os.path.isfile(constraint_path):
            return constraint_path
        else:
            # 检查下报错，实在不行删除文件，重新安装吧！
            fatal(f'{constraint_path} is not exist!!!')
            raise SystemExit(1)
    # 人力不足，无法实现
    raise NotImplementedError("Handling online constraints is not implemented yet.")

def get_requirements() -> List[str]:
    """
    Returns list of path for requirements.txt.
    """
    list = []
    idf_path = os.environ['IDF_PATH']
    requirements_path = os.path.join(idf_path, IDF_PYTHON_REQUIREMENTS_PATH)
    try:
        if os.path.exists(requirements_path):
            for root, _, files in os.walk(requirements_path):
                for f in files:
                    if f.startswith('requirements') and f.endswith('.txt'):
                        list.append(os.path.join(root, f))
    except OSError as error:
        fatal(f'{str(error)}')
        raise SystemExit(1)
    return list

# export command function
def install_python_env(args: argparse.Namespace) -> None:
    '''
    Call install_python_env before,
    Need IDF_DIST_PATH, IDF_TOOLS_PATH, IDF_PATH in os.environ.
    '''
    # 1. 检查python工具是否存在
    idf_dist_path = os.environ.get('IDF_DIST_PATH')
    if not os.path.exists(idf_dist_path):
        fatal('a bad IDF_DIST_PATH:{idf_dist_path}!!!')
        raise SystemExit(1)
    idf_dist_path = os.path.join(idf_dist_path, IDF_PYTHON_TOOLS_NAME)
    if not os.path.isfile(idf_dist_path):
        fatal(f'{idf_dist_path} is bad file!!!')
        raise SystemExit(1)

    # 2. 处理工作工具目录存在情况
    idf_tool_path = os.environ.get('IDF_TOOLS_PATH')
    if os.path.exists(idf_tool_path):
        warn(f'{idf_tool_path} is already exist')
    else:
        try:
            os.makedirs(idf_tool_path, exist_ok=False)
            info(f'create {idf_tool_path}')
        except OSError as error:
            fatal(f'{str(error)}')
            raise SystemExit(1)

    # 3. 处理python虚拟环境存在情况
    idf_python_path = os.path.join(idf_tool_path, IDF_PYTHON_VENV_PATH)           
    if os.path.exists(idf_python_path):
        # 对不住了，我实在时间有限，只能到这里了。
        shutil.rmtree(idf_python_path)
        info(f'del {idf_python_path}')
    try:
        os.makedirs(idf_python_path, exist_ok=False)
        info(f'create {idf_python_path}')
    except OSError as error:
        fatal(f'{str(error)}')
        raise SystemExit(1)

    # 4. 创建python虚拟环境
    python_exe, subdir = get_python_exe_and_subdir()
    idf_export_python_path = os.path.join(idf_python_path, subdir)
    virtualenv_python = os.path.join(idf_export_python_path, python_exe)

    is_virtualenv = hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix)
    # 这里自己想办法解决
    if is_virtualenv:
        fatal('This script was called from a virtual environment, can not create a virtual environment again')
        raise SystemExit(1)

    if subprocess.run([sys.executable, '-m', 'venv', '-h'], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        info(f'Creating a new Python environment in {idf_python_path}')
        virtualenv_options = ['--clear']  # delete environment if already exists
        subprocess.check_call([sys.executable, '-m', 'venv',
                                  *virtualenv_options,
                                  idf_python_path],
                                  stdout=sys.stdout, stderr=sys.stderr)
    else:
        # 这里也没办法，你自己看情况定吧
        pass

    # 5. 处理python虚拟环境PIP问题
    env_copy = os.environ.copy()
    if env_copy.get('PIP_USER')  == 'yes':
        warn('Found PIP_USER="yes" in the environment. Disabling PIP_USER in this shell to install packages into a virtual environment.')
        env_copy['PIP_USER'] = 'no'

    info('Upgrading pip and setuptools...')
    run_args = [virtualenv_python, '-m', 'pip', 'install', '--upgrade', 'pip', 'setuptools']
    constraint_file = get_constraints(online=False)
    if constraint_file:
        run_args += ['--constraint', constraint_file]
    subprocess.check_call(run_args, stdout=sys.stdout, stderr=sys.stderr, env=env_copy)

    run_args = [virtualenv_python, '-m', 'pip', 'install', '--no-warn-script-location']
    requirements_file_list = get_requirements()
    for requirement_file in requirements_file_list:
        run_args += ['-r', requirement_file]
    if constraint_file:
        run_args += ['--upgrade', '--constraint', constraint_file]
        info('Installing Python packages')

    if constraint_file:
        info(f' Constraint file: {constraint_file}')
    info(' Requirement files:')
    info(os.linesep.join(f'  - {path}' for path in requirements_file_list))
    subprocess.check_call(run_args, stdout=sys.stdout, stderr=sys.stderr, env=env_copy)
    print(f'Output:{idf_python_path}')

# export command function
def uninstall_python_env(args: argparse.Namespace) -> None:
    '''
    Call uninstall_python_env before,
    Need IDF_TOOLS_PATH in os.environ.
    '''
    # 1. 检查工作工具目录
    idf_tool_path = os.environ.get('IDF_TOOLS_PATH')
    if not os.path.exists(idf_tool_path):
        warn(f'{idf_tool_path} is not exist')
        return

    # 2. 删除python虚拟环境目录
    idf_python_path = os.path.join(idf_tool_path, IDF_PYTHON_VENV_PATH)           
    if os.path.exists(idf_python_path):
        # 对不住了，我实在时间有限，只能到这里了。
        shutil.rmtree(idf_python_path)
        info(f'del {idf_python_path}')
