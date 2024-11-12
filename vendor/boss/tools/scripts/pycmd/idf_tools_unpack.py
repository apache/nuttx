#!/usr/bin/env python
# coding=utf-8
# Created Info: 2024/11/03, Xi'an
#

import os
import shutil
import sys
from tarfile import TarFile
import tarfile
from typing import Union
from zipfile import ZipFile
from .utils import fatal, rename_with_retry, warn, info

def unpack_gz(filename: str, destination: str) -> None:
    try:
        archive_obj: Union[TarFile, ZipFile] = tarfile.open(filename, 'r:gz')
        if sys.version_info.major == 2:
            # This is a workaround for the issue that unicode destination is not handled:
            # https://bugs.python.org/issue17153
            destination = str(destination)
        archive_obj.extractall(destination)
    except Exception as e:
        fatal(str(e))
        raise SystemExit(1)


def unpack_xz(filename: str, destination: str) -> None:
    raise ValueError('无精力测试')
    try:
        archive_obj = tarfile.open(filename, 'r:xz')
        if sys.version_info.major == 2:
            # This is a workaround for the issue that unicode destination is not handled:
            # https://bugs.python.org/issue17153
            destination = str(destination)
        archive_obj.extractall(destination)
    except Exception as e:
        fatal(str(e))
        raise SystemExit(1)

def unpack_bz2(filename: str, destination: str) -> None:
    raise ValueError('无精力测试')
    try:
        archive_obj = tarfile.open(filename, 'r:bz2')
        if sys.version_info.major == 2:
            # This is a workaround for the issue that unicode destination is not handled:
            # https://bugs.python.org/issue17153
            destination = str(destination)
        archive_obj.extractall(destination)
    except Exception as e:
        fatal(str(e))
        raise SystemExit(1)

def unpack_zip(filename: str, destination: str) -> None:
    try:
        archive_obj = ZipFile(filename)
        common_prefix = os.path.commonprefix(archive_obj.namelist())
        if common_prefix.endswith('/'):
            common_prefix = common_prefix[:-1]
        if not common_prefix:
            raise ValueError('无法确定zip文件的顶层目录')
        if sys.version_info.major == 2:
            # This is a workaround for the issue that unicode destination is not handled:
            # https://bugs.python.org/issue17153
            destination = str(destination)
        output = os.path.join(destination, common_prefix)
        if os.path.isdir(output):
            info(f'del {output}')
            shutil.rmtree(output)
        archive_obj.extractall(destination)
        info(f'Output: {output}')
    except Exception as e:
        fatal(str(e))
        raise SystemExit(1)

# export command function
def unpack(filename: str, destination: str) -> None:
    """
    Extracts file specified by filename into destination depending on its type.
    """
    if not os.path.isfile(filename):
        raise FileNotFoundError

    info(f'Extracting {filename} to {destination}')
    if filename.endswith(('.tar.gz', '.tgz')):
        unpack_gz(filename, destination)
    elif filename.endswith(('.tar.xz')):
        unpack_xz(filename, destination)
    elif filename.endswith(('.tar.bz2')):
        unpack_bz2(filename, destination)
    elif filename.endswith('zip'):
        unpack_zip(filename, destination)
    else:
        raise NotImplementedError('Unsupported archive type')

    # ZipFile on Unix systems does not preserve file permissions while extracting it
    # We need to reset the permissions afterward
    if sys.platform != 'win32' and filename.endswith('zip') and isinstance(archive_obj, ZipFile):
        for file_info in archive_obj.infolist():
            extracted_file = os.path.join(destination, file_info.filename)
            extracted_permissions = file_info.external_attr >> 16 & 0o777  # Extract Unix permissions
            if os.path.exists(extracted_file):
                os.chmod(extracted_file, extracted_permissions)

def do_strip_container_dirs(path: str, levels: int) -> None:
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