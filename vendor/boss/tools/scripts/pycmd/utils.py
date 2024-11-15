#!/usr/bin/env python
# coding=utf-8
# Created Info: 2024/10/31, Xi'an
#

import errno
import hashlib
import os
import platform
import re
import ssl
import subprocess
import sys
import contextlib
import time

from ssl import SSLContext
from urllib.request import Request, urlopen
from typing import IO, Callable, List, Optional, Tuple, Union
from urllib.error import ContentTooShortError
from urllib.response import addinfourl
from .secure.cert import DL_CERT_DICT

SSL_NON_INTERACTIVE = "" #default null do interactive, else not null

# Log tools: fatal, warn, info
is_log_quiet = False

def fatal(text: str, *args: str) -> None:
    """
    Writes ERROR: + text to sys.stderr.
    """
    if not is_log_quiet:
        sys.stderr.write(f'ERROR: {text}\n', *args)


def warn(text: str, *args: str) -> None:
    """
    Writes WARNING: + text to sys.stderr.
    """
    if not is_log_quiet:
        sys.stderr.write(f'WARNING: {text}\n', *args)


def info(text: str, f: Optional[IO[str]]=None, *args: str) -> None:
    """
    Writes text to a stream specified by second arg, sys.stdout by default.
    """
    if not is_log_quiet:
        if f is None:
            f = sys.stdout
        f.write(f'{text}\n', *args)


def mkdir_p(path: str) -> None:
    """
    Makes directory in given path.
    Suppresses error when directory is already created or path is a path to file.
    """
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno != errno.EEXIST or not os.path.isdir(path):
            raise


def get_file_size_sha256(filename: str, block_size: int=65536) -> Tuple[int, str]:
    """
    Gets file size and its sha256.
    """
    sha256 = hashlib.sha256()
    size = 0
    with open(filename, 'rb') as f:
        for block in iter(lambda: f.read(block_size), b''):
            sha256.update(block)
            size += len(block)
    return size, sha256.hexdigest()


def to_shell_specific_paths(paths_list: List[str]) -> List[str]:
    """
    Converts / (linux)  to \\ (Windows) if called under win32 platform.
    """
    if sys.platform == 'win32':
        paths_list = [p.replace('/', os.path.sep) if os.path.sep in p else p for p in paths_list]

    return paths_list


# Platform info
PYTHON_PLATFORM = f'{platform.system()}-{platform.machine()}'

# Identifiers used in tools.json for different platforms.
PLATFORM_WIN32 = 'win32'
PLATFORM_WIN64 = 'win64'
PLATFORM_MACOS = 'macos'
PLATFORM_MACOS_ARM64 = 'macos-arm64'
PLATFORM_LINUX32 = 'linux-i686'
PLATFORM_LINUX64 = 'linux-amd64'
PLATFORM_LINUX_ARM32 = 'linux-armel'
PLATFORM_LINUX_ARMHF = 'linux-armhf'
PLATFORM_LINUX_ARM64 = 'linux-arm64'


class Platforms:
    """
    Mappings from various other names these platforms are known as, to the identifiers above.
    This includes strings produced from "platform.system() + '-' + platform.machine()", see PYTHON_PLATFORM
    definition above.
    """
    # Mappings from various other names these platforms are known as, to the identifiers above.
    # This includes strings produced from "platform.system() + '-' + platform.machine()", see PYTHON_PLATFORM
    # definition above.
    # This list also includes various strings used in release archives of xtensa-esp32-elf-gcc, OpenOCD, etc.
    PLATFORM_FROM_NAME = {
        # Windows
        PLATFORM_WIN32: PLATFORM_WIN32,
        'Windows-i686': PLATFORM_WIN32,
        'Windows-x86': PLATFORM_WIN32,
        'i686-w64-mingw32': PLATFORM_WIN32,
        PLATFORM_WIN64: PLATFORM_WIN64,
        'Windows-x86_64': PLATFORM_WIN64,
        'Windows-AMD64': PLATFORM_WIN64,
        'x86_64-w64-mingw32': PLATFORM_WIN64,
        'Windows-ARM64': PLATFORM_WIN64,
        # macOS
        PLATFORM_MACOS: PLATFORM_MACOS,
        'osx': PLATFORM_MACOS,
        'darwin': PLATFORM_MACOS,
        'Darwin-x86_64': PLATFORM_MACOS,
        'x86_64-apple-darwin': PLATFORM_MACOS,
        PLATFORM_MACOS_ARM64: PLATFORM_MACOS_ARM64,
        'Darwin-arm64': PLATFORM_MACOS_ARM64,
        'aarch64-apple-darwin': PLATFORM_MACOS_ARM64,
        'arm64-apple-darwin': PLATFORM_MACOS_ARM64,
        # Linux
        PLATFORM_LINUX64: PLATFORM_LINUX64,
        'linux64': PLATFORM_LINUX64,
        'Linux-x86_64': PLATFORM_LINUX64,
        'FreeBSD-amd64': PLATFORM_LINUX64,
        'x86_64-linux-gnu': PLATFORM_LINUX64,
        PLATFORM_LINUX32: PLATFORM_LINUX32,
        'linux32': PLATFORM_LINUX32,
        'Linux-i686': PLATFORM_LINUX32,
        'FreeBSD-i386': PLATFORM_LINUX32,
        'i586-linux-gnu': PLATFORM_LINUX32,
        'i686-linux-gnu': PLATFORM_LINUX32,
        PLATFORM_LINUX_ARM64: PLATFORM_LINUX_ARM64,
        'Linux-arm64': PLATFORM_LINUX_ARM64,
        'Linux-aarch64': PLATFORM_LINUX_ARM64,
        'Linux-armv8l': PLATFORM_LINUX_ARM64,
        'aarch64': PLATFORM_LINUX_ARM64,
        PLATFORM_LINUX_ARMHF: PLATFORM_LINUX_ARMHF,
        'arm-linux-gnueabihf': PLATFORM_LINUX_ARMHF,
        PLATFORM_LINUX_ARM32: PLATFORM_LINUX_ARM32,
        'arm-linux-gnueabi': PLATFORM_LINUX_ARM32,
        'Linux-armv7l': PLATFORM_LINUX_ARM32,
        'Linux-arm': PLATFORM_LINUX_ARM32,
    }

    # List of platforms that are not supported by ESP-IDF
    UNSUPPORTED_PLATFORMS = [
        'Linux-armv6l'
    ]

    @staticmethod
    def detect_linux_arm_platform(supposed_platform: Optional[str]) -> Optional[str]:
        """
        We probe the python binary to check exactly what environment the script is running in.

        ARM platform may run on armhf hardware but having armel installed packages.
        To avoid possible armel/armhf libraries mixing need to define user's
        packages architecture to use the same
        See note section in https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html#index-mfloat-abi

        ARM platform may run on aarch64 hardware but having armhf installed packages
        (it happens if a docker container is running on arm64 hardware, but using an armhf image).
        """
        if supposed_platform not in (PLATFORM_LINUX_ARM32, PLATFORM_LINUX_ARMHF, PLATFORM_LINUX_ARM64):
            return supposed_platform

        # suppose that installed python was built with the right ABI
        with open(sys.executable, 'rb') as f:
            # see ELF header description in https://man7.org/linux/man-pages/man5/elf.5.html, offsets depend on ElfN size
            if int.from_bytes(f.read(4), sys.byteorder) != int.from_bytes(b'\x7fELF', sys.byteorder):
                return supposed_platform  # ELF magic not found. Use the default platform name from PLATFORM_FROM_NAME
            f.seek(18)  # seek to e_machine
            e_machine = int.from_bytes(f.read(2), sys.byteorder)
            if e_machine == 183:  # EM_AARCH64, https://github.com/ARM-software/abi-aa/blob/main/aaelf64/aaelf64.rst
                supposed_platform = PLATFORM_LINUX_ARM64
            elif e_machine == 40:  # EM_ARM, https://github.com/ARM-software/abi-aa/blob/main/aaelf32/aaelf32.rst
                f.seek(36)  # seek to e_flags
                e_flags = int.from_bytes(f.read(4), sys.byteorder)
                if e_flags & 0x400:
                    supposed_platform = PLATFORM_LINUX_ARMHF
                else:
                    supposed_platform = PLATFORM_LINUX_ARM32

        return supposed_platform

    @staticmethod
    def get(platform_alias: Optional[str]) -> str:
        """
        Get a proper platform name based on PLATFORM_FROM_NAME dict.
        """
        if not platform_alias:
            raise ValueError('System platform could not be identified.')

        if platform_alias in Platforms.UNSUPPORTED_PLATFORMS:
            raise ValueError(f'Platform \'{platform_alias}\' is not supported by ESP-IDF.')

        if platform_alias == 'any' and CURRENT_PLATFORM:
            platform_alias = CURRENT_PLATFORM
        platform_name = Platforms.PLATFORM_FROM_NAME.get(platform_alias, None)
        if sys.platform == 'linux':
            platform_name = Platforms.detect_linux_arm_platform(platform_name)

        if not platform_name:
            raise ValueError(f'Support for platform \'{platform_alias}\' hasn\'t been added yet.')

        return platform_name

    @staticmethod
    def get_by_filename(file_name: str) -> str:
        """
        Guess the right platform based on the file name.
        """
        found_alias = ''
        for platform_alias in Platforms.PLATFORM_FROM_NAME:
            # Find the longest alias which matches with file name to avoid mismatching
            if platform_alias in file_name and len(found_alias) < len(platform_alias):
                found_alias = platform_alias
        return Platforms.get(found_alias)


def parse_platform_arg(platform_str: str) -> str:
    """
    Parses platform from input string and checks whether it is a valid platform.
    If not, raises SystemExit exception with error message.
    """
    try:
        platform = Platforms.get(platform_str)
    except ValueError as e:
        fatal(str(e))
        raise SystemExit(1)

    return platform


CURRENT_PLATFORM = parse_platform_arg(PYTHON_PLATFORM)


def executable_exists(args: List) -> bool:
    try:
        subprocess.check_output(args)
        return True

    except Exception:
        return False


def run_cmd_check_output(cmd: List[str], input_text: Optional[str]=None, extra_paths: Optional[List[str]]=None) -> bytes:
    """
    Runs command and checks output for exceptions. If AttributeError or TypeError occurs, function re-runs the process.
    If return code was not 0, subprocess.CalledProcessError is raised, otherwise, the original error is masked.
    Returns both stdout and stderr of the run command.
    """
    # If extra_paths is given, locate the executable in one of these directories.
    # Note: it would seem logical to add extra_paths to env[PATH], instead, and let OS do the job of finding the
    # executable for us. However this does not work on Windows: https://bugs.python.org/issue8557.
    if extra_paths:
        found = False
        extensions = ['']
        if sys.platform == 'win32':
            extensions.append('.exe')
        for path in extra_paths:
            for ext in extensions:
                fullpath = os.path.join(path, cmd[0] + ext)
                if os.path.exists(fullpath):
                    cmd[0] = fullpath
                    found = True
                    break
            if found:
                break

    try:
        input_bytes = None
        if input_text:
            input_bytes = input_text.encode()
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, input=input_bytes)
        return result.stdout + result.stderr
    except (AttributeError, TypeError):
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = p.communicate(input_bytes)
        if p.returncode != 0:
            try:
                raise subprocess.CalledProcessError(p.returncode, cmd, stdout, stderr)
            except TypeError:
                raise subprocess.CalledProcessError(p.returncode, cmd, stdout)
        return stdout + stderr


def _report_progress(count: int, block_size: int, total_size: int) -> None:
    """
    Prints progress (count * block_size * 100 / total_size) to stdout.
    """
    percent = int(count * block_size * 100 / total_size)
    percent = min(100, percent)
    sys.stdout.write('\r%d%%' % percent)
    sys.stdout.flush()


def _splittype(url: str) -> Tuple[Optional[str], str]:
    """
    Splits given url into its type (e.g. https, file) and the rest.
    eg: input:"http://example.com:8080/path/to/resource" return:("http", "//example.com/8080/path/to/resource")
    """
    match = re.match('([^/:]+):(.*)', url, re.DOTALL)
    if match:
        scheme, data = match.groups()
        return scheme.lower(), data
    return None, url


def _urlretrieve_ctx(url: str|Request,
                    filename: str,
                    reporthook: Optional[Callable[[int, int, int], None]]=None,
                    data: Optional[bytes]=None,
                    context: Optional[SSLContext]=None) -> Tuple[str, addinfourl]:
    """
    Retrieve data from given URL. An alternative version of urlretrieve which takes SSL context as an argument.
    """
    url_type, path = _splittype(url)

    # urlopen doesn't have context argument in Python <=2.7.9
    extra_urlopen_args = {}
    if context:
        extra_urlopen_args['context'] = context
    with contextlib.closing(urlopen(url, data, **extra_urlopen_args)) as fp:  # type: ignore
        headers = fp.info()

        # Just return the local path and the "headers" for file://
        # URLs. No sense in performing a copy unless requested.
        if url_type == 'file' and not filename:
            return os.path.normpath(path), headers

        # Handle temporary file setup.
        tfp = open(filename, 'wb')
        with tfp:
            result = filename, headers
            bs = 1024 * 8
            size = int(headers.get('content-length', -1))
            read = 0
            blocknum = 0

            if reporthook:
                reporthook(blocknum, bs, size)

            while True:
                block = fp.read(bs)
                if not block:
                    break
                read += len(block)
                tfp.write(block)
                blocknum += 1
                if reporthook:
                    reporthook(blocknum, bs, size)

        if size >= 0 and read < size:
            raise ContentTooShortError(
                'retrieval incomplete: got only %i out of %i bytes'
                % (read, size), result)

    return result


def download(url: str, destination: str) -> Union[None, Exception]:
    """
    Download from given url and save into given destination.
    URL NET: http://example.com/test.txt
    URL LOCAL: file:///C:/path/test.txt
    """
    info(f'Downloading {url}')
    info(f'Destination: {destination}')
    try:
        for site, cert in DL_CERT_DICT.items():
            # For dl.espressif.com and github.com, add the DigiCert root certificate.
            # This works around the issue with outdated certificate stores in some installations.
            if site in url:
                ctx = ssl.create_default_context()
                ctx.load_verify_locations(cadata=cert)
                break
        else:
            ctx = None

        _urlretrieve_ctx(url, destination, _report_progress if not SSL_NON_INTERACTIVE else None, context=ctx)
        sys.stdout.write('\rDone\n')
        return None
    except Exception as e:
        # urlretrieve could throw different exceptions, e.g. IOError when the server is down
        return e
    finally:
        sys.stdout.flush()


def rename_with_retry(path_from: str, path_to: str) -> None:
    """
    Sometimes renaming a directory on Windows (randomly?) causes a PermissionError.
    This is confirmed to be a workaround:
    https://github.com/espressif/esp-idf/issues/3819#issuecomment-515167118
    https://github.com/espressif/esp-idf/issues/4063#issuecomment-531490140
    https://stackoverflow.com/a/43046729
    """
    retry_count = 20 if sys.platform.startswith('win') else 1
    for retry in range(retry_count):
        try:
            os.rename(path_from, path_to)
            return
        except OSError:
            msg = f'Rename {path_from} to {path_to} failed'
            if retry == retry_count - 1:
                fatal(f'{msg}. Antivirus software might be causing this. Disabling it temporarily could solve the issue.')
                raise
            warn(f'{msg}, retrying...')
            # Sleep before the next try in order to pass the antivirus check on Windows
            time.sleep(0.5)


def print_hints_on_download_error(err: str) -> None:
    """
    Prints hint on download error. Tries to specify the message depending on the error.
    """
    info('Please make sure you have a working Internet connection.')

    if 'CERTIFICATE' in err:
        info('Certificate issues are usually caused by an outdated certificate database on your computer.')
        info('Please check the documentation of your operating system for how to upgrade it.')

        if sys.platform == 'darwin':
            info('Running "./Install\\ Certificates.command" might be able to fix this issue.')

        info(f'Running "{sys.executable} -m pip install --upgrade certifi" can also resolve this issue in some cases.')

    # Certificate issue on Windows can be hidden under different errors which might be even translated,
    # e.g. "[WinError -2146881269] ASN1 valor de tag inválido encontrado"
    if sys.platform == 'win32':
        info('give up it, use 3000m buy a new computer.')

    info('Or, You can download from a private repository on Gitee.'
         '  For openocd-esp32, to https://gitee.com/lmos007/openocd-esp32/releases/tag/v0.12.0-esp32-20240318\n'
         '  For gdb, to https://gitee.com/lmos007/binutils-gdb/releases/tag/esp32ulp-elf-2.38_20240113\n'
         '  For ccache, to https://gitee.com/lmos007/ccache/releases/tag/v4.8\n'
         '  For cmake, to https://gitee.com/lmos007/CMake/releases/tag/v3.24.0\n'
         '  For ninja, to https://gitee.com/lmos007/ninja/releases/tag/v1.11.1\n'
         '  For idf_py_exe_tool, to https://gitee.com/lmos007/idf_py_exe_tool/releases/tag/v1.0.3\n'
         '  For esp-rom-elfs, to https://gitee.com/lmos007/esp-rom-elfs/releases/tag/20240305\n'
         '  For xtensa-esp-elf-gcc/riscv32-esp-elf, to https://gitee.com/lmos007/crosstool-NG/releases/tag/esp-13.2.0_20240530\n'
         '  For esp-clang, to https://gitee.com/lmos007/llvm-project/releases/tag/esp-16.0.0-20230516'
        )
