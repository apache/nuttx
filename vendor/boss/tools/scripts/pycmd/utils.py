#!/usr/bin/env python
# coding=utf-8
# Created Info: 2024/10/31, Xi'an
#

import platform
import sys
from typing import IO, Optional

PYTHON_PLATFORM = f'{platform.system()}-{platform.machine()}'

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

