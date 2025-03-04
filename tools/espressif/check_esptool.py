############################################################################
# tools/espressif/check_esptool.py
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################
import argparse
import re
import sys

# Different package required if Python 3.8+
if sys.version_info.minor < 8:
    import pkg_resources

    PYTHON_OLDER = True
else:
    from importlib.metadata import PackageNotFoundError, version

    PYTHON_OLDER = False


parser = argparse.ArgumentParser(
    prog="check_esptool",
    description="Checks esptool version and returns true or \
    false if it matches target version",
)
parser.add_argument("-v", "--version", action="store", required=True)


def parse_version(version_string) -> list:
    """Regex patteren to extract version major and minor."""
    pattern = r"(\d+)\.(\d+)"
    match = re.search(pattern, version_string)

    if match:
        major = int(match.group(1))
        minor = int(match.group(2))
        return [major, minor]
    else:
        return []


def check_version(min_esptool_version: str) -> int:
    """Attempts to read 'esptool' version using pkg_resources (for
    Python <3.8) or importlib. Compare current version with
    'min_esptool_version' and returns.

    Returns:
        True: packages does not exist or outdated
        False: package installed and up-to-date
    """
    if PYTHON_OLDER:
        try:
            version_str = pkg_resources.get_distribution("esptool").version
        except pkg_resources.DistributionNotFound:
            print("esptool.py not found. Please run: 'pip install esptool'")
            print("Run make again to create the nuttx.bin image.")
            return True
    else:
        try:
            version_str = version("esptool")
        except PackageNotFoundError:
            print("esptool.py not found. Please run: 'pip install esptool'")
            print("Run make again to create the nuttx.bin image.")
            return True

    esptool_version = parse_version(version_str)
    min_esptool_version = parse_version(parser.version)

    if esptool_version >= min_esptool_version:
        return False

    print("Unsupported esptool version:", version_str, "expects >=", parser.version)
    print("Upgrade using: 'pip install --upgrade esptool' and run 'make' again")
    return True


if __name__ == "__main__":
    parser = parser.parse_args()
    sys.exit(check_version(parser.version))
