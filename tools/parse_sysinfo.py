#!/usr/bin/env python3
# tools/parse_sysinfo.py
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

import argparse
import os
import re
import sys


def parse_information_from_header(file_path):
    """
    Parses the file that contains information about the host system
    and NuttX configuration(sysinfo.h).

    Args:
        file_path (str): Path of the file to parse.

    Returns:
        dict: The contents parsed from file_path (sysinfo.h) header file.
    """

    VARIABLE_NAMES_REGEX = r"static\s+const\s+char\s+\**([A-Za-z0-9_]+)\s*"
    VARIABLE_VALUES_REGEX = r'"([^"]*)"|{([^}]+)};'
    result = {}
    var_name_to_print_dict = {
        "NUTTX_CFLAGS": "NuttX CFLAGS",
        "NUTTX_CXXFLAGS": "NuttX CXXFLAGS",
        "NUTTX_LDFLAGS": "NuttX LDFLAGS",
        "NUTTX_CONFIG": "NuttX configuration options",
        "SYSTEM_PATH": "Host system PATH",
        "OS_VERSION": "Host system OS",
        "INSTALLED_PACKAGES": "Host system installed packages",
        "PYTHON_MODULES": "Host system installed python modules",
        "ESPRESSIF_BOOTLOADER": "Espressif specific information:\n\nToolchain version",
        "ESPRESSIF_TOOLCHAIN": "Toolchain version",
        "ESPRESSIF_ESPTOOL": "Esptool version",
        "ESPRESSIF_HAL": "HAL version",
        "ESPRESSIF_CHIP_ID": "CHIP ID",
        "ESPRESSIF_FLASH_ID": "Flash ID",
        "ESPRESSIF_SECURITY_INFO": "Security information",
        "ESPRESSIF_FLASH_STAT": "Flash status",
        "ESPRESSIF_MAC_ADDR": "MAC address",
    }

    # Regular expression pattern to match array definition

    keys_pattern = re.compile(VARIABLE_NAMES_REGEX, re.DOTALL)
    values_pattern = re.compile(VARIABLE_VALUES_REGEX, re.DOTALL)

    with open(file_path, "r") as file:
        content = file.read()

        # Match array definition using the regex

        keys_array = keys_pattern.findall(content)
        values_array = values_pattern.findall(content)

        # Process values to print it prettier

        for i in range(len(values_array)):
            tmp_list = []
            for y in range(len(values_array[i])):
                tmp_str = values_array[i][y]
                tmp_str = tmp_str.replace('"', "")
                tmp_str = tmp_str.replace("\n  ", "", 1)
                tmp_str = tmp_str.replace(",", "")

                if tmp_str != "":
                    tmp_list.append(tmp_str)

            values_array[i] = tuple(tmp_list)

        keys_values_to_return = [var_name_to_print_dict[x] for x in keys_array]
        result = dict(zip(keys_values_to_return, values_array))

    return result


# Main #


if __name__ == "__main__":
    """
    Main function for the script. This function is called when the script is
    executed directly. It prints the output generated to stdout.

    Required arguments:
        nuttx_path: The path to the NuttX source directory.

    Optional arguments:
        The command line arguments. The available arguments are:
            -h, --help:
                        Show the help message and exit.
            -f, --file <SYSINFO_FILE>:
                        Provide the diagnostic file output(sysinfo.h).
    """

    # Generic arguments

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("nuttx_path", help="NuttX source directory path.")
    parser.add_argument(
        "-h",
        "--help",
        action="help",
        default=argparse.SUPPRESS,
        help="Show this help message and exit.",
    )
    parser.add_argument(
        "-f",
        "--file",
        default="",
        metavar=("SYSINFO_FILE"),
        help="Provide the diagnostic file output(sysinfo.h).",
    )
    # Parse arguments

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()
    os.chdir(args.nuttx_path)

    parsed_data = parse_information_from_header(args.file)

    # Print the extracted name and values

    if parsed_data:
        for each_key in parsed_data.keys():
            print("{}:".format(each_key))
            for each_value in parsed_data[each_key]:
                print("  {}".format(each_value))
            print("")
    else:
        print("No matching array found.")
