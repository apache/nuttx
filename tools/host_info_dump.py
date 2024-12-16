#!/usr/bin/env python3
# tools/host_sysinfo.py
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
import importlib.util
import os
import platform
import re
import subprocess
import sys

# Custom argparse actions #


class validate_flags_arg(argparse.Action):
    """
    Custom argparse action to validate the number of parameters passed to the
    --flags argument. Exactly three parameters are required.
    """

    def __call__(self, parser, namespace, values, option_string=None):
        if len(values) != 3:
            raise argparse.ArgumentError(
                self,
                "Invalid number of parameters for --flags. Exactly three parameters are required.",
            )
        setattr(namespace, self.dest, values)


# Common functions #


def verbose(info, logs):
    """
    Prepare and print information on build screen.

    Args:
        info (dict): Information dictionary of build system.
        logs (str): Vendor specific information string.

    Returns:
        None.
    """

    if args.verbose:

        out_str = "\n" + "=" * 79 + "\n"

        out_str += "NuttX RTOS info: {}\n\n".format(info["OS_VERSION"])
        if args.flags:
            out_str += "NuttX CFLAGS: {}\n\n".format(info["NUTTX_CFLAGS"])
            out_str += "NuttX CXXFLAGS: {}\n\n".format(info["NUTTX_CXXFLAGS"])
            out_str += "NuttX LDFLAGS: {}\n\n".format(info["NUTTX_LDFLAGS"])

        if args.config:
            out_str += "NuttX configuration options: \n"
            for i in range(len(info["NUTTX_CONFIG"])):
                out_str += "  " + info["NUTTX_CONFIG"][i].replace('"', '\\"') + "\n"
            out_str += "\n\n"

        if args.path:
            out_str += "Host system PATH: \n"
            for i in range(len(info["SYSTEM_PATH"])):
                out_str += "  " + info["SYSTEM_PATH"][i] + "\n"
            out_str += "\n\n"

        if args.packages:
            out_str += "Host system installed packages: \n"
            for i in range(len(info["INSTALLED_PACKAGES"])):
                out_str += "  " + info["INSTALLED_PACKAGES"][i] + "\n"
            out_str += "\n\n"

        if args.modules:
            out_str += "Host system installed python modules: \n"
            for i in range(len(info["PYTHON_MODULES"])):
                out_str += "  " + info["PYTHON_MODULES"][i] + "\n"
            out_str += "\n\n"

        if args.target_info:
            out_str += "Vendor specific information: \n\n"
            out_str += logs
            out_str += "=" * 79 + "\n"

        eprint(out_str)


def eprint(*args, **kwargs):
    """
    Prints the arguments passed to stderr.
    """

    print(*args, file=sys.stderr, **kwargs)


def get_installed_packages():
    """
    Gets the list of packages installed on the host system. This function works on
    Linux (Debian, Red Hat, and Arch-based distros), Windows, and macOS.

    Args:
        None.

    Returns:
        list: Packages (with version) installed on the host system.
    """

    packages = []

    if platform.system() == "Linux":
        package_managers = [
            {
                "name": "Dpkg",
                "command": ["dpkg", "-l"],
                "skip_lines": 5,
                "info_name": 1,
                "info_version": 2,
            },
            {
                "name": "Rpm",
                "command": ["rpm", "-qa", "--queryformat", '"%{NAME} %{VERSION}\\n"'],
                "skip_lines": 0,
                "info_name": 0,
                "info_version": 1,
            },
            {
                "name": "Pacman",
                "command": ["pacman", "-Q", "--queryformat", '"%n %v\\n"'],
                "skip_lines": 0,
                "info_name": 0,
                "info_version": 1,
            },
        ]

        for manager in package_managers:
            try:
                process = subprocess.Popen(manager["command"], stdout=subprocess.PIPE)
                output, error = process.communicate()
                lines = output.decode("utf-8").splitlines()

                # Skip the specified number of lines based on the package manager
                if lines:
                    lines = lines[manager["skip_lines"] :]

                current_packages = []
                for line in lines:
                    package_info = line.split()
                    package = package_info[manager["info_name"]]
                    version = package_info[manager["info_version"]]
                    current_packages.append(f"{package} ({version})")

                if current_packages:
                    packages.extend(current_packages)
                    break

            except (FileNotFoundError, subprocess.CalledProcessError):
                pass

    elif platform.system() == "Windows":
        try:
            output = subprocess.check_output(
                [
                    "powershell",
                    "Get-ItemProperty HKLM:\\Software\\Microsoft\\Windows\\"
                    + "CurrentVersion\\Uninstall\\* | Select-Object DisplayName, DisplayVersion",
                ]
            )
            output = output.decode("utf-8", errors="ignore").split("\r\n")
            for line in output[3:]:
                line = line.strip()
                if line:
                    match = re.match(r"^(.*?)(\s{2,}[^ ]+)?$", line)
                    if match:
                        name = match.group(1).strip()
                        version = (
                            match.group(2).strip() if match.group(2) else "Unknown"
                        )
                        packages.append(f"{name} ({version})")
        except subprocess.CalledProcessError:
            eprint("Error: Failed to get installed packages.")

    elif platform.system() == "Darwin":
        try:
            output = subprocess.check_output(["pkgutil", "--pkgs"])
            output = output.decode("utf-8").split("\n")
            for package in output:
                if "." in package:
                    try:
                        info = subprocess.check_output(
                            ["pkgutil", "--pkg-info", package]
                        )
                        info = info.decode("utf-8")
                        version = info.split("version: ")[1].split("\n")[0]
                    except subprocess.CalledProcessError:
                        version = "Unknown"
                    packages.append(f"{package} ({version})")
        except subprocess.CalledProcessError:
            eprint("Error: Failed to get installed packages.")

    packages.sort()
    return packages


def get_python_modules():
    """
    Gets the list of python modules installed on the host system.

    Args:
        None.

    Returns:
        list: Python modules (with version) installed on the host system.
    """

    modules = []

    output = subprocess.check_output(
        ["pip", "list", "--format=freeze"], universal_newlines=True
    )
    for line in output.splitlines():
        if line.startswith("#"):
            continue
        package_info = line.split("==")
        if len(package_info) > 1:
            modules.append("{}-{}".format(package_info[0], package_info[1]))
        else:
            modules.append(package_info[0])
    return modules


def get_system_path():
    """
    Gets the PATH environment variable.

    Args:
        None.

    Returns:
        str: The PATH environment variable.
    """

    return os.environ.get("PATH", "")


def get_os_version():
    """
    Gets the OS distribution and version. This function works on Linux, Windows,
    and macOS. On Linux, if the distro package is installed, it will be used to
    get the OS distribution.

    Args:
        None.

    Returns:
        str: The OS distribution and version.
    """

    os_name = platform.system()
    sys_id = ""

    if os_name == "Windows":
        os_distro = "Windows"
        os_version = platform.win32_ver()[0]
        sys_id = f"{os_distro} {os_version}"

    elif os_name == "Darwin":
        os_distro = "macOS"
        os_uname = " ".join(platform.uname())
        os_version = platform.mac_ver()[0]
        sys_id = f"{os_distro} {os_version} {os_uname}"

    elif os_name == "Linux":
        os_uname = " ".join(platform.uname())
        try:
            import distro

            sys_id = distro.name(pretty=True) + " " + os_uname
        except ImportError:
            sys_id = os_uname

    return sys_id


def get_compilation_flags(flags):
    """
    Gets the compilation flags used to compile the NuttX source code and splits
    them into a list.

    Args:
        arg (str): The compilation flags.

    Returns:
        list: The compilation flags.
    """

    if not flags:
        return [""]

    flag_list = flags.split(" -")
    flag_list[0] = flag_list[0][1:]
    flag_list = ["-" + flag for flag in flag_list]

    return flag_list


def generate_header(args):
    """
    Generates the sysinfo.h header file that contains information about the host system
    and NuttX configuration that can be used by NuttX applications.

    Args:
        args (argparse.Namespace): The command line arguments.

    Returns:
        str: The contents of the sysinfo.h header file.
    """

    info = {}
    output = ""
    build_output = ""

    output += "/****************************************************************************\n"
    output += " * sysinfo.h\n"
    output += " *\n"
    output += " * Auto-generated by a Python script. Do not edit!\n"
    output += " *\n"
    output += (
        " * This file contains information about the host and target systems that\n"
    )
    output += " * can be used by NuttX applications.\n"
    output += " *\n"
    output += " ****************************************************************************/\n\n"

    output += "#ifndef __SYSTEM_INFO_H\n"
    output += "#define __SYSTEM_INFO_H\n\n"

    # NuttX Compilation Flags

    if args.flags:
        cflags, cxxflags, ldflags = args.flags

        if cflags:
            cflags = cflags[1:-1]
        if cxxflags:
            cxxflags = cxxflags[1:-1]
        if ldflags:
            ldflags = ldflags[1:-1]

        info["NUTTX_CFLAGS"] = get_compilation_flags(cflags)
        info["NUTTX_CXXFLAGS"] = get_compilation_flags(cxxflags)
        info["NUTTX_LDFLAGS"] = get_compilation_flags(ldflags)

        output += "#define NUTTX_CFLAGS_ARRAY_SIZE {}\n".format(
            len(info["NUTTX_CFLAGS"])
        )
        output += "static const char *NUTTX_CFLAGS[NUTTX_CFLAGS_ARRAY_SIZE] =\n{\n"
        for i in range(len(info["NUTTX_CFLAGS"])):
            output += '  "' + info["NUTTX_CFLAGS"][i] + '",\n'
        output += "};\n\n"

        output += "#define NUTTX_CXXFLAGS_ARRAY_SIZE {}\n".format(
            len(info["NUTTX_CXXFLAGS"])
        )
        output += "static const char *NUTTX_CXXFLAGS[NUTTX_CXXFLAGS_ARRAY_SIZE] =\n{\n"
        for i in range(len(info["NUTTX_CXXFLAGS"])):
            output += '  "' + info["NUTTX_CXXFLAGS"][i] + '",\n'
        output += "};\n\n"

        output += "#define NUTTX_LDFLAGS_ARRAY_SIZE {}\n".format(
            len(info["NUTTX_LDFLAGS"])
        )
        output += "static const char *NUTTX_LDFLAGS[NUTTX_LDFLAGS_ARRAY_SIZE] =\n{\n"
        for i in range(len(info["NUTTX_LDFLAGS"])):
            output += '  "' + info["NUTTX_LDFLAGS"][i] + '",\n'
        output += "};\n\n"

    # NuttX Configuration

    info["NUTTX_CONFIG"] = []
    config_path = os.path.abspath("./.config")
    try:
        with open(config_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                info["NUTTX_CONFIG"].append(line)
    except FileNotFoundError:
        print("Error: NuttX configuration file not found: {}".format(config_path))
        sys.exit(1)

    if args.config:
        output += "#define NUTTX_CONFIG_ARRAY_SIZE {}\n".format(
            len(info["NUTTX_CONFIG"])
        )
        output += "static const char *NUTTX_CONFIG[NUTTX_CONFIG_ARRAY_SIZE] =\n{\n"
        for i in range(len(info["NUTTX_CONFIG"])):
            output += '  "' + info["NUTTX_CONFIG"][i].replace('"', '\\"') + '",\n'
        output += "};\n\n"

    # OS Version

    info["OS_VERSION"] = get_os_version()
    output += 'static const char OS_VERSION[] = "{}";\n\n'.format(info["OS_VERSION"])

    # System Path

    if args.path:
        info["SYSTEM_PATH"] = str(get_system_path()).split(":")
        output += "#define SYSTEM_PATH_ARRAY_SIZE {}\n".format(len(info["SYSTEM_PATH"]))
        output += "static const char *SYSTEM_PATH[SYSTEM_PATH_ARRAY_SIZE] =\n{\n"
        for i in range(len(info["SYSTEM_PATH"])):
            output += '  "' + info["SYSTEM_PATH"][i] + '",\n'
        output += "};\n\n"

    # Installed Packages

    if args.packages:
        info["INSTALLED_PACKAGES"] = [str(x) for x in get_installed_packages()]
        output += "#define INSTALLED_PACKAGES_ARRAY_SIZE {}\n".format(
            len(info["INSTALLED_PACKAGES"])
        )
        output += "static const char *INSTALLED_PACKAGES[INSTALLED_PACKAGES_ARRAY_SIZE] =\n{\n"
        for i in range(len(info["INSTALLED_PACKAGES"])):
            output += '  "' + info["INSTALLED_PACKAGES"][i] + '",\n'
        output += "};\n\n"

    # Python Modules

    if args.modules:
        info["PYTHON_MODULES"] = [str(x) for x in get_python_modules()]
        output += "#define PYTHON_MODULES_ARRAY_SIZE {}\n".format(
            len(info["PYTHON_MODULES"])
        )
        output += "static const char *PYTHON_MODULES[PYTHON_MODULES_ARRAY_SIZE] =\n{\n"
        for i in range(len(info["PYTHON_MODULES"])):
            output += '  "' + info["PYTHON_MODULES"][i] + '",\n'
        output += "};\n\n"

    # Vendor Specific

    if args.target_info:
        arch_chip_key = "CONFIG_ARCH_CHIP="
        arch_chip = [s for s in info["NUTTX_CONFIG"] if arch_chip_key in s][0].split(
            arch_chip_key
        )[-1]
        if "esp" in arch_chip:
            vendor_specific_module_path = os.path.abspath("./tools/espressif")

        sys.path.append(os.path.abspath(vendor_specific_module_path))
        vendor_specific_module = importlib.import_module("chip_info")
        get_vendor_info = getattr(vendor_specific_module, "get_vendor_info")
        vendor_output, vendor_build_output = get_vendor_info(info["NUTTX_CONFIG"])
        output += vendor_output
        build_output += vendor_build_output

    verbose(info, build_output)

    output += "#endif /* __SYSTEM_INFO_H */\n"

    return output


# Main #

if __name__ == "__main__":
    """
    Main function for the script. This function is called when the script is
    executed directly. It parses the command line arguments and calls the
    appropriate functions. It also prints the output generated to stdout.

    Required arguments:
        nuttx_path: The path to the NuttX source directory.

    Optional arguments:
        The command line arguments. The available arguments are:
            -h, --help:
                        Show the help message and exit.
            -m, --modules:
                        Get the list of installed Python modules.
            -k, --packages:
                        Get the list of installed system packages.
            -p, --path:
                        Get the system PATH environment variable.
            -c, --config:
                        Get the NuttX compilation configurations used.
            -f, --flags <CFLAGS> <CXXFLAGS> <LDFLAGS>:
                        Provide the NuttX compilation flags used.
            --verbose:
                        Enable printing information during build.
            --target_info:
                        Enable vendor specific information during build.
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
        "-m",
        "--modules",
        action="store_true",
        help="Get the list of installed Python modules.",
    )
    parser.add_argument(
        "-k",
        "--packages",
        action="store_true",
        help="Get the list of installed system packages.",
    )
    parser.add_argument(
        "-p",
        "--path",
        action="store_true",
        help="Get the system PATH environment variable.",
    )
    parser.add_argument(
        "-c",
        "--config",
        action="store_true",
        help="Get the NuttX compilation configurations used.",
    )
    parser.add_argument(
        "-f",
        "--flags",
        nargs=3,
        default=[],
        metavar=("CFLAGS", "CXXFLAGS", "LDFLAGS"),
        action=validate_flags_arg,
        help="Provide the NuttX compilation and linker flags used.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable printing information during build",
    )

    parser.add_argument(
        "--target_info",
        action="store_true",
        help="Enable vendor specific information during build",
    )

    # Parse arguments

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()
    os.chdir(args.nuttx_path)
    header = generate_header(args)
    print(header)
