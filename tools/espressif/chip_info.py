############################################################################
# tools/espressif/chip_info.py
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
import os
import platform
import re
import subprocess

TOPDIR = "."
ARCH_ESP_HALDIR = "{TOPDIR}/arch/{ARCH_DIR}/src/chip/esp-hal-3rdparty"


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


def run_espressif_tool(cmd):
    tool = "esptool.py"
    strings_out = ""
    command = "{} {}".format(tool, cmd)

    strings_out = subprocess.run(
        command, universal_newlines=True, shell=True, capture_output=True
    )

    return strings_out.stdout


def get_espressif_chip_id():
    output = run_espressif_tool("chip_id")
    string_out = next((s for s in output.split("\n") if "Chip ID" in s), "Not found")
    if string_out != "Not found":
        string_out = string_out.split("Warning: ")[-1]
    return string_out


def get_espressif_flash_id():
    strings_out = []
    output = run_espressif_tool("flash_id")

    strings_out.append(
        next(
            (s for s in output.split("\n") if "Manufacturer" in s),
            "Manufacturer: Not found",
        )
    )
    strings_out.append(
        next((s for s in output.split("\n") if "Device" in s), "Device: Not found")
    )

    return strings_out


def get_espressif_security_info():
    output = run_espressif_tool("get_security_info")

    start_string = "====================="
    stop_string = "Hard resetting via RTS pin..."
    output = output.split("\n")
    strings_out = []

    str_out = next((s for s in output if start_string in s), "Not found")
    if str_out != "Not found":
        start_index = output.index(start_string) + 1
        stop_index = output.index(stop_string)
        strings_out = output[start_index:stop_index]
    else:
        strings_out.append(str_out)

    return strings_out


def get_espressif_flash_status():
    output = run_espressif_tool("read_flash_status")

    string_out = next(
        (s for s in output.split("\n") if "Status value" in s), "Not found"
    )
    if string_out != "Not found":
        string_out = string_out.split("Status value: ")[-1]
    return string_out


def get_espressif_mac_address():
    output = run_espressif_tool("read_mac")

    string_out = next((s for s in output.split("\n") if "MAC" in s), "Not found")
    if string_out != "Not found":
        string_out = string_out.split("MAC: ")[-1]
    return string_out


def get_espressif_bootloader_version(bindir):
    """
    Get the bootloader version for Espressif chips from the bootloader binary. This
    function works on Linux, Windows, and macOS.

    Args:
        bindir (str): The path to the bootloader binary directory.

    Returns:
        dict: A dictionary containing the bootloader version for each chip.
    """

    regex = r"^(?=.*\bv\d+(\.\d+){1,2}\b).+$"
    bootloader_chips = [
        "esp32",
        "esp32s2",
        "esp32s3",
        "esp32c2",
        "esp32c3",
        "esp32c6",
        "esp32h2",
    ]
    bootloader_version = {}

    for chip in bootloader_chips:
        file = "bootloader-{}.bin".format(chip)
        path = os.path.join(bindir, file)

        if os.path.isfile(path):
            if platform.system() == "Linux":
                process = subprocess.Popen(["strings", path], stdout=subprocess.PIPE)
            elif platform.system() == "Windows":
                process = subprocess.Popen(
                    [
                        "powershell",
                        "Get-Content -Raw -Encoding Byte {} |".format(path)
                        + " % { [char[]]$_ -join \"\" } | Select-String -Pattern '[\\x20-\\x7E]+'"
                        + " -AllMatches | % { $_.Matches } | % { $_.Value }",
                    ],
                    stdout=subprocess.PIPE,
                )
            elif platform.system() == "Darwin":
                process = subprocess.Popen(
                    ["strings", "-", path], stdout=subprocess.PIPE
                )
            else:
                bootloader_version[chip] = "Not supported on host OS"
                break

            output, error = process.communicate()
            strings_out = output.decode("utf-8", errors="ignore")
            matches = re.finditer(regex, strings_out, re.MULTILINE)

            try:
                bootloader_version[chip] = next(matches).group(0)
            except StopIteration:
                bootloader_version[chip] = "Unknown"

        else:
            bootloader_version[chip] = "Bootloader image not found"

    return bootloader_version


def get_espressif_toolchain_version():
    """
    Get the version of different toolchains used by Espressif chips.

    Args:
        None.

    Returns:
        dict: A dictionary containing the toolchain version for each toolchain.
    """

    toolchain_version = {}
    toolchain_bins = [
        "clang",
        "gcc",
        "xtensa-esp32-elf-gcc",
        "xtensa-esp32s2-elf-gcc",
        "xtensa-esp32s3-elf-gcc",
        "riscv32-esp-elf-gcc",
        "riscv64-unknown-elf-gcc",
    ]

    for binary in toolchain_bins:
        try:
            version_output = subprocess.check_output(
                [binary, "--version"], stderr=subprocess.STDOUT, universal_newlines=True
            )
            version_lines = version_output.split("\n")
            version = version_lines[0].strip()
            toolchain_version[binary] = version
        except (subprocess.CalledProcessError, FileNotFoundError):
            toolchain_version[binary] = "Not found"

    return toolchain_version


def get_espressif_hal_version(hal_dir):
    """
    Get the version of the ESP HAL used by Espressif chips.

    Args:
        None.

    Returns:
        str: The ESP HAL version.
    """

    hal_version = "Not found"

    try:
        if os.path.isdir(os.path.join(hal_dir, ".git")):
            hal_version_output = subprocess.check_output(
                ["git", "describe", "--tags", "--always"],
                cwd=hal_dir,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
            )
            hal_version = hal_version_output.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass

    return hal_version


def get_arch(config):
    """
    Get the architecture type.

    Args:
        config (list): Listed version of .config file.

    Returns:
        str: Architecture name (xtensa or risc-v).
    """

    arch_key = "CONFIG_ARCH="
    arch = [s for s in config if arch_key in s][0].replace(arch_key, "")
    if "risc" in arch:
        return "risc-v"
    return "xtensa"


def check_chip_info(config):
    """
    Get chip info flag to fetch data.

    Args:
        config (list): Listed version of .config file.

    Returns:
        bool: Indication of getting chip information
        bool: Indication of getting chip information on runtime
    """

    chip_info_key = "CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP="
    chip_info_runtime_key = "CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL="
    nxdiag_key = "CONFIG_SYSTEM_NXDIAG="

    chip_info = [s for s in config if chip_info_key in s]
    chip_info_runtime = [s for s in config if chip_info_runtime_key in s]
    nxdiag = [s for s in config if nxdiag_key in s]

    if nxdiag == []:
        return True, False

    return chip_info != [], chip_info_runtime != []


def get_vendor_info(config):
    """
    Get vendor specific information.

    Args:
        config (list): Listed version of .config file.

    Returns:
        str: Parsed vendor specific information for sysinfo.h,
        str: Parsed vendor specific information for print during build
    """

    info = {}
    output = ""
    build_output = ""

    # Espressif bootloader version

    info["ESPRESSIF_BOOTLOADER"] = get_espressif_bootloader_version(TOPDIR)
    output += "#define ESPRESSIF_BOOTLOADER_ARRAY_SIZE {}\n".format(
        len(info["ESPRESSIF_BOOTLOADER"])
    )
    output += "static const char *ESPRESSIF_BOOTLOADER[ESPRESSIF_BOOTLOADER_ARRAY_SIZE] =\n{\n"
    build_output = "Bootloader version:\n"
    for key, value in info["ESPRESSIF_BOOTLOADER"].items():
        output += '  "{}: {}",\n'.format(key, value)
        build_output += "  {}: {},\n".format(key, value)
    output += "};\n\n"
    build_output += "\n\n"

    # Espressif toolchain version

    info["ESPRESSIF_TOOLCHAIN"] = get_espressif_toolchain_version()
    output += "#define ESPRESSIF_TOOLCHAIN_ARRAY_SIZE {}\n".format(
        len(info["ESPRESSIF_TOOLCHAIN"])
    )
    output += (
        "static const char *ESPRESSIF_TOOLCHAIN[ESPRESSIF_TOOLCHAIN_ARRAY_SIZE] =\n{\n"
    )
    build_output += "Toolchain version:\n"
    for key, value in info["ESPRESSIF_TOOLCHAIN"].items():
        output += '  "{}: {}",\n'.format(key, value)
        build_output += "  {}: {},\n".format(key, value)
    output += "};\n\n"
    build_output += "\n\n"

    # Espressif esptool version

    info["ESPRESSIF_ESPTOOL"] = next(
        (s for s in get_python_modules() if "esptool" in s), "Not found"
    )
    output += 'static const char ESPRESSIF_ESPTOOL[] = "{}";\n\n'.format(
        info["ESPRESSIF_ESPTOOL"].split("-")[1]
    )
    build_output += "Esptool version: {}\n\n".format(
        info["ESPRESSIF_ESPTOOL"].split("-")[1]
    )

    # Espressif HAL version

    arch = get_arch(config)
    hal_path = ARCH_ESP_HALDIR.format(TOPDIR=TOPDIR, ARCH_DIR=arch)
    info["ESPRESSIF_HAL"] = get_espressif_hal_version(hal_path)
    output += 'static const char ESPRESSIF_HAL[] = "{}";\n\n'.format(
        info["ESPRESSIF_HAL"]
    )
    build_output += "HAL version: {}\n\n".format(info["ESPRESSIF_HAL"])

    chip_info, chip_runtime = check_chip_info(config)
    if (chip_info and not chip_runtime) and info[
        "ESPRESSIF_ESPTOOL"
    ] not in "Not found":
        info["ESPRESSIF_CHIP_ID"] = get_espressif_chip_id()
        output += 'static const char ESPRESSIF_CHIP_ID[] = "{}";\n\n'.format(
            info["ESPRESSIF_CHIP_ID"]
        )
        build_output += "CHIP ID: = {}\n\n".format(info["ESPRESSIF_CHIP_ID"])

        info["ESPRESSIF_FLASH_ID"] = get_espressif_flash_id()
        output += "#define ESPRESSIF_FLASH_ID_ARRAY_SIZE {}\n".format(
            len(info["ESPRESSIF_FLASH_ID"])
        )
        output += "static const char *ESPRESSIF_FLASH_ID[ESPRESSIF_FLASH_ID_ARRAY_SIZE] =\n{\n"
        build_output += "Flash ID:\n"
        for each_item in info["ESPRESSIF_FLASH_ID"]:
            output += '  "{}",\n'.format(each_item)
            build_output += "  {}\n".format(each_item)
        output += "};\n\n"
        build_output += "\n\n"

        info["ESPRESSIF_SECURITY_INFO"] = get_espressif_security_info()
        output += "#define ESPRESSIF_SECURITY_INFO_ARRAY_SIZE {}\n".format(
            len(info["ESPRESSIF_SECURITY_INFO"])
        )
        output += "static const char *ESPRESSIF_SECURITY_INFO[ESPRESSIF_SECURITY_INFO_ARRAY_SIZE] =\n{\n"
        build_output += "Security information: \n"
        for each_item in info["ESPRESSIF_SECURITY_INFO"]:
            output += '  "{}",\n'.format(each_item)
            build_output += "  {}\n".format(each_item)
        output += "};\n\n"
        build_output += "\n\n"

        info["ESPRESSIF_FLASH_STAT"] = get_espressif_flash_status()
        output += 'static const char ESPRESSIF_FLASH_STAT[] = "{}";\n\n'.format(
            info["ESPRESSIF_FLASH_STAT"]
        )
        build_output += "Flash status: {}\n\n".format(info["ESPRESSIF_FLASH_STAT"])

        info["ESPRESSIF_MAC_ADDR"] = get_espressif_mac_address()
        output += 'static const char ESPRESSIF_MAC_ADDR[] = "{}";\n\n'.format(
            info["ESPRESSIF_MAC_ADDR"]
        )
        build_output += "MAC address:  {}\n\n".format(info["ESPRESSIF_MAC_ADDR"])

    elif chip_runtime:
        output += "#define ESPRESSIF_INFO_SIZE 384\n"
        output += "static char ESPRESSIF_INFO[ESPRESSIF_TOOLCHAIN_ARRAY_SIZE] =\n{\n  0\n};\n\n"

    return output, build_output
