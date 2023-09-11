#!/usr/bin/env python3
############################################################################
# tools/merge_config.py
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
import os

try:
    from kconfiglib import Kconfig
except ModuleNotFoundError:
    print("Please execute the following command to install dependencies:")
    print("pip install kconfiglib")
    exit()

script_path = os.path.split(os.path.realpath(__file__))[0]
topdir = os.path.join(script_path, "..")
kconfig_path = os.path.join(topdir, "Kconfig")

# Set environment variables

os.environ["BINDIR"] = os.path.join(topdir)
os.environ["APPSDIR"] = os.path.join(topdir, "../apps")
os.environ["APPSBINDIR"] = os.path.join(topdir, "../apps")


def merge_configs(output_file, input_files):
    kconf = Kconfig(kconfig_path, suppress_traceback=True)

    kconf.warn_assign_undef = True
    kconf.warn_assign_override = False
    kconf.warn_assign_redun = False

    for input_file in input_files:
        print(kconf.load_config(input_file, replace=False))

    # Save the merged configuration to the output file

    kconf.write_min_config(output_file)
    print(f"Configuration successfully merged to {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge Kconfig configuration files")
    parser.add_argument(
        "-o", "--output", required=True, help="Output merged configuration file"
    )
    parser.add_argument(
        "input_files", nargs="+", help="List of input configuration files"
    )
    args = parser.parse_args()

    merge_configs(args.output, args.input_files)
