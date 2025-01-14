#!/usr/bin/env python3
# tools/gcov.py

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

import argparse
import os
import re
import shutil
import subprocess
import sys


def parse_gcda_data(path):
    with open(path, "r") as file:
        lines = file.read().strip().splitlines()

    started = False
    filename = ""
    output = ""
    size = 0

    for line in lines:
        if line.startswith("gcov start"):
            started = True
            match = re.search(r"filename:(.*?)\s+size:\s*(\d+)Byte", line)
            if match:
                filename = match.group(1)
                size = int(match.group(2))
            continue

        if not started:
            continue

        if line.startswith("gcov end"):
            started = False
            if size != len(output) // 2:
                print(
                    f"Size mismatch for {filename}: expected {size} bytes, got {len(output) // 2} bytes"
                )

            match = re.search(r"checksum:\s*(0x[0-9a-fA-F]+)", line)
            if match:
                checksum = int(match.group(1), 16)
                output = bytearray.fromhex(output)
                expected = sum(output) % 65536
                if checksum != expected:
                    print(
                        f"Checksum mismatch for {filename}: expected {checksum}, got {expected}"
                    )
                    continue

                with open(filename, "wb") as fp:
                    fp.write(output)
                    print(f"write {filename} success")
            output = ""
        else:
            output += line.strip()


def correct_content_path(file, newpath):
    with open(file, "r", encoding="utf-8") as f:
        content = f.read()

    pattern = r"SF:([^\s]*?)/nuttx/include/nuttx"
    matches = re.findall(pattern, content)

    if matches:
        new_content = content.replace(matches[0], newpath)

        with open(file, "w", encoding="utf-8") as f:
            f.write(new_content)


def copy_file_endswith(endswith, source_dir, target_dir, skip_dir):
    print(f"Collect {endswith} files {source_dir} -> {target_dir}")

    if not os.path.exists(target_dir):
        os.makedirs(target_dir)

    for root, _, files in os.walk(source_dir):
        if skip_dir in root:
            continue

        for file in files:
            if file.endswith(endswith):
                source_file = os.path.join(root, file)
                target_file = os.path.join(target_dir, file)
                shutil.copy2(source_file, target_file)


def arg_parser():
    parser = argparse.ArgumentParser(
        description="Code coverage generation tool.", add_help=False
    )
    parser.add_argument("-i", "--input", help="Input dump data")
    parser.add_argument("-t", dest="gcov_tool", help="Path to gcov tool")
    parser.add_argument(
        "-b", dest="base_dir", default=os.getcwd(), help="Compile base directory"
    )
    parser.add_argument(
        "-s",
        dest="gcno_dir",
        default=os.getcwd(),
        help="Directory containing gcno files",
    )
    parser.add_argument(
        "-a",
        dest="gcda_dir",
        default=os.getcwd(),
        nargs="+",
        help="Directory containing gcda files",
    )
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    parser.add_argument(
        "-x",
        dest="only_copy",
        action="store_true",
        help="Only copy *.gcno and *.gcda files",
    )
    parser.add_argument(
        "-o",
        dest="gcov_dir",
        nargs="?",
        default=os.getcwd(),
        help="Directory to store gcov data and report",
    )

    return parser.parse_args()


def main():
    args = arg_parser()

    root_dir = os.getcwd()
    gcov_dir = os.path.abspath(args.gcov_dir)
    gcno_dir = os.path.abspath(args.gcno_dir)

    if os.path.exists(gcov_dir):
        shutil.rmtree(gcov_dir)

    os.makedirs(gcov_dir)

    gcda_dir = []
    for i in args.gcda_dir:
        gcda_dir.append(os.path.abspath(i))

    coverage_file = os.path.join(gcov_dir, "coverage.info")
    result_dir = os.path.join(gcov_dir, "result")

    if args.debug:
        debug_file = os.path.join(gcov_dir, "debug.log")
        sys.stdout = open(debug_file, "w+")

    if args.input:
        parse_gcda_data(os.path.join(root_dir, args.input))

    gcov_data_dir = []

    # Collect gcno, gcda files
    for i in gcda_dir:

        dir = os.path.join(gcov_dir + "/data", os.path.basename(i))
        gcov_data_dir.append(dir)
        os.makedirs(dir)

        copy_file_endswith(".gcno", gcno_dir, dir, gcov_dir)
        copy_file_endswith(".gcda", i, dir, gcov_dir)

    # Only copy files
    if args.only_copy:
        sys.exit(0)

    # lcov tool is required
    if shutil.which("lcov") is None:
        print(
            "Error: Code coverage generation tool is not detected, please install lcov."
        )
        sys.exit(1)

    try:

        # lcov collect coverage data to coverage_file
        command = [
            "lcov",
            "-c",
            "-o",
            coverage_file,
            "--rc",
            "lcov_branch_coverage=1",
            "--gcov-tool",
            args.gcov_tool,
            "--ignore-errors",
            "gcov",
        ]
        for i in gcov_data_dir:
            command.append("-d")
            command.append(i)

        print(command)

        subprocess.run(
            command,
            check=True,
            stdout=sys.stdout,
            stderr=sys.stdout,
        )

        if args.base_dir:
            correct_content_path(coverage_file, args.base_dir)

        # genhtml generate coverage report
        subprocess.run(
            [
                "genhtml",
                "--branch-coverage",
                "-o",
                result_dir,
                coverage_file,
                "--ignore-errors",
                "source",
            ],
            check=True,
            stdout=sys.stdout,
            stderr=sys.stdout,
        )

        print(
            "Copy the following link and open it in the browser to view the coverage report:"
        )
        print(f"file://{os.path.join(result_dir, 'index.html')}")

    except subprocess.CalledProcessError:
        print("Failed to generate coverage file.")
        sys.exit(1)

    for i in gcov_data_dir:
        shutil.rmtree(i)

    os.remove(coverage_file)


if __name__ == "__main__":
    main()
