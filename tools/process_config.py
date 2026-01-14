#!/usr/bin/env python3
# tools/process_config.py
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

import re
import sys
from pathlib import Path


def expand_file(input_path, include_paths, processed=None):
    """
    Recursively expand the file, returning its contents in order as a list of lines.
    """
    if processed is None:
        processed = set()

    input_path = Path(input_path).resolve()
    if input_path in processed:
        return []  # Already processed, avoid duplicate includes
    processed.add(input_path)

    expanded_lines = []

    with input_path.open("r", encoding="utf-8") as f:
        lines = f.readlines()

    for line in lines:
        line_strip = line.strip()
        match = re.match(r'#include\s*[<"]([^">]+)[">]', line_strip)
        if match:
            include_file = match.group(1)
            found = False

            # Check the current directory first

            direct_path = input_path.parent / include_file
            if direct_path.exists():
                expanded_lines.extend(
                    expand_file(direct_path, include_paths, processed)
                )
                found = True
            else:
                # Then check in the include paths

                for path in include_paths:
                    candidate = Path(path) / include_file
                    if candidate.exists():
                        expanded_lines.extend(
                            expand_file(candidate, include_paths, processed)
                        )
                        found = True
                        break

            if not found:
                print(
                    f'ERROR: Cannot find "{include_file}" from {input_path}',
                    file=sys.stderr,
                )
                sys.exit(1)
        else:
            expanded_lines.append(line)

    expanded_lines.append("\n")  # Keep separation between files
    return expanded_lines


def process_file(output_path, input_path, include_paths):
    lines = expand_file(input_path, include_paths)
    with open(output_path, "w", encoding="utf-8") as out:
        out.writelines(lines)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(
            "Usage: process_includes.py <output_file> <input_file> [include_paths...]",
            file=sys.stderr,
        )
        sys.exit(1)

    output_file = Path(sys.argv[1])
    input_file = sys.argv[2]
    include_dirs = sys.argv[3:]

    if output_file.exists():
        output_file.unlink()

    process_file(output_file, input_file, include_dirs)
