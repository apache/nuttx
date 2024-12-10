#!/usr/bin/env bash
# tools/gcov.sh
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.
# The ASF licenses this file to you under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with
# the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

ROOT_DIR=$(pwd)
GCNO_DIR=$ROOT_DIR
GCDA_DIR=$ROOT_DIR

show_help() {
    echo "Usage: $0 [-t gcov_tool] [-s gcno_dir] [-a gcda_dir] [-x] [gcov_dir]"
    echo "  -t gcov_tool: path to gcov tool, e.g. ./nuttx/tools/gcov.sh -t arm-none-eabi-gcov"
    echo "  -s gcno_dir: directory containing gcno files (relative or absolute path allowed)"
    echo "  -a gcda_dir: directory containing gcda files (relative or absolute path allowed)"
    echo "  -x: only copy *.gcno and *.gcda files"
    echo "  gcov_dir: directory to store gcov data and report (optional)"
    exit 1
}

convert_to_absolute_path() {
    local dir_path=$1
    if [ -z "$dir_path" ]; then
        echo "Error: Directory path is empty."
        exit 1
    fi
    # Convert to absolute path if not already
    if [ ! "${dir_path:0:1}" = "/" ]; then
        dir_path=$(realpath "$dir_path" 2>/dev/null)
    fi
    echo "$dir_path"
}

while getopts "a:s:t:xh" opt

do
    case $opt in
        a)
            GCDA_DIR=$(convert_to_absolute_path "$OPTARG")
            if [ ! -d "$GCDA_DIR" ]; then
                echo "Error: Invalid gcda directory: $OPTARG"
                exit 1
            fi
            ;;
        s)
            GCNO_DIR=$(convert_to_absolute_path "$OPTARG")
            if [ ! -d "$GCNO_DIR" ]; then
                echo "Error: Invalid gcno directory: $OPTARG"
                exit 1
            fi
            ;;
        t)
            GCOV_TOOL="--gcov-tool $OPTARG"
            ;;
        x)
            ONLY_COPY=1
            ;;
        h)
            show_help
            ;;
        ?)
            show_help
            ;;
    esac

done

# Handle gcov_dir as the last positional argument

shift $((OPTIND - 1))
if [ $# -gt 1 ]; then
    echo "Error: Too many arguments."
    show_help
fi

if [ $# -eq 1 ]; then
    GCOV_DIR=$(convert_to_absolute_path "$1")
else
    GCOV_DIR=${ROOT_DIR}/gcov
fi

mkdir -p ${GCOV_DIR} ${GCOV_DIR}/data
cd ${GCOV_DIR}

# Collect gcno files

find ${GCNO_DIR}/ -name "*.gcno" -exec cp {} ${GCOV_DIR}/data > /dev/null 2>&1 \;

# Collect gcda files

find ${GCDA_DIR}/ -name "*.gcda" -exec cp {} ${GCOV_DIR}/data > /dev/null 2>&1 \;

# Ensure ONLY_COPY is initialized as an integer
ONLY_COPY=${ONLY_COPY:-0}

if [ "$ONLY_COPY" -eq 1 ]; then
    exit 0
fi

if [ -z "$GCOV_TOOL" ]; then
    echo "Error: -t is a required option."
    show_help
    exit 1
fi

type lcov > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "Code coverage generation tool is not detected, please install lcov"
    exit 1
fi

files=$(find ${GCOV_DIR}/data -name "*.gcda" 2> /dev/null | wc -l)
if [ "$files" == "0" ] ;then
    echo "gcda file not found in directory ${ROOT_DIR}"
    echo "Please run ./nuttx before using gcov.sh to generate the coverage report"
    echo "Or copy the gcda file in the device to ${ROOT_DIR}"
    exit 1
fi

# Generate coverage text report
lcov -c -d ${GCOV_DIR}/data -o coverage.info --rc lcov_branch_coverage=1 ${GCOV_TOOL} --ignore-errors gcov

# Generate coverage page report
genhtml --branch-coverage -o result coverage.info --ignore-errors source

if [ $? -ne 0 ]; then
    echo "Failed to generate coverage file"
    exit 1
fi

echo -e "Copy the following link and open it in the browser to view the coverage report"
echo  "file://${GCOV_DIR}/result/index.html"
