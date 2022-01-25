#!/usr/bin/env bash
# tools/gcov.sh
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

ROOT_DIR=$(cd $(dirname $0)/../../; pwd)

if [ $# == 1 ]; then
    GCOV_DIR=$1
else
    GCOV_DIR=${ROOT_DIR}/gcov
fi

files=$(find -name "*.gcda" 2> /dev/null | wc -l)
if [ "$files" == "0" ] ;then
    echo "Please run ./nuttx before using gcov.sh to generate the coverage report"
    exit 1
fi

type lcov
if [ $? -ne 0 ]; then
    echo "Code coverage generation tool is not detected, please install lcov"
    exit 1
fi

mkdir -p ${GCOV_DIR}
cd ${GCOV_DIR}

# Generate coverage text report
lcov -c -d ${ROOT_DIR} -o coverage.info --rc lcov_branch_coverage=1
# Generate coverage page report
genhtml --branch-coverage -o result coverage.info

if [ $? -ne 0 ]; then
    echo "Failed to generate coverage file"
    exit 1
fi

echo -e "Copy the following link and open it in the browser to view the coverage report"
echo  "file://${GCOV_DIR}/result/index.html"
