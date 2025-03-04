#!/usr/bin/env bash
############################################################################
# tools/ci/cirun.sh
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

set -e
set -o xtrace

WD=$(cd $(dirname $0) && pwd)
WORKSPACE=$(cd $WD/../../../../../../../ && pwd -P)
nuttx=$WORKSPACE/nuttx
logs=${WD}/logs
BOARD=`echo $WD |awk -F '/' '{print $(NF-2)}'`

echo $WD
echo $WORKSPACE

config=$(basename $WD)
if [ "$BOARD" == "sim" ]; then
  target="sim"
  mark="common or ${BOARD}"
else
  if [ "${config:$((-2))}" == "64" ]; then
    BOARD="${BOARD}64"
  fi
  if [ "$BOARD" == "rv-virt" ]; then
    target="qemu"
    mark="qemu or rv_virt"
  else
    target="qemu"
    mark=$target
  fi
fi

core=$target
image=`find ${nuttx} -type f -name 'nuttx'`
path=${image%/*}
cd ${nuttx}/tools/ci/testrun/script
python3 -m pytest -m "${mark}" ./ -B ${BOARD} -P ${path} -L ${logs}/${BOARD}/${core} -R ${target} -C --json=${logs}/${BOARD}/${core}/pytest.json
ret="$?"

#clean
find ${nuttx}/tools/ci/testrun -name '__pycache__' |xargs rm -rf
find ${nuttx}/tools/ci/testrun -name '.pytest_cache' |xargs rm -rf
rm -rf ${logs}
rm -f ${nuttx}/fatfs.img


echo $ret
exit $ret
