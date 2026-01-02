#!/usr/bin/env bash
############################################################################
# boards/risc-v/qemu-rv/rv-virt/configs/citest64/run.sh
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

set -o xtrace

# start from nuttx dir
olddir=$(pwd)
nuttdir=${CURRENTCONFDIR}/../../../../../../
cd ${nuttdir}

# prepare env
dd if=/dev/zero of=fatfs.img bs=512 count=128K
mkfs.fat fatfs.img
chmod 777 ./fatfs.img

# enable venv
source ${NTFCDIR}/venv/bin/activate

# run NTFC
confpath=${CURRENTCONFDIR}/config.yaml
jsonconf=${CURRENTCONFDIR}/session.json
testpath=${NTFCDIR}/external/nuttx-testing
python3 -m ntfc test --testpath=${testpath} --confpath=${confpath} --jsonconf=${jsonconf}

ret="$?"
echo $ret

# disable venv
deactivate

# export test results
artifacts=${ARTIFACTCONFDIR}/ntfc
mkdir -p ${artifacts}
mv pytest.debug.log ${artifacts}
mv result ${artifacts}

rm -f fatfs.img

# restore old dir
cd ${olddir}

exit $ret
