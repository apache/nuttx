#!/usr/bin/env bash
############################################################################
# boards/rh850/u2c/qemu-rh850-u2c/configs/citest/run.sh
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

# run NTFC
confpath=${CURRENTCONFDIR}/config.yaml
jsonconf=${CURRENTCONFDIR}/session.json
testpath=${NTFCDIR}/external/nuttx-testing
ntfc test --testpath=${testpath} --confpath=${confpath} --jsonconf=${jsonconf}

ret="$?"
echo $ret

# a run where every test is skipped also exits 0, so require test_os in
# the report of this session, the newest one under result/

if [ "$ret" -eq 0 ] && ! python3 - <<'EOF'
import glob
import os
import sys
import xml.etree.ElementTree as ET

sessions = sorted(glob.glob("result/*/"), key=os.path.getmtime)
if not sessions:
    sys.exit(1)

last = sessions[-1]
for f in glob.glob(last + "report.xml") + glob.glob(last + "report/*.xml"):
    for case in ET.parse(f).getroot().iter("testcase"):
        bad = [c for c in case if c.tag in ("skipped", "failure", "error")]
        if case.get("name", "").startswith("test_os") and not bad:
            sys.exit(0)
sys.exit(1)
EOF
then
  echo "test_os did not pass"
  ret=1
fi

# export test results
artifacts=${ARTIFACTCONFDIR}/ntfc
mkdir -p ${artifacts}
mv result ${artifacts}

# restore old dir
cd ${olddir}

exit $ret
