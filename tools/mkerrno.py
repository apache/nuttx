#!/usr/bin/env python3
############################################################################
# tools/mkerrno.py
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

import csv

print(
    """
/****************************************************************************
 * include/errno_defs.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* THIS FILE IS GENERATED. DO NOT EDIT DIRECTLY.
 *
 * To regenerate this file, run:
 *
 * % python3 tools/mkerrno.py > include/errno_defs.h
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
"""[
        1:
    ]
)

with open("include/errno.csv") as f:
    for row in csv.reader(f):
        comment = ""
        t = row[3].strip()
        if t == "linux":
            comment = "Linux errno extension"
        if t == "cygwin":
            comment = "Cygwin"
        if comment:
            print(f"#define {row[0]:19s}{row[1]:28s}/* {comment} */")
        else:
            print(f"#define {row[0]:19s}{row[1]}")
        print(f"#define {row[0] + '_STR':19s}{row[2]}")
