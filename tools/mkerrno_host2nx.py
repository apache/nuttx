#!/usr/bin/env python3
############################################################################
# tools/mkerrno_host2nx.py
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
 * arch/sim/src/sim/posix/sim_hosterrno.c
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
 * % python3 tools/mkerrno_host2nx.py \\
 * > arch/sim/src/sim/posix/sim_hosterrno.c
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_errno_to_nx
 *
 * Description:
 *   Covert the host errno to the corresponding NuttX errno.
 *
 * Returned Value:
 *   The corresponding NuttX errno.
 *   Returns default_result for unknown host errno values.
 *
 ****************************************************************************/

int host_errno_to_nx(int host_errno, int default_result)
{
    switch (host_errno)
      {"""[
        1:
    ]
)

with open("include/errno.csv") as f:
    for row in csv.reader(f):
        if row[0] == "EDEADLOCK":
            # Just an alias of EDEADLK.
            continue
        if row[0] == "EOPNOTSUPP":
            # EOPNOTSUPP and ENOTSUP are allowed to have the same value.
            # (Eg. on Linux, they do.)
            # On NuttX, they are different.
            print("#if defined(EOPNOTSUPP) && EOPNOTSUPP != ENOTSUP")
            print(f"        case {row[0]}:")
            print(f"            return {row[1].strip()};")
            print("#endif")
            continue
        if row[0] == "EAGAIN":
            # EWOULDBLOCK is an alias of EAGAIN on NuttX.
            # We don't assume it for the host.
            print("#if defined(EWOULDBLOCK) && EWOULDBLOCK != EAGAIN")
            print("        case EWOULDBLOCK:")
            print(f"            return {row[1].strip()};")
            print("#endif")
        print(f"#if defined({row[0]})")
        print(f"        case {row[0]}:")
        print(f"            return {row[1].strip()};")
        print("#endif")

print(
    """
      }

    /* Convert unknown values to default_result. */

    return default_result;
}
"""[
        1:
    ]
)
