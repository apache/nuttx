############################################################################
# tools/gdb/gdbinit.py
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

import sys
from os import path

here = path.dirname(path.abspath(__file__))

if __name__ == "__main__":
    if here not in sys.path:
        sys.path.insert(0, here)

    if "nuttxgdb" in sys.modules:
        for key in list(sys.modules.keys()):
            if key.startswith("nuttxgdb"):
                del sys.modules[key]

    import nuttxgdb  # noqa: F401
