############################################################################
# tools/gdb/nuttxgdb/debug.py
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

import argparse

import gdb

from . import utils


class DebugPy(gdb.Command):
    """Start debugpy server, so we can debug python code from IDE like VSCode"""

    def __init__(self):
        debugpy = utils.import_check("debugpy", errmsg="Please pip install debugpy")
        if not debugpy:
            return

        self.debugpy = debugpy
        super().__init__("debugpy", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        debugpy = self.debugpy
        if debugpy.is_client_connected():
            gdb.write("debugpy is already running.\n")
            return

        parser = argparse.ArgumentParser(description=DebugPy.__doc__)
        parser.add_argument(
            "-p",
            "--port",
            default=5678,
            type=int,
            help="Server listening port",
        )

        try:
            args = parser.parse_args(gdb.string_to_argv(args))
        except SystemExit:
            return

        debugpy.listen(args.port)
        gdb.write(f"Waiting for connection at localhost:{args.port}\n")
        debugpy.wait_for_client()
        gdb.write("Debugger connected.\n")
