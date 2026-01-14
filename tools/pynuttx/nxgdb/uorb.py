############################################################################
# tools/pynuttx/nxgdb/uorb.py
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

from __future__ import annotations

import argparse
import re
from typing import List

import gdb

from . import fs, utils
from .circbuf import CircBuf
from .protocols import uorb as p


class Sensor(utils.Value, p.SensorUpper):
    """struct sensor_upperhalf_s and enhancement"""

    inode_s = utils.lookup_type("struct inode")
    sensor_upperhalf_s = utils.lookup_type("struct sensor_upperhalf_s")

    def __init__(self, inode: gdb.Value | utils.Value, path=None):
        # inode must be type of struct inode *
        if inode.type.code != gdb.TYPE_CODE_PTR:
            raise ValueError(f"Expect pointer type, got {inode.type}")

        super().__init__(inode["i_private"].cast(self.sensor_upperhalf_s.pointer()))
        self.inode = inode
        self._path = path

    def __repr__(self) -> str:
        state = self.state
        return f"{hex(self)} {self.topicname} {state.nsubscribers} subscribers, {state.nadvertisers} advertisers"

    def details(self) -> str:
        state = self.state
        return f"nbuffer: {state.nbuffer}, latency: {state.min_latency}, interval: {state.min_interval}"

    def __str__(self) -> str:
        return self.__repr__()

    @property
    def path(self):
        return self._path or fs.inode_getpath(self.inode)

    @property
    def nsubscribers(self) -> int:
        return int(self.state["nsubscribers"])

    @property
    def nadvertisers(self) -> int:
        return int(self.state["nadvertisers"])

    @property
    def topicname(self):
        name = self.path.split("/")[-1]
        name = re.sub(r"(\d$)", "", name)
        name = re.sub(r"(_uncal$)", "", name)
        return name

    @property
    def metadata(self) -> p.OrbMetadata:
        return utils.gdb_eval_or_none(f"g_orb_{self.topicname}")

    @property
    def datatype(self) -> gdb.Type:
        """Return the datatype of the topic like struct sensor_accel"""
        return utils.lookup_type(f"struct {self.topicname}")

    @property
    def circbuf(self) -> CircBuf:
        if not self.datatype:
            return None
        return CircBuf(self.buffer, datatype=self.datatype.pointer())


def get_topic_inodes(topic: str = None) -> List[fs.Inode]:
    nodes = (
        (node, path)
        for node, path in fs.foreach_inode()
        if path.startswith("/dev/uorb/") and (not topic or topic in path)
    )
    return nodes


def get_topics(topic: str = None) -> List[Sensor]:
    nodes = get_topic_inodes(topic)
    return (Sensor(node, path=path) for node, path in nodes)


class uORBDump(gdb.Command):
    """Dump uORB topics"""

    formatter = "{:<20} {:<24} {:<6} {:<6} {:<6} {:<6} {:<12} {:<12} {:<20}"
    header = (
        "Address",
        "Topic",
        "Subs",
        "Ads",
        "esize",
        "nbuf",
        "latency",
        "interval",
        "Circbuf",
    )

    def __init__(self):
        super().__init__("uorb", gdb.COMMAND_USER)

    def invoke(self, arg: str, from_tty: bool) -> None:
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument(
            "--topic",
            type=str,
            help="The topic name to dump, e.g. 'sensor_accel'",
            default=None,
        )

        try:
            args = parser.parse_args(gdb.string_to_argv(arg))
        except SystemExit:
            return

        print(self.formatter.format(*self.header))
        for topic in get_topics(topic=args.topic):
            print(
                self.formatter.format(
                    hex(topic),
                    topic.topicname,
                    topic.nsubscribers,
                    topic.nadvertisers,
                    topic.state.esize,
                    topic.state.nbuffer,
                    topic.state.min_latency,
                    topic.state.min_interval,
                    hex(topic.buffer.address),
                )
            )
