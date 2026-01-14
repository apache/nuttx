############################################################################
# tools/pynuttx/nxgdb/protocols/uorb.py
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

from .value import Value


class OrbMetadata(Value):
    """struct orb_metadata_s"""

    o_name: Value
    o_size: Value
    o_format: Value


class SensorMeta(Value):
    """struct sensor_meta_s"""

    esize: Value
    name: Value


class SensorState(Value):
    """struct sensor_state_s"""

    esize: Value
    nbuffer: Value
    min_latency: Value
    min_interval: Value
    nsubscribers: Value
    nadvertisers: Value
    generation: Value
    priv: Value


class SensorUState(Value):
    """struct sensor_ustate_s"""

    esize: Value
    latency: Value
    interval: Value
    generation: Value


class SensorUpper(Value):
    """struct sensor_upperhalf_s"""

    lower: Value
    state: SensorState
    timing: Value
    buffer: Value
    lock: Value
    userlist: Value


class SensorLower(Value):
    """struct sensor_lowerhalf_s"""

    type: Value
    nbuffer: Value
    uncalibrated: Value
    ops: Value
    push_event: Value
    notify_event: Value

    sensor_lock: Value
    sensor_unlock: Value
    priv: Value
    persist: Value


class SensorUser(Value):
    """struct sensor_user_s"""

    node: Value
    fds: Value
    role: Value
    changed: Value
    event: Value
    flushing: Value
    buffersem: Value
    bufferpos: Value
    state: SensorUState
