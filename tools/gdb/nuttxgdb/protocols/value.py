############################################################################
# tools/gdb/nuttxgdb/protocols/value.py
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

from typing import Protocol

import gdb


class Value(Protocol):
    address: Value
    is_optimized_out: bool
    type: gdb.Type
    dynamic_type: gdb.Type
    is_lazy: bool
    bytes: bytes

    def cast(self, type: gdb.Type) -> Value: ...
    def dereference(self) -> Value: ...
    def referenced_value(self) -> Value: ...
    def reference_value(self) -> Value: ...
    def rvalue_reference_value(self) -> Value: ...
    def const_value(self) -> Value: ...
    def dynamic_cast(self, type: gdb.Type) -> Value: ...
    def reinterpret_cast(self, type: gdb.Type) -> Value: ...

    def format_string(
        self,
        raw: bool = ...,
        pretty_arrays: bool = ...,
        pretty_structs: bool = ...,
        array_indexes: bool = ...,
        symbols: bool = ...,
        unions: bool = ...,
        address: bool = ...,
        deref_refs: bool = ...,
        actual_objects: bool = ...,
        static_members: bool = ...,
        max_elements: int = ...,
        max_depth: int = ...,
        repeat_threshold: int = ...,
        format: str = ...,
    ) -> str: ...

    def string(
        self, encoding: str = ..., errors: str = ..., length: int = ...
    ) -> str: ...
