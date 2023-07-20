############################################################################
# tools/gdb/utils.py
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

import gdb


class CachedType:
    """Cache a type object, so that we can reconnect to the new_objfile event"""

    def __init__(self, name):
        self._type = None
        self._name = name

    def _new_objfile_handler(self, event):
        self._type = None
        gdb.events.new_objfile.disconnect(self._new_objfile_handler)

    def get_type(self):
        if self._type is None:
            self._type = gdb.lookup_type(self._name)
            if self._type is None:
                raise gdb.GdbError("cannot resolve type '{0}'".format(self._name))
            if hasattr(gdb, "events") and hasattr(gdb.events, "new_objfile"):
                gdb.events.new_objfile.connect(self._new_objfile_handler)
        return self._type


long_type = CachedType("long")


def get_long_type():
    """Return the cached long type object"""
    global long_type
    return long_type.get_type()


def offset_of(typeobj, field):
    """Return the offset of a field in a structure"""
    element = gdb.Value(0).cast(typeobj)
    return int(str(element[field].address).split()[0], 16)


def container_of(ptr, typeobj, member):
    """Return pointer to containing data structure"""
    return (ptr.cast(get_long_type()) - offset_of(typeobj, member)).cast(typeobj)


class ContainerOf(gdb.Function):
    """Return pointer to containing data structure.

    $container_of(PTR, "TYPE", "ELEMENT"): Given PTR, return a pointer to the
    data structure of the type TYPE in which PTR is the address of ELEMENT.
    Note that TYPE and ELEMENT have to be quoted as strings."""

    def __init__(self):
        super(ContainerOf, self).__init__("container_of")

    def invoke(self, ptr, typename, elementname):
        return container_of(
            ptr, gdb.lookup_type(typename.string()).pointer(), elementname.string()
        )


ContainerOf()


BIG_ENDIAN = 0
LITTLE_ENDIAN = 1
target_endianness = None


def get_target_endianness():
    """Return the endianness of the target"""
    global target_endianness
    if target_endianness is None:
        endian = gdb.execute("show endian", to_string=True)
        if "little endian" in endian:
            target_endianness = LITTLE_ENDIAN
        elif "big endian" in endian:
            target_endianness = BIG_ENDIAN
        else:
            raise gdb.GdbError("unknown endianness '{0}'".format(str(endian)))
    return target_endianness


def read_memoryview(inf, start, length):
    """Read memory from the target and return a memoryview object"""
    m = inf.read_memory(start, length)
    if type(m) is memoryview:
        return m
    return memoryview(m)


def read_u16(buffer, offset):
    """Read a 16-bit unsigned integer from a buffer"""
    buffer_val = buffer[offset : offset + 2]
    value = [0, 0]

    if type(buffer_val[0]) is str:
        value[0] = ord(buffer_val[0])
        value[1] = ord(buffer_val[1])
    else:
        value[0] = buffer_val[0]
        value[1] = buffer_val[1]

    if get_target_endianness() == LITTLE_ENDIAN:
        return value[0] + (value[1] << 8)
    else:
        return value[1] + (value[0] << 8)


def read_u32(buffer, offset):
    """Read a 32-bit unsigned integer from a buffer"""
    if get_target_endianness() == LITTLE_ENDIAN:
        return read_u16(buffer, offset) + (read_u16(buffer, offset + 2) << 16)
    else:
        return read_u16(buffer, offset + 2) + (read_u16(buffer, offset) << 16)


def read_u64(buffer, offset):
    """Read a 64-bit unsigned integer from a buffer"""
    if get_target_endianness() == LITTLE_ENDIAN:
        return read_u32(buffer, offset) + (read_u32(buffer, offset + 4) << 32)
    else:
        return read_u32(buffer, offset + 4) + (read_u32(buffer, offset) << 32)


def read_ulong(buffer, offset):
    """Read a long from a buffer"""
    if get_long_type().sizeof == 8:
        return read_u64(buffer, offset)
    else:
        return read_u32(buffer, offset)


target_arch = None


def is_target_arch(arch):
    """Return True if the target architecture is ARCH"""
    if hasattr(gdb.Frame, "architecture"):
        return arch in gdb.newest_frame().architecture().name()
    else:
        global target_arch
        if target_arch is None:
            target_arch = gdb.execute("show architecture", to_string=True)
        return arch in target_arch


def gdb_eval_or_none(expresssion):
    """Evaluate an expression and return None if it fails"""
    try:
        return gdb.parse_and_eval(expresssion)
    except gdb.error:
        return None


def is_target_smp():
    """Return Ture if the target use smp"""

    if gdb.lookup_global_symbol("g_assignedtasks"):
        return True
    else:
        return False
