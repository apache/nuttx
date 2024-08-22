############################################################################
# tools/gdb/utils.py
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

import re
from typing import List

import gdb

g_symbol_cache = {}
g_type_cache = {}


def backtrace(addresses: List[gdb.Value]) -> List[str]:
    """Convert addresses to backtrace"""
    backtrace = []

    for addr in addresses:
        if not addr:
            break

        func = addr.format_string(symbols=True, address=False)
        sym = gdb.find_pc_line(int(addr))
        source = str(sym.symtab) + ":" + str(sym.line)
        backtrace.append((int(addr), func, source))

    return backtrace


def lookup_type(name, block=None) -> gdb.Type:
    """Return the type object of a type name"""
    global g_type_cache

    key = (name, block)
    if key not in g_type_cache:
        try:
            g_type_cache[key] = (
                gdb.lookup_type(name, block=block) if block else gdb.lookup_type(name)
            )
        except gdb.error:
            g_type_cache[key] = None

    return g_type_cache[key]


long_type = lookup_type("long")

# Common Helper Functions


def get_long_type():
    """Return the cached long type object"""
    global long_type
    return long_type


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


def gdb_eval_or_none(expresssion):
    """Evaluate an expression and return None if it fails"""
    try:
        return gdb.parse_and_eval(expresssion)
    except gdb.error:
        return None


def get_symbol_value(name, locspec="nx_start", cacheable=True):
    """Return the value of a symbol value etc: Variable, Marco"""
    global g_symbol_cache

    # If there is a current stack frame, GDB uses the macros in scope at that frame’s source code line.
    # Otherwise, GDB uses the macros in scope at the current listing location.
    # Reference: https://sourceware.org/gdb/current/onlinedocs/gdb.html/Macros.html#Macros
    try:
        if not gdb.selected_frame():
            gdb.execute(f"list {locspec}", to_string=True)
            return gdb_eval_or_none(name)
    except gdb.error:
        pass

    # Try current frame
    value = gdb_eval_or_none(name)
    if value:
        return value

    # Check if the symbol is already cached
    if cacheable and (name, locspec) in g_symbol_cache:
        return g_symbol_cache[(name, locspec)]

    # There's current frame and no definition found. We need second inferior without a valid frame
    # in order to use the list command to set the scope.
    if len(gdb.inferiors()) == 1:
        gdb.execute(
            f"add-inferior -exec {gdb.objfiles()[0].filename} -no-connection",
            to_string=True,
        )
        g_symbol_cache = {}

    try:
        suppressed = "is on" in gdb.execute(
            "show suppress-cli-notifications", to_string=True
        )
    except gdb.error:
        # Treat as suppressed if the command is not available
        suppressed = True

    if not suppressed:
        # Disable notifications
        gdb.execute("set suppress-cli-notifications on")

    # Switch to inferior 2 and set the scope firstly
    gdb.execute("inferior 2", to_string=True)
    gdb.execute(f"list {locspec}", to_string=True)
    value = gdb_eval_or_none(name)
    if cacheable:
        g_symbol_cache[(name, locspec)] = value

    # Switch back to inferior 1
    gdb.execute("inferior 1", to_string=True)

    if not suppressed:
        gdb.execute("set suppress-cli-notifications off")
    return value


def get_field(val, key, default=None):
    """Get a field from a gdb.Value, return default if key not found"""
    try:
        return val[key] if val else default
    except gdb.error:
        return default


def get_bytes(val, size):
    """Convert a gdb value to a bytes object"""
    try:
        return val.bytes[:size]
    except AttributeError:  # Sometimes we don't have gdb.Value.bytes
        inf = gdb.inferiors()[0]
        mem = inf.read_memory(val.address, size)
        return mem.tobytes()


def import_check(module, name="", errmsg=""):
    try:
        module = __import__(module, fromlist=[name])
    except ImportError:
        gdb.write(errmsg if errmsg else f"Error to import {module}\n")
        return None

    return getattr(module, name) if name else module


def hexdump(address, size):
    inf = gdb.inferiors()[0]
    mem = inf.read_memory(address, size)
    bytes = mem.tobytes()
    for i in range(0, len(bytes), 16):
        chunk = bytes[i : i + 16]
        gdb.write(f"{i + address:08x}  ")
        hex_values = " ".join(f"{byte:02x}" for byte in chunk)
        hex_display = f"{hex_values:<47}"
        gdb.write(hex_display)
        ascii_values = "".join(
            chr(byte) if 32 <= byte <= 126 else "." for byte in chunk
        )
        gdb.write(f"  {ascii_values} \n")


def is_decimal(s):
    return re.fullmatch(r"\d+", s) is not None


def is_hexadecimal(s):
    return re.fullmatch(r"0[xX][0-9a-fA-F]+|[0-9a-fA-F]+", s) is not None


class Hexdump(gdb.Command):
    """hexdump address/symbol <size>"""

    def __init__(self):
        super(Hexdump, self).__init__("hexdump", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        argv = args.split(" ")
        address = 0
        size = 0
        if argv[0] == "":
            gdb.write("Usage: hexdump address/symbol <size>\n")
            return

        if is_decimal(argv[0]) or is_hexadecimal(argv[0]):
            address = int(argv[0], 0)
            size = int(argv[1], 0)
        else:
            var = gdb.parse_and_eval(f"{argv[0]}")
            address = int(var.address)
            size = int(var.type.sizeof)
            gdb.write(f"{argv[0]} {hex(address)} {int(size)}\n")

        hexdump(address, size)


Hexdump()


def nitems(array):
    array_type = array.type
    element_type = array_type.target()
    element_size = element_type.sizeof
    array_size = array_type.sizeof // element_size
    return array_size


# Machine Specific Helper Functions


BIG_ENDIAN = 0
LITTLE_ENDIAN = 1
target_endianness = None


def get_target_endianness():
    """Return the endianness of the target"""
    global target_endianness
    if not target_endianness:
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


try:
    # For some prebuilt GDB, the python builtin module `struct` is not available
    import struct

    def read_u16(buffer, offset):
        """Read a 16-bit unsigned integer from a buffer"""
        if get_target_endianness() == LITTLE_ENDIAN:
            return struct.unpack_from("<H", buffer, offset)[0]
        else:
            return struct.unpack_from(">H", buffer, offset)[0]

    def read_u32(buffer, offset):
        """Read a 32-bit unsigned integer from a buffer"""
        if get_target_endianness() == LITTLE_ENDIAN:
            return struct.unpack_from("<I", buffer, offset)[0]
        else:
            return struct.unpack_from(">I", buffer, offset)[0]

    def read_u64(buffer, offset):
        """Read a 64-bit unsigned integer from a buffer"""
        if get_target_endianness() == LITTLE_ENDIAN:
            return struct.unpack_from("<Q", buffer, offset)[0]
        else:
            return struct.unpack_from(">Q", buffer, offset)[0]

except ModuleNotFoundError:

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


def bswap(val, size):
    """Reverses the byte order in a gdb.Value or int value of size bytes"""
    return int.from_bytes(int(val).to_bytes(size, byteorder="little"), byteorder="big")


def swap16(val):
    return bswap(val, 2)


def swap32(val):
    return bswap(val, 4)


def swap64(val):
    return bswap(val, 8)


target_arch = None


def is_target_arch(arch, exact=False):
    """
    For non extact match, this function will
    return True if the target architecture contains
    keywords of an ARCH family. For example, x86 is
    contained in i386:x86_64.
    For exact match, this function will return True if
    the target architecture is exactly the same as ARCH.
    """
    if hasattr(gdb.Frame, "architecture"):
        archname = gdb.newest_frame().architecture().name()

        return arch in archname if not exact else arch == archname
    else:
        global target_arch
        if target_arch is None:
            target_arch = gdb.execute("show architecture", to_string=True)
            pattern = r'set to "(.*?)"\s*(\(currently (".*")\))?'
            match = re.search(pattern, target_arch)

            candidate = match.group(1)

            if candidate == "auto":
                target_arch = match.group(3)
            else:
                target_arch = candidate

        return arch in target_arch if not exact else arch == target_arch


# Kernel Specific Helper Functions


def is_target_smp():
    """Return Ture if the target use smp"""

    if gdb.lookup_global_symbol("g_assignedtasks"):
        return True
    else:
        return False


# FIXME: support RISC-V/X86/ARM64 etc.
def in_interrupt_context(cpuid=0):
    frame = gdb.selected_frame()

    if is_target_arch("arm"):
        xpsr = int(frame.read_register("xpsr"))
        return xpsr & 0xF
    else:
        # TODO: figure out a more proper way to detect if
        # we are in an interrupt context
        g_current_regs = gdb_eval_or_none("g_current_regs")
        return not g_current_regs or not g_current_regs[cpuid]


def get_arch_sp_name():
    if is_target_arch("arm"):
        # arm and arm variants
        return "sp"
    if is_target_arch("aarch64"):
        return "sp"
    elif is_target_arch("i386", exact=True):
        return "esp"
    elif is_target_arch("i386:x86-64", exact=True):
        return "rsp"
    else:
        # Default to use sp, add more archs if needed
        return "sp"


def get_arch_pc_name():
    if is_target_arch("arm"):
        # arm and arm variants
        return "pc"
    if is_target_arch("aarch64"):
        return "pc"
    elif is_target_arch("i386", exact=True):
        return "eip"
    elif is_target_arch("i386:x86-64", exact=True):
        return "rip"
    else:
        # Default to use pc, add more archs if needed
        return "pc"


def get_register_byname(regname, tcb=None):
    frame = gdb.selected_frame()

    # If no tcb is given then we can directly use the register from
    # the cached frame by GDB
    if not tcb:
        return int(frame.read_register(regname))

    # Ok, let's take it from the context in the given tcb
    arch = frame.architecture()
    tcbinfo = gdb.parse_and_eval("g_tcbinfo")

    i = 0
    for reg in arch.registers():
        if reg.name == regname:
            break
        i += 1

    regs = tcb["xcp"]["regs"].cast(gdb.lookup_type("char").pointer())
    value = gdb.Value(regs + tcbinfo["reg_off"]["p"][i]).cast(
        gdb.lookup_type("uintptr_t").pointer()
    )[0]

    return int(value)


def get_sp(tcb=None):
    return get_register_byname(get_arch_sp_name(), tcb)


def get_pc(tcb=None):
    return get_register_byname(get_arch_pc_name(), tcb)


def get_tcbs():
    # In case we have created/deleted tasks at runtime, the tcbs will change
    # so keep it as fresh as possible
    pidhash = gdb.parse_and_eval("g_pidhash")
    npidhash = gdb.parse_and_eval("g_npidhash")

    return [pidhash[i] for i in range(0, npidhash) if pidhash[i]]
