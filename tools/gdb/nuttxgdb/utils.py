############################################################################
# tools/gdb/nuttxgdb/utils.py
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
import hashlib
import importlib
import json
import os
import re
import shlex
from enum import Enum
from typing import List, Optional, Tuple, Union

import gdb

from .macros import fetch_macro_info, try_expand
from .protocols.thread import Tcb

g_symbol_cache = {}
g_type_cache = {}
g_macro_ctx = None
g_backtrace_cache = {}


class Value(gdb.Value):
    def __init__(self, obj: Union[gdb.Value, Value]):
        super().__init__(obj)

    def __isabstractmethod__(self):
        # Added to avoid getting error using __getattr__
        return False

    def __getattr__(self, key):
        if hasattr(super(), key):
            value = super().__getattribute__(key)
        else:
            value = super().__getitem__(key)

        return Value(value) if not isinstance(value, Value) else value

    def __getitem__(self, key):
        value = super().__getitem__(key)
        return Value(value) if not isinstance(value, Value) else value

    def __format__(self, format_spec: str) -> str:
        try:
            return super().__format__(format_spec)
        except TypeError:
            # Convert GDB value to python value, and then format it
            type_code_map = {
                gdb.TYPE_CODE_INT: int,
                gdb.TYPE_CODE_PTR: int,
                gdb.TYPE_CODE_ENUM: int,
                gdb.TYPE_CODE_FUNC: hex,
                gdb.TYPE_CODE_BOOL: bool,
                gdb.TYPE_CODE_FLT: float,
                gdb.TYPE_CODE_STRING: str,
                gdb.TYPE_CODE_CHAR: lambda x: chr(int(x)),
            }

            t = self.type
            while t.code == gdb.TYPE_CODE_TYPEDEF:
                t = t.target()

            type_code = t.code
            try:
                converter = type_code_map[type_code]
                return f"{converter(self):{format_spec}}"
            except KeyError:
                raise TypeError(
                    f"Unsupported type: {self.type}, {self.type.code} {self}"
                )

    @property
    def address(self) -> Value:
        value = super().address
        return value and Value(value)

    def cast(self, type: str | gdb.Type, ptr: bool = False) -> Optional["Value"]:
        try:
            gdb_type = lookup_type(type) if isinstance(type, str) else type
            if ptr:
                gdb_type = gdb_type.pointer()
            return Value(super().cast(gdb_type))
        except gdb.error:
            return None

    def dereference(self) -> Value:
        return Value(super().dereference())

    def reference_value(self) -> Value:
        return Value(super().reference_value())

    def referenced_value(self) -> Value:
        return Value(super().referenced_value())

    def rvalue_reference_value(self) -> Value:
        return Value(super().rvalue_reference_value())

    def const_value(self) -> Value:
        return Value(super().const_value())

    def dynamic_cast(self, type: gdb.Type) -> Value:
        return Value(super().dynamic_cast(type))


class Backtrace:
    """
    Convert addresses to backtrace
    Usage:
    backtrace = Backtrace(addresses=[0x4001, 0x4002, 0x4003])

    # Access converted backtrace
    addr, func, source = backtrace[0]
    remaining = backtrace[1:]  # Return list of (addr, func, source)

    # Iterate over backtrace
    for addr, func, source in backtrace:
        print(addr, func, source)

    # Append more addresses to convert
    backtrace.append(0x40001234)

    # Print backtrace
    print(str(backtrace))

    # Format backtrace to string
    print("\n".join(backtrace.formatted))

    # Custom formatter
    backtrace = Backtrace(addresses=[0x4001, 0x4002, 0x4003], formatter="{:<6} {:<20} {}")
    """

    def __init__(
        self,
        address: List[Union[gdb.Value, int]] = [],
        formatter="{:<5} {:<36} {}\n",
        break_null=True,
    ):
        self.formatter = formatter  # Address, Function, Source
        self._formatted = None  # Cached formatted backtrace
        self.backtrace = []
        for addr in address:
            if break_null and not addr:
                break
            self.append(addr)

    def __eq__(self, value: Backtrace) -> bool:
        return self.backtrace == value.backtrace

    def __hash__(self) -> int:
        return hash(tuple(self.backtrace))

    def append(self, addr: Union[gdb.Value, int]) -> None:
        """Append an address to the backtrace"""
        if result := self.convert(addr):
            self.backtrace.append(result)
            self._formatted = None  # Clear cached result

    def convert(self, addr: Union[gdb.Value, int]) -> Tuple[int, str, str]:
        """Convert an address to function and source"""
        if not addr:
            return None

        if int(addr) in g_backtrace_cache:
            return g_backtrace_cache[int(addr)]

        if type(addr) is int:
            addr = gdb.Value(addr)

        if addr.type.code is not gdb.TYPE_CODE_PTR:
            addr = addr.cast(gdb.lookup_type("void").pointer())

        func = addr.format_string(symbols=True, address=False)
        sym = gdb.find_pc_line(int(addr))
        source = str(sym.symtab) + ":" + str(sym.line)
        result = (int(addr), func, source)
        g_backtrace_cache[int(addr)] = result
        return result

    @property
    def formatted(self):
        """Return the formatted backtrace string list"""
        if not self._formatted:
            self._formatted = [
                self.formatter.format(hex(addr), func, source)
                for addr, func, source in self.backtrace
            ]

        return self._formatted

    def __repr__(self) -> str:
        return f"Backtrace: {len(self.backtrace)} items"

    def __str__(self) -> str:
        return "".join(self.formatted)

    def __iter__(self):
        for item in self.backtrace:
            yield item

    def __getitem__(self, index):
        return self.backtrace.__getitem__(index)


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


def offset_of(typeobj: Union[gdb.Type, str], field: str) -> Union[int, None]:
    """Return the offset of a field in a structure"""
    if type(typeobj) is str:
        typeobj = gdb.lookup_type(typeobj)

    if typeobj.code is gdb.TYPE_CODE_PTR:
        typeobj = typeobj.target()

    for f in typeobj.fields():
        if f.name == field:
            if f.bitpos is None:
                break
            return f.bitpos // 8

    raise gdb.GdbError(f"Field {field} not found in type {typeobj}")


def container_of(
    ptr: Union[gdb.Value, int], typeobj: Union[gdb.Type, str], member: str
) -> gdb.Value:
    """
    Return a pointer to the containing data structure.

    Args:
        ptr: Pointer to the member.
        t: Type of the container.
        member: Name of the member in the container.

    Returns:
        gdb.Value of the container.

    Example:
        struct foo {
            int a;
            int b;
        };
        struct foo *ptr = container_of(&ptr->b, "struct foo", "b");
    """

    if isinstance(typeobj, str):
        typeobj = gdb.lookup_type(typeobj).pointer()

    if typeobj.code is not gdb.TYPE_CODE_PTR:
        typeobj = typeobj.pointer()

    addr = gdb.Value(ptr).cast(long_type)
    return gdb.Value(addr - offset_of(typeobj, member)).cast(typeobj)


class ContainerOf(gdb.Function):
    """Return pointer to containing data structure.

    $container_of(PTR, "TYPE", "ELEMENT"): Given PTR, return a pointer to the
    data structure of the type TYPE in which PTR is the address of ELEMENT.
    Note that TYPE and ELEMENT have to be quoted as strings."""

    def __init__(self):
        super().__init__("container_of")

    def invoke(self, ptr, typename, elementname):
        return container_of(ptr, typename.string(), elementname.string())


ContainerOf()


class MacroCtx:
    """
    This is a singleton class which only initializes once to
    cache a context of macro definition which can be queried later
    TODO: we only deal with single ELF at the moment for simplicity
    If you load more object files while debugging, only the first one gets loaded
    will be used to retrieve macro information
    """

    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, "instance"):
            cls.instance = super(MacroCtx, cls).__new__(cls)
        return cls.instance

    def __init__(self, filename):
        self._macro_map = {}
        self._file = filename

        self._macro_map = fetch_macro_info(filename)

    @property
    def macro_map(self):
        return self._macro_map

    @property
    def objfile(self):
        return self._file


def parse_and_eval(expression: str, global_context: bool = False):
    """Equivalent to gdb.parse_and_eval, but returns a Value object"""
    gdb_value = gdb.parse_and_eval(expression)
    return Value(gdb_value)


def gdb_eval_or_none(expresssion):
    """Evaluate an expression and return None if it fails"""
    try:
        return parse_and_eval(expresssion)
    except gdb.error:
        return None


def suppress_cli_notifications(suppress=True):
    """Suppress(default behavior) or unsuppress GDB CLI notifications"""
    try:
        suppressed = "is on" in gdb.execute(
            "show suppress-cli-notifications", to_string=True
        )
        if suppress != suppressed:
            gdb.execute(f"set suppress-cli-notifications {'on' if suppress else 'off'}")

        return suppressed
    except gdb.error:
        return True


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
            f'add-inferior -exec "{gdb.objfiles()[0].filename}" -no-connection',
            to_string=True,
        )
        g_symbol_cache = {}

    state = suppress_cli_notifications(True)

    # Switch to inferior 2 and set the scope firstly
    gdb.execute("inferior 2", to_string=True)
    gdb.execute(f"list {locspec}", to_string=True)
    value = gdb_eval_or_none(name)
    if not value:
        # Try to expand macro by reading elf
        global g_macro_ctx
        if not g_macro_ctx:
            if len(gdb.objfiles()) > 0:
                g_macro_ctx = MacroCtx(gdb.objfiles()[0].filename)
            else:
                raise gdb.GdbError("An executable file must be provided")

        expr = try_expand(name, g_macro_ctx.macro_map)
        value = gdb_eval_or_none(expr)

    if cacheable:
        g_symbol_cache[(name, locspec)] = value

    # Switch back to inferior 1
    gdb.execute("inferior 1", to_string=True)
    suppress_cli_notifications(state)
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
    address = int(address)
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


def parse_arg(arg: str) -> Union[gdb.Value, int]:
    """Parse an argument to a gdb.Value or int, return None if failed"""

    if is_decimal(arg):
        return int(arg)

    if is_hexadecimal(arg):
        return int(arg, 16)

    try:
        return parse_and_eval(f"{arg}")
    except gdb.error:
        return None


def alias(name, command):
    try:
        gdb.execute(f"alias {name} = {command}")
    except gdb.error:
        pass


def nitems(array):
    array_type = array.type
    element_type = array_type.target()
    element_size = element_type.sizeof
    array_size = array_type.sizeof // element_size
    return array_size


def sizeof(t: Union[str, gdb.Type]):
    if type(t) is str:
        t = gdb.lookup_type(t)

    return t.sizeof


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
    For non exact match, this function will
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
    return get_register_byname("sp", tcb)


def get_pc(tcb=None):
    return get_register_byname("pc", tcb)


def get_tcbs() -> List[Tcb]:
    # In case we have created/deleted tasks at runtime, the tcbs will change
    # so keep it as fresh as possible
    pidhash = parse_and_eval("g_pidhash")
    npidhash = parse_and_eval("g_npidhash")

    return [pidhash[i] for i in range(0, npidhash) if pidhash[i]]


def get_tcb(pid) -> Tcb:
    """get tcb from pid"""
    g_pidhash = parse_and_eval("g_pidhash")
    g_npidhash = parse_and_eval("g_npidhash")
    tcb = g_pidhash[pid & (g_npidhash - 1)]
    if not tcb or pid != tcb["pid"]:
        return None

    return tcb


def get_tid(tcb):
    """get tid from tcb"""
    if not tcb:
        return None
    try:
        return tcb["group"]["tg_pid"]
    except gdb.error:
        return None


def get_task_name(tcb):
    try:
        name = tcb["name"].cast(gdb.lookup_type("char").pointer())
        return name.string()
    except gdb.error:
        return ""


def switch_inferior(inferior):
    state = suppress_cli_notifications(True)

    if len(gdb.inferiors()) == 1:
        gdb.execute(
            f"add-inferior -exec {gdb.objfiles()[0].filename} -no-connection",
            to_string=True,
        )

    gdb.execute(f"inferior {inferior}", to_string=True)
    return state


def check_version():
    """Check the elf and memory version"""
    state = suppress_cli_notifications()
    switch_inferior(1)
    try:
        mem_version = gdb.execute("p g_version", to_string=True).split("=")[1]
    except gdb.error:
        gdb.write("No symbol g_version found in memory, skipping version check\n")
        suppress_cli_notifications(state)
        return

    switch_inferior(2)
    elf_version = gdb.execute("p g_version", to_string=True).split("=")[1]
    if mem_version != elf_version:
        gdb.write(f"\x1b[31;1mMemory version:{mem_version}")
        gdb.write(f"ELF version:   {elf_version}")
        gdb.write("Warning version not matched, please check!\x1b[m\n")
    else:
        gdb.write(f"Build version: {mem_version}\n")

    switch_inferior(1)  # Switch back
    suppress_cli_notifications(state)


def get_task_tls(tid, key):
    """get task tls from tid and key"""
    tcb = get_tcb(tid)
    if not tcb:
        return None

    try:
        stack_alloc_ptr = tcb["stack_alloc_ptr"].cast(
            lookup_type("struct tls_info_s").pointer()
        )
        tls_value = stack_alloc_ptr["tl_task"]["ta_telem"][int(key)]
        return tls_value.cast(lookup_type("uintptr_t").pointer())
    except gdb.error:
        return None


def get_thread_tls(pid, key):
    """get thread tls from pid and key"""
    tcb = get_tcb(pid)
    if not tcb:
        return None

    try:
        stack_alloc_ptr = tcb["stack_alloc_ptr"].cast(
            lookup_type("struct tls_info_s").pointer()
        )
        tls_value = stack_alloc_ptr["tl_elem"][int(key)]
        return tls_value.cast(lookup_type("uintptr_t").pointer())
    except gdb.error:
        return None


def gather_modules(dir=None) -> List[str]:
    dir = os.path.normpath(dir) if dir else os.path.dirname(__file__)
    return [
        os.path.splitext(os.path.basename(f))[0]
        for f in os.listdir(dir)
        if f.endswith(".py")
    ]


def gather_gdbcommands(modules=None, path=None) -> List[gdb.Command]:
    modules = modules or gather_modules(path)
    commands = []
    for m in modules:
        module = importlib.import_module(f"{__package__}.{m}")
        for c in module.__dict__.values():
            if isinstance(c, type) and issubclass(c, gdb.Command):
                commands.append(c)
    return commands


def get_elf_md5():
    """Return the md5 checksum of the current ELF file"""
    file = gdb.objfiles()[0].filename
    with open(file, "rb") as f:
        hash = hashlib.md5(f.read()).hexdigest()
    return hash


def jsonify(obj, indent=None):
    if not obj:
        return "{}"

    def dumper(obj):
        try:
            return str(obj) if isinstance(obj, gdb.Value) else obj.toJSON()
        except Exception:
            return obj.__dict__

    return json.dumps(obj, default=dumper, indent=indent)


def enum(t: Union[str, gdb.Type], name=None):
    """Create python Enum class from C enum values
    Usage:

    in C:
    enum color_e {
        RED = 1,
        GREEN = 2,
    };

    in python:
    COLOR = utils.enum("enum color_e", "COLOR")
    print(COLOR.GREEN.value) # --> 2
    RED = COLOR(1)
    """
    if type(t) is str:
        t = lookup_type(t) or lookup_type("enum " + t)

    if t and t.code == gdb.TYPE_CODE_TYPEDEF:
        t = t.strip_typedefs()

    if not t or t.code != gdb.TYPE_CODE_ENUM:
        raise gdb.error(f"{t} is not an enum type")

    def commonprefix(m):
        "Given a list of pathnames, returns the longest common leading component"
        if not m:
            return ""
        s1 = min(m)
        s2 = max(m)
        for i, c in enumerate(s1):
            if c != s2[i]:
                return s1[:i]
        return s1

    # Remove the common prefix from names. This is a convention in python.
    # E.g. COLOR.RED, COLOR.GREEN instead of COLOR.COLOR_RED, COLOR.COLOR_GREEN

    prefix = commonprefix([f.name for f in t.fields()])

    names = {f.name[len(prefix) :]: f.enumval for f in t.fields()}

    name = name or prefix[:-1] if prefix[-1] == "_" else prefix
    return Enum(name, names)


class ArrayIterator:
    """An iterator for gdb array or pointer."""

    def __init__(self, array: gdb.Value, maxlen=None, reverse=False):
        type_code = array.type.code
        if type_code not in (gdb.TYPE_CODE_ARRAY, gdb.TYPE_CODE_PTR):
            raise gdb.error(f"Not an array: {array}, type: {array.type}")

        if type_code == gdb.TYPE_CODE_ARRAY:
            if n := nitems(array) > 0:
                maxlen = min(n, maxlen) if maxlen is not None else n

        if maxlen is None:
            raise gdb.error("Need to provide array length.")

        self.array = array
        self.maxlen = maxlen
        self.reverse = reverse
        self.index = maxlen - 1 if reverse else 0

    def __iter__(self):
        return self

    def __next__(self) -> gdb.Value:
        if (not self.reverse and self.index >= self.maxlen) or (
            self.reverse and self.index < 0
        ):
            raise StopIteration

        value = self.array[self.index]
        self.index = self.index - 1 if self.reverse else self.index + 1
        return value


class Hexdump(gdb.Command):
    """hexdump address/symbol <size>"""

    def __init__(self):
        super().__init__("hexdump", gdb.COMMAND_USER)

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
            try:
                var = gdb.parse_and_eval(f"{argv[0]}")
                address = int(var.cast(long_type))
                size = int(argv[1]) if argv[1] else int(var.type.sizeof)
                gdb.write(f"{argv[0]} {hex(address)} {int(size)}\n")
            except Exception as e:
                gdb.write(f"Invalid {argv[0]}: {e}\n")

        hexdump(address, size)


class Addr2Line(gdb.Command):
    """Convert addresses or expressions

    Usage: addr2line address1 address2 expression1
    Example: addr2line 0x1234 0x5678
             addr2line "0x1234 + pointer->abc" &var var->field function_name var
             addr2line $pc $r1 "$r2 + var"
             addr2line [24/08/29 20:51:02] [CPU1] [209] [ap] sched_dumpstack: backtrace| 0: 0x402cd484 0x4028357e
             addr2line -f crash.log
             addr2line -f crash.log -p 123
    """

    formatter = "{:<20} {:<32} {}\n"

    def __init__(self):
        super().__init__("addr2line", gdb.COMMAND_USER)

    def print_backtrace(self, addresses, pid=None):
        if pid:
            gdb.write(f"\nBacktrace of {pid}\n")
        backtraces = Backtrace(addresses, formatter=self.formatter, break_null=False)
        gdb.write(str(backtraces))

    def invoke(self, args, from_tty):
        if not args:
            gdb.write(Addr2Line.__doc__ + "\n")
            return

        parser = argparse.ArgumentParser(
            description="Convert addresses or expressions to source code location"
        )
        parser.add_argument("-f", "--file", type=str, help="Crash log to analyze.")
        parser.add_argument(
            "-p",
            "--pid",
            type=int,
            help="Only dump specified task backtrace from crash file.",
        )

        pargs = None
        try:
            pargs, _ = parser.parse_known_args(gdb.string_to_argv(args))
        except SystemExit:
            pass

        gdb.write(self.formatter.format("Address", "Symbol", "Source"))

        if pargs and pargs.file:
            pattern = re.compile(
                r".*sched_dumpstack: backtrace\|\s*(\d+)\s*:\s*((?:(0x)?[0-9a-fA-F]+\s*)+)"
            )
            addresses = {}
            with open(pargs.file, "r") as f:
                for line in f:
                    match = pattern.match(line)
                    if not match:
                        continue

                    pid = match.group(1)
                    if pargs.pid is not None and pargs.pid != int(pid):
                        continue

                    addresses.setdefault(pid, [])
                    addresses[pid].extend(
                        [int(addr, 16) for addr in match.group(2).split()]
                    )

            for pid, addr in addresses.items():
                self.print_backtrace(addr, pid)
        else:
            addresses = []
            for arg in shlex.split(args.replace(",", " ")):
                if is_decimal(arg):
                    addresses.append(int(arg))
                elif is_hexadecimal(arg):
                    addresses.append(int(arg, 16))
                else:
                    try:
                        var = gdb.parse_and_eval(f"{arg}")
                        addresses.append(var)
                    except gdb.error as e:
                        gdb.write(f"Ignore {arg}: {e}\n")
            self.print_backtrace(addresses)
