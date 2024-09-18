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

import importlib
import json
import os
import re
import shlex
from typing import List, Tuple, Union

import gdb

from .macros import fetch_macro_info, try_expand

g_symbol_cache = {}
g_type_cache = {}
g_macro_ctx = None


def backtrace(addresses: List[Union[gdb.Value, int]]) -> List[Tuple[int, str, str]]:
    """Convert addresses to backtrace"""
    backtrace = []

    for addr in addresses:
        if not addr:
            break

        if type(addr) is int:
            addr = gdb.Value(addr)

        if addr.type.code is not gdb.TYPE_CODE_PTR:
            addr = addr.cast(gdb.lookup_type("void").pointer())

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


def offset_of(typeobj: gdb.Type, field: str) -> Union[int, None]:
    """Return the offset of a field in a structure"""
    for f in typeobj.fields():
        if f.name == field:
            return f.bitpos // 8 if f.bitpos is not None else None

    return None


def container_of(ptr: gdb.Value, typeobj: gdb.Type, member: str) -> gdb.Value:
    """Return pointer to containing data structure"""
    return gdb.Value(ptr.address - offset_of(typeobj, member)).cast(typeobj.pointer())


class ContainerOf(gdb.Function):
    """Return pointer to containing data structure.

    $container_of(PTR, "TYPE", "ELEMENT"): Given PTR, return a pointer to the
    data structure of the type TYPE in which PTR is the address of ELEMENT.
    Note that TYPE and ELEMENT have to be quoted as strings."""

    def __init__(self):
        super().__init__("container_of")

    def invoke(self, ptr, typename, elementname):
        return container_of(
            ptr, gdb.lookup_type(typename.string()).pointer(), elementname.string()
        )


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


def gdb_eval_or_none(expresssion):
    """Evaluate an expression and return None if it fails"""
    try:
        return gdb.parse_and_eval(expresssion)
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
            f"add-inferior -exec {gdb.objfiles()[0].filename} -no-connection",
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
            gdb.write("No macro context found, trying to load from ELF\n")
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
        return gdb.parse_and_eval(f"{arg}")
    except gdb.error:
        return None


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
    if is_target_arch("arm") or is_target_arch("aarch64"):
        # arm and arm variants
        return "sp"
    elif is_target_arch("i386", exact=True):
        return "esp"
    elif is_target_arch("i386:x86-64", exact=True):
        return "rsp"
    else:
        # Default to use sp, add more archs if needed
        return "sp"


def get_arch_pc_name():
    if is_target_arch("arm") or is_target_arch("aarch64"):
        # arm and arm variants
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


def get_tcb(pid):
    """get tcb from pid"""
    g_pidhash = gdb.parse_and_eval("g_pidhash")
    g_npidhash = gdb.parse_and_eval("g_npidhash")
    tcb = g_pidhash[pid & (g_npidhash - 1)]
    if not tcb or pid != tcb["pid"]:
        return None

    return tcb


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


def jsonify(obj, indent=None):
    if not obj:
        return "{}"

    def dumper(obj):
        try:
            return str(obj) if isinstance(obj, gdb.Value) else obj.toJSON()
        except Exception:
            return obj.__dict__

    return json.dumps(obj, default=dumper, indent=indent)


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
            var = gdb.parse_and_eval(f"{argv[0]}")
            address = int(var.address)
            size = int(var.type.sizeof)
            gdb.write(f"{argv[0]} {hex(address)} {int(size)}\n")

        hexdump(address, size)


class Addr2Line(gdb.Command):
    """Convert addresses or expressions

    Usage: addr2line address1 address2 expression1
    Example: addr2line 0x1234 0x5678
             addr2line "0x1234 + pointer->abc" &var var->field function_name var
             addr2line $pc $r1 "$r2 + var"
             addr2line [24/08/29 20:51:02] [CPU1] [209] [ap] sched_dumpstack: backtrace| 0: 0x402cd484 0x4028357e
    """

    def __init__(self):
        super().__init__("addr2line", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        if not args:
            gdb.write(Addr2Line.__doc__ + "\n")
            return

        addresses = []
        for arg in shlex.split(args):
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

        backtraces = backtrace(addresses)
        formatter = "{:<20} {:<32} {}\n"
        gdb.write(formatter.format("Address", "Symbol", "Source"))
        for addr, func, source in backtraces:
            gdb.write(formatter.format(hex(addr), func, source))


class Profile(gdb.Command):
    """Profile a gdb command

    Usage: profile <gdb command>
    """

    def __init__(self):
        self.cProfile = import_check(
            "cProfile", errmsg="cProfile module not found, try gdb-multiarch.\n"
        )
        if not self.cProfile:
            return

        super().__init__("profile", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        self.cProfile.run(f"gdb.execute('{args}')", sort="cumulative")
