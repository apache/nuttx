#!/usr/bin/env python3
# tools/stackusage.py
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

"""Static stack usage analyzer using DWARF debug info and disassembly.

Extracts per-function stack frame sizes from DWARF .debug_frame CFA offsets
(via readelf -wF) and builds call graphs from objdump -d disassembly.
Reports worst-case total stack depth through the call chain, flagging
uncertain cases such as recursion and indirect calls (function pointers).

Works with both GNU and LLVM toolchains (readelf/objdump/addr2line).
No Python package dependencies required.
"""

import argparse
import os
import re
import subprocess
import sys

# Call mnemonics across all supported architectures.
# Direct vs indirect is determined by whether objdump annotates
# the target with <function_name>.
CALL_MNEMONICS = {
    "bl",
    "blx",
    "blr",  # ARM / AArch64
    "call",
    "callq",  # x86 / x86_64 / SPARC
    "jal",
    "jalr",  # MIPS / RISC-V
    "bctrl",  # PowerPC
    "jmpl",  # SPARC
    "bsr",
    "jsr",  # SuperH
    "call0",
    "call4",
    "call8",
    "call12",  # Xtensa
    "callx0",
    "callx4",
    "callx8",
    "callx12",  # Xtensa indirect
    "calli",  # TriCore
}

# Function names that indicate dynamic stack allocation.
ALLOCA_NAMES = frozenset(
    {
        "alloca",
        "__alloca",
        "__builtin_alloca",
        "__builtin_alloca_with_align",
    }
)


def build_symbol_map(prefix, elf):
    """Build address<->name maps by parsing ``readelf -sW`` output.

    Returns (addr_to_name, name_to_addr, func_sizes, is_thumb, machine).
    """

    out = subprocess.run(
        [prefix + "readelf", "-hsW", elf],
        capture_output=True,
        text=True,
    )
    if out.returncode != 0:
        return {}, {}, {}, False, ""

    addr_to_name = {}
    name_to_addr = {}
    func_sizes = {}
    is_thumb = False
    machine = ""

    for line in out.stdout.splitlines():
        # Detect machine type from readelf header
        if "Machine:" in line:
            machine = line.split(":", 1)[1].strip()
            if "ARM" in machine and "AArch64" not in machine:
                is_thumb = True
            continue

        # Symbol table line format (readelf -sW):
        #   Num:    Value  Size Type    Bind   Vis      Ndx Name
        #     1: 08000100    64 FUNC    GLOBAL DEFAULT    1 main
        parts = line.split()
        if len(parts) < 8:
            continue
        if parts[3] != "FUNC":
            continue
        if parts[6] == "UND":
            continue

        try:
            addr = int(parts[1], 16)
            size = int(parts[2])
        except ValueError:
            continue

        name = parts[7]
        if name.startswith("$"):
            continue

        if is_thumb:
            addr = addr & ~1

        if addr in addr_to_name and len(addr_to_name[addr]) >= len(name):
            continue

        addr_to_name[addr] = name
        name_to_addr[name] = addr
        func_sizes[addr] = size

    return addr_to_name, name_to_addr, func_sizes, is_thumb, machine


def parse_stack_sizes(prefix, elf, addr_to_name, name_to_addr, func_sizes):
    """Parse ``readelf -wF`` output for per-function CFA stack sizes.

    Returns funcs dict: name -> {file, line, self, code_size, qualifier,
    reasons}.
    """

    out = subprocess.run(
        [prefix + "readelf", "-wF", elf],
        capture_output=True,
        text=True,
    )

    funcs = {}
    current_addr = None
    max_offset = 0

    for line in out.stdout.splitlines():
        # FDE header: look for pc=ADDR..ADDR
        m = re.search(r"pc=([0-9a-f]+)\.\.", line, re.IGNORECASE)
        if m:
            # Flush previous FDE
            if current_addr is not None:
                name = addr_to_name.get(current_addr)
                if name and name not in funcs:
                    code_size = func_sizes.get(current_addr, 0)
                    if not code_size:
                        sym_addr = name_to_addr.get(name)
                        if sym_addr is not None:
                            code_size = func_sizes.get(sym_addr, 0)
                    funcs[name] = {
                        "file": "?",
                        "line": "?",
                        "self": max_offset,
                        "code_size": code_size,
                        "qualifier": "static",
                        "reasons": [],
                    }
            current_addr = int(m.group(1), 16)
            # Also try with Thumb bit cleared
            if current_addr not in addr_to_name:
                alt = current_addr & ~1
                if alt in addr_to_name:
                    current_addr = alt
            max_offset = 0
            continue

        # CFA row: extract offset from e.g. "sp+16" or "r13+24"
        m = re.search(r"[a-zA-Z]\w*\+(\d+)", line)
        if m and current_addr is not None:
            val = int(m.group(1))
            if val > max_offset:
                max_offset = val

    # Flush last FDE
    if current_addr is not None:
        name = addr_to_name.get(current_addr)
        if name and name not in funcs:
            code_size = func_sizes.get(current_addr, 0)
            if not code_size:
                sym_addr = name_to_addr.get(name)
                if sym_addr is not None:
                    code_size = func_sizes.get(sym_addr, 0)
            funcs[name] = {
                "file": "?",
                "line": "?",
                "self": max_offset,
                "code_size": code_size,
                "qualifier": "static",
                "reasons": [],
            }

    return funcs


def resolve_sources(prefix, elf, funcs, name_to_addr):
    """Batch-resolve source file:line via ``addr2line -fe``."""

    addrs = []
    names = []
    for name in funcs:
        addr = name_to_addr.get(name)
        if addr is not None:
            addrs.append("%x" % addr)
            names.append(name)

    if not addrs:
        return

    out = subprocess.run(
        [prefix + "addr2line", "-fe", elf],
        capture_output=True,
        text=True,
        input="\n".join(addrs) + "\n",
    )

    lines = out.stdout.splitlines()
    # addr2line outputs two lines per address: function_name, file:line
    for i in range(0, min(len(lines), len(names) * 2), 2):
        idx = i // 2
        if idx >= len(names):
            break
        if i + 1 < len(lines):
            loc = lines[i + 1]
            if loc and loc != "??:0" and loc != "??:?":
                parts = loc.rsplit(":", 1)
                if len(parts) == 2:
                    funcs[names[idx]]["file"] = parts[0]
                    funcs[names[idx]]["line"] = parts[1]


def _sp_pattern(machine):
    """Return a compiled regex matching the stack pointer register.

    Architecture-specific SP register names:
      ARM:      sp, r13
      AArch64:  sp
      x86:      %esp, %rsp
      MIPS:     $sp, $29
      RISC-V:   sp, x2
      Xtensa:   a1
      PowerPC:  r1
      SPARC:    %sp, %o6
      SuperH:   r15
      TriCore:  a10, sp
    """

    m = machine.upper()
    if "ARM" in m and "AARCH64" not in m:
        return re.compile(r"\bsp\b|\br13\b", re.IGNORECASE)
    if "AARCH64" in m:
        return re.compile(r"\bsp\b", re.IGNORECASE)
    if "80386" in m or "X86" in m or "AMD" in m:
        return re.compile(r"%[er]sp\b", re.IGNORECASE)
    if "MIPS" in m:
        return re.compile(r"\$sp\b|\$29\b", re.IGNORECASE)
    if "RISC-V" in m:
        return re.compile(r"\bsp\b|\bx2\b", re.IGNORECASE)
    if "XTENSA" in m or "TENSILICA" in m:
        return re.compile(r"\ba1\b", re.IGNORECASE)
    if "POWER" in m or "PPC" in m:
        return re.compile(r"\br1\b", re.IGNORECASE)
    if "SPARC" in m:
        return re.compile(r"\bsp\b|%sp\b|%o6\b", re.IGNORECASE)
    if "SH" in m or "SUPER" in m or "RENESAS" in m:
        return re.compile(r"\br15\b", re.IGNORECASE)
    if "TRICORE" in m:
        return re.compile(r"\ba10\b|\bsp\b", re.IGNORECASE)
    # Fallback: common SP names
    return re.compile(r"\bsp\b", re.IGNORECASE)


def _is_dynamic_sp_sub(mnemonic, operand_str, sp_re):
    """Return True if instruction dynamically adjusts the stack pointer.

    Detects VLA / alloca patterns where a sub instruction adjusts SP
    by a register operand (not an immediate constant).
    """

    if not mnemonic.startswith("sub"):
        return False
    ops = operand_str.lower()
    if not sp_re.search(ops):
        return False
    # ARM/AArch64: immediates use # prefix
    if "#" in ops:
        return False
    # x86 AT&T syntax: $ means immediate
    if "%" in ops and "$" in ops:
        return False
    return True


def parse_call_graph(prefix, elf, machine):
    """Build call graph by parsing ``objdump -d`` output.

    Returns (graph, dynamic_funcs) where:
      graph: dict mapping caller -> set of (callee_name_or_None, is_indirect).
      dynamic_funcs: set of function names with dynamic stack (alloca/VLA).
    """

    sp_re = _sp_pattern(machine)
    graph = {}
    dynamic_funcs = set()
    current_func = None

    proc = subprocess.Popen(
        [prefix + "objdump", "-d", elf],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )

    for line in proc.stdout:
        # Function boundary: "0000abcd <func_name>:"
        m = re.match(r"^[0-9a-f]+ <(.+)>:\s*$", line)
        if m:
            current_func = m.group(1)
            graph.setdefault(current_func, set())
            continue

        if current_func is None:
            continue

        # Instruction line: "   addr:  bytes  mnemonic  operands"
        parts = line.split("\t")
        if len(parts) < 3:
            continue

        mnemonic = parts[2].strip().split()[0].lower()

        # Extract operand string (used by both call and dynamic-SP checks)
        operands = parts[2].strip().split(None, 1)
        operand_str = operands[1] if len(operands) > 1 else ""
        if len(parts) > 3:
            operand_str += " " + parts[3]

        # Check for call instructions
        if mnemonic in CALL_MNEMONICS:
            target_m = re.search(r"<([^>+]+)>", operand_str)
            if target_m:
                callee = target_m.group(1)
                if callee in ALLOCA_NAMES:
                    dynamic_funcs.add(current_func)
                else:
                    graph[current_func].add((callee, False))
            else:
                graph[current_func].add((None, True))
            continue

        # Check for dynamic stack pointer adjustment (VLA / inlined alloca)
        if _is_dynamic_sp_sub(mnemonic, operand_str, sp_re):
            dynamic_funcs.add(current_func)

    proc.wait()
    return graph, dynamic_funcs


def compute_worst_stack(funcs, graph, recursion_depth):
    """Compute worst-case total stack for every function via memoized DFS.

    Args:
        funcs: dict from parse_stack_sizes
        graph: dict from parse_call_graph
        recursion_depth: how many times a recursive cycle body is counted
                         (0 = back-edges contribute nothing)

    Returns dict mapping function name to result dict with keys:
        self, total, uncertain, reasons, stack, self_reasons
    """

    cache = {}

    def dfs(func, path):
        if func in cache:
            return cache[func]

        info = funcs.get(func)
        self_size = info["self"] if info else 0
        self_reasons = set(info["reasons"]) if info else {"no DWARF data"}
        reasons = set(self_reasons)
        uncertain = bool(reasons)

        callees = graph.get(func, set())
        worst_callee = 0
        best_substack = []

        for callee, is_indirect in callees:
            if is_indirect:
                uncertain = True
                reasons.add("indirect call (function pointer)")
                self_reasons.add("indirect call (function pointer)")
                continue

            cur_path = path + [func]
            if callee in cur_path:
                # Recursion detected
                uncertain = True
                idx = cur_path.index(callee)
                cycle = cur_path[idx:] + [callee]
                reason = "recursion: %s" % "->".join(cycle)
                reasons.add(reason)
                self_reasons.add(reason)
                if recursion_depth > 0:
                    cycle_nodes = cycle[:-1]
                    cycle_cost = sum(
                        funcs[c]["self"] if c in funcs else 0 for c in cycle_nodes
                    )
                    cycle_total = cycle_cost * recursion_depth
                    if cycle_total > worst_callee:
                        worst_callee = cycle_total
                        est_frames = []
                        for _ in range(recursion_depth):
                            for node in cycle_nodes:
                                est_frames.append((node, "recursive estimate"))
                        best_substack = est_frames
                continue

            callee_total, callee_unc, callee_reasons, callee_stack, _ = dfs(
                callee, cur_path
            )
            if callee_unc:
                uncertain = True
                reasons.update(callee_reasons)
            if callee_total > worst_callee:
                worst_callee = callee_total
                best_substack = callee_stack

        total = self_size + worst_callee
        result = (
            total,
            uncertain,
            reasons,
            [(func, None)] + best_substack,
            self_reasons,
        )
        cache[func] = result
        return result

    results = {}
    all_funcs = set(funcs.keys()) | set(graph.keys())
    for func in all_funcs:
        total, uncertain, reasons, stack, self_reasons = dfs(func, [])
        # Skip symbols with no DWARF data and zero stack
        if func not in funcs and total == 0:
            continue
        results[func] = {
            "self": funcs[func]["self"] if func in funcs else 0,
            "total": total,
            "uncertain": uncertain,
            "reasons": list(reasons),
            "stack": stack,
            "self_reasons": list(self_reasons),
        }

    return results


def reason_marker(reasons):
    """Return a single-char marker for uncertainty.

    Marker priority (highest first):
      ? = no DWARF data
      * = dynamic stack (alloca/VLA)
      @ = recursion
      ^ = indirect call (function pointer)
    """

    for reason in reasons:
        if reason == "no DWARF data":
            return "?"
    for reason in reasons:
        if reason.startswith("dynamic stack"):
            return "*"
    for reason in reasons:
        if reason.startswith("recursion:"):
            return "@"
    for reason in reasons:
        if reason == "indirect call (function pointer)":
            return "^"
    return ""


def frame_info(funcs, results, frame):
    """Return (size_str, func_name, file:line) for one stack frame.

    size_str is prefixed with a marker indicating the function's own
    uncertainty type (see reason_marker).  func_name includes the code
    size in parentheses when available, e.g. ``main(128)``.
    """

    func, _note = frame
    info = funcs.get(func)
    res = results.get(func)
    if info:
        self_reasons = res["self_reasons"] if res else info["reasons"]
        prefix = reason_marker(self_reasons)
        code_size = info.get("code_size", 0)
        name = "%s(%d)" % (func, code_size) if code_size else func
        return (
            "%s%d" % (prefix, info["self"]),
            name,
            "%s:%s" % (info["file"], info["line"]),
        )
    return ("?", func, "?:?")


def format_text(results, funcs, rank):
    """Format results as aligned table with deepest call stack frames."""

    # Compute common path prefix to shorten File:Line column
    paths = [
        f["file"]
        for f in funcs.values()
        if f["file"] not in ("?", "") and f["file"].startswith("/")
    ]
    if paths:
        prefix = os.path.commonpath(paths)
        if not prefix.endswith("/"):
            prefix += "/"
    else:
        prefix = ""

    maxname = 40
    items = sorted(results.items(), key=lambda x: x[1]["total"], reverse=True)
    if rank > 0:
        items = items[:rank]

    hdr = ("Total", "Self", "Backtrace", "File:Line")
    w = list(map(len, hdr))

    rows = []
    for func, info in items:
        frames = [frame_info(funcs, results, f) for f in info["stack"]]
        # Strip common prefix from file paths
        if prefix:
            plen = len(prefix)
            frames = [
                (
                    sz,
                    fn,
                    loc[plen:] if loc.startswith(prefix) else loc,
                )
                for sz, fn, loc in frames
            ]
        # Derive Total marker from the displayed stack frames' own reasons,
        # not from all propagated reasons across every branch.
        stack_reasons = []
        for f, _note in info["stack"]:
            res = results.get(f)
            if res:
                stack_reasons.extend(res["self_reasons"])
            elif f not in funcs:
                stack_reasons.append("no DWARF data")
        marker = reason_marker(stack_reasons) if info["uncertain"] else ""
        total_str = "%s%d" % (marker, info["total"])
        w[0] = max(w[0], len(total_str))
        for sz, fn, loc in frames:
            w[1] = max(w[1], len(sz))
            w[2] = max(w[2], min(len(fn), maxname))
            w[3] = max(w[3], len(loc))
        rows.append((total_str, frames))

    def trunc(s):
        return s[: maxname - 3] + "..." if len(s) > maxname else s

    fmt = "%%%ds  %%%ds  %%-%ds  %%s" % (w[0], w[1], w[2])
    lines = [fmt % hdr]
    lines.append(fmt % ("-" * w[0], "-" * w[1], "-" * w[2], "-" * w[3]))

    for total_str, frames in rows:
        for i, (sz, fn, loc) in enumerate(frames):
            if i == 0:
                lines.append(fmt % (total_str, sz, trunc(fn + "~"), loc))
            else:
                lines.append(fmt % ("", sz, trunc(fn), loc))
        if not frames:
            lines.append(fmt % (total_str, "", "", ""))

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze per-function stack usage from DWARF debug info, "
        "combined with disassembly-based call graph analysis to compute "
        "worst-case total (self + callees) stack depth.",
    )
    parser.add_argument(
        "elf",
        help="path to ELF file with DWARF debug info",
    )
    parser.add_argument(
        "-p",
        "--prefix",
        default="",
        help="toolchain prefix (e.g. arm-none-eabi- or llvm-)",
    )
    parser.add_argument(
        "-n",
        "--rank",
        type=int,
        default=0,
        help="show top N functions (default: 0 = all)",
    )
    parser.add_argument(
        "-r",
        "--recursion-depth",
        type=int,
        default=0,
        help="assumed recursion depth for stack estimation (default: 0). "
        "When > 0, recursive cycle cost is multiplied by this value.",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.elf):
        print("ELF file not found: %s" % args.elf, file=sys.stderr)
        sys.exit(1)

    # Phase 1: Symbol table
    addr_to_name, name_to_addr, func_sizes, is_thumb, machine = build_symbol_map(
        args.prefix,
        args.elf,
    )
    if not addr_to_name:
        print("No function symbols found in ELF", file=sys.stderr)
        sys.exit(1)

    # Phase 2: Stack sizes from DWARF .debug_frame
    funcs = parse_stack_sizes(
        args.prefix,
        args.elf,
        addr_to_name,
        name_to_addr,
        func_sizes,
    )
    if not funcs:
        print(
            "No stack size data found in DWARF .debug_frame",
            file=sys.stderr,
        )
        sys.exit(1)

    # Phase 3: Source locations via addr2line
    resolve_sources(args.prefix, args.elf, funcs, name_to_addr)

    # Phase 4: Call graph via objdump
    graph, dynamic_funcs = parse_call_graph(args.prefix, args.elf, machine)

    # Mark functions with dynamic stack allocation (alloca/VLA)
    for name in dynamic_funcs:
        if name in funcs:
            funcs[name]["reasons"].append("dynamic stack (alloca/VLA)")

    results = compute_worst_stack(funcs, graph, args.recursion_depth)

    print(format_text(results, funcs, args.rank))


if __name__ == "__main__":
    main()
