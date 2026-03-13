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

"""Static stack usage analyzer combining GCC .su files with objdump call graphs.

Reports per-function self stack usage and worst-case total stack depth through
the call chain. Estimates and flags uncertain cases such as alloca/VLA,
recursion, and indirect calls (function pointers).
"""

import argparse
import csv
import io
import os
import re
import subprocess
import sys

# GCC .su file format: file.c:line:col:function_name\tsize\tqualifier
SU_LINE_RE = re.compile(r"^(.*?\.[a-zA-Z]+):(\d+):(\d+):(.+)\t(\d+)\t(\S+)$")

# objdump function header: 00000000 <function_name>:
FUNC_RE = re.compile(r"^[0-9a-fA-F]+\s+<(.+)>:\s*$")

# Universal call instruction regex covering all NuttX architectures:
#   ARM:      bl, blx, b.w          ARM64:    bl, blr
#   x86:      call, callq           RISC-V:   jal, jalr
#   Xtensa:   call0/4/8/12,         MIPS:     jal, jalr
#             callx0/4/8/12
#   AVR:      rcall, call           z80/z16:  call
#   OR1K:     l.jal, l.jalr         SPARC:    call, jmpl
#   TriCore:  call, calli           HC/Ren:   bsr, jsr
CALL_RE = re.compile(
    r"\s+"
    r"(?:"
    r"bl[rx]?|b\.w"  # ARM / ARM64
    r"|callx?(?:0|4|8|12)"  # Xtensa
    r"|callq?|calli?"  # x86 / TriCore
    r"|jalr?|l\.jalr?"  # RISC-V / MIPS / OR1K
    r"|rcall"  # AVR
    r"|jmpl"  # SPARC
    r"|[bj]sr"  # HC / Renesas
    r")"
    r"\s+(.*)"
)

# Direct call target: optional hex address then <function_name>
DIRECT_TARGET_RE = re.compile(r"(?:[0-9a-fA-F]+\s+)?<([^>+-]+)(?:[+-][^>]*)?>")


def parse_su_files(dirs):
    """Parse all .su files under given directories.

    Returns dict mapping function name to info dict with keys:
      file, line, self, qualifier, reasons
    """

    funcs = {}
    for d in dirs:
        for root, _, files in os.walk(d):
            for f in files:
                if not f.endswith(".su"):
                    continue
                path = os.path.join(root, f)
                with open(path, encoding="utf-8", errors="replace") as fh:
                    for line in fh:
                        line = line.rstrip("\n")
                        m = SU_LINE_RE.match(line)
                        if not m:
                            continue

                        filename = m.group(1)
                        lineno = m.group(2)
                        funcname = m.group(4)
                        size = int(m.group(5))
                        qualifier = m.group(6)

                        reasons = []
                        if "dynamic" in qualifier:
                            if "bounded" in qualifier:
                                reasons.append("dynamic stack (bounded estimate)")
                            else:
                                reasons.append("dynamic stack (alloca/VLA)")

                        if funcname in funcs:
                            prev = funcs[funcname]
                            # Different file:line means ambiguous static funcs
                            if prev["file"] != filename or prev["line"] != lineno:
                                if (
                                    "ambiguous .su (multiple definitions)"
                                    not in prev["reasons"]
                                ):
                                    prev["reasons"].append(
                                        "ambiguous .su (multiple definitions)"
                                    )
                            # Keep entry with largest self size
                            if prev["self"] >= size:
                                continue

                        funcs[funcname] = {
                            "file": filename,
                            "line": lineno,
                            "self": size,
                            "qualifier": qualifier,
                            "reasons": reasons,
                        }
    return funcs


def parse_call_graph(objdump_bin, elf_path):
    """Build call graph from objdump disassembly.

    Returns dict mapping caller -> set of (callee_name_or_None, is_indirect).
    """

    proc = subprocess.Popen(
        [objdump_bin, "-d", elf_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    graph = {}
    current_func = None

    for line in proc.stdout:
        fm = FUNC_RE.match(line)
        if fm:
            current_func = fm.group(1)
            if current_func not in graph:
                graph[current_func] = set()
            continue

        if current_func is None:
            continue

        cm = CALL_RE.search(line)
        if not cm:
            continue

        operand = cm.group(1).strip()
        tm = DIRECT_TARGET_RE.match(operand)
        if tm:
            callee = tm.group(1)
            graph[current_func].add((callee, False))
        else:
            # Register operand = indirect call (function pointer)
            graph[current_func].add((None, True))

    ret = proc.wait()
    if ret != 0:
        print(
            "Error running objdump: %s" % proc.stderr.read().strip(),
            file=sys.stderr,
        )
        sys.exit(1)

    return graph


def compute_worst_stack(funcs, graph, recursion_depth):
    """Compute worst-case total stack for every function via memoized DFS.

    Args:
        funcs: dict from parse_su_files
        graph: dict from parse_call_graph
        recursion_depth: how many times a recursive cycle body is counted
                         (0 = back-edges contribute nothing)

    Returns dict mapping function name to result dict with keys:
      self, total, uncertain, reasons, file, line
    """

    cache = {}

    def dfs(func, path):
        if func in cache:
            return cache[func]

        info = funcs.get(func)
        self_size = info["self"] if info else 0
        reasons = list(info["reasons"]) if info else ["no .su data"]
        uncertain = bool(reasons)

        callees = graph.get(func, set())
        worst_callee = 0

        for callee, is_indirect in callees:
            if is_indirect:
                uncertain = True
                if "indirect call (function pointer)" not in reasons:
                    reasons.append("indirect call (function pointer)")
                continue

            cur_path = path + [func]
            if callee in cur_path:
                # Recursion detected
                uncertain = True
                idx = cur_path.index(callee)
                cycle = cur_path[idx:] + [callee]
                reason = "recursion: %s" % "->".join(cycle)
                if reason not in reasons:
                    reasons.append(reason)
                if recursion_depth > 0:
                    cycle_cost = sum(
                        funcs[c]["self"] if c in funcs else 0 for c in cycle[:-1]
                    )
                    worst_callee = max(worst_callee, cycle_cost * recursion_depth)
                continue

            callee_total, callee_unc, callee_reasons = dfs(callee, cur_path)
            if callee_unc:
                uncertain = True
                for r in callee_reasons:
                    if r not in reasons:
                        reasons.append(r)
            worst_callee = max(worst_callee, callee_total)

        total = self_size + worst_callee
        result = (total, uncertain, reasons)
        cache[func] = result
        return result

    results = {}
    all_funcs = set(funcs.keys()) | set(graph.keys())
    for func in all_funcs:
        total, uncertain, reasons = dfs(func, [])
        info = funcs.get(func)
        results[func] = {
            "self": info["self"] if info else 0,
            "total": total,
            "uncertain": uncertain,
            "reasons": reasons,
            "file": info["file"] if info else "",
            "line": info["line"] if info else "",
        }

    return results


def summarize_reasons(reasons):
    """Summarize detailed reasons into short category labels."""

    categories = []
    for r in reasons:
        if r.startswith("recursion:"):
            cat = "recursion"
        elif r.startswith("indirect"):
            cat = "indirect"
        elif r == "no .su data":
            cat = "no .su"
        elif r.startswith("ambiguous"):
            cat = "ambiguous"
        elif "bounded" in r:
            cat = "dynamic,bounded"
        elif "dynamic" in r:
            cat = "dynamic"
        else:
            cat = r
        if cat not in categories:
            categories.append(cat)
    return ", ".join(categories)


def format_text(results, rank, detail):
    """Format results as aligned text table."""

    items = sorted(results.items(), key=lambda x: x[1]["total"], reverse=True)
    if rank > 0:
        items = items[:rank]

    hdr = ("Function", "Self", "Total", " ", "File:Line", "Reasons")
    widths = list(map(len, hdr))

    rows = []
    for func, info in items:
        loc = "%s:%s" % (info["file"], info["line"]) if info["file"] else ""
        reasons = info["reasons"]
        row = (
            func,
            str(info["self"]),
            str(info["total"]),
            "*" if info["uncertain"] else "",
            loc,
            ", ".join(reasons) if detail else summarize_reasons(reasons),
        )
        rows.append(row)
        for i, val in enumerate(row):
            widths[i] = max(widths[i], len(val))

    fmt = "  ".join("%%-%ds" % w for w in widths)
    lines = [fmt % hdr, fmt % tuple("-" * w for w in widths)]
    lines.extend(fmt % row for row in rows)
    return "\n".join(lines)


def format_csv(results, rank, detail):
    """Format results as CSV."""

    items = sorted(results.items(), key=lambda x: x[1]["total"], reverse=True)
    if rank > 0:
        items = items[:rank]

    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(["Function", "Self", "Total", "Uncertain", "File:Line", "Reasons"])
    for func, info in items:
        loc = "%s:%s" % (info["file"], info["line"]) if info["file"] else ""
        reasons = info["reasons"]
        writer.writerow(
            [
                func,
                info["self"],
                info["total"],
                "*" if info["uncertain"] else "",
                loc,
                ", ".join(reasons) if detail else summarize_reasons(reasons),
            ]
        )
    return buf.getvalue().rstrip("\n")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze per-function stack usage from GCC .su files, "
        "optionally combined with objdump call graph analysis to compute "
        "worst-case total (self + callees) stack depth.",
    )
    parser.add_argument(
        "dirs",
        nargs="+",
        help="directories to search for .su files",
    )
    parser.add_argument(
        "-d",
        "--objdump",
        help="path to objdump binary (enables callee analysis)",
    )
    parser.add_argument(
        "-e",
        "--elf",
        help="path to ELF file (required with --objdump)",
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
    parser.add_argument(
        "--csv",
        action="store_true",
        help="output in CSV format",
    )
    parser.add_argument(
        "--detail",
        action="store_true",
        help="show detailed reasons (default: summary only)",
    )

    args = parser.parse_args()

    if args.objdump and not args.elf:
        parser.error("--elf is required when --objdump is specified")
    if args.elf and not args.objdump:
        parser.error("--objdump is required when --elf is specified")

    funcs = parse_su_files(args.dirs)
    if not funcs:
        print("No .su files found in: %s" % " ".join(args.dirs), file=sys.stderr)
        sys.exit(1)

    if args.objdump and args.elf:
        graph = parse_call_graph(args.objdump, args.elf)
        results = compute_worst_stack(funcs, graph, args.recursion_depth)
    else:
        # Self-only mode: no call graph analysis
        results = {}
        for func, info in funcs.items():
            results[func] = {
                "self": info["self"],
                "total": info["self"],
                "uncertain": bool(info["reasons"]),
                "reasons": list(info["reasons"]),
                "file": info["file"],
                "line": info["line"],
            }

    if args.csv:
        print(format_csv(results, args.rank, args.detail))
    else:
        print(format_text(results, args.rank, args.detail))


if __name__ == "__main__":
    main()
