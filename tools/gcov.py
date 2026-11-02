#!/usr/bin/env python3
# tools/gcov.py

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

"""Code coverage report generator for NuttX.

Supports both GCC gcov and Clang source-based coverage. The toolchain is
inferred from the ELF binary's `.comment` section, and the cross-compile
prefix (if any) is inferred from the ELF machine type, so users no longer
have to specify `-t`/`--toolchain`.

Usage example:

    gcov.py -e build/nuttx -s build -a coverage_data.bin -o report_dir

The required tools (gcov/lcov/genhtml for GCC; llvm-profdata/llvm-cov/
llvm-readelf/ld.lld for Clang) are resolved from PATH; the cross-compile
gcov is matched by arch (e.g. aarch64-none-elf-gcov for EM_AARCH64).

Use `-b OLD=NEW` (repeatable) to rewrite source paths recorded by the
compiler at build time to local paths, mirroring `-fdebug-prefix-map`:

    gcov.py -b /old/build/path=/new/local/path ...
"""

import argparse
import concurrent.futures
import json
import os
import re
import shutil
import struct
import subprocess
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# =============================================================================
# 1. CONSTANTS
# =============================================================================

# LLVM raw profile magic bytes (first 8 bytes of .profraw).
#
# The magic is a uint64_t written by the runtime in native byte order
# (high byte 0xff, low byte 0x81; "lprofr"/"lprofR" in the middle), so
# big-endian platforms produce \xff..\x81 and little-endian platforms
# (ARM aarch64, x86_64, etc.) produce \x81..\xff. Both forms must be
# accepted, mirroring LLVM's RawInstrProfReader::hasFormat which compares
# the on-disk magic against both the native and byte-swapped values.
#
# 64-bit BE: \xff lprofr \x81   /   64-bit LE: \x81 rforpl \xff
# 32-bit BE: \xff lprofR \x81   /   32-bit LE: \x81 Rforpl \xff
PROFRAW_MAGIC_64 = b"\xff\x6c\x70\x72\x6f\x66\x72\x81"
PROFRAW_MAGIC_32 = b"\xff\x6c\x70\x72\x6f\x66\x52\x81"
PROFRAW_MAGICS = (
    PROFRAW_MAGIC_64,
    PROFRAW_MAGIC_64[::-1],
    PROFRAW_MAGIC_32,
    PROFRAW_MAGIC_32[::-1],
)

# gcov binary pack heuristic:
# First 4 bytes are path_len (LE uint32), typically 30~200 for gcda paths.
# Followed by a path string starting with a printable char (not \x00 or \xff).
GCOV_BINPACK_PATH_LEN_MIN = 10
GCOV_BINPACK_PATH_LEN_MAX = 4096

# Suffix appended to a -a input file to name the directory holding the
# coverage data extracted from it (hex dump / binpack -> individual files).
CONVERT_SUFFIX = "_convert"

# ELF e_machine values we care about for cross-prefix inference.
EM_386 = 3
EM_X86_64 = 62
EM_ARM = 40
EM_AARCH64 = 183
EM_RISCV = 243
EM_XTENSA = 94
EM_TRICORE = 44

# Candidate cross-compile gcov binary names per architecture, in priority
# order. The first one found on PATH is used.
GCC_CROSS_PREFIXES = {
    EM_AARCH64: ["aarch64-none-elf-gcov", "aarch64-linux-gnu-gcov"],
    EM_ARM: ["arm-none-eabi-gcov", "arm-linux-gnueabi-gcov"],
    EM_RISCV: ["riscv64-unknown-elf-gcov", "riscv32-unknown-elf-gcov"],
    EM_TRICORE: ["tricore-elf-gcov"],
    EM_XTENSA: ["xtensa-esp32-elf-gcov", "xtensa-esp32s3-elf-gcov"],
    EM_X86_64: ["gcov"],
    EM_386: ["gcov"],
}


# =============================================================================
# 2. LOGGING / SUBPROCESS HELPERS
# =============================================================================


def log(msg: str) -> None:
    print(msg, flush=True)


def warn(msg: str) -> None:
    print(f"warning: {msg}", flush=True)


def die(msg: str, code: int = 1) -> None:
    print(f"error: {msg}", file=sys.stderr, flush=True)
    sys.exit(code)


def print_report_link(report_dir: str) -> None:
    """Print the file:// link to the generated HTML report's index page.

    Only call after confirming the report was produced; the index is the
    common entry point for both the lcov/genhtml and llvm-cov outputs.
    """
    log(
        "Copy the following link and open it in the browser to view the "
        "coverage report:"
    )
    log(f"file://{os.path.join(report_dir, 'index.html')}")


def run_cmd(cmd: List[str], check: bool = True) -> int:
    """Run a subprocess, streaming output, returning its exit code."""
    log(" ".join(cmd))
    return subprocess.run(
        cmd,
        check=check,
        stdout=sys.stdout,
        stderr=sys.stdout,
    ).returncode


# =============================================================================
# 3. FORMAT DETECTION
# =============================================================================


def is_profraw_file(path: str) -> bool:
    """Check if a file is an LLVM profraw file by magic number."""
    try:
        with open(path, "rb") as f:
            return f.read(8) in PROFRAW_MAGICS
    except (IOError, OSError):
        return False


def is_gcov_binpack_file(path: str) -> bool:
    """Check if a file is a gcov binary pack file.

    Binary format: [path_len(4B LE)][path+NUL][data_size(4B LE)][data]...
    Heuristic: first 4 bytes are a small uint32, followed by a printable path.
    """
    try:
        with open(path, "rb") as f:
            header = f.read(8)
            if len(header) < 8:
                return False
            path_len = struct.unpack_from("<I", header, 0)[0]
            if path_len < GCOV_BINPACK_PATH_LEN_MIN:
                return False
            if path_len > GCOV_BINPACK_PATH_LEN_MAX:
                return False
            first_char = header[4]
            return 0x20 <= first_char < 0x7F
    except (IOError, OSError):
        return False


# =============================================================================
# 4. ELF INSPECTOR (toolchain + machine, no external readelf)
# =============================================================================


@dataclass
class ElfInfo:
    toolchain: str  # "gcc" | "clang"
    e_machine: int

    @property
    def arch_name(self) -> str:
        return {
            EM_386: "x86",
            EM_X86_64: "x86_64",
            EM_ARM: "arm",
            EM_AARCH64: "aarch64",
            EM_RISCV: "riscv",
            EM_XTENSA: "xtensa",
        }.get(self.e_machine, f"em{self.e_machine}")


def _read_section(f, header: bytes, name: str) -> bytes:
    """Read a named section's raw contents from an open ELF file.

    Pure-Python ELF parsing; supports both ELF32 and ELF64, both endians.
    Reads only the section-header table, the section-name string table, and
    the matched section — never the whole file. ``header`` must hold at least
    the first 64 bytes of the file (the ELF header). Returns empty bytes if
    the section is missing or the ELF is malformed.
    """
    if len(header) < 0x40 or header[:4] != b"\x7fELF":
        return b""

    elf_class = header[4]  # 1 = ELF32, 2 = ELF64
    little = header[5] == 1
    end = "<" if little else ">"

    if elf_class == 1:
        # ELF32 header; section header: name(0), offset(16), size(20)
        e_shoff = struct.unpack_from(end + "I", header, 0x20)[0]
        e_shentsize = struct.unpack_from(end + "H", header, 0x2E)[0]
        e_shnum = struct.unpack_from(end + "H", header, 0x30)[0]
        e_shstrndx = struct.unpack_from(end + "H", header, 0x32)[0]
        off_off, size_off = 16, 20
    elif elf_class == 2:
        # ELF64 header; section header: name(0), offset(24), size(32)
        e_shoff = struct.unpack_from(end + "Q", header, 0x28)[0]
        e_shentsize = struct.unpack_from(end + "H", header, 0x3A)[0]
        e_shnum = struct.unpack_from(end + "H", header, 0x3C)[0]
        e_shstrndx = struct.unpack_from(end + "H", header, 0x3E)[0]
        off_off, size_off = 24, 32
    else:
        return b""

    if e_shnum == 0 or e_shoff == 0:
        return b""

    # Width of the sh_offset / sh_size fields: 4 bytes on ELF32, 8 on ELF64.
    addr_fmt = end + ("I" if elf_class == 1 else "Q")

    def pread(offset: int, size: int) -> bytes:
        f.seek(offset)
        return f.read(size)

    # Read the whole section-header table (small: e_shnum * e_shentsize).
    sht = pread(e_shoff, e_shnum * e_shentsize)

    def section_header(idx: int) -> bytes:
        start = idx * e_shentsize
        return sht[start : start + e_shentsize]

    # Read .shstrtab (section name string table) first.
    shstr_hdr = section_header(e_shstrndx)
    shstr_off = struct.unpack_from(addr_fmt, shstr_hdr, off_off)[0]
    shstr_size = struct.unpack_from(addr_fmt, shstr_hdr, size_off)[0]
    shstrtab = pread(shstr_off, shstr_size)

    target = name.encode() + b"\x00"

    for i in range(e_shnum):
        hdr = section_header(i)
        if len(hdr) < e_shentsize:
            continue
        sh_name = struct.unpack_from(end + "I", hdr, 0)[0]
        # Match the NUL-terminated name from shstrtab
        end_pos = shstrtab.find(b"\x00", sh_name)
        if end_pos < 0:
            continue
        sec_name = shstrtab[sh_name:end_pos]
        if sec_name + b"\x00" == target:
            sh_off = struct.unpack_from(addr_fmt, hdr, off_off)[0]
            sh_sz = struct.unpack_from(addr_fmt, hdr, size_off)[0]
            return pread(sh_off, sh_sz)
    return b""


def inspect_elf(path: str) -> ElfInfo:
    """Inspect ELF for toolchain and machine type.

    Toolchain is detected from the `.comment` section: presence of "clang"
    means clang, otherwise gcc. e_machine is read from the ELF header. Only
    the ELF header, section-header table, .shstrtab and .comment are read,
    so this is cheap even for very large (e.g. ~700MB clang) binaries.
    """
    with open(path, "rb") as f:
        header = f.read(64)
        if len(header) < 0x40 or header[:4] != b"\x7fELF":
            die(f"not an ELF file: {path}")

        elf_class = header[4]
        if elf_class not in (1, 2):
            die(f"unknown ELF class {elf_class}")
        little = header[5] == 1
        end = "<" if little else ">"

        # e_machine sits at offset 0x12 in both ELF32 and ELF64 headers.
        e_machine = struct.unpack_from(end + "H", header, 0x12)[0]

        comment = _read_section(f, header, ".comment")

    toolchain = "clang" if b"clang" in comment.lower() else "gcc"
    return ElfInfo(toolchain=toolchain, e_machine=e_machine)


# =============================================================================
# 5. DATA EXTRACTION (gcda hex dump / gcov binpack / profraw)
# =============================================================================


def _safe_join(out_dir: str, rel: str) -> Optional[str]:
    """Join ``rel`` (from untrusted dump content) under ``out_dir`` safely.

    Strips leading separators and rejects any path that escapes ``out_dir``
    via ``..`` traversal. Returns the absolute path, or None if it would
    escape (caller should warn and skip).
    """
    rel = rel.lstrip("/").lstrip("\\")
    out_abs = os.path.realpath(out_dir)
    dst = os.path.realpath(os.path.join(out_abs, rel))
    if dst != out_abs and not dst.startswith(out_abs + os.sep):
        return None
    return dst


def parse_gcda_hex_dump(path: str) -> str:
    """Parse hex dump log containing 'gcov start/end' blocks.

    Works for both gcc (.gcda) and clang (.profraw) hex dumps since
    the clang coverage.c uses the same format.
    """
    with open(path, "r") as fp:
        lines = fp.read().strip().splitlines()

    out_dir = path + CONVERT_SUFFIX
    os.makedirs(out_dir, exist_ok=True)

    started = False
    filename = ""
    payload = ""
    size = 0

    for line in lines:
        if line.startswith("gcov start"):
            started = True
            m = re.search(r"filename:(.*?)\s+size:\s*(\d+)Byte", line)
            if m:
                filename = m.group(1)
                size = int(m.group(2))
            continue

        if not started:
            continue

        try:
            if line.startswith("gcov end"):
                started = False
                if size != len(payload) // 2:
                    raise ValueError(
                        f"size mismatch for {filename}: "
                        f"expected {size}, got {len(payload) // 2}"
                    )

                blob = bytearray.fromhex(payload)

                m = re.search(r"checksum:\s*(0x[0-9a-fA-F]+)", line)
                if m:
                    checksum = int(m.group(1), 16)
                    expected = sum(blob) % 65536
                    if checksum != expected:
                        raise ValueError(
                            f"checksum mismatch for {filename}: "
                            f"expected {checksum}, got {expected}"
                        )
                else:
                    # No checksum token: still write the data (it passed the
                    # size check) but flag that it could not be verified.
                    warn(f"no checksum for {filename}; writing unverified data")

                out_path = _safe_join(out_dir, filename)
                if out_path is None:
                    warn(f"skipping unsafe path in dump: {filename}")
                else:
                    os.makedirs(os.path.dirname(out_path), exist_ok=True)
                    with open(out_path, "wb") as out_fp:
                        out_fp.write(blob)
                        log(f"write {out_path} success")
                payload = ""
            else:
                payload += line.strip()
        except Exception as e:
            warn(f"error processing {path}: {e}")
            log(f"  gcov start filename:{filename} size:{size}")
            log(f"  payload[:200]={payload[:200]}")
            payload = ""
            started = False

    return out_dir


def parse_gcov_binpack(path: str) -> str:
    """Parse gcov binary pack file into individual gcda files.

    Binary format: [path_len(4B LE)][path+NUL][data_size(4B LE)][data]...
    """
    with open(path, "rb") as f:
        data = f.read()

    out_dir = path + CONVERT_SUFFIX
    os.makedirs(out_dir, exist_ok=True)

    offset = 0
    count = 0
    errors = 0

    while offset < len(data):
        if offset + 4 > len(data):
            break

        path_len = struct.unpack_from("<I", data, offset)[0]
        offset += 4

        if (
            path_len == 0
            or path_len > GCOV_BINPACK_PATH_LEN_MAX
            or offset + path_len > len(data)
        ):
            warn(f"invalid path_len {path_len} at offset {offset - 4}")
            errors += 1
            break

        filepath = data[offset : offset + path_len].rstrip(b"\x00").decode("utf-8")
        offset += path_len

        if offset + 4 > len(data):
            warn(f"truncated at data_size for {filepath}")
            errors += 1
            break

        data_size = struct.unpack_from("<I", data, offset)[0]
        offset += 4

        if offset + data_size > len(data):
            warn(f"truncated data for {filepath}")
            gcda_data = data[offset:]
            offset = len(data)
            errors += 1
        else:
            gcda_data = data[offset : offset + data_size]
            offset += data_size

        out_path = _safe_join(out_dir, filepath)
        if out_path is None:
            warn(f"skipping unsafe path in binpack: {filepath}")
            errors += 1
            continue
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "wb") as out_fp:
            out_fp.write(gcda_data)
        count += 1

    log(f"parsed binary pack: {count} files ({errors} errors)")
    return out_dir


def find_profraw_files(directory: str) -> List[str]:
    """Recursively find all .profraw files in a directory."""
    found = []
    for root, _, files in os.walk(directory):
        for f in files:
            fpath = os.path.join(root, f)
            if f.endswith(".profraw") or is_profraw_file(fpath):
                found.append(fpath)
    return found


def resolve_data_inputs(inputs: List[str], delete_after: bool) -> List[str]:
    """Normalize -a inputs to a list of directories or profraw files.

    Files are auto-classified:
      * .profraw (binary)        -> kept as-is
      * gcov binpack (binary)    -> parsed to a directory
      * hex-dump text log        -> parsed to a directory
    Directories are passed through.
    """
    out: List[str] = []
    for entry in inputs:
        if not os.path.exists(entry):
            die(f"input not found: {entry}")
        if os.path.isfile(entry):
            abs_path = os.path.abspath(entry)
            if is_profraw_file(abs_path):
                out.append(abs_path)
            elif is_gcov_binpack_file(abs_path):
                out.append(parse_gcov_binpack(abs_path))
                if delete_after:
                    os.remove(entry)
            else:
                out.append(parse_gcda_hex_dump(abs_path))
                if delete_after:
                    os.remove(entry)
        else:
            out.append(os.path.abspath(entry))
    return out


# =============================================================================
# 6. TOOL RESOLUTION
# =============================================================================


@dataclass
class GccTools:
    gcov: str
    lcov: str
    genhtml: str


@dataclass
class LlvmTools:
    profdata: str
    cov: str
    readelf: Optional[str] = None
    lld: Optional[str] = None


def _which_first(candidates: List[str]) -> Optional[str]:
    """Return the first candidate tool found on PATH, or None."""
    for cand in candidates:
        found = shutil.which(cand)
        if found:
            return found
    return None


def resolve_gcc_tools(elf: ElfInfo) -> GccTools:
    """Resolve gcov/lcov/genhtml from PATH based on the ELF arch."""
    candidates = GCC_CROSS_PREFIXES.get(elf.e_machine, ["gcov"])
    gcov = _which_first(candidates)
    if gcov is None:
        die(
            f"could not find gcov for {elf.arch_name} on PATH. " f"Tried: {candidates}."
        )

    lcov = _which_first(["lcov"])
    if lcov is None:
        die("lcov not found on PATH. Install lcov.")

    genhtml = _which_first(["genhtml"])
    if genhtml is None:
        die("genhtml not found on PATH. Install lcov (provides genhtml).")

    log(f"gcov:    {gcov}")
    log(f"lcov:    {lcov}")
    log(f"genhtml: {genhtml}")
    return GccTools(gcov=gcov, lcov=lcov, genhtml=genhtml)


def resolve_llvm_tools() -> LlvmTools:
    """Resolve llvm-profdata/llvm-cov/llvm-readelf/ld.lld from PATH."""
    profdata = _which_first(["llvm-profdata"])
    if profdata is None:
        die("llvm-profdata not found on PATH. Install LLVM.")
    cov = _which_first(["llvm-cov"])
    if cov is None:
        die("llvm-cov not found on PATH. Install LLVM.")
    readelf = _which_first(["llvm-readelf"])
    lld = _which_first(["ld.lld"])

    log(f"llvm-profdata: {profdata}")
    log(f"llvm-cov:      {cov}")
    log(f"llvm-readelf:  {readelf or '(not found, covmap fix disabled)'}")
    log(f"ld.lld:        {lld or '(not found, covmap fix disabled)'}")
    return LlvmTools(profdata=profdata, cov=cov, readelf=readelf, lld=lld)


# =============================================================================
# 7. PATH-MAPPING (-b OLD=NEW)
# =============================================================================


@dataclass
class PathMap:
    pairs: List[Tuple[str, str]] = field(default_factory=list)

    @classmethod
    def parse(cls, raw_args: Optional[List[str]]) -> "PathMap":
        pairs: List[Tuple[str, str]] = []
        for raw in raw_args or []:
            if "=" not in raw:
                die(f"-b expects OLD=NEW form, got: {raw!r}")
            old, new = raw.split("=", 1)
            if not old:
                die(f"-b OLD part must be non-empty: {raw!r}")
            pairs.append((old, new))
        return cls(pairs=pairs)

    def lcov_substitute_args(self) -> List[str]:
        """Render --substitute args for lcov."""
        out: List[str] = []
        for old, new in self.pairs:
            # Use '#' as the sed delimiter to avoid clashing with '/' in paths
            out.extend(["--substitute", f"s#{old}#{new}#g"])
        return out

    def llvm_path_equivalence_args(self) -> List[str]:
        """Render -path-equivalence args for llvm-cov subcommands."""
        out: List[str] = []
        for old, new in self.pairs:
            out.append(f"-path-equivalence={old},{new}")
        return out


# =============================================================================
# 8. UTILITIES
# =============================================================================


def copy_files_with_suffix(suffix: str, source: str, target: str) -> None:
    """Mirror files matching `*<suffix>` from source into target.

    Skips paths inside `target` to avoid recursive copies when target is a
    descendant of source.
    """
    target_abs = os.path.abspath(target)
    for root, _, files in os.walk(source):
        if os.path.abspath(root).startswith(target_abs):
            continue
        for f in files:
            if f.endswith(suffix):
                src = os.path.join(root, f)
                rel = os.path.relpath(src, source)
                dst = os.path.join(target, rel)
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.copy2(src, dst)


def get_lcov_major_version(lcov_path: str) -> int:
    """Return major version of installed lcov, or 1 if undetermined."""
    try:
        result = subprocess.run(
            [lcov_path, "--version"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        m = re.search(r"LCOV version (\d+)", result.stdout)
        if m:
            return int(m.group(1))
    except (subprocess.TimeoutExpired, OSError, FileNotFoundError):
        pass
    return 1


def lcov_info_has_records(info_file: str) -> bool:
    """True if an lcov tracefile has at least one SF: record."""
    try:
        with open(info_file, "r", encoding="utf-8", errors="replace") as fp:
            for line in fp:
                if line.startswith("SF:"):
                    return True
    except (IOError, OSError):
        pass
    return False


# =============================================================================
# 9. GCC PIPELINE: gcda -> .info -> HTML
# =============================================================================


def _resolve_gcov_tool(gcov_path: str) -> Optional[str]:
    """Find the gcov-tool binary that matches a given gcov binary.

    Strategy (first hit wins):
      1. Sibling next to the gcov binary, e.g. /opt/.../bin/gcov-13 →
         /opt/.../bin/gcov-tool-13, or aarch64-none-elf-gcov →
         aarch64-none-elf-gcov-tool. This avoids the Ubuntu pitfall where
         /usr/bin/gcov is gcov-13 but /usr/bin/gcov-tool is the old gcov-tool-11.
      2. Plain `gcov-tool` on PATH (last resort).
    """
    real = os.path.realpath(gcov_path)
    base = os.path.basename(real)  # e.g. x86_64-linux-gnu-gcov-13
    directory = os.path.dirname(real)

    candidates: List[str] = []
    # Mirror "gcov[-suffix]" → "gcov-tool[-suffix]"
    if "gcov" in base:
        head, _, tail = base.partition("gcov")
        candidates.append(os.path.join(directory, head + "gcov-tool" + tail))

    # Also try the literal sibling next to the user-supplied path
    candidates.append(gcov_path + "-tool")

    for c in candidates:
        if os.path.isfile(c):
            return c

    return shutil.which("gcov-tool")


def gcc_merge_gcda(tools: GccTools, gcda_dirs: List[str], out_dir: str) -> None:
    """Merge multiple gcda directories into out_dir using `<gcov>-tool merge`.

    The first directory is copied to ``out_dir``; each subsequent directory
    is merged in via ``gcov-tool merge``. ``out_dir`` must not exist (we make
    a fresh copy every time so reruns are repeatable).
    """
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)

    merge_tool: Optional[str] = None
    if len(gcda_dirs) > 1:
        merge_tool = _resolve_gcov_tool(tools.gcov)
        if merge_tool is None:
            die(
                f"need gcov-tool to merge {len(gcda_dirs)} -a inputs but "
                "no matching gcov-tool was found on PATH. Install the "
                "matching package (e.g. apt install gcc-13)."
            )
        log(f"gcov-tool: {merge_tool}")

    shutil.copytree(gcda_dirs[0], out_dir)
    for extra in gcda_dirs[1:]:
        try:
            run_cmd([merge_tool, "merge", out_dir, extra, "-o", out_dir])
        except subprocess.CalledProcessError:
            die(
                f"gcov-tool merge failed for {extra}. Common causes: "
                "(1) version mismatch between gcov-tool and the .gcda files "
                "('incorrect gcov version' messages above) — install the "
                "matching gcov-tool so it is found on PATH, "
                "(2) the input was already merged in. "
                f"merge_tool={merge_tool}"
            )


def gcc_run_lcov(tools: GccTools, data_dir: str, path_map: PathMap) -> str:
    """Run lcov capture, returning path to the produced .info file."""
    info_file = data_dir + ".info"
    cmd = [
        tools.lcov,
        "-c",
        "-o",
        info_file,
        "--rc",
        "lcov_branch_coverage=1",
        "--gcov-tool",
        tools.gcov,
        "--ignore-errors",
        "gcov",
        "--directory",
        data_dir,
    ]
    cmd.extend(path_map.lcov_substitute_args())
    run_cmd(cmd)
    return info_file


def gcc_run_genhtml(tools: GccTools, info_file: str, report_dir: str) -> None:
    ignore_errors = "source"
    if get_lcov_major_version(tools.lcov) >= 2:
        ignore_errors = "source,category"

    run_cmd(
        [
            tools.genhtml,
            "--branch-coverage",
            "-o",
            report_dir,
            "--ignore-errors",
            ignore_errors,
            info_file,
        ]
    )


def run_gcc_pipeline(
    gcda_inputs: List[str],
    gcno_dir: str,
    result_dir: str,
    elf: ElfInfo,
    path_map: PathMap,
) -> None:
    tools = resolve_gcc_tools(elf)

    data_dir = os.path.join(result_dir, "data")
    report_dir = os.path.join(result_dir, "report")

    gcc_merge_gcda(tools, gcda_inputs, data_dir)
    copy_files_with_suffix(".gcno", gcno_dir, data_dir)

    info_file = gcc_run_lcov(tools, data_dir, path_map)

    if not lcov_info_has_records(info_file):
        warn(
            f"lcov produced no coverage records in {info_file}. This usually "
            "means the local gcov tool cannot decode the collected .gcda "
            "files (toolchain version mismatch — see 'version ... prefer ...' "
            "warnings above). Skipping HTML report generation."
        )
        return

    try:
        gcc_run_genhtml(tools, info_file, report_dir)
    except subprocess.CalledProcessError:
        die("failed to generate coverage report.")

    print_report_link(report_dir)


# =============================================================================
# 10. CLANG PIPELINE: profraw -> profdata -> HTML
# =============================================================================


def clang_merge_profraw(
    tools: LlvmTools, profraw_files: List[str], out_profdata: str
) -> str:
    cmd = [tools.profdata, "merge", "-sparse", "-o", out_profdata]
    cmd.extend(profraw_files)
    run_cmd(cmd)
    return out_profdata


def _check_covmap_one(readelf: str, path: str) -> Optional[str]:
    """Return ``path`` if its ELF has a ``__llvm_covmap`` section, else None."""
    try:
        result = subprocess.run(
            [readelf, "-S", path],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (subprocess.TimeoutExpired, OSError):
        return None
    return path if "__llvm_covmap" in result.stdout else None


def clang_find_covmap_objects(build_dir: str, readelf: str) -> List[str]:
    """Find all .o files containing ``__llvm_covmap`` sections.

    Each check spawns ``llvm-readelf -S``, so the work is I/O / subprocess
    bound — a thread pool parallelises it without the fork/pickle overhead of
    processes. Tiny trees run inline to skip pool spin-up. Output order is
    not preserved — safe because ``ld.lld -r`` is order-insensitive.
    """
    paths: List[str] = []
    for root, _, files in os.walk(build_dir):
        for f in files:
            if f.endswith(".o"):
                paths.append(os.path.join(root, f))

    if not paths:
        return []

    if len(paths) < 32:
        objects: List[str] = []
        for p in paths:
            hit = _check_covmap_one(readelf, p)
            if hit is not None:
                objects.append(hit)
        return objects

    workers = min(32, (os.cpu_count() or 4) * 4)
    objects = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        for hit in ex.map(lambda p: _check_covmap_one(readelf, p), paths):
            if hit is not None:
                objects.append(hit)
    return objects


def clang_link_relocatable(objects: List[str], out_path: str, lld: str) -> str:
    log(f"linking {len(objects)} objects into {out_path} ...")
    run_cmd([lld, "-r", "-o", out_path] + objects)
    return out_path


def _fingerprint_paths(paths: List[str]) -> List[List]:
    """(path, mtime_ns, size) records keying the relocatable cache."""
    records = []
    for p in paths:
        try:
            st = os.stat(p)
        except OSError:
            continue
        records.append([p, st.st_mtime_ns, st.st_size])
    records.sort()
    return records


def _load_cache_fingerprint(cache_path: str) -> Optional[dict]:
    try:
        with open(cache_path) as fp:
            return json.load(fp)
    except (OSError, ValueError):
        return None


def _save_cache_fingerprint(cache_path: str, payload: dict) -> None:
    try:
        with open(cache_path, "w") as fp:
            json.dump(payload, fp)
    except OSError as e:
        warn(f"failed to write cache fingerprint {cache_path}: {e}")


def _find_prebuilt_reloc(build_dir: str) -> Optional[str]:
    """Return a pre-built relocatable covmap object if -s points at one.

    Two accepted forms, both produced once and persisted so reports can
    be generated long after the build tree (the per-source ``.o`` files)
    is gone:

      * ``-s path/to/<obj>.o``  -> the file itself
      * ``-s path/to/dir``      -> ``dir/nuttx_llvm_covmap.o`` or
                                   ``dir/coverage_all.o`` if present

    ``nuttx_llvm_covmap.o`` is the slim LLVM coverage object emitted by the
    build (see tools/gen_llvm_covmap.py); ``coverage_all.o`` is the full
    relocatable that tools/gcov.py itself caches.  Returning a path here lets
    ``clang_resolve_cov_binary`` skip the readelf scan + ld.lld link.
    """
    if build_dir.endswith(".o") and os.path.isfile(build_dir):
        return build_dir
    if os.path.isdir(build_dir):
        for name in ("nuttx_llvm_covmap.o", "coverage_all.o"):
            candidate = os.path.join(build_dir, name)
            if os.path.isfile(candidate):
                return candidate
    return None


def clang_resolve_cov_binary(
    tools: LlvmTools, elf_path: str, build_dir: str, result_dir: str
) -> str:
    """Pick the binary llvm-cov should consult.

    On lld-linked NuttX binaries, __llvm_covfun/__llvm_covmap may sit at
    VMA=0, so llvm-cov refuses to read them from the final ELF; we link
    a relocatable .o from all source objects carrying covmap sections.

    Resolution order:
      0. No -s (build_dir is None): use the ELF directly. llvm-cov reads its
         __llvm_covmap/__llvm_covfun even at VMA 0; it then only reports the
         functions actually linked into the image. Linker-GC'd dead code is
         correctly excluded, so the coverage denominator reflects real code.
      1. A pre-built ``coverage_all.o`` reachable via ``-s`` (file or dir).
         Used as-is -- no build tree required. This includes dead code GC'd
         from the final image, inflating the denominator; use only when you
         specifically want every compiled (even unlinked) function counted.
      2. Scan ``build_dir`` for ``.o`` files with covmap sections and link
         them on the fly (result cached via a ``.fingerprint`` sidecar).
    """
    if build_dir is None:
        log("using the ELF directly for the report (no -s given)")
        return elf_path

    prebuilt = _find_prebuilt_reloc(build_dir)
    if prebuilt is not None:
        log(f"using pre-built relocatable object for report: {prebuilt}")
        return prebuilt

    if not (tools.readelf and tools.lld):
        if not tools.readelf:
            log("note: llvm-readelf not found, skipping relocatable object build.")
        if not tools.lld:
            log("note: ld.lld not found, skipping relocatable object build.")
        return elf_path

    reloc = os.path.join(result_dir, "coverage_all.o")
    cache_path = reloc + ".fingerprint"

    # Cache hit: skip both readelf scan and ld.lld link.
    if os.path.isfile(reloc) and os.path.isfile(cache_path):
        cached = _load_cache_fingerprint(cache_path)
        if cached and cached.get("build_dir") == os.path.abspath(build_dir):
            cached_records = cached.get("objects") or []
            cached_paths = [r[0] for r in cached_records]
            current_records = _fingerprint_paths(cached_paths)
            if current_records == cached_records:
                log(
                    f"cache hit: reusing {reloc} "
                    f"({len(cached_records)} objects unchanged)"
                )
                return reloc
            log("cache miss: covmap object set changed, rebuilding.")

    log("scanning for .o files with __llvm_covmap sections ...")
    objects = clang_find_covmap_objects(build_dir, tools.readelf)
    if not objects:
        log("no .o files with __llvm_covmap found, using ELF binary directly.")
        return elf_path

    try:
        clang_link_relocatable(objects, reloc, tools.lld)
        log(f"using relocatable object for report: {reloc}")
    except subprocess.CalledProcessError as e:
        warn(f"failed to build relocatable object: {e}; falling back to ELF.")
        return elf_path

    _save_cache_fingerprint(
        cache_path,
        {
            "build_dir": os.path.abspath(build_dir),
            "objects": _fingerprint_paths(objects),
        },
    )

    return reloc


def clang_run_report(
    tools: LlvmTools, profdata: str, binary: str, path_map: PathMap
) -> None:
    cmd = [tools.cov, "report", binary, f"-instr-profile={profdata}"]
    cmd.extend(path_map.llvm_path_equivalence_args())
    run_cmd(cmd)


def clang_run_show_html(
    tools: LlvmTools, profdata: str, binary: str, report_dir: str, path_map: PathMap
) -> None:
    cmd = [
        tools.cov,
        "show",
        binary,
        f"-instr-profile={profdata}",
        "-format=html",
        f"-output-dir={report_dir}",
        "-show-line-counts-or-regions",
        "-show-branches=count",
        "-show-directory-coverage",
    ]
    cmd.extend(path_map.llvm_path_equivalence_args())
    run_cmd(cmd)


def clang_run_export_lcov(
    tools: LlvmTools, profdata: str, binary: str, out_info: str, path_map: PathMap
) -> str:
    cmd = [
        tools.cov,
        "export",
        binary,
        f"-instr-profile={profdata}",
        "-format=lcov",
    ]
    cmd.extend(path_map.llvm_path_equivalence_args())
    log(" ".join(cmd))
    with open(out_info, "w") as fp:
        subprocess.run(cmd, check=True, stdout=fp, stderr=sys.stdout)
    return out_info


def run_clang_pipeline(
    data_inputs: List[str],
    gcno_dir: str,
    result_dir: str,
    elf_path: str,
    path_map: PathMap,
) -> None:
    tools = resolve_llvm_tools()

    data_dir = os.path.join(result_dir, "data")
    report_dir = os.path.join(result_dir, "report")
    os.makedirs(data_dir, exist_ok=True)

    # Collect all profraw files
    profraw_files: List[str] = []
    for entry in data_inputs:
        if os.path.isfile(entry):
            profraw_files.append(entry)
        elif os.path.isdir(entry):
            profraw_files.extend(find_profraw_files(entry))
    if not profraw_files:
        die("no .profraw files found in the specified -a paths.")
    log(f"found {len(profraw_files)} profraw file(s)")

    # profraw -> profdata
    profdata = os.path.join(data_dir, "merged.profdata")
    clang_merge_profraw(tools, profraw_files, profdata)

    cov_binary = clang_resolve_cov_binary(tools, elf_path, gcno_dir, result_dir)

    log("\n=== Coverage Summary ===")
    clang_run_report(tools, profdata, cov_binary, path_map)

    # Always export lcov .info so downstream tools (e.g. cross-toolchain merge
    # with gcc coverage via lcov --add-tracefile) can consume it regardless of
    # whether the HTML report succeeded.
    cov_info = os.path.join(result_dir, "coverage.info")
    try:
        clang_run_export_lcov(tools, profdata, cov_binary, cov_info, path_map)
    except subprocess.CalledProcessError as e:
        warn(f"llvm-cov export (lcov format) failed: {e}")

    # HTML: prefer llvm-cov show; fall back to genhtml(lcov-format export).
    try:
        clang_run_show_html(tools, profdata, cov_binary, report_dir, path_map)
        ok = True
    except subprocess.CalledProcessError as e:
        warn(f"llvm-cov show HTML failed: {e}; trying genhtml fallback.")
        ok = False

    if not ok:
        genhtml = shutil.which("genhtml")
        if genhtml is None:
            die("HTML generation failed and genhtml is not available.")
        try:
            run_cmd(
                [
                    genhtml,
                    "--branch-coverage",
                    "-o",
                    report_dir,
                    "--ignore-errors",
                    "source",
                    cov_info,
                ]
            )
            ok = True
        except subprocess.CalledProcessError as e:
            die(f"genhtml fallback also failed: {e}")

    # Only advertise the report if it actually produced an index page.
    if ok and os.path.isfile(os.path.join(report_dir, "index.html")):
        print_report_link(report_dir)
    else:
        warn("HTML report was not generated; no report link to show.")


# =============================================================================
# 11. CLI + MAIN
# =============================================================================


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Code coverage report generator (auto-detects GCC vs Clang from "
            "the ELF binary)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  # Clang (LLVM source-based coverage)
  gcov.py -e build/nuttx -s build -a default.profraw -o report

  # GCC (lcov + genhtml)
  gcov.py -e build/nuttx -s build -a coverage_dir -o report

  # With path remap (works for both backends)
  gcov.py -b /old/build/path=/new/local/path \\
          -e build/nuttx -s build -a default.profraw -o report

Tools are resolved from PATH (gcov/lcov/genhtml for GCC; llvm-profdata/
llvm-cov/llvm-readelf/ld.lld for Clang); make sure the toolchain is on PATH.
""",
    )
    parser.add_argument(
        "-e",
        "--elf",
        dest="elf_binary",
        required=True,
        help="Path to the ELF binary that produced the coverage data. "
        "The toolchain (gcc/clang) and target architecture are inferred "
        "from this file.",
    )
    parser.add_argument(
        "-s",
        dest="gcno_dir",
        required=False,
        default=None,
        help="Directory with .gcno files (gcc) or build output containing "
        ".o files for the covmap relocatable workaround (clang). For "
        "clang it may also point directly at a pre-built LLVM coverage object "
        "(nuttx_llvm_covmap.o or coverage_all.o, as produced by the build), "
        "or a directory containing one, to report from persisted coverage "
        "data without the build tree. Required for gcc. Optional for clang: "
        "when omitted, coverage is read straight from the ELF (-e), which "
        "only reports functions actually linked into the image -- linker "
        "GC'd dead code is correctly excluded.",
    )
    parser.add_argument(
        "-a",
        dest="data_inputs",
        default=["."],
        nargs="+",
        help="Directories or files holding .gcda (gcc) or .profraw (clang) "
        "data. Hex-dump text logs and gcov binary packs are supported.",
    )
    parser.add_argument(
        "-o",
        dest="result_dir",
        default="gcov",
        help="Output directory for intermediate data and the HTML report.",
    )
    parser.add_argument(
        "-b",
        dest="path_map",
        action="append",
        metavar="OLD=NEW",
        help="Rewrite paths recorded in coverage data: replace OLD with NEW. "
        "Maps to lcov --substitute / llvm-cov -path-equivalence. "
        "Repeatable.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Redirect stdout to <result_dir>/debug.log",
    )
    parser.add_argument(
        "--delete",
        action="store_true",
        help="Delete dump files (hex/binpack) after they have been parsed.",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()

    result_dir = os.path.abspath(args.result_dir)
    os.makedirs(result_dir, exist_ok=True)

    if args.debug:
        sys.stdout = open(os.path.join(result_dir, "debug.log"), "w+")

    elf_path = os.path.abspath(args.elf_binary)
    if not os.path.isfile(elf_path):
        die(f"ELF binary not found: {elf_path}")
    elf = inspect_elf(elf_path)
    log(f"ELF: {elf_path}")
    log(f"  toolchain: {elf.toolchain}")
    log(f"  arch:      {elf.arch_name} (e_machine={elf.e_machine})")

    # Resolve gcno_dir / build_dir. -s is required for gcc (the .gcno tree
    # cannot be inferred from the ELF). For clang it is optional: when omitted,
    # coverage is read straight from the ELF. llvm-cov then only sees functions
    # actually linked into the image; sections garbage-collected by the linker
    # (dead code never present in the firmware) are correctly excluded, so the
    # coverage denominator reflects real, executable code.
    if args.gcno_dir:
        gcno_dir = os.path.abspath(args.gcno_dir)
    elif elf.toolchain == "clang":
        log("  no -s given: reading coverage mapping straight from the ELF")
        gcno_dir = None
    else:
        die(
            "-s is required for gcc targets (the .gcno tree cannot be "
            "inferred from the ELF)."
        )

    path_map = PathMap.parse(args.path_map)

    # Normalize -a inputs (parse hex dumps / binpacks if needed)
    data_inputs = resolve_data_inputs(args.data_inputs, args.delete)

    if elf.toolchain == "clang":
        run_clang_pipeline(data_inputs, gcno_dir, result_dir, elf_path, path_map)
    else:
        run_gcc_pipeline(data_inputs, gcno_dir, result_dir, elf, path_map)


if __name__ == "__main__":
    main()
