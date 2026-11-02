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

import argparse
import os
import re
import shutil
import subprocess
import sys

# LLVM raw profile magic bytes (first 8 bytes of .profraw)
# 64-bit: \xff lprofr \x81
# 32-bit: \xff lprofR \x81

PROFRAW_MAGIC_64 = b"\xff\x6c\x70\x72\x6f\x66\x72\x81"
PROFRAW_MAGIC_32 = b"\xff\x6c\x70\x72\x6f\x66\x52\x81"


def is_profraw_file(path):
    """Check if a file is an LLVM profraw file by magic number."""
    try:
        with open(path, "rb") as f:
            magic = f.read(8)
            return magic == PROFRAW_MAGIC_64 or magic == PROFRAW_MAGIC_32
    except (IOError, OSError):
        return False


def detect_toolchain(gcda_dirs):
    """Detect whether coverage data is from gcc or clang.

    Returns 'clang' if any profraw file is found, otherwise 'gcc'.
    """
    for d in gcda_dirs:
        if os.path.isfile(d):
            if is_profraw_file(d):
                return "clang"
            continue
        for root, dirs, files in os.walk(d):
            for f in files:
                fpath = os.path.join(root, f)
                if f.endswith(".profraw") or is_profraw_file(fpath):
                    return "clang"
    return "gcc"


def parse_gcda_data(path):
    """Parse hex dump log containing 'gcov start/end' blocks.

    Works for both gcc (.gcda) and clang (.profraw) hex dumps since
    the clang coverage.c now uses the same format.
    """
    with open(path, "r") as file:
        lines = file.read().strip().splitlines()

    gcda_path = path + "_covert"
    os.makedirs(gcda_path, exist_ok=True)

    started = False
    filename = ""
    output = ""
    size = 0

    for line in lines:
        if line.startswith("gcov start"):
            started = True
            match = re.search(r"filename:(.*?)\s+size:\s*(\d+)Byte", line)
            if match:
                filename = match.group(1)
                size = int(match.group(2))
            continue

        if not started:
            continue

        try:
            if line.startswith("gcov end"):
                started = False
                if size != len(output) // 2:
                    raise ValueError(
                        f"Size mismatch for {filename}: expected {size} bytes, got {len(output) // 2} bytes"
                    )

                match = re.search(r"checksum:\s*(0x[0-9a-fA-F]+)", line)
                if match:
                    checksum = int(match.group(1), 16)
                    output = bytearray.fromhex(output)
                    expected = sum(output) % 65536
                    if checksum != expected:
                        raise ValueError(
                            f"Checksum mismatch for {filename}: expected {checksum}, got {expected}"
                        )

                    outfile = os.path.join(gcda_path, "./" + filename)
                    os.makedirs(os.path.dirname(outfile), exist_ok=True)

                    with open(outfile, "wb") as fp:
                        fp.write(output)
                        print(f"write {outfile} success")
                output = ""
            else:
                output += line.strip()
        except Exception as e:
            print(f"Error processing {path}: {e}")
            print(f"gcov start filename:{filename} size:{size}")
            print(output[:200])

    return gcda_path


def correct_content_path(file, shield: list, newpath):
    with open(file, "r", encoding="utf-8") as f:
        content = f.read()

    for i in shield:
        content = content.replace(i, "")

    new_content = content
    if newpath is not None:
        pattern = r"SF:([^\s]*?)/nuttx/include/nuttx"
        matches = re.findall(pattern, content)

        if matches:
            new_content = content.replace(matches[0], newpath)

    with open(file, "w", encoding="utf-8") as f:
        f.write(new_content)


def copy_file_endswith(endswith, source, target):
    for root, dirs, files in os.walk(source, topdown=True):
        if target in root:
            continue

        for file in files:
            if file.endswith(endswith):
                src_file = os.path.join(root, file)
                dst_file = os.path.join(target, os.path.relpath(src_file, source))
                os.makedirs(os.path.dirname(dst_file), exist_ok=True)
                shutil.copy2(src_file, dst_file)


def find_profraw_files(directory):
    """Recursively find all .profraw files in a directory."""
    profraw_files = []
    for root, dirs, files in os.walk(directory):
        for f in files:
            fpath = os.path.join(root, f)
            if f.endswith(".profraw") or is_profraw_file(fpath):
                profraw_files.append(fpath)
    return profraw_files


# ---- GCC pipeline (lcov + genhtml) ----


def run_lcov(data_dir, gcov_tool):
    output = data_dir + ".info"
    command = [
        "lcov",
        "-c",
        "-o",
        output,
        "--rc",
        "lcov_branch_coverage=1",
        "--gcov-tool",
        gcov_tool,
        "--ignore-errors",
        "gcov",
        "--directory",
        data_dir,
    ]
    print(command)
    subprocess.run(
        command,
        check=True,
        stdout=sys.stdout,
        stderr=sys.stdout,
    )

    return output


def get_lcov_major_version():
    """Get the major version number of the installed lcov/genhtml."""
    try:
        result = subprocess.run(
            ["lcov", "--version"], capture_output=True, text=True, timeout=5
        )
        match = re.search(r"LCOV version (\d+)", result.stdout)
        if match:
            return int(match.group(1))
    except (subprocess.TimeoutExpired, OSError, FileNotFoundError):
        pass
    return 1


def run_genhtml(info, report):
    # 'category' is only supported by lcov >= 2
    ignore_errors = "source"
    if get_lcov_major_version() >= 2:
        ignore_errors = "source,category"

    cmd = [
        "genhtml",
        "--branch-coverage",
        "-o",
        report,
        "--ignore-errors",
        ignore_errors,
        info,
    ]
    print(cmd)
    subprocess.run(
        cmd,
        check=True,
        stdout=sys.stdout,
        stderr=sys.stdout,
    )


def run_merge(gcda_dir1, gcda_dir2, output, merge_tool):
    command = [
        merge_tool,
        "merge",
        gcda_dir1,
        gcda_dir2,
        "-o",
        output,
    ]
    print(command)
    subprocess.run(
        command,
        check=True,
        stdout=sys.stdout,
        stderr=sys.stdout,
    )


# ---- Clang pipeline (llvm-profdata + llvm-cov) ----


def find_llvm_tools(gcov_tool):
    """Derive llvm-profdata, llvm-cov, llvm-readelf, and ld.lld paths from the -t argument.

    The user passes -t with a path like:
      /path/to/bin/clang               -> look for llvm-profdata, llvm-cov
      /path/to/bin/aarch64-none-elf-gcov -> try llvm-profdata in same dir
      llvm-cov                          -> use as-is

    Strategy:
      1. If gcov_tool dir contains llvm-profdata, use it
      2. Try replacing the tool basename with llvm-profdata / llvm-cov
      3. Fall back to PATH lookup
    """
    tool_dir = os.path.dirname(os.path.abspath(gcov_tool))

    profdata = None
    llvm_cov = None
    llvm_readelf = None
    lld = None

    # Check same directory
    for candidate in os.listdir(tool_dir) if os.path.isdir(tool_dir) else []:
        full = os.path.join(tool_dir, candidate)
        if candidate == "llvm-profdata" or candidate.endswith("-llvm-profdata"):
            profdata = full
        if candidate == "llvm-cov" or candidate.endswith("-llvm-cov"):
            llvm_cov = full
        if candidate == "llvm-readelf" or candidate.endswith("-llvm-readelf"):
            llvm_readelf = full
        if candidate == "ld.lld":
            lld = full

    # Fall back to PATH
    if profdata is None:
        profdata = shutil.which("llvm-profdata")
    if llvm_cov is None:
        llvm_cov = shutil.which("llvm-cov")
    if llvm_readelf is None:
        llvm_readelf = shutil.which("llvm-readelf")
    if lld is None:
        lld = shutil.which("ld.lld")

    return profdata, llvm_cov, llvm_readelf, lld


def run_llvm_profdata_merge(profraw_files, output_profdata, profdata_tool):
    """Merge one or more .profraw files into a single .profdata."""
    cmd = [profdata_tool, "merge", "-sparse", "-o", output_profdata] + profraw_files
    print(cmd)
    subprocess.run(cmd, check=True, stdout=sys.stdout, stderr=sys.stdout)
    return output_profdata


def run_llvm_cov_export(profdata, binary, llvm_cov_tool, output_info):
    """Export coverage data in lcov format for genhtml consumption."""
    cmd = [
        llvm_cov_tool,
        "export",
        binary,
        "-instr-profile=" + profdata,
        "-format=lcov",
    ]
    print(cmd)
    with open(output_info, "w") as f:
        subprocess.run(cmd, check=True, stdout=f, stderr=sys.stdout)
    return output_info


def run_llvm_cov_show_html(profdata, binary, llvm_cov_tool, report_dir):
    """Generate HTML report directly using llvm-cov show."""
    cmd = [
        llvm_cov_tool,
        "show",
        binary,
        "-instr-profile=" + profdata,
        "-format=html",
        "-output-dir=" + report_dir,
        "-show-line-counts-or-regions",
        "-show-branches=count",
        "-show-directory-coverage",
    ]
    print(cmd)
    subprocess.run(cmd, check=True, stdout=sys.stdout, stderr=sys.stdout)


def run_llvm_cov_report(profdata, binary, llvm_cov_tool):
    """Print coverage summary to stdout."""
    cmd = [
        llvm_cov_tool,
        "report",
        binary,
        "-instr-profile=" + profdata,
    ]
    print(cmd)
    subprocess.run(cmd, check=True, stdout=sys.stdout, stderr=sys.stdout)


def find_covmap_objects(build_dir, readelf_tool):
    """Find all .o files containing __llvm_covmap sections.

    After final linking with lld, covfun/covmap sections may have VMA=0,
    making llvm-cov unable to read the final ELF directly.  Building a
    relocatable object from the original .o files is the workaround.
    """
    objects = []
    for root, dirs, files in os.walk(build_dir):
        for f in files:
            if not f.endswith(".o"):
                continue
            fpath = os.path.join(root, f)
            try:
                result = subprocess.run(
                    [readelf_tool, "-S", fpath],
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                if "__llvm_covmap" in result.stdout:
                    objects.append(fpath)
            except (subprocess.TimeoutExpired, OSError):
                continue
    return objects


def build_relocatable_object(objects, output_path, lld_tool):
    """Link .o files into a single relocatable object with ld.lld -r."""
    cmd = [lld_tool, "-r", "-o", output_path] + objects
    print(f"Linking {len(objects)} objects into {output_path} ...")
    subprocess.run(cmd, check=True, stdout=sys.stdout, stderr=sys.stdout)
    return output_path


# ---- Main ----


def arg_parser():
    parser = argparse.ArgumentParser(
        description="Code coverage generation tool (supports GCC gcov and Clang source-based coverage).",
        add_help=False,
    )
    parser.add_argument(
        "-t",
        dest="gcov_tool",
        help="Path to gcov tool (gcc) or clang binary / llvm-cov (clang)",
    )
    parser.add_argument("-b", dest="base_dir", help="Compile base directory")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    parser.add_argument(
        "--delete",
        action="store_true",
        help="Delete gcda/profraw files after processing",
    )
    parser.add_argument(
        "-s",
        dest="gcno_dir",
        default=None,
        help="Directory containing gcno files (gcc) or build output directory (clang). "
        "For clang, defaults to the directory of the ELF binary (-e).",
    )
    parser.add_argument(
        "-a",
        dest="gcda_dir",
        default=".",
        nargs="+",
        help="Directory/file containing gcda (gcc) or profraw (clang) data",
    )
    parser.add_argument(
        "-x",
        dest="only_copy",
        action="store_true",
        help="Only copy/extract data files, don't generate report",
    )
    parser.add_argument(
        "-o",
        dest="result_dir",
        default="gcov",
        help="Directory to store coverage data and report",
    )
    parser.add_argument(
        "-e",
        "--elf",
        dest="elf_binary",
        default=None,
        help="Path to ELF binary (required for clang coverage report generation)",
    )
    parser.add_argument(
        "--toolchain",
        dest="toolchain",
        choices=["gcc", "clang", "auto"],
        default="auto",
        help="Force toolchain type (default: auto-detect from data files)",
    )

    return parser.parse_args()


def main_gcc(args, gcda_dirs, gcno_dir, result_dir):
    """GCC coverage pipeline: lcov + genhtml."""

    merge_tool = args.gcov_tool + "-tool"
    data_dir = os.path.join(result_dir, "data")
    report_dir = os.path.join(result_dir, "report")

    # lcov tool is required
    if shutil.which("lcov") is None:
        print("Error: lcov is not detected, please install lcov.")
        sys.exit(1)

    # Merge all gcda directories
    shutil.copytree(gcda_dirs[0], data_dir)
    for gcda_dir in gcda_dirs[1:]:
        run_merge(data_dir, gcda_dir, data_dir, merge_tool)

    # Copy gcno files and run lcov
    copy_file_endswith(".gcno", gcno_dir, data_dir)
    coverage_file = run_lcov(data_dir, args.gcov_tool)

    if args.only_copy:
        sys.exit(0)

    try:
        run_genhtml(coverage_file, report_dir)
        print(
            "Copy the following link and open it in the browser to view the coverage report:"
        )
        print(f"file://{os.path.join(report_dir, 'index.html')}")
    except subprocess.CalledProcessError:
        print("Failed to generate coverage report.")
        sys.exit(1)


def main_clang(args, gcda_dirs, gcno_dir, result_dir):
    """Clang coverage pipeline: llvm-profdata merge + llvm-cov.

    When the build output directory (-s) contains .o files with __llvm_covmap
    sections, automatically builds a relocatable object (ld.lld -r) and uses
    it for report generation.  This works around the issue where lld-linked
    final ELF has covfun/covmap sections with VMA=0.
    """

    data_dir = os.path.join(result_dir, "data")
    report_dir = os.path.join(result_dir, "report")
    os.makedirs(data_dir, exist_ok=True)

    # Find LLVM tools
    profdata_tool, llvm_cov_tool, llvm_readelf, lld_tool = find_llvm_tools(
        args.gcov_tool
    )

    if profdata_tool is None:
        print("Error: llvm-profdata not found. Install LLVM or specify path via -t.")
        sys.exit(1)
    if llvm_cov_tool is None:
        print("Error: llvm-cov not found. Install LLVM or specify path via -t.")
        sys.exit(1)

    print(f"Using llvm-profdata: {profdata_tool}")
    print(f"Using llvm-cov: {llvm_cov_tool}")

    # Collect all profraw files
    profraw_files = []
    for d in gcda_dirs:
        if os.path.isfile(d):
            profraw_files.append(d)
        elif os.path.isdir(d):
            profraw_files.extend(find_profraw_files(d))

    if not profraw_files:
        print("Error: No .profraw files found in the specified paths.")
        sys.exit(1)

    print(f"Found {len(profraw_files)} profraw file(s)")

    # Merge profraw -> profdata
    profdata_file = os.path.join(data_dir, "merged.profdata")
    run_llvm_profdata_merge(profraw_files, profdata_file, profdata_tool)

    if args.only_copy:
        sys.exit(0)

    # Determine ELF binary path
    elf_binary = args.elf_binary
    if elf_binary is None:
        # Try to find nuttx ELF in gcno_dir (build output directory)
        candidates = [
            os.path.join(gcno_dir, "nuttx"),
            os.path.join(gcno_dir, "nuttx.elf"),
        ]
        # Also search for vela_*.elf
        if os.path.isdir(gcno_dir):
            for f in os.listdir(gcno_dir):
                if f.endswith(".elf"):
                    candidates.append(os.path.join(gcno_dir, f))

        for c in candidates:
            if os.path.isfile(c):
                elf_binary = c
                break

    if elf_binary is None:
        print(
            "Error: ELF binary not found. Specify with -e/--elf or ensure "
            "'nuttx' exists in the -s directory."
        )
        sys.exit(1)

    elf_binary = os.path.abspath(elf_binary)
    print(f"Using ELF binary: {elf_binary}")

    # Build relocatable object from .o files if tools are available.
    # This is needed because lld-linked ELF may have covfun/covmap at VMA=0.
    cov_binary = elf_binary
    if llvm_readelf and lld_tool:
        print("\nScanning for .o files with __llvm_covmap sections ...")
        covmap_objects = find_covmap_objects(gcno_dir, llvm_readelf)
        if covmap_objects:
            reloc_obj = os.path.join(result_dir, "coverage_all.o")
            try:
                build_relocatable_object(covmap_objects, reloc_obj, lld_tool)
                cov_binary = reloc_obj
                print(f"Using relocatable object for report: {cov_binary}")
            except subprocess.CalledProcessError as e:
                print(f"Warning: failed to build relocatable object: {e}")
                print("Falling back to ELF binary.")
        else:
            print("No .o files with __llvm_covmap found, using ELF binary directly.")
    else:
        if llvm_readelf is None:
            print("Note: llvm-readelf not found, skipping relocatable object build.")
        if lld_tool is None:
            print("Note: ld.lld not found, skipping relocatable object build.")

    # Print summary report
    print("\n=== Coverage Summary ===")
    run_llvm_cov_report(profdata_file, cov_binary, llvm_cov_tool)

    # Generate HTML report: prefer llvm-cov show (supports directory grouping),
    # fall back to genhtml (lcov) if llvm-cov show fails.
    html_ok = False

    try:
        run_llvm_cov_show_html(profdata_file, cov_binary, llvm_cov_tool, report_dir)
        html_ok = True
    except subprocess.CalledProcessError as e:
        print(f"Warning: llvm-cov show HTML failed: {e}, falling back to genhtml...")

    if not html_ok and shutil.which("genhtml") is not None:
        try:
            coverage_info = os.path.join(data_dir, "coverage.info")
            run_llvm_cov_export(profdata_file, cov_binary, llvm_cov_tool, coverage_info)
            run_genhtml(coverage_info, report_dir)
            html_ok = True
        except subprocess.CalledProcessError as e:
            print(f"Warning: genhtml also failed: {e}")

    if html_ok:
        print(
            "\nCopy the following link and open it in the browser to view the coverage report:"
        )
        print(f"file://{os.path.join(report_dir, 'index.html')}")
    else:
        print("HTML report generation failed. Coverage summary was printed above.")
        sys.exit(1)


def main():
    args = arg_parser()

    root_dir = os.getcwd()
    result_dir = os.path.abspath(args.result_dir)

    os.makedirs(result_dir, exist_ok=True)

    if args.debug:
        debug_file = os.path.join(result_dir, "debug.log")
        sys.stdout = open(debug_file, "w+")

    # Parse hex dump files (works for both gcc and clang)
    gcda_dirs = []
    for i in args.gcda_dir:
        if os.path.isfile(i):
            # Check if it's a binary profraw or a text hex dump log
            if is_profraw_file(
                os.path.join(root_dir, i) if not os.path.isabs(i) else i
            ):
                # Binary profraw file, use directly
                gcda_dirs.append(os.path.abspath(i))
            else:
                # Text file: try to parse as hex dump log
                gcda_dirs.append(parse_gcda_data(os.path.join(root_dir, i)))
                if args.delete:
                    os.remove(i)
        else:
            gcda_dirs.append(os.path.abspath(i))

    # Detect toolchain
    if args.toolchain == "auto":
        toolchain = detect_toolchain(gcda_dirs)
    else:
        toolchain = args.toolchain

    print(f"Detected toolchain: {toolchain}")

    # Resolve gcno_dir: for clang, default to ELF binary's directory
    if args.gcno_dir is not None:
        gcno_dir = os.path.abspath(args.gcno_dir)
    elif toolchain == "clang" and args.elf_binary is not None:
        gcno_dir = os.path.abspath(os.path.dirname(args.elf_binary))
    else:
        gcno_dir = os.path.abspath(".")

    if toolchain == "clang":
        main_clang(args, gcda_dirs, gcno_dir, result_dir)
    else:
        main_gcc(args, gcda_dirs, gcno_dir, result_dir)


if __name__ == "__main__":
    main()
