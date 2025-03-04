#!/usr/bin/env python3

"""
tools/mkfsdata.py

SPDX-License-Identifier: ???

Copyright (c) 2024 Alexey Matveev

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import argparse
import logging
from pathlib import Path
from typing import Iterator, List, Optional, TextIO, Tuple


def get_files(dirname: Path) -> Iterator[Tuple[Path, str]]:
    """
    Generator return files in the directory dirname
    """
    for p in dirname.glob("**/*"):
        if p.is_file():
            relative_full_path = "/" + str(p.relative_to(dirname))
            yield p, relative_full_path


def conver_c_name(prefix: str, r_full_path: str) -> str:
    """
    Convert file name to C variable name
    """
    return "{}{}".format(prefix, r_full_path.replace(".", "_").replace("/", "_"))


def write_f_data(
    fd: TextIO,
    file: Path,
    r_full_path: str,
    is_add_progmem: bool = False,
):
    """
    Write file as hex dump

    Args:
        file - source file
        is_add_progmem - add PROGMEM, for AVR
    """
    rs_name = conver_c_name("data", r_full_path)
    st_progmem = "PROGMEM " if is_add_progmem else ""
    fd.write(
        f"static const unsigned char {rs_name}[] {st_progmem}=\n"
        "{\n"
        f"  /* {r_full_path} */\n\n"
    )

    # write file name
    result = "  "
    for ch in r_full_path:
        result += "0x{0:02x}, ".format(ord(ch))
    result += "0x00,"
    fd.write(result)

    # write file content
    with open(file, "rb") as f_data:
        for j, ch in enumerate(iter(lambda: f_data.read(1), b"")):
            if j % 10 == 0:
                fd.write("\n")
                fd.write("  ")
            fd.write("0x{0:02x}, ".format(ch[0]))
        fd.write("\n")

    # End
    fd.write("};\n")


def write_s_data(
    fd: TextIO,
    r_full_path: str,
    prev_file: Optional[str],
    is_replace_to_root: bool = False,
):
    """
    Write struct httpd_fsdata_file
    """
    if is_replace_to_root:
        st_name = "g_httpdfs_root"
    else:
        st_name = conver_c_name("file", r_full_path)

    if prev_file is None:
        st_prev_name = "NULL"
    else:
        st_prev_name = conver_c_name("file", prev_file)
    rs_name = conver_c_name("data", r_full_path)
    n_len = len(r_full_path) + 1
    fd.write(
        f"const struct httpd_fsdata_file {st_name}[] =\n"
        "    {{"
        f"{st_prev_name}, {rs_name}, {rs_name} + {n_len}, "
        f"sizeof({rs_name}) - {n_len}"
        "}};\n\n"
    )


def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    logger = logging.getLogger()

    args = parser().parse_args()

    # Get files
    files: List[Tuple[Path, str]] = []
    for file, r_full_path in get_files(args.dirname):
        logger.info(f"Adding file {r_full_path}")
        files.append((file, r_full_path))

    with args.out.open("w") as fd:
        # write header
        fd.write(
            '#include "netutils/httpd.h"\n'
            "\n"
            "#ifndef NULL\n"
            "  #define NULL 0\n"
            "#endif\n\n"
        )

        # write content
        for file, r_full_path in files:
            write_f_data(fd, file, r_full_path, args.add_progmem)
            fd.write("\n")  # Add empty line beetween files

        # write list of files
        prev_file = None
        for j, (file, r_full_path) in enumerate(files):
            write_s_data(
                fd=fd,
                r_full_path=r_full_path,
                prev_file=prev_file,
                is_replace_to_root=j == len(files) - 1,
            )
            prev_file = r_full_path

        # end
        fd.write(f"const int g_httpd_numfiles = {len(files)};\n")


def parser() -> argparse.ArgumentParser:
    """Create parser"""
    parser = argparse.ArgumentParser(description="Create C file with data for httpd")
    parser.add_argument(
        "-d",
        "--dirname",
        default="httpd-fs",
        type=Path,
        help="Directory with files to add",
    )
    parser.add_argument(
        "-o",
        "--out",
        default="httpd_fsdata.c",
        type=Path,
        help="Output C file",
    )
    parser.add_argument(
        "--add-progmem",
        action="store_true",
        help="Add PROGMEM to data",
    )
    return parser


if __name__ == "__main__":
    main()
