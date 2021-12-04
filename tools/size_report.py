#!/usr/bin/env python3
#
# Copyright (c) 2021 Xiaomi Corporation
# Copyright (c) 2016, 2020 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0

# Based on a script by:
#       Chereau, Fabien <fabien.chereau@intel.com>

"""
Process an ELF file to generate size report on RAM and ROM.
"""

import argparse
import json
import os
import re
import sys
from pathlib import Path

import elftools
from anytree import NodeMixin, RenderTree, findall_by_attr
from anytree.exporter import DictExporter
from colorama import Fore, init
from elftools.dwarf.descriptions import (
    describe_DWARF_expr,
    describe_form_class,
    set_global_machine_arch,
)
from elftools.dwarf.locationlists import LocationExpr, LocationParser
from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection
from packaging import version

if version.parse(elftools.__version__) < version.parse("0.24"):
    sys.exit("pyelftools is out of date, need version 0.24 or later")


# ELF section flags
SHF_WRITE = 0x1
SHF_ALLOC = 0x2
SHF_EXEC = 0x4
SHF_WRITE_ALLOC = SHF_WRITE | SHF_ALLOC
SHF_ALLOC_EXEC = SHF_ALLOC | SHF_EXEC

DT_LOCATION = re.compile(r"\(DW_OP_addr: ([0-9a-f]+)\)")

SRC_FILE_EXT = (".h", ".c", ".hpp", ".cpp", ".hxx", ".cxx", ".c++")


def get_symbol_addr(sym):
    """Get the address of a symbol"""
    return sym["st_value"]


def get_symbol_size(sym):
    """Get the size of a symbol"""
    return sym["st_size"]


def is_symbol_in_ranges(sym, ranges):
    """
    Given a list of start/end addresses, test if the symbol
    lies within any of these address ranges.
    """
    for bound in ranges:
        if bound["start"] <= sym["st_value"] <= bound["end"]:
            return True

    return False


def get_die_mapped_address(die, parser, dwarfinfo):
    """Get the bounding addresses from a DIE variable or subprogram"""
    low = None
    high = None

    if die.tag == "DW_TAG_variable":
        if "DW_AT_location" in die.attributes:
            loc_attr = die.attributes["DW_AT_location"]
            if parser.attribute_has_location(loc_attr, die.cu["version"]):
                loc = parser.parse_from_attribute(loc_attr, die.cu["version"])
                if isinstance(loc, LocationExpr):
                    addr = describe_DWARF_expr(loc.loc_expr, dwarfinfo.structs)

                    matcher = DT_LOCATION.match(addr)
                    if matcher:
                        low = int(matcher.group(1), 16)
                        high = low + 1

    if die.tag == "DW_TAG_subprogram":
        if "DW_AT_low_pc" in die.attributes:
            low = die.attributes["DW_AT_low_pc"].value

            high_pc = die.attributes["DW_AT_high_pc"]
            high_pc_class = describe_form_class(high_pc.form)
            if high_pc_class == "address":
                high = high_pc.value
            elif high_pc_class == "constant":
                high = low + high_pc.value

    return low, high


def match_symbol_address(symlist, die, parser, dwarfinfo):
    """
    Find the symbol from a symbol list
    where it matches the address in DIE variable,
    or within the range of a DIE subprogram.
    """
    low, high = get_die_mapped_address(die, parser, dwarfinfo)

    if low is None:
        return None

    for sym in symlist:
        if low <= sym["symbol"]["st_value"] < high:
            return sym

    return None


def get_symbols(elf, addr_ranges):
    """
    Fetch the symbols from the symbol table and put them
    into ROM, RAM buckets.
    """
    rom_syms = dict()
    ram_syms = dict()
    unassigned_syms = dict()

    rom_addr_ranges = addr_ranges["rom"]
    ram_addr_ranges = addr_ranges["ram"]

    for section in elf.iter_sections():
        if isinstance(section, SymbolTableSection):
            for sym in section.iter_symbols():
                # Ignore symbols with size == 0
                if get_symbol_size(sym) == 0:
                    continue

                found_sec = False
                entry = {"name": sym.name, "symbol": sym, "mapped_files": set()}

                # If symbol is in ROM area?
                if is_symbol_in_ranges(sym, rom_addr_ranges):
                    if sym.name not in rom_syms:
                        rom_syms[sym.name] = list()
                    rom_syms[sym.name].append(entry)
                    found_sec = True

                # If symbol is in RAM area?
                if is_symbol_in_ranges(sym, ram_addr_ranges):
                    if sym.name not in ram_syms:
                        ram_syms[sym.name] = list()
                    ram_syms[sym.name].append(entry)
                    found_sec = True

                if not found_sec:
                    unassigned_syms["sym_name"] = entry

    ret = {"rom": rom_syms, "ram": ram_syms, "unassigned": unassigned_syms}
    return ret


def get_section_ranges(elf):
    """
    Parse ELF header to find out the address ranges of ROM or RAM sections
    and their total sizes.
    """
    rom_addr_ranges = list()
    ram_addr_ranges = list()
    rom_size = 0
    ram_size = 0

    for section in elf.iter_sections():
        size = section["sh_size"]
        sec_start = section["sh_addr"]
        sec_end = sec_start + size - 1
        bound = {"start": sec_start, "end": sec_end}

        if section["sh_type"] == "SHT_NOBITS":
            # BSS and noinit sections
            ram_addr_ranges.append(bound)
            ram_size += size
        elif section["sh_type"] == "SHT_PROGBITS":
            # Sections to be in flash or memory
            flags = section["sh_flags"]
            if (flags & SHF_ALLOC_EXEC) == SHF_ALLOC_EXEC:
                # Text section
                rom_addr_ranges.append(bound)
                rom_size += size
            elif (flags & SHF_WRITE_ALLOC) == SHF_WRITE_ALLOC:
                # Data occupies both ROM and RAM
                # since at boot, content is copied from ROM to RAM
                rom_addr_ranges.append(bound)
                rom_size += size

                ram_addr_ranges.append(bound)
                ram_size += size
            elif (flags & SHF_ALLOC) == SHF_ALLOC:
                # Read only data
                rom_addr_ranges.append(bound)
                rom_size += size

    ret = {
        "rom": rom_addr_ranges,
        "rom_total_size": rom_size,
        "ram": ram_addr_ranges,
        "ram_total_size": ram_size,
    }
    return ret


def get_die_filename(die, lineprog):
    """Get the source code filename associated with a DIE"""
    file_index = die.attributes["DW_AT_decl_file"].value
    file_entry = lineprog["file_entry"][file_index - 1]

    dir_index = file_entry["dir_index"]
    if dir_index == 0:
        filename = file_entry.name
    else:
        directory = lineprog.header["include_directory"][dir_index - 1]
        filename = os.path.join(directory, file_entry.name)

    path = Path(filename.decode())

    # Prepend output path to relative path
    if not path.is_absolute():
        output = Path(args.output)
        path = output.joinpath(path)

    # Change path to relative to Nuttx base
    try:
        path = path.resolve()
    except OSError as e:
        # built-ins can't be resolved, so it's not an issue
        if "<built-in>" not in str(path):
            raise e

    return path


def do_simple_name_matching(elf, symbol_dict, processed):
    """
    Sequentially process DIEs in compiler units with direct file mappings
    within the DIEs themselves, and do simply matching between DIE names
    and symbol names.
    """
    mapped_symbols = processed["mapped_symbols"]
    mapped_addresses = processed["mapped_addr"]
    unmapped_symbols = processed["unmapped_symbols"]
    newly_mapped_syms = set()

    dwarfinfo = elf.get_dwarf_info()
    location_lists = dwarfinfo.location_lists()
    location_parser = LocationParser(location_lists)

    unmapped_dies = set()

    # Loop through all compile units
    for compile_unit in dwarfinfo.iter_CUs():
        lineprog = dwarfinfo.line_program_for_CU(compile_unit)
        if lineprog is None:
            continue

        # Loop through each DIE and find variables and
        # subprograms (i.e. functions)
        for die in compile_unit.iter_DIEs():
            sym_name = None

            # Process variables
            if die.tag == "DW_TAG_variable":
                # DW_AT_declaration

                # having "DW_AT_location" means this maps
                # to an actual address (e.g. not an extern)
                if "DW_AT_location" in die.attributes:
                    sym_name = die.get_full_path()

            # Process subprograms (i.e. functions) if they are valid
            if die.tag == "DW_TAG_subprogram":
                # Refer to another DIE for name
                if ("DW_AT_abstract_origin" in die.attributes) or (
                    "DW_AT_specification" in die.attributes
                ):
                    unmapped_dies.add(die)

                # having "DW_AT_low_pc" means it maps to
                # an actual address
                elif "DW_AT_low_pc" in die.attributes:
                    # DW_AT_low_pc == 0 is a weak function
                    # which has been overriden
                    if die.attributes["DW_AT_low_pc"].value != 0:
                        sym_name = die.get_full_path()

                # For mangled function names, the linkage name
                # is what appears in the symbol list
                if "DW_AT_linkage_name" in die.attributes:
                    linkage = die.attributes["DW_AT_linkage_name"]
                    sym_name = linkage.value.decode()

            if sym_name is not None:
                # Skip DIE with no reference back to a file
                if "DW_AT_decl_file" not in die.attributes:
                    continue

                is_die_mapped = False
                if sym_name in symbol_dict:
                    mapped_symbols.add(sym_name)
                    symlist = symbol_dict[sym_name]
                    symbol = match_symbol_address(
                        symlist, die, location_parser, dwarfinfo
                    )

                    if symbol is not None:
                        symaddr = symbol["symbol"]["st_value"]
                        if symaddr not in mapped_addresses:
                            is_die_mapped = True
                            path = get_die_filename(die, lineprog)
                            symbol["mapped_files"].add(path)
                            mapped_addresses.add(symaddr)
                            newly_mapped_syms.add(sym_name)

                if not is_die_mapped:
                    unmapped_dies.add(die)

    mapped_symbols = mapped_symbols.union(newly_mapped_syms)
    unmapped_symbols = unmapped_symbols.difference(newly_mapped_syms)

    processed["mapped_symbols"] = mapped_symbols
    processed["mapped_addr"] = mapped_addresses
    processed["unmapped_symbols"] = unmapped_symbols
    processed["unmapped_dies"] = unmapped_dies


def mark_address_aliases(symbol_dict, processed):
    """
    Mark symbol aliases as already mapped to prevent
    double counting.

    There are functions and variables which are aliases to
    other functions/variables. So this marks them as mapped
    so they will not get counted again when a tree is being
    built for display.
    """
    mapped_symbols = processed["mapped_symbols"]
    mapped_addresses = processed["mapped_addr"]
    unmapped_symbols = processed["unmapped_symbols"]
    already_mapped_syms = set()

    for ums in unmapped_symbols:
        for one_sym in symbol_dict[ums]:
            symbol = one_sym["symbol"]
            if symbol["st_value"] in mapped_addresses:
                already_mapped_syms.add(ums)

    mapped_symbols = mapped_symbols.union(already_mapped_syms)
    unmapped_symbols = unmapped_symbols.difference(already_mapped_syms)

    processed["mapped_symbols"] = mapped_symbols
    processed["mapped_addr"] = mapped_addresses
    processed["unmapped_symbols"] = unmapped_symbols


def do_address_range_matching(elf, symbol_dict, processed):
    """
    Match symbols indirectly using address ranges.

    This uses the address ranges of DIEs and map them to symbols
    residing within those ranges, and works on DIEs that have not
    been mapped in previous steps. This works on symbol names
    that do not match the names in DIEs, e.g. "<func>" in DIE,
    but "<func>.constprop.*" in symbol name list. This also
    helps with mapping the mangled function names in C++,
    since the names in DIE are actual function names in source
    code and not mangled version of them.
    """
    if "unmapped_dies" not in processed:
        return

    mapped_symbols = processed["mapped_symbols"]
    mapped_addresses = processed["mapped_addr"]
    unmapped_symbols = processed["unmapped_symbols"]
    newly_mapped_syms = set()

    dwarfinfo = elf.get_dwarf_info()
    location_lists = dwarfinfo.location_lists()
    location_parser = LocationParser(location_lists)

    unmapped_dies = processed["unmapped_dies"]

    # Group DIEs by compile units
    cu_list = dict()

    for die in unmapped_dies:
        cu = die.cu
        if cu not in cu_list:
            cu_list[cu] = {"dies": set()}
        cu_list[cu]["dies"].add(die)

    # Loop through all compile units
    for cu in cu_list:
        lineprog = dwarfinfo.line_program_for_CU(cu)

        # Map offsets from DIEs
        offset_map = dict()
        for die in cu.iter_DIEs():
            offset_map[die.offset] = die

        for die in cu_list[cu]["dies"]:
            if not die.tag == "DW_TAG_subprogram":
                continue

            path = None

            # Has direct reference to file, so use it
            if "DW_AT_decl_file" in die.attributes:
                path = get_die_filename(die, lineprog)

            # Loop through indirect reference until a direct
            # reference to file is found
            if ("DW_AT_abstract_origin" in die.attributes) or (
                "DW_AT_specification" in die.attributes
            ):
                die_ptr = die
                while path is None:
                    if not (die_ptr.tag == "DW_TAG_subprogram") or not (
                        ("DW_AT_abstract_origin" in die_ptr.attributes)
                        or ("DW_AT_specification" in die_ptr.attributes)
                    ):
                        break

                    if "DW_AT_abstract_origin" in die_ptr.attributes:
                        ofname = "DW_AT_abstract_origin"
                    elif "DW_AT_specification" in die_ptr.attributes:
                        ofname = "DW_AT_specification"

                    offset = die_ptr.attributes[ofname].value
                    offset += die_ptr.cu.cu_offset

                    # There is nothing to reference so no need to continue
                    if offset not in offset_map:
                        break

                    die_ptr = offset_map[offset]
                    if "DW_AT_decl_file" in die_ptr.attributes:
                        path = get_die_filename(die_ptr, lineprog)

            # Nothing to map
            if path is not None:
                low, high = get_die_mapped_address(die, location_parser, dwarfinfo)
                if low is None:
                    continue

                for ums in unmapped_symbols:
                    for one_sym in symbol_dict[ums]:
                        symbol = one_sym["symbol"]
                        symaddr = symbol["st_value"]

                        if symaddr not in mapped_addresses:
                            if low <= symaddr < high:
                                one_sym["mapped_files"].add(path)
                                mapped_addresses.add(symaddr)
                                newly_mapped_syms.add(ums)

    mapped_symbols = mapped_symbols.union(newly_mapped_syms)
    unmapped_symbols = unmapped_symbols.difference(newly_mapped_syms)

    processed["mapped_symbols"] = mapped_symbols
    processed["mapped_addr"] = mapped_addresses
    processed["unmapped_symbols"] = unmapped_symbols


def set_root_path_for_unmapped_symbols(symbol_dict, addr_range, processed):
    """
    Set root path for unmapped symbols.

    Any unmapped symbols are added under the root node if those
    symbols reside within the desired memory address ranges
    (e.g. ROM or RAM).
    """
    mapped_symbols = processed["mapped_symbols"]
    mapped_addresses = processed["mapped_addr"]
    unmapped_symbols = processed["unmapped_symbols"]
    newly_mapped_syms = set()

    for ums in unmapped_symbols:
        for one_sym in symbol_dict[ums]:
            symbol = one_sym["symbol"]
            symaddr = symbol["st_value"]

            if is_symbol_in_ranges(symbol, addr_range):
                if symaddr not in mapped_addresses:
                    path = Path(":")
                    one_sym["mapped_files"].add(path)
                    mapped_addresses.add(symaddr)
                    newly_mapped_syms.add(ums)

    mapped_symbols = mapped_symbols.union(newly_mapped_syms)
    unmapped_symbols = unmapped_symbols.difference(newly_mapped_syms)

    processed["mapped_symbols"] = mapped_symbols
    processed["mapped_addr"] = mapped_addresses
    processed["unmapped_symbols"] = unmapped_symbols


def find_common_path_prefix(symbol_dict):
    """
    Find the common path prefix of all mapped files.
    Must be called before set_root_path_for_unmapped_symbols().
    """
    paths = list()

    for _, sym in symbol_dict.items():
        for symbol in sym:
            for file in symbol["mapped_files"]:
                paths.append(file)

    return os.path.commonpath(paths)


class TreeNode(NodeMixin):
    """
    A symbol node.
    """

    def __init__(self, name, identifier, size=0, parent=None, children=None):
        super().__init__()
        self.name = name
        self.size = size
        self.parent = parent
        self.identifier = identifier
        if children:
            self.children = children

    def __repr__(self):
        return self.name


def sum_node_children_size(node):
    """
    Calculate the sum of symbol size of all direct children.
    """
    size = 0

    for child in node.children:
        size += child.size

    return size


def generate_any_tree(symbol_dict, total_size, path_prefix):
    """
    Generate a symbol tree for output.
    """
    root = TreeNode("Root", "root")
    node_no_paths = TreeNode("(no paths)", ":", parent=root)

    if Path(path_prefix) == Path(args.nuttxbase):
        # All source files are under nuttx_base so there is
        # no need for another level.
        node_nuttx_base = root
        node_output_dir = root
        node_workspace = root
        node_others = root
    else:
        node_nuttx_base = TreeNode("nuttx_base", args.nuttxbase)
        node_output_dir = TreeNode("OUTPUT_DIR", args.output)
        node_others = TreeNode("/", "/")

        if args.workspace:
            node_workspace = TreeNode("WORKSPACE", args.workspace)
        else:
            node_workspace = node_others

    # A set of helper function for building a simple tree with a path-like
    # hierarchy.
    def _insert_one_elem(root, path, size):
        cur = None
        node = None
        parent = root
        for part in path.parts:
            if cur is None:
                cur = part
            else:
                cur = str(Path(cur, part))

            results = findall_by_attr(root, cur, name="identifier")
            if results:
                item = results[0]
                item.size += size
                parent = item
            else:
                if node:
                    parent = node
                node = TreeNode(
                    name=str(part), identifier=cur, size=size, parent=parent
                )

    # Mapping paths to tree nodes
    path_node_map = [
        [Path(args.nuttxbase), node_nuttx_base],
        [Path(args.output), node_output_dir],
    ]

    if args.workspace:
        path_node_map.append([Path(args.workspace), node_workspace])

    for name, sym in symbol_dict.items():
        for symbol in sym:
            size = get_symbol_size(symbol["symbol"])
            for file in symbol["mapped_files"]:
                path = Path(file, name)
                if path.is_absolute():
                    has_node = False

                    for one_path in path_node_map:
                        if one_path[0] in path.parents:
                            path = path.relative_to(one_path[0])
                            dest_node = one_path[1]
                            has_node = True
                            break

                    if not has_node:
                        dest_node = node_others
                else:
                    dest_node = node_no_paths

                _insert_one_elem(dest_node, path, size)

    if node_nuttx_base is not root:
        # nuttx_base and OUTPUT_DIR nodes don't have sum of symbol size
        # so calculate them here.
        node_nuttx_base.size = sum_node_children_size(node_nuttx_base)
        node_output_dir.size = sum_node_children_size(node_output_dir)

        # Find out which nodes need to be in the tree.
        # "(no path)", nuttx_base nodes are essential.
        children = [node_no_paths, node_nuttx_base]
        if node_output_dir.height != 0:
            # OUTPUT_DIR may be under nuttx_base.
            children.append(node_output_dir)
        if node_others.height != 0:
            # Only include "others" node if there is something.
            children.append(node_others)

        if args.workspace:
            node_workspace.size = sum_node_children_size(node_workspace)
            if node_workspace.height != 0:
                children.append(node_workspace)

        root.children = children

    root.size = total_size

    # Need to account for code and data where there are not emitted
    # symbols associated with them.
    node_hidden_syms = TreeNode("(hidden)", "(hidden)", parent=root)
    node_hidden_syms.size = root.size - sum_node_children_size(root)

    return root


def node_sort(items):
    """
    Node sorting used with RenderTree.
    """
    return sorted(items, key=lambda item: item.name)


def print_any_tree(root, total_size, depth):
    """
    Print the symbol tree.
    """
    print("{:101s} {:7s} {:8s}".format(Fore.YELLOW + "Path", "Size", "%" + Fore.RESET))
    print("=" * 110)
    for row in RenderTree(root, childiter=node_sort, maxlevel=depth):
        f = len(row.pre) + len(row.node.name)
        s = str(row.node.size).rjust(100 - f)
        percent = 100 * float(row.node.size) / float(total_size)

        cc = cr = ""
        if not row.node.children:
            if row.node.name != "(hidden)":
                cc = Fore.CYAN
                cr = Fore.RESET
        elif row.node.name.endswith(SRC_FILE_EXT):
            cc = Fore.GREEN
            cr = Fore.RESET

        print(
            f"{row.pre}{cc}{row.node.name} {s} {cr}{Fore.BLUE}{percent:6.2f}%{Fore.RESET}"
        )
    print("=" * 110)
    print(f"{total_size:>101}")


def parse_args():
    """
    Parse command line arguments.
    """
    global args

    parser = argparse.ArgumentParser()

    parser.add_argument("-k", "--kernel", required=True, help="Nuttx ELF binary")
    parser.add_argument("-z", "--nuttxbase", required=True, help="Nuttx base path")
    parser.add_argument(
        "-q",
        "--quiet",
        action="store_true",
        help="Do not output anything on the screen.",
    )
    parser.add_argument("-o", "--output", required=True, help="Output path")
    parser.add_argument(
        "-w",
        "--workspace",
        default=None,
        help="Workspace path (Usually the same as TOPDIR)",
    )
    parser.add_argument("target", choices=["rom", "ram", "all"])
    parser.add_argument(
        "-d",
        "--depth",
        dest="depth",
        type=int,
        default=None,
        help="How deep should we go into the tree",
        metavar="DEPTH",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Print extra debugging information"
    )
    parser.add_argument("--json", help="store results in a JSON file.")
    args = parser.parse_args()


def main():
    """
    Main program.
    """
    parse_args()

    # Init colorama
    init()

    assert os.path.exists(args.kernel), "{0} does not exist.".format(args.kernel)
    if args.target == "ram":
        targets = ["ram"]
    elif args.target == "rom":
        targets = ["rom"]
    elif args.target == "all":
        targets = ["rom", "ram"]

    for t in targets:

        elf = ELFFile(open(args.kernel, "rb"))

        assert elf.has_dwarf_info(), "ELF file has no DWARF information"

        set_global_machine_arch(elf.get_machine_arch())

        addr_ranges = get_section_ranges(elf)

        symbols = get_symbols(elf, addr_ranges)

        for sym in symbols["unassigned"].values():
            print("WARN: Symbol '{0}' is not in RAM or ROM".format(sym["name"]))

        symbol_dict = None

        if args.json:
            jsonout = args.json
        else:
            jsonout = os.path.join(args.output, f"{t}.json")

        symbol_dict = symbols[t]
        symsize = addr_ranges[f"{t}_total_size"]
        ranges = addr_ranges[t]

        if symbol_dict is not None:
            processed = {
                "mapped_symbols": set(),
                "mapped_addr": set(),
                "unmapped_symbols": set(symbol_dict.keys()),
            }

            do_simple_name_matching(elf, symbol_dict, processed)
            mark_address_aliases(symbol_dict, processed)
            do_address_range_matching(elf, symbol_dict, processed)
            mark_address_aliases(symbol_dict, processed)
            common_path_prefix = find_common_path_prefix(symbol_dict)
            set_root_path_for_unmapped_symbols(symbol_dict, ranges, processed)

            if args.verbose:
                for sym in processed["unmapped_symbols"]:
                    print("INFO: Unmapped symbol: {0}".format(sym))

            root = generate_any_tree(symbol_dict, symsize, common_path_prefix)
            if not args.quiet:
                print_any_tree(root, symsize, args.depth)

            exporter = DictExporter()
            data = dict()
            data["symbols"] = exporter.export(root)
            data["total_size"] = symsize
            with open(jsonout, "w") as fp:
                json.dump(data, fp, indent=4)


if __name__ == "__main__":
    main()
