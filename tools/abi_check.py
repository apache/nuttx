#!/usr/bin/env python3
# tools/abi_check.py
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

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
from itertools import zip_longest

from elftools.elf.elffile import ELFFile

program_description = """
This tool is used to check the binary compatibility of static libraries and has the following features:
    1. The input consists of multiple static libraries and an ELF file. The tool searches
       for external APIs used by the static libraries, then locates these API function signatures
       in the ELF file, and outputs the results as a JSON file.
    2. Using the first feature, with the static libraries unchanged,
       the tool can take a new ELF file and an old ELF file as input, output two JSON files,
       and compare the function signatures of functions with the same name in the two JSON files.
       The comparison includes return values, parameters, and if they are structures,
       it also compares the structure size, member offsets, member types, etc.
    3.When the input is a single ELF file, the tool can check if structures with the same name have different members.
"""

struct_re = re.compile(r"struct\s+(\w+)\s+{")
member_re = re.compile(r"{(.*?)};", re.DOTALL)
file_re = re.compile(r"/\*\s*<[\da-f]+>\s+([\w\/\.\-_]+\.h:\d+)\s*\*/", re.DOTALL)


def member_mismatch(member1, member2):
    member1 = member1.replace("_Bool", "bool")
    member1 = member1.replace(" ", "")
    member2 = member2.replace("_Bool", "bool")
    member2 = member2.replace(" ", "")

    return member1 != member2


def print_struct(name, member, fileinfo):
    print(f"struct {name} {{")
    print(member)
    print(f"}}; at {fileinfo}")


def struct_check(elf):
    p = subprocess.Popen(
        ["pahole", "-M", "--sort", "-I", elf],
        env=os.environ.copy(),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    out, err = p.communicate()
    if p.returncode != 0:
        print(f"Error: {err}")
        sys.exit(1)

    struct_blocks = re.split(r"/\* Used at:", out)
    structs = {}
    for block in struct_blocks:
        name_match = struct_re.search(block)
        if not name_match:
            continue

        struct_name = name_match.group(1)
        if not struct_name:
            continue

        member_match = member_re.search(block)
        if not member_match:
            continue

        file_match = file_re.search(block)
        if not file_match:
            continue

        members = member_match.group(1)
        fileinfo = file_match.group(1)
        if struct_name in structs and member_mismatch(structs[struct_name][0], members):
            print_struct(struct_name, structs[struct_name][0], structs[struct_name][1])
            print("------")
            print_struct(struct_name, members, fileinfo)
            print("")

        else:
            structs[struct_name] = (members, fileinfo)


def objfile_iter(path):
    """Get .obj file from .a file"""

    temp_dir = tempfile.mkdtemp()
    for p in path:
        subprocess.run(["ar", "x", p], cwd=temp_dir)

    for filename in os.listdir(temp_dir):
        yield os.path.join(temp_dir, filename)


def symbol_collect(path):
    """get undefined and defined symbols from an object file"""

    obj = ELFFile.load_from_path(path)
    symtab = obj.get_section_by_name(".symtab")
    if symtab is None:
        return (None, None)

    undef = []
    defined = []

    for symbol in symtab.iter_symbols():
        if (
            symbol["st_info"]["type"] == "STT_NOTYPE"
            and symbol["st_info"]["bind"] == "STB_GLOBAL"
        ):
            undef.append(symbol.name)
        else:
            defined.append(symbol.name)

    return undef, defined


def resolve_type(dwarfinfo, die):
    """Get type name and size from DIE"""

    if die is None:
        return ("void", 0)

    type_size = attr.value if (attr := die.attributes.get("DW_AT_byte_size")) else 0

    def get_name(die, prefix="", suffix="", default="void", size=0):
        name = die.attributes.get("DW_AT_name")
        return (
            f"{prefix}{name.value.decode('utf-8')}{suffix}" if name else default,
            size,
        )

    if die.tag == "DW_TAG_base_type":
        return get_name(die, size=type_size)

    if die.tag == "DW_TAG_pointer_type":
        base_type_die = resolve_referenced_die(dwarfinfo, die)
        if base_type_die:
            name, size = resolve_type(dwarfinfo, base_type_die)
            return (f"{name}*", type_size)
        else:
            return ("void*", type_size)

    if die.tag == "DW_TAG_typedef":
        base_type_die = resolve_referenced_die(dwarfinfo, die)
        _, size = resolve_type(dwarfinfo, base_type_die)

        return get_name(die, default="unknown", size=size)

    if die.tag == "DW_TAG_structure_type":
        return get_name(
            die, prefix="struct ", default="anonymous struct", size=type_size
        )

    if die.tag == "DW_TAG_union_type":
        return get_name(die, prefix="union ", default="anonymous union", size=type_size)

    if die.tag == "DW_TAG_enumeration_type":
        return get_name(die, prefix="enum ", default="anonymous enum", size=type_size)

    if die.tag == "DW_TAG_subroutine_type":
        base_type_die = resolve_referenced_die(dwarfinfo, die)
        return resolve_type(dwarfinfo, base_type_die)

    if die.tag == "DW_TAG_array_type":
        base_type_die = resolve_referenced_die(dwarfinfo, die)
        name, _ = resolve_type(dwarfinfo, base_type_die)
        size = 0
        for child in die.iter_children():
            if (
                child.tag == "DW_TAG_subrange_type"
                and "DW_AT_upper_bound" in child.attributes
            ):
                size = child.attributes["DW_AT_upper_bound"].value + 1
                break

        return (f"{name}[{size}]", size)

    if die.tag == "DW_TAG_const_type":
        base_type_die = resolve_referenced_die(dwarfinfo, die)
        name, size = resolve_type(dwarfinfo, base_type_die)

        return (f"const {name}", size)

    return ("void", type_size)


def resolve_referenced_die(dwarfinfo, die):
    """Get the referenced DIE"""

    if "DW_AT_type" in die.attributes:
        type_offset = die.attributes["DW_AT_type"].value
        cu_offset = die.cu.cu_offset
        absolute_offset = type_offset + cu_offset
        return dwarfinfo.get_DIE_from_refaddr(absolute_offset)
    elif "DW_AT_specification" in die.attributes:
        type_offset = die.attributes["DW_AT_specification"].value
        cu_offset = die.cu.cu_offset
        absolute_offset = type_offset + cu_offset
        return dwarfinfo.get_DIE_from_refaddr(absolute_offset())

    return None


def resolve_field_info(dwarfinfo, die):
    """Get field information from DIE"""

    if die.tag != "DW_TAG_member":
        return None

    field_name = die.attributes.get("DW_AT_name", None)
    field_name = field_name.value.decode("utf-8")
    field_type_die = resolve_referenced_die(dwarfinfo, die)
    field_type, field_size = resolve_type(dwarfinfo, field_type_die)
    field_offset = die.attributes.get("DW_AT_data_member_location", None)
    field_offset = field_offset.value if field_offset else "unknown"

    return {
        "name": field_name,
        "type": field_type,
        "size": field_size,
        "offset": field_offset,
    }


def find_combination_die(dwarfinfo, die):
    if die.tag == "DW_TAG_structure_type" or die.tag == "DW_TAG_structure_type":
        return die

    if "DW_AT_type" in die.attributes:
        base_type_die = resolve_referenced_die(dwarfinfo, die)
        return find_combination_die(dwarfinfo, base_type_die)

    return None


def resolve_combination_type(dwarfinfo, die):
    combination_die = find_combination_die(dwarfinfo, die)
    if not combination_die:
        return []

    field_info = []
    for child in combination_die.iter_children():
        if child.tag == "DW_TAG_member":
            field_info.append(resolve_field_info(dwarfinfo, child))

    return field_info


def get_die_file_path(die, dwarfinfo):
    cu = die.cu
    name = cu.get_top_DIE().attributes.get("DW_AT_name")
    return name.value.decode("utf-8") if name else "unknown file"


def die_is_prototyped(die):
    return (
        die.tag == "DW_TAG_subprogram"
        and "DW_AT_external" in die.attributes
        and "DW_AT_name" in die.attributes
        and "DW_AT_prototyped" in die.attributes
    )


def die_is_real_function(die):
    return (
        die.tag == "DW_TAG_subprogram"
        and "DW_AT_external" in die.attributes
        and "DW_AT_name" in die.attributes
        and ("DW_AT_low_pc" in die.attributes or "DW_AT_inline" in die.attributes)
    )


def find_function_signature(elf_path, function_list, die_check):

    signature = []
    function_list = function_list.copy()
    with open(elf_path, "rb") as f:
        elffile = ELFFile(f)
        if not elffile.has_dwarf_info():
            print("No DWARF debug info found in the ELF file.")
            return (signature, function_list)

        dwarf_info = elffile.get_dwarf_info()

        for CU in dwarf_info.iter_CUs():
            for die in (die for die in CU.iter_DIEs() if die_check(die)):
                name = die.attributes["DW_AT_name"].value.decode("utf-8")
                if "DW_AT_linkage_name" in die.attributes:
                    if name not in function_list:
                        name = die.attributes["DW_AT_linkage_name"].value.decode(
                            "utf-8"
                        )

                if name not in function_list:
                    continue

                prototype = {}

                return_type = "void"
                return_type_size = 0
                retrun_type_field = []

                if "DW_AT_type" in die.attributes:
                    return_type_ref = resolve_referenced_die(dwarf_info, die)
                    return_type, return_type_size = resolve_type(
                        dwarf_info, return_type_ref
                    )
                    retrun_type_field = resolve_combination_type(
                        dwarf_info, return_type_ref
                    )

                prototype["return"] = {
                    "type": return_type,
                    "size": return_type_size,
                    "field": retrun_type_field,
                }
                prototype["parameters"] = []
                for child in die.iter_children():
                    if child.tag != "DW_TAG_formal_parameter":
                        continue

                    param_type_ref = resolve_referenced_die(dwarf_info, child)
                    param_type, param_type_size = resolve_type(
                        dwarf_info, param_type_ref
                    )
                    param_name = (
                        child.attributes["DW_AT_name"].value.decode("utf-8")
                        if "DW_AT_name" in child.attributes
                        else "unnamed"
                    )

                    param_type_field = resolve_combination_type(
                        dwarf_info, param_type_ref
                    )
                    prototype["parameters"].append(
                        {
                            "name": param_name,
                            "type": param_type,
                            "size": param_type_size,
                            "field": param_type_field,
                        }
                    )

                prototype["file_path"] = get_die_file_path(die, dwarf_info)
                signature.append((name, prototype))
                function_list.remove(name)

            dwarf_info._cu_cache.clear()
            dwarf_info._cu_offsets_map.clear()

    return (signature, function_list)


def print_function_signature(function_name, prototype):
    print(f"{prototype['return']['type']} {function_name} (", end="")
    for i in range(len(prototype["parameters"])):
        print(
            f"{prototype['parameters'][i]['type']} {prototype['parameters'][i]['name']}",
            end="",
        )
        if i != len(prototype["parameters"]) - 1:
            print(", ", end="")
    print(");")

    print(
        f"  Return type {prototype['return']['type']} size: {prototype['return']['size']}"
    )
    if len(prototype["return"]["field"]) != 0:
        print("  Return type fields:")
        for field in prototype["return"]["field"]:
            print(
                f"    - Name: {field['name']}, Type: {field['type']}, Size: {field['size']}, Offset: {field['offset']}"
            )

    for param in prototype["parameters"]:
        print(f"  Parameter {param['name']} type {param['type']} size: {param['size']}")
        if len(param["field"]) != 0:
            print("  Parameter fields:")
            for field in param["field"]:
                print(
                    f"    - Name: {field['name']}, Type: {field['type']}, Size: {field['size']}, Offset: {field['offset']}"
                )

    print(f"File at {prototype['file_path']}")


def dump_result(result):
    for function_name, prototype in result[0]:
        print_function_signature(function_name, prototype)

    for u in result[1]:
        print(f"Function {u} not found in the elf file")


def prototype_diff(prototype1, prototype2):
    if prototype1["type"] != prototype2["type"]:
        print(
            f"  Prototype {prototype1['type']} is different with {prototype2['type']}"
        )

    print(f"  {prototype1['type']} have Different:")
    if prototype1["field"] != [] and prototype2["field"] != []:
        for field1, field2 in zip_longest(prototype1["field"], prototype2["field"]):
            if field1 is None and field2 is not None:
                print(f"    Field2 {field2['type']} not found in field1")
                continue

            if field1 is None and field2 is not None:
                print(f"    Field1 {field1['type']} not found in field2")
                continue

            if field1["type"] != field2["type"]:
                print(f"    Field {field1['type']} type is different")
            if field1["size"] != field2["size"]:
                print(f"    Field {field1['type']} size is different")
            if field1["offset"] != field2["offset"]:
                print(f"    Field {field1['type']} offset is different")

    else:
        if prototype1["size"] != prototype2["size"]:
            print(
                f"    size is different  {prototype1['size']} != {prototype2['size']}"
            )


def diff(result1, result2):
    dict1 = dict(result1)
    dict2 = dict(result2)

    for function_name in dict1.keys():
        if function_name not in dict2:
            print(f"Function {function_name} not found in the second result")
            continue

        prototype1 = dict1[function_name]
        prototype2 = dict2[function_name]
        if prototype1["return"] != prototype2["return"]:
            print(f"Function {function_name} return type is different")
            prototype_diff(prototype1["return"], prototype2["return"])
            print("")

        if len(prototype1["parameters"]) != len(prototype2["parameters"]):
            print(f"Function {function_name} parameters count is different")
            print("")
            continue

        i = 0
        for param1, param2 in zip(prototype1["parameters"], prototype2["parameters"]):
            i += 1
            if (
                param1["type"] != param2["type"]
                or param1["size"] != param2["size"]
                or param1["field"] != param2["field"]
            ):
                print(f"Function {function_name} parameter {i} is different")
                prototype_diff(param1, param2)
                print("")


def find_signature_lib(args, undef):
    signature = []
    for objfile in objfile_iter(args.lib):
        if result := find_function_signature(objfile, undef, die_is_prototyped):
            signature += result[0]
            undef = result[1]

    result = []
    result.append(signature)
    result.append(undef)
    return result


def parse_symbols(args):
    undef = []
    defined = []

    for objfile in objfile_iter(args.lib):

        u, d = symbol_collect(objfile)
        if u is not None:
            undef.extend(u)
        if d is not None:
            defined.extend(d)

    undef = list(set(undef))
    defined = list(set(defined))
    for i in range(len(undef) - 1, -1, -1):
        if undef[i] in defined:
            undef.remove(undef[i])

    signature, not_found = find_function_signature(
        args.elf, undef, die_is_real_function
    )
    signature = sorted(signature, key=lambda x: x[0])
    not_found = sorted(not_found)

    result = []
    result.append(signature)
    result.append(not_found)

    if args.check:
        lib_result = find_signature_lib(args, undef)
        json.dump(list(lib_result), open(f"lib_{args.json}", "w"), indent=4)

    if args.dump:
        print("Dump Elf function signature")
        dump_result(result)

    json.dump(list(result), open(f"{args.json}", "w"), indent=4)


def args_parse():
    parser = argparse.ArgumentParser(
        description=program_description, formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("-a", "--lib", nargs="+", help="Path to liba.so or lib.a")

    parser.add_argument("-e", "--elf", help="Path to elf file")
    parser.add_argument(
        "-c",
        "--check",
        action="store_true",
        help="""If the static library contains debug information,
try to find the function in the static library,
and output the result to lib_<json> file
        """,
    )
    parser.add_argument("-d", "--dump", action="store_true", help="Dump result")
    parser.add_argument(
        "-j", "--json", default="out.json", help="Save result to json file"
    )
    parser.add_argument(
        "-s", "--struct_check", action="store_true", help="Dump struct different"
    )
    parser.add_argument("-i", "--input_json", nargs=2, help="Diff two json files")

    args = parser.parse_args()

    if (not args.input_json and not args.struct_check) and (
        args.elf is None or args.lib is None
    ):
        print("Error: elf and lib must be provided")
        exit(1)

    if args.struct_check and args.elf is None:
        print("Error: elf must be provided")
        exit(1)

    if os.path.exists(args.json):
        print(f"Warning file {args.json} already exists, Will be overwritten")

    if args.check and os.path.exists(f"lib_{args.json}"):
        print(f"Warning file lib_{args.json} already exists, Will be overwritten")

    return args


if __name__ == "__main__":
    args = args_parse()
    if args.input_json:
        result1 = json.load(open(args.input_json[0]))
        result2 = json.load(open(args.input_json[1]))
        diff(result1[0], result2[0])
    elif args.struct_check:
        struct_check(args.elf)
    else:
        parse_symbols(args)
