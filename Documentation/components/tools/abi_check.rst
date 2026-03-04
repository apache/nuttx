================
``abi_check.py``
================

``abi_check.py`` is a Python tool for checking binary compatibility based on
DWARF debug information.

It supports three related workflows:

1. Given one or more static libraries (``.a``) and an ELF file, collect the
   undefined (external) symbols referenced by the libraries, locate those
   functions in the ELF file, and write their function signatures to a JSON
   file.
2. Generate two JSON files from two different ELF files (for example, an old
   build and a new build), and compare the signatures of functions with the
   same name (return type, parameters, and for structs also size, member
   offsets, member types, etc.).
3. Given a single ELF file, detect structs with the same name but different
   members.

Prerequisites:

- Python 3
- ``pyelftools`` (used to read ELF/DWARF)
- ``ar`` (used to extract object files from ``.a`` archives)
- ``pahole`` (only for ``--struct_check``)

.. note::

   Although the help text mentions ``.so``, the current implementation uses
   ``ar x`` on each ``--lib`` input, so it expects ``.a`` archives.

Help message::

  $ python3 tools/abi_check.py -h
  usage: abi_check.py [-h] [-a LIB [LIB ...]] [-e ELF] [-c] [-d] [-j JSON] [-s]
                      [-i INPUT_JSON INPUT_JSON]

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

  options:
    -h, --help            show this help message and exit
    -a LIB [LIB ...], --lib LIB [LIB ...]
                          Path to liba.so or lib.a
    -e ELF, --elf ELF     Path to elf file
    -c, --check           If the static library contains debug information,
                          try to find the function in the static library,
                          and output the result to lib_<json> file
    -d, --dump            Dump result
    -j JSON, --json JSON  Save result to json file
    -s, --struct_check    Dump struct different
    -i INPUT_JSON INPUT_JSON, --input_json INPUT_JSON INPUT_JSON
                          Diff two json files

Examples::

  # 1) Extract signatures for external APIs referenced by one or more archives
  $ python3 tools/abi_check.py -a libfoo.a libbar.a -e nuttx -j out.json

  # 2) Compare signatures across two ELF files
  $ python3 tools/abi_check.py -a libfoo.a libbar.a -e nuttx_old -j old.json
  $ python3 tools/abi_check.py -a libfoo.a libbar.a -e nuttx_new -j new.json
  $ python3 tools/abi_check.py -i old.json new.json

  # 3) Find struct definition mismatches within a single ELF
  $ python3 tools/abi_check.py -e nuttx -s
