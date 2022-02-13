#!/usr/bin/env bash
############################################################################
# tools/mkallsyms.sh
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

export LC_ALL=C

usage="Usage: $0 <ELFBIN> <CROSSDEV>"

# Get the symbol table

# Extract all of the symbols from the ELF files and create a
# list of sorted, unique undefined variable names.

# Now output the symbol table as a structure in a C source file.  All
# undefined symbols are declared as void* types.  If the toolchain does
# any kind of checking for function vs. data objects, then this could
# failed

nm="${2}nm"
filt="${2}c++filt"
if [ -f "${1}" ];then
  count=`${nm} -n ${1} | grep -E " [T|t] "  | uniq | wc -l`
else
  count=0
fi

echo "#include <nuttx/compiler.h>"
echo "#include <nuttx/symtab.h>"
echo ""
echo "#if defined(__GNUC__) && !defined(__clang__)"
echo "#  pragma GCC diagnostic ignored \"-Wbuiltin-declaration-mismatch\""
echo "#endif"
echo ""
echo "const int             g_nallsyms = ${count} + 2;"
echo "const struct symtab_s g_allsyms[${count} + 2] = "
echo "{"

# Add start address boundary

echo "  { \"Unknown\", (FAR const void *)0x00000000 },"

if [ -f "${1}" ];then
  ${nm} -n ${1} | grep -E " [T|t] "  | uniq | \
  while read addr type name
  do
    echo "  { \"`${filt} -p $name`\", (FAR const void *)0x$addr },"
  done
fi

# Add end address boundary

echo "  { \"Unknown\", (FAR const void *)0xffffffff },"

echo "};"
