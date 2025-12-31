#!/usr/bin/env bash
# find_symbol_callers.sh
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

# find_symbol_callers.sh - Find all call sites of specified symbols
#
# Usage: ./find_symbol_callers.sh <elf_file> <symbol_name> [source_root]
#
# Examples:
#   ./find_symbol_callers.sh nuttx __aeabi_f2d
#   ./find_symbol_callers.sh nuttx "__aeabi_.*" ../
#

set -e

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Check arguments
if [ $# -lt 2 ]; then
    echo "Usage: $0 <elf_file> <symbol_name> [source_root]"
    echo ""
    echo "Examples:"
    echo "  $0 nuttx __aeabi_f2d"
    echo "  $0 nuttx __aeabi_f2d ./"
    echo "  $0 nuttx malloc"
    echo ""
    exit 1
fi

ELF_FILE="$1"
SYMBOL_NAME="$2"
SRC_ROOT="${3:-.}"

# Check toolchain prefix
if command -v arm-none-eabi-objdump &> /dev/null; then
    TOOLCHAIN_PREFIX="arm-none-eabi-"
elif command -v objdump &> /dev/null; then
    TOOLCHAIN_PREFIX=""
else
    echo -e "${RED}Error: objdump tool not found${NC}"
    exit 1
fi

OBJDUMP="${TOOLCHAIN_PREFIX}objdump"
NM="${TOOLCHAIN_PREFIX}nm"
ADDR2LINE="${TOOLCHAIN_PREFIX}addr2line"

# Check if ELF file exists
if [ ! -f "$ELF_FILE" ]; then
    echo -e "${RED}Error: ELF file not found: $ELF_FILE${NC}"
    exit 1
fi

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Symbol Call Analysis Tool${NC}"
echo -e "${CYAN}========================================${NC}"
echo -e "ELF File: ${GREEN}$ELF_FILE${NC}"
echo -e "Symbol: ${GREEN}$SYMBOL_NAME${NC}"
echo -e "Source Root: ${GREEN}$SRC_ROOT${NC}"
echo ""

# Create temporary file
DISASM_FILE=$(mktemp /tmp/disasm_XXXXXX.txt)
trap "rm -f $DISASM_FILE" EXIT

echo -e "${YELLOW}[1/5] Generating disassembly...${NC}"
$OBJDUMP -d "$ELF_FILE" > "$DISASM_FILE"
echo -e "${GREEN}✓ Disassembly complete${NC}"
echo ""

# Find symbol addresses
echo -e "${YELLOW}[2/5] Finding symbol addresses...${NC}"
SYMBOL_ADDRS=$($NM "$ELF_FILE" | grep -E " [TtWw] " | grep -E "$SYMBOL_NAME" | awk '{print $1, $3}')

if [ -z "$SYMBOL_ADDRS" ]; then
    echo -e "${RED}Error: Symbol '$SYMBOL_NAME' not found${NC}"
    echo ""
    echo "Hint: Use the following command to view all available symbols:"
    echo "  $NM $ELF_FILE | grep -i '$SYMBOL_NAME'"
    exit 1
fi

echo -e "${GREEN}Found symbols:${NC}"
echo "$SYMBOL_ADDRS" | while read addr name; do
    echo -e "  ${BLUE}0x$addr${NC} - ${MAGENTA}$name${NC}"
done
echo ""

# Find all call sites
echo -e "${YELLOW}[3/5] Finding call sites...${NC}"

# Build address regex pattern
ADDR_PATTERN=$(echo "$SYMBOL_ADDRS" | awk '{print $1}' | sed 's/^0*//' | paste -sd '|')

if [ -z "$ADDR_PATTERN" ]; then
    echo -e "${RED}Error: Cannot build address pattern${NC}"
    exit 1
fi

# Find all bl/b/jmp instructions to these addresses
CALL_SITES=$(grep -E "bl|b\.w|jmp" "$DISASM_FILE" | grep -E "($ADDR_PATTERN)" | \
    sed 's/^[[:space:]]*//' | awk '{print $1}' | sed 's/://')

if [ -z "$CALL_SITES" ]; then
    echo -e "${YELLOW}No direct call sites found (possibly inlined or unused)${NC}"
    echo ""
    exit 0
fi

CALL_COUNT=$(echo "$CALL_SITES" | wc -l)
echo -e "${GREEN}Found $CALL_COUNT call site(s)${NC}"
echo ""

# Analyze each call site
echo -e "${YELLOW}[4/5] Analyzing caller functions...${NC}"
echo ""

declare -A CALLER_FUNCS
CALLER_COUNT=0

for call_addr in $CALL_SITES; do
    # Find the containing function from disassembly
    FUNC_INFO=$(awk -v addr="$call_addr" '
        /^[0-9a-f]+ <.*>:$/ { 
            funcname=$0
            funcaddr=$1
        }
        $1 == addr":" { 
            print funcaddr "|" funcname
            exit
        }
    ' "$DISASM_FILE")
    
    if [ -n "$FUNC_INFO" ]; then
        FUNC_ADDR=$(echo "$FUNC_INFO" | cut -d'|' -f1)
        FUNC_NAME=$(echo "$FUNC_INFO" | cut -d'|' -f2)
        
        # Deduplicate and count
        if [ -z "${CALLER_FUNCS[$FUNC_NAME]}" ]; then
            CALLER_FUNCS[$FUNC_NAME]="$FUNC_ADDR|1"
            CALLER_COUNT=$((CALLER_COUNT + 1))
        else
            OLD_COUNT=$(echo "${CALLER_FUNCS[$FUNC_NAME]}" | cut -d'|' -f2)
            NEW_COUNT=$((OLD_COUNT + 1))
            CALLER_FUNCS[$FUNC_NAME]="$FUNC_ADDR|$NEW_COUNT"
        fi
    fi
done

echo -e "${GREEN}Found $CALLER_COUNT distinct caller function(s):${NC}"
echo ""

# Print caller function list
for func in "${!CALLER_FUNCS[@]}"; do
    INFO="${CALLER_FUNCS[$func]}"
    ADDR=$(echo "$INFO" | cut -d'|' -f1)
    COUNT=$(echo "$INFO" | cut -d'|' -f2)
    echo -e "${CYAN}$func${NC}"
    echo -e "  Address: ${BLUE}0x$ADDR${NC}"
    echo -e "  Call count: ${MAGENTA}$COUNT${NC}"
done
echo ""

# Find source code locations
echo -e "${YELLOW}[5/5] Finding source code locations...${NC}"
echo ""

for func in "${!CALLER_FUNCS[@]}"; do
    INFO="${CALLER_FUNCS[$func]}"
    ADDR=$(echo "$INFO" | cut -d'|' -f1)
    
    # Extract function name (remove address and brackets)
    FUNC_SIMPLE=$(echo "$func" | sed 's/^[0-9a-f]* <//' | sed 's/>:.*//')
    
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}Function: ${MAGENTA}$FUNC_SIMPLE${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # Try using addr2line
    LINE_INFO=$($ADDR2LINE -e "$ELF_FILE" -f -i "0x$ADDR" 2>/dev/null || echo "")
    
    if [ -n "$LINE_INFO" ] && ! echo "$LINE_INFO" | grep -q "^?"; then
        echo -e "${GREEN}Debug info:${NC}"
        echo "$LINE_INFO" | head -2
    fi
    
    # Search for function definition in source code
    if [ -d "$SRC_ROOT" ]; then
        SRC_FILES=$(grep -rl "$FUNC_SIMPLE" "$SRC_ROOT" --include="*.c" --include="*.cpp" 2>/dev/null | head -5)
        
        if [ -n "$SRC_FILES" ]; then
            echo -e "${GREEN}Possible source files:${NC}"
            echo "$SRC_FILES" | while read file; do
                # Find function definition line
                LINE_NUM=$(grep -n "^\(static \)\?.*$FUNC_SIMPLE\s*(" "$file" 2>/dev/null | head -1 | cut -d':' -f1)
                if [ -n "$LINE_NUM" ]; then
                    echo -e "  ${BLUE}$file:$LINE_NUM${NC}"
                else
                    echo -e "  ${BLUE}$file${NC}"
                fi
            done
        else
            echo -e "${YELLOW}Function definition not found in source code${NC}"
        fi
    fi
    
    # Show disassembly code at call sites
    echo -e "${GREEN}Disassembly snippet:${NC}"
    for call_addr in $CALL_SITES; do
        CALL_FUNC=$(awk -v addr="$call_addr" '
            /^[0-9a-f]+ <.*>:$/ { funcname=$0 }
            $1 == addr":" { print funcname; exit }
        ' "$DISASM_FILE")
        
        if [ "$CALL_FUNC" = "$func" ]; then
            # Show 5 lines before and after call site
            grep -A5 -B5 "^[[:space:]]*$call_addr:" "$DISASM_FILE" | \
                sed "s/^[[:space:]]*$call_addr:/  >>> $call_addr:/" | head -11
            echo ""
        fi
    done
    
    echo ""
done

echo -e "${CYAN}========================================${NC}"
echo -e "${GREEN}Analysis complete!${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""
echo -e "${YELLOW}Summary:${NC}"
echo -e "  • Symbol called ${MAGENTA}$CALL_COUNT${NC} time(s)"
echo -e "  • Involves ${MAGENTA}$CALLER_COUNT${NC} distinct function(s)"
echo ""
