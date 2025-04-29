#!/usr/bin/env bash
############################################################################
# tools/symbolprefix.sh
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
############################################################################

set -e

skip_sections=()
skip_symbols=()

# Parse arguments
args=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-section)
            if [ -n "$2" ]; then
                skip_sections+=("$2")
                shift 2
            else
                echo "Error: --skip-section requires an argument"
                exit 1
            fi
            ;;
        --skip-symbol)
            if [ -n "$2" ] && [ -f "$2" ]; then
                while IFS= read -r line || [[ -n "$line" ]]; do
                    [[ -n "$line" ]] && skip_symbols+=("$line")
                done < "$2"
                shift 2
            else
                echo "Error: --skip-symbol requires a valid file path"
                exit 1
            fi
            ;;
        *)
            args+=("$1")
            shift
            ;;
    esac
done

set -- "${args[@]}"

if [ $# -lt 3 ]; then
    echo "Usage: $0 [--skip-section <section>]... [--skip-symbol <file>] <input_file(s)...> <output_file_or_dir> <prefix>"
    echo "input is a static library or objfile"
    echo "output can be a file or output folder"
    echo "prefix is to add prefix to the sections and symbols in these .a and .o"
    echo "--skip-section <section>: skip this section and its symbols from being renamed"
    echo "--skip-symbol <file>: specify a file with symbol names to skip (one per line)"
    echo "If you want to make a section a shared section,
you can use --skip-section <section> to avoid adding prefix in
the section definition. After adding this option,
a symbol remove_prefix_symbols.txt file belonging to
this section will be generated in the corresponding folder.
You can use --skip-symbol remove_prefix_symbols.txt
the next time you use the script to avoid renaming these symbols."
    exit 1
fi

prefix="${@: -1}"
output="${@: -2:1}"
inputs=("${@:1:$(($#-2))}")

if [ -n "$OBJCOPY" ]; then
    echo "OBJCOPY is defined: $OBJCOPY"
else
    echo "OBJCOPY is not defined, using default objcopy"
    OBJCOPY=objcopy
fi

if [ -n "$READELF" ]; then
    echo "READELF is defined: $READELF"
else
    echo "READELF is not defined, using default readelf"
    READELF=readelf
fi

# Check input files exist
for input in "${inputs[@]}"; do
    if [ ! -f "$input" ]; then
        echo "Error: input file '$input' not found"
        exit 1
    fi
done

# Check output is valid
is_output_dir=0
if [ ${#inputs[@]} -gt 1 ]; then
    if [ -d "$output" ]; then
        is_output_dir=1
    else
        echo "Error: multiple input files require output to be a directory"
        exit 1
    fi
elif [ -d "$output" ]; then
    is_output_dir=1
fi

# Check if section should be skipped
should_skip_section() {
    local section="$1"
    for skip in "${skip_sections[@]}"; do
        if [ "$section" = "$skip" ]; then
            return 0
        fi
    done
    return 1
}

# Call obj copy to rename sections and symbol
process_one() {
    local input="$1"
    local output_file="$2"
    local prefix="$3"

    local rename_args=()

    # search all alloc section to rename
    while read -r section flags; do
        if [[ "$flags" == *A* ]]; then
            if should_skip_section "$section"; then
                echo "[$(basename "$input")] Skipping section $section"
                continue
            fi

            newname=".${prefix}${section#.}"
            rename_args+=(--rename-section "$section=$newname")
        fi
    done < <($READELF -W -S "$input" | awk '
            /^\s*\[[[:space:]]*[0-9]+[[:space:]]*\]/ {
                sub(/^\s*\[[[:space:]]*[0-9]+[[:space:]]*\]\s+/, "")
                split($0, fields, /[[:space:]]+/)
                name = fields[1]
                flags = fields[7]
                print name, flags
            }
            ' | sort -u)

    $OBJCOPY "${rename_args[@]}" --prefix-symbols=$prefix "$input" "$output_file"
}

remove_prefix_args=()
remove_prefix_symbols=""
remove_symbol_prefix() {
    local input="$1"
    local output_file="$2"
    local prefix="$3"

    # Find skipped sections and their indices
    while read -r section ndx; do
        ndx="${ndx//[^0-9]/}"
        if should_skip_section "$section"; then
            echo "[$(basename "$input")] Removing prefix from symbols in section $section"
            # Find symbols with the same index as the skipped section
            while read -r symbol sym_ndx; do
                if [ "$sym_ndx" = "$ndx" ]; then
                    # Remove prefix from symbol
                    newname="${symbol#$prefix}"
                    remove_prefix_args+=(--redefine-sym "$symbol=$newname")
                    remove_prefix_symbols+="$newname\n"
                    echo "[$(basename "$input")] Removing prefix from symbol $symbol -> $newname"
                fi
            done < <($READELF -Ws "$input" | awk '/^\s*[0-9]+:/{print $8, $7}')
        fi
    done < <($READELF -W -S "$input" | awk '/^\s*\[[0-9]+/{print $2, $1}')
}

# Handle all file
for input in "${inputs[@]}"; do
    if [ "$is_output_dir" -eq 1 ]; then
        filename=$(basename "$input")
        final_output="${output%/}/$filename"
    else
        final_output="$output"
    fi

    if [[ "$input" == *.a ]]; then
        echo "[$(basename "$input")] lib Processing $input -> $final_output"
        temp_dir=$(mktemp -d)
        lib=$(realpath "$input")
        cd "$temp_dir"
        ar x "$lib"
        cd "$OLDPWD"
        if [ -z "$(ls -A "$temp_dir")" ]; then
            cp "$lib" "$final_output"
        else
            for objfile in "$temp_dir"/*.o; do
                process_one "$objfile" "$objfile" "$prefix"
                remove_symbol_prefix "$objfile" "$objfile" "$prefix"
            done

            # keep the order of the file in the archive
            final_output=$(realpath "$final_output")
            ar t "$lib" > "$temp_dir/file_list.txt"
            cd "$temp_dir"
            ar rcs "$final_output" $(cat "$temp_dir/file_list.txt")
            cd "$OLDPWD"
        fi

        rm -rf "$temp_dir"
   else
        echo "[$(basename "$input")] obj Processing $input -> $final_output"
        process_one "$input" "$final_output" "$prefix"
        remove_symbol_prefix "$final_output" "$final_output" "$prefix"
    fi
done

# Remove prefix from the skip section symbols
if [[ ${#remove_prefix_args[@]} -ne 0 ]]; then
  for rename in "$output"/*; do
    if [[ "$rename" == *.a || "$rename" == *.o ]]; then
      $OBJCOPY "${remove_prefix_args[@]}" "$rename" "$rename"
    fi
  done
fi

# Save the removed prefix symbols to a file
if [[ -n "$remove_prefix_symbols" ]]; then
  echo -e "$remove_prefix_symbols" > "$output/remove_prefix_symbols.txt"
  echo "[$(basename "$input")] Removed prefix symbols saved to $output/remove_prefix_symbols.txt"
fi

# Remove prefix from the symbols
if [[ ${#skip_symbols[@]} -ne 0 ]]; then
    echo "[$(basename "$input")] Skipping symbols from $input"
    for symbol in "${skip_symbols[@]}"; do
        remove_prefix_args+=(--redefine-sym "$prefix$symbol=$symbol")
    done

    for rename in "$output"/*; do
      if [[ "$rename" == *.a || "$rename" == *.o ]]; then
        echo $OBJCOPY "${remove_prefix_args[@]}" "$rename" "$rename"
        $OBJCOPY "${remove_prefix_args[@]}" "$rename" "$rename"
      fi
    done
fi

echo "add prefix '$prefix' to all sections and symbols in $output"
