#!/usr/bin/env bash
# tools/process_config.sh
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

process_file() {
    local output_file="$1"
    local input_file="$2"
    local include_paths=("${!3}")

    while IFS= read -r line || [ -n "$line" ]
    do
        if [[ $line == \#include* ]]; then
            local include_file=$(echo $line | sed -E 's/#include [<"](.+)[">]/\1/')
            local found=false

            # Check current directory first

            if [ -f $include_file ]; then
                process_file $output_file $include_file include_paths[@]
                found=true
            else
                # Then check in the include paths

                for path in "${include_paths[@]}"; do
                    local full_path="$path/$include_file"
                    if [ -f $full_path ]; then
                        process_file $output_file $full_path include_paths[@]
                        found=true
                        break
                    fi
                done
            fi

            # Configuration file not found

            if [ "$found" = false ]; then
                echo "ERROR: Can't find \"$include_file\" in current directory or search paths."
                rm $output_file
                exit 1
            fi
        else
            echo "$line" >> $output_file
        fi
    done < "$input_file"
}

usage() {
    echo "Usage: $0 [OPTIONS] [INPUT_FILE]"
    echo "  -h    display this help"
    echo "  -I    include path, can be specified multiple times"
    echo "  -o    output file"
    echo "  [INPUT_FILE]  the file to be processed"
}

while [[ $# -gt 0 ]]
do
    key="$1"

    case $key in
        -I)
            INCLUDE_PATHS+=("$2")
            shift
            shift
            ;;
        -o)
            OUTPUT_FILE="$2"
            shift
            shift
            ;;
        *)
            if [ -z "$INPUT_FILE" ]; then
                INPUT_FILE="$1"
                shift
            else
                echo "Error: Multiple input files specified"
                usage
                exit 1
            fi
            ;;
    esac
done

if [ -z "$INPUT_FILE" ]; then
    echo "Error: Input file not specified"
    usage
    exit 1
fi

if [ -z "$OUTPUT_FILE" ]; then
    echo "Error: Output file not specified"
    usage
    exit 1
fi

echo "" > $OUTPUT_FILE
process_file "$OUTPUT_FILE" "$INPUT_FILE" INCLUDE_PATHS[@]
