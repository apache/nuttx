#!/usr/bin/env python3
# tools/process_config.py
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

import json
import os
import re
import shutil
import sys
from collections import OrderedDict
from pathlib import Path


def parse_config_line(line):
    """
    Parse a configuration line and return the key and value.

    Args:
        line (str): A line from a configuration file

    Returns:
        tuple: (key, value) if the line contains a configuration, (None, None) otherwise

    Handles two formats:
    1. "# CONFIG_XXX is not set" -> returns (CONFIG_XXX, 'n')
    2. "CONFIG_XXX=value" -> returns (CONFIG_XXX, value)
    """
    line = line.strip()
    if not line:
        return None, None

    # Handle "# CONFIG_XXX is not set" format
    if line.startswith("# ") and line.endswith(" is not set"):
        config_name = line.split()[1]
        return config_name, "n"

    # Handle "CONFIG_XXX=value" format
    if "=" in line:
        key, value = line.split("=", 1)
        return key, value

    return None, None


def opposite(value):
    if value == "n":
        return "y"
    else:
        return "n"


def expand_file(input_path, include_paths, processed=None, tree_node=None):
    """
    Recursively expand a configuration file with #include directives.

    Args:
        input_path (str): Path to the input configuration file
        include_paths (list): List of directories to search for included files
        processed (set, optional): Set of already processed files to avoid circular includes
        tree_node (dict, optional): Node in the configuration tree being built

    Returns:
        tuple: (list of expanded lines, tree structure node)

    This function:
    1. Reads the input file line by line
    2. Processes #include directives by recursively expanding included files
    3. Parses configuration lines to build a configuration dictionary
    4. Builds a tree structure representing the file inclusion hierarchy
    5. Returns the expanded content and the tree structure
    """
    if processed is None:
        processed = set()
    if tree_node is None:
        tree_node = {
            "file": str(input_path),
            "includes": [],
            "configs": OrderedDict(),
            "include_lines": [],
            "raw_content": [],  # Store original content for postprocessing
        }

    input_path = Path(input_path).resolve()
    if input_path in processed:
        return [], tree_node
    processed.add(input_path)

    expanded_lines = []
    current_configs = OrderedDict()

    with input_path.open("r", encoding="utf-8") as f:
        lines = f.readlines()

    # Save original content for postprocessing
    tree_node["raw_content"] = [line.rstrip("\n") for line in lines]

    for line in lines:
        line_strip = line.strip()
        match = re.match(r"#include\s*[<\"]([^\">]+)[\">]", line_strip)
        if match:
            include_file = match.group(1)
            found = False

            # Record original include line for postprocessing
            tree_node["include_lines"].append(line.rstrip("\n"))

            # Check current directory first
            direct_path = input_path.parent / include_file
            if direct_path.exists():
                include_node = {
                    "file": str(direct_path),
                    "includes": [],
                    "configs": OrderedDict(),
                    "include_lines": [],
                    "raw_content": [],
                }
                tree_node["includes"].append(include_node)

                # Recursively expand the included file
                included_lines, include_node = expand_file(
                    direct_path, include_paths, processed, include_node
                )
                expanded_lines.extend(included_lines)

                # Merge configurations (later configurations override earlier ones)
                for key, value in include_node["configs"].items():
                    current_configs[key] = value
                    tree_node["configs"][key] = value

                found = True
            else:
                # Check include paths
                for path in include_paths:
                    candidate = Path(path) / include_file
                    if candidate.exists():
                        include_node = {
                            "file": str(candidate),
                            "includes": [],
                            "configs": OrderedDict(),
                            "include_lines": [],
                            "raw_content": [],
                        }
                        tree_node["includes"].append(include_node)

                        # Recursively expand the included file
                        included_lines, include_node = expand_file(
                            candidate, include_paths, processed, include_node
                        )
                        expanded_lines.extend(included_lines)

                        # Merge configurations
                        for key, value in include_node["configs"].items():
                            current_configs[key] = value
                            tree_node["configs"][key] = value

                        found = True
                        break

            if not found:
                print(
                    f'ERROR: Cannot find "{include_file}" from {input_path}',
                    file=sys.stderr,
                )
                sys.exit(1)
        else:
            # Parse configuration line
            key, value = parse_config_line(line)
            if key is not None:
                current_configs[key] = value
                tree_node["configs"][key] = value
            expanded_lines.append(line)

    expanded_lines.append("\n")  # Maintain separation between files
    return expanded_lines, tree_node


def preprocess(output_path, input_path, include_paths, tree_output_path=None):
    """
    Process a configuration file with #include directives.

    Args:
        output_path (str): Path to write the expanded configuration
        input_path (str): Path to the input configuration file
        include_paths (list): List of directories to search for included files
        tree_output_path (str, optional): Path to write the tree structure

    This function:
    1. Expands the input file by processing #include directives
    2. Writes the expanded configuration to output_path
    3. Optionally writes the tree structure to tree_output_path for postprocessing
    """
    lines, tree = expand_file(input_path, include_paths)

    # Write expanded configuration
    with open(output_path, "w", encoding="utf-8") as out:
        out.writelines(lines)

    # Write tree structure if requested
    if tree_output_path and tree["includes"]:
        with open(tree_output_path, "w", encoding="utf-8") as f:
            json.dump(tree, f, indent=2, ensure_ascii=False)


def get_all_included_configs(tree):
    """
    Extract all configuration options from included files.

    Args:
        tree (dict): The configuration tree structure

    Returns:
        OrderedDict: Dictionary of configuration options from all included files

    This function recursively traverses the tree to collect all configurations
    from files included via #include directives.
    """
    included_configs = OrderedDict()

    def collect_configs(node):
        for include in node.get("includes", []):
            collect_configs(include)
        for key, value in node.get("configs", {}).items():
            included_configs[key] = value

    # Collect configurations from included files only (not the main file)
    for include in tree.get("includes", []):
        collect_configs(include)

    return included_configs


def get_main_configs(tree):
    """
    Extract configuration options from the main file (excluding #include directives).

    Args:
        tree (dict): The configuration tree structure

    Returns:
        OrderedDict: Dictionary of configuration options from the main file
    """
    main_configs = OrderedDict()
    for line in tree["raw_content"]:
        key, value = parse_config_line(line)
        if key is not None:
            main_configs[key] = value
    return main_configs


def get_current_configs(config_path):
    """
    Parse the current full configuration file.

    Args:
        config_path (str): Path to the current configuration file

    Returns:
        OrderedDict: Dictionary of configuration options from the current file
    """
    configs = OrderedDict()
    with open(config_path, "r", encoding="utf-8") as f:
        for line in f:
            key, value = parse_config_line(line)
            if key is not None:
                configs[key] = value
    return configs


def postprocess_inner(tree_path, added, changed, removed, output_path):
    """
    Postprocess configuration changes to generate a defconfig with #include directives.

    This function takes the specific changes (added, changed, removed) calculated
    by postprocess and applies them to the original defconfig structure
    represented by the tree, producing a new defconfig file.

    Args:
        tree_path (str): Path to the config_tree.json generated during preprocessing of the ORIGINAL defconfig.
        added (dict): {key: value} - Configurations added by the user.
        changed (dict): {key: (old_value, new_value)} - Configurations changed by the user.
        removed (dict): {key: old_value} - Configurations removed by the user.
        output_path (str): Path where the new defconfig should be written.
    """
    # 1. Load the original tree structure (this represents the structure of the ORIGINAL defconfig)
    with open(tree_path, "r", encoding="utf-8") as f:
        original_tree = json.load(f, object_pairs_hook=OrderedDict)

    # 2. Get the original configuration sets from the tree
    original_included_configs = get_all_included_configs(original_tree)
    original_main_configs = get_main_configs(original_tree)

    # 3. Dictionary to store the final configurations that will go into the main defconfig file
    final_main_configs = OrderedDict()

    # --- Logic to determine final content of the main defconfig file ---

    # a. Handle configurations that were originally in included files
    #    We only place them in the main defconfig if they were explicitly added/changed/removed.
    #    If untouched, they remain in their included files implicitly.
    for key in original_included_configs:
        if key in added:
            # User added/changed a config that was originally in an included file.
            # It must now be explicitly set in the main defconfig to override the included value.
            final_main_configs[key] = added[key]
        elif key in changed:
            # User changed a config that was originally in an included file.
            final_main_configs[key] = changed[key][1]  # Use the new value
        elif key in removed:
            # User removed a config that was originally in an included file.
            # To "remove" it, we explicitly set it to opposite orig value in the main defconfig.
            # This overrides the value from the included file.
            final_main_configs[key] = opposite(removed[key])

    # b. Handle configurations that were originally in the main file
    #    They should generally stay represented in the main file output.
    for key in original_main_configs:
        if key in added:
            # User added/changed a config that was already in the main file.
            final_main_configs[key] = added[key]
        elif key in changed:
            # User changed a config that was in the main file.
            final_main_configs[key] = changed[key][1]  # Use the new value
        elif key in removed:
            # User removed a config that was in the main file.
            # Explicitly set to  opposite orig value to override its previous state.
            final_main_configs[key] = opposite(removed[key])
        else:
            # Config was in the original main file and user did NOT touch it.
            # According to the new logic, we should PRESERVE these in the output main defconfig
            # to maintain the structure and non-default values from the original main file.
            # This prevents the output from becoming sparse if the user only made minor changes.
            final_main_configs[key] = original_main_configs[key]

    # c. Handle configurations that are entirely new (not present in original main or included)
    #    These must go into the main defconfig file.
    for key, value in added.items():
        if key not in original_main_configs and key not in original_included_configs:
            final_main_configs[key] = value

    # 4. Write the final output defconfig file
    with open(output_path, "w", encoding="utf-8") as f:
        # Write the original #include directives to preserve the structure
        for include_line in original_tree.get("include_lines", []):
            f.write(include_line + "\n")

        # Add a newline for separation if there were includes
        if original_tree.get("include_lines"):
            f.write("\n")

        # Write the final configurations for the main file in sorted order
        final_write_list = []
        for key, value in final_main_configs.items():
            if value == "n":
                final_write_list.append(f"# {key} is not set\n")
            else:
                final_write_list.append(f"{key}={value}\n")

        # Sort configurations for consistent and readable output
        final_write_list.sort()
        for write_line in final_write_list:
            f.write(write_line)


def get_config_diff(old_config, new_config):
    """
    Compare two config dictionaries and return the differences.

    Args:
        old_config (dict): The original configuration.
        new_config (dict): The modified configuration.

    Returns:
        tuple: (added, changed, removed)
            added (dict): Items in new_config but not in old_config.
            changed (dict): Items with different values. {key: (old_value, new_value)}.
            removed (dict): Items in old_config but not in new_config.
    """
    added = {}
    changed = {}
    removed = {}

    # Find added and changed items
    for key, new_value in new_config.items():
        if key not in old_config:
            added[key] = new_value
        elif old_config[key] != new_value:
            changed[key] = (old_config[key], new_value)  # (old_value, new_value)

    # Find removed items
    for key in old_config:
        if key not in new_config:
            removed[key] = old_config[key]

    return added, changed, removed


def load_config_file(filepath):
    """
    Load a .config or defconfig file into an OrderedDict.
    """
    config = OrderedDict()
    try:
        with open(filepath, "r") as f:
            for line in f:
                key, value = parse_config_line(line)
                if key is not None:
                    config[key] = value
    except FileNotFoundError:
        print(
            f"Warning: Config file {filepath} not found. Treating as empty.",
            file=sys.stderr,
        )
    return config


def postprocess(
    tree_path, original_defconfig_path, modified_defconfig_path, output_defconfig_path
):
    """
    An improved postprocess that compares defconfig files before and after modification.

    This function addresses the issue where Kconfig's savedefconfig omits default values,
    making it hard to distinguish user deletions from optimizations.

    Args:
        tree_path (str): Path to the config_tree.json generated during preprocessing.
        original_defconfig_path (str): Path to the defconfig file BEFORE user modification.
        modified_defconfig_path (str): Path to the defconfig file AFTER user modification.
        output_defconfig_path (str): Path where the updated defconfig should be written.
    """
    # 1. Load the defconfig files
    defconfig_original = load_config_file(original_defconfig_path)
    defconfig_modified = load_config_file(modified_defconfig_path)

    # 2. Compare the defconfig files to find the actual user changes
    added, changed, removed = get_config_diff(defconfig_original, defconfig_modified)

    # 3. Use the new postprocess_inner function to generate the final defconfig
    #    Pass the calculated differences (added, changed, removed) and the original tree.
    postprocess_inner(tree_path, added, changed, removed, output_defconfig_path)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(
            "Usage: process_config.py <mode> [options]",
            file=sys.stderr,
        )
        print("Modes:", file=sys.stderr)
        print(
            "  preprocess <output_file> <input_file> [include_paths...] [--tree <tree_file>]",
            file=sys.stderr,
        )
        print(
            "  postprocess <tree_file> <original_defconfig> <modified_defconfig> <output_defconfig>",
            file=sys.stderr,
        )
        sys.exit(1)

    mode = sys.argv[1]

    if mode == "preprocess":
        if len(sys.argv) < 4:
            print(
                "Usage: preprocess <output_file> <input_file> [include_paths...] [--tree <tree_file>]",
                file=sys.stderr,
            )
            sys.exit(1)

        output_file = Path(sys.argv[2])
        input_file = sys.argv[3]
        include_dirs = []
        tree_file = None

        # Parse arguments
        i = 4
        while i < len(sys.argv):
            if sys.argv[i] == "--tree" and i + 1 < len(sys.argv):
                tree_file = sys.argv[i + 1]
                i += 2
            else:
                include_dirs.append(sys.argv[i])
                i += 1

        if output_file.exists():
            output_file.unlink()

        preprocess(output_file, input_file, include_dirs, tree_file)

    elif mode == "postprocess":
        if len(sys.argv) < 6:
            print(
                "Usage: postprocess <tree_file> <original_defconfig> <modified_defconfig> <output_defconfig>",
                file=sys.stderr,
            )
            sys.exit(1)

        tree_file = sys.argv[2]
        original_defconfig = sys.argv[3]
        modified_defconfig = sys.argv[4]
        output_defconfig = sys.argv[5]
        if Path(tree_file).is_file():
            post_defconfig = output_defconfig + "tmp"
            postprocess(
                tree_file, original_defconfig, modified_defconfig, post_defconfig
            )
            shutil.copy2(post_defconfig, output_defconfig)
            os.remove(post_defconfig)
        else:
            shutil.copy2(modified_defconfig, output_defconfig)

    else:
        print(f"Unknown mode: {mode}", file=sys.stderr)
        sys.exit(1)
