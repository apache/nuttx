#!/usr/bin/env python3
"""
generate_cmake_presets.py - Generate CMake User Presets for all NuttX boards/configs.
Run from NuttX root directory, or as a Git hook.
"""

import json
import os
import subprocess
import sys

# --- Locate NuttX root (parent of the tools/ directory) ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
NUTTX_ROOT = os.path.dirname(SCRIPT_DIR)  # since script is under tools/

os.chdir(NUTTX_ROOT)

GENERATED_PRESETS_FILE = "CMakeUserPresets.json"
USER_PRESETS_FILE = ""

print(f"[generate_cmake_presets] NuttX root: {NUTTX_ROOT}")

# --- Run ./tools/configure.sh -L to get all board configs ---
try:
    output = subprocess.check_output(
        ["./tools/configure.sh", "-L"], text=True, stderr=subprocess.DEVNULL
    )
except FileNotFoundError:
    print("Error: ./tools/configure.sh not found. Are you in the NuttX root directory?")
    sys.exit(1)
except subprocess.CalledProcessError as e:
    print(f"Error running configure.sh: {e}")
    sys.exit(1)

configs = []
for line in output.strip().splitlines():
    line = line.strip()
    if line and ":" in line and not line.startswith("Board Configurations:"):
        configs.append(line)

if not configs:
    print("Warning: No board configurations found.")
    # Keep empty list

presets = []
for config in configs:
    board, conf = config.split(":", 1)
    preset_name = f"{board}_{conf}".replace("-", "_").replace(".", "_")
    presets.append(
        {
            "name": preset_name,
            "inherits": "default",
            "cacheVariables": {"BOARD_CONFIG": config},
        }
    )

with open(GENERATED_PRESETS_FILE, "w") as f:
    json.dump({"version": 3, "configurePresets": presets}, f, indent=2)

print(f"Generated {len(presets)} board presets in '{GENERATED_PRESETS_FILE}'")
