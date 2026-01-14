#!/usr/bin/env python3
############################################################################
# tools/codeowners/codeoweners.py
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

import os
import re
import subprocess
import sys
from multiprocessing import Pool
from typing import TypeAlias

# Number of processes to use
N_PROCESSES: int = 20

# Top `n` authors to list as code owners in the CODEOWNERS file
TOP_N_AUTHORS: int = 5

# git repository folder
GIT_FOLDER: str = ".git"

# Command to get author email and number of changes
CHANGE_STATS_CMD: list[str] = [
    "git",
    "--no-pager",
    "log",
    '--pretty="%ae"',
    "--shortstat",
]

# Command to list all files tracked by git
GIT_FILE_CMD: list[str] = ["git", "ls-files"]

# Locations to ignore for code owner generation
IGNORE_LOCS: list[str] = [
    # Generated release notes
    "Documentation/ReleaseNotes/*",
    # Boiler plate files
    "**/__init__.py",
    # Files that aren't really necessary to have owners
    "**/Make.defs",
    "**/CMakeLists.txt",
    # Image files from documentation
    "*.png",
    "*.jpg",
]

# Represents a code owner by email
CodeOwner: TypeAlias = str


class Ownership:
    """Represents an ownership relationship for a path."""

    def __init__(self, owner: CodeOwner, path: str, changes: int) -> None:
        """Creates a new ownership relationship."""
        self.owner: CodeOwner = owner
        self.path: str = path
        self.changes: int = changes

    def __str__(self) -> str:
        return f"Ownership(path='{self.path}', owner='{self.owner}', changes={self.changes})"

    def __repr__(self) -> str:
        return self.__str__()


# Represents all contributors to a file
Roster: TypeAlias = dict[str, list[Ownership]]


def parse_change_count(changestr: str) -> int:
    """Parses a string summary of changes to the file into an integer number of changes."""

    change_types = changestr.split(",")

    # Skip the number of files changed, which is always the first index
    count = 0
    for change in change_types[1:]:
        value = re.search(r"\d+", change)
        if value is not None:
            count += int(value.group())

    return count


def get_owners(path: str) -> list[Ownership]:
    """Gets ownership relationships for `path`."""

    cmd = CHANGE_STATS_CMD.copy() + [path]
    raw_logs = subprocess.run(cmd, capture_output=True, text=True).stdout
    logs = [e for e in raw_logs.split("\n") if e != ""]

    ownerships: dict[CodeOwner, Ownership] = dict()
    for raw_email, raw_changes in zip(logs[::2], logs[1::2]):
        owner = raw_email.replace('"', "")
        change_count = parse_change_count(raw_changes)

        if ownerships.get(owner) is None:
            ownerships[owner] = Ownership(
                path=path,
                owner=owner,
                changes=change_count,
            )
        else:
            ownerships[owner].changes += change_count

    return list(ownerships.values())


def sort_by_changes(owners: list[Ownership]) -> None:
    """Sorts `owners` by the number of changes each owner has made, from most to least changes."""
    owners.sort(key=lambda o: o.changes, reverse=True)


def filter_owners(owners: list[Ownership], ignore: list[CodeOwner]) -> list[Ownership]:
    """Returns a list composing of the original `owners` list, but without any of the owners in the `ignore` list."""
    return [o for o in owners if o.owner not in ignore]


def get_files() -> list[str]:
    """Returns a list of currently tracked files in the git repository."""

    ignored_dirs = []
    for loc in IGNORE_LOCS:
        ignored_dirs.append(f":!:{loc}")
    cmd = GIT_FILE_CMD.copy() + ["--"] + ignored_dirs
    return (
        subprocess.run(cmd, text=True, capture_output=True).stdout.strip().split("\n")
    )


def parse_ignore_list(path: str) -> list[CodeOwner]:
    """Parses a file of code owners to ignore into a list."""

    owners = []
    with open(path, "r") as file:
        for line in file:
            owners.append(line.strip())
    return owners


def generate_codeowners(roster: Roster) -> None:
    """Generates the output code-owner file in the console from the `roster`."""

    for file, ownerships in roster.items():
        print(file, end=" ")
        for o in ownerships:
            print(o.owner, end=" ")
        print()


def main() -> None:

    # Check that we are running in the repository root
    if GIT_FOLDER not in os.listdir():
        print("You should only run this script from the root of the git repository.")
        return

    ignore_list = []

    if len(sys.argv) < 2:
        print(
            "You can pass in the path to the list of emails to ignore when generating the code owner file."
            "This prevents these authors from getting auto-requested in PR reviews."
        )
        print("Ex: ./tools/codeowners.py ./tools/codeowners-ignore.txt")
    else:
        ignore_list = parse_ignore_list(sys.argv[1])

    source_files = get_files()
    roster: Roster = dict()

    with Pool(N_PROCESSES) as p:

        for file, ownerships in zip(source_files, p.map(get_owners, source_files)):
            sort_by_changes(ownerships)  # Sort by greatest change count
            ownerships = ownerships[:TOP_N_AUTHORS]  # Only use the top `N`
            ownerships = filter_owners(
                ownerships, ignore_list
            )  # Filter authors who wish to be ignored

            # Skip empty lines since they do nothing of value
            if len(ownerships) == 0:
                continue

            # Store in roster
            roster[file] = ownerships

    # Print like code-owner file
    generate_codeowners(roster)


if __name__ == "__main__":
    main()
