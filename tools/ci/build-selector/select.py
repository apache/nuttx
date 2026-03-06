#!/usr/bin/env python3
############################################################################
# tools/ci/build-selector/select.py
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

"""
The intention of this script is to analyze the paths of modified NuttX files for
the minimum number of configurations/builds that will sufficiently test the
change set. The end result is (ideally) that CI processes run only the builds
necessary for testing a change set, taking advantage of our knowledge about the
source tree (changes in boards/arm64/bcm2711 are localized to the BCM2711
*only*).

INTERFACE:

The input of this program is the change set in the form of a list of modified
in-tree files. This is passed via the command line.

The output of this program is a list of configurations (corresponding to in-tree
defconfig files) that together form an adequate test for the change set

REQUIREMENTS:

- Granularity of the selected builds should be as small as single configurations
  (i.e. a change to qemu-armv8a/configs/nsh/defconfig should only result in
  qemu-armv8a:nsh being built).

- For now, any changes outside of arch/ or boards/ is to be considered "complex"
  and result in all configurations being built. TODO: improve this

- If we have two modified defconfig files in two separate architectures (or
  boards, etc.), this program should not result in all configurations being
  selected for building. It should only select those two (or more)
  configurations associated with the modified defconfigs.
  Ex: boards/arm/rp2040/raspberrypi-pico/configs/nsh/defconfig,
  boards/arm64/qemu-armv8a/configs/nsh/defconfig should result in only
  raspberrypi-pico:nsh and qemu-armv8a:nsh being built

- The above applies to any two changes that do not share a common root ancestor;
  we should always select the minimum necessary builds to test the modified
  files.
"""

import sys
from pathlib import Path


def collapse_paths(pathset: set[Path]) -> list[Path]:
    """
    Collapses a `pathset` into those paths which are the greatest common
    divisors for the set. Only unique parents of changed files remain in the
    returned list.

    Return: A list of paths that are unique parents of all paths in `pathset`.
    """

    collapsed: list[Path] = list(pathset)

    i = 0
    while i < len(collapsed):

        # See if there exists another path in the set that supersedes this
        # one. If there is, then we can skip over this path. If there isn't,
        # then this path is one of our unique paths found so far and it can go
        # in the list

        deleted = False
        for other_path in pathset:

            # Do not consider self-to-self comparisons
            if collapsed[i] == other_path:
                continue

            # A superseding path is reason to delete this one
            if collapsed[i].is_relative_to(other_path):
                del collapsed[i]
                deleted = True
                break

        # Only move to the next item if we didn't delete something
        if not deleted:
            i += 1

    return collapsed


def arch_to_board(path: Path) -> Path:
    """
    This function converts any arch/ paths into their corresponding paths in
    boards/. For example, arch/arm/src/rp2040 should become boards/arm/rp2040.

    The structure of the path names we care to analyze:
    arch/<arch-name>/[src|include]/<chip-name>/* (we don't care after chip name)
    boards/<arch-name>/<chip-name>/<board-name>/configs/<config-name>/*
    """

    # Path doesn't need to be converted
    if not path.is_relative_to("arch/"):
        return path

    new_path = str(path).replace("arch", "boards")
    new_path = new_path.replace("src/", "")
    new_path = new_path.replace("include/", "")

    return Path(new_path)


def main() -> None:

    # The only argument to this program is a list of changed files
    # EX: $ select arch/Kconfig drivers/sensors/bmi270.c ...

    raw_change_set: list[str] = sys.argv[1:]
    change_set: list[Path] = [Path(p) for p in raw_change_set]

    # If the change set contains any paths which do not fall under boards/ or
    # arch/, then this is a complex PR and we immediately select all builds

    for path in change_set:
        if not path.is_relative_to("boards/") and not path.is_relative_to("arch/"):
            for build in Path("boards/").rglob("defconfig"):
                print(build)
            return

    # This is the more complex case. We now want to resolve the minimum set of
    # builds that will test the change set.

    # Files in the <config-name>/ subdir trigger builds of modified configs
    # Files in the <board-name>/ subdir trigger builds of configs for only that
    # board
    # Files in the <chip-name>/ subdir trigger builds of configs for only that
    # chip
    # Files in the <arch-name>/ subdir trigger builds of configs for only that
    # arch

    # First step is to strip away all the names of the files. This gives us only
    # the relevant directories. We will maintain this in a list to avoid
    # considering duplicates (i.e. all files in the same subdir resolve to one
    # subdir).

    touched_paths: set[Path] = set()
    for path in change_set:
        touched_paths.add(path.parent)

    # We now have a set of affected paths, not including duplicates. We should
    # see if there is any overlap. For example, let's say the change set had the
    # files:
    #
    # - arch/arm/Kconfig
    # - arch/arm/src/rp2040/rp2040_pio.c
    #
    # Here, our `touched_paths` are {arch/arm, arch/arm/src/rp2040}
    #
    # We can see that 'arch/arm' is our greatest common divisor, and so we can
    # remove 'arch/arm/src/rp2040' from consideration because all of its
    # selected builds will have to be selected for 'arch/arm' anyways.

    minimal_list: list[Path] = collapse_paths(touched_paths)

    # At this point we have a minimum list of paths that encompass all the
    # changes. We must convert arch/ paths to their corresponding board/ path in
    # order to get the appropriate list of builds to select

    minimal_list = [arch_to_board(p) for p in minimal_list]

    # We also want to make sure that we only care about the first three levels
    # of board/ paths if the path does not point to a config/ directory. For
    # instance, searching boards/arm/rp2040/raspberrypi-pico/src for defconfig
    # files does nothing. We should search boards/arm/rp2040/raspberrypi-pico
    # This is what `path.parents[-5]` does (syntax is strange)

    for i in range(len(minimal_list)):
        if "configs" not in str(minimal_list[i]) and len(minimal_list[i].parents) > 4:
            minimal_list[i] = minimal_list[i].parents[-5]

    # Remove any new duplicates

    minimal_list = collapse_paths(set(minimal_list))

    # If we create a list of all defconfig files that are children of these
    # paths, we have a minimum list of builds that covers the change set!

    selected_builds: list[Path] = list()
    for path in set(minimal_list):
        defconfigs = list(path.rglob("defconfig"))
        selected_builds.extend(defconfigs)

    # Since we chose selected builds with our minimal_set, there should be no
    # overlap in the selected defconfigs. If there is, I wrote bad logic :)
    assert len(selected_builds) == len(set(selected_builds))

    # Now, let's output the selected builds
    for build in selected_builds:
        print(build)


if __name__ == "__main__":
    main()
