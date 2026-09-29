#!/usr/bin/env python3
# tools/ci/check_doc_coverage.py
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

"""Check that boards/ and Documentation/platforms/ describe the same hardware.

A board supported by NuttX lives in::

    boards/<arch>/<chip>/<board>/

and is documented in::

    Documentation/platforms/<arch>/.../boards/<board>/index.rst

The documentation is allowed to be more finely grained than the source tree --
Documentation/platforms/avr/atmega/atmega2560/ has a page per part where
boards/avr/atmega/ has none -- so the board page is looked up at any depth.
What has to match is the set of boards, not the shape of the directories.

Nothing used to enforce that mapping, so the two trees drifted: boards were
added without a page, pages outlived the board they described, and several
directories ended up spelled differently on each side.  The drift is not a
state, it is a rate -- it grows with every board that is merged -- so it has
to be checked rather than fixed once.

Every board page also has to carry the tags that say where it belongs::

    .. tags:: arch:<arch>, chip:<chip>

Both are derived from the directory the page lives in, so they can be checked
rather than trusted.  That is what keeps the tag index usable: a filter that
covers some of the boards, or that spells the same architecture two ways, is
worse than no filter at all.

Known gaps are listed in Documentation/platforms/doc-coverage-ignore.txt so
that the check can be switched on without first writing every missing page.
That file is a to-do list: entries should only ever be removed from it.

Exits non-zero when a board or a page is missing and not listed there, or when
a board page is missing or contradicts its arch: or chip: tags.
"""

import argparse
import re
import sys
from pathlib import Path

# Directories under boards/<arch>/ and boards/<arch>/<chip>/ that hold shared
# code rather than a board port.
NOT_A_BOARD = {"common", "drivers", "tools"}

# The controlled tag vocabulary.  See
# Documentation/contributing/doc_templates/board.rst for what each one means.
#
# arch: and chip: name directories, so they are derived from where the page
# lives.  part: is the exact chip on the board and only a human knows it.
# vendor: is derived from the chip family through chip-vendors.txt.
#
# What a board offers and how far its port has been taken are deliberately not
# tags.  Those are facts about the hardware, and a hand written tag that
# covers a twentieth of the boards reads as "only these boards have Ethernet",
# which is worse than saying nothing.  They belong in the Support Status and
# Peripheral Support sections of the page.
NAMESPACES = {"arch", "chip", "part", "vendor"}

VENDOR_FILE = Path("Documentation/platforms/chip-vendors.txt")

# Chip directories that document hardware the source tree no longer has.  The
# pages are kept on purpose, as a record of what happened:
# Documentation/platforms/arm/bcm2708/ says that the Raspberry Pi Zero port
# was never finished and was removed in NuttX 7.28.  Adding to this set should
# take a conversation, not a commit.
REMOVED_FROM_TREE = {"arm/bcm2708"}

IGNORE_FILE = Path("Documentation/platforms/doc-coverage-ignore.txt")


def repo_root(start):
    """Walk up from `start` until the directory holding boards/ is found."""
    path = start.resolve()
    for candidate in [path, *path.parents]:
        if (candidate / "boards").is_dir() and (candidate / "Documentation").is_dir():
            return candidate
    sys.exit(f"error: no NuttX checkout found at or above {start}")


def source_boards(root):
    """Every <arch>/<board> supported by the source tree."""
    return set(source_board_chips(root))


def source_board_chips(root):
    """<arch>/<board> -> the chip directory boards/ files it under."""
    found = {}
    for chip in (root / "boards").glob("*/*"):
        if not chip.is_dir() or chip.name in NOT_A_BOARD:
            continue
        for board in chip.iterdir():
            if board.is_dir() and board.name not in NOT_A_BOARD:
                found[f"{chip.parent.name}/{board.name}"] = chip.name
    return found


def documented_boards(root):
    """Every <arch>/<board> that has a page under Documentation/platforms/."""
    found = set()
    platforms = root / "Documentation" / "platforms"
    for page in platforms.glob("*/**/boards/*/index.rst"):
        arch = page.relative_to(platforms).parts[0]
        found.add(f"{arch}/{page.parent.name}")
    return found


def load_vendors(root):
    """Read chip-vendors.txt into (exact rules, prefix rules)."""
    exact, prefix = {}, []
    path = root / VENDOR_FILE
    if not path.is_file():
        return exact, prefix
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 2:
            sys.exit(f"{VENDOR_FILE}:{number}: expected '<family> <vendor>'")
        name, vendor = fields
        if name.endswith("*"):
            prefix.append((name[:-1], vendor))
        else:
            exact[name] = vendor
    # Longest prefix wins, so that stm32* does not shadow a longer rule.
    prefix.sort(key=lambda rule: -len(rule[0]))
    return exact, prefix


def vendor_of(family, vendors):
    exact, prefix = vendors
    if family in exact:
        return exact[family]
    for start, vendor in prefix:
        if family.startswith(start):
            return vendor
    return None


def chip_families(root):
    """The chip families of each architecture, taken from the source tree.

    arch/<arch>/src/<family>/ is what makes a family a family.  Deriving this
    from Documentation/platforms/ instead would be circular: a directory
    misspelled in the documentation would then be accepted as the correct
    spelling, which is how Documentation/platforms/tricore/tc4d9/ survived
    next to arch/tricore/src/tc4da/.
    """
    families = {}
    for arch in sorted((root / "arch").iterdir()):
        src = arch / "src"
        if src.is_dir():
            families[arch.name] = {
                d.name
                for d in src.iterdir()
                if d.is_dir() and d.name not in ("common", "cmake")
            }
    return families


def split_path(chain, known):
    """Split a documentation path into the chip families and the parts in it.

    Documentation/platforms/ may be finer grained than the source tree:
    avr/atmega/atmega2560/ documents one part of the atmega family, which has
    a single directory in arch/avr/src/.  A level that is a family is a chip,
    a level below it names a part.
    """
    chips = [name for name in chain if name in known]
    parts = [name for name in chain if name not in known]
    return chips, parts


def check_board_tags(root):
    """Check the arch: and chip: tags of every board page against its path."""
    platforms = root / "Documentation" / "platforms"
    vendors = load_vendors(root)
    families = chip_families(root)
    board_chips = source_board_chips(root)
    problems = []
    for page in sorted(platforms.glob("*/**/boards/*/index.rst")):
        parts = page.relative_to(platforms).parts
        arch = parts[0]
        known = families.get(arch, set())
        chain = list(parts[1 : parts.index("boards")])
        chips, chip_parts = split_path(chain, known)
        rel = page.relative_to(root)
        if not chips:
            problems.append(
                f"error: {rel} sits under platforms/{arch}/{chain[0]}/, which "
                f"is not a chip family: arch/{arch}/src/{chain[0]}/ does not exist"
            )
            continue
        expected = (
            [f"arch:{arch}"]
            + [f"chip:{c}" for c in chips]
            + [f"part:{c}" for c in chip_parts]
        )

        filed_under = board_chips.get(f"{arch}/{parts[parts.index('boards') + 1]}")
        if filed_under is not None and filed_under != chips[0]:
            problems.append(
                f"error: {rel} is under platforms/{arch}/{chips[0]}/, but the "
                f"source tree files this board under boards/{arch}/"
                f"{filed_under}/"
            )

        vendor = vendor_of(chips[0], vendors)
        if vendor is None:
            problems.append(
                f"error: {rel} is built around the chip family {chips[0]!r}, "
                f"which has no rule in {VENDOR_FILE}; add one saying who makes it"
            )
        elif vendor not in ("-", "?"):
            expected.append(f"vendor:{vendor}")

        text = page.read_text(encoding="utf-8", errors="surrogateescape")
        match = re.search(r"^\.\.\s+tags::\s*(.+)$", text, re.M)
        if not match:
            problems.append(
                f"error: {rel} has no '.. tags::' line; it needs at least "
                f"{', '.join(expected)}"
            )
            continue

        tags = [t.strip() for t in match.group(1).split(",") if t.strip()]
        missing = [t for t in expected if t not in tags]
        if missing:
            problems.append(f"error: {rel} is missing the tags {', '.join(missing)}")
        for tag in tags:
            if tag != tag.lower():
                problems.append(
                    f"error: {rel} has the tag {tag!r}; tags are lower case"
                )
            if ":" not in tag:
                problems.append(
                    f"error: {rel} has the tag {tag!r} with no namespace; use "
                    "arch:, chip:, vendor:, peripheral: or status:"
                )
            elif tag.split(":", 1)[0] not in NAMESPACES:
                problems.append(
                    f"error: {rel} has the tag {tag!r}; the namespace is not "
                    f"one of {', '.join(sorted(NAMESPACES))}"
                )
            elif tag.startswith("chip:") and tag[5:] not in known:
                problems.append(
                    f"error: {rel} has the tag {tag!r}, but "
                    f"arch/{arch}/src/{tag[5:]}/ does not exist; the exact "
                    "chip on the board goes in part:"
                )
            elif tag.startswith("vendor:") and tag not in expected:
                problems.append(
                    f"error: {rel} has the tag {tag!r}, which disagrees with "
                    f"{VENDOR_FILE}"
                )
    problems += check_platform_tags(root, vendors, families)
    problems += check_platform_tree(root, families)
    problems += check_vendor_rules(root, families)
    return problems


def check_platform_tree(root, families):
    """Every directory under Documentation/platforms/ must exist in the source.

    Documentation/platforms/<arch>/<family>/ mirrors arch/<arch>/src/<family>/.
    Checking it is what keeps a misspelling from quietly becoming the name
    everything else is written against.
    """
    platforms = root / "Documentation" / "platforms"
    problems = []
    for arch_dir in sorted(d for d in platforms.iterdir() if d.is_dir()):
        arch = arch_dir.name
        if arch not in families:
            problems.append(
                f"error: Documentation/platforms/{arch}/ has no counterpart in "
                f"arch/{arch}/src/"
            )
            continue
        for chip_dir in sorted(d for d in arch_dir.iterdir() if d.is_dir()):
            name = chip_dir.name
            if (
                name in NOT_A_BOARD
                or name in families[arch]
                or f"{arch}/{name}" in REMOVED_FROM_TREE
            ):
                continue
            problems.append(
                f"error: Documentation/platforms/{arch}/{name}/ is not a chip "
                f"family: arch/{arch}/src/{name}/ does not exist"
            )
    return problems


def check_vendor_rules(root, families):
    """Every rule in chip-vendors.txt must still name a chip family."""
    everything = set().union(*families.values()) if families else set()
    problems = []
    path = root / VENDOR_FILE
    if not path.is_file():
        return problems
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        name = line.split()[0]
        if name.endswith("*"):
            if any(f.startswith(name[:-1]) for f in everything):
                continue
        elif name in everything:
            continue
        problems.append(
            f"error: {VENDOR_FILE}:{number}: the rule for {name!r} matches no "
            "chip family under arch/*/src/; delete it"
        )
    return problems


def check_platform_tags(root, vendors, families):
    """Check the tags of the chip and architecture pages under platforms/.

    These pages are not required to carry tags, but the ones that do must
    agree with their path for the same reason board pages do.
    """
    platforms = root / "Documentation" / "platforms"
    boards = {p for p in platforms.glob("*/**/boards/*/index.rst")}
    problems = []
    for page in sorted(platforms.rglob("*.rst")):
        if page in boards:
            continue
        text = page.read_text(encoding="utf-8", errors="surrogateescape")
        match = re.search(r"^\.\.\s+tags::\s*(.+)$", text, re.M)
        if not match:
            continue
        parts = page.relative_to(platforms).parts[:-1]
        if not parts:
            continue
        arch, chain = parts[0], list(parts[1:])
        known = families.get(arch, set())
        chips, _ = split_path(chain, known)
        rel = page.relative_to(root)
        for tag in [t.strip() for t in match.group(1).split(",") if t.strip()]:
            namespace, _, value = tag.partition(":")
            if namespace == "arch" and value != arch:
                problems.append(
                    f"error: {rel} has {tag!r} but sits under platforms/{arch}/"
                )
            elif namespace == "chip" and value not in known:
                problems.append(
                    f"error: {rel} has {tag!r}, but arch/{arch}/src/{value}/ "
                    "does not exist; the exact chip goes in part:"
                )
            elif namespace == "vendor" and chips:
                expected = vendor_of(chips[0], vendors)
                if expected in (None, "-", "?") or tag != f"vendor:{expected}":
                    problems.append(
                        f"error: {rel} has {tag!r}, which disagrees with "
                        f"{VENDOR_FILE}"
                    )
    return problems


def read_ignore(root):
    """Known gaps, one <arch>/<board> per line; '#' starts a comment."""
    path = root / IGNORE_FILE
    if not path.is_file():
        return set()
    entries = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            entries.add(line)
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-C",
        "--directory",
        default=".",
        type=Path,
        help="path inside the NuttX checkout to check (default: .)",
    )
    parser.add_argument(
        "--list-stale",
        action="store_true",
        help="list ignore entries that are no longer needed and exit",
    )
    args = parser.parse_args()

    root = repo_root(args.directory)
    in_source = source_boards(root)
    in_docs = documented_boards(root)
    ignored = read_ignore(root)

    undocumented = sorted(in_source - in_docs - ignored)
    orphaned = sorted(in_docs - in_source - ignored)
    stale_ignores = sorted(ignored - ((in_source - in_docs) | (in_docs - in_source)))

    if args.list_stale:
        for entry in stale_ignores:
            print(entry)
        return 0

    for board in undocumented:
        arch, name = board.split("/")
        print(
            f"error: {board} is supported by boards/{arch}/*/{name}/ but has "
            f"no page under Documentation/platforms/{arch}/*/boards/{name}/"
        )
    tag_problems = check_board_tags(root)
    for problem in tag_problems:
        print(problem)

    for board in orphaned:
        arch, name = board.split("/")
        print(
            f"error: Documentation/platforms/{arch}/*/boards/{name}/ describes "
            f"a board that boards/{arch}/ does not have"
        )

    print(
        f"{len(in_source)} boards in the source tree, {len(in_docs)} documented, "
        f"{len(ignored - set(stale_ignores))} known gaps listed in {IGNORE_FILE}"
    )

    if stale_ignores:
        print(
            f"note: {len(stale_ignores)} entries in {IGNORE_FILE} are no longer "
            "needed and can be deleted:"
        )
        for entry in stale_ignores:
            print(f"  {entry}")

    if tag_problems:
        print(
            f"\n{len(tag_problems)} board pages have a tag problem.  The tags "
            "are what make the tag index a usable filter; see "
            "Documentation/contributing/doc_templates/board.rst."
        )

    if undocumented or orphaned or tag_problems:
        print(
            "\nAdd the missing page (see Documentation/contributing/doc_templates/"
            f"board.rst), or add the board to {IGNORE_FILE} if it cannot be "
            "documented yet."
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
