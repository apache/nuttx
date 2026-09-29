##############################################################################
# Documentation/_extensions/tags_overview.py
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
##############################################################################

"""Group the tag overview page by namespace instead of listing it flat.

``sphinx_tags`` writes one page per tag and one overview page listing every
tag in a single alphabetical toctree.  With a few dozen tags that is fine.
NuttX has around 180, three quarters of them in the ``chip:`` namespace, and
the flat list is unusable: the reader scrolls past 100 chip tags to find the
handful of architectures.

The tags are already namespaced -- ``arch:``, ``chip:``, ``part:``,
``vendor:`` -- so the grouping is in the data and only the presentation throws
it away.  This extension runs after ``sphinx_tags`` and
rewrites the overview it generated.

``chip:`` and ``part:`` are shown together but not mixed.  Every family is
listed once, alphabetically, so that looking one up never depends on knowing
something about it first.  Parts follow, grouped under the family whose boards
carry them -- that has to be worked out from the pages, since the name of a
part does not say which family it belongs to.  A part used by boards of two
families is listed under both, which is simply true.

Nothing here needs updating when a tag is added.
"""

import re
from collections import defaultdict
from pathlib import Path

from sphinx.application import Sphinx
from sphinx.util import logging

__version__ = "1.0.0"

logger = logging.getLogger(__name__)

# Runs after sphinx_tags, which connects at the default priority of 500.
PRIORITY = 800

# Heading shown for each namespace, in the order they appear on the page.
# Architecture and vendor come first because they are how somebody arrives
# without knowing a part number yet.
NAMESPACES = [
    ("arch", "Architecture", "The instruction set, as named under ``arch/``."),
    ("vendor", "Vendor", "Who makes the chip."),
]

CHIP_HEADING = "Chip"
CHIP_INTRO = (
    "A chip family is a directory under ``arch/<arch>/src/``, and these are "
    "all of them. The exact part fitted to a board is listed under its family."
)
FAMILY_HEADING = "Families"
FAMILY_INTRO = "Grouped by the architecture they belong to."
PART_HEADING = "Parts"
PART_INTRO = "Grouped under the family whose boards carry them."

TAG_NAME = re.compile(r"^Tags:\s*(\S.*)$", re.M)
TAG_PAGE = re.compile(r"^\s+\.\./(\S+\.rst)\s*$", re.M)
BOARD_PATH = re.compile(r"^platforms/([^/]+)/(.+?)/boards/[^/]+/index\.rst$")
PLATFORM_PATH = re.compile(r"^platforms/([^/]+)/")


def _collect(tags_dir):
    """Read what sphinx_tags generated: tag name -> (page file, page paths)."""
    tags = {}
    for path in sorted(tags_dir.glob("*.rst")):
        if path.name == "tagsindex.rst":
            continue
        text = path.read_text(encoding="utf-8")
        match = TAG_NAME.search(text)
        if not match:
            continue
        tags[match.group(1).strip()] = (path.name, TAG_PAGE.findall(text))
    return tags


def _architectures(pages):
    """The architectures the pages of one tag live under.

    A chip family is a directory under arch/<arch>/src/, so its architecture
    is the directory its pages sit in.  A handful of names exist under more
    than one architecture -- qemu under arm, arm64 and x86_64 -- and those are
    listed under each, because each really is a different chip.
    """
    found = set()
    for page in pages:
        match = PLATFORM_PATH.match(page)
        if match:
            found.add(match.group(1))
    return found


def _chip_families(pages):
    """Chip directories the pages of one tag live in.

    A board page is at platforms/<arch>/<chip>/boards/<board>/index.rst, and
    <chip> may itself be nested (avr/atmega/atmega2560); the first component
    is the family.
    """
    families = set()
    for page in pages:
        match = BOARD_PATH.match(page)
        if match:
            families.add(match.group(2).split("/")[0])
    return families


# Below this many entries a list is better off in one column: splitting three
# names across the width makes the reader's eye travel further than reading
# them straight down, and leaves a ragged second column.
COLUMN_THRESHOLD = 5


def _toctree(names, tags):
    """A toctree of the given tags, in columns only when it is worth it."""
    body = [".. toctree::", "   :maxdepth: 1", ""] + _entries(names, tags)
    if len(names) < COLUMN_THRESHOLD:
        return body + [""]
    return (
        [".. container:: tag-columns", ""]
        + [f"   {line}" if line else "" for line in body]
        + [""]
    )


def _entries(names, tags):
    """toctree lines of the form ``tag (count) <file>``, alphabetically."""
    lines = []
    for name in sorted(names):
        filename, pages = tags[name]
        lines.append(f"   {name} ({len(pages)}) <{filename}>")
    return lines


def _tree(rows):
    """Draw rows of (depth, label, count, target) as a tree.

    A map of the page, at the top of the page.  The sections below say the
    same thing over a screen and a half of headings; this says it in one,
    which is what somebody arriving here wants first.

    Every row links to the section it names, so the map is also the way to
    get there.  Padding is measured on the label the reader sees, not on the
    markup, or the columns come out ragged.
    """
    lines, depths = [], [d for d, _, _, _ in rows]
    for i, (depth, label, count, target) in enumerate(rows):
        prefix = ""
        for level in range(depth):
            more = any(
                depths[j] == level
                for j in range(i + 1, len(rows))
                if min(depths[i + 1 : j + 1] or [level]) >= level
            )
            prefix += "\u2502   " if more else "    "
        last = not any(
            depths[j] == depth
            for j in range(i + 1, len(rows))
            if min(depths[i + 1 : j + 1] or [depth]) >= depth
        )
        prefix += "\u2514\u2500\u2500 " if last else "\u251c\u2500\u2500 "

        shown = f":ref:`{label} <{target}>`" if target else label
        width = len(prefix) + len(label)
        if count is None:
            lines.append(prefix + shown)
        else:
            lines.append(prefix + shown + " " * max(1, 46 - width) + f"{count:>4}")
    return lines


def _label(kind, name):
    """A reference label for a section, so the tree can point at it.

    Explicit labels rather than '#anchor' links: Sphinx derives section ids
    from the titles, and two sections whose titles happen to match would then
    silently send the reader to the wrong one.
    """
    slug = re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-")
    return f"tag-{kind}-{slug}"


def _section(title, underline, intro=None, label=None):
    out = [f".. _{label}:", ""] if label else []
    out += [title, underline * len(title), ""]
    if intro:
        out += [intro, ""]
    return out


PARTS_SHOWN = 6


def _overview_tree(grouped, tags):
    """The map drawn at the top of the page."""
    rows = []
    for namespace, title, _ in NAMESPACES:
        names = grouped.get(namespace)
        if names:
            rows.append((0, title, len(names), _label("section", title)))

    families = grouped.get("chip") or []
    parts = grouped.get("part") or []
    if not (families or parts):
        return []

    rows.append((0, CHIP_HEADING, None, _label("section", CHIP_HEADING)))
    if families:
        by_arch = defaultdict(list)
        for name in families:
            for arch in _architectures(tags[name][1]) or ["(unknown)"]:
                by_arch[arch].append(name)
        # The count has to be what the rows below add up to, not the number of
        # distinct names: qemu is a family of arm, arm64 and x86_64, and is
        # listed under each because each is a different chip.
        rows.append(
            (
                1,
                FAMILY_HEADING,
                sum(len(v) for v in by_arch.values()),
                _label("section", FAMILY_HEADING),
            )
        )
        # Alphabetical, like the sections below.  Ordering the map by size and
        # the page by name makes the reader look twice for the same thing.
        for arch in sorted(by_arch):
            rows.append((2, arch, len(by_arch[arch]), _label("arch", arch)))

    if parts:
        # A leaf: the families underneath are reached through the family you
        # already found, and repeating 35 of them here would make the map as
        # long as the page it maps.
        by_family = defaultdict(list)
        for name in parts:
            for family in _chip_families(tags[name][1]) or ["(unknown)"]:
                by_family[family].append(name)
        rows.append(
            (
                1,
                f"{PART_HEADING}, under {len(by_family)} families",
                sum(len(v) for v in by_family.values()),
                _label("section", PART_HEADING),
            )
        )

    return (
        ["", ".. parsed-literal::", ""] + [f"   {line}" for line in _tree(rows)] + [""]
    )


def write_overview(app: Sphinx):
    tags_dir = Path(app.srcdir) / app.config.tags_output_dir
    index = tags_dir / "tagsindex.rst"
    if not index.is_file():
        return

    tags = _collect(tags_dir)
    if not tags:
        return

    grouped = defaultdict(list)
    for name in tags:
        namespace = name.split(":", 1)[0] if ":" in name else ""
        grouped[namespace].append(name)

    out = [
        ":orphan:",
        "",
        ".. _tagoverview:",
        "",
        # Puts the class on the outermost <section>, which is what
        # _static/custom.css hangs the indentation and the guide lines off.
        ".. rst-class:: tag-overview",
        "",
        "Tags",
        "####",
        "",
        "Every board page is tagged with the architecture and the chip it is "
        "built around, so that the boards can be reached by what they are "
        "rather than only by where they sit in the tree.",
        "",
    ]
    out += _overview_tree(grouped, tags)

    for namespace, title, intro in NAMESPACES:
        names = grouped.get(namespace)
        if not names:
            continue
        out += _section(title, "=", intro, _label("section", title))
        out += _toctree(names, tags)

    families = sorted(grouped.get("chip") or [])
    parts = grouped.get("part") or []
    if families or parts:
        out += _section(CHIP_HEADING, "=", CHIP_INTRO, _label("section", CHIP_HEADING))

    if families:
        # A chip tag names its family, so it needs no working out.  Grouping
        # them by architecture is what turns a run of 134 names into
        # something you can scan: nobody arrives at this page without already
        # knowing whether they are holding an Arm or a RISC-V board.
        out += _section(
            FAMILY_HEADING, "-", FAMILY_INTRO, _label("section", FAMILY_HEADING)
        )
        by_arch = defaultdict(list)
        for name in families:
            for arch in _architectures(tags[name][1]) or ["(unknown)"]:
                by_arch[arch].append(name)
        for arch in sorted(by_arch):
            out += _section(arch, "~", label=_label("arch", arch))
            out += _toctree(by_arch[arch], tags)

    if parts:
        out += _section(PART_HEADING, "-", PART_INTRO, _label("section", PART_HEADING))
        by_family = defaultdict(list)
        for name in parts:
            found = _chip_families(tags[name][1])
            for family in found or ["(unknown family)"]:
                by_family[family].append(name)
        for family in sorted(by_family):
            out += _section(family, "~")
            out += _toctree(by_family[family], tags)

    leftover = [n for ns, names in grouped.items() if not ns for n in names]
    if leftover:
        # A tag with no namespace has no place in the index; it came from
        # somewhere that does not follow the board template.
        logger.warning(
            "tags_overview: %d tags have no namespace: %s",
            len(leftover),
            ", ".join(sorted(leftover)),
        )
        out += _section("Ungrouped", "=")
        out += _toctree(leftover, tags)

    index.write_text("\n".join(out) + "\n", encoding="utf-8")
    logger.info(
        "[tags_overview] grouped %d tags into %d sections",
        len(tags),
        sum(1 for ns, _, _ in NAMESPACES if grouped.get(ns)) + bool(families or parts),
    )


def setup(app: Sphinx):
    app.connect("builder-inited", write_overview, priority=PRIORITY)
    return {
        "version": __version__,
        "parallel_read_safe": True,
        "parallel_write_safe": True,
    }
