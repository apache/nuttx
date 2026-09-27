##############################################################################
# Documentation/_extensions/review_highlight.py
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

"""Highlight prose that was written for the reorganisation, for review.

This is a review aid and is not meant to ship.  It marks the text that is
new -- as opposed to text that was moved from elsewhere in the
documentation -- so that a reviewer can tell the two apart at a glance
instead of reading every page against git.

Two ways to mark something:

  * list a document in REVIEW_PAGES and the whole page is highlighted;
  * list "docname#section-id" in REVIEW_SECTIONS and only that section is;
  * wrap the block in ".. container:: review-authored" in the source, for
    prose that is not a whole section -- a lead paragraph or a note.

Delete this file, its entry in conf.py's extensions list and the
.review-authored rules in _static/custom.css to switch it all off.
"""

from docutils import nodes
from sphinx.application import Sphinx

__version__ = "1.0.0"

#: Pages written from scratch.
REVIEW_PAGES = {
    "os/index",
    "os/scheduling/index",
    "os/ipc/index",
    "os/time/index",
    "os/interrupts/index",
    "os/video",
    "os/openamp",
    "os/libs/libbuiltin",
    "os/libs/libm",
    "os/filesystem/romfs",
    "os/filesystem/aio",
    "os/filesystem/userfs",
    "os/drivers/character/bch",
    "os/drivers/special/pipes",
    "os/drivers/special/clk",
    "os/drivers/special/usrsock",
    "os/drivers/special/rwbuffer",
    "os/drivers/special/devicetree",
    "os/drivers/character/nullzero",
    "os/drivers/character/ipcc",
    "os/drivers/character/loop",
    "os/drivers/character/efuse",
    "os/filesystem/nfs",
    "os/concurrency/index",
    # Section and front-matter pages whose prose is entirely new.  guides/index
    # keeps only its title from the page it replaced; components/index,
    # implementation/index and reference/index were retitled and rewritten down
    # to the toctree; the last three are new files.
    "guides/index",
    "components/index",
    "implementation/index",
    "reference/index",
    "ReleaseNotes/index",
    "about/index",
    "developing/index",
}

#: Sections added to a page that already existed.
REVIEW_SECTIONS = {
    # nuttx_tasking and memory/index mark their added blocks inline with
    # ".. container:: review-authored", because only part of each section is
    # new and marking the whole section would claim text that was not.
    "os/memory/index": {"memory-layout-and-paging"},
    "os/memory/shm": {"architecture-interface"},
    "os/memory/paging": {"application-notes"},
    "os/drivers/character/leds/index": {"architecture-interface"},
    "os/arch/index": {"architecture-interface"},
    "os/drivers/index": {"how-drivers-work"},
    "os/filesystem/index": {"files-and-permissions"},
    "os/libs/index": {"algorithms"},
    "os/networking/index": {"congestion-control"},
    "os/drivers/special/power/index": {"design"},
    # The home page keeps the upstream paragraph that introduces NuttX, so the
    # four sections below it are marked rather than the whole page.
    "index": {
        "where-to-start",
        "understanding-the-system",
        "working-on-nuttx",
        "how-this-documentation-is-organised",
    },
    "os/scheduling/smp": {"architecture-interface"},
    "platforms/x86_64/intel64/index": {"supported-boards"},
}

CLASS = "review-authored"


def mark(app: Sphinx, doctree, docname: str) -> None:
    if docname in REVIEW_PAGES:
        for node in doctree.children:
            if isinstance(node, nodes.section):
                node["classes"] = node.get("classes", []) + [CLASS]
        return

    wanted = REVIEW_SECTIONS.get(docname)
    if not wanted:
        return
    for node in doctree.findall(nodes.section):
        if wanted.intersection(node.get("ids", [])):
            node["classes"] = node.get("classes", []) + [CLASS]


def setup(app: Sphinx):
    app.connect("doctree-resolved", mark)
    return {
        "version": __version__,
        "parallel_read_safe": True,
        "parallel_write_safe": True,
    }
