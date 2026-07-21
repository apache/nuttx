#!/usr/bin/env python3
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
"""Parse ``depends-on:`` declarations from a NuttX pull request body.

Supported forms are a single reference, an inline list, or multiple declaration
lines. References may use ``owner/repo/pull/N`` or a full GitHub URL. Bullet-list
continuations are not supported.

Declarations must start a line with at most three spaces and must be outside
Markdown code blocks. Repositories are restricted to ``NUTTX_REPO`` and
``APPS_REPO``; PR numbers are positive JavaScript-safe integers.

CLI:
    python3 depends_on.py                 # print result JSON
    python3 depends_on.py --print-state   # print state used by the edit gate
    python3 depends_on.py --github-output # write workflow outputs and report
"""

from __future__ import annotations

import json
import os
import re
import sys

# Limit PR numbers to JavaScript's exact integer range because the comment
# workflow reads the report. The 19-digit regex limit also avoids huge int()
# conversions; it is not a GitHub business limit.
_MAX_SAFE_PR_NUMBER = 9007199254740991  # 2**53 - 1
_TOKEN_RE = re.compile(
    r"^(?:https://github\.com/)?"
    r"(?P<repo>[A-Za-z0-9._-]+/[A-Za-z0-9._-]+)/pull/(?P<num>[1-9][0-9]{0,18})$"
)

# CommonMark treats four leading spaces or a tab as indented code. Anchoring
# also excludes prose and prefixes such as "not-depends-on:".
_MARKER_RE = re.compile(r"^ {0,3}depends-on:[ \t]*(?P<rest>.*)$", re.IGNORECASE)

# CommonMark fences may be indented by at most three spaces.
_FENCE_RE = re.compile(r"^ {0,3}(?:```|~~~)")


def allowed_repos_from_env():
    return (
        os.environ.get("NUTTX_REPO", "apache/nuttx"),
        os.environ.get("APPS_REPO", "apache/nuttx-apps"),
    )


def _split_tokens(text):
    # GitHub Markdown does not treat Unicode separators as declaration
    # boundaries, so split tokens on ASCII separators only.
    return [t for t in re.split(r"[ \t\r\n\[\],]+", text.strip()) if t]


def _lines(body):
    """Split on newline forms rendered by GitHub Markdown."""
    return (body or "").replace("\r\n", "\n").replace("\r", "\n").split("\n")


def has_declaration(body):
    """True if the body has a line-anchored depends-on: outside code fences."""
    in_fence = False
    for ln in _lines(body):
        if _FENCE_RE.match(ln):
            in_fence = not in_fence
            continue
        if not in_fence and _MARKER_RE.match(ln):
            return True
    return False


def _declared_tokens(body):
    """Yield tokens from single-line declarations outside code fences."""
    in_fence = False
    for ln in _lines(body):
        if _FENCE_RE.match(ln):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        m = _MARKER_RE.match(ln)
        if m:
            for t in _split_tokens(m.group("rest")):
                yield t


def parse_dependencies(body, allowed_repos=None, pr_number=None, head_sha=None):
    """Return the structured dependency result for ``body``."""
    if allowed_repos is None:
        allowed_repos = allowed_repos_from_env()

    deps = []
    warnings = []
    seen = set()
    for t in _declared_tokens(body):
        m = _TOKEN_RE.match(t)
        if not m:
            continue  # stray text on a declaration line; ignore
        repo = m.group("repo")
        num = int(m.group("num"))
        if num > _MAX_SAFE_PR_NUMBER:
            continue  # beyond JS safe integer; comment workflow can't handle it
        if repo not in allowed_repos:
            warnings.append("Ignoring unsupported dependency repo: " + repo)
            continue
        key = (repo, num)
        if key not in seen:
            seen.add(key)
            deps.append({"repo": repo, "number": num})

    has_decl = not deps and has_declaration(body)
    if has_decl:
        warnings.append(
            "Found a 'depends-on:' line but no valid dependency was parsed. "
            "Declare dependencies on the same line as 'depends-on:', e.g. "
            "depends-on: [{}/pull/<N> {}/pull/<M>]".format(*allowed_repos)
        )

    return {
        "version": 1,
        "pr_number": pr_number,
        "head_sha": head_sha,
        "status": "ok" if deps else "invalid" if has_decl else "none",
        "dependencies": deps,
        "warnings": warnings,
    }


def dep_ref(dep):
    return "{}/pull/{}".format(dep["repo"], dep["number"])


def _int_or_none(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def main(argv):
    body = os.environ.get("PR_BODY", "")
    result = parse_dependencies(
        body,
        pr_number=_int_or_none(os.environ.get("PR_NUMBER")),
        head_sha=os.environ.get("HEAD_SHA") or None,
    )
    refs = [dep_ref(d) for d in result["dependencies"]]

    if "--print-state" in argv:
        print(result["status"])
        for r in refs:
            print(r)
        return 0

    if "--github-output" in argv:
        for w in result["warnings"]:
            print("::warning::" + w)
        out_lines = [
            "depends_on=" + " ".join(refs),
            "status=" + result["status"],
        ]
        gh_out = os.environ.get("GITHUB_OUTPUT")
        if gh_out:
            with open(gh_out, "a", encoding="utf-8") as f:
                f.write("\n".join(out_lines) + "\n")
        else:
            print("\n".join(out_lines))
        # Write the report only when a depends-on declaration is present
        # (status ok/invalid).  status=none means there is nothing to report and
        # nothing to comment; existing (historical) comments are left untouched.
        report_path = os.environ.get("REPORT_PATH")
        if report_path and result["status"] != "none":
            parent = os.path.dirname(report_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(report_path, "w", encoding="utf-8") as f:
                json.dump(result, f)
        return 0

    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
