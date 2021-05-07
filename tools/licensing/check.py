#!/usr/bin/env python3

############################################################################
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

import getopt
import json
import os
import re
import subprocess
import sys

import termcolor

committers_json = None
non_commiters_json = None
author_mappings_json = None

verbose_level = 0
color = True


def colored(s, c):
    if color:
        return termcolor.colored(s, c)
    else:
        return s


def commit_attributions(c):
    regex = re.compile("(?i)(?:by|from|author|Co-authored-by):? +(.+)")
    return re.findall(regex, c["message"]) + re.findall(regex, c["body"])


def get_headers(s):
    return re.findall("(?i)/\*\*\*.+?(?:Copyright).+?\*\*\*+/", s, re.DOTALL)


def get_file(blob):
    try:
        return subprocess.check_output(
            ["git", "cat-file", "-p", blob], stderr=subprocess.DEVNULL
        ).decode()
    except subprocess.CalledProcessError:
        return None


def header_authors(header):
    results = re.findall("[Aa]uthors?: +(.+?) *(?:Redistribution)", header, re.DOTALL)
    results = [re.split("\n[ *]+", result) for result in results]
    results = sum(results, [])  # flatten
    results = [
        re.sub("[Cc]opyright:?( ?.[Cc].)? *([12][0-9]{3}[,-]? ?)", "", result)
        for result in results
    ]
    results = list(filter(lambda s: s != "", results))  # remove empty strings
    return results


# Search for an author name in Apache's committers/non-committers
# database. It will return (apacheID,name) if there's a match or
# None if not. apacheID might be None if there's no Apache ID
# for author


def search_for_cla(name):
    for k, v in committers_json["committers"].items():
        if v == name:
            return (k, v)

    if name in non_committers_json["non_committers"]:
        return (None, name)

    return None


# Returns the same as above, but this takes an author
# (which may include an email include an email used
# to look for alternative author names for this person)


def author_has_cla(author):
    if "@" in author:
        matches = re.match("^(.+?)(?: +([^ ]+@[^ ]+ *))$", author)
        if not matches:
            return None  # found an '@' but it wasn't an email, so this is most likely not really an author
        name = matches.group(1)
        email = matches.group(2).lstrip("<").rstrip(">")
    else:
        name = author.strip()
        email = None

    vvvprint("name: %s email: %s" % (name, email if email else "?"))

    # first look for name directly

    result = search_for_cla(name)
    if result:
        return result

    # otherwise, get all available alternative names for author
    # and look for each

    if email and (email in author_mappings_json):
        result = search_for_cla(author_mappings_json[email])
        if result:
            return result

    # Nothing matched

    return None


def header_copyrights(header):
    results = re.findall(
        " \* *[Cc]opyright:?(?: ?.[Cc].)? *(?:[12][0-9]{3}[,-]? ?)* *(.+)", header
    )
    return [re.sub("(. )?[Aa]ll rights reserved.?", "", result) for result in results]


def report_cla(author):
    cla = author_has_cla(author)
    if cla:
        (apacheid, name) = cla
        print(colored("✓", "green"), end=" ")
    else:
        apacheid = None
        print(colored("✗", "red"), end=" ")

    if apacheid:
        print("%s (ID: %s)" % (author, apacheid))
    else:
        print(author)


def analyze(j):
    complete_attributions = set()
    complete_authors = set()
    complete_copyrights = set()

    vprint("file has %i commits" % len(j))
    for commit in j:
        authors = set()

        vprint(colored("-", "yellow"))
        vprint(colored("commit: ", "green") + commit["commit"])
        vprint(colored("blob: ", "green") + commit["blob"])
        vprint(colored("date: ", "green") + commit["date"])
        vprint(
            colored("author: ", "green")
            + ("%s <%s>" % (commit["author"], commit["author-email"]))
        )

        attributions = commit_attributions(commit)
        if len(attributions) > 0:
            vprint(colored("attributions:", "green"))
            for attribution in attributions:
                vprint(attribution)

        complete_attributions |= set(attributions)
        complete_authors |= set([commit["author"] + " " + commit["author-email"]])

        # skip deletion commits

        vprint(colored("blob:", "green"), end=" ")
        if commit["blob"] == "0000000000000000000000000000000000000000":
            vprint("zero (deletion)")
            continue

        file_contents = get_file(commit["blob"])

        # skip inaccessible blobs (probably lived in a submodule)

        if not file_contents:
            vprint("inaccessible")
            continue
        else:
            vprint("available")

        headers = get_headers(file_contents)

        vprint(colored("header authors:", "green"))
        for header in headers:
            ha = header_authors(header)
            authors |= set(ha)
            vprint(ha)

        complete_authors |= set(authors)

        vprint(colored("header copyrights:", "green"))
        copyrights = set()
        for header in headers:
            hc = header_copyrights(header)
            copyrights |= set(hc)
            vprint(hc)

        vprint(colored("commit description:", "green"))
        vprint(commit["message"])

        if commit["body"]:
            vprint(colored("commit msg body:", "green"))
            vprint(commit["body"])

        vvprint(colored("headers:", "green"))
        for header in headers:
            vvprint(header)

        complete_copyrights |= copyrights

    vprint(colored("----\n", "yellow"))

    print(colored("COMPLETE REPORT:", "blue"))
    print(colored("attributions:", "green"))
    if len(complete_attributions) == 0:
        print("*none detected*")
    else:
        for attribution in complete_attributions:
            report_cla(attribution)

    print(colored("authors:", "green"))
    for author in complete_authors:
        report_cla(author)

    print(colored("copyrights:", "green"))
    print("\n".join(complete_copyrights))


def print_help():
    print("Usage: check.py [-v] [-n] <JSON file>\n")
    print(
        "  -v\tIncrease verbosity (add up to three times)\n"
        "  -n\tDo not use color for output"
    )


def vprint(*args, **kwargs):
    if verbose_level > 0:
        print(*args, **kwargs)


def vvprint(*args, **kwargs):
    if verbose_level > 1:
        print(*args, **kwargs)


def vvvprint(*args, **kwargs):
    if verbose_level > 2:
        print(*args, **kwargs)


#####

# First try to load the CLAs JSONs:

try:
    with open(
        os.path.dirname(os.path.abspath(__file__)) + "/icla-info.json", "r"
    ) as file:
        committers_json = json.load(file)

    with open(
        os.path.dirname(os.path.abspath(__file__)) + "/icla-info_noid.json", "r"
    ) as file:
        non_committers_json = json.load(file)
except Exception:
    print(
        "Could not open CLA JSON files, please read README.md for download instructions"
    )
    sys.exit(2)

# Open author mappings JSON

with open(
    os.path.dirname(os.path.abspath(__file__)) + "/author_mappings.json", "r"
) as file:
    author_mappings_json = json.load(file)

try:
    opts, args = getopt.getopt(sys.argv[1:], "hnv")
except getopt.GetoptError:
    print_help()
    sys.exit(2)
for opt, arg in opts:
    if opt == "-h":
        print_help()
        sys.exit()
    elif opt == "-v":
        verbose_level = verbose_level + 1
    elif opt == "-n":
        color = False

if len(args) != 1:
    print_help()
    sys.exit(2)

f = args[0]

if not f:
    print_help()
    sys.exit(2)

if f == "-":
    j = json.load(sys.stdin)
else:
    with open(f, "r") as file:
        j = json.load(file)

analyze(j)
