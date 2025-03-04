#!/usr/bin/env python3
############################################################################
# tools/doreleasenotes.py
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

import argparse
import re

from github import Github
from PyInquirer import prompt
from tqdm import tqdm

prj = "apache/nuttx"


def doreleasenotes(release, gh_token):
    gh = Github(gh_token)
    gh_repo = gh.get_repo(prj)
    branches = list(gh_repo.get_branches())

    # Create the branch list
    print("Listing project branches")
    print("Branch               Release Date")
    for br in branches:
        commit = br.commit
        if br.name != "master" and re.match("releases/", br.name) is None:
            continue
        if br.name != "master":
            comparison = gh_repo.compare("master", br.commit.sha)
            commit = gh_repo.get_commit(f"{br.commit.sha}~{comparison.ahead_by}")
        commiter_date = commit.commit.committer.date
        commiter_date.replace(microsecond=0)

        print(str(br.name) + " " + str(commiter_date.isoformat()))

    # Generate ther release notes
    print("\n")
    print("Please select a range for release notes generation")
    print("Date times should be in the GitHub format as listed here:")
    print("YYYY-MM-DDTHH:MM:SS+00:00\n")

    questions = [
        {
            "type": "input",
            "name": "start",
            "message": "PR range start",
        },
        {
            "type": "input",
            "name": "end",
            "message": "PR range end",
        },
    ]
    answer = prompt(questions)

    query = f'repo:{gh_repo.full_name} is:pr is:merged merged:{answer["start"]}..{answer["end"]}'

    issues = gh.search_issues(query=query)
    print(f"Found {issues.totalCount} PRs that could be part of this release")

    prs = {}
    for pr in issues:
        prs[pr.number] = pr

    pr_nums = set(prs.keys())

    existing_pr_nums = set()
    projs = gh_repo.get_projects(state="all")
    for proj in projs:
        existing_pr_nums.add(pr.number)

    of_interest = sorted(pr_nums.difference(existing_pr_nums))
    print(f"Need to sort {len(of_interest)} PRs\n")

    print("Creating release notes")
    f = open(release + ".md", "w")
    with tqdm(
        total=len(of_interest),
        desc="Adding PRs",
        bar_format="{l_bar}{bar} [ time left: {remaining} ]",
    ) as pbar:
        for pr_id in of_interest:
            pr = prs[pr_id].as_pull_request()
            f = open(release + ".md", "a")
            f.write(
                "* [#"
                + str(pr.number)
                + "](https://github.com/apache/nuttx/pull/"
                + str(pr.number)
                + ") "
                + str(pr.title)
                + "\n"
            )
            f.close()
            pbar.update(1)


print("Simple tool to generate the NuttX release notes")
parser = argparse.ArgumentParser()
parser.add_argument(dest="release", type=str, help="Add release version")
parser.add_argument(dest="gh_token", type=str, help="Add Github Token")
args = parser.parse_args()

doreleasenotes(args.release, args.gh_token)
