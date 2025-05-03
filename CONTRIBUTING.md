# Apache NuttX RTOS Contributing Guidelines

Thank you for your interest in contributing to Apache NuttX RTOS :-)

If you haven't yet read
[The Inviolable Principles of NuttX]( https://github.com/apache/nuttx/blob/master/INVIOLABLES.md)
please do so first.

Please note that NuttX supports over 15 different architectures, 360+ boards,
and 1600+ configurations, that are used in various projects and products around
the world. Remember that any code change may affect different users and their
businesses. Please try to keep your contributions high quality and compatible.

To help us process your contributions smoothly, it is very important that you
follow the guidelines. NuttX is maintained by a small team of volunteers from
around the world. These rules are based on daily experiences and help us
to preserve long term self-compatibility and maintenance of the project.

This document is split into two main parts:

 1. Contribution Rules.
 2. Contribution Templates.

If you need more information please read the
[Full Contributing Documentation](https://nuttx.apache.org/docs/latest/contributing/index.html)
or ask questions at our [Social Media channels](https://nuttx.apache.org/community).




## 1. Contribution Rules.

### 1.1. Goals.

The rules presented are here to ensure high quality code and documentation,
standardized pull requests processing, as well as long term self-compatibility
and maintenance of the project.

Because every change may affect users, products, or services around the world,
all rules apply equally to all authors, reviewers, committers and maintainers.

This is our Check-List for processing every incoming pull request.
Also, we filter out breaking changes and handle them accordingly.


### 1.2. Requirements are mandatory.

Each Contribution is a Pull Request (PR) that consists of one or more git
commits (COMMIT). Both PRs and COMMITs **must** adhere to requirements
presented in the Contributing Guidelines or will be auto-rejected
until fixed. Special cases are possible and outlined in a separate section
of dedicated rules.


### 1.3. Proper change description.

Proper description of change is mandatory. Description **must** contain
detailed explanation on:

 * what is the purpose of change
 * why it is necessary
 * what it does / adds / fixes
 * how things are added / changed / fixed / updated
 * what is the impact (build / runtime / api / what area)
 * how it was tested (build host, compiler, target, logs)
 * references (i.e. `nuttx` and `nuttx-apps` changes).
 * dependencies (if change depends on another change).

Local code build and real world hardware runtime test logs
**must** be provided (for code related changes), at least short form.
For fixes, "before" and "after" logs comparison is welcome.

Description can be a single sentence, several sentences long, or bullet
points, but must be enough for anyone to understand change goals and details.
Usually it will look similar for PR and git commit message.

Good description when read once will make others understand your work.
This allows faster review, quick understanding of change history (what
changed, why, how), and eventual problem identification / fix in future.


### 1.4. COMMIT and PR descriptions are equally important.

Git commit messages are as important as PR descriptions.

Git log provides offline descriptions of each change that is
git client / interface independent.


### 1.5. COMMIT requirements.

Proper git commit message according to template (see 2.1) is **mandatory**,
or change is auto-rejected until fixed. Build and runtime logs are
optional here if these are too long and already provided in the PR.

Git commit message consists of:

 1. Topic with functional area prefix, `:` mark, short self-explanatory
    functional context, `.` mark.
 2. Blank line.
 3. Description on what is changed, how, and why. May use several
    lines, short sentences, or bullet points (see 1.3).
 4. Blank line.
 5. Signature (created with `git commit -s`).

Valid git commit example:

```
net/can: Add g_ prefix to can_dlc_to_len and len_to_can_dlc.

Add g_ prefix to can_dlc_to_len and len_to_can_dlc to
follow NuttX coding style conventions for global symbols,
improving code readability and maintainability.
* you can also use bullet points.
* to note different things briefly.

Signed-off-by: AuthorName <Valid@EmailAddress>
```


### 1.6. COMMIT message mandatory fields.

Each git commit message **must** contain these fields, or change is
auto-rejected until fixed:

 1. topic.
 2. description.
 3. signature (`git commit -s`).

Although this seems to repeat rule 1.5, it clearly filters out commits
with no topic, description, or signature.


### 1.7. PR requirements.

1. Proper description (see 1.3) of PR according to template (see 2.3)
   is **mandatory**. All fields are required or change is auto-rejected
   until fixed.
2. For code changes build and runtime logs are **mandatory** to prove code
   was tested on **at least one** real world hardware target.
3. Pull Requests should be as small as possible and focused on only one
   functional change.
4. Changes affecting different functionalities must be provided in a separate
   Pull Requests.
5. PR may contain several commits but every single commit included
   must not break overall build, runtime, and compatibility,
   especially for other components.
6. PR that breaks build or runtime in any way is considered a Breaking Change
   (see 1.12), is not welcome, and requires special handling (see 1.13).
7. PR that introduces new feature must have Documentation included (see 1.8).
8. Work-in-progress PRs should be marked `[WIP]` tag and set as "draft".
9. When changes for a dedicated function must be bundled together in order
   to maintain functionality and self-compatibility, exception can
   be made, and it must be clearly stated that there is no other way and
   that this is not a Breaking Change.

Note that our small team of maintainers is processing hundreds of PRs every
month. Providing high quality COMMITs and PRs really helps a lot!

Please provide only finished and verified solutions. Every PR code update
(`git push -f` to your branch that represents the PR) each time triggers big
GitHub CI machinery that builds / cross-compiles your code to various targets
on various build hosts. This is not only slow but also expensive.


### 1.8. Documentation requirements.

Changes **must** come with a documentation update (where applicable).

Documentation is part of the `nuttx` git repository. If code change is part
of `nuttx-apps` repository then separate PR in `nuttx` repository is required.
Otherwise documentation should come in the same PR as the code update.

It is advised that the code and documentation should be split into two
separate commits in the same PR. This helps backporting and separates
possible code and documentation build errors. Squashing code together with
documentation into a single commit should be avoided, but is acceptable.

If change presents new functionality documentation **must** be provided
in the same PR along with the code (not in the future).

If change requires existing documentation update it must be contained
in the same PR along with the code change (not in the future).

Successful documentation build logs (shortcut) are welcome.

This helps us keep documentation in sync with the code.

See:
1. https://github.com/apache/nuttx/tree/master/Documentation
2. https://github.com/apache/nuttx/blob/master/INVIOLABLES.md


### 1.9. Zero Trust approach to user testing.

We implement zero trust approach to user-provided testing.

It is the change author's duty to provide real world hardware build and runtime
logs for **at least one** real world hardware device.

Remember that any code change may break things for others.
Please do your utmost to avoid that.

This helps us filter out untested code commits.

See: https://github.com/apache/nuttx/blob/master/INVIOLABLES.md


### 1.10. Long term maintenance and self-compatibility.

Long term maintenance and self-compatibility is our ultimate goal.

Alternative solutions and non-invasive approaches are preferred. Offer
user a choice and compatibility instead of breaking existing features.

Breaking changes are avoided and planned towards next major release (see 1.12).

See: https://github.com/apache/nuttx/blob/master/INVIOLABLES.md


### 1.11. Experimental features.

Experimental code and incomplete or experimental features that do not impact
overall project self-compatibility in terms of Breaking Changes (see 1.12)
**must** be clearly marked with a `[EXPERIMENTAL]` tag in the git commit topic
and PR title that will propagate to Release Notes.

See: https://github.com/apache/nuttx/blob/master/INVIOLABLES.md


### 1.12. Breaking changes not welcome.

Breaking changes are not welcome. **We do not "break by design"**. Please
provide only high quality code and think about other NuttX users.

Because thousands of users / companies and their projects / products depend
on NuttX code, we strongly prefer self-compatibility and long-term maintenance
over "change is good" or "enforced changes" ideologies.

"Breaking change" is anything that alters Build / Kernel / Architecture / API,
alters both nuttx and nuttx-apps repo at the same time, breaks
build / runtime / api for a single or many boards / architectures /
applications, breaks self-compatibility, breaks maintenance,
breaks build / runtime compatibility with existing release code (packages)
both for nuttx and nuttx-apps, etc.

Remember that any code change may impact other users and their businesses!

Changes should provide maximum self-compatibility with existing solutions,
should not impact build and runtime in a negative way, or may require design
reconsiderations such as providing alternative non-invasive user selectable
solutions (see 1.10).

When breaking changes are unavoidable, these may be accepted, but need prior
discussion and agreement of the community and special handling (see 1.13).

See: https://github.com/apache/nuttx/blob/master/INVIOLABLES.md


### 1.13. Breaking changes handling process.

This rule complements "Breaking changes not welcome" rule.

We avoid breaking changes unless absolutely necessary and unavoidable
(i.e. security fix, broken code, etc). Special case considerations apply:

 1. First reviewer that recognizes a breaking change should block
    accidental merge with "Request Changes" mark and ask for discussion.
    This PR cannot be merged until all requests are resolved.
 2. PR is marked as "Draft" to avoid accidental merge.
 3. Detailed discussion should follow both in PR **and**
    [dev@ Mailing List](https://nuttx.apache.org/community/#mailing-list).
 4. Alternative non-breaking solution is researched alongside the community.
 5. Breaking change after discussion / updates is voted on the mailing
    list and requires at least 4 +1 binding votes with no -1 binding votes.
    Any -1 binding vote is a
    [veto](https://apache.org/foundation/voting.html#Veto) and blocks
    the change. NuttX PMC members cast binding votes.
    See [Apache Voting Process](https://apache.org/foundation/voting.html).
 6. Breaking changes **must** be verified on **various different real
    world hardware architectures**, build and runtime logs are
    **mandatory**. Help of the community is welcome.
 7. Breaking change requires at least 4 independent positive PR reviews
    (see 1.16), all discussions resolved, and zero "request changes".
 8. Change must be well documented (build / runtime test logs, pr, git
    commit, documentation, release notes, etc) with clear notes on how to
    fix the introduced problems.
 9. Breaking Change must be clearly marked with a `[BREAKING]` tag in the
    git commit topic and PR title that will propagate to Release Notes.

See: https://github.com/apache/nuttx/blob/master/INVIOLABLES.md


### 1.14. Breaking changes build and runtime test logs are mandatory.

Breaking changes are special case where build and runtime test logs
(i.e. `apps/ostest`) for **more than one** different architecture is
**mandatory**. Community support is welcome.

QEMU tests **do not count** here as in the past they did not catch problems
that revealed on a real world hardware after change was already merged.

Although this seems to repeat rule 1.13, it ensures real world hardware
verification and minimizes possible negative impact on various users.

See: https://github.com/apache/nuttx/blob/master/INVIOLABLES.md


### 1.15. Review requirements.

Before PR can be merged to the master repository it requires:

 * 2 or more independent positive reviews.
 * 4 or more independent positive reviews for Breaking Changes (see 1.12).

In future we plan to add "areas" (such as trivial typo fixes or documentation
only updates) that may require only one reviewer.


### 1.16. Reviews independence.

PR reviews should come from independent organizations.
NuttX is an Open-Source independent and unbiased project.

Each PMC Member, Committer, and Reviewer must report up-to-date Affiliation
for clear identification. Commits should contain business email where
applicable.

When code comes from the same organization as positive review, then review
is not considered independent, but still may provide helpful insight.

Self review is not allowed.

See: https://github.com/apache/nuttx/blob/master/INVIOLABLES.md


### 1.17. Merge rules.

1. Each change **must** be provided as PR that undergoes independent review
   process (see 1.16).
2. Single company / organization commit, review, and merge is not allowed.
3. Merge of PRs with unresolved discussions and "change request" marks
   is not allowed.
3. Self committed code merge with or without review is not allowed.
4. Direct push to master is not allowed.

Breaking these rules will be punished.

See: https://github.com/apache/nuttx/blob/master/INVIOLABLES.md




## 2. Contribution Templates.

### 2.1. Source Code.

Your source code **must** conform to the [NuttX C Coding Standard](https://nuttx.apache.org/docs/latest/contributing/coding_style.html).

Code is automatically checked for each PR (and PR updates) by GitHub Continuous
Integration (CI) system. Fails at `lint/*` step indicate code style errors.

You may verify the code styling issues with:

 1. `./tools/checkpatch.sh -g HEAD~...HEAD`
 2. `./tools/checkpatch.sh -f path/to/your/file.c`

Note that we require you to solve encountered issues and adapt all modified
files even if you did not introduce the problem yourself. This way, every
contribution gets us closer to compliance. Thank you!


### 2.2. Commits.

This template provides git commit example as described in requirement 1.5.

```
net/can: Add g_ prefix to can_dlc_to_len and len_to_can_dlc.

Add g_ prefix to can_dlc_to_len and len_to_can_dlc to
follow NuttX coding style conventions for global symbols,
improving code readability and maintainability.
* you can also use bullet points.
* to note different thing briefly.

Signed-off-by: AuthorName <Valid@EmailAddress>
```

Note that first line of the commit message (topic) will be automatically
used as pull request title and the rest is added as description.
Use it as a starting point to describe your Pull-Request (PR).


### 2.3. Pull Requests.

This template uses MarkDown and provides example Pull Request content
as described in requirement 1.7.

````
## Summary

  * Why change is necessary (fix, update, new feature)?
  * What functional part of the code is being changed?
  * How does the change exactly work (what will change and how)?
  * Related [NuttX Issue](https://github.com/apache/nuttx/issues) reference if applicable.
  * Related NuttX Apps [Issue](https://github.com/apache/nuttx-apps/issues) / [Pull Request](https://github.com/apache/nuttx-apps/pulls) reference if applicable.

## Impact

  * Is new feature added? Is existing feature changed? NO / YES (please describe if yes).
  * Impact on user (will user need to adapt to change)? NO / YES (please describe if yes).
  * Impact on build (will build process change)? NO / YES (please describe if yes).
  * Impact on hardware (will arch(s) / board(s) / driver(s) change)? NO / YES (please describe if yes).
  * Impact on documentation (is update required / provided)? NO / YES (please describe if yes).
  * Impact on security (any sort of implications)? NO / YES (please describe if yes).
  * Impact on compatibility (backward/forward/interoperability)? NO / YES (please describe if yes).
  * Anything else to consider or add?

## Testing

  I confirm that changes are verified on local setup and works as intended:
  * Build Host(s): OS (Linux,BSD,macOS,Windows,..), CPU(Intel,AMD,ARM), compiler(GCC,CLANG,version), etc.
  * Target(s): arch(sim,RISC-V,ARM,..), board:config, etc.
  * you can also provide steps on how to reproduce the problem and verify the change.

  Testing logs before change:

  ```
  build and runtime logs before change goes here
  ```

  Testing logs after change:

  ```
  build and runtime logs after change goes here
  ```

## PR verification Self-Check

  * [ ] This PR introduces only one functional change.
  * [ ] I have updated all required description fields above.
  * [ ] My PR adheres to Contributing [Guidelines](https://github.com/apache/nuttx/blob/master/CONTRIBUTING.md) and [Documentation](https://nuttx.apache.org/docs/latest/contributing/index.html) (git commit title and message, coding standard, etc).
  * [ ] My PR is still work in progress (not ready for review).
  * [ ] My PR is ready for review and can be safely merged into a codebase.
````
