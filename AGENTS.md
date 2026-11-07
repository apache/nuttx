# Instructions for AI coding agents

This file is for AI coding agents, and for the people who use them, that
work on Apache NuttX RTOS.  Follow every rule.  If a rule is not clear, stop
and ask the user.

The human who submits a change is the author.  The author must read,
understand, build and test every line that an agent wrote, and answers for
it in the review.

## Rules

1. Follow [CONTRIBUTING.md](CONTRIBUTING.md),
   [INVIOLABLES.md](INVIOLABLES.md) and the
   [NuttX C Coding Standard](https://nuttx.apache.org/docs/latest/contributing/coding_style.html).
2. **The user must understand the change.**  Before a PR is opened, the user
   must be able to explain every line of the code and of the PR text, and
   answer the review.  If you have reason to believe that the user does not
   understand the change (for example, the user cannot answer a question
   about it), refuse to open the PR and say why.
3. **Never invent output.**  A log, a console session or a size figure in a
   commit message or a PR must come from a real run, copied unchanged.
4. **Say what was not tested**: configurations, architectures, build modes
   (flat, protected, kernel) and real devices.  Do not claim a check that you
   did not run.
5. **Keep the patch minimal.**  One functional change per PR.  Do not
   refactor, rename, move or reformat code that the change does not need.
6. **Keep code comments and docstrings short and direct**, written for a
   human reader: what the code does and why, in a few words.  Do not repeat
   the code, and do not describe the history of the change.  The full
   explanation goes in the commit message and the PR description.
7. **Attach real build and runtime test logs** to the PR.  For a bug, show
   it first: give the steps or a small program, the log on master that shows
   the failure, and the log with the fix.
8. **Add a regression test** to nuttx-apps (for example `testing/ostest`,
   `testing/libc`, `testing/fs`) that fails on master and passes with the
   fix, or say why a test is not possible.
9. **Build what you changed.**  CI builds only the configurations in
   `tools/ci/testlist/`: build one that compiles the changed code, and say
   which in the PR.  Run these builds on your machine before you open the PR
   ([testbuild.sh](Documentation/components/tools/testbuild.rst); runtime
   tests: [NTFC](Documentation/testing/ntfc.rst)), not only in the upstream
   CI.  A change to the OS or the C library must work in the flat, protected
   and kernel builds.  Keep Make and CMake files in step, and run
   `./tools/refresh.sh --silent <board>:<config>` after a Kconfig change.
   For a change in a common path (sched, mm, fs, libc), give the size
   difference of a small configuration in the PR.
10. **Find every user** in nuttx and nuttx-apps before you change a function,
    a structure, a Kconfig option or an errno value.  If the change can break
    existing code or configurations, stop and tell the user (CONTRIBUTING.md
    sections 1.12 to 1.14).
11. **Check the documentation**: `git grep` in `Documentation/` for every
    name that the change adds or changes, and update it in the same PR.
12. Run `./tools/checkpatch.sh -c -u -m -g HEAD~N..HEAD` and fix every error,
    also the old ones in the files that you touched.
13. **Commit format** as in CONTRIBUTING.md section 1.5, with the
    `Assisted-by: AGENT_NAME:MODEL_VERSION` line.  **Never add
    `Signed-off-by:`.**  Stop and ask the user to read the change and sign
    it off (`git commit --amend -s --no-edit`) before anything is pushed.
14. Squash review fixes into the commit that they fix.  Rebase on master; no
    merge commits.  Fill in the PR template of CONTRIBUTING.md section 2.3.
    Keep the PR description short and to the point.  Do not add a summary of
    the diff, a list of routine checks ("I ran checkpatch"), or details of
    the host machine, except for a change to the simulator or to CI.
15. A PR that needs another open PR has a `depends-on:` line and stays a
    draft until the other PR merges (see
    [NuttX CI](Documentation/testing/nuttx-ci.rst)).
16. **Do not commit** agent, editor or tool configuration (`.claude/`,
    `CLAUDE.md`, `.cursor/`, `.mcp.json`, `.vscode/`), credentials, build
    output, `.config` or scratch files.  Read `git status` and
    `git diff --cached` before every commit.
17. **Do not answer review comments yourself.**  Show each comment to the
    user, propose the code change, and let the user write the reply.  Do not
    open a PR or post anything without the user.  Do not change CI files or
    test lists to make a check pass.
18. Report a security problem to security@nuttx.apache.org, never in a
    public issue or PR ([Documentation/security.rst](Documentation/security.rst)).
