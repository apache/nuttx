.. include:: /substitutions.rst
.. _making-changes:

Making Changes Using Git
========================

The Apache NuttX project uses the `Git version control system <https://git-scm.com/book/en/v2/Getting-Started-About-Version-Control>`_
to track changes, and the source code is hosted on `GitHub <https://www.github.com>`_.

If you want to make changes to NuttX, for your own personal use, or to submit them back to project to improve NuttX,
that's easy. For the purposes of this guide, you'll need a `GitHub <https://www.github.com>`_ account, since
the Apache NuttX team uses GitHub. (You could also use git locally, or save your changes to other sites like
`GitLab <https://about.gitlab.com/>`_ or `BitBucket <https://bitbucket.org>`_, but that's beyond the scope of this
guide).

Here's how to do it:

#. Set your git user name and email

    .. code-block:: bash

       $ cd nuttx/
       $ git config --global user.name "Your Name"
       $ git config --global user.email "yourname@somedomaincom"

#. Sign in to GitHub

   If you don't have a `GitHub <https://www.github.com>`_ account, it's free to
   sign up.

#. Fork the Projects

   Visit both these links and hit the Fork button in the upper right of the page:

   * `NuttX <https://github.com/apache/incubator-nuttx/>`_
   * `NuttX Apps <https://github.com/apache/incubator-nuttx-apps/>`_

#. Clone the Repositories

   On the GitHub web page for your forked ``incubator-nuttx`` project, copy the clone url – get it by hitting the
   green ``Clone or Download`` button in the upper right. Then do this:

    .. code-block:: bash

       $ git clone <your forked incubator-nuttx project clone url> nuttx
       $ cd nuttx
       $ git remote add upstream https://github.com/apache/incubator-nuttx.git

   Do the same for your forked ``incubator-nuttx-apps`` project:

    .. code-block:: bash

       $ cd ..
       $ git clone <your forked incubator-nuttx-apps project clone url> apps
       $ cd apps
       $ git remote add upstream https://github.com/apache/incubator-nuttx-apps.git

#. Create a Local Git Branch

   Now you can create local git branches and push them to GitHub:

    .. code-block:: bash

       $ git checkout -b test/my-new-branch
       $ git push

Git Workflow With an Upstream Repository
----------------------------------------

The main NuttX git repository is called an "upstream" repository - this is because it's the main source of truth, and
its changes flow downstream to people who've forked that repository, like us.

Working with an upstream repo is a bit more complex, but it's worth it since you can submit fixes and features
to the main NuttX repos. One of the things you need to do regularly is keep your local repo in sync
with the upstream. I work with a local branch, make changes, pull new software from the upstream and merge it in,
maybe doing that several times. Then when everything works, I get my branch ready to do a Pull Request. Here's how:

#. Fetch upstream changes and merge into my local master:

    .. code-block:: bash

       $ git checkout master
       $ git fetch upstream
       $ git merge upstream/master
       $ git push

#. Merge my local master with my local branch:

    .. code-block:: bash

       $ git checkout my-local-branch
       $ git merge master
       $ git push

#. Make changes and push them to my fork

    .. code-block:: bash

       $ vim my-file.c
       $ git add my-file.c
       $ git commit my-file.c
       $ git push

#. Repeat 1-3 as necessary

#. Run the ``tools/checkpatch.sh`` script on your files

   When your code runs, then you're almost ready to submit it. But first you need to check the code to ensure
   that it conforms to the NuttX :ref:`contributing/coding_style:C Coding Standard`.
   The ``tools/checkpatch.sh`` script will do that. Here's the usage info:

    .. code-block:: bash

       $ ./tools/checkpatch.sh -h
       USAGE: ./tools/checkpatch.sh [options] [list|-]

       Options:
       -h
       -c spell check with codespell(install with: pip install codespell
       -r range check only (used with -p and -g)
       -p <patch list> (default)
       -g <commit list>
       -f <file list>
       -  read standard input mainly used by git pre-commit hook as below:
          git diff --cached | ./tools/checkpatch.sh -

   Run it against your files and correct all the errors in the code you added, so that
   ``tools/checkpatch.sh`` reports no errors. Then commit the result.
   For example:

    .. code-block:: bash

       $ ./tools/checkpatch.sh -f my-file.c
       arch/arm/src/sama5/hardware/my-file.c:876:82: warning: Long line found
       $ # fix errors
       $ vim my-file.c
       $ # run again
       $ ./tools/checkpatch.sh -f my-file.c

   If you have made a lot of changes, you can also use this bash commandline to see the errors for all the changed C
   files in your branch (assumes you are currently on the branch that has the changed files):

    .. code-block:: bash

       $ git diff --name-only master | egrep "\.c|\.h" | xargs echo | xargs ./tools/checkpatch.sh -f | less

   Note that there are some bugs in the ``nxstyle`` program that ``checkpatch.sh`` uses, so
   it may report a few errors that are not actually errors. The committers will help you
   find these. (Or view the
   `nxstyle Issues <https://github.com/apache/incubator-nuttx/issues?q=is%3Aissue+is%3Aopen+nxstyle>`_.)
   |br|
   |br|

#. Commit the fixed files

    .. code-block:: bash

       $ git add my-file.c
       $ git commit my-file.c
       $ git push

Submitting Your Changes to NuttX
--------------------------------

  Pull requests let you tell others about changes you've pushed to a branch in a repository on GitHub. Once a pull
  request is opened, you can discuss and review the potential changes with collaborators and add follow-up commits
  before your changes are merged into the base branch.

  (from GitHub's `About pull requests <https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/about-pull-requests>`_ page)

Before you do a Pull Request, the NuttX team will usually want all the changes you made in your branch "squashed" into
a single commit, so that when they review your changes, there's a clean view of the history. If there are changes
after Pull Request review feedback, they can be separate commits. Here's the easiest way I found to do that initial
squash before submitting the Pull Request:

#. Check out my branch

    .. code-block:: bash

       $ git checkout my-branch

#. Fetch the upstream code

    .. code-block:: bash

       $ git fetch upstream

#. Rebase onto the upstream code

    .. code-block:: bash

       $ git rebase upstream/master

#. Push to your remote

   This needs to a force push with ``-f``.

    .. code-block:: bash

       $ git push -u my-branch -f

#. Create a GitHub Pull Request

   A Pull Request is how you ask your upstream to review and merge your changes.

   Here's `GitHub's instructions for creating a Pull Request <https://help.github.com/en/github/collaborating-with-issues-and-pull-requests/creating-a-pull-request>`_.

#. Get Pull Request feedback and implement changes

   Get suggestions for improvements from reviewers, make changes, and push them to the branch. Once the reviewers are
   happy, they may suggest squashing and merging again to make a single commit. In this case you would repeat steps
   1 through 6.

Git Resources
-------------

* `Git Cheat Sheet (by GitHub) <https://github.github.com/training-kit/downloads/github-git-cheat-sheet.pdf>`_
* `Git Book (online) <https://git-scm.com/book/en/v2>`_
* `NuttX Code Contribution Workflow (draft) <https://cwiki.apache.org/confluence/display/NUTTX/Code+Contribution+Workflow>`_
  – All the details are here if you need them!
