=================
``checkpatch.sh``
=================

``checkpatch.sh`` is a bash script that makes use of ``nxstyle`` and
``codespell`` tools to format patches and ensure that files conform to NuttX
coding standard. It is used in NuttX's GitHub CI.

Help message:

.. code:: console

   $ tools/checkpatch.sh -h
   USAGE: tools/checkpatch.sh [options] [list|-]

   Options:
   -h
   -c spell check with codespell (install with: pip install codespell)
   -u encoding check with cvt2utf (install with: pip install cvt2utf)
   -r range check only (coupled with -p or -g)
   -p <patch file names> (default)
   -m Check commit message (coupled with -g)
   -g <commit list>
   -f <file list>
   -x format supported files (only .py, requires: pip install black)
   -  read standard input mainly used by git pre-commit hook as below:
      git diff --cached | ./tools/checkpatch.sh -
   Where a <commit list> is any syntax supported by git for specifying git revision, see GITREVISIONS(7)
   Where a <patch file names> is a space separated list of patch file names or wildcard. or *.patch
