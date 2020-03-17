Formatting Tools
================

This file describes how to use formatting tools to format NuttX source code.

Coding Standard
---------------

See the Coding Standard for how the code should be formatted:

https://cwiki.apache.org/confluence/display/NUTTX/Coding+Standard


nxstyle
-------

This is a NuttX tool. it's located at `tools/nxstyle.c.` Run it using the `checkpatch.sh` script:

./tools/checkpatch.sh

nxstyle has some bugs that haven't been fixed; see the incubator-nuttx Github Issues for more info.


clang-format
------------

There is an in-progress experimental effort to make clang-format work for nuttx. It doesn't work well yet. Here are
instructions for Linux:

1. Install clang-format

  $ sudo apt install clang-format-9

2. Run it

  $ cd nuttx/
  $ find sched/ -iname "*.h" -or -iname "*.c" | xargs clang-format-9 -i -style=file

3. See the differences in the files

  $ git diff

4. Revert if necessary

  $ git stash; git stash drop
