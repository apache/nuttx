====================
``nxstyle_sweep.sh``
====================

Run ``nxstyle`` over whole directories, or over the repository, and collect
what it reports. Usage:

.. code:: console

   $ tools/nxstyle_sweep.sh [options] [<dir> ...]

     -o <file>   Diagnostics                (default: nxstyle-errors.txt)
     -f <file>   Failing files, worst first (default: nxstyle-files.txt)
     -l          List failing files instead of writing the reports
     -s          Summarise the diagnostics by message
     -b <path>   Use an existing nxstyle binary
     -j <n>      Number of parallel checks
     -a          Check every file, not only those tracked by git
     -h          Show this help

Options must precede the directories, which default to the whole repository
and are relative to its top. The exit status is non-zero if anything was
reported.

See also ``nxstyle``.
