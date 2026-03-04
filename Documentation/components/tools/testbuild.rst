================
``testbuild.sh``
================

This script automates building of a set of configurations. The intent is
simply to assure that the set of configurations build correctly. The -h
option shows the usage:

.. code:: console

   $ ./testbuild.sh -h
   USAGE: tools/testbuild.sh -h [-l|m|c|g|n] [-d] [-e <extraflags>] [-x] [-j <ncpus>] [-a <appsdir>] [-t <topdir>] [-p]
          [-A] [-C] [-G] [-N] [-R] [-S] [--codechecker] <testlist-file>

   Where:
     -h will show this help test and terminate
     -l|m|c|g|n selects Linux (l), macOS (m), Cygwin (c),
        MSYS/MSYS2 (g) or Windows native (n). Default Linux
     -d enables script debug output
     -e pass extra c/c++ flags such as -Wno-cpp via make command line
     -x exit on build failures
     -j <ncpus> passed on to make.  Default:  No -j make option.
     -a <appsdir> provides the relative path to the apps/ directory.  Default ../apps
     -t <topdir> provides the absolute path to top nuttx/ directory.  Default ../nuttx
     -p only print the list of configs without running any builds
     -A store the build executable artifact in ARTIFACTDIR (defaults to ../buildartifacts
     -C Skip tree cleanness check.
     -G Use "git clean -xfdq" instead of "make distclean" to clean the tree.
        This option may speed up the builds. However, note that:
          * This assumes that your trees are git based.
          * This assumes that only nuttx and apps repos need to be cleaned.
          * If the tree has files not managed by git, they will be removed
            as well.
     -N Use CMake with Ninja as the backend.
     -R execute "run" script in the config directories if exists.
     -S Adds the nxtmpdir folder for third-party packages.
     --codechecker enables CodeChecker statically analyze the code.
     <testlist-file> selects the list of configurations to test.  No default

   Your PATH variable must include the path to both the build tools and the
   kconfig-frontends tools

This script needs two pieces of information:

1. A description of the platform that you are testing on.  This description
   is provided by the optional -l, -m, -c, -g and -n options.

2. A list of configurations to build.  That list is provided by a test
   list file.  The final, non-optional parameter, <testlist-file>,
   provides the path to that file.

The test list file is a sequence of build descriptions, one per line.  One
build descriptions consists of two comma separated values. For example::

    stm32f429i-disco:nsh
    arduino-due:nsh
    /arm
    /risc-v

The first value is the usual configuration description of the form
``<board-name>:<configuration-name>`` or ``/<folder-name>`` and must correspond to a
configuration or folder in the nuttx/boards directory.

The second value is valid name for a toolchain configuration to use
when building the configuration.  The set of valid toolchain
configuration names depends on the underlying architecture of the
configured board.

The prefix ``-`` can be used to skip a configuration::

  -stm32f429i-disco/nsh

or skip a configuration on a specific host(e.g. Darwin)::

  -Darwin,sim:rpserver
