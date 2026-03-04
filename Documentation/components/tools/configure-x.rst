======================================================================================
``configure.sh``, ``configure.bat``, ``configure.c``, ``cfgparser.c``, ``cfgparser.h``
======================================================================================

``configure.sh`` is a bash script that is used to configure NuttX for a given
target board in a environment that supports POSIX paths (Linux, Cygwin, macOS,
or similar).  See :doc:`/components/boards` or
Documentation/NuttXPortingGuide.html for a description of how to configure NuttX
with this script.

configure.c, cfgparser.c, and cfgparser.h can be used to build a work-alike
program as a replacement for configure.sh.  This work-alike program would be
used in environments that do not support Bash scripting (such as the Windows
native environment).

configure.bat is a small Windows batch file that can be used as a replacement
for configure.sh in a Windows native environment.  configure.bat is actually
just a thin layer that executes configure.exe if it is available. If
configure.exe is not available, then configure.bat will attempt to build it
first.

In order to build configure.exe from configure.c in the Windows native
environment, two assumptions are made:

1) You have installed the MinGW GCC toolchain.  This toolchain can be
   downloaded from http://www.mingw.org/.  It is recommended that you not
   install the optional MSYS components as there may be conflicts.
2) That path to the bin/ directory containing mingw-gcc.exe must be
   included in the PATH variable.
