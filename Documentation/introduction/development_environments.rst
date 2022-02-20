.. todo:: revise and update links

========================
Development Environments
========================

Linux + GNU ``make`` + GCC/binutils for Linux
=============================================

The is the most natural development environment for NuttX. Any version
of the GCC/binutils toolchain may be used. There is a highly modified
`buildroot <http://buildroot.uclibc.org/>`__ available for download from
the `NuttX
Bitbucket.org <https://bitbucket.org/nuttx/buildroot/downloads/>`__
page. This download may be used to build a NuttX-compatible ELF
toolchain under Linux or Cygwin. That toolchain will support ARM, m68k,
m68hc11, m68hc12, and SuperH ports. The buildroot GIT may be accessed in
the NuttX `buildroot GIT <https://bitbucket.org/nuttx/buildroot>`__.

Linux + GNU ``make`` + SDCC for Linux
=====================================

Also very usable is the Linux environment using the
`SDCC <http://sdcc.sourceforge.net/>`__ compiler. The SDCC compiler
provides support for the 8051/2, z80, hc08, and other microcontrollers.
The SDCC-based logic is less well exercised and you will likely find
some compilation issues if you use parts of NuttX with SDCC that have
not been well-tested.

Windows with Cygwin + GNU ``make`` + GCC/binutils (custom built under Cygwin)
=============================================================================

This combination works well too. It works just as well as the native
Linux environment except that compilation and build times are a little
longer. The custom NuttX
`buildroot <https://bitbucket.org/nuttx/buildroot/downloads/>`__
referenced above may be build in the Cygwin environment as well.

Windows with Cygwin + GNU ``make`` + SDCC (custom built under Cygwin)
=====================================================================

I have never tried this combination, but it would probably work just
fine.

Windows with Cygwin + GNU ``make`` + Windows Native Toolchain
=============================================================

This is a tougher environment. In this case, the Windows native
toolchain is unaware of the Cygwin *sandbox* and, instead, operates in
the native Windows environment. The primary difficulties with this are:

-  **Paths**. Full paths for the native toolchain must follow Windows
   standards. For example, the path ``/home/my\ name/nuttx/include`` my
   have to be converted to something like
   ``'C:\cygwin\home\my name\nuttx\include'`` to be usable by the
   toolchain.
-  **Symbolic Links** NuttX depends on symbolic links to install
   platform-specific directories in the build system. On Linux, true
   symbolic links are used. On Cygwin, emulated symbolic links are used.
   Unfortunately, for native Windows applications that operate outside
   of the Cygwin *sandbox*, these symbolic links cannot be used.
-  **Dependencies** NuttX uses the GCC compiler's ``-M`` option to
   generate make dependencies. These dependencies are retained in files
   called ``Make.deps`` throughout the system. For compilers other than
   GCC, there is no support for making dependencies in this way.

**Supported Windows Native Toolchains**. At present, the following
Windows native toolchains are in use:

#. GCC built for Windows (such as CodeSourcery, Atollic, devkitARM,
   etc.),
#. SDCC built for Windows,
#. the ZiLOG XDS-II toolchain for Z16F, z8Encore, and eZ80Acclaim parts.

Windows Native (``CMD.exe``) + GNUWin32 (including GNU ``make``) + MinGW Host GCC compiler + Windows Native Toolchain
=====================================================================================================================

Build support has been added to support building natively in a Windows
console rather than in a POSIX-like environment.

This build:

#. Uses all Windows style paths
#. Uses primarily Windows batch commands from cmd.exe, with
#. A few extensions from GNUWin32

This capability first appeared in NuttX-6.24 and should still be
considered a work in progress because: (1) it has not been verfied on
all targets and tools, and (2) still lacks some of the creature-comforts
of the more mature environments. The windows native build logic
initiated if ``CONFIG_WINDOWS_NATIVE=y`` is defined in the NuttX
configuration file:

At present, this build environment also requires:

**Windows Console**. The build must be performed in a Windows console
window. This may be using the standard ``CMD.exe`` terminal that comes
with Windows. I prefer the ConEmu terminal which can be downloaded from:
http://code.google.com/p/conemu-maximus5/

**GNUWin32**. The build still relies on some Unix-like commands. I
usethe GNUWin32 tools that can be downloaded from
http://gnuwin32.sourceforge.net/. See the top-level ``nuttx/README.txt``
file for some download, build, and installation notes.

**MinGW-GCC**. MinGW-GCC is used to compiler the C tools in the
``nuttx/tools`` directory that are needed by the build. MinGW-GCC can be
downloaded from http://www.mingw.org/. If you are using GNUWin32, then
it is recommended that you not install the optional MSYS components as
there may be conflicts.

Wine + GNU ``make`` + Windows Native Toolchain
==============================================

I've never tried this one, but I off the following reported by an ez80
user using the ZiLOG ZDS-II Windows-native toolchain:

   "I've installed ZDS-II 5.1.1 (IDE for ez80-based boards) on wine
   (windows emulator for UNIX) and to my surprise, not many changes were
   needed to make GIT snapshot of NuttX buildable... I've tried nsh
   profile and build process completed successfully. One remark is
   necessary: NuttX makefiles for ez80 are referencing ``cygpath``
   utility. Wine provides similar thing called ``winepath`` which is
   compatible and offers compatible syntax. To use that, ``winepath``
   (which itself is a shell script) has to be copied as ``cygpath``
   somewhere in ``$PATH``, and edited as in following patch:

   "Better solution would be replacing all ``cygpath`` references in
   ``Makefiles`` with ``$(CONVPATH)`` (or ``${CONVPATH}`` in shell
   scripts) and setting ``CONVPATH`` to ``cygpath`` or ``winepath``
   regarding to currently used environment.

Other Environments
==================

**Environment Dependencies**. The primary environmental dependency of
NuttX are (1) GNU make, (2) bash scripting, and (3) Linux utilities
(such as cat, sed, etc.). If you have other platforms that support GNU
make or make utilities that are compatible with GNU make, then it is
very likely that NuttX would work in that environment as well (with some
porting effort). If GNU make is not supported, then some significant
modification of the Make system would be required.

**MSYS**. I have not used MSYS but what I gather from talking with NuttX
users is that MSYS can be used as an alternative to Cygwin in any of the
above Cygwin environments. This is not surprising since MSYS is based on
an older version of Cygwin (cygwin-1.3). MSYS has been modified,
however, to interoperate in the Windows environment better than Cygwin
and that may be of value to some users.

MSYS, however, cannot be used with the native Windows NuttX build
because it will invoke the MSYS bash shell instead of the ``CMD.exe``
shell. Use GNUWin32 in the native Windows build environment.
