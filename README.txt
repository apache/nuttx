README
^^^^^^

  o Installation
  o Configuring NuttX
  o Toolchains
  o Building NuttX
  o Documentation

INSTALLATION
^^^^^^^^^^^^

Download and Unpack:

  Download and unpack the NuttX tarball.  If you are reading this, then
  you have probably already done that.  After unpacking, you will end
  up with a directory called nuttx-version (where version is the NuttX
  version number). You might want to rename that directory nuttx to
  match the various instructions in the documentation and some scripts
  in the source tree.

Install Directories with Spaces in the Path

  The nuttx build directory should reside in a path that contains no
  spaces in any higher level directory name.  For example, under
  Cygwin, your home directory might be formed from your first and last
  names like: "/home/First Last". That will cause strange errors when
  the make system tries to build.

  [Actually, that problem is probably not to difficult to fix.  Some
   Makefiles probably just need some pathes within double quotes]i

  I work around spaces in the home directory name, by creating a
  new directory that does not contain any spaces, such as /home/nuttx.
  Then I install NuttX in /home/nuttx and always build from 
  /home/nuttx/nuttx.

A Note about Header Files:

  Some toolchains are built with header files extracted from a C-library
  distribution (such as newlib).  For those toolchains, NuttX must be
  compiled without using the standard header files that are distributed
  with your toolchain.  This prevents including conflicting, incompatible
  header files (such as stdio.h).

  Certain header files, such as setjmp.h and varargs.h, may still be
  needed from your toolchain, however.  If that is the case, one solution
  is to copy those header file from your toolchain into the NuttX include
  directory.

  Also, if you prefer to use the stdint.h and stdbool.h header files from
  your toolchain, those could be copied into the include/ directory too.
  Using most other header files from your toolchain would probably cause
  errors.

CONFIGURING NUTTX
^^^^^^^^^^^^^^^^^

"Canned" NuttX configuration files are retained in:

  configs/<board-name>/<config-dir>

Where <board-name> is the name of your development board and <config-dir>.
Configuring NuttX requires only copying three files from the <config-dir>
to the directly where you installed NuttX (TOPDIR):

  Copy configs/<board-name>/<config-dir>/Make.def to ${TOPDIR}/Make.defs
    Make.defs describes the rules needed by you tool chain to compile
    and link code.  You may need to modify this file to match the
    specific needs of your toolchain.

  Copy configs/<board-name>/<config-dir>/setenv.sh to ${TOPDIR}/setenv.sh
    setenv.sh is an optional convenience file that I use to set
    the PATH variable to the toolchain binaries.  You may chose to
    use setenv.sh or not.  If you use it, then it may need to be
    modified to include the path to your toolchain binaries.

  Copy configs/<board-name>/<config-dir>/defconfig to ${TOPDIR}/.config
    The defconfig file holds the actual build configuration.  This
    file is included by all other make files to determine what is
    included in the build and what is not.  This file is also used
    to generate a C configuration header at include/nuttx/config.h.

General information about configuring NuttX can be found in:

  ${TOPDIR}/configs/README.txt
  ${TOPDIR}/configs/<board-name>/README.txt

There is a configuration script in the tools/ directory that makes this
easier.  It is used as follows:

  cd ${TOPDIR}/tools
  ./configure.sh <board-name>/<config-dir>

TOOLCHAINS
^^^^^^^^^^

Cross-Development Toolchains

  In order to build NuttX for your board, you will have to obtain a cross-
  compiler to generate code for your target CPU.  For each board,
  configuration, there is a README.txt file (at configs/<board-name>/README.txt).
  That README file contains suggestions and information about appropriate
  tools and development environments for use with your board.

  In any case, the script, setenv.sh that was deposited in the top-
  level directory when NuttX was configured should be edited to set
  the path to where you installed the toolchain.  The use of setenv.sh
  is optional but can save a lot of confusion in the future.

NuttX Buildroot Toolchain

  For many configurations, a DIY set of tools is available for NuttX.  These
  tools can be downloaded from the NuttX SourceForge file repository.  After
  unpacking the buildroot tarball, you can find instructions for building
  the tools in the buildroot/configs/README.txt file.

  Check the README.txt file in the configuration director for your board
  to see if you can use the buildroot toolchain with your board (this
  README.txt file is located in configs/<board-name>/README.txt).

  This toolchain is available for both the Linux and Cygwin development
  environments.

BUILDING NUTTX
^^^^^^^^^^^^^^

NuttX builds in-place in the source tree.  You do not need to create
any special build directories.  Assuming that your Make.defs is setup
properly for your tool chain and that setenv.sh contains the path to where
your cross-development tools are installed, the following steps are all that
are equired to build NuttX:

  cd ${TOPDIR}
  . ./setenv.sh
  make

At least one configuration (eagle100) requires additional command line
arguments on the make command.  Read ${TOPDIR}/configs/<board-name>/README.txt
to see if that applies to your target.

CYGWIN BUILD PROBLEMS
^^^^^^^^^^^^^^^^^^^^^

If you see strange behaviour when building under Cygwin then you may have
a problem with your PATH variable.  For example, if you see failures to
locate files that are clearly present, that may mean that you are using
the wrong version of a tool.  For example, you may not be using Cywgin's
'make' program at /usr/bin/make.  Try:

    $ which make
    /usr/bin/make

When you install some toolchains (such as Yargarto or CodeSourcery tools),
they may modify your PATH variable to include a path to their binaries.
At that location, they make have GNUWin32 versions of the tools.  So you
might actually be using a version of make that does not understand Cygwin
pathes.

The solution is either:

1. Edit your PATH to remove the path to the GNUWin32 tools, or
2. Put /usr/local/bin, /usr/bin, and /bin at the front of your path:

   $ export PATH=/usr/local/bin:/usr/bin:/bin:$PATH

DOCUMENTATION
^^^^^^^^^^^^^

Additional information can be found in the Documentation/ directory and
also in README files that are scattered throughout the source tree.  The
documentation is in HTML and can be access by loading the following file
into your Web browser:

  Documentation/index.html

NuttX documentation is also available online at http://www.nuttx.org.

Below is a guide to the available README files in the NuttX source tree:

 |
 |- arch/
 |   |
 |   |- arm
 |   |   `- src
 |   |       `- lpc214x/README.txt
 |   |- sh/
 |   |   |- include/
 |   |   |   |-m16c/README.txt
 |   |   |   |-sh1/README.txt
 |   |   |   `-README.txt
 |   |   |- src/
 |   |   |   |-common/README.txt
 |   |   |   |-m16c/README.txt
 |   |   |   |-sh1/README.txt
 |   |   |   `-README.txt
 |   `- z80/
 |   |   `- src/
 |   |       `- z80/README.txt
 |   `- README.txt
 |- configs/
 |   |- avr32dev1/
 |   |   `- README.txt
 |   |- c5471evm/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- demo0s12ne64/
 |   |   `- README.txt
 |   |- ea3131/
 |   |   `- README.txt
 |   |- eagle100/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- ez80f910200kitg/
 |   |   |- ostest/README.txt
 |   |   `- README.txt
 |   |- ez80f910200zco/
 |   |   |- dhcpd/README.txt
 |   |   |- httpd/README.txt
 |   |   |- nettest/README.txt
 |   |   |- nsh/README.txt
 |   |   |- ostest/README.txt
 |   |   |- poll/README.txt
 |   |   `- README.txt
 |   |- lm3s5965-ek/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- m68332evb/
 |   |   |- include/README.txt
 |   |   `- src/README.txt
 |   |- mbed/
 |   |   `- README.txt
 |   |- mcu123-lpc214x/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- mx1ads/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- ntosd-dm320/
 |   |   |- doc/README.txt
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- nucleus1g/
 |   |   `- README.txt
 |   |- olimex-lpc17xx/
 |   |   `- README.txt
 |   |- olimex-lpc2378/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- olimex-strp711/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- pjrc-8051/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- sam3u-ek/
 |   |   `- README.txt
 |   |- sim/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- skp16c26/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- stm3210e-eval/
 |   |   |- include/README.txt
 |   |   |- RIDE/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- us7032evb1/
 |   |   |- bin/README.txt
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- xtrs/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- z16f2800100zcog/
 |   |   |- ostest/README.txt
 |   |   |- pashello/README.txt
 |   |   `- README.txt
 |   |- z80sim/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- z8encore000zco/
 |   |   |- ostest/README.txt
 |   |   `- README.txt
 |   |- z8f64200100kit/
 |   |   |- ostest/README.txt
 |   |   `- README.txt
 |   `- README.txt
 |- drivers/
 |   `- README.txt
 |- examples/
 |   |- nsh/README.txt
 |   |- pashello/README.txt
 |   `- README.txt
 |- graphics/
 |   `- README.txt
 |- libxx/
 |   `- README.txt
 |- netutils/
 |   |- telnetd/README.txt
 |   `- README
 `- tools/
     `- README.txt
