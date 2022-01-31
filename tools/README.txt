tools/README.txt
================

This README file addresses the contents of the NuttX tools/ directory.

The tools/ directory contains miscellaneous scripts and host C programs
that are necessary parts of the NuttX build system.  These files
include:

cmpconfig.c
-----------

  This C file can be used to build a utility for comparing two NuttX
  configuration files.

Config.mk
---------

  Config.mk contains common definitions used by many configuration files.
  This file (along with <nuttx>/.config) must be included at the top of
  each configuration-specific Make.defs file like:

    include $(TOPDIR)/.config
    include $(TOPDIR)/tools/Config.mk

  Subsequent logic within the configuration-specific Make.defs file may then
  override these default definitions as necessary.

checkpatch.sh
-------------
  checkpatch.sh is a bash script that make use of nxstyle and codespell tools
  to format patches and files conform to NuttX coding standard. For example,
  it has been used in NuttX github action PR check build.

  $ tools/checkpatch.sh -h
  USAGE: ./tools/checkpatch.sh [options] [list|-]

  Options:
  -h
  -c spell check with codespell(install with: pip install codespell)
  -r range check only (coupled with -p or -g)
  -p <patch list> (default)
  -g <commit list>
  -f <file list>
  -  read standard input mainly used by git pre-commit hook as below:
     git diff --cached | ./tools/checkpatch.sh -
  Where a <commit list> is any syntax supported by git for specifying git revision, see GITREVISIONS(7)
  Where a <patch file names> is a space separated list of patch file names or wildcard. or *.patch

configure.sh
configure.bat
configure.c, cfgparser.c, and cfgparser.h
------------

  configure.sh is a bash script that is used to configure NuttX for a given
  target board in a environment that supports POSIX paths (Linux, Cygwin,
  macOS, or similar).  See boards/README.txt or Documentation/NuttXPortingGuide.html
  for a description of how to configure NuttX with this script.

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

convert-comments.c
------------------

  Convert C++-style comments to C89 C-style comments.  Usage:

    convert-comments <source-file> <out-file>

detab.c
-------

  Convert tabs to spaces in a file.  Usage:

    detab [-4] <source-file> <out-file>

  Default <source-file> tab size is 8 spaces;  -4 selects 4 space tab size.

discover.py
-----------

  Example script for discovering devices in the local network.
  It is the counter part to apps/netutils/discover

gencromfs.c
-----------

  This is a C program that is used to generate CROMFS file system images.
  Usage is simple:

    gencromfs <dir-path> <out-file>

  Where:

    <dir-path> is the path to the directory will be at the root of the
      new CROMFS file system image.
    <out-file> the name of the generated, output C file.  This file must
      be compiled in order to generate the binary CROMFS file system
      image.

initialconfig.c
---------------

  This is a C file that can be used to create an initial configuration.
  This permits creating a new configuration from scratch, without
  relying on any existing board configuration in place.  This utility
  will create a barebones .config file sufficient only for
  instantiating the symbolic links necessary to do a real configuration.

kconfig2html.c
--------------

  This is a C file that can be used to build a utility for converting the
  NuttX configuration in the Kconfig files to an HTML document.  This
  auto-generated documentation will, eventually, replace the manually
  updated configuration documentation that is falling woefully behind.

  $ tools/kconfig2html.exe -h
  USAGE: tools/kconfig2html [-d] [-a <apps directory>] {-o <out file>] [<Kconfig root>]
         tools/kconfig2html [-h]

  Where:

    -a : Select relative path to the apps/ directory. This path is relative
         to the <Kconfig directory>.  Default: ../apps
    -o : Send output to <out file>.  Default: Output goes to stdout
    -d : Enable debug output
    -h : Prints this message and exits
    <Kconfig root> is the directory containing the root Kconfig file.
         Default <Kconfig directory>: .

  NOTE: In order to use this tool, some configuration must be in-place with
  all necessary symbolic links.  You can establish the configured symbolic
  links with:

    make context

  or more quickly with:

    make .dirlinks

Libraries.mk, FlatLibs.mk, ProtectedLibs.mk, and KernelLib.mk
-------------------------------------------------------------

  Libraries.mk has the build rules for all NuttX libraries.

  FlatLibs.mk, ProtectedLibs.mk, and KernelLib.mk:  These control the
  selection of libraries to be built, depending on the selected build mode.

lowhex.c
--------

  Convert hexadecimal representation in a file from upper- to lower-case.
  Usage:

    lowhex <source-file> <out-file>

Makefile.[unix|win]
-----------------

  Unix.mk is the Makefile used when building NuttX in Unix-like systems.
  It is selected from the top-level Makefile.

  Win.mk is the Makefile used when building natively under Windows.
  It is selected from the top-level Makefile.

mkconfig.c, cfgdefine.c, and cfgdefine.h
----------------------------------------

  These are C files that are used to build mkconfig program.  The mkconfig
  program is used during the initial NuttX build.

  When you configure NuttX, you will copy a configuration file called .config
  in the top level NuttX directory (See boards/README.txt or
  Documentation/NuttXPortingGuide.html).  The first time you make NuttX,
  the top-level makefile will build the mkconfig executable from mkconfig.c
  (using Makefile.host).  The top-level Makefile will then execute the mkconfig
  program to convert the .config file in the top level directory into
  include/nuttx/config.h.  config.h is a another version of the NuttX
  configuration that can be included by C files.

mkconfigvars.sh
---------------

  The HTML documentation expects to have a copy of the auto-generated
  configuration variable documentation Documentation/NuttXConfigVariables.html.
  The script mkconfigvars.sh is a simple script that can be used to
  re-generated that file as needed.

  $ tools/mkconfigvars.sh -h
  tools/mkconfigvars.sh is a tool for generation of configuration variable documentation

  USAGE: tools/mkconfigvars.sh [-d|h] [-v <major.minor.patch>]

  Where:
    -v <major.minor.patch>
       The NuttX version number expressed as a major, minor and patch number separated
       by a period
    -d
       Enable script debug
    -h
       show this help message and exit

mkexport.sh and Export.mk
-------------------------------

  These implement part of the top-level Makefile's 'export' target.  That
  target will bundle up all of the NuttX libraries, header files, and the
  startup object into an export-able, binary NuttX distribution.  The
  Export.mk is used only by the mkexport.sh script to parse out options
  from the top-level Make.defs file.

  USAGE: tools/mkexport.sh [-d] [-z] [-u] -t <top-dir> [-x <lib-ext>] -l "lib1 [lib2 [lib3 ...]]"

  This script also depends on the environment variable MAKE which is set
  in the top-level Makefile before starting mkexport.sh.  If MAKE is not
  defined, the script will set it to `which make`.

mkfsdata.pl
-----------

  This perl script is used to build the "fake" file system and CGI support
  as needed for the apps/netutils/webserver.  It is currently used only
  by the Makefile at apps/examples/uip.  That example serves as an example
  of how to configure the uIP webserver "fake" file system.

  NOTE:  This perl script comes from uIP and was (probably) written
  by Adam Dunkels.  uIP has a license that is compatible with NuttX.

mkversion.c, cfgdefine.c, and cfgdefine.h
-----------------------------------------

  This is C file that is used to build mkversion program.  The mkversion
  program is used during the initial NuttX build.

  When you build NuttX there should be a version file called .version in
  the top level NuttX directory (See Documentation/NuttXPortingGuide.html).
  The first time you make NuttX, the top-level makefile will build the
  mkversion executable from mkversion.c (using Makefile.host).  The top-level
  Makefile will then execute the mkversion program to convert the
  .version file in the top level directory into include/nuttx/version.h.
  version.h provides version information that can be included by C files.

mksyscall.c, cvsparser.c, and cvsparser.h
-----------------------------------------

  This is a C file that is used to build mksyscall program.  The mksyscall
  program is used during the initial NuttX build by the logic in the top-
  level syscall/ directory.

  If you build NuttX as a separately compiled, monolithic kernel and separate
  applications, then there is a syscall layer that is used to get from the
  user application space to the NuttX kernel space.  In the user application
  "proxies" for each of the kernel functions are provided.  The proxies have
  the same function signature as the kernel function, but only execute a
  system call.

  Within the kernel, there are "stubs" for each of the system calls.  The
  stubs receive the marshalled system call data, and perform the actually
  kernel function call (in kernel-mode) on behalf of the proxy function.

  Information about the stubs and proxies is maintained in a comma separated
  value (CSV) file in the syscall/ directory.  The mksyscall program will
  accept this CVS file as input and generate all of the required proxy or
  stub files as output.  See syscall/README.txt for additional information.

mksymtab.c, cvsparser.c, and cvsparser.h
----------------------------------------

  This is a C file that is used to build symbol tables from comma separated
  value (CSV) files.  This tool is not used during the NuttX build, but
  can be used as needed to generate files.

  USAGE: ./mksymtab [-d] <cvs-file> <symtab-file> [<symtab-name> [<nsymbols-name>]]

  Where:

    <cvs-file>      : The path to the input CSV file (required)
    <symtab-file>   : The path to the output symbol table file (required)
    <symtab-name>   : Optional name for the symbol table variable
                      Default: "g_symtab"
    <nsymbols-name> : Optional name for the symbol table variable
                      Default: "g_nsymbols"
    -d              : Enable debug output

  Example:

    cd nuttx/tools
    cat ../syscall/syscall.csv ../lib/libc.csv | sort >tmp.csv
    ./mksymtab.exe tmp.csv tmp.c

mkctags.sh
----------

  A script for creating ctags from Ken Pettit.  See http://en.wikipedia.org/wiki/Ctags
  and http://ctags.sourceforge.net/

nxstyle.c
---------

  I am embarrassed that this is here.  This program is a complete hack
  but, unfortunately, it has become so useful to me that I need to keep
  it here.

  A little background:  I have tinkered with pretty printers for some
  time and have not been happy with the results.  An alternative that
  occurred to me would be just a standard checker that examines a C
  file that gives warnings for violations of the coding standard.

  This turns out to be more difficult that you might think. A pretty
  printer understands C syntax:  They break the file up into its C
  components then reassembles the output in the format. But parsing the
  C loses the original file layout and so it not useful in this case.

  This program instead, uses a collection of heuristics (i.e., hacks and
  bandaids) to examine the C file for obvious violations of the coding
  standard.  This program is completely ignorant of C syntax; it simply
  performs crude pattern matching to check the file.

  Prints formatted messages that are classified as info, warn, error,
  fatal. In a parsable format that can be used by editors and IDEs.

  Usage: nxstyle [-m <excess>] [-v <level>] [-r <start,count>] <filename>
         nxstyle -h this help
         nxstyle -v <level> where level is
                    0 - no output
                    1 - PASS/FAIL
                    2 - output each line (default)

  See also indent.sh and uncrustify.cfg

pic32mx
-------

  This directory contains build tools used only for PIC32MX/Z platforms

bdf-convert.c
-------------

  This C file is used to build the bdf-converter program.  The bdf-converter
  program can be used to convert fonts in Bitmap Distribution Format (BDF)
  into fonts that can be used in the NX graphics system.

  Below are general instructions for creating and installing a new font
  in the NX graphic system:

    1. Locate a font in BDF format,
    2. Use the bdf-converter program to convert the BDF font to the NuttX
       font format.  This will result in a C header file containing
       definitions.  That header file should be installed at, for example,
       libnx/nxfonts/nxfonts_myfont.h.

  Create a new NuttX configuration variable.  For example, suppose
  you define the following variable:  CONFIG_NXFONT_MYFONT.  Then
  you would need to:

    3. Define CONFIG_NXFONT_MYFONT=y in your NuttX configuration file.

  A font ID number has to be assigned for each new font.  The font ID
  is defined in the file include/nuttx/nx/nxfonts.h.  Those definitions
  have to be extended to support your new font.  Look at how the font ID
  enabled by CONFIG_NXFONT_SANS23X27 is defined and add an ID for your
  new font in a similar fashion:

    4. include/nuttx/nx/nxfonts.h. Add your new font as a possible system
       default font:

       #if defined(CONFIG_NXFONT_SANS23X27)
       # define NXFONT_DEFAULT FONTID_SANS23X27
       #elif defined(CONFIG_NXFONT_MYFONT)
       # define NXFONT_DEFAULT FONTID_MYFONT
       #endif

       Then define the actual font ID.  Make sure that the font ID value
       is unique:

       enum nx_fontid_e
       {
         FONTID_DEFAULT     = 0      /* The default font */
       #ifdef CONFIG_NXFONT_SANS23X27
         , FONTID_SANS23X27 = 1      /* The 23x27 sans serif font */
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
         , FONTID_MYFONT    = 2      /* My shiny, new font */
       #endif
       ...

  Now add the font to the NX build system.  There are several files that
  you have to modify to do this.  Look how the build system uses the
  font CONFIG_NXFONT_SANS23X27 for examples:

    5. nuttx/graphics/Makefile.  This file needs logic to auto-generate
       a C source file from the header file that you generated with the
       the bdf-converter program.  Notice NXFONTS_FONTID=2; this must be
       set to the same font ID value that you defined in the
       include/nuttx/nx/nxfonts.h file.

       genfontsources:
         ifeq ($(CONFIG_NXFONT_SANS23X27),y)
          @$(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=1 EXTRAFLAGS=$(EXTRAFLAGS)
        endif
         ifeq ($(CONFIG_NXFONT_MYFONT),y)
          @$(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=2 EXTRAFLAGS=$(EXTRAFLAGS)
        endif

    6. nuttx/libnx/nxfonts/Make.defs.  Set the make variable NXFSET_CSRCS.
       NXFSET_CSRCS determines the name of the font C file to build when
       NXFONTS_FONTID=2:

       ifeq ($(CONFIG_NXFONT_SANS23X27),y)
       NXFSET_CSRCS    += nxfonts_bitmaps_sans23x27.c
       endif
       ifeq ($(CONFIG_NXFONT_MYFONT),y)
       NXFSET_CSRCS    += nxfonts_bitmaps_myfont.c
       endif

    7. nuttx/libnx/nxfonts/Makefile.sources.  This is the Makefile used
       in step 5 that will actually generate the font C file.  So, given
       your NXFONTS_FONTID=2, it needs to determine a prefix to use for
       auto-generated variable and function names and (again) the name of
       the auto-generated file to create (this must be the same name that
       was used in nuttx/libnx/nxfonts/Make.defs):

       ifeq ($(NXFONTS_FONTID),1)
       NXFONTS_PREFIX    := g_sans23x27_
       GEN_CSRC    = nxfonts_bitmaps_sans23x27.c
       endif
       ifeq ($(NXFONTS_FONTID),2)
       NXFONTS_PREFIX    := g_myfont_
       GEN_CSRC    = nxfonts_bitmaps_myfont.c
       endif

    8. graphics/libnx/nxfonts_bitmaps.c.  This is the file that contains
       the generic font structures.  It is used as a "template" file by
       nuttx/libnx/nxfonts/Makefile.sources to create your customized
       font data set.

       #if NXFONTS_FONTID == 1
       #  include "nxfonts_sans23x27.h"
       #elif NXFONTS_FONTID == 2
       #  include "nxfonts_myfont.h"
       #else
       #  error "No font ID specified"
       #endif

       Where nxfonts_myfont.h is the NuttX font file that we generated in
       step 2 using the bdf-converter tool.

    9. libnx/nxfonts/nxfonts_getfont.c.  Finally, we need to extend the
       logic that does the run-time font lookups so that can find our new
       font.  The lookup function is NXHANDLE nxf_getfonthandle(enum nx_fontid_e fontid).
       The new font information needs to be added to data structures used by
       that function:

       #ifdef CONFIG_NXFONT_SANS23X27
       extern const struct nx_fontpackage_s g_sans23x27_package;
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
       extern const struct nx_fontpackage_s g_myfont_package;
       #endif

       static FAR const struct nx_fontpackage_s *g_fontpackages[] =
       {
       #ifdef CONFIG_NXFONT_SANS23X27
       &g_sans23x27_package,
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
       &g_myfont_package,
       #endif
       NULL
       };

define.sh and define.bat
------------------------

  Different compilers have different conventions for specifying pre-
  processor definitions on the compiler command line.  This bash
  script allows the build system to create command line definitions
  without concern for the particular compiler in use.

  The define.bat script is a counterpart for use in the native Windows
  build.

flash_writer.py
---------------

  This flash writer is using the xmodem for firmware transfer on
  boards based on cxd56 chip (Ex. Spresense).  This tool depends on
  the xmodem package (https://pypi.org/project/xmodem/).

  for flashing the .spk image to the board please use:
  tools/flash_writer.py -s -c /dev/ttyUSB0 -d -b 115200 -n nuttx.spk

ide_exporter.py
---------------

  This Python script will help to create NuttX project in the IAR and
  uVision IDEs.  These are few simple the steps to export the IDE
  workspaces.

  1) Start the NuttX build from the Cygwin command line before trying to
     create your project by running:

       make V=1 |& tee build_log

     This is necessary to certain auto-generated files and directories that
     will be needed.   This will provide the build log to construct the IDE
     project also.

  2) Export the IDE project base on that make log. The script usage:

     usage: ide_exporter.py [-h] [-v] [-o OUT_DIR] [-d] build_log {iar,uvision_armcc,uvision_gcc} template_dir

     positional arguments:
       build_log             Log file from make V=1
       {iar,uvision_armcc,uvision_gcc}
                             The target IDE: iar, uvision_gcc, (uvision_armcc is experimental)
       template_dir          Directory that contains IDEs template projects

     optional arguments:
       -h, --help            show this help message and exit
       -v, --version         show program's version number and exit
       -o OUT_DIR, --output OUT_DIR
                             Output directory
       -d, --dump            Dump project structure tree

     Example:
        cd nuttx
        make V=1 |& tee build_log

        ./tools/ide_exporter.py makelog_f2nsh_c  iar ./boards/<arch>/<chip>/<board>/ide/template/iar -o ./boards/<arch>/<chip>/<board>/ide/nsh/iar

        or

        ./tools/ide_exporter.py makelog_f2nsh_c uvision_gcc ./boards/<arch>/<chip>/<board>/ide/template/uvision_gcc/ -o ./boards/<arch>/<chip>/<board>/ide/nsh/uvision

  3) Limitations:
     - IAR supports C only. Iar C++ does not compatible with g++ so disable
       C++ if you want to use IAR.
     - uvision_armcc : nuttx asm (inline and .asm) can't be compiled with
       armcc so do not use this option.
     - uvision_gcc : uvision project that uses gcc. Need to specify path to
       gnu toolchain.
       In uVison menu, select:

         Project/Manage/Project Items.../FolderExtension/Use GCC compiler/ PreFix, Folder

  4) Template projects' constrains:
     - mcu, core, link script shall be configured in template project
     - Templates' name are fixed:
        - template_nuttx.eww  : IAR nuttx workspace template
        - template_nuttx_lib.ewp : IAR nuttx library project template
        - template_nuttx_main.ewp : IAR nuttx main project template
        - template_nuttx.uvmpw : uVision workspace
        - template_nuttx_lib.uvproj : uVision library project
        - template_nuttx_main.uvproj : uVision main project
     - iar:
        - Library option shall be set to 'None' so that IAR could use nuttx
           libc
        - __ASSEMBLY__ symbol shall be defined in assembler
     - uVision_gcc:
        - There should be one fake .S file in projects that has been defined
          __ASSEMBLY__ in assembler.
        - In Option/CC tab : disable warning
        - In Option/CC tab : select Compile thump code (or Misc control =
          -mthumb)
        - template_nuttx_lib.uvproj shall add 'Post build action' to copy .a
          file to .\lib
        - template_nuttx_main.uvproj Linker:
          - Select 'Do not use Standard System Startup Files' and 'Do not
            use Standard System Libraries'
          - Do not select 'Use Math libraries'
          - Misc control = --entry=__start

    5) How to create template for other configurations:
        1) uVision with gcc toolchain:
            - Copy 3 uVision project files
            - Select the MCU for main and lib project
            - Correct the path to ld script if needed
        2) iar:
            - Check if the arch supports IAR (only armv7-m is support IAR
              now)
            - Select the MCU for main and lib project
            - Add new ld script file for IAR

    NOTE:  Due to bit rot, the template files for the stm3220g-eval and for
    the stm32f429-disco have been removed from the NuttX repository.  For
    reference, they can be found in the Obsoleted repository at
    Obsoleted/stm32f429i_disco/ltcd/template and at
    Obsoleted/stm3220g-eval/template.

incdir.sh, incdir.bat, and incdir.c
-----------------------------------

  Different compilers have different conventions for specifying lists
  of include file paths on the compiler command line.  This incdir.sh
  bash script allows the build system to create include file paths without
  concern for the particular compiler in use.

  The incdir.bat script is a counterpart for use in the native Windows
  build.  However, there is currently only one compiler supported in
  that context:  MinGW-GCC.

  incdir.c is a higher performance version of incdir.sh, converted to C.

indent.sh
---------

  This script can be used to indent .c and .h files in a manner similar
  to the NuttX coding style.  It doesn't do a really good job, however
  (see below and the comments at the top of the indent.sh file).

  USAGE:
    tools/indent.sh [-d] [-p] -o <out-file> <in-file>
    tools/indent.sh [-d] [-p] <in-file-list>
    tools/indent.sh [-d] -h

  Where:
    -<in-file>
      A single, unformatted input file
    -<in-file-list>
      A list of unformatted input files that will be reformatted in place.
    -o <out-file>
      Write the single, reformatted <in-file> to <out-file>.  <in-file>
      will not be modified.
    -d
      Enable script debug
    -p
      Comments are pre-formatted.  Do not reformat.
    -h
      Show this help message and exit

  The conversions make by the indent.sh script differs from the NuttX coding
  style in that:

    1. The coding standard requires that the trailing */ of a multi-line
       comment be on a separate line.  By default, indent.sh will put the
       final */ on the same line as the last comment text.  If your C file
       already has properly formatted comments then using the -p option will
       eliminate that bad behavior
    2. If your source file has highly formatted comments containing things
       such as tables or lists, then use the -p option to preserve those
       pre-formatted comments.
    3. I usually align things vertically (like '=' in assignments),
    4. indent.sh puts a bogus blank line at the top of the file,
    5. I don't like the way it handles nested conditional compilation
       intermixed with code.  I prefer the preprocessor conditional tests
       be all right justified in that case.
    6. I also indent brackets differently on structures than does this script.
    7. I normally use no spaces in casts.  indent.sh adds spaces in casts like
      "(FAR void *)&foo" becomes "(FAR void *) & foo".
    8. When used with header files, the initial idempotence conditional test
       causes all preprocessor directives to be indented in the file.  So for
       header files, you will need to substitute "^#  " with "#" in the
       converted header file.

   You will manually need to check for the issues listed above after
   performing the conversions.  nxstyle.c provides a good test that will
   catch most of the indent.sh screw-ups.  Together, they do a pretty good
   job of formatting.

   See also nxstyle.c and uncrustify.cfg

kconfig.bat
-----------

  Recent versions of NuttX support building NuttX from a native Windows
  CMD.exe shell.  But kconfig-frontends is a Linux tool and is not yet
  available in the pure CMD.exe environment.  At this point, there are
  only a few options for the Windows user (see the top-level README.txt
  file).

  You can, with some effort, run the Cygwin kconfig-mconf tool directly
  in the CMD.exe shell.  In this case, you do not have to modify the
  .config file, but there are other complexities:  You need to
  temporarily set the Cygwin directories in the PATH variable and
  then run kconfig-mconf outside of the Make system.

  kconfig.bat is a Windows batch file at tools/kconfig.bat that automates
  these steps.  It is used from the top-level NuttX directory like:

    tools/kconfig menuconfig

  NOTE: There is currently an issue with accessing DOS environment
  variables from the Cygwin kconfig-mconf running in the CMD.exe shell.
  The following change to the top-level Kconfig file seems to work around
  these problems:

     config APPSDIR
          string
     -   option env="APPSDIR"
     +   default "../apps"

link.sh, link.bat, copydir.sh, copydir.bat, unlink.sh, and unlink.bat
---------------------------------------------------------------------

  Different file systems have different capabilities for symbolic links.
  Some Windows file systems have no native support for symbolic links.
  Cygwin running under Windows has special links built in that work with
  all cygwin tools.  However, they do not work when Windows native tools
  are used with cygwin.  In that case something different must be done.

  If you are building under Linux or under cygwin with a cygwin tool
  chain, then your Make.defs file may have definitions like the
  following:

    DIRLINK = $(TOPDIR)/tools/link.sh
    DIRUNLINK = (TOPDIR)/tools/unlink.sh

  The first definition is not always present because link.sh is the
  default.  link.sh is a bash script that performs a normal, Linux-style
  symbolic link;  unlink.sh is a do-it-all unlinking script.

  But if you are building under cygwin using a Windows native toolchain
  within a POSIX framework (such as Cygwin), then you will need something
  like the following in you Make.defs file:

    DIRLINK = $(TOPDIR)/tools/copydir.sh
    DIRUNLINK = (TOPDIR)/tools/unlink.sh

  copydir.sh will copy the whole directory instead of linking it.

  Finally, if you are running in a pure native Windows environment with
  a CMD.exe shell, then you will need something like this:

    DIRLINK = $(TOPDIR)/tools/copydir.bat
    DIRUNLINK = (TOPDIR)/tools/unlink.bat

  Note that this will copy directories.  link.bat might also be used in
  this case.  link.bat will attempt to create a symbolic link using the
  NTFS mklink.exe command instead of copying files.  That logic, however,
  has not been verified as of this writing.

Makefile.host
-------------

  This is the makefile that is used to make the mkconfig program from
  the mkconfig.c C file, the cmpconfig program from cmpconfig.c C file,
  the mkversion program from the mkconfig.c C file, or the mksyscall
  program from the mksyscall.c file.  Usage:

  cd tools/
  make -f Makefile.host <program>

mkromfsimg.sh
-------------

  This script may be used to automate the generation of a ROMFS file system
  image.  It accepts an rcS script "template" and generates an image that
  may be mounted under /etc in the NuttX pseudo file system.

  TIP: Edit the resulting header file and mark the generated data values
  as 'const' so that they will be stored in FLASH.

mkdeps.c, cnvwindeps.c, mkwindeps.sh, and mknulldeps.sh
-------------------------------------------------------

  NuttX uses the GCC compiler's capabilities to create Makefile dependencies.
  The program mkdeps is used to run GCC in order to create the dependencies.
  If a NuttX configuration uses the GCC toolchain, its Make.defs file (see
  boards/README.txt) will include a line like:

    MKDEP = $(TOPDIR)/tools/mkdeps[.exe] (See NOTE below)

  If the NuttX configuration does not use a GCC compatible toolchain, then
  it cannot use the dependencies and instead it uses mknulldeps.sh:

    MKDEP = $(TOPDIR)/tools/mknulldeps.sh

  The mknulldeps.sh is a stub script that does essentially nothing.

  mkwindeps.sh is a version that creates dependencies using the Windows
  native toolchain.  That generates Windows native paths in the dependency
  file.  But the mkwindeps.sh uses cnvwindeps.c to convert the Windows
  paths to POSIX paths.  This adds some time to the Windows dependency
  generation but is generally the best option available for that mixed
  environment of Cygwin with a native Windows GCC toolchain.

  mkdeps.c generates mkdeps (on Linux) or mkdeps.exe (on Windows).
  However, this version is still under-development.  It works well in
  the all POSIX environment or in the all Windows environment but also
  does not work well in mixed POSIX environment with a Windows toolchain.
  In that case, there are still issues with the conversion of things like
  'c:\Program Files' to 'c:program files' by bash.  Those issues may,
  eventually be solvable but for now continue to use mkwindeps.sh in
  that mixed environment.

 netusb.sh
 ---------

    Helper script used to set up the CDC ECM Ethernet Over USB driver,
    host routes, and IP Tables rules to support networking with a NuttX
    system that has a CDC ECM Ethernet Over USB driver configured. Only
    supported on Linux.

    General usage:

      $ ./tools/netusb.sh
      Usage: tools/netusb.sh <main-interface> <usb-net-interface> <on|off>

    This has been tested on the SAMA5D3-Xplained board; see
    `boards/arm/sama5/sama5d3-xplained/README.txt` for more information on how
    to configure the CDC ECM driver for that board.

README.txt
----------

  This file!

refresh.sh
----------

  [NOTE: This script with --silent is really obsolete.  refresh with the
   silent option really adds default values.  However, as of 217-07-09,
   defconfig files are retained in a compressed format, i.e., with default
   values removed.  So the --silent option will accomplish nothing.
   Without --silent, you will have the opportunity over override the default
   value from the command line and, in that case, the script may still have
   some minimal value.]

  This is a bash script that automatics refreshing of board default
  configuration (defconfig) files.  It does not do anything special
  that you cannot do manually, but is useful for me when I have to
  update dozens of configuration files.

  Configuration files have to be updated because over time, the
  configuration settings change:  New configurations are added and
  new dependencies are added.  So an old configuration file may
  not be usable anymore until it is refreshed.

  Help is also available:

    $ tools/refresh.sh --help
    tools/refresh.sh is a tool for refreshing board configurations

    USAGE: ./refresh.sh [options] <board>/<config>+

    Where [options] include:
      --debug
         Enable script debug
      --silent
         Update board configuration without interaction
      --defaults
         Do not prompt for new default selections; accept all recommended default values
      --help
         Show this help message and exit
      <board>
         The board directory under nuttx/boards
      <config>
         The board configuration directory under nuttx/boards/<arch>/<chip>/<board>

  The steps to refresh the file taken by refresh.sh are:

  1. Make tools/cmpconfig if it is not already built.
  2. Copy the defconfig file to the top-level NuttX
     directory as .config (being careful to save any previous
     .config file that you might want to keep!).
  3. Execute 'make oldconfig' to update the configuration.
     'make oldconfig' will prompt you for each change in the
     configuration that requires that you make some decision.
     With the --silent option, the script will use 'make
     oldefconfig' instead and you won't have to answer any
     questions;  the refresh will simply accept the default
     value for any new configuration settings.
  4. Then it runs tools/cmpconfig to show the real differences
     between the configuration files.  Configuration files are
     complex and things can move around so a simple 'diff' between
     two configuration files is often not useful.  But tools/cmpconfig
     will show only the meaningful differences between the two
     configuration files.
  4. It will edit the .config file to comment out the setting
     of the CONFIG_APPS_DIR= setting.  This setting should not
     be in checked-in defconfig files because the actually must
     be determined at the next time that the configuration is
     installed.
  5. Finally, the refreshed defconfig file is copied back in
     place where it can be committed with the next set of
     difference to the command line.  If you select the --silent
     option, this file copy will occur automatically.  Otherwise,
     refresh.sh will prompt you first to avoid overwriting the
     defconfig file with changes that you may not want.

rmcr.c
------

  Removes all white space from the end of lines.  Whitespace here
  includes space characters, TAB characters, horizontal and vertical
  TABs, and carriage returns.  Lines will be terminated with the
  newline character only.

sethost.sh
----------

  Saved configurations may run on Linux, Cygwin (32- or 64-bit), or other
  platforms.  The platform characteristics can be changed use 'make
  menuconfig'.  Sometimes this can be confusing due to the differences
  between the platforms.  Enter sethost.sh

  sethost.sh is a simple script that changes a configuration to your
  host platform.  This can greatly simplify life if you use many different
  configurations.  For example, if you are running on Linux and you
  configure like this:

    $ tools/configure.sh board:configuration

  The you can use the following command to both (1) make sure that the
  configuration is up to date, AND (2) the configuration is set up
  correctly for Linux:

    $ tools/sethost.sh -l

  Or, if you are on a Windows/Cygwin 64-bit platform:

    $ tools/sethost.sh -c

  Other options are available:

    $ ./sethost.sh -h

    USAGE: ./sethost.sh [-l|m|c|g|n] [make-opts]
           ./sethost.sh -h

    Where:
      -l|m|c|g|n selects Linux (l), macOS (m), Cygwin (c),
         MSYS/MSYS2 (g) or Windows native (n). Default Linux
      make-opts directly pass to make
      -h will show this help test and terminate

simhostroute.sh
---------------

   Helper script used to set up the tap driver, host routes,
   and IP Tables rules to support networking with the
   simulator under Linux.  General usage:

     $ tools/simhostroute.sh
     Usage: tools/simhostroute.sh <interface> <on|off>

  See boards/sim/sim/sim/NETWORK-LINUX.txt for further information

simbridge.sh
------------

   Helper script used to set up a bridge to support networking with the
   simulator under Linux.  General usage:

     $ tools/simbridge.sh
     Usage: tools/simbridge.sh <interface> <on|off>

  See boards/sim/sim/sim/NETWORK-LINUX.txt for further information

showsize.sh
-----------

  Show the top 10 biggest memory hogs in code and data spaces.  This
  must be executed from the top-level NuttX directory like:

    $ tools/showsize.sh
    TOP 10 BIG DATA
    ...
    TOP 10 BIG CODE
    ...

testbuild.sh
------------

  This script automates building of a set of configurations.  The intent is
  simply to assure that the set of configurations build correctly.  The -h
  option shows the usage:

    $ ./testbuild.sh -h

    USAGE: ./testbuild.sh [-l|m|c|g|n] [-d] [-e <extraflags>] [-x] [-j <ncpus>] [-a <appsdir>] [-t <topdir>] [-p] [-G] <testlist-file>
           ./testbuild.sh -h

    Where:
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
      -R execute "run" script in the config directories if exists.
      -h will show this help test and terminate
      <testlist-file> selects the list of configurations to test.  No default

    Your PATH variable must include the path to both the build tools and the
    kconfig-frontends tools

  These script needs two pieces of information.

    a. A description of the platform that you are testing on.  This description
       is provided by the optional -l, -m, -c, -g and -n options.
    b. A list of configurations to build.  That list is provided by a test
       list file.  The final, non-optional parameter, <testlist-file>,
       provides the path to that file.

  The test list file is a sequence of build descriptions, one per line.  One
  build descriptions consists of two comma separated values.  For example:

    stm32f429i-disco:nsh,CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL
    arduino-due:nsh,CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL
    /arm,CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL
    /risc-v,CONFIG_RISCV_TOOLCHAIN_GNU_RVGL

  The first value is the usual configuration description of the form
  <board-name>:<configuration-name> or /<folder-name> and must correspond to a
  configuration or folder in the nuttx/boards directory.

  The second value is valid name for a toolchain configuration to use
  when building the configuration.  The set of valid toolchain
  configuration names depends on the underlying architecture of the
  configured board.

  The prefix '-' can be used to skip a configuration:
  -stm32f429i-disco/nsh

  or skip a configuration on a specific host(e.g. Darwin):
  -Darwin,sim:rpserver

uncrustify.cfg
--------------

  This is a configuration script for the uncrustify code beautifier.
  Uncrustify does well with forcing braces into "if" statements and
  indenting per the NuttX C coding standard. It correctly does things
  like placing all braces on separate lines at the proper indentation
  level.  It cannot handle certain requirements of the coding standard
  such as

    - FAR attributes in pointer declarations.
    - The NuttX standard function header block comments.
    - Naming violations such as use of CamelCase variable names,
      lower case pre-processor definitions, etc.

  Comment blocks, function headers, files headers, etc. must be formatted
  manually.

  Its handling of block comments is fragile. If the comment is perfect,
  it leaves it alone, but if the block comment is deemed to need a fix
  it starts erroneously indenting the continuation lines of the comment.

    - uncrustify.cfg messed up the indent of most block comments.
      cmt_sp_before_star_cont is applied inconsistently.  I added

        cmt_indent_multi = false # disable all multi-line comment changes

      to the .cfg file to limit its damage to block comments.
    - It is very strict at wrapping lines at column 78. Even when column 79
      just contained the '/' of a closing "*/".  That created many
      bad continuation lines.
    - It moved '{' that opened a struct to the line defining the struct.
      nl_struct_brace = add (or force) seemed to be ignored.
    - It also aligned variable names in declarations and '=' signs in
      assignment statements in a seemingly arbitrary manner. Making changes
      that were not necessary.

  NOTE: uncrustify.cfg should *ONLY* be used with new files that have an
  inconsistent coding style.  uncrustify.cfg should get you in the ballpark,
  but you should expect to review and hand-edit the files to assume 100%
  compliance.

  WARNING: *NEVER* use uncrustify.cfg for modifications to existing NuttX
  files.  It will probably corrupt the style in subtle ways!

  This was last verified against uncrustify 0.66.1 by Bob Feretich.

  About uncrustify:  Uncrustify is a highly configurable, easily modifiable
  source code beautifier.  To learn more about uncrustify:

    http://uncrustify.sourceforge.net/

  Source code is available on GitHub:

    https://github.com/uncrustify/uncrustify

  Binary packages are available for Linux via command line installers.
  Binaries for both Windows and Linux are available at:

    https://sourceforge.net/projects/uncrustify/files/

  See also indent.sh and nxstyle.c

zds
---

  This directory contains build tools used only with the ZDS-II
  platforms (z8, ez80, zNeo).

zipme.sh
--------

  I use this script to create the nuttx-xx.yy.tar.gz tarballs for
  release.  It is handy because it also does the kind of clean up
  that you need to do to make a clean code release.
  It can also PGP sign the final tarballs and create their SHA512 hash.
  Any VCS files or directories are excluded from the final tarballs.

  $ ./tools/zipme.sh -h
    USAGE="USAGE: ./tools/zipme.sh [-d|h|v|s] [-b <build]> [-e <exclude>] [-k <key-id>] [<major.minor.patch>]"
  Examples:
      ./tools/zipme.sh -s 9.0.0
        Create version 9.0.0 tarballs and sign them.
      ./tools/zipme.sh -s -k XXXXXX 9.0.0
        Same as above but use the key-id XXXXXX to sign the tarballs
      ./tools/zipme.sh -e "*.swp tmp" 9.0.0
        Create the tarballs but exclude any .swp file and the "tmp" directory.
