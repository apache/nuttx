# APACHE NUTTX

* Introduction
* Community
  - Getting Help
  - Mailing Lists
  - Issue Tracker
  - Source Code
  - Website Source Code
* Environments
  - Installing Cygwin
  - Ubuntu Bash under Windows 10
  - Using macOS
* Installation
  - Download and Unpack
  - Semi-Optional apps/ Package
  - Installation Directories with Spaces in the Path
  - Downloading from Repositories
  - Related Repositories
  - Notes about Header Files
* Configuring NuttX
  - Instantiating "Canned" Configurations
  - Refreshing Configurations
  - NuttX Configuration Tool
  - Finding Selections in the Configuration Menus
  - Reveal Hidden Configuration Options
  - Make Sure that You are on the Right Platform
  - Comparing Two Configurations
  - Making defconfig Files
  - Incompatibilities with Older Configurations
  - NuttX Configuration Tool under DOS
* Toolchains
  - Cross-Development Toolchains
  - NuttX Buildroot Toolchain
* Shells
* Building NuttX
  - Building
  - Re-building
  - Build Targets and Options
  - Native Windows Build
  - Installing GNUWin32
* Cygwin Build Problems
  - Strange Path Problems
  - Window Native Toolchain Issues
* Documentation

# INTRODUCTION

Apache NuttX is a real-time operating system (RTOS) with an emphasis on
standards compliance and small footprint. Scalable from 8-bit to 64-bit
microcontroller environments, the primary governing standards in NuttX are
POSIX and ANSI standards. Additional standard APIs from Unix and other
common RTOSs (such as VxWorks) are adopted for functionality not available
under these standards, or for functionality that is not appropriate for
deeply-embedded environments (such as fork()).

Extensive documentation can be found on the project wiki:
  <https://cwiki.apache.org/NUTTX/NuttX>

For brevity, many parts of the documentation will refer to Apache NuttX as
simply NuttX.

# COMMUNITY

Every volunteer project obtains its strength from the people involved in
it.  We invite you to participate as much or as little as you choose.

We encourage you to:

  - Use our project and provide feedback.
  - Provide us with use-cases.
  - Report bugs and submit patches.
  - Contribute code or documentation.

## Getting Help

The best place to get help is the developer's mailing list.  Please see
the following section:

## Mailing Lists

Get help using NuttX or contribute to the project on our mailing lists:

  * <dev@nuttx.apache.org> is for people who want to contribute code to NuttX.
    * To subscribe, send an email to <dev-subscribe@nuttx.apache.org>.
    * To unsubscribe, send an email to <dev-unsubscribe@nuttx.apache.org>.
    * View the archives at:
      <https://www.mail-archive.com/dev@nuttx.apache.org/>

  * <commits@nuttx.apache.org> is a read-only list that notifies subscribers
  about commit messages and patches to NuttX.
    * To subscribe, send an email to <commits-subscribe@nuttx.apache.org>.
    * To unsubscribe, send an email to <commits-unsubscribe@nuttx.apache.org>.
    * View the archives at:
      <https://www.mail-archive.com/commits@nuttx.apache.org/>

## Reporting Security Issues

Found a vulnerability? See our security policy [here](.github/SECURITY.md).

## Issue Tracker

### Bug Reports:

Found bug? Send an email to the dev list: <dev@nuttx.apache.org>

Before submitting an issue, please:

  - Verify that the bug does in fact exist.

  - Search the mailing list archives to verify there is no existing issue
    reporting the bug you've found.

  - Consider tracking down the bug yourself in the NuttX source code and
    submitting a patch along with your bug report.  This is a great time
    saver for the NuttX developers and helps ensure the bug will be fixed
    quickly.

### Feature Requests:

Enhancement requests for new features are also welcome. The more concrete
and rational the request is, the greater the chance it will incorporated
into future releases.

## Source Code

The project sources are in two Git repositories.  The core OS is in nuttx
and the apps repository is in nuttx-apps.  These are housed in GitBox on
ASF servers and also mirrored at GitHub.  These are kept in sync, so you
can use whichever option you prefer.

  - NuttX core OS repository:

    - Primary:
      <https://gitbox.apache.org/repos/asf?p=nuttx.git>

    - GitHub Mirror:
      <https://github.com/apache/nuttx>

  - Apps repository:

    - Primary:
      <https://gitbox.apache.org/repos/asf?p=nuttx-apps.git>

    - GitHub Mirror:
      <https://github.com/apache/nuttx-apps>

## Website Source Code

The project website sources are accessible via the website source code
  repository which is also mirrored in GitHub:

- Primary:
  <https://gitbox.apache.org/repos/asf?p=nuttx-website.git>

- GitHub Mirror:
  <https://github.com/apache/nuttx-website>

# ENVIRONMENTS

  NuttX requires a POSIX development environment such as you would find under
  Linux or macOS.  NuttX may also be installed and built on Windows system
  if you also provide such a POSIX development environment.  Options for a
  POSIX development environment under Windows include:

  - An installation of Linux on a virtual machine (VM) in Windows.  I have
    not been happy using a VM myself.  I have had stability problems with
    open source VMs and commercial VMs cost more than I want to spend.
    Sharing files with Linux running in a VM is awkward;  sharing devices
    connected to the Windows box with Linux in a VM is, at the very least,
    confusing;  Using Windows tools (such as Segger J-Link) with files
    built under the Linux VM is not a possibility.

  - The Cygwin environment.  Instructions for installation of Cygwin on a
    Windows system are provided in the following paragraph, "Installing
    Cygwin".  Cygwin is a mature, well-tested, and very convenient
    environment.  It is especially convenient if you  need to
    integrate with Windows tools and files.  Downsides are that the
    installation time is very long and the compile times are slow.

  - Ubuntu/Bash shell under Windows 10.  This is a new option under
    Windows 10.  See the section "Ubuntu Bash under Windows 10" below.
    This is an improvement over Cygwin if your concern is compile time;
    its build performance is comparable to native Linux, certainly better
    than the Cygwin build time.  It also installs in a tiny fraction of
    the time as Cygwin, perhaps 20 minutes for the basic Ubuntu install
    (vs. more than a day for the complete Cygwin install).

    There have been even more recent ports of Linux environment to
    Windows.  I need to update this section to include some mention of
    these alternatives.

  - The MSYS environment.  MSYS derives from an older version of Cygwin
    simplified and adapted to work more naturally in the Windows
    environment.  See <http://www.mingw.org/wiki/MSYS> if you are
    interested in using MSYS.  The advantages of the MSYS environment is
    that it is better integrted with the native Windows environment and
    lighter weight; it uses only a  minimal number of add-on POSIX-land
    tools.

    The download link in that Wiki takes you to the SourceForge download
    site.  The SourceForge MSYS project has been stagnant for some time.
    The MSYS project has more recently moved to
    <http://odsn.net/projects/sfnet_mingwbundle>.  Downloads of current .zip
    files are available there but no instructions for the installation.

  - MSYS2 appears to be a re-write of MSYS based on a newer version of
    Cygwin.  Is it available at <https://www.msys2.org>.  A windows
    installer is available at that site along with very good installation
    instructions.  The download is relatively quick (at least compared to
    Cygwin) and the 'pacman' package management tool supports supports
    simple system updates.  For example, 'pacman -S git' will install the
    GIT command line utilities.

  - Other POSIX environments.  Check out:

    - UnxUtils: <https://sourceforge.net/projects/unxutils/>,
      <https://en.wikipedia.org/wiki/UnxUtils>
    - MobaXterm: <https://mobaxterm.mobatek.net/>
    - Gow: <https://github.com/bmatzelle/gow/wiki>

    **Disclaimer**:  In principle, these should work.  However, I have never
    used any of these environments and cannot guarantee that there is
    not some less-than-obvious issues.

NuttX can also be installed and built on a native Windows system, but with
some potential tool-related issues (see the discussion "Native Windows
Build" under "Building NuttX" below).  GNUWin32 is used to provide
compatible native windows tools.

## Installing Cygwin

Installing Cygwin on your Windows PC is simple, but time consuming.  See
<http://www.cygwin.com/> for installation instructions. Basically you just
need to download a tiny setup.exe program and it does the real, network
installation for you.

Some Cygwin installation tips:

  1. Install at `C:\cygwin`

  2. Install **everything**:  "Only the minimal base packages from the
     Cygwin distribution are installed by default. Clicking on categories
     and packages in the setup.exe package installation screen will
     provide you with the ability to control what is installed or updated.
     Clicking on the "Default" field next to the "All" category will
     provide you with the opportunity to install every Cygwin package.
     Be advised that this will download and install hundreds of megabytes
     to your computer."

If you use the "default" installation, you will be missing many
of the Cygwin utilities that you will need to build NuttX.  The
build will fail in numerous places because of missing packages.

NOTE: The last time I installed **everything**, the download was
about 5GiB.  The server I selected was also very slow so it took
over a day to do the whole install!

NOTE: You don't really have to install **everything** but I cannot
answer the question "Then what should I install?"  I don't know
the answer to that and so will continue to recommend installing
**everything**.

You should certainly be able to omit "Science", "Math", and
"Publishing".  You can try omitting KDE, Gnome, GTK, and other
graphics packages if you don't plan to use them.

Perhaps a minimum set would be those packages listed below for the
"Ubuntu Bash under Windows 10" installation?

**UPDATE**:  Sergey Frolov had success with the following minimal
Cygwin configuration:

   1. After starting the Cygwin installer, keep the recommended
      packages that are pre-selected in the default configuration.

   2. Using the installation tools, add the following packages:

          make (GNU make)  bison        libgmp3-dev
          gcc-core         byacc        libmpfr-dev
          gcc-g++          gperf        libmpc-dev
          flex             gdb          automake-1.15
          libncurses-dev   libgmp-dev   curl

After installing Cygwin, you will get lots of links for installed
tools and shells.  I use the RXVT native shell.  It is fast and reliable
and does not require you to run the Cygwin X server (which is neither
fast nor reliable).  Unless otherwise noted, the rest of these
instructions assume that you are at a bash command line prompt in
either Linux or in Cygwin shell.

## Using MSYS

MSYS is an environment the derives from Cygwin.  Thus, most things said
about Cygwin apply equally to MSYS.  This section will, then, focus on
the differences when using MSYS, specifically MSYS2.

Here is it assumed that you have already downloaded and installed MSYS2
from https://www.msys2.org using the windows installer available at that
location.  It is also assumed that you have brought in the necessary
tools using the 'pacman' package management tool Tools needed including:

    pacman -S git
    pacman -S make
    pacman -S gcc
    pacman -S gdb

And possibly others depending upon your usage.  Then you will need to
build and install kconfig-frontends per the instructions of the top-level
README.txt file in the tools repository.  This requires the following
additional tools:

    pacman -S bison
    pacman -S curl
    pacman -S gperf
    pacman -S ncurses-devel
    pacman -S automake-wrapper
    pacman -S autoconf
    pacman -S pkg-config

Because of some versioning issues, I had to run 'aclocal' prior to
running the kconfig-frontends configure script.  See "Configuring NuttX"
below for further information.

Unlike Cygwin, MSYS does not support symbolic links.  The 'ln -s' command
will, in fact, copy a directory!  This means that you Make.defs file will
have to include definitions like:

    ifeq ($(CONFIG_WINDOWS_MSYS),y)
      DIRLINK = $(TOPDIR)/tools/copydir.sh
      DIRUNLINK = $(TOPDIR)/tools/unlink.sh
    endif

This will force the directory copies to work in a way that can be handled
by the NuttX build system.  NOTE:  The default link.sh script has been
updated so that is should now be MSYS2 compatible.  The above is preferred
but no longer necessary in the Make.defs file.

To build the simulator under MSYS, you also need:

    pacman -S zlib-devel

It appears that you cannot use directory names with spaces in them like
"/c/Program\ Files \(86\)" in the MSYS path variable.  I worked around this
by create Windows junctions like this:

  1. Open the a windows command terminal,

  2. cd to `c:\msys64`, then

  3. `mklink /j programfiles "C:/Program\ Files"` and

  4. `mklink /j programfiles86 "C:/Program\ Files\ \(x86\)"`

     They then show up as `/programfiles` and `/programfiles86` with the MSYS2
     sandbox.  Those paths can then be used with the PATH variable.  I had
     to do something similar for the path to the GNU Tools "ARM Embedded
     Toolchain" which also has spaces in the path name.

## Ubuntu Bash under Windows 10

A better version of a command-line only Ubuntu under Windows 10 (beta)
has recently been made available from Microsoft.

### Installation

Installation instructions abound on the Internet complete with screen
shots.  I will attempt to duplicate those instructions in full here.
Here are the simplified installation steps:

  - Open *Settings*.

  - Click on *Update & security*.

  - Click on *For Developers*.

  - Under *Use developer features*, select the *Developer mode* option to
    setup the environment to install Bash.

  - A message box should pop up.  Click *Yes* to turn on developer mode.

  - After the necessary components install, you'll need to restart your
    computer.

    Once your computer reboots:

  - Open *Control Panel*.

  - Click on *Programs*.

  - Click on *Turn Windows features on or off*.

  - A list of features will pop up, check the *Windows Subsystem for Linux
    (beta)* option.

  - Click *OK*.

  - Once the components installed on your computer, click the *Restart
    now* button to complete the task.

    After your computer restarts, you will notice that Bash will not appear in
    the *Recently added* list of apps, this is because Bash isn't actually
    installed yet. Now that you have setup the necessary components, use the
    following steps to complete the installation of Bash:

  - Open *Start*, do a search for `bash.exe`, and press *Enter*.

  - On the command prompt, type `y` and press Enter to download and install
    Bash from the Windows Store.  This will take awhile.

  - Then you'll need to create a default UNIX user account. This account
    doesn't have to be the same as your Windows account. Enter the
    username in the required field and press Enter (you can't use the
    username `admin`).

  - Close the `bash.exe` command prompt.

  Now that you completed the installation and setup, you can open the Bash
  tool from the Start menu like you would with any other app.

### Accessing Windows Files from Ubuntu

File systems will be mounted under `/mnt` so for example `C:\Program Files`
appears at `/mnt/c/Program Files`.  This is as opposed to Cygwin where
the same directory would appear at `/cygdrive/c/Program Files`.

With these differences (perhaps a few other Windows quirks) the Ubuntu
install works just like Ubuntu running natively on your PC.

A good tip for file sharing is to use symbolic links within your Ubuntu
home directory.  For example, suppose you have your `projects` directory
at `C:\Documents\projects`.  Then you can set up a link to the `projects/`
directory in your Ubuntu directory like:

    ln -s /mnt/c/Documents/projects projects

### Accessing Ubuntu Files From Windows

In Ubuntu Userspace for Windows, the Ubuntu file system root directory is
at:

    %localappdata%\lxss\rootfs

Or

    C:\Users\Username\AppData\Local\lxss\rootfs

However, I am unable to see my files under the rootfs\home directory.
After some looking around, I find the home directory
`%localappdata%\lxss\home`.

With that trick access to the `/home` directory, you should actually be
able to use Windows tools outside of the Ubuntu sandbox with versions of
NuttX built within the sandbox using that path.

### Executing Windows Tools from Ubuntu

You can also execute Windows tools from within the Ubuntu sandbox:

    /mnt/c/Program\ Files\ \(x86\)/Microchip/xc32/v1.43/bin/xc32-gcc.exe --version
    Unable to translate current working directory. Using C:\WINDOWS\System32
    xc32-gcc.exe (Microchip Technology) 4.8.3 MPLAB XC32 Compiler v1.43 Build date: Mar  1 2017
    ...

The error message indicates that there are more issues:  You cannot mix
Windows tools that use Windows style paths in an environment that uses
POSIX paths.  I think you would have to use Linux tools only from within
the Ubuntu sandbox.

### Install Ubuntu Software

Use `sudo apt-get install <package name>`.  As examples, this is how
you would get GIT:

    sudo apt-get install git

This will get you a compiler for your host PC:

    sudo apt-get install gcc

This will get you an ARM compiler for your target:

    sudo apt-get install gcc-arm-none-eabi

**NOTE**: That is just an example.  I am not sure if apt-get will give you a
current or usable compiler.  You should carefully select your toolchain
for the needs of your project.

You will also need to get the kconfig-frontends configuration as
described below under *NuttX Configuration Tool*.  In order to build the
kconfig-frontends configuration tool you will also need: `make`, `gperf`,
`flex`, `bison`, and `libncurses-dev`.

That is enough to do a basic NuttX build.

### Integrating with Windows Tools

If you want to integrate with Windows native tools, then you would need
deal with the same kind of craziness as with integrating Cygwin with
native toolchains, see the section *Cygwin Build Problems* below.

However, there is currently no build support for using Windows native
tools with Ubuntu under Windows.  This tool combination is made to work
with Cygwin through the use of the `cygpath -w` tool that converts paths
from say `/cydrive/c/Program Files` to `C:\Program Files`.  There is,
however, no corresponding tool to convert `/mnt/c/Program Files` in the
Ubuntu environment.

### Graphics Support

The Ubuntu version support by Microsoft is a command-line only version.
There is no support for Linux graphics utilities.

This limitation is not a limitation of Ubuntu, however, only in what
Microsoft is willing to support.  If you install a X-Server, then you
can also use basic graphics utilities.  See for example:

  <http://www.howtogeek.com/261575/how-to-run-graphical-linux-desktop-applications-from-windows-10s-bash-shell/>

Many Linux graphics programs would, however, also require a graphics
framework like GTK or Qt.  So this might be a trip down the rabbit hole.

### Using macOS

You need to install at least the following tools specific to macOS.

  * flock (used by APPDIR build logic)

A macOS port is available at: <https://github.com/discoteq/flock>

    brew tap discoteq/discoteq
    brew install flock

  If you want to build the sim:

  * Xcode (the native compiler and the rest of the toolchain)

  * ELF toolchain (if you want to build modules for CONFIG_LIBC_MODLIB)

    brew install x86_64-elf-gcc

# INSTALLATION

There are two ways to get NuttX:  You may download released, stable
tarballs from either the project website.  Or you may get NuttX by
cloning the GIT repositories.  Let's consider the released tarballs
first:

## Download and Unpack

Download and unpack the NuttX tarball.  If you are reading this, then
you have probably already done that.  After unpacking, you will end
up with a directory called nuttx-version (where version is the NuttX
version number). You might want to rename that directory nuttx to
match the various instructions in the documentation and some scripts
in the source tree.

  * Download location:

    <https://nuttx.apache.org/download/>

  * Legacy download locations:

    <https://bitbucket.org/nuttx/nuttx/downloads>
    <https://sourceforge.net/projects/nuttx/files/nuttx/>

## Semi-Optional apps/ Package

All NuttX libraries and example code used to be in included within
the NuttX source tree.  As of NuttX-6.0, this application code was
moved into a separate tarball, the apps tarball.  If you are just
beginning with NuttX, then you will want to download the versioned
apps tarball along with the NuttX tarball.  If you already have your
own product application directory, then you may not need the apps
tarball.

It is called "Semi-optional" because if you don't have some `apps/`
directory, NuttX will *fail* to build! You do not necessarily need
to use the NuttX apps tarball but may, instead, provide your own
custom application directory.  Such a custom directory would need
to include a valid Makefile to support the build and a valid Kconfig
file to support the configuration.  More about these files later.

Download then unpack the apps tarball in the same directory where you
unpacked the NuttX tarball.  After you unpack the apps tarball, you
will have a new directory called apps-version (where the version
should exactly match the version of the NuttX tarball).  Again, you
might want to rename the directory to simply apps/ to match what
you read in the documentation

After unpacking (and renaming) the apps tarball, you will have two
directories side by side like this:

             |
        +----+----+
        |         |
      nuttx/     apps/

This is important because the NuttX build will expect to find the
apps directory in that (default) location.  That default location
can be changed by modifying your NuttX configuration file, but that
is another story.

## Installation Directories with Spaces in the Path

The nuttx build directory should reside in a path that contains no
spaces in any higher level directory name.  For example, under
Cygwin, your home directory might be formed from your first and last
names like: `/home/First Last`. That will cause strange errors when
the make system tries to build.

[Actually, that problem is probably not too difficult to fix.  Some
 Makefiles probably just need some paths within double quotes]

I work around spaces in the home directory name, by creating a
new directory that does not contain any spaces, such as `/home/nuttx`.
Then I install NuttX in `/home/nuttx` and always build from
`/home/nuttx/nuttx-code`.

## Downloading from Repositories

### Cloning the Repository

**BEFORE** cloning repositories on any Windows platform do the following GIT
command:

    git config --global core.autocrlf false

That will avoid conversions of linefeeds (newlines, \n) to carriage
return plus linefeed sequences (\r\n)

The current NuttX du jour is available in from a GIT repository.  Here are
instructions for cloning the core NuttX RTOS (corresponding to the nuttx
tarball discussed above):

    git clone https://gitbox.apache.org/repos/asf/nuttx.git nuttx

-or-

    git clone https://github.com/apache/nuttx.git nuttx

And the semi-optional apps/ application directory and be cloned like:

    git clone https://gitbox.apache.org/repos/asf/nuttx-apps.git apps

-or-

    git clone https://github.com/apache/nuttx-apps.git apps

That will give you the same directory structure like this:

             |
        +----+----+
        |         |
      nuttx/     apps/

### Configuring the Clones

The following steps need to be performed for each of the repositories.
After changing to the clone directory:

Set your identity:

    git config --global user.name "My Name"
    git config --global user.email my.name@example.com

Colorized diffs are much easier to read:

    git config --global color.branch auto
    git config --global color.diff auto
    git config --global color.interactive auto
    git config --global color.status auto

Checkout other settings

    git config --list

### Cloning NuttX Inside Cygwin

If you are cloning the NuttX repository, it is recommended to avoid
automatic end of lines conversions by git. These conversions may break
some scripts like configure.sh. Before cloning, do the following:

    git config --global core.autocrlf false

## Related Repositories

These are standalone repositories:

  * <https://gitbox.apache.org/repos/asf/nuttx-apps>
    or
    <https://github.com/apache/nuttx-apps.git>

    This directory holds an optional package of applications and libraries
    can be used with the NuttX RTOS.  There is a README.txt file there that
    will provide more information about that package.

  * <https://bitbucket.org/nuttx/nxwidgets>

    This is the NuttX C++ graphics support.  This includes NxWM, the tiny
    NuttX Window Manager.

  * <https://bitbucket.org/nuttx/uclibc>

    This repository contains a version of the uClibc++ C++ library.  This code
    originates from <http://cxx.uclibc.org/> and has been adapted for NuttX by the
    RGMP team (<http://rgmp.sourceforge.net/wiki/index.php/Main_Page>).

  * <https://bitbucket.org/nuttx/buildroot>

    A environment that you can to use to build a custom, NuttX GNU toolchain.

  * <https://bitbucket.org/nuttx/tools>

    There are snapshots of some tools here that you will need to work with
    NuttX:  kconfig-frontends, genromfs, and others.

## Notes about Header Files

### Other C-Library Header Files

When a GCC toolchain is built, it must be built against a C library.
The compiler together with the contents of the C library completes the
C language definition and provides the complete C development
environment.  NuttX provides its own, built-in C library.  So the
complete, consistent C language definition for use with NuttX comes from
the combination of the compiler and the header files provided by the
NuttX C library.

When a GCC toolchain is built, it incorporates the C library header
files into the compiler internal directories and, in this way, the C
library really becomes a part of the toolchain.  If you use the NuttX
buildroot toolchain as described below under "NuttX Buildroot
Toolchain", your GCC toolchain will build against the NuttX C library
and will incorporate the NuttX C library header files as part of the
toolchain.

If you use some other, third-party tool chain, this will not be the
case, however.  Those toolchains were probably built against some
other, incompatible C library distribution (such as newlib).  Those
tools will have incorporated the incompatible C library header files
as part of the toolchain.  These incompatible header files must *not*
be used with NuttX because they will conflict with definitions in the
NuttX built-in C-Library.  For such toolchains that include header
files from a foreign C-Library, NuttX must be compiled without using
the standard header files that are distributed with your toolchain.
This prevents including conflicting, incompatible header files such
as stdio.h.

The math.h and stdarg.h are probably the two most trouble some header
files to deal with.  These troublesome header files are discussed in
more detail below.

### Header Files Provided by Your Toolchain

Certain header files, such as `setjmp.h`, `stdarg.h`, and `math.h`, may still
be needed from your toolchain and your compiler may not, however, be able
to find these if you compile NuttX without using standard header files
(i.e., with `-nostdinc`).  If that is the case, one solution is to copy
those header file from your toolchain into the NuttX include directory.

### Duplicated Header Files

There are also a few header files that can be found in the `nuttx/include`
directory which are duplicated by the header files from your toolchain.
stdint.h and stdbool.h are examples.  If you prefer to use the `stdint.h`
and `stdbool.h` header files from your toolchain, those could be copied
into the `nuttx/include/` directory. Using most other header files from
your toolchain would probably cause errors.

### math.h

Even though you should not use a foreign C-Library, you may still need
to use other, external libraries with NuttX.  In particular, you may
need to use the math library, libm.a.  NuttX supports a generic, built-in
math library that can be enabled using `CONFIG_LIBM=y`.  However, you may
still want to use a higher performance external math library that has
been tuned for your CPU.  Sometimes such tuned math libraries are
bundled with your toolchain.

The math library header file, `math.h`, is a then special case.  If you do
nothing, the standard math.h header file that is provided with your
toolchain will be used.

If you have a custom, architecture specific math.h header file, then
that header file should be placed at `arch/<cpu>/include/math.h`.  There
is a stub `math.h` header file located at `include/nuttx/lib/math.h`.  This stub
header file can be used to "redirect" the inclusion to an architecture-
specific math.h header file.  If you add an architecture specific math.h
header file then you should also define `CONFIG_ARCH_MATH_H=y` in your
NuttX Configuration file.  If `CONFIG_ARCH_MATH_H` is selected, then the
top-level Makefile will copy the stub math.h header file from
`include/nuttx/lib/math.h` to `include/math.h` where it will become the system
`math.h` header file.  The stub `math.h` header file does nothing other
than to include that architecture-specific `math.h` header file as the
system `math.h` header file.

### float.h

If you enable the generic, built-in math library, then that math library
will expect your toolchain to provide the standard `float.h` header file.
The float.h header file defines the properties of your floating point
implementation.  It would always be best to use your toolchain's `float.h`
header file but if none is available, a default `float.h` header file will
be provided if this option is selected.  However, there is no assurance
that the settings in this `float.h` are actually correct for your platform!

### stdarg.h

In most cases, the correct version of stdarg.h is the version provided
with your toolchain.  However, sometimes there are issues with
using your toolchains `stdarg.h`.  For example, it may attempt to draw in
header files that do not exist in NuttX or perhaps the header files that
it uses are not compatible with the NuttX header files.  In those cases,
you can use an architecture-specific `stdarg.h` header file by defining
`CONFIG_ARCH_STDARG_H=y`.

See the discussion above for the `math.h` header.  This setting works
exactly the same for the `stdarg.h` header file.

# CONFIGURING NUTTX

## Instantiating "Canned" Configurations

### `configure.sh` and `configure.bat`

"Canned" NuttX configuration files are retained in:

    boards/<arch-name>/<chip-name>/<board-name>/configs/<config-dir>

Where `<board-name>` is the name of your development board and `<config-dir>`
is the name of the sub-directory containing a specific configuration for
that board. `<arch-name>` and `<chip-name>` refer to characteristics of the
MCU used on the board: `<arch-name>` is the CPU architecture implemented
by the MCU; `<chip-name>` identifies the MCU chip family.  Only a few
steps are required to instantiate a NuttX configuration, but to make the
configuration even easier there are scripts available in the tools/
sub-directory combines those simple steps into one command.

There is one tool for use with any Bash-like shell that does configuration
steps.  It is used as follows:

    tools/configure.sh <board-name>:<config-dir>

There is an alternative Windows batch file that can be used in the windows
native environment like:

    tools\configure.bat <board-name>:<config-dir>

And, to make sure that other platforms are supported, there is also a
C program at tools/configure.c that can be compiled to establish the
board configuration.

See `tools/README.txt` for more information about these scripts.

General information about configuring NuttX can be found in:

    {TOPDIR}/boards/README.txt
    {TOPDIR}/boards/<arch-name>/<chip-name>/<board-name>/README.txt

### The Hidden Configuration Scripts:

As mentioned above, there are only a few simple steps to instantiating a
NuttX configuration.  Those steps are hidden by the configuration scripts
but are summarized below:

  1. Copy Files

     Configuring NuttX requires only copying two files from the
     `<config-dir>` to the directory where you installed NuttX (TOPDIR):

       * Copy `boards/<arch-name>/<chip-name>/<board-name>/configs/<config-dir>/Make.def` to `{TOPDIR}/Make.defs`

         OR

       * Copy `boards/<arch-name>/<chip-name>/<board-name>/scripts/Make.def`
         to `{TOPDIR}/Make.defs`

         Make.defs describes the rules needed by your tool chain to compile
         and link code.  You may need to modify this file to match the
         specific needs of your toolchain.  NOTE that a configuration may
         have its own unique Make.defs file in its configuration directory or
         it may use a common Make.defs file for the board in the scripts/
         directory.  The first takes precedence.

       * Copy `boards/<arch-name>/<chip-name>/<board-name>/configs/<config-dir>/defconfig` to `{TOPDIR}/.config`

         The defconfig file holds the actual build configuration.  This
         file is included by all other make files to determine what is
         included in the build and what is not.  This file is also used
         to generate a C configuration header at `include/nuttx/config.h`.

       * Copy other, environment-specific files to `{TOPDIR}`

         This might include files like .gdbinit or IDE configuration files
         like .project or .cproject.

  2. Refresh the Configuration

     New configuration setting may be added or removed.  Existing settings
     may also change there values or options.  This must be handled by
     refreshing the configuration as described below.

     NOTE:  NuttX uses only compressed defconfig files.  For the NuttX
     defconfig files, this refreshing step is *NOT* optional; it is also
     necessary to uncompress and regenerate the full making file.  This is
     discussed further below.

## Refreshing Configurations

Configurations can get out of date.  As new configuration settings are
added or removed or as dependencies between configuration settings
change, the contents of a default configuration can become out of synch
with the build systems.  Hence, it is a good practice to "refresh" each
configuration after configuring and before making.  To refresh the
configuration, use the NuttX Configuration Tool like this:

    make oldconfig

AFTER you have instantiated the NuttX configuration as described above.
The configuration step copied the .config file into place in the top-level
NuttX directory; 'make oldconfig' step will then operate on that .config
file to bring it up-to-date.

If your configuration is out of date, you will be prompted by 'make oldconfig'
to resolve the issues detected by the configuration tool, that is, to
provide values for the new configuration options in the build system.  Doing
this can save you a lot of problems down the road due to obsolete settings in
the default board configuration file.  The NuttX configuration tool is
discussed in more detail in the following paragraph.

Confused about what the correct value for a new configuration item should
be?  Enter ? in response to the 'make oldconfig' prompt and it will show
you the help text that goes with the option.

If you don't want to make any decisions are willing to just accept the
recommended default value for each new configuration item, an even easier
way is:

    make olddefconfig

The olddefconfig target will simply bring your configuration up to date with
the current Kconfig files, setting any new options to the default value.
No questions asked.

## NuttX Configuration Tool

An automated tool has been incorporated to support re-configuration
of NuttX.  This tool is based on the kconfig-frontends application available
at <https://bitbucket.org/nuttx/tools/src/master/kconfig-frontends/>.  (This
is a snapshot of the old <http://ymorin.is-a-geek.org/projects/kconfig-frontends>
which is no longer available.)  This application provides a tool called
`kconfig-mconf` that is used by the NuttX top-level Makefile. The following
make target is provided:

    make menuconfig

This make target will bring up NuttX configuration menus.

**WARNING**:  Never do `make menuconfig` on a configuration that has
not been converted to use the kconfig-frontends tools!  This will
damage your configuration (see
<https://cwiki.apache.org/confluence/display/NUTTX/Converting+Legacy+Configurations+to+Use+kconfig-mconf>).

NuttX also supports kconfiglib(https://github.com/ulfalizer/Kconfiglib) by default,
which is a Kconfig tool implemented in Python 2/3. Compared with kconfig-frontends,
kconfiglib provides NuttX with the possibility of multi-platform support(configure
NuttX in Winodws native/Visual Studio), and also kconfiglib has a stronger Kconfig
syntax check, this will help developers to avoid some Kconfig syntax errors.
Install kconfiglib via following command:

    pip install kconfiglib

If you are a working on Windows, which also need the support of windows-curses:

    pip install windows-curses

**NOTE**:  It should be noted that kconfiglib does not support **modules** attributes.
(<https://github.com/ulfalizer/Kconfiglib/blob/master/kconfiglib.py#L3239-L3254>,
the community seems to have stopped updating), if the features depends on
`CONFIG_BUILD_LOADABLE`, kconfiglib may not be a good choice.

How do we tell a new configuration from an old one? See "Incompatibilities
with Older Configurations" below.

The `menuconfig` make target depends on two things:

  1. The Kconfig configuration data files that appear in almost all
     NuttX directories.  These data files are the part that is still
     under development (patches are welcome!).  The Kconfig files
     contain configuration information for the configuration settings
     relevant to the directory in which the Kconfig file resides.

     NOTE: For a description of the syntax of this configuration file,
     see kconfig-language.txt in the tools repository at
     <https://bitbucket.org/nuttx/tools>

  2. The `kconfig-mconf` tool. `kconfig-mconf` is part of the
     kconfig-frontends package.  You can download that package from the
     snapshot in the tools repository at <https://bitbucket.org/nuttx/tools>.

     Building kconfig-frontends under Linux may be as simple as
     `configure; make; make install` but there may be some build
     complexities, especially if you are building under Cygwin.  See
     the more detailed build instructions in the top-level README.txt
     file of the tools repository at <https://bitbucket.org/nuttx/tools>.

     The `make install` step will, by default, install the `kconfig-mconf`
     tool at `/usr/local/bin/mconf`.  Where ever you choose to
     install `kconfig-mconf`, make certain that your PATH variable includes
     a path to that installation directory.

     The kconfig-frontends tools will not build in a native Windows
     environment directly "out-of-the-box".  For the Windows native
     case, you can use the modified version of kconfig-frontends
     that can be found at

     <http://uvc.de/posts/linux-kernel-configuration-tool-kconfig-under-windows.html>

     or a more recent port that can be found at

     <http://reclonelabs.com/more-kconfig-awesomeness-for-windows/>.

The basic configuration order is "bottom-up":

   - Select the build environment,
   - Select the processor,
   - Select the board,
   - Select the supported peripherals
   - Configure the device drivers,
   - Configure the application options on top of this.

This is pretty straight forward for creating new configurations
but may be less intuitive for modifying existing configurations.

Another ncurses-based tool that is an option to kconfig-mconf is
kconfig-nconf.  The differences are primary in in the aesthetics of the
UI.  If you have kconfig-nconf built, then you can invoke that front end
with:

     make nconfig

If you have an environment that supports the Qt or GTK graphical systems
(probably KDE or gnome, respectively, or Cygwin under Windows with Qt or
GTK installed), then you can also build the graphical kconfig-frontends,
kconfig-qconf and kconfig-gconf.  In these case, you can start the
graphical configurator with either:

     make qconfig

 or

     make gconfig

Some keyboard shortcuts supported by kconfig-mconf, the tool that runs
when you do 'make menuconfig':

   - `?` will bring up the mconfig help display.

   - `/` can be used find configuration selections.

   - `Z` can be used to reveal hidden configuration options

These last two shortcuts are described further in the following
paragraphs.

## Finding Selections in the Configuration Menus

The NuttX configuration options have gotten complex and it can be very
difficult to find options in the menu trees if you are not sure where
to look.  The "basic configuration order" describe above can help to
narrow things down.

But if you know exactly what configuration setting you want to select,
say `CONFIG_XYZ`, but not where to find it, then the `make menuconfig`
version of the tool offers some help:  By pressing the '/' key, the
tool will bring up a menu that will allow you to search for a
configuration item.  Just enter the string `CONFIG_XYZ` and press ENTER.
It will show you not only where to find the configuration item, but
also all of the dependencies related to the configuration item.

## Reveal Hidden Configuration Options

If you type `Z`, then `kconfig-mconf` will change what is displayed.
Normally, only enabled features that have all of their dependencies met
are displayed.  That is, of course, not very useful if you would like to
discover new options or if you are looking for an option and do not
realize that the dependencies have not yet been selected and, hence, it
is not displayed.

But if you enter `Z`, then every option will be shown, whether or not its
dependencies have been met.  You can then see everything that could be
selected with the right dependency selections.  These additional options
will be shown the `-` for the selection and for the value (since it
cannot be selected and has no value).  About all you do is to select
the `<Help>` option to see what the dependencies are.

## Make Sure that You are on the Right Platform

Saved configurations may run on Linux, Cygwin (32- or 64-bit), or other
platforms.  The platform characteristics can be changed use `make
menuconfig`.  Sometimes this can be confusing due to the differences
between the platforms.  Enter `sethost.sh`

sethost.sh is a simple script that changes a configuration to your
host platform.  This can greatly simplify life if you use many different
configurations.  For example, if you are running on Linux and you
configure like this:

    tools/configure.sh board:configuration

The you can use the following command to both (1) make sure that the
configuration is up to date, AND (2) the configuration is set up
correctly for Linux:

    tools/sethost.sh -l

Or, if you are on a Windows/Cygwin 64-bit platform:

    tools/sethost.sh -c

Or, for MSYS/MSYS2:

    tools/sethost.sh -g

Other options are available from the help option built into the
script.  You can see all options with:

    tools/sethost.sh -h

Recently, the options to the configure.sh (and configure.bat) scripts have
been extended so that you both setup the configuration, select for the host
platform that you use, and uncompress and refresh the defconfig file all in
one command like:

    tools/configure.sh -l board:configuration

For a Linux host or for a Windows/Cygwin host:

    tools/configure.sh -c board:configuration

Other options are available from the help option built into the
script.  You can see all options with:

    tools/configure.sh -h

## Comparing Two Configurations

If you try to compare two configurations using 'diff', you will probably
not be happy with the result.  There are superfluous things added to
the configuration files that make comparisons with the human eye
difficult.

There is a tool at nuttx/tools/cmpconfig.c that can be built to simplify
these comparisons.  The output from this difference tool will show only
the meaningful differences between two configuration files.  This tool is
built as follows:

    cd nuttx/tools
    make -f Makefile.host

This will create a program called 'cmpconfig' or 'comconfig.exe' on Windows.

Why would you want to compare two configuration files?  Here are a few
of the reasons why I do this

  1. When I create a new configuration I usually base it on an older
     configuration and I want to know, "What are the options that I need to
     change to add the new feature to the older configurations?"  For example,
     suppose that I have a boardA/nsh configuration and I want to create a
     boardA/nxwm configuration.  Suppose I already have boardB/nsh and
     boardB/nxwm configurations.  Then by comparing the boardB/nsh with the
     boardB/nxwm I can see the modifications that I would need to make to my
     boardA/nsh to create a new  boardA/nxwm.

  2. But the most common reason that I use the 'cmpconfig' program is to
     check the results of "refreshing" a configuration with 'make oldconfig'
     (see the paragraph "Refreshing Configurations" above).  The 'make
     oldconfig' command will make changes to my configuration and using
     'cmpconfig', I can see precisely what those changes were and if any
     should be of concern to me.

  3. The 'cmpconfig' tool can also be useful when converting older, legacy
     manual configurations to the current configurations based on the
     kconfig-frontends tools.  See the following paragraph.

## Making `defconfig` Files

### `.config` Files as `defconfig` Files:

The minimum `defconfig` file is simply the generated `.config` file with
CONFIG_APPS_DIR setting removed or commented out.  That setting provides
the name and location of the `apps/` directory relative to the `nuttx` build
directory.  The default is `../apps/`, however, the apps directory may be
any other location and may have a different name.  For example, the name
of versioned NuttX releases are always in the form `apps-xx.yy` where `xx.yy`
is the version number.

### Finding the `apps/` Directory Path:

When the default configuration is installed using one of the scripts or
programs in the NuttX tools directory, there will be an option to provide
the path to the `apps/` directory.  If not provided, then the configure tool
will look around and try to make a reasonable decision about where the
`apps/` directory is located.

### Compressed `defconfig` Files:

The `Makefile` also supports an option to generate very small `defconfig`
files.  The `.config` files are quite large and complex.  But most of the
settings in the `.config` file simply have the default settings from the
`Kconfig` files.  These `.config` files can be converted into small `defconfig`
file:

    make savedefconfig

That make target will generate a defconfig file in the top-level
directory.  The size reduction is really quite remarkable:

    wc -l .config defconfig
     1085 .config
       82 defconfig
     1167 total

In order to be usable, the `.config` file installed from the compressed
defconfig file must be reconstituted using:

    make olddefconfig

  > **NOTE 1**:  Only compressed defconfig files are retained in the NuttX repository.
  > All patches and PRs that attempt to add or modify a defconfig file MUST
  > use the compressed defconfig format as created by 'make savdefconfig.'

  > **NOTE 2**:  When 'make savedefconfig' runs it will try several things some of
  > which are expected to fail.  In these cases you will see an error message
  > from make followed by "(ignored)."  You should also ignore these messages

**CAUTION**:  This size reduction was accomplished by removing all setting
from the `.config` file that were at the default value. `make olddefconfig`
can regenerate the original `.config` file by simply restoring those default
settings.  The underlying assumption here is, of course, that the default
settings do not change.  If the default settings change, and they often
do, then the original `.config` may not be reproducible.

So if your project requires 100% reproducibility over a long period of
time, you make want to save the complete `.config` files vs. the standard,
compressed `defconfig` file.

### Configuring with "Compressed" defconfig Files:

As described above `defconfig`, all NuttX `defconfig` files are compressed
using `make savedeconfig`.  These compressed `defconfig` files are
generally not fully usable as they are and may not build the target
binaries that you want because the compression process removed all of
the default settings from the `defconfig` file.  To restore the default
settings, you should run the following after configuring:

    make olddefconfig

That will restore the the missing defaulted values.

Using this command after configuring is generally a good practice anyway:
Even if the `defconfig` files are not "compressed" in this fashion, the
`defconfig` file may be old and the only way to assure that the installed
`.config` is is up to date is via `make oldconfig` or `make olddefconfig`.
See the paragraph above entitled "Refreshing Configurations" for
additional information.

## Incompatibilities with Older Configurations

**WARNING**

The current NuttX build system supports *only* the new compressed,
`defconfig` configuration files generated using the `kconfig-frontends` tools
as described in the preceding section.  Support for the older, legacy,
manual configurations was eliminated in NuttX 7.0; support for
uncompressed `.config-files-as-defconfig` files was eliminated after
NuttX-7.21.  All configurations must now be done using the
`kconfig-frontends` tool.  The older manual configurations and the new
`kconfig-frontends` configurations are not compatible.  Old legacy
configurations can *not* be used with the `kconfig-frontends` tool and,
hence, cannot be used with releases of NuttX 7.0 and beyond:

If you run `make menuconfig` with a legacy configuration the resulting
configuration will probably not be functional.

  > Q: How can I tell if a configuration is a new kconfig-frontends
  >   configuration or an older, manual configuration?
  >
  > A: Only old, manual configurations will have an appconfig file
  >
  > Q: How can I convert a older, manual configuration into a new,
  >    kconfig-frontends toolchain.
  >
  > A: Refer to <https://cwiki.apache.org/confluence/display/NUTTX/Converting+Legacy+Configurations+to+Use+kconfig-mconf>

**WARNING**

As described above, whenever you use a configuration, you really should
always refresh the configuration with the following command *before* you
make NuttX:

    make oldconfig

OR

    make olddefconfig

This will make sure that the configuration is up-to-date in the event that
it has lapsed behind the current NuttX development (see the paragraph
"Refreshing Configurations" above).  But this only works with *new*
configuration files created with the kconfig-frontends tools.

Further, this step is *NOT* optional with the new, compressed defconfig
files.  It is a necessary step that will also uncompress the defconfig
file, regenerating the `.config` and making it usable for NuttX builds.

Never do `make oldconfig` (OR `make menuconfig`) on a  configuration that
has not been converted to use the kconfig-frontends tools!  This will
damage your configuration (see
<https://cwiki.apache.org/confluence/display/NUTTX/Converting+Legacy+Configurations+to+Use+kconfig-mconf>).

## NuttX Configuration Tool under DOS

  Recent versions of NuttX support building NuttX from a native Windows
  console window (see *Native Windows Build* below).  But `kconfig-frontends`
  is a Linux tool.  At one time this was a problem for Windows users, but
  now there are two specially modified versions of the `kconfig-frontends`
  tools that can be used.  One can be found here:
  <http://uvc.de/posts/linux-kernel-configuration-tool-kconfig-under-windows.html>

  The configuration steps of the most recent versions of NuttX require the
  `kconfig-tweak` tool that is not not available in the the above.  However,
  there has been an update to this `Kconfig` Windows tools that does include
  `kconfig-tweak`:  http://reclonelabs.com/more-kconfig-awesomeness-for-windows/

  Source code is available here: <https://github.com/reclone/kconfig-frontends-win32>
  and <https://github.com/reclone/kconfig-frontends-win32/releases>

  It is also possible to use the version of `kconfig-frontends` built
  under Cygwin outside of the Cygwin *sandbox* in a native Windows
  environment:

  1. You can run the configuration tool using Cygwin.  However, the
     Cygwin `Win.mk` will complain so to do this will, you have
     to manually edit the `.config` file:

     a. Delete the line: `CONFIG_WINDOWS_NATIVE=y`

     b. Change the apps/ directory path, `CONFIG_APPS_DIR` to use Unix
        style delimiters.  For example, change `..\apps` to `../apps`

     And of course, after you use the configuration tool you need to
     restore `CONFIG_WINDOWS_NATIVE=y` and the correct `CONFIG_APPS_DIR`.

  2. You can, with some effort, run the Cygwin `kconfig-mconf` tool
     directly in the Windows console window.  In this case, you do not
     have to modify the `.config` file, but there are other complexities:

     a. You need to temporarily set the Cygwin directories in the PATH
        variable then run `kconfig-mconf` manually like:

          kconfig-mconf Kconfig

        There is a Windows batch file at `tools/kconfig.bat` that automates
        these steps:

         tools/kconfig menuconfig

     b. There is an issue with accessing DOS environment variables from
        the Cygwin `kconfig-mconf` running in the Windows console.  The
        following change to the top-level `Kconfig` file seems to work
        around these problems:

          config APPSDIR
              string
          -   option env="APPSDIR"
          +   default "../apps"

# TOOLCHAINS

## Cross-Development Toolchains

  In order to build NuttX for your board, you will have to obtain a cross-
  compiler to generate code for your target CPU.  For each board,
  configuration, there is a `README.txt` file (at
  `boards/<arch-name>/<chip-name>/<board-name>/README.txt`).
  That README file contains suggestions and information about appropriate
  tools and development environments for use with your board.

  In any case, the PATH environment variable will need to be updated to
  include the location where the build can find the toolchain binaries.

## NuttX Buildroot Toolchain

  For many configurations, a DIY set of tools is available for NuttX.  These
  tools can be downloaded from the NuttX Bitbucket.org file repository.  After
  unpacking the buildroot tarball, you can find instructions for building
  the tools in the `buildroot/boards/README.txt` file.

  Check the README.txt file in the configuration directory for your board
  to see if you can use the buildroot toolchain with your board (this
  README.txt file is located in
  `boards/<arch-name>/<chip-name>/<board-name>/README.txt`).

  This toolchain is available for both the Linux and Cygwin development
  environments.

  Advantages:  (1) NuttX header files are built into the tool chain,
  and (2) related support tools like NXFLAT tools, the ROMFS
  genromfs tools, and the kconfig-frontends tools can be built into your
  toolchain.

  Disadvantages:  This tool chain is not was well supported as some other
  toolchains.  GNU tools are not my priority and so the buildroot tools
  often get behind.  For example, until recently there was no EABI support
  in the NuttX buildroot toolchain for ARM.

  NOTE: For Cortex-M3/4, there are OABI and EABI versions of the buildroot
  toolchains.  If you are using the older OABI toolchain the prefix for
  the tools will be `arm-nuttx-elf-`; for the EABI toolchain the prefix will
  be `arm-nuttx-eabi-`. If you are using the older OABI toolchain with
  an ARM Cortex-M3/4, you will need to set CONFIG_ARM_TOOLCHAIN_BUILDROOT_OABI
  in the `.config` file in order to pick the right tool prefix.

  If the make system ever picks the wrong prefix for your toolchain, you
  can always specify the prefix on the command to override the default
  like:

    make CROSSDEV=arm-nuttx-elf

# SHELLS

The NuttX build relies on some shell scripts.  Some are inline in the
Makefiles and many are executable scripts in the `tools/`. directory.  The
scripts were all developed using bash and many contain bash shell
dependencies.

Most of the scripts begin with `#!/bin/bash` to specifically select the
bash shell.  Some still have `#!/bin/sh` but I haven't heard any complaints
so these must not have bash dependencies.

There are two shell issues that I have heard of:

  1. Linux where `/bin/sh` refers to an incompatible shell (like `ksh` or `csh`).

     In this case, bash is probably available and the `#!/bin/bash` at the
     beginning of the file should do the job.  If any scripts with `#!/bin/sh`
     fail, try changing that to `#!/bin/bash` and let me know about the change.

  2. FreeBSD with the Bourne Shell and no bash shell.

     The other, reverse case has also been reported on FreeBSD setups that
     have the Bourne shell, but not bash.  In this base, `#!/bin/bash` fails
     but `#!/bin/sh` works okay.  My recommendation in this case is to create
     a symbolic link at `/bin/bash` that refers to the Bourne shell.

     There may still be issues, however, with certain the `bash`-centric scripts
     that will require modifications.

# BUILDING NUTTX

## Building

NuttX builds in-place in the source tree.  You do not need to create
any special build directories.  Assuming that your Make.defs is setup
properly for your tool chain and that PATH environment variable contains
the path to where your cross-development tools are installed, the
following steps are all that are required to build NuttX:

    cd {TOPDIR}
    make

At least one configuration (eagle100) requires additional command line
arguments on the make command.  Read
`{TOPDIR}/boards/<arch-name>/<chip-name>/<board-name>/README.txt` to see
if that applies to your target.

## Re-building

Re-building is normally simple -- just type make again.

But there are some things that can "get you" when you use the Cygwin
development environment with Windows native tools.  The native Windows
tools do not understand Cygwin's symbolic links, so the NuttX make system
does something weird:  It copies the configuration directories instead of
linking to them (it could, perhaps, use the NTFS `mklink` command, but it
doesn't).

A consequence of this is that you can easily get confused when you edit
a file in one of the linked (i.e., copied) directories, re-build NuttX,
and then not see your changes when you run the program.  That is because
build is still using the version of the file in the copied directory, not
your modified file!

Older versions of NuttX did not support dependencies in this
configuration.  So a simple work around this annoying behavior in this
case was the following when you re-build:

     make clean_context all

This 'make' command will remove of the copied directories, re-copy them,
then make NuttX.

However, more recent versions of NuttX do support dependencies for the
Cygwin build.  As a result, the above command will cause everything to be
rebuilt (because it removes and will cause recreating the
`include/nuttx/config.h` header file).  A much less gracefully but still
effective command in this case is the following for the ARM configuration:

    rm -rf arch/arm/src/chip arch/arm/src/board

This "kludge" simple removes the copied directories.  These directories
will be re-created when you do a normal 'make' and your edits will then be
effective.

## Build Targets and Options

### Build Targets

Below is a summary of the build targets available in the top-level
NuttX Makefile:

  * `all`

    The default target builds the NuttX executable in the selected output
    formats.

  * `clean`

    Removes derived object files, archives, executables, and temporary
    files, but retains the configuration and context files and directories.

  * `distclean`

    Does 'clean' then also removes all configuration and context files.
    This essentially restores the directory structure to its original,
    unconfigured stated.

Application housekeeping targets.  The APPDIR variable refers to the user
application directory.  A sample `apps/` directory is included with NuttX,
however, this is not treated as part of NuttX and may be replaced with a
different application directory.  For the most part, the application
directory is treated like any other build directory in the `Makefile` script.
However, as a convenience, the following targets are included to support
housekeeping functions in the user application directory from the NuttX
build directory.

  * `apps_clean`

    Perform the clean operation only in the user application directory

  * `apps_distclean`

    Perform the distclean operation only in the user application directory.
    The apps/.config file is preserved so that this is not a "full" distclean
    but more of a configuration "reset" for the application directory.

  * `export`

    The export target will package the NuttX libraries and header files into
    an exportable package.  Caveats: (1) These needs some extension for the KERNEL
    build. (2) The logic in tools/mkexport.sh only supports GCC and, for example,
    explicitly assumes that the archiver is 'ar'

  * `flash` (or `download` : DEPRECATED)

    This is a helper target that will rebuild NuttX and flash it to the target
    system in one step.  The operation of this target depends completely upon
    implementation of the FLASH command in the user Make.defs file.  It will
    generate an error if the FLASH command is not defined.

The following targets are used internally by the make logic but can be invoked
from the command under certain conditions if necessary.

  * `depend`

    Create build dependencies. (NOTE:  There is currently no support for build
    dependencies under Cygwin using Windows-native toolchains.)

  * `context`

    The context target is invoked on each target build to assure that NuttX is
    properly configured.  The basic configuration steps include creation of the
    the `config.h` and `version.h` header files in the `include/nuttx` directory and
    the establishment of symbolic links to configured directories.

  * `clean_context`

    This is part of the `distclean` target.  It removes all of the header files
    and symbolic links created by the context target.

### Build Options

Of course, the value any make variable an be overridden from the make command
line.  However, there is one particular variable assignment option that may
be useful to you:

  * `V=1`

    This is the build "verbosity flag."  If you specify `V=1` on the make command
    line, you will see the exact commands used in the build. This can be very
    useful when adding new boards or tracking down compile time errors and
    warnings (Contributed by Richard Cochran).

## Native Windows Build

The beginnings of a Windows native build are in place but still not often
used as of this writing.  The build was functional but because of lack of
use may find some issues to be resolved with this build configuration.

The windows native build logic initiated if CONFIG_WINDOWS_NATIVE=y is
defined in the NuttX configuration file:

This build:

  - Uses all Windows style paths
  - Uses primarily Windows batch commands from cmd.exe, with
  - A few extensions from GNUWin32

In this build, you cannot use a Cygwin or MSYS shell. Rather the build must
be performed in a Windows console window. Here is a better terminal than the
standard issue, CMD.exe terminal:  ConEmu which can be downloaded from:
<https://sourceforge.net/projects/conemu/> or <https://conemu.github.io/>.

Build Tools.  The build still relies on some Unix-like commands.  I use
the GNUWin32 tools that can be downloaded from <http://gnuwin32.sourceforge.net/>
using the *Download all* selection.  Individual packages can be download
instead if you know what you are doing and want a faster download (No, I
can't tell you which packages you should or should not download).

NOTE:  It should be possible to use Cygwin or MSYS2 in place of the GNUWin32
tools.  There are, however, complexities in doing that because those tools
depend on the shell environment and use DLLs that are not found (at least
not without the correct setup).

Host Compiler:  I use the MingGW GCC compiler which can be downloaded from
<http://www.mingw.org/>.  If you are using GNUWin32, then it is recommended
the you not install the optional MSYS components as there may be conflicts.

Kconfig-frontends:  See the section entitled "NuttX Configuration Tool
under DOS" for information about installing the `kconfig-frontend` tools to
run natively under Windows.

This capability should still be considered a work in progress because:

  1. It has not been verified on all targets and tools, and
  2. it still lacks some of the creature-comforts of the more mature
     environments.

## Installing GNUWin32

  The Windows native build will depend upon a few Unix-like tools that can be
  provided either by MSYS or GNUWin32.  The GNUWin32 are available from
  <http://gnuwin32.sourceforge.net/>.  GNUWin32 provides ports of tools with a
  GPL or similar open source license to modern MS-Windows (Microsoft Windows
  2000 / XP / 2003 / Vista / 2008 / 7).  See
  <http://gnuwin32.sourceforge.net/packages.html> for a list of all of the tools
  available in the GNUWin32 package.

  The SourceForge project is located here:
  <http://sourceforge.net/projects/gnuwin32/>.  The project is still being
  actively supported (although some of the Windows ports have gotten very old).

  Some commercial toolchains include a subset of the GNUWin32 tools in the
  installation.  My recommendation is that you download the GNUWin32 tools
  directly from the sourceforge.net website so that you will know what you are
  using and can reproduce your build environment.

  GNUWin32 Installation Steps:

  The following steps will download and execute the GNUWin32 installer.

1. Download `GetGNUWin32-x.x.x.exe` from
   <http://sourceforge.net/projects/getgnuwin32/files/>.  This is the
   installer.  The current version as of this writing is 0.6.3.

2. Run the installer.

3. Accept the license.

4. Select the installation directory.  My recommendation is the
   directory that contains this README file (`<this-directory>`).

5. After running `GetGNUWin32-0.x.x.exe`, you will have a new directory
   `<this-directory>/GetGNUWin32`

   Note that the GNUWin32 installer didn't install GNUWin32.  Instead, it
   installed another, smarter downloader.  That downloader is the GNUWin32
   package management tool developed by the Open SSL project.

   The following steps probably should be performed from inside a DOS shell.

6. Change to the directory created by `GetGNUWin32-x.x.x.exe`

      cd GetGNUWin32

7. Execute the download.bat script.  The download.bat script will download
   about 446 packages!  Enough to have a very complete Linux-like environment
   under the DOS shell.  This will take awhile.  This step only downloads
   the packages and the next step will install the packages.

      download

8. This step will install the downloaded packages.  The argument of the
   install.bat script is the installation location.  C:\gnuwin32 is the
   standard install location:

     install C:\gnuwin32

  **NOTE**:  This installation step will install *all* GNUWin32 packages... far
  more than you will ever need.  If disc space is a problem for you, you might
  need to perform a manual installation of the individual ZIP files that you
  will find in the `<this directory>/GetGNUWin32/packages` directory.

9. Make sure that you add the GNUWin32 tools to your path variable:

         set PATH=C:\gnuwin32\bin;%PATH%

  **WARNING**:  Make sure you have `C:\MinGW\bin` in your path before any other
  directory that contains `libiconv-2.dll`. Apparently the `as.exe` in some
  MinGW distributions are dependent on that DLL, and having an old
  version of it in the path somewhere (for example GnuWin32 tools) will
  cause as.exe to pick up the older version that doesn't have the entry
  point it's looking for.

# CYGWIN BUILD PROBLEMS

## Performance

Build performance under Cygwin is really not so bad, certainly not as good
as a Linux build.  However, often you will find that the performance is
not just bad but terrible.  If you are seeing awful performance.. like two
or three compilations per second.. the culprit is usually your Windows
Anti-Virus protection interfering with the build tool program execution.

I use Cygwin quite often and I use Windows Defender.  In order to get good
build performance, I routinely keep the Windows Defender "Virus & Threat
Protections Settings" screen up:  I disable "Real-Time Protection" just
before entering 'make' then turn "Real-Time Protection" back on when the
build completes.  With this additional nuisance step, I find that build
performance under Cygwin is completely acceptable.

## Strange Path Problems

If you see strange behavior when building under Cygwin then you may have
a problem with your PATH variable.  For example, if you see failures to
locate files that are clearly present, that may mean that you are using
the wrong version of a tool.  For example, you may not be using Cygwin's
'make' program at /usr/bin/make.  Try:

    which make
    /usr/bin/make

When you install some toolchains (such as Yargarto or CodeSourcery tools),
they may modify your PATH variable to include a path to their binaries.
At that location, they may have GNUWin32 versions of the tools.  So you
might actually be using a version of make that does not understand Cygwin
paths.

The solution is either:

  1. Edit your PATH to remove the path to the GNUWin32 tools, or

  2. Put /usr/local/bin, /usr/bin, and /bin at the front of your path:

         export PATH=/usr/local/bin:/usr/bin:/bin:$PATH

## Window Native Toolchain Issues

There are many popular Windows native toolchains that may be used with NuttX.
Examples include CodeSourcery (for Windows), devkitARM, and several vendor-
provided toolchains.  There are several limitations with using a and Windows
based toolchain in a Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath' utility
     but you might easily find some new path problems.  If so, check out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic links
     are used in NuttX (e.g., include/arch).  The make system works around these
     problems for the Windows tools by copying directories instead of linking them.
     But this can also cause some confusion for you:  For example, you may edit
     a file in a "linked" directory and find that your changes had no effect.
     That is because you are building the copy of the file in the "fake" symbolic
     directory.  If you use a Windows toolchain, you should get in the habit of
     making like this:

         make clean_context all

     An alias in your .bashrc file might make that less painful.  The rebuild
     is not a long as you might think because there is no dependency checking
     if you are using a native Windows toolchain.  That bring us to #3:

## General Pre-built Toolchain Issues

To continue with the list of "Window Native Toolchain Issues" we can add
the following.  These, however, are really just issues that you will have
if you use any pre-built toolchain (vs. building the NuttX toolchain from
the NuttX buildroot package):

There may be incompatibilities with header files, libraries, and compiler
built-in functions detailed below.  For the most part, these issues
are handled in the existing make logic.  But if you are breaking new ground,
then you may encounter these:

  1. Header Files.  Most pre-built toolchains will build with a foreign C
     library (usually newlib, but maybe uClibc or glibc if you are using a
     Linux toolchain).  This means that the header files from the foreign
     C library will be built into the toolchain.  So if you `#include <stdio.h>`,
     you will get the stdio.h from the incompatible, foreign C library and
     not the nuttx `stdio.h` (at `nuttx/include/stdio.h`) that you wanted.

     This can cause confusion in the builds and you must always be
     sure the `-nostdinc` is included in the `CFLAGS`.  That will assure that
     you take the include files only from

  2. Libraries.  What was said above header files applies to libraries.
     You do not want to include code from the libraries of any foreign
     C libraries built into your toolchain.  If this happens you will get
     perplexing errors about undefined symbols.  To avoid these errors,
     you will need to add `-nostdlib` to your `CFLAGS` flags to assure that
     you only take code from the NuttX libraries.

     This, however, may causes other issues for libraries in the toolchain
     that you do want (like `libgcc.a` or `libm.a`).  These are special-cased
     in most Makefiles, but you could still run into issues of missing
     libraries.

  3. Built-Ins.  Some compilers target a particular operating system.
     Many people would, for example, like to use the same toolchain to
     develop Linux and NuttX software.  Compilers built for other
     operating systems may generate incompatible built-in logic and,
     for this reason, `-fno-builtin` should also be included in your
     C flags

     And finally you may not be able to use NXFLAT.

  4. NXFLAT. If you use a pre-built toolchain, you will lose all support
     for NXFLAT.  NXFLAT is a binary format described in
     Documentation/NuttXNxFlat.html.  It may be possible to build
     standalone versions of the NXFLAT tools; there are a few examples
     of this in the buildroot repository at <https://bitbucket.org/nuttx/buildroot>
     However, it is possible that there could be interoperability issues
     with your toolchain since they will be using different versions of
     binutils and possibly different ABIs.

## Building Original Linux Boards in Cygwin

Some default board configurations are set to build under Linux and others
to build under Windows with Cygwin.  Various default toolchains may also
be used in each configuration.  It is possible to change the default
setup.  Here, for example, is what you must do in order to compile a
default Linux configuration in the Cygwin environment using the
CodeSourcery for Windows toolchain.  After instantiating a "canned"
NuttX configuration, run the target 'menuconfig' and set the following
items:

    Build Setup->Build Host Platform->Windows
    Build Setup->Windows Build Environment->Cygwin
    System Type->Toolchain Selection->CodeSourcery GNU Toolchain under Windows

In Windows 7 it may be required to open the Cygwin shell as Administrator
("Run As" option, right button) you find errors like "Permission denied".

## Recovering from Bad Configurations

Many people make the mistake of configuring NuttX with the "canned"
configuration and then just typing `make` with disastrous consequences;
the build may fail with mysterious, uninterpretable, and irrecoverable
build errors.  If, for example, you do this with an unmodified Linux
configuration in a Windows/Cgwin environment, you will corrupt the
build environment.  The environment will be corrupted because of POSIX vs
Windows path issues and with issues related to symbolic links.  If you
make the mistake of doing this, the easiest way to recover is to just
start over: Do `make distclean` to remove every trace of the corrupted
configuration, reconfigure from scratch, and make certain that the set
the configuration correctly for your platform before attempting to make
again.

Just fixing the configuration file after you have instantiated the bad
configuration with 'make' is not enough.

# DOCUMENTATION

Additional information can be found in the Documentation/ directory and
also in README files that are scattered throughout the source tree.  The
documentation is in HTML and can be access by loading the following file
into your Web browser:

    Documentation/index.html

NuttX documentation is also available online at <https://nuttx.apache.org/>.

Below is a guide to the available README files in the NuttX source tree:

    nuttx/
     |
     |- arch/
     |   |
     |   |- arm/
     |   |   `- src
     |   |       |- common
     |   |       |   `- README_lwl_console.txt
     |   |       |- lpc214x
     |   |       |    `-README.txt
     |   |       `- stm32l4
     |   |           `- README.txt
     |   |- renesas/
     |   |   |- include/
     |   |   |   `-README.txt
     |   |   |- src/
     |   |   |   `-README.txt
     |   |- x86/
     |   |   |- include/
     |   |   |   `-README.txt
     |   |   `- src/
     |   |       `-README.txt
     |   `- z80/
     |   |   `- src/
     |   |       |- z80/README.txt
     |   |       `- z180/README.txt, z180_mmu.txt
     |   `- README.txt
     |- audio/
     |   `-README.txt
     |- boards/
     |   |- arm/
     |   |   |- a1x/
     |   |   |   `- pcduino-a10/
     |   |   |       `- README.txt
     |   |   |- am335x/
     |   |   |   `- beaglebone-black/
     |   |   |       `- README.txt
     |   |   |- c5471/
     |   |   |   `- c5471evm/
     |   |   |       `- README.txt
     |   |   |- cxd56xx/
     |   |   |   `- spresense/
     |   |   |       `- README.txt
     |   |   |- dm320/
     |   |   |   `- ntosd-dm320/
     |   |   |       |- doc/README.txt
     |   |   |       `- README.txt
     |   |   |- efm32/
     |   |   |   |- efm32-g8xx-stk/
     |   |   |   |   `- README.txt
     |   |   |   |- efm32gg-stk3700/
     |   |   |   |   `- README.txt
     |   |   |   `- olimex-efm32g880f128-stk/
     |   |   |       `- README.txt
     |   |   |- imx6/
     |   |   |   `- sabre-6quad/
     |   |   |       `- README.txt
     |   |   |- imxrt/
     |   |   |   |- imxrt1050-evk/
     |   |   |   |   `- README.txt
     |   |   |   |- imxrt1060-evk/
     |   |   |   |   `- README.txt
     |   |   |   `- teensy-4.x/
     |   |   |       `- README.txt
     |   |   |- kinetis/
     |   |   |   |- freedom-k28f/
     |   |   |   |   `- README.txt
     |   |   |   |- freedom-k64f/
     |   |   |   |   `- README.txt
     |   |   |   |- freedom-k66f/
     |   |   |   |   `- README.txt
     |   |   |   |- kwikstik-k40/
     |   |   |   |   `- README.txt
     |   |   |   |- teensy-3.x/
     |   |   |   |   `- README.txt
     |   |   |   |- twr-k60n512/
     |   |   |   |   `- README.txt
     |   |   |   `- twr-k64f120m/
     |   |   |       `- README.txt
     |   |   |- kl/
     |   |   |   |- freedom-kl25z/
     |   |   |   |   `- README.txt
     |   |   |   |- freedom-kl26z/
     |   |   |   |   `- README.txt
     |   |   |   `- teensy-lc/
     |   |   |       `- README.txt
     |   |   |- lc823450/
     |   |   |   `- lc823450-xgevk/
     |   |   |       `- README.txt
     |   |   |- lpc17xx_40xx/
     |   |   |   |- lincoln60/
     |   |   |   |   `- README.txt
     |   |   |   |- lpc4088-devkit/
     |   |   |   |   `- README.txt
     |   |   |   |- lpc4088-quickstart/
     |   |   |   |   `- README.txt
     |   |   |   |- lpcxpresso-lpc1768/
     |   |   |   |   `- README.txt
     |   |   |   |- lx_cpu/
     |   |   |   |   `- README.txt
     |   |   |   |- mbed/
     |   |   |   |   `- README.txt
     |   |   |   |- mcb1700/
     |   |   |   |   `- README.txt
     |   |   |   |- olimex-lpc1766stk/
     |   |   |   |   `- README.txt
     |   |   |   |- open1788/
     |   |   |   |   `- README.txt
     |   |   |   |- pnev5180b/
     |   |   |   |   `- README.txt
     |   |   |   |- u-blox-c027/
     |   |   |   |   `- README.txt
     |   |   |   `- zkit-arm-1769/
     |   |   |       `- README.txt
     |   |   |- lpc214x/
     |   |   |   |- mcu123-lpc214x/
     |   |   |   |   `- README.txt
     |   |   |   `- zp214xpa/
     |   |   |       `- README.txt
     |   |   |- lpc2378/
     |   |   |   `- olimex-lpc2378/
     |   |   |       `- README.txt
     |   |   |- lpc31xx/
     |   |   |   |- ea3131/
     |   |   |   |   `- README.txt
     |   |   |   |- ea3152/
     |   |   |   |   `- README.txt
     |   |   |   `- olimex-lpc-h3131/
     |   |   |       `- README.txt
     |   |   |- lpc43xx/
     |   |   |   |- bambino-200e/
     |   |   |   |   `- README.txt
     |   |   |   |- lpc4330-xplorer/
     |   |   |   |   `- README.txt
     |   |   |   |- lpc4337-ws/
     |   |   |   |   `- README.txt
     |   |   |   |- lpc4357-evb/
     |   |   |   |   `- README.txt
     |   |   |   `- lpc4370-link2/
     |   |   |       `- README.txt
     |   |   |- lpc54xx/
     |   |   |   `- lpcxpresso-lpc54628/
     |   |   |       `- README.txt
     |   |   |- max326xx/
     |   |   |   `- max32660-evsys/
     |   |   |       `- README.txt
     |   |   |- moxart/
     |   |   |   `- moxa/
     |   |   |- nrf52/
     |   |   |   `- nrf52-generic/
     |   |   |       `- README.txt
     |   |   |- nuc1xx/
     |   |   |   `- nutiny-nuc120/
     |   |   |       `- README.txt
     |   |   |- s32k1xx/
     |   |   |   |- s32k118evb/
     |   |   |   |   `- README.txt
     |   |   |   |- s32k146evb/
     |   |   |   |   `- README.txt
     |   |   |   `- s32k148evb/
     |   |   |       `- README.txt
     |   |   |- sam34/
     |   |   |   |- arduino-due/
     |   |   |   |   `- README.txt
     |   |   |   |- flipnclick-sam3x/
     |   |   |   |   `- README.txt
     |   |   |   |- sam3u-ek/
     |   |   |   |   `- README.txt
     |   |   |   |- sam4cmp-db/
     |   |   |   |   `- README.txt
     |   |   |   |- sam4e-ek/
     |   |   |   |   `- README.txt
     |   |   |   |- sam4l-xplained/
     |   |   |   |   `- README.txt
     |   |   |   |- sam4s-xplained/
     |   |   |   |   `- README.txt
     |   |   |   `- sam4s-xplained-pro/
     |   |   |       `- README.txt
     |   |   |- sama5/
     |   |   |   |- sama5d2-xult/
     |   |   |   |   `- README.txt
     |   |   |   |- giant-board/
     |   |   |   |   `- README.md
     |   |   |   |- sama5d3x-ek/
     |   |   |   |   `- README.txt
     |   |   |   |- sama5d3-xplained/
     |   |   |   |   `- README.txt
     |   |   |   `- sama5d4-ek/
     |   |   |       `- README.txt
     |   |   |- samd2l2/
     |   |   |   |- arduino-m0/
     |   |   |   |   `- README.txt
     |   |   |   |- samd20-xplained/
     |   |   |   |   `- README.txt
     |   |   |   |- samd21-xplained/
     |   |   |   |   `- README.txt
     |   |   |   `- saml21-xplained/
     |   |   |       `- README.txt
     |   |   |- samd5e5/
     |   |   |   `- metro-m4/
     |   |   |      `- README.txt
     |   |   |- samv7/
     |   |   |   |- same70-qmtech/
     |   |   |   |   `- README.txt
     |   |   |   |- same70-xplained/
     |   |   |   |   `- README.txt
     |   |   |   `- samv71-xult/
     |   |   |      `- README.txt
     |   |   |- stm32/
     |   |   |   |- axoloti/
     |   |   |   |   `- README.txt
     |   |   |   |- b-g474e-dpow1/
     |   |   |   |   `- README.txt
     |   |   |   |- clicker2-stm32/
     |   |   |   |   `- README.txt
     |   |   |   |- cloudctrl/
     |   |   |   |   `- README.txt
     |   |   |   |- emw3162/
     |   |   |   |   `- README.txt
     |   |   |   |- fire-stm32v2/
     |   |   |   |   `- README.txt
     |   |   |   |- hymini-stm32v/
     |   |   |   |   `- README.txt
     |   |   |   |- maple/
     |   |   |   |   `- README.txt
     |   |   |   |- mikroe-stm32f4/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-f103rb/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-f207zg/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-f302r8/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-f303re/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-f303ze/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-f334r8/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-f410rb/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-f446re/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-f4x1re/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-l152re/
     |   |   |   |   `- README.txt
     |   |   |   |- olimexino-stm32/
     |   |   |   |- olimex-stm32-e407/
     |   |   |   |   `- README.txt
     |   |   |   |- olimex-stm32-h405/
     |   |   |   |   `- README.txt
     |   |   |   |- olimex-stm32-h407/
     |   |   |   |   `- README.txt
     |   |   |   |- olimex-stm32-p107/
     |   |   |   |- olimex-stm32-p207/
     |   |   |   |   `- README.txt
     |   |   |   |- olimex-stm32-p407/
     |   |   |   |   `- README.txt
     |   |   |   |- omnibusf4/
     |   |   |   |   `- README.txt
     |   |   |   |- photon/
     |   |   |   |   `- README.txt
     |   |   |   |- shenzhou/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32_tiny/
     |   |   |   |   `- README.txt
     |   |   |   |- stm3210e-eval/
     |   |   |   |   `- README.txt
     |   |   |   |- stm3220g-eval/
     |   |   |   |   `- README.txt
     |   |   |   |- stm3240g-eval/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32butterfly2/
     |   |   |   |- stm32f103-minimum/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32f334-disco/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32f3discovery/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32f411e-disco/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32f429i-disco/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32f4discovery/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32ldiscovery/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32vldiscovery/
     |   |   |   |   `- README.txt
     |   |   |   `- viewtool-stm32f107/
     |   |   |       `- README.txt
     |   |   |- stm32f0l0g0/
     |   |   |   |- b-l072z-lrwan1/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-f072rb/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-f091rc/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-g070rb/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-g071rb/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-l073rz/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32f051-discovery/
     |   |   |   |   `- README.txt
     |   |   |   `- stm32f072-discovery/
     |   |   |       `- README.txt
     |   |   |- stm32f7/
     |   |   |   |- nucleo-144/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32f746g-disco/
     |   |   |   |   |- configs/fb/README.txt
     |   |   |   |   |- configs/nxdemo/README.txt
     |   |   |   |   |- configs/nxterm/README.txt
     |   |   |   |   `- README.txt
     |   |   |   |- stm32f746-ws/
     |   |   |   `- stm32f769i-disco/
     |   |   |       `- README.txt
     |   |   |- stm32h7/
     |   |   |   `- nucleo-h743zi/
     |   |   |       `- README.txt
     |   |   |- stm32l4/
     |   |   |   |- b-l475e-iot01a/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-l432kc/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-l452re/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-l476rg/
     |   |   |   |   `- README.txt
     |   |   |   |- nucleo-l496zg/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32l476-mdk/
     |   |   |   |   `- README.txt
     |   |   |   |- stm32l476vg-disco/
     |   |   |   |   `- README.txt
     |   |   |   `- stm32l4r9ai-disco/
     |   |   |       `- README.txt
     |   |   |- str71x/
     |   |   |   `- olimex-strp711/
     |   |   |       `- README.txt
     |   |   |- tiva/
     |   |   |   |- dk-tm4c129x/
     |   |   |   |   `- README.txt
     |   |   |   |- eagle100/
     |   |   |   |   `- README.txt
     |   |   |   |- ekk-lm3s9b96/
     |   |   |   |   `- README.txt
     |   |   |   |- launchxl-cc1310/
     |   |   |   |   `- README.txt
     |   |   |   |- launchxl-cc1312r1/
     |   |   |   |   `- README.txt
     |   |   |   |- lm3s6432-s2e/
     |   |   |   |   `- README.txt
     |   |   |   |- lm3s6965-ek/
     |   |   |   |   `- README.txt
     |   |   |   |- lm3s8962-ek/
     |   |   |   |   `- README.txt
     |   |   |   |- lm4f120-launchpad/
     |   |   |   |   `- README.txt
     |   |   |   |- tm4c123g-launchpad/
     |   |   |   |   `- README.txt
     |   |   |   `- tm4c1294-launchpad/
     |   |   |       `- README.txt
     |   |   |- tms570/
     |   |   |   |- launchxl-tms57004/
     |   |   |   |   `- README.txt
     |   |   |   `- tms570ls31x-usb-kit/
     |   |   |       `- README.txt
     |   |   `- xmc4/
     |   |       `- xmc4500-relax/
     |   |           `- README.txt
     |   |- avr/
     |   |   |- at32uc3/
     |   |   |   `- avr32dev1/
     |   |   |       `- README.txt
     |   |   |- at90usb/
     |   |   |   |- micropendous3/
     |   |   |   |   `- README.txt
     |   |   |   `- teensy-2.0/
     |   |   |       `- README.txt
     |   |   `- atmega/
     |   |       |- amber/
     |   |       |   `- README.txt
     |   |       |- arduino-mega2560/
     |   |       |   `- README.txt
     |   |       `- moteino-mega/
     |   |           `- README.txt
     |   |- hc/
     |   |   `- m9s12/
     |   |       |- demo9s12ne64/
     |   |       |   `- README.txt
     |   |       `- ne64badge/
     |   |           `- README.txt
     |   |- mips/
     |   |   |- pic32mx/
     |   |   |   |- mirtoo/
     |   |   |   |   `- README.txt
     |   |   |   |- pic32mx7mmb/
     |   |   |   |   `- README.txt
     |   |   |   |- pic32mx-starterkit/
     |   |   |   |   `- README.txt
     |   |   |   |- sure-pic32mx/
     |   |   |   |   `- README.txt
     |   |   |   `- ubw32/
     |   |   |       `- README.txt
     |   |   `-pic32mz/
     |   |       |- chipkit-wifire/
     |   |       |   `- README.txt
     |   |       |- flipnclick-pic32mz/
     |   |       |   `- README.txt
     |   |       `- pic32mz-starterkit/
     |   |           `- README.txt
     |   |- misoc/
     |   |   `- lm32/
     |   |       `- misoc/
     |   |           `- README.txt
     |   |- or1k/
     |   |   `- mor1kx/
     |   |       `- or1k/
     |   |           `- README.txt
     |   |- renesas/
     |   |   |- m16c/
     |   |   |   `- skp16c26/
     |   |   |       `- README.txt
     |   |   `-sh1/
     |   |       `- us7032evb1/
     |   |           `- README.txt
     |   |- risc-v/
     |   |- sim/
     |   |   `- sim/
     |   |       `- sim/
     |   |           |- include/README.txt
     |   |           `- README.txt
     |   |- x86/
     |   |   `- qemu/
     |   |       `- qemu-i486/
     |   |           `- README.txt
     |   |- xtensa/
     |   |   `- esp32/
     |   |       `- esp32-core/
     |   |           `- README.txt
     |   |- z16/
     |   |   `- z16f/
     |   |       `- z16f2800100zcog/
     |   |           |- configs/nsh/README.txt
     |   |           |- configs/ostest/README.txt
     |   |           |- configs/pashello/README.txt
     |   |           `- README.txt
     |   |- z80/
     |   |   |- ez80/
     |   |   |   |- ez80f910200kitg/
     |   |   |   |   |- configs/ostest/README.txt
     |   |   |   |   `- README.txt
     |   |   |   |- ez80f910200zco/
     |   |   |   |   |- configs/dhcpd/README.txt
     |   |   |   |   |- configs/httpd/README.txt
     |   |   |   |   |- configs/nettest/README.txt
     |   |   |   |   |- configs/nsh/README.txt
     |   |   |   |   |- configs/poll/README.txt
     |   |   |   |   `- README.txt
     |   |   |   |- makerlisp/
     |   |   |   |   |- configs/nsh_flash/README.txt
     |   |   |   |   |- configs/nsh_ram/README.txt
     |   |   |   |   |- configs/sdboot/README.txt
     |   |   |   |   `- README.txt
     |   |   |   `- z80x/
     |   |   |       |- configs/nsh_flash/README.txt
     |   |   |       |- configs/nsh_ram/README.txt
     |   |   |       |- configs/sdboot/README.txt
     |   |   |       `- README.txt
     |   |   |- z180/
     |   |   |   `- p112/
     |   |   |       `- README.txt
     |   |   |- z8/
     |   |   |   |- z8encore000zco/
     |   |   |   |   |- configs/ostest/README.txt
     |   |   |   |   `- README.txt
     |   |   |   `- z8f64200100kit/
     |   |   |       |- configs/ostest/README.txt
     |   |   |       `- README.txt
     |   |   `- z80/
     |   |       `- z80sim/
     |   |           `- README.txt
     |   `-README.txt
     |- drivers/
     |   |- eeprom/
     |   |   `- README.txt
     |   |- lcd/
     |   |   | README.txt
     |   |   `- pcf8574_lcd_backpack_readme.txt
     |   |- mtd/
     |   |   `- README.txt
     |   |- sensors/
     |   |   `- README.txt
     |   |- syslog/
     |   |   `- README.txt
     |   `- README.txt
     |- fs/
     |   |- binfs/
     |   |   `- README.txt
     |   |- cromfs/
     |   |   `- README.txt
     |   |- mmap/
     |   |   `- README.txt
     |   |- nxffs/
     |   |   `- README.txt
     |   |- smartfs/
     |   |   `- README.txt
     |   |- procfs/
     |   |   `- README.txt
     |   |- spiffs/
     |   |   `- README.md
     |   `- unionfs/
     |       `- README.txt
     |- graphics/
     |   `- README.txt
     |- libs/
     |   |- README.txt
     |   |- libc/
     |   |   |- zoneinfo
     |   |   |   `- README.txt
     |   |   `- README.txt
     |   |- libdsp/
     |   |   `- README.txt
     |   |- libnx/
     |   |   |- nxfongs
     |   |   |   `- README.txt
     |   |   `- README.txt
     |   |- libxx/
     |   `- README.txt
     |- mm/
     |   |- shm/
     |   |   `- README.txt
     |   `- README.txt
     |- net/
     |   |- sixlowpan
     |   |   `- README.txt
     |   `- README.txt
     |- pass1/
     |   `- README.txt
     |- syscall/
     |   `- README.txt
     `- tools/
         `- README.txt

Below is a guide to the available README files in the semi-optional apps/
source tree:

    apps/
     |- examples/
     |   |- bastest/README.txt
     |   |- json/README.txt
     |   |- pashello/README.txt
     |   `- README.txt
     |- gpsutils/
     |   `- minmea/README.txt
     |- graphics/
     |   |- tiff/README.txt
     |   `- traveler/tools/tcledit/README.txt
     |- interpreters/
     |   |- bas/
     |   |  `- README.txt
     |   |- ficl/
     |   |  `- README.txt
     |   `- README.txt
     |- modbus/
     |   `- README.txt
     |- netutils/
     |   |- discover/
     |   |  `- README.txt
     |   |- ftpc/
     |   |  `- README.txt
     |   |- json/
     |   |  `- README.txt
     |   |- telnetd/
     |   |  `- README.txt
     |   `- README.txt
     |- nshlib/
     |   `- README.txt
     |- NxWidgets/
     |   `- README.txt
     |- system/
     |   |- cdcacm/
     |   |  `- README.txt
     |   |- i2c/
     |   |  `- README.txt
     |   |- inifile/
     |   |  `- README.txt
     |   |- install/
     |   |  `- README.txt
     |   |- nsh/
     |   |  `- README.txt
     |   |- nxplayer/
     |   |  `- README.txt
     |   |- psmq/
     |   |  `- README.txt
     |   |- symtab/
     |   |   `- README.txt
     |   |- termcurses/
     |   |   `- README.txt
     |   |- usbmsc/
     |   |  `- README.txt
     |   `- zmodem/
     |      `- README.txt
     `- wireless
         |- bluetooth/
         |  `- btsak/
         |     `- README.txt
         `- ieee802154
            `- i8sak/
               `- README.txt

Additional README.txt files in the other, related repositories:

    NxWidgets/
     |- Doxygen
     |   `- README.txt
     |- tools
     |   `- README.txt
     |- UnitTests
     |   `- README.txt
     `- README.txt

    buildroot/
     `- README.txt

    tools/
     `- README.txt

    uClibc++/
     `- README.txt
