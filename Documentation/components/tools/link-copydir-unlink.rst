.. _build_system_linking:

============================================================
``link.[sh|bat]``, ``copydir.[sh|bat]``, ``unlink.[sh|bat]``
============================================================

Different file systems have different capabilities for symbolic links.
Some Windows file systems have no native support for symbolic links.
Cygwin running under Windows has special links built in that work with
all cygwin tools.  However, they do not work when Windows native tools
are used with cygwin.  In that case something different must be done.

If you are building under Linux or under cygwin with a cygwin tool
chain, then your Make.defs file may have definitions like the
following::

    DIRLINK = $(TOPDIR)/tools/link.sh
    DIRUNLINK = (TOPDIR)/tools/unlink.sh

The first definition is not always present because link.sh is the
default.  link.sh is a bash script that performs a normal, Linux-style
symbolic link;  unlink.sh is a do-it-all unlinking script.

But if you are building under cygwin using a Windows native toolchain
within a POSIX framework (such as Cygwin), then you will need something
like the following in you Make.defs file::

    DIRLINK = $(TOPDIR)/tools/copydir.sh
    DIRUNLINK = (TOPDIR)/tools/unlink.sh

copydir.sh will copy the whole directory instead of linking it.

Finally, if you are running in a pure native Windows environment with
a CMD.exe shell, then you will need something like this::

    DIRLINK = $(TOPDIR)/tools/copydir.bat
    DIRUNLINK = (TOPDIR)/tools/unlink.bat

Note that this will copy directories.  link.bat might also be used in
this case.  link.bat will attempt to create a symbolic link using the
NTFS mklink.exe command instead of copying files.  That logic, however,
has not been verified as of this writing.
