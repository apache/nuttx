=============================
Running Applications from NSH
=============================

This page describes current and planned features for the NuttX NuttShell (NSH).
Think of this page as a roadmap. Most of the features have been implemented and
incorporated into NSH; others are just ideas that are being tossed around (noted
as NOT implemented). The NSH document will be updated as new features are included.

There are currently be three ways to execute applications from NSH:

#. Executing Builtin Applications

   **Builtin Applications**
   The current implementation of NSH allows for the execution
   of "named" or "builtin" applications. The applications are simply task entry points
   in the common FLASH image that have names assigned to them. Simply entering that name
   from the NSH command line will cause the associated application to run.

   See the NSH documentation for further details.

   **Example**

   For example:

     .. code-block:: bash

        NuttShell (NSH) NuttX-6.24
        nsh> hello
        Hello, World!!
        nsh>

   **Configuration Settings**

   This functionality depends on these configuration settings:

     * ``CONFIG_BUILTIN=y`` Enables NuttX support for builtin applications, and
     * ``CONFIG_NSH_BUILTIN_APPS=y`` Enables NSH support for builtin applications

   In additional to other configuration needed by NSH.

#. Executing Application from a File System

   **OS Interfaces**

   NuttX supports running applications that reside on a file system as well.
   The standard interfaces used to do this include:

     * ``execv()``
     * ``execl()``
     * ``posix_spawn()``


   **Configuration Settings**

   Execution of applications on a file system is currently supported by Nuttx.
   This feature is enabled with:

     * ``CONFIG_LIBC_EXECFUNCS=y`` Enable support for ``execv()``, ``execl()``,
       and ``posix_spawn()``, and
     * ``CONFIG_NSH_FILE_APPS=y`` Enable execution of programs in a file system by NSH.

   **Example**

   When this feature is enabled, you will be able to do the following:

   .. code-block:: bash

      NuttShell (NSH) NuttX-6.24
      nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard
      nsh> ls -l /mnt/sdcard
      /mnt/sdcard:
      -rw-r--r--  198 hello
      nsh> /mnt/sdcard/hello
      Hello, World!!
      nsh>

   **The PATH Variable**

   Notice that the absolute path to the hello program was used. This can be
   simplified by setting the following in your configuration:

     * ``CONFIG_BINFMT_EXEPATH=y`` Enable use of the path variable to find executable programs.


   **Example**

   Then, the example becomes:

   .. code-block:: bash

      NuttShell (NSH) NuttX-6.24
      nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard
      nsh> ls -l /mnt/sdcard
      /mnt/sdcard:
        -rw-r--r--  198 hello
      nsh> set PATH /mnt/sdcard
      nsh> hello
      Hello, World!!
      nsh>

   **Pre-initialized PATH Variables**

   One final simplification: The initial PATH variable can be configured so that
   the PATH to your executable programs is already defined when NSH starts. This
   is down with:

     * ``CONFIG_PATH_INITIAL="/mnt/sdcard"``

   This functionality is already in place.

#. Executing Builtin Applications from a File System

   **Namespace Management**

   NuttX supports a virtual file system (VFS) that may used to manage namespaces.
   Linux uses its VFS to manage nearly all named objects (pipes, locks, message
   queues, etc.). And it is a good policy for NuttX to do this too. NuttX already
   does this for things like drivers and mountpoints. Why not extend this mechanism
   to handle named, builtin applications too?

   **Proposed Configuration Options**

   The following configuration options are planned:

     * ``CONFIG_BUILTIN=y`` Will still be needed to enable NuttX support for
       builtin applications, and
     * ``CONFIG_NSH_BUILTIN_APPS=y`` Will still be needed enable to stimulate
       programs to register themselves as builtin applications.

   And in addition:

     * ``CONFIG_FS_BINFS=y`` Enable the BINFS file system that is used to access
       builtin applications as files,
     * ``CONFIG_LIBC_EXECFUNCS=y`` Enable support for ``execv()``, ``execl()``, and
       ``posix_spawn()``,
     * ``CONFIG_NSH_FILE_APPS=y`` Enable execution of programs in a file system by NSH,
     * ``CONFIG_BINFMT_EXEPATH=y`` Enable the (optional) use of the path variable to
       find executable programs, and
     * ``CONFIG_PATH_INITIAL="/mnt/sdcard:/bin"`` Optional initial value of the
       PATH variable.

   **Example**

   When this feature is enable, will will be able to do this (where myapplication is
   the name of some, arbitrary "built-in" application):

   .. code-block:: bash

      NuttShell (NSH) NuttX-6.24
      nsh> mount -t binfs /bin
      nsh> ls -l /bin
      /bin:
        -rw-r--r--  0 myapplication
      nsh> echo $PATH
      /mnt/sdcard:/bin
      nsh> myapplication
      ... and your FLASH based application runs ...
      nsh>

   **Auto-Mounting BINFS**

   BINFS, like any file system, could be mounted by an startup script
   at ``/etc/init.d/rcS``.

   But since BINFS may be used in many configurations that don't otherwise
   need a startup script, perhaps some configurations would be helpful:

     * ``CONFIG_NSH_AUTOMOUNT_BINFS=y`` Automatically mount the BINFS file
       system on startup (**NOT implemented**)
     * ``CONFIG_NSH_BINFS_MOUNTPOINT="/bin"`` The BINFS mountpoint (NOT implemented).

   Or perhaps a better thing to do would just be to make it easier to add a
   startup script?

   **Next Steps**

   In the longer term, I would like to see an option to move most of the larger
   NSH commands out of RAM and built them as standalone programs that can reside,
   for example, on an SD card (**NOT implemented**).
