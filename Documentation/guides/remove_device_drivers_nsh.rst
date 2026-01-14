================================
Removing Device Drivers with NSH
================================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Removing+Device+Drivers+with+NSH

NuttX and Unix-like Operating Systems Compared
==============================================

There are many things that are called device drivers. In this context, the
discussion is limited to **character device drivers**. In NuttX, character
device drivers are represented by device driver nodes in the top-level :doc:`pseudo
filesystem </components/filesystem/pseudofs>`.

Standard Unix-like operating systems also support device driver nodes, which
superficially resemble NuttX device driver nodes: Both look like files and
usually reside under the top-level ``/dev`` directory. Both can be accessed
with standard POSIX file system commands such as ``open()``, ``close()``,
``read()``, ``write()``, and so forth. However, the similarity ends there.

The payload of a standard Unix-like operating system device driver node is a
device major and minor number. These major and minor device numbers are used to
look up the actual device driver interface using internal OS logic and data
structures. A NuttX device node, by contrast, directly holds the device driver
interface with no intervening lookup. This design is less flexible, but it is
more efficient and conserves limited resources in an embedded system.

In standard Unix-like operating systems, the device node can simply be deleted
using the shell command ``rm`` or the programmatic interface ``unlink()``. The
node is removed, and nothing special happens to the underlying device driver
(except that it may no longer be accessible).

In NuttX, if the device node were removed in the same way, the entire device
interface would also be removed, effectively breaking the driver. Internally,
NuttX supports a function called ``unregister_driver()`` that can be invoked
to remove a device driver. Therefore, removing the device driver node must
behave as though ``unregister_driver()`` were called.

The unlink() Method
===================

How is this accomplished in NuttX? It is done via a special device driver
method called ``unlink()``.

NuttX device drivers are implemented via a vtable of function pointers. That
vtable defines the interface between the pseudo-file system and the device
driver. This vtable is the structure ``struct file_operations`` defined in
``[nuttx]/include/nuttx/fs/fs.h``. It provides several interfaces that closely
match the standard POSIX interfaces—``open()``, ``close()``, ``read()``,
``write()``, etc.—and also includes a method called ``unlink()``. This
``unlink()`` method is called by the NuttX VFS when a user removes a device
driver node.

.. note::

   Removal of device driver nodes is only permitted if
   ``CONFIG_DISABLE_PSEUDOFS_OPERATIONS`` is **not** defined. All pseudo-file
   system operations may be suppressed to reduce the FLASH footprint in systems
   with extremely limited resources.

Removing a Device Node from NSH
===============================

Below is a summary of what happens when a device node is deleted using the NSH
``rm`` command:

1. The user enters the ``rm`` command. The NSH parser recognizes the command
   and transfers control to the NSH function ``cmd_rm()``.

2. ``cmd_rm()`` verifies the command, then calls the standard POSIX
   ``unlink()`` interface. The logic in the VFS ``unlink()`` function in
   ``[nuttx]/fs/vfs/fs_unlink.c`` is then executed.

3. The VFS ``unlink()`` detects that the target to be removed is a device node
   in the top-level pseudo-file system. It calls the device driver's
   ``unlink()`` method. It also removes the device node from the
   pseudo-filesystem. However, the underlying resources required to support
   the device driver interface may remain until the device driver frees those
   resources.

4. When the device driver's ``unlink()`` method is called, it determines if
   the device resources can be freed immediately. If so, it frees those
   resources. If, for example, there are still open references to the device
   driver, it may defer freeing the resources until the last client has closed
   the device driver and there are no open references. In such a case, it may
   set a flag indicating that the device driver has been unlinked.

5. If freeing of device driver resources has been deferred, that flag will be
   examined later. For instance, when the last client of the device driver
   closes its reference to the driver, it checks whether the unlink operation
   was deferred. If so, it frees any remaining device driver resources at that
   time.

.. warning::

   Some character device driver instances do not implement the ``unlink()``
   method. If problems arise when attempting to remove character drivers as
   described in this Wiki page, a missing ``unlink()`` method is the most
   likely cause.
