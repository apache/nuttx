=================
NuttX File System
=================

NuttX includes an optional, scalable file system. This file-system may be
omitted altogether; NuttX does not depend on the presence of any file system.

.. _root_fs:

**Pseudo Root File System**. A simple *in-memory*, *pseudo* file
system can be enabled by default. This is an *in-memory* file
system because it does not require any storage medium or block
driver support. Rather, file system contents are generated
on-the-fly as referenced via standard file system operations
(open, close, read, write, etc.). In this sense, the file system
is *pseudo* file system (in the same sense that the Linux
``/proc`` file system is also referred to as a pseudo file
system).

Any user supplied data or logic can be accessed via the
pseudo-file system. Built in support is provided for character and
block drivers in the ``/dev`` pseudo file system directory.

**Mounted File Systems** The simple in-memory file system can be
extended my mounting block devices that provide access to true
file systems backed up via some mass storage device. NuttX
supports the standard ``mount()`` command that allows a block
driver to be bound to a mountpoint within the pseudo file system
and to a file system. At present, NuttX supports the standard VFAT
and ROMFS file systems, a special, wear-leveling NuttX FLASH File
System (NXFFS), as well as a Network File System client (NFS
version 3, UDP).

**Comparison to Linux** From a programming perspective, the NuttX
file system appears very similar to a Linux file system. However,
there is a fundamental difference: The NuttX root file system is a
pseudo file system and true file systems may be mounted in the
pseudo file system. In the typical Linux installation by
comparison, the Linux root file system is a true file system and
pseudo file systems may be mounted in the true, root file system.
The approach selected by NuttX is intended to support greater
scalability from the very tiny platform to the moderate platform.

Virtual File System (VFS)
=========================

Virtual File System provides a unified interface for various file systems to
be able to co-exist together by exposing a blueprint that each file system
needs to implement. This also allows the file system to be free from worry
about the device driver implementations for storage devices, as they also
expose a unified way of accessing the underlying devices.

How VFS works
-------------

Threads are controllable sequences of instruction execution with their own
stacks. Each task in NuttX is represented by a Task Control Block (TCB) (TCB
is defined in ``include/nuttx/sched.h``) and tasks are organized in task
lists.

All threads that are created by ``pthread_create()`` are part of the same task
group. A task group (defined in ``include/nuttx/sched.h``) is a shared
structure pointed to by the TCBs of all the threads that belong to the same
task group, and this task group contains all the resources shared across the
task group which includes *file descriptors* in the form of a **file list**.

A file list (defined in ``include/nuttx/fs/fs.h``) contains file structures
that denote open files (along with a spinlock to manage access to the file
list). With the devices listed in the :ref:`root file system <root_fs>` (on
points like ``/dev/led``, ``/dev/mmcsd0``, etc. which are henceforth called
blockdriver mount points) in an unmounted state, storage devices can be
mounted using the ``mount()`` command (to any point like ``/dir/abcd``) with
any specific supported file system, which internally calls its implemented
``mountpt_operations->bind()`` method and passes the blockdriver's mount
point inode to it, thus creating a **mount point**. The blockdriver mount
point inode will have a ``mountpt->i_private`` which contains any (file system
dependent) information about the mount and is to be filled by the file system
during the execution of ``mountpt_operations->bind()`` (and usually this data
includes a pointer to the blockdriver mount point as well). After that,
according to system calls, the other exposed functions of the filesystem
are called as per need.

VFS Interface
-------------

VFS allows file systems to expose their own implementations of methods
belonging to a unified interface:

* **File operations**

.. c:function:: int open(FAR struct file *filep, FAR const char *relpath, int oflags, mode_t mode)
  
  Opens a file. Files are required to be opened before any other file
  operations are performed on it.

  :param FAR struct file * filep: Open file's file structure pointer. The
    ``filep->f_priv`` member needs to be set here with the file system
    specific data that represents an open file.
  :param FAR const char * relpath: Relative path of the file from the root of
    the mounted file system.
  :param int oflags: Flags in a bit field that specify the mode for openning
    the file (eg. ``O_RDONLY``, ``O_RDWR``, etc. defined in
    ``include/fcntl.h``).
  :param mode_t mode: Specifies the mode (permissions). If ``oflags`` include
    ``O_CREAT``, then this contains the mode for the file to be created.
  :returns: Status of openning a file.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int close(FAR struct file *filep)

  This closes the opened file, and ideally syncs all the changes to the file
  to be written to the disk, as well as free the memory allocated to store the
  open file's data.

  :param FAR struct file * filep: Open file's file structure pointer.
  :returns: Status of closing a file.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: ssize_t read(FAR struct file *filep, FAR char *buffer, size_t buflen)
  
  Reads maximum ``buflen`` bytes from an opened file (from the current offset
  the opened file descriptor is pointing at if the file system supports
  seeking).

  :param FAR struct file * filep: Open file's file structure pointer.
  :param FAR char * buffer: Buffer to store the read data.
  :param size_t buflen: Length of the maximum number of bytes to be read.
  :returns: Number of bytes read.
  :retval > 0: Size of bytes read.
  :retval < 0: Error.

.. c:function:: ssize_t write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
  
  Writes maximum ``buflen`` bytes to an opened file (from the current offset
  the opened file is at if the file system supports seeking).

  :param FAR struct file * filep: Open file's file structure pointer.
  :param FAR char * buffer: Buffer which contains the data to be written.
  :param size_t buflen: Length of the maximum number of bytes to be written.
  :returns: Number of bytes written.
  :retval > 0: Size of bytes written.
  :retval < ``buflen``: Insufficient storage or file size limit reached
  :retval < 0: Error.

.. NOTE::
  POSIX requires that a ``read()`` after a ``write()`` should get the newly
  written data, but not all file systems conform to POSIX, especially as
  POSIX requires atomic writes, which is not usually implemented
  as it can impact performance.

  To be POSIX compliant in concurrent situations, either the writes have to
  be atomic, or read is blocked with a lock until an on-going write is
  finished, which, as stated, would impact performance.

.. c:function:: off_t seek(FAR struct file *filep, off_t offset, int whence)

  Underlying implementation of ``lseek()``, it allows the open file's file
  structure to point to any particular location in the file.

  :param FAR struct file * filep: Open file's file structure pointer.
  :param off_t offset: The offset required.
  :param int whence: This controls how the offset it applied. It can have
    values (defined in ``/include/sys/types.h``):
    
    * SEEK_SET: Offset from start of file.
    * SEEK_CUR: Offset from current location in file.
    * SEEK_END: Offset *after* end of file.

.. NOTE::

  According to POSIX, ``lseek()`` to any point after the end of the file
  *does not* by itself increase the size of the file. Later writes to this
  part will, however, increase it to at least the end of the written data, and
  the "gap" before this written data should be filled with ``\0`` in case of
  any reads after such a write operation.

.. c:function:: int ioctl(FAR struct file *filep, int cmd, unsigned long arg)

  It is the underlying implementation of ``ioctl()`` (I/O Control).
  ``ioctl()`` manipulates the underlying device parameters of files.

  :param FAR struct file * filep: Open file's file structure pointer.
  :param int cmd: It can take a variety of values (which are defined in
    ``include/nuttx/fs/ioctl.h``). It represents the command that will be
    carried out on the file. Both the filesystem, as well as the device driver
    needs to support the command in order for the function to run.
  :param unsigned long arg: Additional argument that may be required for
    ioctl. Details for what is required is written in the comments beside
    the desired ioctl command in ``include/nuttx/fs/ioctl.h``.
  :returns: Status of ioctl operation.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int mmap(FAR struct file *filep, FAR struct mm_map_entry_s *map)

  Underlying implementation of ``mmap()``. ``mmap()`` creates a new mapping
  in the virtual address space of the calling process.

  :param FAR struct file * filep: Open file's file structure pointer.
  :param FAR struct mm_map_entry_s * map: mmap entry strucutre pointer, which
    includes the virtual address.
  :returns: Status of mmap operation.
  :retval OK (0): Success.
  :retval < 0: Error.

.. NOTE::
  NuttX operates in a flat open address space. Therefore, it generally does
  not require ``mmap()`` functionality. There are two notable exceptions where
  ``mmap()`` functionality is required:

  1. ``mmap()`` is the API that is used to support direct access to random
     access media under the following very restrictive conditions:
    
    a. The filesystem implements the mmap file operation.  Any file
       system that maps files contiguously on the media should support
       this ioctl. (vs. file system that scatter files over the media
       in non-contiguous sectors).  As of this writing, ROMFS is the
       only file system that meets this requirement.

    b. The underlying block driver supports the BIOC_XIPBASE ioctl
       command that maps the underlying media to a randomly accessible
       address. At  present, only the RAM/ROM disk driver does this.
       
  2. If CONFIG_FS_RAMMAP is defined in the configuration, then mmap() will
     support simulation of memory mapped files by copying files whole
     into RAM.

.. c:function:: int truncate(FAR struct file *filep, off_t length)

  Shrinks or expands the file to be of the desired size.

  :param FAR struct file * filep: Open file's file structure pointer.
  :param off_t length: Final size of the file.
  :returns: Status of truncate operation.
  :retval OK (0): Success.
  :retval < 0: Error.


.. c:function:: int poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)

  Underlying implementation of ``poll()``. The ``poll()`` function provides
  applications with a mechanism for multiplexing input/output over a set of
  file descriptors.

  :param FAR struct file * filep: Open file's pointer.
  :param FAR struct pollfd * fds: The structure describing the events to be
    monitored, OR NULL if this is a request to stop monitoring events.
  :param bool setup: true: Setup up the poll; false: Teardown the poll
  :returns: Status of poll operation.
  :retval OK (0): Success.
  :retval < 0: Error.

* **Additional open file specific operations**

.. c:function:: int sync(FAR struct file *filep)

  This synchronizes the on-disk file system state of the file with the
  in-memory file system state, ie. commits the file system caches to the disk.

  :param FAR struct file * filep: Open file's ``struct file`` (defined in 
    ``include/nuttx/fs/fs.h``) pointer.
  :returns: Status of syncing a file.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int dup(FAR const struct file *oldp, FAR struct file *newp)

  Duplicate an open file structure.
  
  :param FAR const struct file * oldp: Pointer to structure that is to be
    duplicated.
  :param FAR struct file * newp: Pointer to structure in which the duplicate
    data will be stored.
  :returns: Status of duplicating open file's structure.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int fstat(FAR const struct file *filep, FAR struct stat *buf)
  
  Obtain information about an open file.

  :param FAR const struct file * filep: Open file's pointer.
  :param FAR struct stat * buf: Pointer to the ``struct stat`` (defined in
    ``include/sys/stat.h``).
  :returns: Status of obtaining open file's information.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int fchstat(FAR const struct file *filep, FAR const struct stat *buf, int flags)

  Change file stats. It can change the mode, timestamps and ownership.

  :param FAR struct file * filep: Open file's pointer.
  :param FAR const struct stat * buf: Pointer to stat structure describing
    the values that need to be updated.
  :param int flags: Bit field that can include (defined in
    ``include/nuttx/fs/fs.h``):
  
    * ``CH_STAT_MODE``
    * ``CH_STAT_UID``
    * ``CH_STAT_GID``
    * ``CH_STAT_ATIME``
    * ``CH_STAT_MTIME``
    
    This describes what needs to be updated.
  :returns: Status of changin open file's stats.
  :retval OK (0): Success.
  :retval < 0: Error.

* **Directory operations**

.. c:function:: int opendir(FAR struct inode *mountpt, FAR const char *relpath, FAR struct fs_dirent_s **dir)

  Opens a directory stream for the provided directory. Other directory
  operations can be used after this to do various directory related operations
  . We say the directory stream points to the first entry, but you need
  ``readdir()`` to read the first entry.

  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :param FAR const char * relpath: Relative path from the root of the point
    point of the directory.
  :param FAR struct fs_dirent_s ** dir: A directory stream structure pointer
    which needs to be populated with the required fields (defined in
    ``include/nuttx/fs/fs.h``).
  :returns: Status of openning the directory.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int closedir(FAR struct inode *mountpt, FAR struct fs_dirent_s *dir)

  Closes a directory stream, as well as deallocates any memory used while
  while openning a directory stream.

  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :param FAR struct fs_dirent_s ** dir: A directory stream structure pointer
    which was previously allocated (and needs to be freed).
  :returns: Status of closing the directory.
  :retval OK (0): Success.

.. c:function:: int readdir(FAR struct inode *mountpt, FAR struct fs_dirent_s *dir, FAR struct dirent *entry)

  This reads the next directory entry in a directory stream. If the stream
  points to the base of the directory, then the first directory entry in the
  directory is given.

  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :param FAR struct fs_dirent_s ** dir: A directory stream structure pointer.
  :param FAR struct dirent * entry: Pointer to the directory entry. This will
    be modified to point to the directory entry after it in the directory.
  :returns: Status of reading the directory.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int rewinddir(FAR struct inode *mountpt, FAR struct fs_dirent_s *dir)

  Resets the directory stream back to the first entry, like it was after
  openning.

  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :param FAR struct fs_dirent_s ** dir: A directory stream structure pointer.
  :returns: Status of rewinding the directory.
  :retval OK (0): Success.
  :retval < 0: Error.

* **Volume-relations operations**

.. c:function:: int bind(FAR struct inode *blkdriver, FAR const void *data, FAR void **handle)

  This is where the file system related data is initialized, and
  is part of the mount process.

  :param FAR struct inode * blkdriver: Pointer to the block driver's device
    inode. This needs to be opened in this function.
  :param FAR const void * data: The options provided during mount.
  :param FAR void ** handle: Whatever data ``handle`` points to is attached
    to the ``mountpt`` inode after this function is called during the mount
    process. This way, this file system's other methods can receive
    this information if they have access to ``mountpt`` inode, by accessing
    ``mountpt->i_private``.
  :returns: Status of binding operation.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int unbind(FAR void *handle, FAR struct inode **blkdriver, unsigned int flags)

  This is part of the unmounting process. The file system first
  needs to assess the flags passed to it and appropriately do the tasks
  required by these flags, and then it needs to free the private data
  (``handle`` and any allocated members), as well as close the
  previously-opened (during mount) block driver's inode.

  :param FAR void * handle: Private data of the file-system.
  :param FAR struct inode ** blkdriver: The device inode of the block driver's
    device inode.
  :param unsigned int flags: Flags dictate the actions needed to be carried
    out before the file system data is removed and the block driver inode is
    closed. The values can be (as defined in ``include/sys/mount.h``):
    
    * ``MNT_FORCE``
    * ``MNT_DETACH``
    * ``MNT_EXPIRE``
    * ``UMOUNT_NOFOLLOW``

  :returns: Status of unbinding operation.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int statfs(FAR struct inode *mountpt, FAR struct statfs *buf)

  Provides stats for that instance of the file system. The exact
  stats that are provided can be viewed in the members of
  ``struct statfs`` (in file ``include/sys/statfs.h``).

  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :param FAR struct statfs * buf: Buffer that needs to be filled with the
    relevant file system information.
  :returns: Status of finding the filesystem stats operation.
  :retval OK (0): Success.
  :retval < 0: Error.


* **Path operations**

.. c:function:: int unlink(FAR struct inode *mountpt, FAR const char *relpath)

  Removes a file, specifically, removes a name from the file system.

  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :param FAR const char * relpath: The relative path of the file from the root
    of the file system.
  :returns: Status of unlinking operation.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int mkdir(FAR struct inode *mountpt, FAR const char *relpath, mode_t mode)

  Creates a directory.

  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :param FAR const char * relpath: Relative path of the new directory from the
    root of the file system.
  :param mode_t mode: The mode (permissions) for the directory.
  :returns: Status of creating a directory operation.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int rmdir(FAR struct inode *mountpt, FAR const char *relpath)

  Removes a directory.

  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :param FAR const char * relpath: Relative path of the directory from the
    root of the file system.
  :returns: Status of removing a directory operation.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int rename(FAR struct inode *mountpt, FAR const char *oldrelpath, FAR const char *newrelpath)

  Renames a file or a directory

  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :param FAR const char * oldrelpath: Existing path of the file or directory.
  :param FAR const char * newrelpath: New path of the file or directory.
  :returns: Status of renaming a file or a directory operation.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int stat(FAR struct inode *mountpt, FAR const char *relpath, FAR struct stat *buf)

  Information about a file or a directory. The exact information that is
  provided can be viewed in the members of ``struct stat``
  (in file ``include/sys/stat.h``).

  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :param FAR const char * relpath: Relative path of the file or directory from
    the root of the file system.
  :param FAR struct stat * buf: Buffer that needs to be filled with the
    relevant file or directory information.
  :returns: Status of finding information about a file or directory.1 operation.
  :retval OK (0): Success.
  :retval < 0: Error.

.. c:function:: int chstat(FAR struct inode *mountpt, FAR const char *relpath, FAR const struct stat *buf, int flags)

  Change the stats of a file or directory.

  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :param FAR const char * relpath: Relative path of the file or directory from
    the root of the file system.
  :param FAR const struct stat * buf: Contains the new stat information. Access
    only the ones that are required according to the flags.
  :param int flags: A bit field that can have values including ``CH_STAT_MODE``
    , ``CH_STAT_UID``, ``CH_STAT_GID``, ``CH_STAT_ATIME`` or ``CH_STAT_MTIME``
    which are or-ed together.

.. c:function:: int syncfs(FAR struct inode *mountpt)

  This works like ``sync()`` but instead of the file, it syncs the entire
  filesystem's metadata.
  
  :param FAR struct inode * mountpt: Mount point inode of the file system.
  :returns: Status of syncing file system metadata operation.
  :retval OK (0): Success.
  :retval < 0: Error.


The file systems can have their own implementations for these functions
under-the-hood, but the user does not have to worry about the underlying file
system during file I/O, as the file system has to expose its implementations
in a unified interface.

.. NOTE::
  Each file system has to globally expose their implementations of the unified
  interface as defined by ``struct mountpt_operations`` (in
  ``include/fs/fs.h``) to one of the lists defined in ``fs/mount/fs_mount.c``
  depending on the type of the file system.

  They also need their own `magic number <https://en.wikipedia.org/wiki/Magic_number_(programming)>`_
  to be listed in ``include/sys`` and in ``fs_gettype`` function (in
  ``fs/mount/fs_gettype.c``) for identification of the filesystem.


File systems
============

NuttX provides support for a variety of file systems out of the box.

.. toctree::
  :maxdepth: 1

  aio.rst
  binfs.rst
  cromfs.rst
  fat.rst
  hostfs.rst
  littlefs.rst
  mmap.rst
  mnemofs.rst
  nfs.rst
  nxffs.rst
  partition.rst
  procfs.rst
  romfs.rst
  rpmsgfs.rst
  smartfs.rst
  shmfs.rst
  spiffs.rst
  tmpfs.rst
  unionfs.rst
  userfs.rst
  zipfs.rst

FS Categories
-------------

File systems can be divided into these categories on the basis of the drivers
they require:

1. They require a block device driver. They include vfat, romfs, smartfs, and
   littlefs.
2. They require MTD drivers. They include romfs, spiffs, littlefs.
3. They require neither block nor MTD drivers. They include nxffs, tmpfs, nfs
   binfs, procfs, userfs, hostfs, cromfs, unionfs, rpmsgfs, and zipfs.

The requirements are specified by declaring the filesystem in the proper
array in ``fs/mount/fs_mount.c``.