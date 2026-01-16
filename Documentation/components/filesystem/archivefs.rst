=========
ArchiveFS
=========

Overview
========

ArchiveFS is a read-only filesystem driver for NuttX RTOS that allows mounting
archive files (ZIP, 7ZIP, TAR, etc.) as virtual filesystems. It uses the
libarchive library to parse various archive formats and provides transparent
access to files within archives.

Features
========

- Read-only access: Extract and read files from archives without extracting to disk
- Multiple format support, including:

  - ZIP
  - 7ZIP
  - TAR
  - CPIO
  - AR
  - CAB
  - ISO9660
  - LHA
  - MTREE
  - RAR (versions 4 and 5)
  - RAW
  - WARC
  - XAR
  - EMPTY

- Directory traversal: List and navigate archive contents as directories
- File operations: Open, read, seek, and stat files within archives
- Memory efficient: Uses streaming access without full extraction
- Thread safe: Uses mutex for concurrent access protection
- Seek optimization: Supports forward seeking with efficient skipping

Benefits
========

1. Space efficient: No need to extract archives to disk
2. Memory efficient: Streaming access reduces RAM usage
3. Flexible: Supports 15+ archive formats
4. Easy to use: Standard POSIX-like file operations
5. Configurable: Enable only needed formats to save binary size

Configuration
=============

ArchiveFS is configured via Kconfig options.

Core Configuration
------------------

.. code-block:: bash

  CONFIG_FS_ARCHIVEFS=y

This enables ArchiveFS support.

Buffer Size
-----------

.. code-block:: bash

  CONFIG_FS_ARCHIVEFS_BUFFER_SIZE=32768

This buffer is used by libarchive to read and write data. Default is 32768 bytes.

Format Support
--------------

Enable All Formats
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

  CONFIG_FS_ARCHIVEFS_FORMAT_ALL=y

Enabling all formats increases binary size.

Enable Individual Formats
~~~~~~~~~~~~~~~~~~~~~~~~~

When ``CONFIG_FS_ARCHIVEFS_FORMAT_ALL`` is not set, you can enable formats
individually. ZIP is enabled by default.

.. code-block:: bash

  # ZIP (default enabled)
  CONFIG_FS_ARCHIVEFS_FORMAT_ZIP=y

  # Other formats (disabled by default)
  CONFIG_FS_ARCHIVEFS_FORMAT_7ZIP=n
  CONFIG_FS_ARCHIVEFS_FORMAT_AR=n
  CONFIG_FS_ARCHIVEFS_FORMAT_CAB=n
  CONFIG_FS_ARCHIVEFS_FORMAT_CPIO=n
  CONFIG_FS_ARCHIVEFS_FORMAT_EMPTY=n
  CONFIG_FS_ARCHIVEFS_FORMAT_ISO9660=n
  CONFIG_FS_ARCHIVEFS_FORMAT_LHA=n
  CONFIG_FS_ARCHIVEFS_FORMAT_MTREE=n
  CONFIG_FS_ARCHIVEFS_FORMAT_RAR=n
  CONFIG_FS_ARCHIVEFS_FORMAT_RAR_V5=n
  CONFIG_FS_ARCHIVEFS_FORMAT_RAW=n
  CONFIG_FS_ARCHIVEFS_FORMAT_TAR=n
  CONFIG_FS_ARCHIVEFS_FORMAT_WARC=n
  CONFIG_FS_ARCHIVEFS_FORMAT_XAR=n

Note: Enabling ``CONFIG_FS_ARCHIVEFS_FORMAT_ALL`` will include all format
support and significantly increase binary size. For production builds, enable
only the formats you actually need.

Dependencies
============

Required
--------

- libarchive: ``CONFIG_UTILS_LIBARCHIVE=y``

Optional Compression Libraries
------------------------------

Depending on the archive formats used:

- XZ compression: ``CONFIG_UTILS_XZ=y`` (for LZMA/XZ compressed archives)
- Zlib: ``CONFIG_LIB_ZLIB=y`` (for DEFLATE compressed archives)

Example Configuration
---------------------

.. code-block:: bash

  CONFIG_FS_ARCHIVEFS=y
  CONFIG_FS_ARCHIVEFS_FORMAT_ZIP=y
  CONFIG_FS_ARCHIVEFS_BUFFER_SIZE=32768
  CONFIG_UTILS_LIBARCHIVE=y
  CONFIG_LIB_ZLIB=y

Usage
=====

Basic Mounting
--------------

Archive files can be mounted at any mount point. The archive path is specified
using the ``-o`` option:

.. code-block:: bash

  nsh> mount -t archivefs -o /path/to/archive.zip /mnt

Where:

- ``-t archivefs`` specifies the filesystem type
- ``-o /path/to/archive.zip`` specifies the archive to mount
- ``/mnt`` is the mount point

Demo: Running ArchiveFS on QEMU
-------------------------------

This example uses the MPS3-AN547 board on QEMU.

1. Prepare the archive file on the host:

.. code-block:: bash

  dd if=/dev/urandom of=testfile.bin bs=1K count=10
  7z a -tzip -mm=LZMA archive.zip testfile.bin

2. Configure and build NuttX:

.. code-block:: bash

  cmake -B build -DBOARD_CONFIG=boards/arm/mps/mps3-an547/configs/archivefs
  cmake --build build

3. Start QEMU and load the archive into memory:

.. code-block:: bash

  qemu-system-arm -M mps3-an547 -m 2G -nographic \
    -kernel build/nuttx.bin \
    -gdb tcp::1127 \
    -device loader,file=archive.zip,addr=0x60000000

4. Use ArchiveFS in the NuttX shell:

.. code-block:: bash

  nsh> mount -t archivefs -o /dev/ram1 /archivefs
  nsh> ls -l /archivefs/
  -rw-r--r--        0     0     10240 archivefs  testfile.bin
  nsh> cat /archivefs/testfile.bin
  nsh> mount -t tmpfs tmp
  nsh> cp /archivefs/testfile.bin /tmp/testfile.bin
  nsh> ls -l /tmp/testfile.bin
  -rw-r--r--        0     0     10240 tmp        testfile.bin
  nsh> umount /archivefs

Working with Multiple Archives
------------------------------

.. code-block:: bash

  nsh> mount -t archivefs -o /path/to/archives/data.zip /data
  nsh> mount -t archivefs -o /path/to/archives/config.zip /config
  nsh> mount -t archivefs -o /path/to/archives/resources.zip /resources

  nsh> mount
  /data  archivefs  /path/to/archives/data.zip
  /config archivefs  /path/to/archives/config.zip
  /resources archivefs /path/to/archives/resources.zip

Working with Directory Structures
---------------------------------

.. code-block:: bash

  nsh> ls -R /archivefs
  /archivefs:
  dir1/
  dir2/
  file1.txt

  /archivefs/dir1:
  subdir1/
  file2.txt

  /archivefs/dir1/subdir1:
  file3.txt

  /archivefs/dir2:
  file4.txt

  nsh> cat /archivefs/dir1/subdir1/file3.txt

Supported Operations
====================

File Operations
---------------

``open()``

.. code-block:: c

  int fd = open("/archivefs/path/to/file.txt", O_RDONLY);

``read()``

.. code-block:: c

  ssize_t bytes_read = read(fd, buffer, sizeof(buffer));

``seek()``

.. code-block:: c

  off_t offset = lseek(fd, 0, SEEK_SET);
  offset = lseek(fd, 100, SEEK_CUR);
  offset = lseek(fd, 0, SEEK_END);

Seek behavior:

- Forward seeking uses a seek buffer to skip data efficiently.
- Backward seeking reopens the archive entry and is more expensive.

``stat()`` / ``fstat()``

.. code-block:: c

  struct stat st;
  int ret = stat("/archivefs/file.txt", &st);

``dup()``

.. code-block:: c

  int new_fd = dup(fd);

``close()``

.. code-block:: c

  close(fd);

Directory Operations
--------------------

.. code-block:: c

  DIR *dir = opendir("/archivefs");
  struct dirent *entry;
  while ((entry = readdir(dir)) != NULL)
    {
      printf("File: %s\n", entry->d_name);
    }
  closedir(dir);

Filesystem Operations
---------------------

``statfs()``

.. code-block:: c

  struct statfs fs;
  statfs("/archivefs", &fs);
  printf("Filesystem type: 0x%lx\n", fs.f_type);

``mount()`` / ``umount()``

.. code-block:: bash

  nsh> mount -t archivefs -o /path/to/archive.zip /mnt
  nsh> umount /mnt

Technical Implementation
========================

Architecture
------------

ArchiveFS integrates with the NuttX VFS layer and uses libarchive for parsing
archives. Key components include:

1. VFS interface (``g_archivefs_operations``)
2. Per-file private data structure (``archivefs_priv_s``)
3. Callback functions (read/seek/close)
4. Mutex-based synchronization for thread safety

Memory Management
-----------------

- Uses ``fs_heap`` for allocations
- Buffer size is configurable via ``CONFIG_FS_ARCHIVEFS_BUFFER_SIZE``
- Seek buffer is allocated on demand
- Resources are released in ``archivefs_free()``

Seek Optimization
-----------------

- Forward seeks skip data via a buffer without reopening the archive
- Backward seeks reopen the archive entry to re-position
- File position is tracked in ``filep->f_pos``

Format Support
--------------

.. list-table::
  :widths: 25 35 10 30
  :header-rows: 1

  * - Format
    - Kconfig Option
    - Default
    - Description
  * - ZIP
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_ZIP``
    - y
    - ZIP archives
  * - 7ZIP
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_7ZIP``
    - n
    - 7-Zip archives
  * - TAR
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_TAR``
    - n
    - TAR archives
  * - CPIO
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_CPIO``
    - n
    - CPIO archives
  * - AR
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_AR``
    - n
    - Unix AR archives
  * - CAB
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_CAB``
    - n
    - Microsoft CAB archives
  * - ISO9660
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_ISO9660``
    - n
    - ISO 9660 images
  * - LHA
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_LHA``
    - n
    - LHA/LZH archives
  * - MTREE
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_MTREE``
    - n
    - BSD mtree format
  * - RAR
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_RAR``
    - n
    - RAR archives (v4)
  * - RAR_V5
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_RAR_V5``
    - n
    - RAR archives (v5)
  * - RAW
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_RAW``
    - n
    - Raw data
  * - WARC
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_WARC``
    - n
    - Web ARChive format
  * - XAR
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_XAR``
    - n
    - Extensible Archive Format
  * - EMPTY
    - ``CONFIG_FS_ARCHIVEFS_FORMAT_EMPTY``
    - n
    - Empty archives

Error Handling
--------------

ArchiveFS converts libarchive errors to standard errno values:

.. list-table::
  :widths: 25 15 60
  :header-rows: 1

  * - libarchive Code
    - errno
    - Description
  * - ``ARCHIVE_RETRY``
    - ``EAGAIN``
    - Operation should be retried
  * - ``ARCHIVE_WARN``
    - ``ENOEXEC``
    - Warning (non-fatal)
  * - ``ARCHIVE_FAILED``
    - ``EINVAL``
    - Operation failed
  * - ``ARCHIVE_FATAL``
    - ``EPERM``
    - Fatal error

Example Use Cases
=================

OTA Updates
-----------

Deliver OTA updates as compressed archives:

.. code-block:: bash

  nsh> wget -o /tmp/ota_update.zip https://example.com/updates/latest.zip
  nsh> mount -t archivefs -o /tmp/ota_update.zip /ota
  nsh> cat /ota/version.txt
  nsh> copy /ota/firmware.bin /dev/flash0

Troubleshooting
===============

Mount Fails with "No such device"
---------------------------------

Cause: Archive file not found or inaccessible.

.. code-block:: bash

  nsh> ls -l /path/to/archive.zip
  nsh> cat /path/to/archive.zip > /dev/null

Cannot Open Files in Archive
----------------------------

Cause: File format not supported.

Solution:

- Enable the required ``CONFIG_FS_ARCHIVEFS_FORMAT_*`` option
- For testing, enable ``CONFIG_FS_ARCHIVEFS_FORMAT_ALL``

Out of Memory Errors
--------------------

Cause: Buffer size too large or insufficient RAM.

.. code-block:: bash

  CONFIG_FS_ARCHIVEFS_BUFFER_SIZE=16384

Slow Performance
----------------

Cause: Frequent backward seeks or small buffer size.

Solution:

- Increase buffer size if RAM allows
- Minimize backward seeks in application code
- Consider caching frequently accessed files

Debugging
---------

Enable debugging options:

.. code-block:: bash

  CONFIG_DEBUG_FS=y
  CONFIG_DEBUG_FS_ERROR=y
  CONFIG_DEBUG_FEATURES=y

References
==========

- libarchive: https://libarchive.org/
- NuttX Filesystems: https://nuttx.apache.org/docs/latest/components/filesystem/index.html
- VFS Interface: https://nuttx.apache.org/docs/latest/components/filesystem/index.html#vfs-interface
