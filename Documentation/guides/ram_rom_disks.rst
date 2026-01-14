=======================
RAM Disks and ROM Disks
=======================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/RAM+Disks+and+ROM+Disks

NSH mkrd Command
================

The typical way to create a RAM disk is by using the NuttShell (NSH) ``mkrd``
command. The syntax is:

.. code-block:: shell

   mkrd [-m <minor>] [-s <sector-size>] <nsectors>

This command creates a RAM disk consisting of ``<nsectors>`` sectors, each of
size ``<sector-size>`` (or 512 bytes if ``<sector-size>`` is not specified). The
RAM disk is then registered as ``/dev/ram<minor>``. If ``<minor>`` is not
specified, ``mkrd`` attempts to register the RAM disk as ``/dev/ram0``.

Internally, the NSH ``mkrd`` command is a simple wrapper around the OS
``boardctl()`` interface, using the ``BOARDIOC_MKRD`` command. “Under the hood,”
this ``boardctl()`` command performs the following:

1. Allocates kernel-space memory with ``kmm_malloc()`` of size ``<nsectors>``
   times ``<sector-size>``
2. Zeros the allocated memory, and
3. Calls the OS-internal function ``ramdisk_register()`` to create the RAM disk.

NSH ROMFS /etc Support
======================

A ROM disk is a block device created from a read-only file system image stored
in FLASH or other ROM. There is no NSH command available to create a ROM disk
at runtime. However, it is possible to enable ROM disk support in NSH using the
``CONFIG_NSH_ROMFSETC`` option, as described in the section on NSH start-up
scripts in the `NSH User Guide <https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=139629410>`_.

Any application is able to create a ROM disk using the ``boardctl()`` interface
with the ``BOARDIOC_ROMDISK`` command.

Creating RAM Disks in Board Bring-Up Logic
==========================================

RAM disks may be created in board-specific initialization logic that runs in
supervisor mode. That logic might look as follows:

.. code-block:: c

  int board_ramdisk(int minor, unsigned int sectsize, unsigned int nsectors)
  {
    size_t allocsize = (size_t)sectsize * (size_t)nsectors;
    FAR uint8_t *buffer;
  
    /* Allocate the memory backing up the ramdisk */
  
    buffer = (FAR uint8_t *)kmm_zalloc(allocsize);
    if (buffer == NULL)
      {
        return -ENOMEM;
      }
  
    /* Then register the ramdisk */
  
    ret = ramdisk_register(minor, buffer, nsectors, sectsize,
                          RDFLAG_WRENABLED | RDFLAG_FUNLINK);
    if (ret < 0)
      {
        kmm_free(buffer);
      }
  
    return ret;
  }

Alternatively, this could be replaced by a call to the OS internal function
``mkrd()``.

Creating ROM Disks in Board Bring-Up Logic
==========================================

.. note::

   Currently, the ``romdisk_register()`` function is only available within the
   OS. Certain logic in ``apps/`` directly calls ``romdisk_register()``, which
   violates the portable POSIX OS interface. The correct approach for an
   application is to create a ROM disk via ``boardctl(BOARDIOC_ROMDISK)`` as
   described above. Calling ``romdisk_register()`` directly is not only a
   violation of the NuttX portable interface, but also is not allowed in
   PROTECTED or KERNEL build modes.

ROM disks, i.e., read-only disks in FLASH, can be created by board bring-up
logic in a way similar to RAM disks, with the following caveats:

- The FLASH region is not allocated; the FLASH address, the sector size, and the
  number of sectors must already be known.
- The ``romdisk_register()`` function is used instead of ``ramdisk_register()``.

A simple example could look like:

.. code-block:: c

  int board_romdisk(int minor, FAR uint8_t *buffer, unsigned int sectsize,
                    unsigned int nsectors)
  {
    /* Register the romdisk */
  
    return romdisk_register(minor, buffer, nsectors, sectsize);
  }

Calling ``romdisk_register()`` is equivalent to calling ``ramdisk_register()``
with the final parameter ``flags == 0``.

Most ROM disks use the ROMFS file system, although CROMFS is another option.
Creating ROMFS file system images involves several steps. Tools are available
to simplify the process of building ROMFS images, but that topic is outside the
scope of this Wiki page.