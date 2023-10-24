===========================================
``usbmsc`` USB Mass Storage Device Commands
===========================================

This add-on registers a block device driver, then exports the block the device
using the USB storage class driver. In order to use this add-on, your
board-specific logic must provide the function::

  void board_usbmsc_initialize(void);

This function will be called by the ``system/usbmsc`` indirectly via the ``boardctl``
``BOARDIOC_USBDEV_CONTROL`` command in order to do the actual registration of the
block device drivers. For examples of the implementation of
``board_usbmsc_initialize()`` see
``boards/arm/lpc214x/mcu123-lpc214x/src/up_usbmsc.c`` or
``boards/arm/stm32/stm3210e-eval/src/usbmsc.c``.

Configuration options:

- ``CONFIG_NSH_BUILTIN_APPS`` – This add-on can be built as two NSH "built-in"
  commands if this option is selected: ``msconn`` will connect the USB mass
  storage device; ``msdis`` will disconnect the USB storage device.

- ``CONFIG_BOARDCTL`` – Enables the ``boardctl()`` interfaces.

- ``CONFIG_BOARDCTL_USBDEVCTRL`` – Enables the ``BOARDIOC_USBDEV_CONTROL``
  ``boardctl()`` command.

- ``CONFIG_SYSTEM_USBMSC_NLUNS`` – Defines the number of logical units (LUNs)
  exported by the USB storage driver. Each LUN corresponds to one exported block
  driver (or partition of a block driver). May be ``1``, ``2``, or ``3``. Default is
  ``1``.

- ``CONFIG_SYSTEM_USBMSC_DEVMINOR1`` – The minor device number of the block driver
  for the first LUN. For example, ``N`` in ``/dev/mmcsdN``. Used for registering the
  block driver. Default is zero.

- ``CONFIG_SYSTEM_USBMSC_DEVPATH1`` – The full path to the registered block
  driver. Default is ``/dev/mmcsd0``

- ``CONFIG_SYSTEM_USBMSC_DEVMINOR2`` and ``CONFIG_SYSTEM_USBMSC_DEVPATH2``
  Similar parameters that would have to be provided if
  ``CONFIG_SYSTEM_USBMSC_NLUNS`` is ``2`` or ``3``. No defaults.

- ``CONFIG_SYSTEM_USBMSC_DEVMINOR3`` and ``CONFIG_SYSTEM_USBMSC_DEVPATH3``
  Similar parameters that would have to be provided if
  ``CONFIG_SYSTEM_USBMSC_NLUNS`` is ``3``. No defaults.

- ``CONFIG_SYSTEM_USBMSC_DEBUGMM`` – Enables some debug tests to check for memory
  usage and memory leaks.

If ``CONFIG_USBDEV_TRACE`` is enabled (or ``CONFIG_DEBUG_FEATURES`` and
``CONFIG_DEBUG_USB``), then the code will also manage the USB trace output. The
amount of trace output can be controlled using:

- ``CONFIG_SYSTEM_USBMSC_TRACEINIT`` – Show initialization events.
- ``CONFIG_SYSTEM_USBMSC_TRACECLASS`` – Show class driver events.
- ``CONFIG_SYSTEM_USBMSC_TRACETRANSFERS`` – Show data transfer events.
- ``CONFIG_SYSTEM_USBMSC_TRACECONTROLLER`` – Show controller events.
- ``CONFIG_SYSTEM_USBMSC_TRACEINTERRUPTS`` – Show interrupt-related events.

Error results are always shown in the trace output

**Note 1**: When built as an NSH add-on command (``CONFIG_NSH_BUILTIN_APPS=y``),
Caution should be used to assure that the SD drive (or other storage device) is
not in use when the USB storage device is configured. Specifically, the SD
driver should be unmounted like::

  nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard  # Card is mounted in NSH
  ...
  nsh> umount /mnd/sdcard                     # Unmount before connecting USB!!!
  nsh> msconn                                 # Connect the USB storage device
  ...
  nsh> msdis                                  # Disconnect USB storate device
  nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard  # Restore the mount

Failure to do this could result in corruption of the SD card format.

**Note 2**: This add-on used internal USB device driver interfaces. As such, it
relies on internal OS interfaces that are not normally available to a user-space
program. As a result, this add-on cannot be used if a NuttX is built as a
protected, supervisor kernel (``CONFIG_BUILD_PROTECTED`` or
``CONFIG_BUILD_KERNEL``).
