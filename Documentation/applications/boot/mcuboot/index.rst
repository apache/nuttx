=======
MCUboot
=======

Description
-----------

The NuttX port of MCUboot secure boot library expects that the platform provides
a Flash storage with the following partitions:

- ``CONFIG_MCUBOOT_PRIMARY_SLOT_PATH``: MTD partition for the application
  firmware image PRIMARY slot;
- ``CONFIG_MCUBOOT_SECONDARY_SLOT_PATH``: MTD partition for the application
  firmware image SECONDARY slot;
- ``CONFIG_MCUBOOT_SCRATCH_PATH``: MTD partition for the Scratch area;

Also, these are optional features that may be enabled:

- ``CONFIG_MCUBOOT_WATCHDOG``: If ``CONFIG_WATCHDOG`` is enabled, MCUboot shall
  reset the watchdog timer indicated by ``CONFIG_MCUBOOT_WATCHDOG_DEVPATH`` to
  the current timeout value, preventing any imminent watchdog timeouts.

The porting layer of MCUboot library consists of the following interfaces:

- ``<flash_map_backend/flash_map_backend.h>``, for enabling MCUboot to manage
  the application firmware image slots in the device storage.
- ``<mcuboot_config/mcuboot_config.h>``, for configuration of MCUboot's features.
- ``<mcuboot_config/mcuboot_logging.h>``, for providing logging capabilities.
- ``<os/os_malloc.h>``, for providing MCUboot access to the OS memory management interfaces.
- ``<sysflash/sysflash.h>``, for configuration of the system's flash area organization.

The NuttX port of MCUboot is implemented at application-level and requires minimal
knowledge about characteristics of the underlying storage device. This is achieved
by means of the ``BCH`` and ``FTL`` subsystems, which enable MCUboot to manage MTD
partitions via character device drivers using standard POSIX filesystem operations
(e.g. ``open()`` / ``close()`` / ``read()`` / ``write()``).

Creating MCUboot-compatible application firmware images
-------------------------------------------------------

One common use case for MCUboot is to integrate it to a firmware update agent, which
is an important component of a secure firmware update subsystem. Through MCUboot APIs
an application is able to install a newly received application firmware image and,
once this application firmware image is assured to be valid, the application may
confirm it as a stable image. In case that application firmware image is deemed
bogus, MCUboot provides an API for invalidating that update, which will induce a
rollback procedure to the most recent stable application firmware image.

The ``CONFIG_EXAMPLES_MCUBOOT_UPDATE_AGENT`` example demonstrates this workflow by
downloading an application firmware image from a webserver, installing it and triggering
the firmware update process for the next boot after a system reset. There is also the
``CONFIG_EXAMPLES_MCUBOOT_SLOT_CONFIRM``, which is a fairly simple example that just
calls an MCUboot API for confirming the executing application firmware image as stable.

For more information about all MCUboot examples, see ``examples/mcuboot`` directory.

Using MCUboot on NuttX as a secure boot solution
------------------------------------------------

NuttX port for MCUboot also enables the creation of a secure bootloader application
requiring minimal platform-specific implementation. The logical implementation for
the secure boot is performed at application-level by the MCUboot library. Once MCUboot
validates the application firmware image, it delegates the loading and execution of the
application firmware image to a platform-specific routine, which is accessed via
``boardctl(BOARDIOC_BOOT_IMAGE)`` call. Each platform must then provide an implementation
for the ``board_boot_image()`` for executing the required actions in order to boot a new
application firmware image (e.g. deinitialize peripherals, load the Program Counter register
with the application firmware image entry point address).

The MCUboot bootloader application may be enabled by selecting the ``CONFIG_MCUBOOT_BOOTLOADER``
option.

Assumptions
-----------

IOCTL MTD commands
~~~~~~~~~~~~~~~~~~

The implementation of ``<flash_map_backend/flash_map_backend.h>`` expects that the MTD
driver for a given image partition handles the following ``ioctl`` commands:

- ``MTDIOC_GEOMETRY``, for retrieving information about the geometry of the MTD,
  required for the configuration of the size of each flash area.
- ``MTDIOC_ERASESTATE``, for retrieving the byte value of an erased cell of the MTD,
  required for the implementation of ``flash_area_erased_val()`` interface.

Write access alignment
~~~~~~~~~~~~~~~~~~~~~~

Through ``flash_area_align()`` interface MCUboot expects that the implementation provides
the shortest data length that may be written via ``flash_area_write()`` interface. The NuttX
implementation passes through the ``BCH`` and ``FTL`` layers, which appropriately handle the
write alignment restrictions of the underlying MTD. So The NuttX implementation of
``flash_area_align()`` is able to return a fixed value of 1 byte, even if the MTD does not
support byte operations.

Limitations
-----------

``<flash_map_backend/flash_map_backend.h>`` functions are not multitasking-safe
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

MCUboot's documentation imposes no restrictions regarding the usage of its public interfaces,
which doesn't mean they are thread-safe.
But, regarding NuttX implementation of the ``<flash_map_backend/flash_map_backend.h>``, it is
safe to state that they are **not** multitasking-safe. NuttX implementation manages the MTD
partitions via character device drivers. As file-descriptors cannot be shared between different
tasks, if one task calls ``flash_area_open`` and another task calls ``flash_area_<read/write/close>``
passing the same ``struct flash_area`` instance, it will result in failure.
