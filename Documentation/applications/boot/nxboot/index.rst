===========================
``nxboot`` NuttX Bootloader
===========================

NuttX Bootloader (nxboot) can be used to provide update and recovery
capabilities for NuttX based devices. The bootloader implements an
algorithm that uses three partitions/areas: primary, secondary and
tertiary. Primary area is used to run the image and thus is usually
located in a program memory. Secondary and tertiary areas are used
to store update image or recovery image and can be located on
external flash memory for example.

The images for the bootloader have version located in their headers. Note
that bootloader/image characteristics may differ for different version
and a portable application should take this into account.

Algorithm Description
---------------------

The update is performed with a simple copy from update area to primary
area with recovery being created in recovery area if not already present.
Once image is confirmed by the user, the image in update area is
confirmed as well, update area becomes recovery area and vice versa.
This means the recovery is always present (except for the first update)
and subsequent updates just copy image from update to primary. This
makes the update significantly faster and more considerable to flash
wear while keeping the recovery/revert possibility.

Not confirmed image is reverted to recovery image if reboot occurs.

The bootable image consists of a header :c:struct:`nxboot_img_header`
containing magic value, header version, header size, CRC32 of the image
including some parts of the header, size of the image excluding the header,
platform identifier, pointer to extended headers and and firmware version.
The header is located prior to the image itself and has a configurable size
``CONFIG_NXBOOT_HEADER_SIZE``. The CRC is calculated from the entire image
including header except for magic, header version and header size fields.
Extended headers are currently not supported, but the header already has
a reserved space for a pointer to it.

The image compatible with nxboot bootloader can be uploaded both directly
to the primary area via physical programmer as STlink or JTAG and to the
update partition via some external application (over Ethernet, USB, CAN, etc.).
The update and recovery slots can be located in the the primary flash as
well, but this halts the program execution during write operations, so it is
not recommended if external flash can be used. The uploaded image is detected
by the bootloader during the next boot and update occurs.

Bootloader has an internal magic value that is used to detect updated images.
Once update occurs, the image is copied from update to primary partition with
the internal magic value and the first erase page of the update slot is erased.
The image with internal magic value is considered valid only if its recovery
exists, therefore the image confirmation is done by writing the first erase
page (copying by write page size from primary) back to the update slot. It is
recommended to use :c:func:`nxboot_confirm` API to confirm the image. This
approach wears the first sector of the update partition a bit more, but
avoids image's tails completely and simplifies both internal and API logic.

The application can use function :c:func:`nxboot_get_state` to determine
what partition is update and recovery and thus where the update image
should be stored. It is also possible to use function :c:func:`nxboot_open_update_partition`
that determines the correct partition for updates and returns the opened
file descriptor. This is a recommended approach as it avoids possible mistakes.

Hardware Requirements
---------------------

As mentioned above, the bootloader utilizes three partitions with the
first one typically located in program flash.

The bootloader itself requires the minimal knowledge about characteristics
of the underlying storage device. This is achieved by means of the ``BCH``
and ``FTL`` subsystems, which enable the bootloader to manage MTD partitions
via character device drivers using standard POSIX filesystem operations
(e.g. ``open()`` / ``close()`` / ``read()`` / ``write()``).

It is currently required to access the partitions via mentioned ``BCH``
and ``FTL`` subsystems, but future enhancement could provide a direct access
with ``MTD`` layer.

Configuration
-------------

Following configuration options are available:

- ``CONFIG_BOOT_NXBOOT``: Enables NuttX Bootloader build.
- ``CONFIG_NXBOOT_PRIMARY_SLOT_PATH``:
    The path to the application firmware image primary slot character
    device driver. The image runs from this location. Default is ``dev/ota0``.
- ``CONFIG_NXBOOT_SECONDARY_SLOT_PATH``:
    The path to the application firmware image primary slot character device
    driver. This is either update or recovery slot.. Default is ``dev/ota1``.
- ``CONFIG_NXBOOT_TERTIARY_SLOT_PATH``:
    The path to the application firmware image primary slot character device
    driver. This is either update or recovery slot.. Default is ``dev/ota2``.
- ``CONFIG_NXBOOT_HEADER_SIZE``:
    Size of the image header. Note that this size should be aligned with the
    program memory write page size!
- ``CONFIG_NXBOOT_PLATFORM_IDENTIFIER```:
    64 bits large platform identifier. This is a unique platform identifier
    used by the bootloader to verify whether the image should be run on a
    given platform. An update (or even a firmware uploaded via a programmer)
    is rejected if the value in image's header doesn't match this option.
- ``CONFIG_NXBOOT_BOOTLOADER``:
    This option builds and links a bootloader application. This application
    should be an entry function for NuttX. It checks for possible
    update/revert operation, performs it and boot the correct image.
- ``CONFIG_NXBOOT_SWRESET_ONLY``:
    This option ensures the update/revert is performed only for software
    reset. This way the board can keep its image (even if not confirmed)
    during for example power shutdown and perform update/revert only if
    expected based on user/maintainer input. Following reset causes
    are considered as software resets.

    ``BOARDIOC_RESETCAUSE_CPU_SOFT``: software reset

    ``BOARDIOC_RESETCAUSE_CPU_RWDT``: watchdog error

    ``BOARDIOC_RESETCAUSE_PIN``: reset button

- ``NXBOOT_PREVENT_DOWNGRADE`` :
    NXboot uses Semantic Version 2.0.0 (without build metadata). By default,
    the update is performed for every version that doesn't match the
    currently running one. If NXBOOT_PREVENT_DOWNGRADE selected, update is
    performed only for newer versions (according to Semantic Version
    preference rules).

		``WARNING``: NXboot currently implements preferences only for
		``MAJOR.MINOR.PATCH`` and ignores prerelease.

Image Creation
--------------

The image intended to be booted by nxboot bootloader must have prepended header
to be recognized and worked with correctly. A Python script ``nximage.py``
located in ``apps/boot/nxboot/tools`` directory can be used to generate
nxboot compatible image.

.. code-block:: bash

  python3 apps/boot/nxboot/tools/nximage.py  \
		--version "VERSION" \
		--header_size CONFIG_NXBOOT_HEADER_SIZE \
		--identifier CONFIG_NXBOOT_PLATFORM_IDENTIFIER \
		nuttx.bin image.img

It takes input parameters ``--version`` with your image's version,
``--header_size`` with the configured size of the header and ``--identifier``.
with the platform identifier. The input file is a binary ``nuttx.bin``, output
with added header is ``image.img``.

Image version adheres to `Semantic Versioning 2.0.0 <https://semver.org/spec/v2.0.0.html>`__
without the usage of build metadata. The used format is
MAJOR.MINOR:PATCH-prerelease. The image version is important for the update
to occur as the bootloader automatically rejects update firmware that
has the same version as the already running firmware.
Configuration options:

Public API
----------

Enabling ``CONFIG_BOOT_NXBOOT`` option provides following NXboot API.

.. code-block:: c

  #include <nxboot.h>

.. c:struct:: nxboot_img_version
.. code-block:: c

  #define NXBOOT_HEADER_PRERELEASE_MAXLEN 94

  struct nxboot_img_version
  {
    /* MAJOR version */
    uint16_t major;
    /* MINOR version */
    uint16_t minor;
    /* PATCH version */
    uint16_t patch;
    /* Additional pre-release version */
    char pre_release[NXBOOT_HEADER_PRERELEASE_MAXLEN];
  };

.. c:struct:: nxboot_hdr_version
.. code-block:: c

  struct nxboot_hdr_version
  {
    /* Header major version */
    uint8_t major;
    /* Header minor version */
    uint8_t minor;
  };

.. c:struct:: nxboot_img_header
.. code-block:: c

  #define NXBOOT_HEADER_MAGIC     0x534f584e
  #define NXBOOT_HEADER_MAGIC_INT 0xaca0abb0

  struct nxboot_img_header
  {
    /* Header magic */
    uint32_t magic;
    /* Version of the header */
    struct nxboot_hdr_version hdr_version;
    /* Size of the header */
    uint16_t header_size;
    /* CRC of the image, exceluding the previous header fields. */
    uint32_t crc;
    /* Image size (excluding the header) */
    uint32_t size;
    /* Platform identifier */
    uint64_t identifier;
    /* Address of optional extended headers */
    uint32_t extd_hdr_ptr;
    /* Image version */
    struct nxboot_img_version img_version;
  };

.. c:enum:: nxboot_update_type
.. code-block:: c

    enum nxboot_update_type
    {
      /* No action to do */
      NXBOOT_UPDATE_TYPE_NONE = 0,
      /* Update will take place upon reboot */
      NXBOOT_UPDATE_TYPE_UPDATE = 1,
      /* Revert will take place upon reboot */
      NXBOOT_UPDATE_TYPE_REVERT = 2,
    };

.. c:struct:: nxboot_state
.. code-block:: c

  #define NXBOOT_PRIMARY_SLOT_NUM 0
  #define NXBOOT_SECONDARY_SLOT_NUM 1
  #define NXBOOT_TERTIARY_SLOT_NUM 2

  struct nxboot_state
  {
    /* Number of update slot */
    int update;
    /* Number of recovery slot */
    int recovery;
    /* True if recovery image contains valid recovery */
    bool recovery_valid;
    /* True if image in a primary slot has a recovery (even non valid) */
    bool recovery_present;
    /* True if primary slot is confirmed */
    bool primary_confirmed;
    /* True if update slot has a valid image */
    enum nxboot_update_type next_boot;
  };

.. c:function:: int nxboot_get_state(struct nxboot_state *state)

  This function can be used to determine whether primary image is
  confirmed or not. This provides more direct access to confirm
  state compared to nxboot_get_state function that returns the full
  state of the bootloader.

  :param state: A pointer to ``struct nxboot_state`` structure.

  :return: 0 on success, -1 and sets errno on failure.

.. c:function:: int nxboot_open_update_partition(void)

  Gets the current bootloader state and opens the partition to which an
  update image should be stored. It returns the valid file descriptor to
  this partition, the user is responsible for writing to it and closing
  if afterwards.

  :return: Valid file descriptor on success, -1 and sets errno on failure.

.. c:function:: int nxboot_get_confirm(void)

  Obtains the information if currently running image is confirmed (and
  thus stable) or not.

  :return: 1 means confirmed, 0 not confirmed, -1 and sets errno on failure.

.. c:function:: int nxboot_confirm(void)

  Confirms the image currently located in primary partition and marks
  its copy in update partition as a recovery.

  :return: 0 on success, -1 and sets errno on failure.

.. c:function:: int nxboot_perform_update(bool check_only)

  Checks for the possible firmware update and performs it by copying
  update image to primary slot or recovery image to primary slot in case
  of the revert. In any situation, this function ends with the valid
  image in primary slot.

  This is an entry point function that should be called from the
  bootloader application.

  :param check_only: Only repairs corrupted update.

  :return: 0 on success, -1 and sets errno on failure.
