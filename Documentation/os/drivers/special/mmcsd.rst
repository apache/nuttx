====================
MMCSD Device Drivers
====================

NuttX presents an MMC or SD card as a block device, ``/dev/mmcsdN``, which a
file system is then mounted on. The driver is in two parts: a generic upper
half that knows the card protocols, and a lower half that moves bytes over
whichever bus the card is wired to.

-  ``include/nuttx/mmcsd.h``. All structures and APIs needed to work with
   MMCSD drivers are provided in this header file.

There are two upper halves, because the two buses are not variations of one
protocol:

-  ``drivers/mmcsd/mmcsd_sdio.c`` drives a card through a real SD/MMC host
   controller, over the :doc:`SDIO interface <sdio>`. This is the one that can
   use a wide bus and high speed timing.
-  ``drivers/mmcsd/mmcsd_spi.c`` drives a card over SPI, which every card
   supports as a fallback. It is one bit wide, slower, and has no notion of
   bus width or speed switching.

Implementers of SDIO lower-half drivers should refer to the
:doc:`SDIO Driver Documentation <sdio>` for details on R2/CSD response
handling and correctly implementing the lower-half interface.

Registering a card slot
=======================

Board logic registers one slot per card, once the bus it is on exists. For a
card on an SD host controller:

.. code-block:: c

   #include <nuttx/mmcsd.h>

   struct sdio_dev_s *sdio = sdio_initialize(0);

   mmcsd_slotinitialize(0, sdio);      /* becomes /dev/mmcsd0 */

The first argument is the minor number, which names the device. For a card
on SPI instead:

.. code-block:: c

   struct spi_dev_s *spi = board_spibus_initialize(0);

   mmcsd_spislotinitialize(0, 0, spi);

where the second argument is the slot number, meaningful only on boards with
more than one, and bounded by ``CONFIG_MMCSD_NSLOTS``.

Neither call needs a card to be present. An empty slot is not an error: the
driver reports the slot as empty, arms the media inserted event and returns
success, so a board does not have to know at start up whether a card is
fitted.

Bus width and speed
===================

A card comes out of reset on a one bit bus at a low clock, because that is
all a card is guaranteed to answer on before anything is known about it. The
upper half then negotiates upward, and each step is taken only if the card
and the host both allow it:

#. **Identification.** ``CLOCK_IDMODE``, below 400kHz, while the card is
   identified and its CID and CSD are read.
#. **Bus width.** If the card's SCR says it supports four bits and the host
   did not report ``SDIO_CAPS_1BIT_ONLY``, ACMD6 widens the bus and the clock
   moves to ``CLOCK_SD_TRANSFER_4BIT``. Otherwise the card stays at
   ``CLOCK_SD_TRANSFER_1BIT``.
#. **High speed timing.** See below.

An eMMC part follows the same shape with its own commands and its own
``CLOCK_MMC_TRANSFER_*`` rates, and can be eight bits wide where the host
reports ``SDIO_CAPS_8BIT``.

High speed timing
-----------------

A card in default speed is rated to 25MHz. High speed timing doubles that,
and is usually the difference between a card that transfers at 10MB/s and one
that transfers at 20MB/s.

Getting there needs agreement from three parties, and the upper half checks
each in turn:

**The host must be able to clock it.** A lower half that can drive 50MHz says
so by reporting ``SDIO_CAPS_SD_HS_MODE`` from ``SDIO_CAPSET``. A host that
omits the capability is never asked, and its cards stay at the default rate.
The capability exists rather than the upper half simply trying, because
nothing in the card protocol tells the upper half what a particular board's
wiring will stand.

**The card must support it.** The upper half sends CMD6, SWITCH_FUNC, asking
for function 1 of function group 1, the access mode group. Every other group
is passed 0xF, "no influence", so exactly one function is changed and nothing
else about the card's configuration moves.

**The card must confirm it.** CMD6 returns a 512 bit status data structure,
in which the field for function group 1 reports the function the card
actually selected. A card that cannot comply reports 0xF there and the
command still succeeds, so the R1 response alone proves nothing. Only when
that field reads 1 does the upper half raise the clock to
``CLOCK_SD_TRANSFER_4BIT_HS``.

Anything else, at any step, leaves the card at ``CLOCK_SD_TRANSFER_4BIT``.
That includes a card predating the switch: CMD6 arrived in version 1.10 of
the physical layer specification, and the SCR says which version a card
implements, so older cards are not asked at all.

High speed is attempted only on the four bit path. A card narrow enough to
need the one bit path predates the switch command in any case.

.. note::

   A lower half that reports ``SDIO_CAPS_SD_HS_MODE`` **must** handle
   ``CLOCK_SD_TRANSFER_4BIT_HS`` in its clock method. The card is clocked
   before the switch can have happened and again afterwards, so a driver that
   treated the two rates alike would run a card still in default speed timing
   past the 25MHz it is rated for.

Configuration
=============

``CONFIG_MMCSD`` builds the family. The options worth knowing:

============================== ===============================================
``MMCSD_NSLOTS``               Slots per host, for boards with more than one
``MMCSD_MMCSUPPORT``           Recognise eMMC and MMC parts, not only SD
``MMCSD_READONLY``             Refuse writes; the card is still readable
``MMCSD_MULTIBLOCK_LIMIT``     Largest multi-block transfer, 0 for no limit
``MMCSD_IOCSUPPORT``           ``MMC_IOC_CMD`` and ``MMC_IOC_MULTI_CMD``
``MMCSD_HAVE_CARDDETECT``      The board wires a card detect pin
``MMCSD_HAVE_WRITEPROTECT``    The board wires a write protect pin
``MMCSD_SPI``                  Build the SPI upper half
``MMCSD_SPICLOCK``             Clock for the SPI path, default 20MHz
``MMCSD_PROCFS``               Add ``/proc/mmcsd``
============================== ===============================================

Passing commands to a card
==========================

With ``CONFIG_MMCSD_IOCSUPPORT``, ``MMC_IOC_CMD`` sends one command to the
card and ``MMC_IOC_MULTI_CMD`` sends a sequence of up to
``MMC_IOC_MAX_CMDS`` (255) of them without releasing the card in between.
Both take the same descriptor as Linux, ``struct mmc_ioc_cmd``, so tooling
written against that interface works unchanged. One transfer is bounded by
``MMC_IOC_MAX_BYTES``, 512KiB.

This is how RPMB and other vendor specific facilities are reached, and it is
deliberately low level: it hands a command to the card and does not interpret
the result.

Inspecting a card
=================

``CONFIG_MMCSD_PROCFS`` adds ``/proc/mmcsd``, with a directory per registered
card holding:

============= =================================================
``type``      ``SD`` or ``MMC``
``cid``       The card identification register, as raw hex
``csd``       The card specific data register, as raw hex
============= =================================================

The registers are reported as the card gives them rather than decoded, so
reading them means having the specification to hand. They are the quickest
way to tell whether the card the driver found is the card that was expected.
