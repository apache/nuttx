==========================================
Morse Micro Wi-Fi HaLow (IEEE 802.11ah)
==========================================

This driver supports the Morse Micro MM6108 and MM8108 Wi-Fi HaLow
transceivers connected to the host through SPI, for example the Quectel
FGH100M module used on the Seeed Studio Wi-Fi HaLow module for XIAO.

Wi-Fi HaLow (IEEE 802.11ah) works in the sub-1 GHz bands (e.g. 902-928 MHz
in the US, 863-868 MHz in Europe), giving a longer range and a lower power
consumption than the 2.4/5 GHz Wi-Fi. For the network stack it behaves like
any other Wi-Fi interface: the driver registers a ``wlanN`` network device
that is configured with the standard wireless IOCTLs, for example with the
:doc:`wapi </applications/wireless/wapi/index>` tool.

The driver is located in ``drivers/wireless/ieee80211/morse`` and supports
the station (STA) mode with open, OWE and WPA3-SAE security.

Architecture
============

::

   +----------------------------------------------+
   |  NuttX network stack / wapi / dhcpc          |
   +----------------------------------------------+
   |  morse_netdev.c   netdev lower half and      |
   |                   wireless operations        |
   +----------------------------------------------+
   |  libmorse         802.11ah UMAC, WPA3-SAE    |   pre-built
   |  (Morse Micro)    supplicant, rate control,  |   library
   |                   chip transport (SPI)       |
   +----------------------+-----------------------+
   |  morse_mmosal.c      |  morse_mmhal.c        |
   |  OS abstraction:     |  hardware abstraction:|
   |  kthreads, mutexes,  |  SPI, GPIOs, IRQs,    |
   |  semaphores, timers  |  firmware and BCF     |
   +----------------------+-----------------------+
   |  NuttX kernel        |  board lower half     |
   |                      |  (morse_lower_s)      |
   +----------------------+-----------------------+

The 802.11ah MAC, the supplicant and the chip transport protocol are
provided by ``libmorse``, the Morse Micro host library that is also used by
the Morse Micro FreeRTOS and Zephyr ports. The NuttX driver implements the
interfaces that ``libmorse`` needs from the platform:

- ``mmosal`` (``morse_mmosal.c``): tasks, mutexes, semaphores, queues and
  timers on top of the NuttX kernel services. The timers run on the low
  priority work queue.
- ``mmhal`` (``morse_mmhal.c``): SPI transfers through the NuttX SPI
  driver, control lines and interrupts through the board lower half, and
  the chip firmware and board configuration file that are embedded in the
  image.
- the network interface (``morse_netdev.c``): it uses the network lower
  half interface (``netdev_lowerhalf_s``) and its wireless operations. The
  ``libmorse`` datapath uses 802.3 frames, so the interface is an
  Ethernet-like ``NET_LL_IEEE80211`` link.

External files
==============

The driver itself is Apache-2.0 licensed, but it needs a few files from the
`Morse Micro MM-IoT-SDK <https://github.com/MorseMicro/mm-iot-sdk>`_. They
are downloaded at build time (about 3 MB) to
``drivers/wireless/ieee80211/morse/mm-iot-sdk-<version>`` and removed by
``make distclean``:

.. list-table::
   :header-rows: 1
   :widths: 45 35 20

   * - File
     - Content
     - License
   * - ``framework/morselib/include/*.h``
     - libmorse API headers
     - Apache-2.0
   * - ``framework/morselib/lib/<core>/libmorse.a``
     - pre-built host library
     - Morse Micro Binary Distribution License
   * - ``framework/morsefirmware/mm6108.mbin``
     - chip firmware (``mm8108b2-rl.mbin`` for the MM8108)
     - Morse Micro Binary Distribution License
   * - ``framework/src/mmregdb/mmregdb.[ch]``
     - channel lists of each regulatory domain
     - Apache-2.0
   * - ``framework/src/mmpktmem/mmpktmem_heap.c``
     - packet memory allocator
     - Apache-2.0
   * - ``framework/src/mmutils/mmutils.h``
     - utility macros
     - Apache-2.0
   * - ``framework/tools/buildsystem/convert-bin-to-mbin.py``
     - BCF conversion tool
     - Apache-2.0
   * - ``LICENSES/*``
     - license texts
     - 

``libmorse.a`` is available for Cortex-M4F, Cortex-M7F and Cortex-M33F; the
driver selects the right one from the architecture configuration. With
the Make build the pre-built library is merged into ``libdrivers.a``
(``PREBUILT_LIBS`` in ``drivers/Makefile``), with CMake it is added with
``nuttx_add_extra_library()``. In both cases only the referenced objects are
linked.

The chip also needs a *board configuration file* (BCF) with the RF
calibration and the regulatory limits of the module. The default one is the
Quectel FGH100M-H BCF, downloaded from the
`morse-firmware <https://github.com/MorseMicro/morse-firmware>`_
repository and converted to the mbin format with the SDK tool. This requires
``python3`` with the ``pyelftools`` package (``pip install pyelftools``).

The files that are already present are not downloaded again. To build
without network access, copy the files listed above (keeping the SDK
directory layout) to ``drivers/wireless/ieee80211/morse/mm-iot-sdk-<version>``
and use ``CONFIG_IEEE80211_MORSE_BCF_SDK`` or
``CONFIG_IEEE80211_MORSE_BCF_CUSTOM``, or also copy the converted
``bcf_fgh100mhaamd.mbin`` to ``drivers/wireless/ieee80211/morse``.

Configuration
=============

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Option
     - Description
   * - ``CONFIG_IEEE80211_MORSE``
     - Enable the driver.
   * - ``CONFIG_IEEE80211_MORSE_MM6108``, ``CONFIG_IEEE80211_MORSE_MM8108``
     - Transceiver.
   * - ``CONFIG_IEEE80211_MORSE_SDK_VERSION``
     - SDK version the files are downloaded from (default ``2.13.1``).
   * - ``CONFIG_IEEE80211_MORSE_BCF_*``
     - BCF: Quectel FGH100M-H (default), one of the SDK BCFs, or a custom mbin file.
   * - ``CONFIG_IEEE80211_MORSE_COUNTRY``
     - Default regulatory domain (default ``US``).
   * - ``CONFIG_IEEE80211_MORSE_SPI_FREQUENCY``
     - Default SPI clock, if the board does not set one.
   * - ``CONFIG_IEEE80211_MORSE_TASK_PRIORITY``
     - Priority of the driver tasks.
   * - ``CONFIG_IEEE80211_MORSE_CONNECT_TIMEOUT``
     - Time the connect request waits for the link (default 30 s).
   * - ``CONFIG_IEEE80211_MORSE_LIBLOG``
     - Route the libmorse messages to the syslog.

The driver requires ``CONFIG_NETDEV_WIRELESS_IOCTL``. ``libmorse`` is built
against the newlib C library headers: the driver selects
``CONFIG_ARCH_SETJMP_H`` and ``CONFIG_LIBC_CTYPE_TABLE`` (the newlib
compatible ``_ctype_`` table), and ``CONFIG_LIBC_LOCALTIME`` is not supported
(its ``mktime()`` writes the NuttX specific ``tm_gmtoff``/``tm_zone`` fields
of ``struct tm``).

WPA3-SAE needs a good entropy source, so the driver depends on
``CONFIG_DEV_RANDOM`` or ``CONFIG_DEV_URANDOM``: enable the hardware random
number generator (e.g. ``CONFIG_STM32_RNG``) and ``CONFIG_DEV_RANDOM``. On the
STM32H7 the RNG kernel clock is HSI48, so ``CONFIG_STM32_HSI48`` must be
enabled as well, otherwise the RNG reports a clock error.

The image includes the ~380 KB chip firmware: the driver needs about 1 MB
of flash. On the WeAct STM32H743 ``halow`` configuration the whole system
uses about 57 KB of heap while connected.

Board interface
===============

The board logic describes how the transceiver is connected with a
``struct morse_lower_s`` (``include/nuttx/wireless/morse.h``) and registers
the driver with ``morse_register()``:

====================== ======================================================
Signal                 Description
====================== ======================================================
SPI (SCK, MOSI, MISO)  SPI mode 0. The chip select is driven by the board
                       ``<spi>select()`` function for ``devid``.
RESET_N                Output, active low. Held low until the driver boots
                       the chip (``reset``).
SPI_IRQ                Input, active low, falling edge interrupt
                       (``irq_asserted``, ``irq_attach``, ``irq_enable``).
                       ``irq_enable`` is called from interrupt context.
WAKE                   Output, active high (``wake``). Optional.
BUSY                   Input, active high, rising edge interrupt (``busy``,
                       ``busy_attach``, ``busy_enable``). Optional.
====================== ======================================================

.. code-block:: c

   #include <nuttx/wireless/morse.h>

   static const struct morse_lower_s g_halow_lower =
   {
     .devid        = SPIDEV_WIRELESS(0),
     .frequency    = 20000000,
     .reset        = halow_reset,
     .irq_asserted = halow_irq_asserted,
     .irq_attach   = halow_irq_attach,
     .irq_enable   = halow_irq_enable,

     /* WAKE and BUSY not connected: the chip is kept always awake */
   };

   int board_halow_initialize(void)
   {
     struct spi_dev_s *spi = stm32_spibus_initialize(1);

     return morse_register(spi, &g_halow_lower);
   }

``morse_register()`` boots the chip (downloading its firmware through SPI),
reads its MAC address and registers the network interface.
``boards/arm/stm32h7/weact-stm32h743/src/stm32_morse.c`` is a complete
example.

WAKE and BUSY lines (power save)
================================

``libmorse`` puts the chip in power save when the bus is idle (after a few
milliseconds): it de-asserts WAKE and stops listening to SPI_IRQ. When the
chip has an event for the host (a received packet, the end of a scan, ...)
it asserts BUSY, which wakes up the host. So **the power save only works if
both WAKE and BUSY are connected**.

If the board lower half does not provide ``wake`` or ``busy`` (``NULL``),
the driver keeps the chip always awake: it reports BUSY as permanently
asserted, which prevents ``libmorse`` from suspending the host interface,
and it disables the 802.11 power save of the station. This costs more
power but works with only the SPI, SPI_IRQ and RESET_N lines.

.. warning::

   On the **Seeed Studio Wi-Fi HaLow module for XIAO** (Quectel FGH100M-H)
   the WAKE and BUSY pads (D1 and D4) are **not connected to the module**:
   the series resistors near those pins are not populated (DNP). If the
   board declares these lines anyway, the driver works until the first time
   the chip is put to sleep and then it stops receiving events: a scan never
   completes, the chip holds SPI_IRQ asserted and BUSY never rises. Leave
   ``wake``/``busy`` unset for this module, or populate the two resistors to
   use the power save.

Also note that this module needs the 5V pad connected: it supplies the RF
front end, without it the module enumerates and receives but cannot
transmit.

Usage
=====

Wi-Fi HaLow only allows open, OWE and WPA3-SAE networks (no WPA2). A
passphrase selects WPA3-SAE (with PMF required); WPA/WPA2 requests are
mapped to SAE::

   nsh> ifconfig wlan0 up
   nsh> wapi scan wlan0
   bssid / frequency / signal level / encode / ssid
   1c:bc:ec:30:f7:fb	9.035e+08	-14	0800	silex-ap
   nsh> wapi psk wlan0 mypassphrase 3 3
   nsh> wapi essid wlan0 silex-ap 1
   nsh> renew wlan0
   nsh> ifconfig
   wlan0	Link encap:Ethernet HWaddr 68:24:99:44:6b:6b at RUNNING mtu 1500
   	inet addr:10.0.3.245 DRaddr:10.0.0.1 Mask:255.255.240.0
   nsh> wapi sense wlan0
   -6

The frequency is reported in Hz. ``wapi sense`` reports the RSSI (dBm) of
the access point. ``wapi disconnect wlan0`` disconnects (the carrier goes
down).

The regulatory domain selects the channels and the transmit power limits.
It must match the country where the device is used and the access point
configuration, otherwise the access point is not found. It can be changed
while disconnected::

   nsh> wapi country wlan0 AU

The supported codes are the ones of the SDK regulatory database (AU, CA,
EU, GB, IN, JP, KR, NZ and US), restricted by the regions of the BCF.

Troubleshooting
===============

- **The SPI data is shifted by one or more bits**: signal integrity. With
  jumper wires, use the lowest GPIO slew rate for SCK and MOSI and keep a
  ground wire next to the SPI wires. On the WeAct STM32H743 the bus is
  reliable up to 18.75 MHz this way.
- **The scan never completes / no events after the boot**: WAKE/BUSY
  declared but not connected, see above.
- **The system hangs while connecting**: the SAE handshake reads a lot of
  random numbers; check that the RNG clock is configured (HSI48 on the
  STM32H7).
- **The access point is not found**: check the country code and the BCF
  regions.
- Enable ``CONFIG_DEBUG_WIRELESS_INFO`` and ``CONFIG_IEEE80211_MORSE_LIBLOG``
  for more messages.

Limitations
===========

- Only the station mode is supported (no access point or relay mode).
- The SPI transfers are polled (no DMA).
- Only the SPI host interface is supported (no SDIO).
