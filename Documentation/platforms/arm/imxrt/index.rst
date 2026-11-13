===========
NXP i.MX RT
===========

The i.MX RT series of chips from NXP Semiconductors is based around an ARM Cortex-M7 core running
at 500 MHz, 600 MHz or 1 GHz based on particular MCUs

Supported MCUs
==============

The following list includes MCUs from i.MX RT series and indicates whether they are supported in NuttX

======  ==============  ===============  =================
MCU     Support         Core             Frequency
======  ==============  ===============  =================
RT500   No              Cortex-M33       200 MHz
RT600   No              Cortex-M33       300 MHz
RT700   No              Cortex-M33       325 MHz
RT1010  No              Cortex-M7        500 MHz
RT1015  No              Cortex-M7        500 MHz
RT1020  Yes             Cortex-M7        500 MHz
RT1024  No              Cortex-M7        500 MHz
RT1050  Yes             Cortex-M7        600 MHz
RT1060  Yes             Cortex-M7        600 MHz
RT1064  Yes             Cortex-M7        600 MHz
RT1160  No              Cortex-M7 + M4   600 Mhz + 240 MHz
RT1170  Yes (Only M7)   Cortex-M7 + M4   1 GHz + 400 MHz
RT1180  Yes             Cortex-M7 + M33  800 Mhz + 300 MHz
======  ==============  ===============  =================

i.MX RT118x
===========

The i.MX RT118x family is supported on both the Cortex-M33 and Cortex-M7
cores. The Cortex-M33 is the boot core; it can run NuttX directly, or it
can run a small bootloader that releases the Cortex-M7 to run a separate
NuttX image.

The current clock configuration runs the Cortex-M7 at 798 MHz and the
Cortex-M33 at 266 MHz due to ARM_PLL and divider setting limitations.

RT118x uses the generation 3 clock configuration driver for the clock
roots found on this family. It also uses the generation 3 IOMUXC driver
for the RT118x pinmux layout and GPIO mode selection.

RT118x uses the RGPIO driver and the matching RGPIO interrupt driver
instead of the legacy i.MX RT GPIO block. The version 2 eDMA driver is
selected for the newer RT118x eDMA IP.

The EdgeLock Enclave driver provides the messaging path used for RT118x
ELE firmware, and the TRDC driver configures access to RT118x memory and
peripheral resources. The Cortex-M7 release helper is used by Cortex-M33
firmware to set the Cortex-M7 boot vector and start the M7 core.

NETC architecture driver
------------------------

``CONFIG_IMXRT_NETC`` provides a NETDEV lower-half Ethernet driver for
RT118x. ENETC0 (PF3, SI0) connects directly to a PHY on physical link 4;
ENETC1 (PF4, SI0) connects through switch CPU port 4 to selected external
ports 0--3. These are independent interfaces and may operate separately
or together; ENETC0 is not bridged to the switch.

Required configuration
~~~~~~~~~~~~~~~~~~~~~~

On an RT118x target, enable ``CONFIG_NET``, ``CONFIG_NET_ETHERNET``,
``CONFIG_NETDEVICES`` and ``CONFIG_MM_IOB``. In the **NETC Ethernet ports**
menu, select at least one of the following:

* ``CONFIG_IMXRT_NETC_ENETC0`` enables the independent direct-PHY interface
  on physical link 4.
* ``CONFIG_IMXRT_NETC_ENETC1_PORT0``, ``CONFIG_IMXRT_NETC_ENETC1_PORT1``,
  ``CONFIG_IMXRT_NETC_ENETC1_PORT2`` and
  ``CONFIG_IMXRT_NETC_ENETC1_PORT3`` select external switch ports 0--3.
  Select each required port; selected ports share one ENETC1 interface.

These selections enable the hidden ``CONFIG_IMXRT_NETC`` symbol, which
automatically selects ``CONFIG_NETDEV_LATEINIT`` and ``CONFIG_SCHED_LPWORK``.
Do not try to enable ``CONFIG_IMXRT_NETC`` directly.

Also ensure that the PHY drivers used by the board are enabled through
``CONFIG_ETH0_PHY_*`` and/or ``CONFIG_ETH1_PHY_*`` under
**Network Device/PHY Support**. These selections enable compilation of
the corresponding PHY drivers. A board may instead select the driver
symbols directly in its Kconfig, as the i.MXRT1180-EVK does for its enabled
NETC ports. The per-port PHY descriptors described below are required in
addition to enabling the PHY drivers.

The driver requires ``CONFIG_IOB_BUFSIZE >= 128``,
``CONFIG_IOB_NBUFFERS >= 96`` and ``CONFIG_IOB_THROTTLE <= 16``.
The TX descriptor choices require at least 96, 160, 320 or 640 IOBs for
8, 16, 32 or 64 descriptors respectively; the default is 16 descriptors.
Also keep ``CONFIG_NET_ETH_PKTSIZE <= 1518`` and
``CONFIG_NET_LL_GUARDSIZE <= 32``, and disable ``CONFIG_NET_PROMISCUOUS``.

For example, for a board using an RTL8201F PHY, the following configuration
fragment selects its PHY driver and switch port 0 with 16 TX and RX
descriptors::

  CONFIG_NET=y
  CONFIG_NET_ETHERNET=y
  CONFIG_NETDEVICES=y
  CONFIG_MM_IOB=y
  CONFIG_IMXRT_NETC_ENETC1_PORT0=y
  CONFIG_ETH0_PHY_RTL8201F=y
  CONFIG_IMXRT_NETC_TX16=y
  CONFIG_IMXRT_NETC_RX16=y
  CONFIG_IOB_BUFSIZE=128
  CONFIG_IOB_NBUFFERS=160
  CONFIG_IOB_THROTTLE=0
  # CONFIG_NET_PROMISCUOUS is not set

For the direct interface alone, replace
``CONFIG_IMXRT_NETC_ENETC1_PORT0=y`` with ``CONFIG_IMXRT_NETC_ENETC0=y``.
Enable both to use both interfaces. This fragment covers driver selection
and buffer sizing, not a complete board or network configuration. Kconfig
selection alone does not initialize NETC or configure its PHYs. Replace
the example PHY selection with the model selections needed by the board.

Board integration
~~~~~~~~~~~~~~~~~

Call ``imxrt_netc_initialize(&config)`` from board late initialization with
a ``static const`` configuration using the public types in
``imxrt_netc.h``. The driver copies the configuration. Supply all interfaces
together; shared IERB setup runs once.

* Match the board configuration to the Kconfig selections. For
  ``CONFIG_IMXRT_NETC_ENETC0``, set both ``enetc[0].enabled`` and
  ``enetc[0].ports[0].enabled`` to ``true``; its other port slots must be
  disabled. For each selected ``CONFIG_IMXRT_NETC_ENETC1_PORTn``, set
  ``enetc[1].enabled`` and ``enetc[1].ports[n].enabled`` to ``true``.
  These slots describe external switch ports, not the CPU port.
  Each enabled controller needs at least one enabled port. Disabled entries
  are ignored; Kconfig does not populate these board configuration fields.
* Give each controller a distinct valid unicast MAC and an exclusive
  ``msgintr`` instance (1--6), including its NVIC interrupt and all three
  message registers. MAC changes take effect at the next interface up.

PHY configuration
"""""""""""""""""

Each enabled port must supply its strapped ``phyaddr`` (0--31) and
``interface`` in ``imxrt_netc_port_s``. NETC selects PHYLIB; board Kconfig
must enable the corresponding PHY drivers. PHYLIB matches drivers by the
hardware PHY ID and owns model-specific initialization and link decoding.
MDIO endpoints are hardware-selected, not board-selectable.

Set ``interface`` to ``PHY_INTERFACE_MII``, ``PHY_INTERFACE_RMII``,
``PHY_INTERFACE_RMII_PHY_CLK_OUT``, or an appropriate
``PHY_INTERFACE_RGMII`` variant. For RMII, use ``PHY_INTERFACE_RMII`` when
NETC supplies the 50 MHz reference clock, or
``PHY_INTERFACE_RMII_PHY_CLK_OUT`` when the PHY supplies it. NETC and the
PHY driver derive clock direction from this single interface setting.

Enable ``CONFIG_IMXRT_NETC_PHYINIT`` only if the board needs the optional
``board_netc_phyinit(unsigned int if_idx,
const struct imxrt_netc_port_s *config)`` hook, and provide that function
in board code. It runs for each enabled port during port preparation,
after interface mode and clock-direction setup but before MDIO probing.
``if_idx`` identifies physical link 0--4. Use this hook for board-specific
GPIO/reset setup and complete any required reset delays before returning.

The separate optional ``phy.ops->phyinit`` callback runs after PHY
identification and before advertisement. It receives a PHY descriptor
with the resolved address, MDIO operations and the hardware-selected
MDIO endpoint. Use these supplied parameters for PHY-specific register
setup, and restore the default PHY page before returning.

The board owns pinmux, clock direction, PHY reset/timing, NETCMIX reset
release and TRDC permissions. Stabilize the NETC root clock before
initialization and keep it unchanged; its frequency must be supported and
in whole MHz. RMII MAC clock output requires SION on the reference-clock
pad. Apply RGMII delays at only one end and advertise only board/PHY-supported
modes.

DMA uses aligned heap allocations with cache maintenance and address
translation through ``imxrt_dma_address()``. Retain the supported TCM
partition (256 KB M7 DTCM or 128 KB M33 system TCM) and ensure system-bus
accessibility, TRDC permissions and external-memory setup. Size the common
IOB pool for aggregate traffic when both interfaces operate.

Operation and validation
~~~~~~~~~~~~~~~~~~~~~~~~

Interface names depend on successful registrations and other network
devices; identify interfaces by MAC rather than hardware index.
Initialization stops at the first error, leaving earlier registrations
active. Do not blindly retry after partial success.

ENETC0 enables its MAC after full-duplex negotiation and removes carrier
on link loss or PHY errors. ENETC1 carrier requires an operational CPU path
and at least one selected external link. Interface down and TX-stall
recovery reset only the affected ENETC function; external switching and
link monitoring continue for ENETC1. Failed recovery leaves the interface
failed, and DMA references are retained if quiesce/reset fails.

PHY ioctls use Clause 22 registers 0--31. ENETC0 uses the resolved PHY
address directly; ENETC1 encodes the switch port in ``phy_id[9:8]`` and
address in ``phy_id[4:0]``. ``SIOCGMIIPHY`` preserves the requested port.
Register access returns ``ENETDOWN`` after interface down disables PF
memory access; address queries do not access MDIO. MDIO helpers require
task context and an initialized endpoint; runtime multi-register PHY
paging requires the network lock.

Data and Instruction Cache
==========================

Supported i.MX RT MCUs provide separate instruction and data caches. The data cache is configured
as write-back by default where supported, but it can be changed to write-through via Kconfig.

Write-back data cache generally provides better performance than write-through, but care must
be taken because not all peripheral drivers and DMA users may be safe with write-through caching
unless proper cache maintenance is implemented.

Some devices in the i.MX RT10xx series do not support write-through data cache reliably due to an
Arm erratum. On those parts, write-back should be used instead.

Newer devices, such as the i.MX RT1170, support write-through data cache operation correctly.

Tickless OS
===========

With Tickless OS, the periodic, timer interrupt is eliminated and replaced with a one-shot,
interval timer, that becomes event driven instead of polled. This allows to run the MCU with
higher resolution without using more of the CPU bandwidth processing useless interrupts.

Only tickless via an alarm is currently supported for i.MX RT MCU, which can be selected by
CONFIG_SCHED_TICKLESS_ALARM option. CONFIG_USEC_PER_TICK option determines the resolution
of time reported by :c:func:`clock_systime_ticks()` and the resolution of times that can be set
for certain delays including watchdog timers and delayed work. It is important that value set in
CONFIG_USEC_PER_TICK represents the frequency of GPT timer that runs the tickless mode. Clock
source of the timer is 16.6 MHz, which is then divided by the prescaler value from 1 to 4096.
Possible values for CONFIG_USEC_PER_TICK are 10 or 100 for example.

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======
Peripheral  Support
==========  =======
ACMP        No
ADC         Yes
CAN         Yes
CSI         No
DAC         No
eLCDIF      Yes
ENC         Yes
ENET        Yes
FlexIO      Yes
GPIO        Yes
I2S         Yes
PWM         Yes
SAI         No
SPDIF       No
SPI         Yes
UART        Yes
uSDHC       Yes
USB         Yes
==========  =======

ACMP
----

The circout for comparing two analog input voltages designed to operate across the full
range of supply voltage (rail-to-rail operation).

ADC
---

ADC driver with the successive approximation analog/digital converter. The lower-half of
this driver is initialize by calling :c:func:`imxrt_adcinitialize`.

ADC module can use either continuous trigger (next conversion is started as soon as the
previous is finished) or hardware trigger. This option is selected by IMXRT_ADCx_ETC
(x = 1, 2) config option. If IMXRT_ADCx_ETC = -1 then continuous trigger is used. If
corresponding XBAR number is put in IMXRT_ADCx_ETC then that signal is used to trigger
the ADC conversion (for example PWM signal can be used as a source). For PWM XBAR options
please refer to PWM chapter of this documentation.

Hardware triggering is currently limited to maximum of 8 channels. HW trigger is automatically
disabled if there are more than 8 channels.

DMA is currently not supported for ADC modules.

CAN
---

FlexCAN driver is supported in MCUs i.MX RT1020 and higher. i.MX RT106x have both classical
CAN and also one CAN FD while i.MX RT1170 have 3 CAN FD peripherals. FlexCAN driver in imxrt
works beyond SocketCAN driver layout. The lower-half of this driver is initialize by
calling :c:func:`imxrt_cannitialize()`.

There is an booting option that automatically provides initialization of network interface
in the early stages of booting and therefore calling :c:func:`imxrt_cannitialize()` via
board specific logic is not necessary. This however works only when there is only one
interface in the chip. For running more interfaces (like CAN and Ethernet), network late
initialization must be turn on by CONFIG_NETDEV_LATEINIT and board specific logic must call
lower-half part of drivers.

CSI
---

CMOS Sensor interface which enables the chip to connect directly to external CMOS image sensors.

DAC
---

Digital/analog converter for external signal is only supported in i.MX RT1170 MCU. It is 12 bit
lower power, general purpose DAC.

eLCDIF
------

The enhanced Liquid Crystal Display interface (LCDIF) is a general purpose display controller.

ENC
---

The enhanced quadrature encoder/decoder module supported in i.MX RT1015 and higher. The lower-half
of this driver is initialize by calling :c:func:`imxrt_qeinitialize`.

ENET
----

Ethernet driver supported in i.MX RT1020 and higher. The lower-half of this driver is initialize
by calling :c:func:`imxrt_netnitialize`.

There is an booting option that automatically provides initialization of network interface
in the early stages of booting and therefore calling :c:func:`imxrt_cannitialize()` via
board specific logic is not necessary. This however works only when there is only one
interface in the chip. For running more interfaces (like CAN and Ethernet), network late
initialization must be turn on by CONFIG_NETDEV_LATEINIT and board specific logic must call
lower-half part of drivers.

FlexIO
------

A configurable module providing a range of functionality like emulation of a variety of
serial/parallel communication protocols, flexible 16-bit timers or programmable logic blocks.
This module is supported in i.MX RT1010 and higher.

GPIO
----

Pins can be configured using :c:func:`imxrt_config_gpio` function. Writing to pins is
done by :c:func:`imxrt_gpio_write` function and reading is done by :c:func:`imxrt_gpio_read`.

MCUs i.MX RT1060 and higher includes both standard speed GPIOs (1-5) and high speed
GPIOS (6-9). Regular and high speed GPIO share the same pins (GPIO1 is with GPIO6 etc),
therefore IOMUXC_GPR_GPR26-29 registers are used to determine what module is used for the
GPIO pins.

I2C
---

Inter-Integrated Circout module supporting an interface to an I2C bus as master and/or
as a slave. The lower-half of this driver is initialize by calling :c:func:`imxrt_i2cbus_initialize`.

PWM
---

Pulse width modulator supported in i.MX RT1010 and higher. Multiple channels option is available.
Output on pin B is currently supported only as a complementary option to pin A.
The lower-half of this driver is initialize by calling :c:func:`imxrt_pwminitialize`.

PWM module can be synchronized by an external signal. The external signal used for synchronization
is selected by IMXRT_FLEXPWMx_MODx_SYNC_SRC config option. The number in IMXRT_FLEXPWMx_MODx_SYNC_SRC
corresponds with the XBAR number. Following numbers can be used for synchronization of PWMs with other
PWM module when using iMXRT1020, iMXRT1050 or iMXRT1060.

- PWM1 Module 1 = 40
- PWM1 Module 2 = 41
- PWM1 Module 3 = 42
- PWM1 Module 4 = 43
- PWM2 Module 1 = 44
- PWM2 Module 2 = 45
- PWM2 Module 3 = 46
- PWM2 Module 4 = 47
- PWM3 Module 1 = 48
- PWM3 Module 2 = 49
- PWM3 Module 3 = 50
- PWM3 Module 4 = 51
- PWM4 Module 1 = 52
- PWM4 Module 2 = 53
- PWM4 Module 3 = 54
- PWM4 Module 4 = 55

iMXRT1170 has different XBAR connections:

- PWM1 Module 1 = 74
- PWM1 Module 2 = 75
- PWM1 Module 3 = 76
- PWM1 Module 4 = 77
- PWM2 Module 1 = 78
- PWM2 Module 2 = 79
- PWM2 Module 3 = 80
- PWM2 Module 4 = 81
- PWM3 Module 1 = 82
- PWM3 Module 2 = 83
- PWM3 Module 3 = 84
- PWM3 Module 4 = 85
- PWM4 Module 1 = 86
- PWM4 Module 2 = 87
- PWM4 Module 3 = 88
- PWM4 Module 4 = 89

Option IMXRT_FLEXPWMx_MODx_TRIG allows the module to generate a trigger signal. The trigger is generated on
timer capture of either period or duty cycle value based on the configuration.

SAI
---

Synchronous audio interface provided by I2C module. Supported in i.MX RT1015 and higher.

SPDIF
-----

Sony/Philips digital interface audio block. It is a stereo transceiver that allows the
processor to receive and transmit digital audio. Supported in i.MX RT1010 and higher.

SPI
---

Serial Peripheral interface module that supports an interface to an SPI bus as a master
and/or a slave. The lower-half of this driver is initialize by calling :c:func:`imxrt_lpsibus_initialize`.

UART
----

Universal Asynchronous Receiver/Transmitter module. UART is initialized automatically during
MCU boot.

uSDHC
-----

The uSDHC driver provides SD/MMC support and is available on the i.MX RT118x.
The i.MX RT1180 EVK NSH configuration enables USDHC1 for its onboard MicroSD
socket.

USB
---

Console communication over USB is supported via CDC-ACM. Only USB Device is currently supported
for i.MX RT in NuttX

Multicore: RT1170 CM4 over RPTUN
================================

The i.MX RT1170 pairs the Cortex-M7 with a Cortex-M4. NuttX runs on the CM7 and can load, release
and hold a firmware on the CM4 through the NuttX RPTUN device on top of the OpenAMP framework
(``CONFIG_IMXRT_RPTUN``, which selects ``CONFIG_RPTUN_LOADER``). The CM4 firmware is the virtio
device side and provides the resource table in its ELF image; it may be bare metal. The table must
carry a carveout resource for the shared window after the table: NuttX 13 rptun places the vrings
and the rpmsg buffers in it and refuses the device without it, NuttX 12.12 rptun parses it and lays
out the window on its own.

The board registers the remote with :c:func:`imxrt_rptun_init`, passing the remote name, the ELF
path, the address environment (CM4 device addresses to CM7 physical addresses), the boot vector and
whether to autostart. The option is offered only on parts with a CM4 (``IMXRT_HAVE_CM4``) and
selects ``DEV_SIMPLE_ADDRENV`` for libmetal.
The driver:

- Programs the boot vector into ``IOMUXC_LPSR_GPR0/1`` and reads it back.
- Releases the core with ``SRC_SCR.BT_RELEASE_M4``. The slice stays under reset until its first
  release and cannot be held again afterwards: a software reset of the slice is a pulse after which
  the core restarts from the boot vector. To stop the core the driver writes a ``wfi`` loop over the
  start of the image and pulses the reset, so the core parks there until the next start reloads the
  image. A reset completion is awaited only when restarting a core that was already released.
- Keeps a CM4 lockup local by setting ``SRC_SRMR`` so it does not reset the whole chip.
- Exchanges virtqueue kicks with the CM4 on MU-A channel 0. Neither side reads the mailbox word; a
  kick into a full mailbox is coalesced with the pending one and the receiver rescans every
  virtqueue. The CM4 firmware must therefore read MU-B ``RR0`` before it scans, so that a kick
  coalesced during the scan is covered by it.
- Starts the remote from :c:func:`rptun_initialize` when the board sets ``autostart``; otherwise
  ``RPTUNIOC_START`` on ``/dev/rptun/<name>`` or :c:func:`rptun_boot` starts it.

The ``fmu-v6xrt:rptun`` configuration is the reference user: ``imxrt_cm4_initialize()`` in the board
bring-up registers ``/dev/rptun/cm4`` with the memory map below, maps the shared window non-cacheable
and leaves the core held. Copy an ELF built for the CM4 to the SD card as
``CONFIG_FMU_V6XRT_CM4_FIRMWARE`` (``/mnt/sd/cm4.elf``), then from nsh::

    mount -t vfat /dev/mmcsd0 /mnt/sd
    rptun start /dev/rptun/cm4
    rptun ping /dev/rptun/cm4 100 64 3 0

Memory map rules the board must follow:

- The CM7 reaches the CM4 TCM through the LMEM backdoor window at ``IMXRT_OCRAM_M4_BASE``. Only the
  first 128 KB of that window, the CM4 code TCM, is reachable; writes to the second half do not
  reach the system TCM. The resource table, vrings and rpmsg buffers must live in the code TCM.
- The CM4 boots from the backdoor alias of its code TCM (``0x20200000``), as NXP MCMGR does; the
  CM4-native ``0x1FFE0000`` does not boot.
- The board maps the memory the CM4 writes as non-cacheable (for example an MPU region over the
  shared window) before calling :c:func:`imxrt_rptun_init`. The driver cleans the D-cache over the
  loaded image before releasing the core.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
