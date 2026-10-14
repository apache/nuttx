================
TI/Hercules RM57
================

.. tags:: chip:rm57, arch:armv7-r

The RM57 is part of TI's Hercules family of ARM Cortex-R5F safety
microcontrollers, aimed at applications that need lockstep execution and
built-in self-test (industrial, medical, and automotive safety systems).
NuttX currently supports the **RM57L843** part.

.. warning::

   Support for this chip family is new and has been tested on a single
   LAUNCHXL2-RM57L (RM57L843 Rev B). The table below says how far each
   peripheral was tested. Where it says *loopback only*, the
   peripheral's internal loopback mode was used: the driver logic is
   tested, but nothing at the pins (signal timing, a real device, a real
   bus) has been checked.

Peripheral Support
==================

The following list indicates RM57 peripherals currently supported in NuttX:

============== =====
Peripheral     Notes
============== =====
Core           Lockstep-safe start-up, PLL (300MHz GCLK, 150MHz HCLK,
               75MHz VCLK), SRAM ECC, MPU, VIM interrupt controller and
               RTI system tick. Tested on hardware with ``ostest``.
ESM            Error Signalling Module. With
               ``CONFIG_ARMV7R_DECODEFIQ`` a FIQ handler reports and
               clears group 1/2 errors (``CONFIG_RM57_ESM_PANIC`` panics
               instead). Tested on hardware. See `Safety diagnostics`_.
Self test      Optional start-up diagnostics (``CONFIG_RM57_SELFTEST``):
               PBIST, STC/LBIST, CCM-R5F, eFuse, L2RAMW. Tested on
               hardware, including fault injection.
SCI            SCI1-SCI4. SCI1/LIN1 and SCI2/LIN2 are dual-role SCI/LIN
               modules; the serial driver always runs them in SCI
               compatibility mode (GCR1 LIN_MODE=0). Interrupt-driven by
               default; optional RX/TX DMA (see `Serial DMA`_ below).
               Used for the console. Tested on hardware: console with and
               without DMA, SCI3 in internal loopback with DMA.
LIN            Configuration and polled-transfer scaffold only
               (``CONFIG_RM57_LIN``, requires ``CONFIG_EXPERIMENTAL``) -
               not a complete LIN protocol stack. See
               ``arch/arm/src/rm57/rm57_lin.h`` for what is and is not
               implemented.
DMA            32-channel, 48-request-line DMA controller
               (``CONFIG_RM57_DMA``). Used by the serial driver's RX/TX
               DMA options and by the DCAN IF3 DMA receive path.
DCAN           All four DCAN (Bosch D_CAN, CAN 2.0A/B) controllers
               (``CONFIG_RM57_DCANn``). Per instance either a character
               device (``/dev/can<n>``) or a SocketCAN network interface
               (``can<n>``); both frontends can coexist in one image on
               different instances. See `DCAN`_ below. Loopback only (the
               board has no CAN transceiver).
GIO            General purpose I/O with pin interrupts
               (``CONFIG_RM57_GIO_IRQ``). Used for the LEDs and buttons.
               Tested on hardware.
MibSPI         SPI master on all five MibSPI modules
               (``CONFIG_RM57_MIBSPIn``). See `MibSPI`_ below. Loopback
               only.
I2C            I2C1 and I2C2 master (``CONFIG_RM57_I2Cn``). See `I2C`_
               below. Loopback only.
ADC            Both MibADC modules (``CONFIG_RM57_ADC1`` /
               ``CONFIG_RM57_ADC2``), 10- or 12-bit. Each of a module's
               three conversion groups is registered as its own
               ``/dev/adcN``. Calibration, self test, magnitude compare
               interrupts, hardware triggers, DMA result transfer and the
               enhanced channel selection mode are all supported, along with
               the three on-die temperature sensors
               (``CONFIG_RM57_ADC_TEMPSENSOR``). See `ADC (MibADC)`_ below.
               Conversions and temperature sensors tested on hardware;
               DMA, self test and magnitude compare are build-tested only.
EMAC           10/100 Ethernet MAC + EMAC Control module + MDIO
               (``CONFIG_RM57_EMAC``). Single netdev interface (``eth0``),
               MII or RMII (``RM57_EMAC_MII`` / ``RM57_EMAC_RMII``). See
               `EMAC/MDIO (Ethernet)`_ below. MII tested on hardware.
============== =====

.. todo::

   Not implemented yet: N2HET, ePWM, eCAP, eQEP, EMIF, the CRC module,
   the RTI digital watchdog and flash programming. Contributions are
   welcome.

Safety diagnostics
==================

``CONFIG_RM57_SELFTEST`` runs the start-up diagnostics before the OS
starts: an ESM group 3 check, the PBIST ROM tests (with the errata
PBIST#4 workaround), the STC self-check and CPU logic BIST, the four
CCM-R5F diagnostics in all three modes, the eFuse controller tests
(power-on reset only), and the L2RAMW ECC and address decode tests. The
result of each test is printed on the console. A failing test drives
nERROR, and with ``CONFIG_RM57_SELFTEST_HALT_ON_FAIL`` stops the boot.

The CCM-R5F is off while a debugger is attached, so the ROM tests, the
STC and the CCM diagnostics report ``skipped`` in a debug session. The
``CONFIG_RM57_SELFTEST_INJECT_*`` options inject one fault per build to
check that the detection path works. All five injected faults were
detected on the board. The ``selftest`` board configuration shows the
console output.

At run time, VIM channel 0 (the ESM high-level interrupt) is hard-wired
to FIQ. With ``CONFIG_ARMV7R_DECODEFIQ`` the ESM handler decodes,
reports and clears each pending channel. Without that option the FIQ
vector does nothing, and a latched nonmaskable group 2 error is never
acknowledged.

MibSPI
======

All five MibSPI modules work as SPI masters through the standard
``spi_dev_s`` interface. Each module can be set up in one of three ways:

- compatibility mode with polled transfers (default);
- compatibility mode with DMA (``CONFIG_RM57_MIBSPIn_DMA``);
- multi-buffer mode using transfer group 0
  (``CONFIG_RM57_MIBSPIn_MULTIBUF``).

Chip selects are driven as GIO. On this board only MibSPI1 and MibSPI3
are routed to headers, and they are registered as ``/dev/spi1`` and
``/dev/spi3``.

``CONFIG_RM57_MIBSPI_LOOPBACK`` adds a start-up test. It uses
``SPIGCR1.LOOPBACK`` and sends patterns of 1 to 1000 words, 8 and 16 bits
wide, at 1MHz and 37.5MHz, then reads them back. Every mode passes this
test on both modules. Internal loopback cannot show the clock polarity
and phase at the pins, so the CPOL/CPHA mapping and the real SPICLK rate
are not measured yet.

I2C
===

I2C1 and I2C2 work as interrupt-driven masters through ``i2c_master_s``
(standard and fast mode, repeated START, ``I2C_M_NOSTART``,
``CONFIG_I2C_RESET``). Slave mode and DMA are not supported. The
LAUNCHXL2-RM57L has no I2C pull-up resistors, so a real bus needs
external ones (2.2k to 4.7k).

``CONFIG_RM57_I2C_LOOPBACK`` adds a start-up test that uses digital
loopback (``MDR.DLB``) on each bus at 100kHz and 400kHz. On the board,
all tests pass, and a timed 2048-byte write takes 101% of the nominal
time. Digital loopback only works for master transmit, so master
receive, real ACK/NACK handling and the SCL timing at the pin are not
measured yet.

DCAN
====

Each of the four DCAN controllers has 64 ECC-protected message objects.
The driver partitions them into a transmit FIFO at the top of the range
(``CONFIG_RM57_DCAN_TXFIFO_DEPTH``, default 8) and receive filter FIFOs
below it (``CONFIG_RM57_DCAN_RXFIFO_DEPTH`` objects per acceptance
filter, up to ``CONFIG_RM57_DCAN_NFILTERS_MAX`` filters, with an
accept-everything filter installed while no user filter is active).
Message-object interrupts for reception are routed to the DCAN's level 1
interrupt line, transmission and status/error to level 0.

Per instance, ``RM57_DCANn_CHARDRIVER`` selects the classic character
driver (bit timing, acceptance filter, loopback/silent mode, bus-off
recovery, NART and ABOM ioctls; error reporting with
``CONFIG_CAN_ERRORS``) and ``RM57_DCANn_SOCKET`` selects SocketCAN
(``candump``/``cansend`` work out of the box; error frames with
``CONFIG_NET_CAN_ERRORS``, bit timing via ``SIOCGCANBITRATE`` /
``SIOCSCANBITRATE``).

Three receive paths are available per instance:

- ``RM57_DCANn_RX_PIO`` (default): interrupt per message, read through
  interface register set 2.
- ``RM57_DCANn_RX_IF3IRQ``: the message handler auto-copies received
  objects into interface register set 3; the dedicated IF3 interrupt
  reads them without any interface-register busy-wait.
- ``RM57_DCANn_RX_IF3DMA`` (``CONFIG_EXPERIMENTAL``): as above, but a
  DMA channel in auto-initiation mode moves the IF3 image to memory;
  a watchdog guards against lost request pulses.

The CAN clock (VCLKA1) is mapped to VCLK (75 MHz with the default board
clocking); the bit-timing search targets a 75% sample point, matching
TI's HALCoGen defaults (500 kbit/s -> BRP=10, TSEG1=10, TSEG2=4).
DCAN1-DCAN3 use dedicated balls (A10/B10, H2/H1, M18/M19); DCAN4 is
pin-multiplexed with GIOB[2] and GIOB[3] (F2/W10) and the board claims
those balls when ``RM57_DCAN4`` is enabled.  The LAUNCHXL2-RM57L has no
CAN transceiver on board - every DCAN ball runs to a BoosterPack header
at CMOS levels, so a real bus needs an external transceiver, while the
loopback test mode still drives the TX ball for logic-analyzer capture.
The board configurations ``can`` (character driver,
loopback self-test), ``cansock`` (SocketCAN with candump/cansend) and
``candma`` (IF3 DMA receive path) are provided. In loopback on the board,
``can -n 8`` receives all 8 frames, and the bit timing the driver
computes is the same as HALCoGen's. A real bus (ACK from another node,
arbitration, bus-off recovery) has not been tested.

Serial DMA
==========

Each SCI port can optionally use the DMA controller for RX and/or TX,
independently, via ``CONFIG_RM57_SCIn_RXDMA`` / ``CONFIG_RM57_SCIn_TXDMA``
(n = 1-4; both require ``CONFIG_RM57_DMA``). Because SCI1/LIN1 and
SCI2/LIN2 are run in SCI-compatibility mode (single-buffer, MBUF_MODE=0),
the SCI module generates one DMA request per byte in each direction (TRM
SPNU562A section 28.2.3) - there is no multi-byte burst transfer for a
single UART frame.

RX DMA writes into a per-port circular ring buffer sized by
``CONFIG_RM57_SERIAL_RXDMA_BUFFER_SIZE`` (rounded up to a multiple of the
Cortex-R5's 32-byte D-cache line size). New bytes are picked up when the
DMA half/full ("HBC"/"BTC") interrupt fires, which alone would leave
short, infrequent messages sitting in the ring buffer until it is half
full. Unlike, say, the STM32 USART, the SCI has no idle-line interrupt
to signal the end of a shorter burst, so ``rm57_serial_dma_poll()``
flushes the rings instead.

``CONFIG_RM57_SERIAL_RXDMA_POLL`` (enabled by default whenever a port
uses RX DMA) dedicates **RTI compare 1** to calling it at
``CONFIG_RM57_SERIAL_RXDMA_POLL_HZ`` (1 kHz by default), which bounds
the added receive latency by one poll period and is what makes an RX
DMA console usable interactively. Compare 1 shares free-running counter
0 with the system tick on compare 0, so no additional timer hardware is
consumed. Disable the option if every RX DMA port is used purely for
bulk transfers and the interrupt is not wanted.

The underlying hardware DMA request lines used are fixed per SCI
instance (datasheet SPNS215C Table 6-41):

================ ========== ==========
SCI instance     RX request TX request
================ ========== ==========
SCI1/LIN1        28         29
SCI3             30         31
SCI2/LIN2        40         41
SCI4             42         43
================ ========== ==========

ADC (MibADC)
============

The RM57L843 has two MibADC modules. ADC1 serves 32 analog inputs and ADC2
serves 25, of which 16 are the same balls ADC1 sees. Both are successive
approximation converters that run at either 10- or 12-bit resolution, and
both are enabled independently with ``CONFIG_RM57_ADC1`` and
``CONFIG_RM57_ADC2``.

Conversion groups
-----------------

Each module contains three independent conversion groups - the Event group,
Group1 and Group2. A group owns its channel selection mask, trigger source,
sampling window, slice of the results memory and VIM interrupt channel, so
the driver registers each enabled group as a separate character device.
Devices are numbered ``/dev/adc0`` upwards in the order the groups are
enabled, ADC1 before ADC2 and Event before Group1 before Group2.

A group is enabled with ``CONFIG_RM57_ADCn_EV``, ``..._G1`` or ``..._G2``,
and configured with the options underneath it: ``..._CHANMASK`` selects the
channels, ``..._ACQ`` the sampling window, ``..._BUFS`` its share of the
results memory, ``..._THRESHOLD`` how many results accumulate before the
group interrupts, and ``..._CONTINUOUS`` whether the channels are converted
once per trigger or continuously.

Results memory
--------------

Each module has a 64-word results RAM that its three groups divide between
them. The hardware allocates it in *pairs* of buffers through a single
register, so the split is computed for the whole module from every group's
``..._BUFS`` value. The Event group and Group1 counts must therefore be even,
and a module's three groups must not ask for more than 64 words in total;
both rules are checked at compile time.

The RAM is auto-initialized to zero at start-up through the system module's
memory initialization controller, which is what makes it safe to switch on
the parity checking enabled by default with ``CONFIG_RM57_ADC_PARITY``. A
parity error is reported to the Error Signalling Module.

Conversion clock
----------------

ADCLK is VCLK divided by a prescaler. Rather than expose the divider,
``CONFIG_RM57_ADCn_CLOCK`` names the wanted frequency and the driver derives
the prescaler from ``BOARD_VCLK_FREQUENCY``, rounding so the result never
exceeds the request. SPNS215 specifies a ``tc(ADCLK)`` of at least 33ns, an
ADCLK ceiling of 30MHz, which is asserted at compile time. The default of
9.375MHz is VCLK/8 on a board running VCLK at 75MHz.

Triggering
----------

Group1 and Group2 are software triggered by default: writing the channel
selection starts a conversion. Setting ``CONFIG_RM57_ADCn_Gx_HWTRIG`` makes
them wait for one of eight hardware sources instead, selected with
``..._TRIGSRC`` - the ADxEVT pin, four N2HET1 outputs, an RTI compare match
or two GIOB pins. The Event group has no such choice: it is always started
by its trigger source, so only the source and its edge are configurable.

When the ADxEVT pin is not used as a trigger it can serve as a general
purpose I/O through the ``ANIOC_RM57_EVTPIN`` ioctl. AD1EVT is on ball N19,
which can also carry MII_RX_ER. The LAUNCHXL2-RM57L takes MII_RX_ER from its
dedicated ball U5 instead, so AD1EVT is still available when
``CONFIG_RM57_EMAC`` is enabled.

Result transfer
---------------

By default a group's interrupt handler drains the results FIFO and hands
each sample to the upper half. Selecting ``CONFIG_RM57_ADC1_Gx_DMA`` moves
them with the DMA controller instead, into a two-half ring that lets the ADC
fill one half while the other is delivered as a batch. ADC1 has dedicated
DMA request lines; ADC2's are alternate functions shared with MibSPI1 and
MibSPI3 and would need the system module's request source mux reprogrammed,
so ADC2 has no DMA option.

Calibration and diagnostics
---------------------------

With ``CONFIG_RM57_ADC_CALIBRATE`` (the default) the driver runs the offset
error calibration when a module's first group is registered: four internal
reference voltages are converted, and the deviation of their average from
mid scale is loaded into the correction register that the hardware adds to
every subsequent result. ``ANIOC_RM57_CALIBRATE`` repeats it on demand and
``ANIOC_RM57_MIDPOINTCAL`` runs the differential variant, which cancels the
mismatch of the internal bridge resistors.

``ANIOC_RM57_SELFTEST`` drives each reference onto one input through an
internal test resistor and classifies the channel as healthy, open, or
shorted to ground or supply.

``CONFIG_RM57_ADC_MAGCOMP`` builds support for the three hardware magnitude
comparators of each module, which interrupt when a monitored channel crosses
a threshold - either a fixed value or the latest result of a second channel.
The generic ``ANIOC_WDOG_UPPER`` and ``ANIOC_WDOG_LOWER`` commands are backed
by the first two of them.

Channel order
-------------

Channels are normally converted in ascending channel number. ADC1 also
supports a look-up table based order, built with
``CONFIG_RM57_ADC_ENHANCED_CHSEL`` and loaded through
``ANIOC_RM57_SET_CHNSEQ``, which converts an arbitrary user-defined sequence
and can drive an external analog multiplexer alongside it. ADC2 does not
implement this mode.

On-chip temperature sensors
---------------------------

The device carries three on-die temperature sensors, each multiplexed onto one
ADC channel: sensor 1 on ADC1 channel 31, sensors 2 and 3 on ADC2 channels 31
and 30. They are enabled with ``CONFIG_RM57_ADC_TEMPSENSOR`` and selected
individually with ``CONFIG_RM57_ADC_TEMP1`` to ``..._TEMP3``.

Out of reset all three are powered down and their channels are wired to the
external balls instead, so the driver has to release them and re-mux them
through the IOMM. Sensor 1 shares its channel with the ``AD1IN[31]`` ball
(T13), which means enabling it gives up that ball as an analog input; sensors
2 and 3 sit on channels that have no ball at all, so they cost nothing.

Readings are calibrated. During device test TI records three
(ADC reading, temperature) points per sensor in OTP, and the driver reads them
back and interpolates, so results come out in real degrees Celsius rather than
raw codes. The calibration was taken with ADREFHI at 3.3V, which is what this
board supplies, so no scaling correction is applied. If OTP holds no usable
calibration the driver reports the raw reading and refuses to invent a
temperature.

By default each enabled sensor is registered as a character device,
``CONFIG_RM57_ADC_TEMP_PATH`` with the device number appended - ``/dev/temp0``
and upwards, numbered from ``CONFIG_RM57_ADC_TEMP_DEVNO``. One ``read()``
returns a whole ``struct rm57_adc_temp_s``::

    struct rm57_adc_temp_s temp;
    int fd = open("/dev/temp0", O_RDONLY);

    read(fd, &temp, sizeof(temp));
    if (temp.calibrated)
      {
        printf("%d.%03d C\n", temp.millicel / 1000, abs(temp.millicel % 1000));
      }

Every read runs a fresh conversion and none of them reports end of file, so
read a fixed length rather than pointing ``cat`` at the node.

``CONFIG_RM57_ADC_TEMP_UORB`` swaps that for the sensor framework, registering
each sensor as ``/dev/uorb/sensor_tempN`` instead::

    nsh> sensortest -n 5 temp0

That is the friendlier interface - ``poll()`` and the ``sensortest``
application come with it - but it selects ``CONFIG_SENSORS`` and pulls in the
whole UORB upper half, which is a lot to carry for a die temperature. It also
costs the calibration flag, since ``struct sensor_temp`` has nowhere to put
it: an uncalibrated sensor fails the read with ``ENODATA`` rather than handing
back a qualified reading.

Blocking reads are not among what it buys, though. These sensors are
fetch-mode, and the upper half serves a blocking reader by parking it on a
per-subscriber semaphore that only a pushing driver ever posts - so a
fetch-mode node must be opened ``O_NONBLOCK``, which is what ``sensortest``
does and what ``sensor_poll()`` is written to accommodate. Pointing ``cat`` at
one of these nodes hangs.

The type is ``SENSOR_TYPE_TEMPERATURE`` rather than
``SENSOR_TYPE_AMBIENT_TEMPERATURE``: these sensors measure the junction, not
the room. Readings are fetched on demand - a conversion runs when someone
reads - so there is no sampling period to configure and ``SNIOC_SET_INTERVAL``
is accepted and ignored.

Under either arrangement the sensors are also reachable through the
``ANIOC_RM57_GET_TEMP`` ioctl on any ``/dev/adcN``, which fills in the same
``struct rm57_adc_temp_s``.

A temperature reading borrows one conversion group for the duration of a
single conversion, saving and restoring its channel selection, mode and
interrupt enables. A group that is converting continuously therefore loses a
few microseconds of sampling to each temperature read.

Shared channels
---------------

16 channels are wired to both modules. When both convert the same ball their
sampling windows must either match exactly or not overlap at all, with at
least two ADC cycles between them (TRM SPNU562A section 22.1). The simplest
way to satisfy this is to give both groups the same ``..._ACQ`` value.

The board configuration ``rm57l843-launchxl2:adc`` exercises ADC1 Group1 and
ADC2 Group1 with the ``adc`` example.

EMAC/MDIO (Ethernet)
=====================

The RM57L843 has a single 10/100 Ethernet MAC built from three sub-blocks
(SPNU562A chapter 31): the EMAC module itself (TX/RX CPPI DMA engines,
statistics, MAC address/hash filtering), the EMAC Control module (VIM
interrupt pacing/routing), and the MDIO module (a shared PHY management
bus, independent of the MAC). ``CONFIG_RM57_EMAC`` enables all three
together and registers one ``netdev_register()`` interface
(``NET_LL_ETHERNET``) - there is no separate character-device frontend,
unlike DCAN.

Descriptor rings (TX: ``CONFIG_RM57_EMAC_NTXDESC``, default 8; RX:
``CONFIG_RM57_EMAC_NRXDESC``, default 16) live in the EMAC's internal 8KB
CPPI RAM, which this port's MPU maps as non-cacheable device memory, so
descriptor fields need no cache maintenance. Packet buffers are ordinary
cacheable SRAM; TX uses a small zero-copy free-buffer pool (buffers are
swapped into ``dev->d_buf``, not copied) while RX buffers are fixed 1:1
with RX descriptors and processed synchronously in place. Both rings are
managed as CPPI "append to a running queue" chains (SPNU562A Section
31.2.6.2), including the documented EOQ-race recovery for restarting a
channel that halted between the driver reading its tail and appending a
new descriptor.

``CONFIG_RM57_EMAC_MII`` / ``CONFIG_RM57_EMAC_RMII`` select the PHY
interface mode. The IOMM reset default is RMII, but the LAUNCHXL2-RM57L's
DP83630 PHY is strapped for MII with no way to change it, so that board
must select MII. The EMAC's internal logic clock (``VCLKA4_DIVR_EMAC``)
must be exactly 25MHz for MII or 50MHz for RMII (SPNS215 Section 6.6.3);
see the ``BOARD_VCLKA4_*`` comment in the board's ``board.h``. Only MII
has been tested on hardware. The RMII path (clocked from PLL2) builds,
but no board available to this port wires RMII.

The MDIO module addresses up to 32 PHYs (``CONFIG_RM57_EMAC_PHYADDR``,
default 1) over a single-transaction register interface
(``USERACCESS0``), and generates a link-change interrupt
(``LINKINT0``, routed through the ``C0_MISC_PULSE`` VIM channel) that
this driver always keeps enabled for its own ``netdev_carrier_on/off``
tracking; ``CONFIG_RM57_EMAC_PHY_INTERRUPT`` additionally exposes that
event to userspace via ``SIOCMIINOTIFY``/``phy_notify``.
``CONFIG_RM57_EMAC_AUTONEG`` (default) negotiates speed/duplex; disabling
it forces the mode selected by ``CONFIG_RM57_EMAC_ETHFD`` /
``CONFIG_RM57_EMAC_ETH100MBPS``. ``CONFIG_RM57_EMAC_LOOPBACK`` sets
``MACCONTROL.LOOPBACK`` and skips the PHY link wait entirely, so the
descriptor ring and interrupt path can be exercised standalone without a
PHY or cable - the Ethernet equivalent of ``RM57_DCAN_LOOPBACK``.

Multicast group membership (``CONFIG_NET_MCASTGROUP``) is implemented via
the EMAC's 64-bit hash address table (SPNU562A Section 31.5.37, 6-bit
XOR-fold of the destination address), with a per-bit reference count
since two joined addresses can hash to the same bit.

The four EMAC Control module interrupt pulses this device routes to the
VIM (SPNS215 Table 6-39) are, in channel order: ``C0_MISC_PULSE`` (76,
statistics/host-error/MDIO events), ``C0_TX_PULSE`` (77, TX completion),
``C0_THRESH_PULSE`` (78, RX flow-control threshold - not used by this
driver), ``C0_RX_PULSE`` (79, RX completion). Each ISR follows the
disable-service-acknowledge-reenable discipline SPNU562A Section
31.2.17.3 documents: the top-half clears the relevant EMAC Control
enable bit and defers to the low-priority work queue, which does the
real work, writes the ``MACEOIVECTOR`` acknowledge key, then re-enables
the source last.

The ``eth`` board configuration (``CONFIG_NETDEV_LATEINIT``) derives a
locally-administered MAC address from the device's unique die ID at
board bring-up; without ``CONFIG_NETDEV_LATEINIT`` the interface
self-registers via ``arm_netinitialize()`` with a fixed placeholder
address (``02:00:00:00:00:01``) instead.

Tested on the LAUNCHXL2-RM57L at 100Mbit/s full duplex: DHCP, ping (RTT
2-7ms through a home router), DNS, NSH over telnet and HTTP all work.
With ``iperf`` the best measured TCP throughput is 19.4Mbit/s for
transmit (board as client) and 9.2Mbit/s for receive (board as server).
After a 10MB transfer, every EMAC error counter reads zero. On some boots
the DHCP server does not answer the first DISCOVERs, so the ``eth``
configuration raises ``CONFIG_NETUTILS_DHCPC_RETRIES`` to 10.

Supported Boards
=================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
