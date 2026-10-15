=====================
LAUNCHXL2-RM57L
=====================

.. tags:: chip:rm57, arch:armv7-r, vendor:ti

The `LAUNCHXL2-RM57L <https://www.ti.com/tool/LAUNCHXL2-RM57L>`__ is a
LaunchPad-format evaluation board from Texas Instruments built around the
Hercules RM57L843 safety microcontroller (dual Cortex-R5F cores in
lockstep). It's aimed at evaluating TI's Hercules safety MCU family rather
than any particular application.

.. warning::

   This board port is new. The console, LEDs, buttons, serial DMA,
   Ethernet, ADC, temperature sensors and start-up diagnostics are tested
   on this board. DCAN, MibSPI and I2C are only tested in their internal
   loopback modes, because the LaunchPad has no CAN transceiver and no
   SPI or I2C device. Pin assignments were checked against the board
   schematic (SPRR397). The PLL values come from TI's HALCoGen project,
   and the CAN bit timing and the I2C transfer times measured on the
   board confirm them.

Features
========

* TI Hercules RM57L843 microcontroller
* Dual-core lockstep ARM Cortex-R5F, running at 150 MHz HCLK (300 MHz
  PLL/GCLK) in the current clock configuration
* 4 MB of on-chip program flash, 512 KB of on-chip SRAM
* Onboard XDS110 debug probe (JTAG)
* 2 user LEDs and 2 user buttons
* 10/100 Ethernet (DP83630 PHY, MII)
* MibSPI1 and MibSPI3 on the BoosterPack headers; I2C1 and I2C2 on the
  BoosterPack and proto headers (no pull-up resistors on the board)
* SCI1 wired up as the serial console; SCI2-SCI4 are supported by the
  driver but not enabled by this board's default configuration (and
  SCI2-SCI4 pin-mux is not programmed by this board port - see the
  `Pin Mapping`_ note below)
* Both MibADC modules, with analog inputs broken out on the BoosterPack
  and edge headers and the ADC reference tied to the 3.3V analog supply
* An on-board trim potentiometer wired to AD1IN[6], which gives the ADC an
  adjustable 0V to +3V3 input with no external wiring (see the ``adc``
  configuration below)
* Three on-die temperature sensors, read through the ADC and calibrated from
  factory data in OTP

.. note::

   The :doc:`RM57 chip documentation page </platforms/arm/rm57/index>`
   lists the chip peripherals that are not implemented yet (N2HET, ePWM,
   eCAP, eQEP and others).

Buttons and LEDs
================

LEDs
----

The LAUNCHXL2-RM57L has two user LEDs, labeled **B6** and **B7** on the
board silkscreen, driven by GIOB[6] and GIOB[7] respectively. They are
active high. With ``CONFIG_ARCH_LEDS`` NuttX uses them to show the OS
state; otherwise they are available through ``CONFIG_USERLED``.

User Buttons
------------

The two user buttons, **B4** and **B5**, are on GIOB[4] and GIOB[5]. They
are active low and use the internal pull-up. They are supported through
``board_buttons()``, and with ``CONFIG_ARCH_IRQBUTTONS`` and
``CONFIG_RM57_GIO_IRQ`` through ``board_button_irq()`` with an interrupt
on both edges. The ``buttons`` configuration has a small demo: holding
B4 lights LED B6, and holding B5 lights LED B7.

Pin Mapping
===========

Only the pins used by the currently supported peripherals are listed.

.. list-table::
   :widths: auto
   :header-rows: 1

   * - Pin
     - Signal
     - Notes
   * - (n/a)
     - LIN1RX / LIN1TX
     - SCI1, used as the serial console
   * - P4 / T5
     - LIN2RX / LIN2TX
     - SCI2, multiplexed away from N2HET2[19]/N2HET2[20]
   * - W3 / N2
     - SCI3RX / SCI3TX
     - SCI3, multiplexed away from N2HET1[6]/N2HET1[13]
   * - A13 / B13
     - SCI4RX / SCI4TX
     - SCI4, multiplexed away from N2HET1[17]/N2HET1[19]
   * - J2
     - GIOB[6]
     - User LED B6
   * - F1
     - GIOB[7]
     - User LED B7
   * - (n/a)
     - GIOB[4] / GIOB[5]
     - User buttons B4 / B5, active low
   * - T9 / F4
     - MDCLK / MDIO
     - MDIO bus to the DP83630 PHY (address 1)
   * - U7 / U8 / R1 / T2 / G4 / E4
     - MII_TX_CLK / MII_TXD[0:3] / MII_TXEN
     - EMAC transmit
   * - T4 / U4 / T3 / U3 / V3 / U6 / U5
     - MII_RXCLK / MII_RXD[0:3] / MII_RX_DV / MII_RX_ER
     - EMAC receive
   * - W4 / V4
     - MII_COL / MII_CRS
     - EMAC collision and carrier sense
   * - A12
     - ECLK1
     - 25MHz reference clock for the PHY (the PHY has no crystal)
   * - E1 / A6
     - GIOA[3] / GIOA[4]
     - PHY power-down and reset, driven at bring-up
   * - J5.7 / J6.6 / J6.7
     - MIBSPI1CLK / SIMO / SOMI
     - MibSPI1, BoosterPack site 2. Chip selects nCS[0:2] on J6.3, J6.8,
       J6.9
   * - J1.7 / J2.6 / J2.7
     - MIBSPI3CLK / SIMO / SOMI
     - MibSPI3, BoosterPack site 1. Chip selects nCS[0:3] on J5.5, J2.3,
       J2.8, J2.9 (nCS[2:3] only when I2C1 is disabled)
   * - B2 / C3
     - I2C1 SDA / SCL
     - J2.8 / J2.9, taken from MibSPI3 nCS[2:3] when ``CONFIG_RM57_I2C1``
       is enabled
   * - G17 / G16
     - I2C2 SDA / SCL
     - Proto header J10.9 / J10.8
   * - T19
     - AD1IN[6]
     - On-board 1k trim potentiometer U10 (wiper, with 4700pF to GND);
       sweeps 0V to +3V3. ADC1 only - this ball has no AD2IN function
   * - J1-2 / J1-6
     - AD1IN[16] / AD2IN[0], AD1IN[17] / AD2IN[1]
     - Analog inputs on the BoosterPack header, shared between both ADCs
   * - J3-3 / J3-4
     - AD1IN[0], AD1IN[7]
     - Analog inputs on the BoosterPack header, ADC1 only
   * - J3-5 to J3-8
     - AD1IN[18:21] / AD2IN[2:5]
     - Analog inputs on the BoosterPack header, shared between both ADCs
   * - V15 / V16
     - ADREFHI / ADREFLO
     - ADC reference, tied to VCCAD (3.3V) and VSSAD through 0 ohm links

.. note::

   Most MII signals can also be routed to an alternate ball. This board
   connects the PHY to the dedicated balls listed above, which are the
   reset defaults; the alternates go to the BoosterPack headers (see
   the ``BOARD_PINMUX_EMAC`` comment in ``board.h``). The PHY is strapped
   for MII, so RMII cannot be used on this board.

.. note::

   The RM57L843 SCI1/LIN1 pins are used at their reset-default (primary)
   function, so no pin-mux configuration is required for the console.
   Header/connector pin numbers for LIN1RX/LIN1TX have not been confirmed
   against the LAUNCHXL2-RM57L schematic. SCI2/LIN2, SCI3 and SCI4 share
   their balls with N2HET1/N2HET2 and are re-multiplexed at boot by
   ``BOARD_PINMUX_INITIALIZER`` in ``include/board.h``, but only when the
   corresponding ``CONFIG_RM57_SCIn`` is selected.

Serial Console
==============

SCI1 is used as the serial console. The default configuration is:

* 9600 baud, 8 data bits, no parity, 2 stop bits

The console goes over the XDS110's USB serial port. ``ethnsh`` uses
38400 baud, because a slow console makes network warnings costly. Rates
above 38400 are not recommended: the baud divider has no fractional
part, so 115200 is off by 1.7%.

The baud rate and stop bits are configurable via ``CONFIG_SCI1_BAUD`` and
``CONFIG_SCI1_2STOP``.

Additional SCI ports (SCI2-SCI4) can be enabled via
``CONFIG_RM57_SCI2``/``RM57_SCI3``/``RM57_SCI4``, which also pulls their
RX/TX balls into the pin-mux table applied at boot; whichever one is *not*
selected as the console (``CONFIG_SCIn_SERIAL_CONSOLE``) is registered as
``/dev/ttyS0``, ``/dev/ttyS1``, etc. in SCI1-SCI4 order. SCI1/LIN1 and
SCI2/LIN2 are dual-role SCI/LIN modules; the serial driver always runs
them in SCI-compatibility mode. See
:doc:`the RM57 chip documentation </platforms/arm/rm57/index>` for the
serial RX/TX DMA options and the LIN configuration scaffold.

Power Supply
============

The LAUNCHXL2-RM57L can be powered over USB through the onboard XDS110
debug probe. Consult the
`LAUNCHXL2-RM57L user's guide <https://www.ti.com/tool/LAUNCHXL2-RM57L>`_
for the full range of supported input voltages.

Debugging
=========

The board's onboard XDS110 probe exposes a JTAG interface (Hercules'
ICEpick-C JTAG router is JTAG-only; XDS110's default SWD mode is not
usable here). OpenOCD can be used with a Cortex-R5 (``cortex_r4`` driver)
target configuration that routes through the ICEpick-C, followed by GDB
to load and debug the image.

On Rev B silicon, errata GCM#60 (SPNZ233B) can leave VCLK at the wrong
rate after an external reset, and the debugger causes one each time it
connects. If something works after a power cycle but not under the
debugger (Ethernet is the most likely), suspect this first.

Configurations
==============

nsh
---

Basic NuttShell configuration (console enabled on SCI1, at 9600 baud),
with ``ostest``::

    nsh> ostest

buttons
-------

``nsh`` with ``CONFIG_ARCH_BUTTONS``, ``CONFIG_ARCH_IRQBUTTONS`` and
``CONFIG_RM57_GIO_IRQ``, and with ``CONFIG_ARCH_LEDS`` disabled so that
the board code controls the LEDs. While B4 is held, LED B6 is lit, and while B5 is
held, LED B7 is lit. The LEDs are switched from the GIO interrupt
handler, so this checks the whole GIO interrupt path.

scitest
-------

NuttShell plus all four SCI ports, for exercising the serial driver and
the SCI pin multiplexing. SCI1 remains the console (9600 baud, 8N2, on
the XDS110 backchannel); SCI2, SCI3 and SCI4 come up at 115200 baud, 8N1
and are registered as ``/dev/ttyS0``, ``/dev/ttyS1`` and ``/dev/ttyS2``
respectively. Their RX/TX balls are multiplexed at boot (see the Pin
Mapping table above), so they must be wired to a USB-serial adapter, or
looped back to each other, to be useful.

``CONFIG_SERIAL_TERMIOS`` is enabled, and the ``serialblaster``,
``serialrx`` and ``termios`` examples are built in::

    nsh> serialrx /dev/ttyS1 1000     # count 1000 received bytes
    nsh> serialblaster /dev/ttyS1     # transmit continuously
    nsh> echo hello > /dev/ttyS2      # one-shot write

Both ``serialrx`` and ``serialblaster`` take the device path as their
first argument and default to ``/dev/ttyS0``.

scidma
------

Identical to ``scitest``, except that all four SCI ports - the console
included - run with both RX and TX DMA
(``CONFIG_RM57_SCIn_RXDMA``/``TXDMA``, eight DMA channels in total).
The two configurations are an A/B pair for the same serial tests.

RX DMA hands received bytes to the upper half when the DMA half/full
interrupt fires, i.e. only after ``CONFIG_RM57_SERIAL_RXDMA_BUFFER_SIZE
/ 2`` bytes (32 by default). To keep short messages and interactive
console input from stalling behind that threshold, RTI compare 1 calls
``rm57_serial_dma_poll()`` at ``CONFIG_RM57_SERIAL_RXDMA_POLL_HZ``
(1 kHz here), bounding the extra latency by one poll period. Both the
console and the test ports therefore behave the same as in ``scitest``,
only with the transfers themselves done by DMA.

SCI3 is also put in internal loopback (``CONFIG_RM57_SCI3_LOOPBACK``),
so ``/dev/ttyS1`` receives what it sends without any wiring. This is how
the DMA receive path was tested on the board.

can, candma, cansock
--------------------

NuttShell with DCAN1 in loopback test mode (``CONFIG_RM57_DCAN_LOOPBACK``),
so the controller receives its own frames while still driving the TX
ball (A10) for a logic analyzer. No transceiver or bus is needed.

* ``can``: character driver ``/dev/can0`` with the ``can`` example::

      nsh> can -n 8

* ``candma``: the same, with the IF3 DMA receive path
  (``CONFIG_RM57_DCAN1_RX_IF3DMA``, experimental).
* ``cansock``: SocketCAN interface ``can0`` with ``candump`` and
  ``cansend``::

      nsh> ifup can0
      nsh> candump can0 &
      nsh> cansend can0 123#DEADBEEF

eth
---

NuttShell plus the EMAC (``CONFIG_RM57_EMAC=y``, MII mode, DP83630 PHY
at address 1), with the full IPv4 stack (TCP/UDP/ICMP/ARP),
``CONFIG_NETDEV_LATEINIT`` (so board bring-up can derive a real,
die-ID-based MAC address before registering the interface - see
`EMAC/MDIO (Ethernet) <#emac-mdio-ethernet>`_ on the chip documentation
page), NSH's automatic network initialization
(``CONFIG_NSH_NETINIT``, DHCP by default), and the ``ping`` utility::

    nsh> ifconfig
    nsh> ping 192.168.1.1

If the network doesn't come up automatically, check ``ifconfig eth0``
for the assigned address and bring the link up manually with
``ifup eth0`` / ``ifdown eth0``. Some routers ignore the first DHCP
requests after the link comes up, so ``CONFIG_NETUTILS_DHCPC_RETRIES``
is set to 10.

ethnsh
------

``eth`` plus a telnet server for NSH, ``iperf`` and a small web server,
with the console at 38400 baud. From a PC on the same network::

    $ telnet <board-ip>
    $ iperf -c <board-ip>            # board receives:   nsh> iperf -s
    $ iperf -s                       # board transmits:  nsh> iperf -c <pc-ip>

The best results measured on the board are 19.4Mbit/s transmit and
9.2Mbit/s receive.

adc
---

NuttShell plus both MibADC modules (``CONFIG_RM57_ADC1=y`` and
``CONFIG_RM57_ADC2=y``, 12-bit) and the ``adc`` example. Offset
calibration runs at bring-up and the magnitude comparators are built in.

``/dev/adc0`` is ADC1 Group1 converting AD1IN[6], which the board wires to
the wiper of trim potentiometer U10. The pot's ends sit on +3V3 and GND, so
turning it sweeps the input across the full conversion range and **this
configuration needs no external wiring at all**::

    nsh> adc -n 10

Readings should track the pot from close to 0 at one end to close to 4095
at the other, since ADREFHI is the 3.3V analog supply, and hold to within
about one LSB at a fixed setting.

The offset calibration reports itself on the boot log when
``CONFIG_DEBUG_ANALOG_INFO`` is on, and is worth a glance the first time a
board runs::

    rm57_adc_calibrate: ADC1: calibration offset -1, correction 1
    rm57_adc_calibrate: ADC2: calibration offset 0, correction 0

A correction of a couple of LSB is what a healthy converter looks like. A
large one means the calibration conversion is not sampling properly, and since
the hardware adds that value to *every* subsequent result, it would shift the
whole channel.

The three on-die temperature sensors are enabled in this configuration too and
show up as character devices::

    nsh> ls /dev/temp*
    /dev/temp0
    /dev/temp1
    /dev/temp2
    nsh> hexdump /dev/temp0 -n 12

``/dev/temp0`` is sensor 1 on ADC1, ``/dev/temp1`` and ``/dev/temp2`` are
sensors 2 and 3 on ADC2. A read returns a ``struct rm57_adc_temp_s``: the raw
code, the temperature in milli-Celsius and the flag saying the OTP calibration
was usable. All three read the same die, so at idle they should agree within a
few degrees, sitting somewhat above room temperature. Warming the package with
a finger should move them within a second or two.

Turning on ``CONFIG_RM57_ADC_TEMP_UORB`` (with ``CONFIG_SYSTEM_SENSORTEST``)
replaces those nodes with UORB ones instead - that is what the ``temp``
configuration below does.

``/dev/adc1`` is ADC2 Group1 converting AD2IN[0]. That ball is shared with
AD1IN[16] and comes out on J1-2, so it floats until something drives it;
J1-1 (+3V3) and J3-2 (GND) sit next to it on the BoosterPack header, which
is enough to check both rails with a single jumper.

.. note::

   Both groups use ``_ACQ=30``, a sampling window of 32 ADCLK cycles or
   about 3.4µs, rather than the 8 the Kconfig defaults to. The trim pot
   presents up to 250 ohms at its wiper and drives the 4700pF capacitor at
   the pin, an RC of roughly 1.2µs, and a window of about three time
   constants keeps the settling error near one LSB even when AD1IN[6] is
   converted in sequence with channels at very different voltages. The
   datasheet minimum, ``td(SH)`` of 0.2µs, is far below either value.

temp
----

The ``adc`` configuration with the three on-die temperature sensors published
through the sensor framework instead of as character devices
(``CONFIG_RM57_ADC_TEMP_UORB``), plus ``sensortest`` to read them and the
sensor subsystem's debug output so the boot log carries each sensor's factory
calibration::

    nsh> ls /dev/uorb
    sensor_temp0
    sensor_temp1
    sensor_temp2
    nsh> sensortest -n 5 temp0

``sensor_temp0`` is sensor 1 on ADC1; ``sensor_temp1`` and ``sensor_temp2``
are sensors 2 and 3 on ADC2. All three read the same die at different points
on it. On a board idling at a room temperature of about 22°C they read 41.6°C,
40.0°C and 44.7°C a minute after reset, stable to about ±0.2°C - one LSB - so
a few degrees of spread between them is normal and not a fault.

During bring-up the boot log is the more informative half. Each sensor's three
OTP calibration points are printed as they were read, along with a trial
conversion::

    Sensor 1 OTP point 1: 233K (-40150 mC) reads 1427
    Sensor 1 OTP point 2: 303K (29850 mC) reads 1879
    Sensor 1 OTP point 3: 398K (124850 mC) reads 2461
    Sensor 1 reads raw 1923, 37032 mC

So the device is characterised at -40°C, +30°C and +125°C, and the reading
rises with temperature at roughly six codes per degree. Blank OTP reads as
``0xffff``; a "temperature" outside the 200-500K plausibility window is
rejected, and would suggest the OTP region is being fetched as normal memory
with errata L2FMC#5 substituting instruction opcodes for the data.

Note that the trial conversion above reads lower than the numbers further up -
37.0°C against 41.6°C - because it runs a second after reset, before the die
has warmed to its idle temperature. That drift is itself a check on the sign
of the calibration. Pressing a finger on the package makes all three readings
*fall*, which is also right: the die runs hotter than skin, so a finger is a
heatsink rather than a heat source. Use hot air if you want to confirm the
other direction.

.. note::

   Reading one of these nodes with ``cat`` will hang, and that is the sensor
   framework's fetch model rather than a driver fault: a fetch-mode sensor
   produces a sample only when someone asks for one, and the upper half parks
   a blocking reader on a semaphore that nothing in that model can post.
   ``sensor_poll()`` handles the non-blocking case instead, which is why
   ``sensortest`` - it opens ``O_NONBLOCK`` - works. Use it, or open the node
   non-blocking yourself.

spi, spitest
------------

NuttShell with MibSPI1 (compatibility mode with DMA) and MibSPI3
(multi-buffer mode), registered as ``/dev/spi1`` and ``/dev/spi3``, and
the ``spi`` tool. ``spitest`` also enables ``CONFIG_RM57_MIBSPI_LOOPBACK``,
which runs a loopback test on both buses at boot and prints one ``ok`` or
error line per pattern. To try a real exchange, connect SIMO to SOMI
(J6.6 to J6.7 for MibSPI1) and run::

    nsh> spi exch -b 1 -x 4 deadbeef

i2c
---

NuttShell with I2C1 and I2C2, registered as ``/dev/i2c1`` and
``/dev/i2c2``, and the ``i2c`` tool. ``CONFIG_RM57_I2C_LOOPBACK`` runs the
digital loopback test on both buses at 100kHz and 400kHz at boot. Each
bus ends with ``I2Cn: loopback self-test passed (0 failed)``. Enabling
I2C1 takes
MibSPI3 nCS[2:3] away. Real devices need external pull-up resistors::

    nsh> i2c bus
    nsh> i2c dev -b 2 0x03 0x77

selftest
--------

The ``nsh`` configuration with the start-up safety diagnostics enabled
(``CONFIG_RM57_SELFTEST`` and its sub-options). Before the OS starts, the
boot code runs:

* a check that no ESM group 3 error survived the reset;
* the PBIST ROM tests, preceded by the workaround for errata PBIST#4;
* the STC self-check and the CPU logic BIST (125 intervals by default). Each
  of the two ends in a CPU-only reset, so the boot code is entered three
  times;
* the four CCM-R5F diagnostics (CPU compare, VIM compare, inactivity monitor
  and power domain monitor) in self-test, error forcing and self-test error
  forcing mode;
* the eFuse controller stuck-at-zero and ECC self-tests, after a power-on
  reset only;
* the L2RAMW ECC (single-bit and double-bit) and redundant address decode
  diagnostics.

The outcome is printed on the console before the NuttShell banner::

    SELFTEST: reset cause 00008028 power-on cpu external
    SELFTEST: ESM group3: PASS
    SELFTEST: PBIST ROM init: PASS
    SELFTEST: PBIST ROM: PASS
    SELFTEST: STC self-check: PASS
    SELFTEST: STC CPU LBIST: PASS
    SELFTEST: CCM CPU compare: PASS
    SELFTEST: CCM VIM compare: PASS
    SELFTEST: CCM inactivity: PASS
    SELFTEST: CCM power domain: PASS
    SELFTEST: eFuse: PASS
    SELFTEST: L2RAM ECC: PASS
    SELFTEST: L2RAM addr decode: PASS

A failing diagnostic prints ``SELFTEST FAIL: <name> step ... info ...``,
drives nERROR active and, with ``CONFIG_RM57_SELFTEST_HALT_ON_FAIL``, stops
there.

.. note::

   Several diagnostics force a real error, so the nERROR LED flashes briefly
   on every boot. The CCM-R5F is disabled while a debugger is attached, so
   the ROM tests, the STC and the CCM diagnostics are reported as ``skipped``
   in a debug session.

The ``CONFIG_RM57_SELFTEST_INJECT_*`` choice injects one fault per build to
prove that the detection path works: a RAM algorithm on a ROM group, an STC
time-out, a wrong eFuse self-test signature, a CPU compare mismatch or an
L2RAM double-bit error. The last two are left pending and are reported at
run time by the ESM handler as group 2 channel 2 and channel 7.
