====================
Realtek RTL8730E-EVB
====================

.. tags:: chip:rtl8730e, arch:arm, vendor:realtek

.. figure:: rtl8730e_evb.png
   :scale: 50 %
   :align: center
   :alt: Realtek RTL8730E EVB development board

   The RTL8730E EVB development board.

The RTL8730E_EVB is a Realtek RTL8730E evaluation board built around the
RTL8730EH (DDR2 64 MB, 16 MB NOR flash).  NuttX runs on the dual-core Arm
Cortex-A32 application processor at up to 1.2 GHz as BL33 in the ATF boot
chain.  See the :doc:`RTL8730E chip documentation <../../index>` for the full
SoC specifications and vendor-SDK dependency.

Features
========

* RTL8730EH: dual-core Arm Cortex-A32 up to 1.2 GHz, 64 MB DDR2, 16 MB NOR
  flash
* Symmetric Multi-Processing (SMP) on both Cortex-A32 cores
* Wi-Fi 6 (802.11ax) dual-band (2.4 / 5 GHz) station and SoftAP
* Bluetooth 5 dual-mode (BR/EDR + LE)
* SPI NOR flash with littlefs at ``/data``
* LOG-UART console at 1500000 8N1

Supported in this NuttX port:

* NSH shell over the LOG-UART console
* SMP on both Cortex-A32 cores; tasks are dispatched across CPU0 and CPU1
  by the NuttX SMP scheduler
* GPIO pins exposed as ``/dev/gpioN`` character devices (input, output and
  interrupt), driven through the SDK fwlib ROM layer
* General-purpose UARTs exposed as ``/dev/ttySN`` serial devices, driven
  through the SDK fwlib ROM layer (the LOG-UART owns the console and
  ``/dev/ttyS0``)
* I2C master buses exposed as ``/dev/i2cN`` character devices, driven in
  polling mode through the SDK fwlib register layer
* SPI master buses exposed as ``/dev/spiN`` character devices, driven in
  polling mode through the SDK fwlib register layer
* PWM output exposed as a ``/dev/pwm0`` character device, driven directly on
  the SDK fwlib timer layer
* General-purpose timers exposed as ``/dev/timerN`` character devices, with
  microsecond timeouts and a periodic expiration callback
* On-chip watchdog exposed as a ``/dev/watchdog0`` character device
* littlefs persistent storage mounted at ``/data`` (a dedicated SPI NOR flash
  partition), backing the Wi-Fi key-value store
* Wi-Fi station and SoftAP through the ``wapi`` tool
* DHCP client (STA) and DHCP server (SoftAP)
* ``/etc`` read-only ROMFS populated from the board's ``src/etc/`` tree

Configurations
==============

Build and flash any of these per the :doc:`RTL8730E build instructions
<../../index>`.

.. code:: console

   $ ./tools/configure.sh rtl8730e_evb:<config-name>

nsh
---

Networking-enabled NSH with SMP, littlefs at ``/data``, and the ``wapi``
Wi-Fi tool.  The console is the LOG-UART at 1500000 8N1.

gpio
----

Minimal NSH with the GPIO driver and the ``gpio`` example enabled (no Wi-Fi).
The board registers three pins from its pin table (see
``boards/arm/rtl8730e/rtl8730e_evb/src/rtl8730e_gpio.c``): an output at
``/dev/gpio0``, an input at ``/dev/gpio1`` and an interrupt pin at
``/dev/gpio2``.  Edit that table to match a board's wiring.  Exercise them
with the example::

    nsh> gpio -o 1 /dev/gpio0     # drive the output high
    nsh> gpio /dev/gpio1          # read the input
    nsh> gpio -w 1 /dev/gpio2     # wait for a falling-edge interrupt

Pins are encoded with the ``AMEBA_PA()`` / ``AMEBA_PB()`` helpers from
``arch/arm/src/common/ameba/ameba_gpio.h`` (port A/B, pin 0-31), matching the
Ameba SDK ``PinName`` layout.

uart
----

Minimal NSH with the general-purpose UART driver and the ``serialrx`` /
``serialblaster`` examples enabled (no Wi-Fi). The LOG-UART owns the console
and ``/dev/ttyS0``, so the board registers UART0-2 from its table (see
``boards/arm/rtl8730e/rtl8730e_evb/src/rtl8730e_uart.c``) as ``/dev/ttyS1-3``
at 115200 8N1. Edit that table -- controller, TX/RX pads and baud -- to match a
board's wiring. The TX/RX pads use the same ``AMEBA_PA()`` / ``AMEBA_PB()``
encoding as the GPIO table; the driver muxes them to the UART function and
pulls RX high through the SDK ROM. Exercise a port with the examples (loop TX
back to RX, or wire it to a host serial adapter)::

    nsh> serialblaster /dev/ttyS1 26   # stream a test pattern out /dev/ttyS1
    nsh> serialrx      /dev/ttyS1 26   # receive and dump bytes from /dev/ttyS1

The line format can be changed at runtime through ``tcsetattr()`` (the config
enables ``CONFIG_SERIAL_TERMIOS``). UART3 is reserved for Bluetooth and is not
exposed by the driver.

i2c
---

Minimal NSH with the I2C master driver and the ``i2ctool`` (``system/i2c``)
enabled (no Wi-Fi). The board registers its I2C controllers from a table (see
``boards/arm/rtl8730e/rtl8730e_evb/src/rtl8730e_i2c.c``): I2C0 at
``/dev/i2c0`` on PA10/PA9, I2C1 at ``/dev/i2c1`` on PA4/PA3 and I2C2 at
``/dev/i2c2`` on PB11/PB10 (SCL/SDA). Edit that table -- controller and
SCL/SDA pads -- to match a board's wiring; the pads use the same
``AMEBA_PA()`` / ``AMEBA_PB()`` encoding as the GPIO table and are muxed to
the I2C function through the SDK ROM. Unlike the GPIO and UART crossbars, a
pad reaches exactly one I2C controller, and pads the audio codec drives
cannot be used. The I2C bus is open-drain, so fit external pull-ups on
SCL/SDA. Probe a bus with the tool::

    nsh> i2c dev -b 0 0x03 0x77     # scan /dev/i2c0 for devices

spi
---

Minimal NSH with the SPI master driver and the ``spi`` tool (``system/spi``)
enabled (no Wi-Fi). The board registers two buses from its table (see
``boards/arm/rtl8730e/rtl8730e_evb/src/rtl8730e_spi.c``): SPI0 at
``/dev/spi0`` with CLK/MOSI/MISO/CS on PB18/PB19/PB20/PB21 and SPI1 at
``/dev/spi1`` on PA11/PA9/PA10/PA12. The pads use the same ``AMEBA_PA()`` /
``AMEBA_PB()`` encoding as the GPIO table and are muxed to the SPI function
through the SDK ROM, while the chip-select is driven as a plain GPIO. As with
I2C -- and unlike the GPIO and UART crossbars -- a pad reaches exactly one
controller, so edit that table within each controller's own set: PB18-PB21
for SPI0, PB25-PB28 or PA9-PA12 for SPI1. Pads the audio codec drives cannot
be used. Exercise a bus with the tool::

    nsh> spi exch -b 0 -x 4 deadbeef     # full-duplex transfer on /dev/spi0

pwm
---

Minimal NSH with the PWM driver and the ``pwm`` example (``examples/pwm``)
enabled (no Wi-Fi). The board registers one timer at ``/dev/pwm0`` (see
``boards/arm/rtl8730e/rtl8730e_evb/src/rtl8730e_pwm.c``): TIM8 drives six
compare channels off one shared time base, so every channel shares one
frequency and each carries its own duty cycle. The table routes the channels
to PA7/PA9/PA10/PA4/PA5/PA6. Despite the chip having a single
``PINMUX_FUNCTION_PWM`` code, each pad is tied to one channel -- the pin-mux
table names a pad's PWM function ``HS_PWM<n>``, which is channel n + 1 -- so
an entry may only be cleared to ``AMEBA_PWM_PIN_NC`` or swapped for another
pad carrying the same ``HS_PWM<n>``; PB14-PB16 are the alternative for
channels 1-3. Avoid PA14/PA13, which are SWD_CLK/SWD_DATA and stay muxed to
the debug port unless the board calls ``Pinmux_Swdoff()``. Set
``CONFIG_PWM_NCHANNELS`` to the number of channels used. Exercise it with the
example::

    nsh> pwm -d 25 -f 1000     # 1 kHz, 25% duty on /dev/pwm0

timer
-----

Minimal NSH with the timer driver and the ``timer`` example
(``examples/timer``) enabled (no Wi-Fi). The board registers two timers (see
``boards/arm/rtl8730e/rtl8730e_evb/src/rtl8730e_timer.c``): ``/dev/timer0``
is TIM2 and ``/dev/timer1`` is TIM3, both 32-bit LP basic timers clocked at
32.768 kHz, so a tick is ~30.5 us and a single timeout reaches ~36 hours.
Neither has any board wiring. TIM0 and TIM1 are not exposed: the ROM claims
TIM0 as the system timer behind ``DelayUs()``/``SYSTIMER_*``, and the vendor
HAL claims TIM1 as its application and UART-timeout timer. Exercise a timer
with the example::

    nsh> timer                 # 20 samples 0.1 s apart, 1 s timeout, on /dev/timer0

wdg
---

Minimal NSH with the watchdog driver and the ``wdog`` example (no Wi-Fi).
The board registers the on-chip watchdog at ``/dev/watchdog0`` (see
``boards/arm/rtl8730e/rtl8730e_evb/src/rtl8730e_wdg.c``): WDG4, the CA32
non-secure system watchdog. It has no board wiring (it is an internal
timer) and, once enabled, cannot be stopped by software -- the stop()
operation instead arms the early-interrupt and refreshes the counter from
its handler to inhibit the reset. Exercise it with the example, which
opens the device, sets a timeout, and pings it::

    nsh> wdog                  # run the watchdog example on /dev/watchdog0

Wi-Fi
=====

Station (connect to an AP)::

    nsh> wapi mode  wlan0 2
    nsh> wapi psk   wlan0 <password> 3
    nsh> wapi essid wlan0 <ssid> 1
    nsh> renew wlan0

SoftAP (become an access point, with a DHCP server for clients)::

    nsh> wapi mode   wlan0 3
    nsh> wapi psk    wlan0 <password> 3
    nsh> wapi essid  wlan0 <ssid> 1
    nsh> dhcpd_start wlan0

Stop the SoftAP with ``wapi essid wlan0 <ssid> 0``.

License Exceptions
==================

This board depends on Realtek vendor code that is not part of NuttX and is
subject to its own license:

* The prebuilt Wi-Fi / Bluetooth firmware image and the Realtek ``ameba-rtos``
  SDK libraries/headers linked into the image.  See the SDK's own license; the
  SDK is auto-fetched and is not redistributed in the NuttX tree.
* The prebuilt ATF BL2 and BL32 binaries under ``prebuilt/`` are provided by
  Realtek and are subject to the Realtek binary license included in the SDK.
