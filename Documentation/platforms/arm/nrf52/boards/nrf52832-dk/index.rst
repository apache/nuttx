===========
nRF52832-DK
===========

.. tags:: chip:nrf52, chip:nrf52832

The `NRF52832-DK (PCA10040) <https://www.nordicsemi.com/Products/Development-hardware/nrf52-dk>`_
is a development board for the nRF52832 SoC from Nordic.

Serial Console
==============

The PCA10040 default console is the UART0.

The PCA10040 does not have RS-232 drivers or serial connectors on board.
UART0 is connected to the virtual COM port:

========  =====
Signal    PIN
========  =====
UART0-RX  P0.08
UART0-TX  P0.06
========  =====

LEDs and Buttons
================

LEDs
----
The PCA10040 has 4 user-controllable LEDs:

====  =======
LED   MCU
====  =======
LED1  P0.17
LED2  P0.18
LED3  P0.19
LED4  P0.20
====  =======

A low output illuminates the LED.

CONFIG_ARCH_LEDS
----------------

If CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
control of the application.  The following interfaces are then available
for application control of the LEDs::

  uint32_t board_userled_initialize(void);
  void board_userled(int led, bool ledon);
  void board_userled_all(uint32_t ledset);

Pushbuttons
-----------

=======  =======
BUTTON   MCU
=======  =======
BUTTON1  P0.13
BUTTON2  P0.14
BUTTON3  P0.15
BUTTON4  P0.16
=======  =======

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follow::

  tools/configure.sh nrf52832-dk:<subdir>

Where <subdir> is one of the following:

jumbo
-----

This configuration enables many Apache NuttX features.  This is
mostly to help provide additional code coverage in CI, but also
allows for a users to see a wide range of features that are
supported by the OS.

nsh
----

Basic NuttShell configuration (console enabled in UART0, exposed via J-Link VCOM
connection, at 115200 bps).

ostest_tickless
---------------

This is a NSH configuration that includes ``apps/testing/ostest`` as a builtin and
enable support for the tick-less OS.

sdc
----

Enables Nordic's SoftDevice controller and uses NuttX BLE stack for the host-layer.
The ``btsak`` application is included as a builtin.

sdc_nimble
----------

Enables Nordic's SoftDevice controller and uses nimBLE for the host-layer.
The ``nimble`` test application can be used to enable a simple GATT server.

nxscope_rtt
------------

Configuration demonstrating NxScope stream over Segger RTT interface.

A total of 3 RTT channels are configured:

=========== =================  ============== ================
RTT channel Function           Up buffer size Down buffer size
=========== =================  ============== ================
0           serial console     1024           16
1           SystemView         2048           0
2           NxScope interface  2048           128
=========== =================  ============== ================

See :doc:`/applications/examples/nxscope/index` and
:doc:`/applications/logging/nxscope/index` for more details.

Device detection with ``nxscli`` client::

    $ nxscli rtt nRF52832_XXAA 2 2048 pdevinfo          
    Auto-search for RTT block address                                                         
    JLink interface is SWD
    connecting to nRF52832_XXAA ...
    connected, starting RTT...
    RTT started, 3 up bufs, 3 down bufs.
    INFO:nxscli:enable (0, <class 'nxscli.plugins.devinfo.PluginDevinfo'>, {})
    INFO:nxscli:connecting to nxs device...
    INFO:nxscli:connected!
    INFO:nxscli:started <nxscli.plugins.devinfo.PluginDevinfo object at 0x7f4a3a4d2270>

    Device Summary
      Channels:         32
      Divider support:  yes
      Ack support:      no
      Flags:            0x01
      RX padding:       0

    Stream
      Connected:        yes
      Started:          no
      Overflow count:   0
      Bitrate:          0.0 B/s

    Channel State
      Applied enabled:  none
      Buffered enabled: none

    Channels
    +----+--------+--------+-----+-------+----+-----+
    | ID | Name   | Type   | Dim | Valid | En | Div |
    +====+========+========+=====+=======+====+=====+
    |  0 | chan0  | UINT8  |   1 | yes   | no |   0 |
    |  1 | chan1  | INT8   |   1 | yes   | no |   0 |
    |  2 | chan2  | UINT16 |   1 | yes   | no |   0 |
    |  3 | chan3  | INT16  |   1 | yes   | no |   0 |
    |  4 | chan4  | UINT32 |   1 | yes   | no |   0 |
    |  5 | chan5  | INT32  |   1 | yes   | no |   0 |
    |  6 | chan6  | UINT64 |   1 | yes   | no |   0 |
    |  7 | chan7  | INT64  |   1 | yes   | no |   0 |
    |  8 | chan8  | FLOAT  |   1 | yes   | no |   0 |
    |  9 | chan9  | DOUBLE |   1 | yes   | no |   0 |
    | 10 | chan10 | UB8    |   1 | yes   | no |   0 |
    | 11 | chan11 | B8     |   1 | yes   | no |   0 |
    | 12 | chan12 | UB16   |   1 | yes   | no |   0 |
    | 13 | chan13 | B16    |   1 | yes   | no |   0 |
    | 14 | chan14 | UB32   |   1 | yes   | no |   0 |
    | 15 | chan15 | B32    |   1 | yes   | no |   0 |
    | 16 | chan16 | FLOAT  |   3 | yes   | no |   0 |
    | 17 | chan17 | FLOAT  |   3 | yes   | no |   0 |
    | 18 | chan18 | NONE   |   0 | yes   | no |   0 |
    | 19 | -      | UNDEF  |   0 | no    | no |   0 |
    | 20 | -      | UNDEF  |   0 | no    | no |   0 |
    | 21 | -      | UNDEF  |   0 | no    | no |   0 |
    | 22 | -      | UNDEF  |   0 | no    | no |   0 |
    | 23 | -      | UNDEF  |   0 | no    | no |   0 |
    | 24 | -      | UNDEF  |   0 | no    | no |   0 |
    | 25 | -      | UNDEF  |   0 | no    | no |   0 |
    | 26 | -      | UNDEF  |   0 | no    | no |   0 |
    | 27 | -      | UNDEF  |   0 | no    | no |   0 |
    | 28 | -      | UNDEF  |   0 | no    | no |   0 |
    | 29 | -      | UNDEF  |   0 | no    | no |   0 |
    | 30 | -      | UNDEF  |   0 | no    | no |   0 |
    | 31 | -      | UNDEF  |   0 | no    | no |   0 |
    +----+--------+--------+-----+-------+----+-----+

    closing...
    INFO:nxscli:disconnecting from nxs device...
    INFO:nxscli:disconnected!

nxscope_uart
------------

Configuration demonstrating NxScope stream over USART interface.
It uses UART0 connected to J-Link VCOM for NxScope, and 2 RTT
channels for debug purposes are enabled:

=========== =================  ============== ================
RTT channel Function           Up buffer size Down buffer size
=========== =================  ============== ================
0           serial console     1024           16
1           SystemView         2048           0
=========== =================  ============== ================

See :doc:`/applications/examples/nxscope/index` and
:doc:`/applications/logging/nxscope/index` for more details.
