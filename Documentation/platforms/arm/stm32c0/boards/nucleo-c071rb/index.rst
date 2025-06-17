================
ST Nucleo C071RB
================

.. tags:: chip:stm32, chip:stm32c0, chip:stm32c071

The Nucleo C071RB is a member of the Nucleo-64 board family.

USB not supported yet.

Buttons
=======

B1 USER: the user button is connected to the I/O PC13 of the STM32
microcontroller.

Serial Console
==============

At default USART2 connected to "Virtual COM Port" is used as serial console.

Configurations
==============

nsh
---

Configures the NuttShell (nsh) located at apps/examples/nsh.  The
Configuration enables the serial interfaces on USART2.  Support for
builtin applications is disabled.

jumbo
-----

This configuration enables many Apache NuttX features.  This is
mostly to help provide additional code coverage in CI, but also
allows for a users to see a wide range of features that are
supported by the OS.

Enabled features:

- NSH

- ADC with DMA enabled using A0 and A1 pins

- button with software debouncing enabled (no RC filter on the board)

analogscope
-----------

Configuration demonstrating ADC data stream using NxScope over UART
connected to VCOM.

See :doc:`/applications/system/adcscope/index` for more details.

Device detection with ``nxscli`` client::

    $ nxscli serial /dev/ttyACM0 pdevinfo               
    INFO:nxscli:enable (0, <class 'nxscli.plugins.devinfo.PluginDevinfo'>, {}) 
    INFO:nxscli:connecting to nxs device...
    INFO:nxscli:connected!
    INFO:nxscli:started <nxscli.plugins.devinfo.PluginDevinfo object at 0x7f090d810830>

    Device Summary
      Channels:         6
      Divider support:  no
      Ack support:      no
      Flags:            0x00
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
    +----+-------+-------+-----+-------+----+-----+
    | ID | Name  | Type  | Dim | Valid | En | Div |
    +====+=======+=======+=====+=======+====+=====+
    |  0 | chan0 | INT16 |   1 | yes   | no |   0 |
    |  1 | chan1 | INT16 |   1 | yes   | no |   0 |
    |  2 | chan2 | INT16 |   1 | yes   | no |   0 |
    |  3 | chan3 | INT16 |   1 | yes   | no |   0 |
    |  4 | chan4 | INT16 |   1 | yes   | no |   0 |
    |  5 | chan5 | INT16 |   1 | yes   | no |   0 |
    +----+-------+-------+-----+-------+----+-----+

    closing...
    INFO:nxscli:disconnecting from nxs device...
    INFO:nxscli:disconnected!
