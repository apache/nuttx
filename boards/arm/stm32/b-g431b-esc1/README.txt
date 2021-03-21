README
======

The B-G431B-ESC Discovery kit board is based on the STM32G431CB microcontroller,
the L6387 driver and STL180N6F7 power MOSFETs.

UART/USART PINS
---------------

USART2 is accessible through J3 pads and ST LINK Virtual Console:
  USART2_TX - PB3
  USART2_RX - PB4

Configuration Sub-directories
-------------------------

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on USART2.
