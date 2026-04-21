================
ST Nucleo F412ZG
================

.. tags:: chip:stm32, chip:stm32f4, chip:stm32f412

This page discusses issues unique to NuttX configurations for the ST
Nucleo F412ZG board from ST Micro.  See

  http://www.st.com/en/evaluation-tools/nucleo-f412zg.html

NucleoF412ZG:

- Microprocessor: 32-bit ARM Cortex M4 at 100MHz STM32F412ZG
- Memory: 1 MB Flash and 256 KB SRAM
- ADC: 1x12-bit, 2.4 MSPS A/D converter: up to 16 channels
- DMA: 2x8-stream DMA controllers with FIFOs and burst support
- Timers: Up to 17 timers: up to 12 16-bit, 2 32-bit timers, two
  watchdog timers, and a SysTick timer
- GPIO: Up to 114 I/O ports with interrupt capability
- I2C: Up to 4 I2C interfaces
- USARTs: Up to 4 USARTs
- SPIs: Up to 5 SPIs (5 I2S)
- SDIO interface (SD/MMC/eMMC)
- Advanced connectivity: USB 2.0 full-speed device/host/OTG controller with PHY
- 2x CAN (2.0B Active)
- True random number generator
- CRC calculation unit
- 96-bit unique ID
- RTC

See:
https://www.st.com/content/ccc/resource/technical/document/user_manual/group0/26/49/90/2e/33/0d/4a/da/DM00244518/files/DM00244518.pdf/jcr:content/translations/en.DM00244518.pdf

- Peripherals: 3 led, 2 push button
- Debug: Serial wire debug and JTAG interfaces
- Expansion I/F Ardino and Morpho Headers

Hardware
========

Buttons
-------

B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
microcontroller.

LEDs
----

The Nucleo F410RB provide a single user LED, LD2.  LD2
is the green LED connected to Arduino signal D13 corresponding to MCU I/O
PA5 (pin 21) or PB13 (pin 34) depending on the STM32target.

- When the I/O is HIGH value, the LED is on.
- When the I/O is LOW, the LED is off.

These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
events as follows when the red LED (PE24) is available::

    SYMBOL                Meaning                   LD2
    -------------------  -----------------------  -----------
    LED_STARTED          NuttX has been started     OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF
    LED_IRQSENABLED      Interrupts enabled         OFF
    LED_STACKCREATED     Idle stack created         ON
    LED_INIRQ            In an interrupt            No change
    LED_SIGNAL           In a signal handler        No change
    LED_ASSERTION        An assertion failed        No change
    LED_PANIC            The system has crashed     Blinking
    LED_IDLE             MCU is is sleep mode       Not used

Thus if LD2, NuttX has successfully booted and is, apparently, running
normally.  If LD2 is flashing at approximately 2Hz, then a fatal error
has been detected and the system has halted.

Serial Consoles
===============

USART1
------

Pins and Connectors::

    RXD: PA11  CN10 pin 14
         PB7   CN7 pin 21
    TXD: PA10  CN9 pin 3, CN10 pin 33
         PB6   CN5 pin 3, CN10 pin 17

    NOTE:  You may need to edit the include/board.h to select different USART1
    pin selections.

TTL to RS-232 converter connection::

    Nucleo CN10 STM32F412ZG
    ----------- ------------
    Pin 21 PA9  USART1_RX   *Warning you make need to reverse RX/TX on
    Pin 33 PA10 USART1_TX    some RS-232 converters
    Pin 20 GND
    Pin 8  U5V

To configure USART1 as the console::

    CONFIG_STM32_USART1=y
    CONFIG_USART1_SERIALDRIVER=y
    CONFIG_USART1_SERIAL_CONSOLE=y
    CONFIG_USART1_RXBUFSIZE=256
    CONFIG_USART1_TXBUFSIZE=256
    CONFIG_USART1_BAUD=115200
    CONFIG_USART1_BITS=8
    CONFIG_USART1_PARITY=0
    CONFIG_USART1_2STOP=0

USART2
------

Pins and Connectors::

    RXD: PA3   CN9 pin 1 (See SB13, 14, 62, 63). CN10 pin 37
         PD6
    TXD: PA2   CN9 pin 2(See SB13, 14, 62, 63). CN10 pin 35
         PD5

USART2 is available on the board headers. It is not the default ``nsh``
console path.

TTL to RS-232 converter connection::

    Nucleo CN9  STM32F412ZG
    ----------- ------------
    Pin 1  PA3  USART2_RX   *Warning you make need to reverse RX/TX on
    Pin 2  PA2  USART2_TX    some RS-232 converters

Solder Bridges.  This configuration requires:

- SB62 and SB63 Closed: PA2 and PA3 on STM32 MCU are connected to D1 and D0
  (pin 7 and pin 8) on Arduino connector CN9 and ST Morpho connector CN10
  as USART signals.  Thus SB13 and SB14 should be OFF.

- SB13 and SB14 Open:  PA2 and PA3 on STM32F103C8T6 (ST-LINK MCU) are
  disconnected to PA3 and PA2 on STM32 MCU.

To configure USART2 as the console::

    CONFIG_STM32_USART2=y
    CONFIG_USART2_SERIALDRIVER=y
    CONFIG_USART2_SERIAL_CONSOLE=y
    CONFIG_USART2_RXBUFSIZE=256
    CONFIG_USART2_TXBUFSIZE=256
    CONFIG_USART2_BAUD=115200
    CONFIG_USART2_BITS=8
    CONFIG_USART2_PARITY=0
    CONFIG_USART2_2STOP=0

USART3
------

Pins and Connectors::

    RXD: PD9
    TXD: PD8

By default the board solder bridges connect USART3 to the on-board
ST-LINK Virtual COM Port. This is the default ``nsh`` console path in
NuttX.

To configure USART3 as the console::

    CONFIG_STM32_USART3=y
    CONFIG_USART3_SERIALDRIVER=y
    CONFIG_USART3_SERIAL_CONSOLE=y
    CONFIG_USART3_RXBUFSIZE=256
    CONFIG_USART3_TXBUFSIZE=256
    CONFIG_USART3_BAUD=115200
    CONFIG_USART3_BITS=8
    CONFIG_USART3_PARITY=0
    CONFIG_USART3_2STOP=0

USART6
------

Pins and Connectors::

    RXD: PC7    CN5 pin2, CN10 pin 19
         PA12   CN10, pin 12
    TXD: PC6    CN10, pin 4
         PA11   CN10, pin 14

To configure USART6 as the console::

    CONFIG_STM32_USART6=y
    CONFIG_USART6_SERIALDRIVER=y
    CONFIG_USART6_SERIAL_CONSOLE=y
    CONFIG_USART6_RXBUFSIZE=256
    CONFIG_USART6_TXBUFSIZE=256
    CONFIG_USART6_BAUD=115200
    CONFIG_USART6_BITS=8
    CONFIG_USART6_PARITY=0
    CONFIG_USART6_2STOP=0

Virtual COM Port
----------------

The NUCLEO-F412ZG ST-LINK virtual COM port is connected to USART3 on
PD8/PD9. As shipped, the default solder bridge configuration enables
this routing, so the default ``nsh`` configuration uses USART3 for the
console.

Configurations
==============

nsh
---

Configures the NuttShell (nsh) located at apps/examples/nsh for the
Nucleo-F412ZG board.  The configuration enables the serial interfaces
on USART3.  Support for builtin applications is enabled, but in the base
configuration no builtin applications are selected (see NOTES below).

NOTES:

1. This configuration uses the mconf-based configuration tool.  To
   change this configuration using that tool, you should:

   a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
      see additional README.txt files in the NuttX tools repository.

   b. Execute 'make menuconfig' in nuttx/ in order to start the
      reconfiguration process.
