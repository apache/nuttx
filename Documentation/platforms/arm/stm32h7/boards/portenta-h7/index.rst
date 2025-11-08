===================
Arduino Portenta H7
===================

This page discusses issues unique to NuttX configurations for the
Arduino Portenta H7 board. This port applies to all versions of
Portenta H7:

* Portenta H7
* Portenta H7 Little
* Portenta H7 Little Connected

Features
========

Arduino Portenta H7 boards come with:

* STM32H747 dual-core processor
* 8 MB SDRAM
* 16 MB NOR Flash
* 10/100 Ethernet Phy
* PMIC MC34PF1550A0EP
* Fuel Gauge Bat MAX1726
* USB HS
* Secure element: NXP SE0502 (Portenta H7) or Microchip ATECC608 (H7 Little and
  H7 Little Connected)
* Wi-Fi/Bluetooth Module (except H7 Lite)
* DisplayPort over USB-C (Portenta H7 only)

Documentation: https://docs.arduino.cc/hardware/portenta-h7/

Status
======

- NSH works on USART1.

Pin Mapping
===========

====== ============ ============== ================== =================
Pin    Pin Arduino  Signal Arduino Signal STM32       Notes
====== ============ ============== ================== =================
J1-33  D14          UART1_TX       USART1_TX (PA10)   Default Console
J1-35  D13          UART1_RX       USART1_RX (PA9)    Default Console
J1-34  N/A          UART0_TX       UART4_TX (PA0)
J1-36  N/A          UART0_RX       UART4_RX (PI9)
J1-43  N/A          I2C1_SDA       I2C1_SDA (PB7)
J1-45  N/A          I2C1_SCL       I2C1_SCL (PB6)
J1-44  D11          I2C0_SDA       I2C3_SDA (PH8)
J1-46  D12          I2C0_SDL       I2C3_SDC (PH7)
I2-40  D10          SPI1_CIPO      SPI2_MISO (PC2)
I2-38  D9           SPI1_CK        SPI2_SCK (PI1)
I2-42  D8           SPI1_COPI      SPI2_MOSI (PC3)
I2-36  D7           SPI1_CS        SPI2_NSS (PI0)
J1-49  N/A          CAN1_TX        FDCAN1_TX (PH13)
J1-51  N/A          CAN1_RX        FDCAN1_RX (PH13)
====== ============ ============== ================== =================

Flashing
========

.. note::
   The on-board PMIC isn't supported yet, so we rely on the Arduino bootloader.
   DO NOT ERASE the default bootloader, or you'll brick the board!

Flashing with dfu-utils
-----------------------

1. Press the reset button twice. The green LED should start flashing rapidly.
   You can check if dfu works with the ``dfu-util -l`` command. It should return::

     Found DFU: [2341:035b] ver=0200, devnum=75, cfg=1, intf=0, path="3-6.3", alt=3, name="@Arduino  boot  v.25   /0x00000000/0*4Kg", serial="003D00473133511137323532"
     Found DFU: [2341:035b] ver=0200, devnum=75, cfg=1, intf=0, path="3-6.3", alt=2, name="@Ext File Flash  0MB   /0x00000000/0*4Kg", serial="003D00473133511137323532"
     Found DFU: [2341:035b] ver=0200, devnum=75, cfg=1, intf=0, path="3-6.3", alt=1, name="@Ext RAW  Flash 16MB   /0x90000000/4096*4Kg", serial="003D00473133511137323532"
     Found DFU: [2341:035b] ver=0200, devnum=75, cfg=1, intf=0, path="3-6.3", alt=0, name="@Internal Flash  2MB   /0x08000000/01*128Ka,15*128Kg", serial="003D00473133511137323532"

2. Flash firmware after bootloader code (offset = 0x08040000)::

     dfu-util --device 0x2341:0x035b -D nuttx.bin -a0 --dfuse-address=0x08040000:leave

Flashing with debugger
----------------------

To connect an external debugger to Portent H7 you need Portenta Breakout and
20-pin MIPI connector. Another option is to solder directly to the test pins
on the board.

OpenOCD works with ``target/stm32h7x_dual_bank.cfg``.

Configurations
==============

Each portenta-h7 configuration is maintained in a sub-directory and
can be selected as follow::

    tools/configure.sh portenta-h7:<subdir>

Where <subdir> is one of the following:

nsh_cm7
-------

Configures the NuttShell (nsh) located at apps/examples/nsh. This
configuration enables a serial console on UART1.
Configuration dedicated for CM7 core.

jumbo_cm7
---------

This configuration enables many Apache NuttX features.  This is
mostly to help provide additional code coverage in CI, but also
allows for user to see a wide range of features that are
supported by the OS.
Configuration dedicated for CM7 core.
