=================
stm32f401rc-rs485
=================

This page discusses issues unique to NuttX configurations for the
NuttX STM32F4-RS485 development board.

.. figure:: stm32f401rc_rs485.jpg
   :align: center

Board information
=================

This board was release on NuttX International Workshop 2023 and developed based on
STM32F401RCT6 microcontroller.

STM32F401RCT6 microcontroller features:
 - Arm 32-bit Cortex®-M4 CPU with FPU
 - 256 Kbytes of Flash memory
 - 64 Kbytes of SRAM
 - Serial wire debug (SWD) & JTAG interfaces
 - Up to 81 I/O ports with interrupt capability
 - Up to 11 communication interfaces
 - Up to 3 I2C interfaces
 - Up to 3 USARTs
 - Up to 4 SPIs
 - SDIO interface
 - USB 2.0 full-speed device/host/OTG controller with on-chip PHY


The board features:

- Digital I2C Temperature Sensor (TMP75)
- 2K bits (256x8) I2C EEPROM
- On-board RS485 Transceiver
- Two Analog Input Stages with Amplifier Buffer
- Two Analog Output Stages with Amplifier Buffer
- MicroSD Connector supporting 1 or 4-bit bus
- Four User LEDs
- Four User Buttons
- USB for DFU (Device Firmware Update) and USB device functionality, as well as powering the board
- Onboard voltage regulator from 5V to 3.3V
- SWD Pins for use as STLink (Pin header) and TC2030-IDC 6-Pin Tag-Connect Plug-of-Nails™ Connector
- Crystal for HS 8MHz
- Crystal for RTC 32.768KHz

Board documentation:
https://github.com/lucaszampar/NuttX_STM32F4_RS485_DevBoard

As F4 series have a USB DFuSe-capable BootROM [AN2606], the board can be flashed
via `dfu-util` over USB, or via `stm32flash` over UART without any debuggers.

LEDs
====

The STM32F4-RS485 has 4 software controllable LEDs.

=====  =====
LED    PINS
=====  =====
LED_1  PC0
LED_2  PC1
LED_4  PC2
LED_5  PC3
=====  =====

User Buttons
============

The STM32F4-RS485 has 4 user switches.

======= ===== ======
SWITCH  PINS  LABEL
======= ===== ======
SWIO_1  PB13  SW3
SWIO_2  PB14  SW4
SWIO_3  PB15  SW5
SWIO_4  PC6   SW6[1]
======= ===== ======

[1] The switch SWIO_4 (SW6) is disabled due a conflict with PIN
PC6 when using USART6. 

UARTs
=====

The STM32F4-RS485 has 1 USART available for user.

USART6
------

========== =======
UART/USART PINS
========== =======
TX         PC6 [1]
RX         PC7
CK         PA8
========== =======

[1] Warning you make need to reverse RX/TX on some RS-232 converters

SDCard support
==============

The STM32F4-RS485 has 1 SDCard slot connected as below:

========== =====
SDIO       PINS
========== =====
SDIO_D0    PC8
SDIO_D1    PC9
SDIO_D2    PC10
SDIO_D3    PC11
SDIO_DK    PC12
========== =====

EEPROM
======

The STM32F4-RS485 development board has serial EEPROM HX24LC02B, with 2k bits (256x8) and internally
organized with 32 pages of 8 bytes each. It is connected through I2C as below:

====== =====
I2C    PINS
====== =====
SDA    PB7
SCL    PB8
====== =====

Users can enable EERPOM support on STM32F4-RS485 by following below configuration:

- Configure basic nsh::

       ./tools/configure.sh -l stm32f401rc-rs485:nsh

- Enable the following configs::

       CONFIG_DEV_ZERO=y
       CONFIG_EEPROM=y
       CONFIG_FS_PROCFS=y
       CONFIG_I2C=y
       CONFIG_I2C_EE_24XX=y
       CONFIG_STM32_I2C1=y

- Build and flash the STM32F4-RS485.
- Use dd command to write and read data from EEPROM as below::

       nsh> dd if=/dev/zero of=/dev/eeprom
       nsh: dd: write failed: 1
       nsh> dd if=/dev/console of=/dev/eeprom bs=1 count=4
       (type "Hello")
       nsh> dd if=/dev/eeprom of=/dev/console bs=4 count=1
       Hellonsh>

Temperature Sensor
==================

The STM32F4-RS485 development board has a temperature sensor TMP75 (TMP75AIDR) connected through I2C as below:

====== =====
I2C    PINS
====== =====
SDA    PB7
SCL    PB8
====== =====

RS485 Transceiver
=================

The STM32F4-RS485 development board has a half-duplex RS-485 transceiver, the BL3085B it is connected
through USART2 as below:

==========   =====
USART2       PINS
==========   =====
USART2_RX    RO
USART2_RTS   DE, /RE
USART2_RX    DI
==========   =====

A/D Converter
=============

The STM32F4-RS485 development board has two Analog to Digital converters with Amplifier Buffer (1COS724SR)
and connected as below:

======= =====
PWM     PINS
======= =====
PWM_1   PB6
PWM_2   PA6
======= =====

D/C Converter
=============

The STM32F4-RS485 development board has two Digital to Analog converters with Amplifier Buffer (1COS724SR)
and connected as below:

======= =====
ADC     PINS
======= =====
ADC_1   PA0
ADC_2   PA4
======= =====

Configurations
==============

Each stm32f401rc-rs485 configuration is maintained in a sub-directory and
can be selected as follow::

    tools/configure.sh stm32f401rc-rs485:<subdir>

Where <subdir> is one of the following:


Configuration Directories
-------------------------

nsh
---

Configures the NuttShell (nsh) located at apps/examples/nsh. This
configuration enables a serial console on USART6.

sdcard
------

Configures the NuttShell (nsh) and enables SD card support.
The stm32f401rc-rs485 has an onboard microSD slot that should
be automatically registered as the block device /dev/mmcsd0 when
an SD card is present.  The SD card can then be mounted by the
NSH commands::

       nsh> mount -t procfs /proc
       nsh> mount -t vfat /dev/mmcsd0 /mnt

modbus_slave
------------

Configures the NuttShell (nsh) and enables modbus in slave mode. This
configuration enables a serial console on USART6. The RS-485 is connected
to USART2. Follow below precedure to use modbus test aplication, you will
need a USB to RS-485 converter to connect the board to a PC via RS-485.

NuttShell configuration:

Run modbus application at NSH::

       nsh> modbus -help
       USAGE: modbus [-d|e|s|q|h]

       Where:
         -d : Disable protocol stack
         -e : Enable the protocol stack
         -s : Show current status
         -q : Quit application
         -h : Show this information

       nsh> modbus -e

PC Configuration:

Download and install mbpoll aplication::

       sudo apt install mbpoll

Check which TTY USB port is being used by you USB to RS-485 converter::

       sudo dmesg
       [99846.668209] usb 1-1.3: Product: USB Serial
       [99846.676313] ch341 1-1.3:1.0: ch341-uart converter detected
       [99846.677454] usb 1-1.3: ch341-uart converter now attached to ttyUSB1

Run the mbpoll as below::

       mbpoll -a 10 -b 38400 -t 3 -r 1000 -c 4 /dev/ttyUSB1 -R


At PC terminal you will see the mbpoll application receiving the random values
generated by STM32F401RC-RS485 and transmitted over RS-485::

       mbpoll 1.0-0 - FieldTalk(tm) Modbus(R) Master Simulator
       Copyright © 2015-2019 Pascal JEAN, https://github.com/epsilonrt/mbpoll
       This program comes with ABSOLUTELY NO WARRANTY.
       This is free software, and you are welcome to redistribute it
       under certain conditions; type 'mbpoll -w' for details.

       Protocol configuration: Modbus RTU
       Slave configuration...: address = [10]
                               start reference = 1000, count = 4
       Communication.........: /dev/ttyUSB1,      38400-8E1
                               t/o 1.00 s, poll rate 1000 ms
       Data type.............: 16-bit register, input register table
       -- Polling slave 10... Ctrl-C to stop)
       [1000]: 	58080 (-7456)
       [1001]: 	0
       [1002]: 	0
       [1003]: 	0
       -- Polling slave 10... Ctrl-C to stop)
       [1000]: 	6100
       [1001]: 	0
       [1002]: 	0
       [1003]: 	0
       -- Polling slave 10... Ctrl-C to stop)
       [1000]: 	51010 (-14526)
       [1001]: 	0
       [1002]: 	0
       [1003]: 	0
       -- Polling slave 10... Ctrl-C to stop)
       [1000]: 	12528
       [1001]: 	0
       [1002]: 	0
       [1003]: 	0

modbus_master
-------------

Configures the NuttShell (nsh) and enables modbus in msater mode. This
configuration enables a serial console on USART6. The RS-485 is connected
to USART2. Follow below precedure to use modbusmaster test aplication, you will
need a USB to RS-485 converter to connect the board to a PC via RS-485.

PC Configuration:

Download and install diagslave aplication from https://www.modbusdriver.com/diagslave.html.

Check which TTY USB port is being used by you USB to RS-485 converter::

       sudo dmesg
       [99846.668209] usb 1-1.3: Product: USB Serial
       [99846.676313] ch341 1-1.3:1.0: ch341-uart converter detected
       [99846.677454] usb 1-1.3: ch341-uart converter now attached to ttyUSB1

Run the diagslave as below::

       sudo diagslave -a 10 -b 38400 /dev/ttyUSB1

At PC terminal you will see the diagslave application listening to address 10,
notice that this address is configurable via MODBUSMASTER_SLAVEADDR::

       diagslave 3.4 - FieldTalk(tm) Modbus(R) Diagnostic Slave Simulator
       Copyright (c) 2002-2021 proconX Pty Ltd
       Visit https://www.modbusdriver.com for Modbus libraries and tools.

       Protocol configuration: Modbus RTU, frame tolerance = 0ms
       Slave configuration: address = 10, master activity t/o = 3.00s
       Serial port configuration: /dev/ttyUSB1, 38400, 8, 1, even

       Server started up successfully.
       Listening to network (Ctrl-C to stop)
       Slave  10: readHoldingRegisters from 2, 1 references
       .......

NuttShell configuration:

Run modbusmaster application at NSH::

       NuttShell (NSH) NuttX-12.4.0
       nsh> modbusmaster
       Initializing modbus master...
       Creating poll thread.
       Sending 100 requests to slave 10
       mbmaster_main: Exiting poll thread.
       Modbus master statistics:
       Requests count:  100
       Responses count: 100
       Errors count:    0
       Deinitializing modbus master...

The application modbusmaster will send 100 requests, you can check on diagslave::

       Server started up successfully.
       Listening to network (Ctrl-C to stop)
       Slave  10: readHoldingRegisters from 2, 1 references
       Slave  10: readHoldingRegisters from 2, 1 references
       Slave  10: readHoldingRegisters from 2, 1 references
       Slave  10: readHoldingRegisters from 2, 1 references
       Slave  10: readHoldingRegisters from 2, 1 references
       Slave  10: readHoldingRegisters from 2, 1 references
       Slave  10: readHoldingRegisters from 2, 1 references
