=============
at32f437-mini
=============

.. tags:: arch:arm, chip:at32, vendor:artery

The **AT32F437-MINI** is an evaluation and development board manufactured
by Artery Technology, designed around the high-performance **AT32F437VMT7**
microcontroller.

Features
========

Microcontroller
---------------

* **Core**: ARM Cortex-M4 with FPU and DSP instructions
* **Clock**: Up to 288 MHz operating frequency (8 MHz HSE crystal on-board)
* **Internal Flash**: 4 MB with zero-wait execution
* **Internal SRAM**: 384 KB (configurable up to 512 KB)
* **Package**: LQFP100

Board Peripherals
-----------------

* **User LEDs**: 2 user-controllable LEDs (PE2, PE3)
* **Serial Interfaces**:
  * USART1 on PA9/PA10 (used as default serial console)
  * USART2 with RS-485 transceiver support (PD4, PD5, PD6)
* **Networking**: 10/100 Mbps Ethernet RMII interface with onboard LAN8720A PHY
* **CAN**: CAN1 interface with transceiver support (PD0, PD1)
* **Storage**:
  * MicroSD card slot connected via 4-bit SDIO
  * Onboard SPI/QSPI NOR Flash (W25Qxx)
* **USB**: Full-Speed USB 2.0 OTG (OTGFS1) via micro-USB connector
* **Analog & Timers**:
  * ADC1 channel 8 (PB0)
  * Advanced timer TIM20 with PWM output on PE2
  * On-chip RTC (Real-Time Clock)

Pin Mapping and Peripherals
===========================

User LEDs
---------

The AT32F437-MINI provides two user LEDs driven directly by GPIO port E:

==== ========== ==============================
LED  Pin        Description
==== ========== ==============================
LED1 ``PE2``    User LED 1 (Active High)
LED2 ``PE3``    User LED 2 (Active High)
==== ========== ==============================

Serial Ports (USART / UART)
---------------------------

USART1 is configured as the default system console:

====== ========== ========== ==========================================
USART  Pin (RX)   Pin (TX)   Notes
====== ========== ========== ==========================================
USART1 ``PA10``   ``PA9``    Default serial console (115200 8N1)
USART2 ``PD6``    ``PD5``    RS-485 transceiver (Direction control: ``PD4``)
====== ========== ========== ==========================================

Controller Area Network (CAN)
-----------------------------

CAN1 is routed to Port D:

==== ========== ========== ==============================================
CAN  Pin (RX)   Pin (TX)   Notes
==== ========== ========== ==============================================
CAN1 ``PD0``    ``PD1``    Compatible with character and socket drivers
==== ========== ========== ==============================================

Ethernet (10/100 Mbps RMII)
---------------------------

The board integrates a Microchip/SMSC LAN8720A PHY operating in RMII mode:

=============== ========== ==========================================
Signal          Pin        Description
=============== ========== ==========================================
RMII_REF_CLK    ``PA1``    Reference Clock (50 MHz)
ETH_MDIO        ``PA2``    Management Data I/O
RMII_CRS_DV     ``PA7``    Carrier Sense / Receive Data Valid
ETH_MDC         ``PC1``    Management Data Clock
RMII_RXD0       ``PC4``    Receive Data bit 0
RMII_RXD1       ``PC5``    Receive Data bit 1
RMII_TX_EN      ``PB11``   Transmit Enable
RMII_TXD0       ``PB12``   Transmit Data bit 0
RMII_TXD1       ``PB13``   Transmit Data bit 1
ETH_RST         ``PA3``    Active-low PHY Hardware Reset
=============== ========== ==========================================

MicroSD Card (SDIO)
-------------------

The onboard MicroSD card socket utilizes the 4-bit SDIO1 peripheral:

======== ========== ==========================================
Signal   Pin        Description
======== ========== ==========================================
SDIO_CMD ``PD2``    Command / Response line
SDIO_CLK ``PC12``   Clock line
SDIO_D0  ``PC8``    Data bit 0
SDIO_D1  ``PC9``    Data bit 1
SDIO_D2  ``PC10``   Data bit 2
SDIO_D3  ``PC11``   Data bit 3
======== ========== ==========================================

USB OTG Full-Speed (OTGFS1)
---------------------------

The USB OTG Full-Speed peripheral is routed to the onboard micro-USB port:

====== ========== ==========================================
Signal Pin        Description
====== ========== ==========================================
OTG_D- ``PA11``   USB Differential Data Minus
OTG_D+ ``PA12``   USB Differential Data Plus
====== ========== ==========================================

External SPI Flash (W25Qxx)
---------------------------

An external SPI NOR Flash chip is connected to SPI1:

====== ========== ==========================================
Signal Pin        Description
====== ========== ==========================================
CLK    ``PB3``    SPI Serial Clock
MISO   ``PB4``    Master In / Slave Out
MOSI   ``PB5``    Master Out / Slave In
CS     ``PD7``    Active-low Chip Select
====== ========== ==========================================

Analog Inputs & PWM
-------------------

* **ADC**: ADC1 Channel 8 is available on pin ``PB0``.
* **PWM**: Advanced Timer 20 (TIM20) Channel 1 is mapped to pin ``PE2``.
* **RTC**: Driven by the on-chip Real-Time Clock peripheral.

Configurations
==============

All configurations are located in ``boards/arm/at32/at32f437-mini/configs/``:

nsh
---

Basic NuttShell (NSH) interactive command line environment running over the
primary serial console (USART1 at 115200 baud, 8N1).

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:nsh
   $ make -j

After flashing, connect a serial terminal (115200 8N1) to PA9/PA10:

.. code-block:: text

   NuttShell (NSH) NuttX-12.2.1
   nsh> ps
     PID GROUP PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK           STACK COMMAND
       0     0   0 FIFO     Kthread N-- Ready              0000000000000000 001016 Idle_Task
       2     2 100 RR       Task    --- Running            0000000000000000 002016 nsh_main
   nsh>

adc
---

Enables the ADC character driver and sample application (``examples/adc``)
measuring voltage on ADC1 Channel 8 (pin ``PB0``).

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:adc
   $ make -j

can_char
--------

Configures CAN1 using the traditional NuttX CAN character device driver
(``/dev/can0``). Used in conjunction with ``examples/can``.

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:can_char
   $ make -j

can_socket
----------

Configures CAN1 as a native SocketCAN network interface (``can0``). Allows
standard POSIX socket programming (``AF_CAN``) and network utilities.

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:can_socket
   $ make -j

eth
---

Enables the onboard LAN8720A RMII Ethernet transceiver with the NuttX TCP/IP
network stack, DHCP client, and network utilities (``ping``, ``ifconfig``,
``netinit``).

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:eth
   $ make -j

msc
---

Configures the USB OTG port as a USB Mass Storage Class (MSC) device,
exposing a block driver (RAM disk or SD card) to a host PC.

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:msc
   $ make -j

pwm
---

Enables the PWM character driver on TIM20 Channel 1 (pin ``PE2``). Can be
tested with ``examples/pwm``.

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:pwm
   $ make -j

rtc
---

Enables the hardware Real-Time Clock (RTC) driver. Provides persistent timekeeping
and alarm functions accessible via standard POSIX time APIs.

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:rtc
   $ make -j

sdcard
------

Enables 4-bit SDIO driver support with FAT filesystem automounting for the
onboard MicroSD card slot. The card is accessible at ``/mnt/sdcard``.

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:sdcard
   $ make -j

systemview
----------

Enables SEGGER SystemView instrumentation for real-time task scheduling and
interrupt tracing.

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:systemview
   $ make -j

usbnsh
------

Configures the NuttShell console to communicate over the USB OTG port
utilizing the USB CDC/ACM serial device profile (``/dev/ttyACM0`` on Linux).

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:usbnsh
   $ make -j

usbserial
---------

Configures the USB OTG port as a generic USB CDC/ACM serial communications port.

.. code-block:: console

   $ ./tools/configure.sh at32f437-mini:usbserial
   $ make -j

Building and Flashing
=====================

Building
--------

1. Configure the board for the desired target (for example, ``nsh``):

   .. code-block:: console

      $ ./tools/configure.sh at32f437-mini:nsh

2. Compile the NuttX ELF and binary images:

   .. code-block:: console

      $ make -j$(nproc)

Flashing
--------

The board can be flashed using OpenOCD, J-Link, or pyOCD through the onboard
SWD debug header:

Using OpenOCD:

.. note::

   The target configuration script ``target/at32f437xx.cfg`` is provided as
   part of Artery's official OpenOCD release (available in the Artery AT32 BSP
   and AT-Link utility packages). Upstream OpenOCD requires adding Artery's
   target script to the OpenOCD scripts path or using the PyOCD / J-Link alternatives below.

.. code-block:: console

   $ openocd -f interface/cmsis-dap.cfg -f target/at32f437xx.cfg -c "program nuttx.bin 0x08000000 verify reset exit"

Using J-Link:

.. code-block:: console

   $ JLinkExe -device AT32F437VMT7 -if SWD -speed 4000 -autoconnect 1
   J-Link> loadfile nuttx.hex
   J-Link> r
   J-Link> g
   J-Link> q

Using pyOCD:

.. code-block:: console

   $ pyocd pack install at32f437vmt7
   $ pyocd flash -t at32f437vmt7 nuttx.bin
