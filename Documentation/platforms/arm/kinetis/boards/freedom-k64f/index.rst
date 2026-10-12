============
freedom-k64f
============

.. tags:: arch:arm, chip:kinetis, vendor:nxp

The Freescale / NXP Freedom K64F (FRDM-K64F) is an ultra-low-cost, compact
development board designed for rapid prototyping with the NXP Kinetis K64
microcontroller family.

The board features the **MK64FN1M0VLL12** MCU, built around an ARM Cortex-M4
core with hardware Floating Point Unit (FPU), running at up to 120 MHz with 1 MB
of Flash memory and 256 KB of SRAM. The board is mechanically and electrically
compatible with the Arduino Uno Revision 3 (R3) form factor.

Key Features
============

- **Microcontroller**: NXP Kinetis MK64FN1M0VLL12 (100-pin LQFP)
  - 32-bit ARM Cortex-M4 with single-precision hardware FPU
  - Core clock frequency up to 120 MHz
  - 1 MB dual-bank embedded Flash memory (with read-while-write capability)
  - 256 KB SRAM
  - Hardware Cryptographic Acceleration Unit (CAU) and Random Number Generator (RNGB)
- **Ethernet**:
  - Onboard 10/100 Mbps RMII Ethernet PHY (Micrel / Microchip KSZ8081RNA)
  - Integrated RJ-45 connector with built-in magnetics
- **Mass Storage**:
  - Push-push MicroSD memory card slot connected to SDHC (4-bit SDIO interface)
  - Dedicated card-detection switch on pin PTE6
- **USB**:
  - Dual-role Full-Speed USB 2.0 Device/Host controller with Micro-B USB connector
  - Crystal-less USB operation capability
- **Sensors**:
  - NXP FXOS8700CQ 6-axis sensor (3-axis digital accelerometer and 3-axis digital magnetometer)
  - Connected via I2C bus 0 (I2C0)
- **User Interface**:
  - Tricolor (RGB) user LED (Red on PTB22, Blue on PTB21, Green on PTE26)
  - Two user pushbuttons: SW2 (PTC6) and SW3 (PTA4 / NMI)
  - Dedicated system reset pushbutton (SW1)
- **Debug & Programming (OpenSDAv2)**:
  - Onboard Freescale OpenSDAv2 circuit running CMSIS-DAP / mbed bootloader firmware
  - Mass Storage Device (MSD) drag-and-drop flash programming
  - Virtual Serial Port (VCOM) over USB for the default NuttShell console
  - CMSIS-DAP debug interface for run-control and GDB debugging
- **Expansion Connectors**:
  - Arduino R3 compatible footprint outer header pins
  - Additional inner row headers for full MCU I/O breakout
  - Footprint for 2.4 GHz RF module (Nordic nRF24L01+) and Bluetooth module

Hardware Configuration
======================

Serial Console
--------------

The Kinetis K64 provides multiple UART modules. NuttX supports two primary
serial console configurations:

USB VCOM Console (Default)
~~~~~~~~~~~~~~~~~~~~~~~~~~

The default serial console uses **UART0**, which is routed directly to the
onboard OpenSDAv2 microcontroller:

- **RX**: PTB16 (UART0_RX)
- **TX**: PTB17 (UART0_TX)
- **Baud rate**: 115200 8N1

Connecting the OpenSDA micro-USB port (J26) to a host PC exposes a virtual serial
port (e.g., ``/dev/ttyACM0`` on Linux or ``COMx`` on Windows), giving immediate
access to the NuttShell prompt without requiring external FTDI adapters.

Arduino Shield Serial Console
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Alternatively, UART3 is routed to the Arduino R3 standard header pins on
connector J1:

- **UART3_RX**: PTC16 (Arduino Pin D0 / J1 Pin 2)
- **UART3_TX**: PTC17 (Arduino Pin D1 / J1 Pin 4)
- **Baud rate**: 115200 8N1

This interface allows attaching standard Arduino serial shields or RS-232 / TTL
expansion boards.

Switching Console to UART3 (Arduino Shield)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To switch the primary serial console from OpenSDA (UART0) to the Arduino
shield pins on UART3, configure the following settings via ``make menuconfig``:

.. code-block:: text

   # Disable UART0 as serial console
   CONFIG_KINETIS_UART0 is not set
   CONFIG_UART0_SERIAL_CONSOLE is not set

   # Enable UART3 as serial console
   CONFIG_KINETIS_UART3=y
   CONFIG_UART3_SERIALDRIVER=y
   CONFIG_UART3_SERIAL_CONSOLE=y
   CONFIG_UART3_RXBUFSIZE=256
   CONFIG_UART3_TXBUFSIZE=256
   CONFIG_UART3_BAUD=115200
   CONFIG_UART3_BITS=8
   CONFIG_UART3_PARITY=0
   CONFIG_UART3_2STOP=0

RGB LED
-------

The FRDM-K64F provides an onboard tricolor (RGB) LED. The cathodes are connected
to MCU GPIO pins, so driving a pin low illuminates the corresponding color:

===== ========================================================
LED   K64 Port Pin / Function
===== ========================================================
RED   PTB22 / SPI2_SOUT / FB_AD29 / CMP2_OUT (Active Low)
BLUE  PTB21 / SPI2_SCK / FB_AD30 / CMP1_OUT (Active Low)
GREEN PTE26 / ENET_1588_CLKIN / UART4_CTS_b / RTC_CLKOUT (Active Low)
===== ========================================================

When ``CONFIG_ARCH_LEDS`` is enabled, NuttX indicates kernel state using the
RGB LED:

==================== ======================= ===== ===== =====
Symbol               Meaning                 RED   GREEN BLUE
==================== ======================= ===== ===== =====
``LED_STARTED``      NuttX has started       OFF   OFF   OFF
``LED_HEAPALLOCATE`` Heap allocated          OFF   OFF   ON
``LED_IRQSENABLED``  Interrupts enabled      OFF   OFF   ON
``LED_STACKCREATED`` Idle stack created      OFF   ON    OFF
``LED_INIRQ``        Handling an interrupt   N/C   N/C   N/C
``LED_SIGNAL``       Inside signal handler   N/C   N/C   N/C
``LED_ASSERTION``    Assertion failed        N/C   N/C   N/C
``LED_PANIC``        Kernel crash / panic    FLASH OFF   OFF
==================== ======================= ===== ===== =====

During normal operation, the green LED remains illuminated. In the event of a
fatal kernel panic, the red LED flashes.

User Pushbuttons
----------------

The board includes two user pushbuttons:

====== ======================= ==============================================
Switch Port Pin                Features
====== ======================= ==============================================
SW2    PTC6 / LLWU_P10         User button, Low-Leakage Wake-up unit input
SW3    PTA4 / NMI_b / LLWU_P3  User button, Non-Maskable Interrupt, Wake-up
====== ======================= ==============================================

.. note::
   Pin PTA4 is multiplexed with the Cortex-M4 Non-Maskable Interrupt (NMI).
   By default, NuttX configures PTA4 as a standard GPIO with pull-up.

Ethernet (KSZ8081RNA PHY)
-------------------------

The onboard Micrel / Microchip KSZ8081RNA Ethernet PHY connects to the K64
Ethernet MAC via Reduced Media Independent Interface (RMII):

============ ================= ===================== ========================
PHY Signal   Board Net Name    K64 Port Pin          Pinmux Function
============ ================= ===================== ========================
MDIO         RMII0_MDIO        PTB0                  PIN_RMII0_MDIO
MDC          RMII0_MDC         PTB1                  PIN_RMII0_MDC
RXD0         RMII0_RXD_0       PTA13                 PIN_RMII0_RXD0
RXD1         RMII0_RXD_1       PTA12                 PIN_RMII0_RXD1
CRS_DV       RMII0_CRS_DV      PTA14                 PIN_RMII0_CRS_DV
RXER         RMII0_RXER        PTA5                  PIN_RMII0_RXER
REF_CLK      RMII_RXCLK        PTA18 / EXTAL0        50 MHz Clock Input
TXEN         RMII0_TXEN        PTA15                 PIN_RMII0_TXEN
TXD0         RMII0_TXD_0       PTA16                 PIN_RMII0_TXD0
TXD1         RMII0_TXD_1       PTA17                 PIN_RMII0_TXD1
RST_B        PHY_RST_B         Reset line            Active Low
============ ================= ===================== ========================

.. important::
   The FRDM-K64F board does not provide an external pull-up resistor on the
   MDIO signal line. Therefore, the internal pull-up resistor on the MCU must
   be enabled via ``CONFIG_KINETIS_ENET_MDIOPULLUP=y`` in the NuttX configuration.

MicroSD Card Slot (SDHC)
------------------------

A push-push MicroSD slot connects directly to the K64 SD Host Controller (SDHC)
over a 4-bit SDIO interface:

============ ================= =====================
SD Card Net  Board Signal      K64 Port Pin
============ ================= =====================
DAT0         SDHC0_D0          PTE1
DAT1         SDHC0_D1          PTE0
DAT2         SDHC0_D2          PTE5
CD/DAT3      SDHC0_D3          PTE4
CMD          SDHC0_CMD         PTE3
CLK          SDHC0_DCLK        PTE2
CARD_DETECT  D_CARD_DETECT     PTE6 (Active Low switch)
============ ================= =====================

Using the MicroSD Card
~~~~~~~~~~~~~~~~~~~~~~

Once booted, the SD card block driver registers at ``/dev/mmcsd0``.

Manual Mount and Unmount:

.. code-block:: bash

   # Mount FAT filesystem to /mnt/sdcard
   nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard

   # List files and read contents
   nsh> ls /mnt/sdcard
   nsh> cat /mnt/sdcard/test.txt

   # Always unmount before removing the card
   nsh> umount /mnt/sdcard

.. note::
   If the ``mount`` command returns ``mount failed: 19``, this corresponds to
   POSIX error ``ENODEV`` (No such device), indicating that no MicroSD card is
   present in the slot.

Auto-Mounter Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~

NuttX provides an automated filesystem mounter that detects insertion and
removal via the card-detect switch on pin PTE6:

.. code-block:: text

   CONFIG_FS_AUTOMOUNTER=y
   CONFIG_FRDMK64F_SDHC_AUTOMOUNT=y
   CONFIG_FRDMK64F_SDHC_AUTOMOUNT_FSTYPE="vfat"
   CONFIG_FRDMK64F_SDHC_AUTOMOUNT_BLKDEV="/dev/mmcsd0"
   CONFIG_FRDMK64F_SDHC_AUTOMOUNT_MOUNTPOINT="/mnt/sdcard"
   CONFIG_FRDMK64F_SDHC_AUTOMOUNT_DDELAY=1000
   CONFIG_FRDMK64F_SDHC_AUTOMOUNT_UDELAY=2000

With the auto-mounter enabled, inserting a MicroSD card mounts the FAT filesystem
at ``/mnt/sdcard`` automatically, and removing it unmounts the volume.

.. warning::
   While the auto-mounter automatically unmounts the volume upon detecting card
   removal via the card-detect switch, this occurs after the card has already
   begun disconnecting. If the card is physically removed while write operations
   or cache flushes are in progress, data loss or filesystem corruption can still
   occur. Users should always ensure all files are closed and flush/unmount the
   filesystem before physically extracting the MicroSD card.

Flashing and Debugging
======================

The FRDM-K64F features the OpenSDAv2 multi-function debug interface. There are
several methods for flashing and debugging NuttX firmware:

OpenSDAv2 Mass Storage Device (MSD) Drag-and-Drop
-------------------------------------------------

1. Build NuttX to generate a raw binary image (``nuttx.bin``):

   .. code-block:: bash

      ./tools/configure.sh freedom-k64f:nsh
      make -j$(nproc)

2. Connect a micro-USB cable between your host computer and the OpenSDA USB port
   (**J26**, labeled ``SDA``).

3. The board mounts on your host system as a removable mass-storage device named
   ``FRDM-K64F`` or ``MBED``.

4. Copy ``nuttx.bin`` onto the drive:

   .. code-block:: bash

      cp nuttx.bin /media/$USER/FRDM-K64F/

5. The OpenSDA status LED blinks during programming. Once complete, the board
   resets automatically and begins executing NuttX.

pyOCD (CMSIS-DAP)
-----------------

`pyOCD <https://pyocd.io/>`_ is an open-source tool for programming and
debugging ARM Cortex-M microcontrollers over CMSIS-DAP:

.. code-block:: bash

   # Flash the binary
   pyocd flash -t k64f nuttx.bin

   # Or start a GDB debug server on port 3333
   pyocd gdbserver -t k64f

OpenOCD
-------

You can also use OpenOCD via the CMSIS-DAP interface:

.. code-block:: bash

   openocd -f interface/cmsis-dap.cfg -f target/kx.cfg \
           -c "program nuttx.bin 0x00000000 verify reset exit"

J-Link OpenSDA Firmware
-----------------------

The OpenSDAv2 processor can be loaded with Segger J-Link OpenSDA firmware,
converting the onboard interface into a standard Segger J-Link debugger.
Once loaded, you can program and debug using ``JLinkExe``:

.. code-block:: bash

   JLinkExe -device MK64FN1M0xxx12 -if SWD -speed 4000 -autoconnect 1
   loadfile nuttx.bin 0x00000000
   r
   g

Supported Configurations
========================

The board configurations for the FRDM-K64F are located under
``boards/arm/kinetis/freedom-k64f/configs/``.

nsh
---

The default NuttShell (NSH) configuration provides an interactive command-line
interface over the OpenSDAv2 virtual serial port.

- **Console**: UART0 (PTB16 RX / PTB17 TX) at 115200 8N1 via OpenSDA VCOM
- **Memory**: 128 KB configured SRAM heap, ProcFS filesystem
- **Building**:

  .. code-block:: bash

     ./tools/configure.sh freedom-k64f:nsh
     make -j$(nproc)
     cp nuttx.bin /media/$USER/FRDM-K64F/

netnsh
------

Extends the NSH environment with full 10/100 Mbps Ethernet network support via
the onboard KSZ8081 PHY.

- **Networking Features**:

  - IPv4 TCP/IP and UDP protocol stacks
  - ICMP socket support for the NSH ``ping`` utility
  - Telnet daemon utility (``CONFIG_NETUTILS_TELNETD=y``)
  - TFTP client (``tftpc``) for network file transfers
  - HTTP web client (``webclient``) for ``wget`` commands
  - Dynamic DHCP client (``dhcpc``) or static IP configuration (default ``10.0.0.2``)

- **Network Bringup Options**:

  - ``CONFIG_NETINIT_THREAD=y``: Runs network bringup on an asynchronous background thread, preventing boot pauses when the network cable is disconnected.
  - ``CONFIG_NETDEV_PHY_IOCTL=y``: Enables PHY link monitoring and carrier detection.

- **Building**:

  .. code-block:: bash

     ./tools/configure.sh freedom-k64f:netnsh
     make -j$(nproc)
     cp nuttx.bin /media/$USER/FRDM-K64F/

- **Network Verification**:

  .. code-block:: bash

     # From NSH prompt (via OpenSDA VCOM):
     nsh> ifconfig
     nsh> ping 10.0.0.1

  .. note::
     To enable an interactive remote NSH shell over Telnet, enable
     ``CONFIG_NSH_TELNET=y`` in ``make menuconfig``. Once enabled, connect
     from a host terminal using ``telnet 10.0.0.2``.

cdcacm
------

Configures the NuttShell console on OpenSDA UART0 while enabling the native
K64 USB Micro-B port (connector J22) as a CDC/ACM serial device
(``/dev/ttyACM0``).

- **Console**: UART0 (OpenSDA VCOM) at 115200 8N1
- **USB Device**: CDC/ACM virtual serial class driver (``CONFIG_CDCACM=y``)
- **Utilities**: Builtin ``sercon`` and ``serdis`` commands to dynamically connect and disconnect the CDC/ACM interface
- **Peripherals**: FlexTimer PWM (FTM0 channel 2) and SDHC MicroSD storage support
- **Building**:

  .. code-block:: bash

     ./tools/configure.sh freedom-k64f:cdcacm
     make -j$(nproc)
     cp nuttx.bin /media/$USER/FRDM-K64F/

- **Using the CDC/ACM Serial Interface**:

  1. Connect a USB cable to the native K64 USB Micro-B port (**J22**).
  2. From the NSH prompt on the OpenSDA console, attach the USB device:

     .. code-block:: bash

        nsh> sercon

  3. The host computer enumerates the serial device (e.g., ``/dev/ttyACM0`` on Linux).
  4. To disconnect the USB device cleanly:

     .. code-block:: bash

        nsh> serdis

demo
----

A comprehensive feature demonstration configuration enabling multiple onboard
peripherals:

- **Features**:

  - NSH console on OpenSDA UART0
  - FXOS8700CQ 6-axis accelerometer/magnetometer sensor example application (``examples/fxos8700cq``)
  - 10/100 Mbps Ethernet networking stack with DNS client and netcat
  - FTM0 PWM driver
  - SDHC MicroSD storage support

- **Building**:

  .. code-block:: bash

     ./tools/configure.sh freedom-k64f:demo
     make -j$(nproc)
     cp nuttx.bin /media/$USER/FRDM-K64F/
