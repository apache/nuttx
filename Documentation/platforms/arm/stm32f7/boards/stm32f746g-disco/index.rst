===================
ST STM32F746G-DISCO
===================

.. tags:: chip:stm32, chip:stm32f7, chip:stm32f746

This page discusses issues unique to NuttX configurations for the
STMicro STM32F746G-DISCO development board featuring the STM32F746NGH6
MCU. The STM32F746NGH6  is a 216MHz Cortex-M7 operation with 1024Kb Flash
memory and 300Kb SRAM. The board features:

- On-board ST-LINK/V2 for programming and debugging,
- Mbed-enabled (mbed.org)
- 4.3-inch 480x272 color LCD-TFT with capacitive touch screen
- Camera connector
- SAI audio codec
- Audio line in and line out jack
- Stereo speaker outputs
- Two ST MEMS microphones
- SPDIF RCA input connector
- Two pushbuttons (user and reset)
- 128-Mbit Quad-SPI Flash memory
- 128-Mbit SDRAM (64 Mbits accessible)
- Connector for microSD card
- RF-EEPROM daughterboard connector
- USB OTG HS with Micro-AB connectors
- USB OTG FS with Micro-AB connectors
- Ethernet connector compliant with IEEE-802.3-2002

Refer to the http://www.st.com website for further information about this
board (search keyword: stm32f746g-disco)

Development Environment
=======================

The Development environments for the STM32F746G-DISCO board are identical
to the environments for other STM32F boards.  For full details on the
environment options and setup, see the README.txt file in the
boards/arm/stm32f7/stm32f746g-disco directory.

LEDs and Buttons
================

LEDs
----

The STM32F746G-DISCO board has numerous LEDs but only one, LD1 located
near the reset button, that can be controlled by software (LD2 is a power
indicator, LD3-6 indicate USB status, LD7 is controlled by the ST-Link).

LD1 is controlled by PI1 which is also the SPI2_SCK at the Arduino
interface.  One end of LD1 is grounded so a high output on PI1 will
illuminate the LED.

This LED is not used by the board port unless CONFIG_ARCH_LEDS is defined.
In that case, the usage by the board port is defined in include/board.h
and src/stm32_leds.c. The LEDs are used to encode OS-related events as
follows:

    =================== ======================= ======
    SYMBOL              Meaning                 LD1
    =================== ======================= ======
    LED_STARTED         NuttX has been started  OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF
    LED_IRQSENABLED     Interrupts enabled      OFF
    LED_STACKCREATED    Idle stack created      ON
    LED_INIRQ           In an interrupt         N/C
    LED_SIGNAL          In a signal handler     N/C
    LED_ASSERTION       An assertion failed     N/C
    LED_PANIC           The system has crashed  FLASH
    =================== ======================= ======

  Thus is LD1 is statically on, NuttX has successfully  booted and is,
  apparently, running normally.  If LD1 is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

Buttons
-------

Pushbutton B1, labelled "User", is connected to GPIO PI11.  A high
value will be sensed when the button is depressed.

Serial Console
==============

The STM32F469G-DISCO uses USART1 connected to "Virtual COM", so when you
plug it on your computer it will be detected as a USB port (i.e. ttyACM0):

  ======  ========= =====
  V.COM   FUNCTION  GPIO
  ======  ========= =====
  RXD     USART1_RX PB7
  TXD     USART1_TX PA9
  ======  ========= =====

All you need to do after flashing NuttX on this board is use a serial
console tool (minicom, picocom, screen, hyperterminal, teraterm, putty,
etc ) configured to 115200 8n1.

Configurations
==============

Common Configuration Information
--------------------------------

Each STM32F746G-DISCO configuration is maintained in a sub-directory and
can be selected as follow::

    tools/configure.sh stm32f746g-disco:<subdir>

Where <subdir> is one of the sub-directories listed below.

NOTES:

1. These configurations use the mconf-based configuration tool.  To
   change this configuration using that tool, you should:

   a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
      see additional README.txt files in the NuttX tools repository.

   b. Execute 'make menuconfig' in nuttx/ in order to start the
      reconfiguration process.

2. By default, these configurations use the USART1 for the serial
   console.  Pins are configured to that RX/TX are available at
   pins D0 and D1 of the Arduion connectors.  This should be compatible
   with most RS-232 shields.

3. All of these configurations are set up to build under Windows using the
   "GNU Tools for ARM Embedded Processors" that is maintained by ARM
   (unless stated otherwise in the description of the configuration).

         https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

   As of this writing (2015-03-11), full support is difficult to find
   for the Cortex-M7, but is supported by at least this release of
   the ARM GNU tools:

         https://launchpadlibrarian.net/209776344/release.txt

   hat toolchain selection can easily be reconfigured using
   'make menuconfig'.  Here are the relevant current settings:

   Build Setup::

         CONFIG_HOST_WINDOWS=y               : Window environment
         CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

   System Type -> Toolchain::

         CONFIG_ARM_TOOLCHAIN_GNU_EABI=y  : GNU ARM EABI toolchain

   NOTE: As of this writing, there are issues with using this tool at
   the -Os level of optimization.  This has not been proven to be a
   compiler issue (as least not one that might not be fixed with a
   well placed volatile qualifier).  However, in any event, it is
   recommend that you use not more that -O2 optimization.

Configuration Directories
-------------------------

nsh
---

Configures the NuttShell (NSH) located at apps/examples/nsh.  The
Configuration enables the serial interfaces on USART1.  Support for
built-in applications is enabled, but in the base configuration no
built-in applications are selected.

netnsh
------

This configuration is similar to the nsh but a lot more hardware
peripherals are enabled, in particular Ethernet, as well as networking
support.  It is similar to the stm32f769i-disco/netnsh
configuration. This configuration uses USART1 for the serial console.
USART1 is connected to the ST-link virtual com inside board.h to remove
the need of a extra serial connection to use this board.

dropbear
--------

This configuration brings up the `Dropbear <https://matt.ucc.asn.au/dropbear/dropbear.html>`__
SSH server so that an NSH session can be reached over the network. It pulls in
the on-board Ethernet MAC, the DHCP client and ``/dev/urandom`` so the link is
configured automatically at boot, and it persists the SSH host key and the
user database on the board's on-board Micron N25Q QSPI NOR flash, mounted as
LittleFS at ``/data``.

Connect an Ethernet cable to a router/switch before powering on the board.
When it boots it requests an address over DHCP and starts the Dropbear
daemon. Check the assigned address with::

    nsh> ifconfig
    eth0    Link encap:Ethernet HWaddr 00:e0:de:ad:be:ef at RUNNING mtu 1486
            inet addr:192.168.1.xx DRaddr:192.168.1.x Mask:255.255.255.0

The first time the daemon runs it generates an ECDSA host key and stores it at
``/data/dropbear_ecdsa_host_key`` (``CONFIG_NETUTILS_DROPBEAR_HOSTKEY_PATH``), and
the user accounts are read from ``/data/passwd`` (``CONFIG_FSUTILS_PASSWD_PATH``).
Because both files live on the persistent ``/data`` partition, the host key and
the credentials survive reboots, so clients do not see a changing host key.

Add a user from the board before connecting (the password file is created on the
first ``useradd``)::

    nsh> useradd admin mypassword

From a host on the same network, open an SSH session and run NSH commands
remotely::

    $ ssh admin@192.168.1.xx
    admin@192.168.1.xx's password:
    nsh>

Each session runs over a pseudo-terminal (``CONFIG_PSEUDOTERM``). The daemon
keeps running after a session ends, so multiple clients can connect over time.

This configuration also enables Dropbear's ``scp`` helper
(``CONFIG_NETUTILS_DROPBEAR_SCP``), so files can be copied to and from the
board from a host on the same network::

    $ scp -O localfile.txt admin@192.168.1.xx:/data/localfile.txt
    $ scp -O admin@192.168.1.xx:/data/localfile.txt .

Only the legacy scp protocol is supported (no SFTP), so with OpenSSH 9.0 or
newer the ``-O`` flag is required to force the client to use it.

.. note:: This board's serial console (USART1, ``/dev/ttyACM0`` over the
  on-board ST-LINK/V2-1 virtual COM port) remains available at the same time
  as the SSH sessions and is useful for the initial ``useradd`` and for
  recovery if the network is unreachable.

lgvl
----

STM32F746G-DISCO LittlevGL demo example.

The LTDC is initialized during boot up.
This configuration uses USART1 for the serial console.
USART1 is connected to the ST-link virtual com inside board.h to remove
the need of a extra serial connection to use this board.
From the nsh command line execute the lvgldemo example::

      nsh> lvgldemo

The test will execute the calibration process and then run the
LittlevGL demo project.

STM32F746G-DISCO LTDC Framebuffer demo example
==============================================

Configure and build

tools/configure.sh stm32f746g-disco:fb
make

Configuration

This configuration provides 1 LTDC with
16bpp pixel format and a resolution of 480x272.

Loading

st-flash write nuttx.bin 0x8000000

Executing

The ltdc is initialized during boot up.  Interaction with NSH is via the serial
console provided by ST-LINK USB at 115200 8N1 baud.
From the nsh commandline execute the fb example::

  nsh> fb

The test will put a pattern of concentric squares in the framebuffer and
terminate.

STM32F746G-DISCO NX Terminal example
====================================

Configure and build

tools/configure.sh stm32f746g-disco:nxterm
make

Configuration

This configuration provides 1 LTDC with
16bpp pixel format and a resolution of 480x272.

Trickiest part of config is increasing max message size (CONFIG_MQ_MAXMSGSIZE=256).
NX server - client communication cannot be established with default value 8 bytes.

Loading

st-flash write nuttx.bin 0x8000000

or

openocd -f interface/stlink.cfg -f target/stm32f7x.cfg
telnet localhost 4444
> program nuttx verify reset

Executing

The ltdc is initialized during boot up.  Interaction with NSH is via the serial
console provided by ST-LINK USB at 115200 8N1 baud.

From the nsh commandline execute the example::

  nsh> nxterm

The test will show terminal window on the screen.

STM32F746G-DISCO NX demo example
================================

Configure and build::

  tools/configure.sh stm32f746g-disco:nxdemo
  make

Configuration

This configuration provides 1 LTDC with
16bpp pixel format and a resolution of 480x272.

Trickiest part of config is increasing max message size (CONFIG_MQ_MAXMSGSIZE=256).
NX server - client communication cannot be established with default value 8 bytes.

Loading::

  st-flash write nuttx.bin 0x8000000

or::

  openocd -f interface/stlink.cfg -f target/stm32f7x.cfg
  telnet localhost 4444
  > program nuttx verify reset

Executing

The ltdc is initialized during boot up.  Interaction with NSH is via the serial
console provided by ST-LINK USB at 115200 8N1 baud.

There are two graphics examples provided in this configuration:
- nxdemo
- nxhello

Use help command to show list of examples available::

  nsh> help

From the nsh commandline execute the example::

  nsh> nxdemo

The test will draw animated lines, squares and circles on the device screen.
