===================================
Nanopower nPZero G1S DevKit (STM32)
===================================

.. tags:: arch:arm, chip:stm32l0, part:stm32l053, vendor:nanopower

This page discusses the NuttX support for the STM32L053C8U6 MCU host board of
the nPZero G1S Development Kit (NPZ-G1S-DK) from Nanopower Semiconductor.

The kit is made of:

- the nPZero G1S Development Board (DevBoard), with the nPZero G1S
  power-saving IC, the power sources, the power switch LEDs and four PMOD
  connectors for the peripherals,
- the MCU host, an STM32L053C8U6 (64 KB of FLASH, 8 KB of SRAM) on a board
  plugged in the Arduino Uno connectors of the DevBoard, with a 10-pin JTAG/SWD
  connector,
- a LIS2DW12 accelerometer board (SPI) for the PMOD 1 or 3 connectors,
- an AS6212 temperature sensor board (I2C) for the PMOD 2 or 4 connectors.

The nPZero switches the MCU host off and on: at each wake-up, NuttX boots,
the application reads the wake-up reason, configures the nPZero again and
sends the Idle command, which switches the host off. See
:doc:`/os/drivers/special/power/npzero`.

Pins
====

====== ============== ===========================================
Pin    Signal         Notes
====== ============== ===========================================
PA2    USART2_TX      Console, MCU UART connector J19 of the DevBoard
PA3    USART2_RX      Console, MCU UART connector J19 of the DevBoard
PB6    I2C1_SCL       nPZero host I2C
PB7    I2C1_SDA       nPZero host I2C
PA12   LED            MCU host LED (``CONFIG_ARCH_LEDS``)
PA13   SWDIO          JTAG/SWD connector J1 of the MCU host
PA14   SWCLK          JTAG/SWD connector J1 of the MCU host
====== ============== ===========================================

The system clock is the 16 MHz HSI: the MCU host has no crystal, and the
STM32 I2C driver computes its timings for a 16 MHz I2C clock.

The LEDs D1 to D6 of the DevBoard show the power supply, the host switch and
the four peripheral switches of the nPZero (jumper J20 enables them).

Hardware setup
==============

As described in the nPZero G1S DevKit User Guide:

#. Plug the MCU host in the Arduino connectors of the DevBoard and check that
   the jumper J18 (power of the MCU host) is in place.
#. Connect the LIS2DW12 board to PMOD 3 and the AS6212 board to PMOD 4.
#. Connect a USB to UART adapter to J19 (the User Guide uses an FTDI
   TTL-232R-5V). The console is 115200 8N1.
#. Select the power source with S1 (banana/J11, USB-C or coin cell) and the
   voltage with S2 (1.8 V or 3.3 V).
#. Press S5 to reset the nPZero and the MCU host.

The host I2C bus is shared with the I2C peripherals.  After a reset of the
nPZero, the power switches of the peripherals are off, and an unpowered
AS6212 board on PMOD 2 or 4 slows down the rise of SDA (to about 10 us,
100 us when SCL is low): the STM32 then reports an arbitration loss and
``npzero_register()`` fails with ``-EAGAIN`` (-11).  The vendor firmware
fails in the same way.

With ``CONFIG_NPZERO_G1S_DK_PMOD_POWER`` (enabled by default), the board
enables the power switches of PMOD 3 and 4 in the PSWCTL register with a
slow (about 2 kHz) bit-banged I2C transfer on PB6/PB7, before the I2C driver
is initialized.  The other fields of PSWCTL are kept.

Do not plug a sensor board while the host is running: the unpowered board
blocks the bus, and the STM32 I2C driver then times out (``-ETIMEDOUT``)
until the MCU host is reset.

Flashing
========

Connect a J-Link (or another SWD probe) to J1 of the MCU host. The host must be
powered while it is flashed, but the nPZero switches it off in Idle state.
To keep it on, connect VBAT (TP30) to VCC (pin 6 of the PMOD MCU connector
J17), press S5, flash, and then remove the connection.

With OpenOCD, the MCU host is lost by the debugger after a reset (``Could not
find MEM-AP``), so ``program ... reset`` fails. Halt the core, write the FLASH
and restart it with a system reset request instead (tested with an ORBTrace
Mini; ``<probe>.cfg`` is the configuration of your SWD probe)::

  $ openocd -f <probe>.cfg -f target/stm32l0.cfg \
      -c "init; halt; flash write_image erase nuttx.bin 0x08000000" \
      -c "verify_image nuttx.bin 0x08000000" \
      -c "mww 0xE000ED0C 0x05FA0004; shutdown"

Or with the J-Link tools (not tested)::

  $ JLinkExe -device STM32L053C8 -if SWD -speed 4000 -autoconnect 1
  J-Link> loadbin nuttx.bin 0x08000000
  J-Link> r
  J-Link> g

Configurations
==============

Each configuration is selected with::

  $ ./tools/configure.sh npzero-g1s-dk:<subdir>
  $ make

npzero
------

The application is the ``npzero`` example
(:doc:`/applications/examples/npzero/index`), started without NSH. At each
wake-up it shows the wake-up reason and the values of the sensors, configures
the nPZero to poll the accelerometer every 5 s and the temperature sensor every
30 s, and enters Idle state after one second. The host is woken up when the X
acceleration is above about 61 mg, when the temperature is above 25 C or below
10 C, or after 300 s.

This is the configuration for the normal nPZero operation: it uses about
38 KB of FLASH.

nsh
---

NSH on USART2, with the nPZero driver (``/dev/npzero0``) and the ``npzero``
command. ``npzero -s`` shows the status only, while ``npzero`` configures the
nPZero and enters Idle state, which switches the host off. After a reset of
the nPZero (S5)::

  nsh> npzero -s
  nPZero G1S ID: 0x60
  STA1: 0x01 STA2: 0x00
  nsh> npzero
  nPZero G1S ID: 0x60
  STA1: 0x01 STA2: 0x00
  Configured, Idle state in 1000 ms

The 8 KB of SRAM is the limit of this configuration: it uses
``CONFIG_DEFAULT_SMALL`` and only the ``cat``, ``echo``, ``help`` and ``ls``
commands are enabled.

Notes
=====

``CONFIG_BOARD_LOOPSPERMSEC`` is scaled from the STM32L0538-DISCO (32 MHz) to
16 MHz and has not been calibrated.
