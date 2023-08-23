=============
STM32L476-mdk
=============

This page discusses issues unique to NuttX configurations for STM32L476ME
part in the Motorola MDK.  This is referred to as the MuC in Motorola
technical documentation.

STM32L476ME:

- Microprocessor: 32-bit ARM Cortex M4 at 80MHz STM32L476ME
- Memory: 1024 KB Flash and 96+32 KB SRAM
- ADC: 3x12-bit, 2.4 MSPS A/D converter: up to 24 channels
- DMA: 16-stream DMA controllers with FIFOs and burst support
- Timers:Up to 11 timers: up to eight 16-bit, two 32-bit timers, two
  watchdog timers, and a SysTick timer
- GPIO: Up to 51 I/O ports with interrupt capability
- I2C: Up to 3 x I2C interfaces
- USARTs: Up to 3 USARTs, 2 UARTs, 1 LPUART
- SPIs: Up to 3 SPIs
- SAIs: Up to 2 dual-channel audio interfaces
- CAN interface
- SDIO interface (not connected)
- QSPI interface (not connected)
- USB: USB 2.0 full-speed device/host/OTG controller with on-chip PHY
- CRC calculation unit
- RTC

Acronyms
========

MDK is, of course, the Motorola Development Kit.
MuC is the acronym that is used to refer to the STM32L476ME on the MDK board.
MHB is the acronym given to Toshiba Interface Bridge, part number T6WV7XBG.
See https://toshiba.semicon-storage.com/us/product/assp/interface-bridge.html
NuttX runs the MuC.

Flashing
========

The MDK has a built-in FTDI to support flashing from openocd.  There are a
few extensions to openocd that haven't been integrated upstream yet.  To
flash (or debug) the MDK, you will need the code from::

  $ git clone https://github.com/MotorolaMobilityLLC/openocd

Refer to detailed OpenOCD build instructions at developer.motorola.com

After building, you can flash the STM32L476 (MuC) with the following
command::

  $ openocd -f board/moto_mdk_muc.cfg -c "program nuttx.bin 0x08000000 reset exit"

You may need to be super-user in order access the USB device.

NOTE:  In order for the debug Type C connector to power the phone, the DIP
Switch B4 must be in the ON position.  See the MDK User Guide at
developer.motorola.com for more information on the hardware including the DIP
switches.

Or you can use the GDB server.  To start the GDB server::

  $ openocd -f board/moto_mdk_mu_reset.cfg &

Then start GDB::

  $ arm-none-linux-gdb
  (gdb) target extended-remote localhost:3333
  (gdb) set can-use-hw-watchpoints 1

You can load code into FLASH like::

  (gdb) mon halt
  (gdb) load nuttx
  (gdb) file nuttx
  (gdb) mon reset

NOTE:  There is a special version of GDB 7.11 available with some additional,
MDK-specific features.  It is available in a MotorolaMobilityLLC github.com
repository.

Serial Console
==============

The serial console is configured on USART3 using MUC_UART3_TX (PC10) and
MUC_UART_RX (PC11).  This connects to the FT4232 part which supports 4
CDC/ACM serial ports.  The MuC console is on port C which will probably be
/dev/ttyUSB2 on your Linux host.  Port A (ttyUSB0) is the MuC SWD debug
interface.  Ports B and D are the MHB debug and console ports, respectively.

The serial terminal that you use must be configured to use the /dev/ttyUSB2
device at 11500 baud, no parity, 8 bits of data, 1 stop bit (115200 8N1 in
minicom-speak) and with no flow control.  Minicom works well.

You will probably need to be super-user in order access the /dev/ttyUSB2
device::

  $ sudo minicom mdk

When mdk is the name of my saved configuration using the above serial
configuration.

The Motorola documentation also mentions picocom.  NSH also works well with
picocom::

  $ sudo apt install picocom
  $ sudo picocom -b 115200 /dev/ttyUSB2

Everything else defaults correctly.  Ctrl-A then Ctrl-X will terminate
either the minicom or the picocom session.
