README
======

This README discusses issues unique to NuttX configurations for STM32L476ME
part in the Motorola MDK.  This is referred to as the MuC in Motorola
technical documentation.

STM32L476ME:

  Microprocessor: 32-bit ARM Cortex M4 at 80MHz STM32L476ME
  Memory:         1024 KB Flash and 96+32 KB SRAM
  ADC:            3x12-bit, 2.4 MSPS A/D converter: up to 24 channels
  DMA:            16-stream DMA controllers with FIFOs and burst support
  Timers:         Up to 11 timers: up to eight 16-bit, two 32-bit timers, two
                  watchdog timers, and a SysTick timer
  GPIO:           Up to 51 I/O ports with interrupt capability
  I2C:            Up to 3 x I2C interfaces
  USARTs:         Up to 3 USARTs, 2 UARTs, 1 LPUART
  SPIs:           Up to 3 SPIs
  SAIs:           Up to 2 dual-channel audio interfaces
  CAN interface
  SDIO interface (not connected)
  QSPI interface (not connected)
  USB:            USB 2.0 full-speed device/host/OTG controller with on-chip PHY
  CRC calculation unit
  RTC

Flashing
========

The MDK has a builtin FTDI to support flashing from openocd.  There are a
few extensions to openocd that haven't been integrated upstream yet.  To
flash (or debug) the MDK, you will need the code from:

  git clone https://github.com/MotorolaMobilityLLC/openocd

Refer to detailed OpenOCD build instructions at developer.motorola.com

After building, you can flash the STM32L476 (MuC) with the following
command:

  openocd -f board/moto_mdk_muc.cfg -c "program nuttx.bin 0x08000000 reset exit"

Serial Console
==============

The serial console is configured on USART3 using MUC_UART3_TX (PC10) and
MUC_UART_RX (PC11).  This connects to the FT4232 part which supports 4
CDC/ACM serial ports.  The MuC console is on port C which will probably be
/dev/ttyUSB2 on your Linux host.  Port A (ttyUSB0) is the MuC SWD debug
interface.  Ports B and D are the MHB* debug and console ports, respectively.

*Then MHB is the acronym given to Toshiba Interface Bridged, part number
T6WV7XBG.  See https://toshiba.semicon-storage.com/us/product/assp/interface-bridge.html
