README
======

This README discusses issues unique to NuttX configurations for Motorola
MDK.

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

Flashing:

The MDK has a builtin FTDI to support flashing from openocd.  There are a few extensions
to openocd that haven't been integrated upstream yet.  To flash (or debug) the MDK, you
will need the code from:
	git clone https://github.com/MotorolaMobilityLLC/openocd

After building, you can flash with the following command:
	openocd -f board/moto_mdk_muc.cfg -c "program nuttx.bin 0x08000000 reset exit"
