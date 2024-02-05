=========================
ON Semiconductor LC823450
=========================

(Dual core ARM Cortex-M3). In NuttX-7.22,
Masayuki Ishikawa contributed support for both the LC823450 architecture
and for ON Semiconductor's **LC823450XGEVK board**:

   The LC823450XGEVK is an audio processing system Evaluation Board Kit
   used to demonstrate the LC823450. This part can record and playback,
   and offers High-Resolution 32-bit & 192 kHz audio processing
   capability. It is possible to cover most of the functions necessary
   for a portable audio with only this LSI as follows. It has Dual CPU
   and DSP with High processing capability, and internal 1656K-Byte
   SRAM, which make it possible to implement large scale program. And it
   has integrated analog functions (low-power Class D HP amplifier, PLL,
   ADC etc.) so that PCB space and cost is reduced, and it has various
   interface (USB, SD, SPI, UART, etc.) to make extensibility high. Also
   it is provided with various function including SBC/AAC codec by DSP
   and UART and ASRC (Asynchronous Sample Rate Converter) for Bluetooth®
   audio. It is very small chip size in spite of the multi-funciton as
   described above and it realizes the low power consumption. Therefore,
   it is applicable to portable audio markets such as Wireless headsets
   and will show high performance.

Further information about the LC823450XGEVK is available on from the the
`ON
Semiconductor <http://www.onsemi.com/PowerSolutions/evalBoard.do?id=LC823450XGEVK>`__
website as are LC823450 `related technical
documents <http://www.onsemi.com/PowerSolutions/supportDoc.do?type=AppNotes&rpn=LC823450>`__.
Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/arm/lc823450/lc823450-xgevk/README.txt>`__
file for details of the NuttX port.

This port is intended to test LC823450 features including SMP. Supported
peripherals include UART, TIMER, RTC, GPIO, DMA, I2C, SPI, LCD, eMMC,
and USB device. ADC, Watchdog, IPC2, and I2S support was added by
Masayuki Ishikawa in NuttX-7.23. Bluetooth, SPI, and *PROTECTED* build
support were added by Masayuki Ishikawa in NuttX-7.26. Support for for
SPI flash boot was added in NuttX-7.28.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
