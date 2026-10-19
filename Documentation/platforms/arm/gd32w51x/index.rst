===================
GigaDevice GD32W51x
===================

The GD32W51x is a Wi-Fi 4 (802.11 b/g/n, 2.4 GHz) MCU built around an Arm
Cortex-M33 core with TrustZone. Its peripherals are the ones of the
GD32VW55x family (same register layout and the same non-secure memory map),
so most drivers are shared in design with ``arch/risc-v/src/gd32vw55x``.

- Arm Cortex-M33 (ARMv8-M Mainline), up to 180 MHz, single-precision FPU, DSP
- TrustZone: SAU and IDAU, secure and non-secure aliases of every peripheral
- 384/448 KB SRAM, 1/2 MB in-package (SIP) flash
- 2.4 GHz 802.11 b/g/n radio

Supported chips
===============

============== ========= ========= ========
Chip           Package   Flash     SRAM
============== ========= ========= ========
GD32W515PIQ6   QFN56     2048 KB   448 KB
GD32W515TIQ6   QFN36     2048 KB   448 KB
GD32W515TGQ6   QFN36     1024 KB   384 KB
============== ========= ========= ========

Clocks
======

``gd32w51x_clockconfig()`` runs the PLL from the HXTAL to a 180 MHz system
clock. The board gives the crystal frequency in ``BOARD_HXTAL_FREQUENCY``
(a whole number of MHz) and the PLL input divider is derived from it:

============ =========== =================================================
Clock        Frequency   Notes
============ =========== =================================================
SYSCLK, AHB  180 MHz     PLL: HXTAL / (HXTAL in MHz) x 360 / 2
APB2         90 MHz      USART2, SPI0, ADC, TIMER0/15/16
APB1         45 MHz      USART0/1, I2C, TIMER1..5, watchdogs
Timers       90/180 MHz  2 x APB (TIMERSEL = 0)
============ =========== =================================================

The core voltage is raised (``PMU_CTL0.LDOVS``) before switching to the PLL,
and the instruction cache in front of the flash is enabled at boot.

TrustZone
=========

The GD32W515 chips come with the TZEN option bit set. Clearing it requires a
security protection level regression, which mass-erases the flash, so this
port keeps TrustZone enabled and runs NuttX in **secure state** with the SAU
disabled (everything is secure):

- The image is linked at the non-secure alias of the flash (0x08000000). With
  TZEN set, the boot ROM starts it through the secure alias (0x0c000000).
- The flash pages that hold the image must be marked secure in the
  ``FMC_SECMCFGx`` option bytes: the core in secure state cannot fetch code
  from a non-secure page. See the board documentation.
- ``gd32w51x_tzinit()`` releases the GPIO pins (``GPIOx_SCFG``) at boot. The
  bus peripherals are non-secure out of reset, so every driver uses the
  non-secure aliases (0x4xxxxxxx), as on a part without TrustZone.
- The program memory driver erases and programs the secure flash pages
  through the secure register set of the FMC.
- The QSPI is a secure peripheral: its alternate functions only reach
  secure pins, so the QSPI driver marks its pins secure again.
- A non-secure DMA channel reads the secure SRAM as zero: the DMA driver
  makes every channel it allocates secure (``DMA_CHxSCTL``).

Peripheral support
==================

========== ======= ==========================================================
Peripheral Support Notes
========== ======= ==========================================================
GPIO       Yes     Including EXTI interrupts
USART      Yes     USART0, USART1 and USART2 (UART1/UART2 in Kconfig)
I2C        Yes     I2C0 and I2C1, I2C0 kernel clock from IRC16M
SPI        Yes     SPI0 and SPI1, with DMA (I2S mode not supported)
ADC        Yes     12-bit, software trigger
TIMER      Yes     PWM; input capture not tested on hardware yet
RTC        Yes     Clocked from the IRC32K
WDT        Yes     FWDGT and WWDGT
TRNG       Yes     /dev/random
CRC        Yes
FMC        Yes     Program memory (MTD), e.g. for LittleFS
DMA        Yes     DMA0 and DMA1 (SPI0 is served by DMA1)
QSPI       Yes     Indirect mode, polled; the pins are made secure
TSI        Yes     Software trigger, polled (board touch keys as buttons)
USBFS      No
Wi-Fi      No      The vendor libraries support 26 and 40 MHz crystals only
========== ======= ==========================================================

Supported boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
