
The directory contains start-up and board level functions. 
Execution starts in the following order:

 - sysclock, immediately after reset stm32_rcc calls external
   clock configuration when
     CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=y
   is set. It must be set for the VSN board.
   
 - boot, performs initial chip and board initialization
 - ...
 - nsh, as central application last.
 
 


JTAG options:
=============

CONFIG_STM32_JTAG_FULL_ENABLE			// Complete parallel and serial
CONFIG_STM32_JTAG_NOJNTRST_ENABLE		// no JNTRST pin
CONFIG_STM32_JTAG_SW_ENABLE				// serial (dual pin) only (can coexist besides the FRAM on SPI3)
