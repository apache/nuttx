
The directory contains start-up and board level functions. 
Execution starts in the following order:

 - sysclock, immediately after reset stm32_rcc calls external
   clock configuration when
     CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=y
   is set. It must be set for the VSN board.
   
 - boot, performs initial chip and board initialization
 - ...
 - nsh, as central application last.
