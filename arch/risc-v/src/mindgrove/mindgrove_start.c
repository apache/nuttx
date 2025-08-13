/****************************************************************************
 * Included Files
 ****************************************************************************/

 #include <nuttx/config.h>

 #include <stdint.h>
 
 #include <nuttx/init.h>
 #include <arch/board/board.h>
 #include "chip.h"
 
 #include "riscv_internal.h"
//  #include "riscv_arch.h"
 #include"mindgrove_clockconfig.h"
 
 /****************************************************************************
  * Pre-processor Definitions
  ****************************************************************************/
 
  #ifdef CONFIG_DEBUG_FEATURES
  #  define showprogress(c) riscv_lowputc(c)
  #else
  #  define showprogress(c)
  #endif
 
  void mindgrove_lowsetup(void);
  void mindgrove_boardinitialize(void);

 uintptr_t g_idle_topstack = MINDGROVE_IDLESTACK_TOP;
 
 void __mindgrove_start(void)
 {
  
   const uint32_t *src;
   uint32_t *dest;
 
   /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
    * certain that there are no issues with the state of global variables.
    */
 
   for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
     {
       *dest++ = 0;
     }
 
   /* Move the initialized data section from his temporary holding spot in
    * FLASH into the correct place in SRAM.  The correct place in SRAM is
    * give by _sdata and _edata.  The temporary location is in FLASH at the
    * end of all of the other read-only data (.text, .rodata) at _eronly.
    */
 
   /* for vexriscv the full image is loaded in ddr ram */
 
   for (src = (uint32_t *)_eronly, dest = (uint32_t *)_sdata; dest < (uint32_t *)_edata; )
     {
       *dest++ = *src++;
     }
 
   /* Setup PLL */
 
   // mindgrove_clockconfig();
 
  /* Configure the UART so we can get debug output */
 
   mindgrove_lowsetup();
 
   // CURRENT_REGS = NULL;
   // showprogress('A');
 
 #ifdef USE_EARLYSERIALINIT
   riscv_earlyserialinit();
 #endif
 
   // showprogress('B');
 
   /* Do board initialization */
 
   mindgrove_boardinitialize();
 
   // showprogress('C');
 
   /* Call nx_start() */
 
   nx_start();
 
   /* Shouldn't get here */
 
   for (; ; );
 }