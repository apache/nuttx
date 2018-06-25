README.txt
==========

STM32F746G-DISCO LTDC Framebuffer demo example

Preparation
-----------

As of writing this text, SDRAM support is not implemented for this board.
Therefore to make this demo work following changes are required to heap
management function:


diff --git a/stm32_allocateheap.c b/stm32_allocateheap.c
--- a/stm32_allocateheap.c
+++ b/stm32_allocateheap.c
@@ -93,7 +93,7 @@
 /* Set the start and end of SRAM1 and SRAM2 */
 
 #define SRAM1_START  STM32_SRAM1_BASE
-#define SRAM1_END    (SRAM1_START + STM32F7_SRAM1_SIZE)
+#define SRAM1_END    (SRAM1_START + 131072)
 
 #define SRAM2_START  STM32_SRAM2_BASE
 #define SRAM2_END    (SRAM2_START + STM32F7_SRAM2_SIZE)
@@ -385,17 +385,17 @@ void up_addregion(void)
 
   /* Allow user-mode access to the STM32F20xxx/STM32F40xxx SRAM2 heap */
 
-  stm32_mpu_uheap((uintptr_t)SRAM2_START, SRAM2_END-SRAM2_START);
+/*  stm32_mpu_uheap((uintptr_t)SRAM2_START, SRAM2_END-SRAM2_START); */
 
 #endif
 
   /* Colorize the heap for debug */
 
-  up_heap_color((FAR void *)SRAM2_START, SRAM2_END-SRAM2_START);
+/*  up_heap_color((FAR void *)SRAM2_START, SRAM2_END-SRAM2_START); */
 
   /* Add the STM32F20xxx/STM32F40xxx SRAM2 user heap region. */
 
-  kumm_addregion((FAR void *)SRAM2_START, SRAM2_END-SRAM2_START);
+/*  kumm_addregion((FAR void *)SRAM2_START, SRAM2_END-SRAM2_START); */
 
 #ifdef HAVE_DTCM
 #if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)


Configure and build
-------------------

tools/configure.sh stm32f746g-disco/fb
make


Configuration
------------

This configuration provides 1 LTDC with
8pp CLUT pixel format and a resolution of 480x272.


Loading
-------

st-flash write nuttx.bin 0x8000000


Executing
---------

The ltdc is initialized during boot up.  Interaction with NSH is via the serial
console at 115200 8N1 baud.  From the nsh comandline execute the fb example:

  nsh> fb

The test will put a pattern of concentric squares in the framebuffer and
terminate.
