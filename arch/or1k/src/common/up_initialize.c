/****************************************************************************
 * arch/or1k/src/common/up_initialize.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/board/board.h>
#include <arch/spr.h>

#include "up_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  It is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_OR1K_ICACHE
void up_enable_icache(void)
{
  uint32_t iccfg;
  uint32_t sr;
  uint32_t bir;

  mfspr(SPR_SYS_ICCFGR, iccfg);

  syslog(LOG_INFO,
         "ICACHE NCW: %d NCS: %d CBS: %d CCRI: %d CBIRI: %d CBPRI: %d\n",
         (iccfg & SPR_ICCFGR_NCW_MASK) >> SPR_ICCFGR_NCW_SHIFT,
         (iccfg & SPR_ICCFGR_NCS_MASK) >> SPR_ICCFGR_NCS_SHIFT,
         (iccfg & SPR_ICCFGR_CBS) ? 1 : 0,
         (iccfg & SPR_ICCFGR_CCRI) ? 1 : 0,
         (iccfg & SPR_ICCFGR_CBIRI) ? 1 : 0,
         (iccfg & SPR_ICCFGR_CBPRI) ? 1 : 0,
         (iccfg & SPR_ICCFGR_CBLRI) ? 1 : 0);

  /* Invalidate blocks */

  bir = 0xffffffff;
  mtspr(SPR_ICACHE_BIR, bir);

  mfspr(SPR_SYS_SR, sr);
  sr |= SPR_SR_ICE;
  mtspr(SPR_SYS_SR, sr);
}
#endif

#ifdef CONFIG_OR1K_DCACHE
void up_enable_dcache(void)
{
  uint32_t dccfg;
  uint32_t sr;
  uint32_t bir;

  mfspr(SPR_SYS_DCCFGR, dccfg);

  syslog(LOG_INFO,
         "DCACHE NCW: %d NCS: %d CBS: %d CCRI: %d CBIRI: %d CBPRI: %d\n",
         (dccfg & SPR_DCCFGR_NCW_MASK) >> SPR_DCCFGR_NCW_SHIFT,
         (dccfg & SPR_DCCFGR_NCS_MASK) >> SPR_DCCFGR_NCS_SHIFT,
         (dccfg & SPR_DCCFGR_CBS) ? 1 : 0,
         (dccfg & SPR_DCCFGR_CCRI) ? 1 : 0,
         (dccfg & SPR_DCCFGR_CBIRI) ? 1 : 0,
         (dccfg & SPR_DCCFGR_CBPRI) ? 1 : 0,
         (dccfg & SPR_DCCFGR_CBLRI) ? 1 : 0);

  bir = 0xffffffff;
  mtspr(SPR_DCACHE_BIR, bir);

  mfspr(SPR_SYS_SR, sr);
  sr |= SPR_SR_DCE;
  mtspr(SPR_SYS_SR, sr);
}
#endif

/****************************************************************************
 * Name: up_color_intstack
 *
 * Description:
 *   Set the interrupt stack to a value so that later we can determine how
 *   much stack space was used by interrupt handling logic
 *
 ****************************************************************************/

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 3
static inline void up_color_intstack(void)
{
  uint32_t *ptr = (uint32_t *)&g_intstackalloc;
  ssize_t size;

  for (size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
       size > 0;
       size -= sizeof(uint32_t))
    {
      *ptr++ = INTSTACK_COLOR;
    }
}
#else
#  define up_color_intstack()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS initialization after the
 *   basic OS services have been initialized.  The architecture specific
 *   details of initializing the OS will be handled here.  Such things as
 *   setting up interrupt service routines, starting the clock, and
 *   registering device drivers are some of the things that are different
 *   for each processor and hardware platform.
 *
 *   up_initialize is called after the OS initialized but before the user
 *   initialization logic has been started and before the libraries have
 *   been initialized.  OS services and driver services are available.
 *
 ****************************************************************************/

void up_initialize(void)
{
  /* Colorize the interrupt stack */

  up_color_intstack();

  /* Add any extra memory fragments to the memory manager */

  up_addregion();

#ifdef CONFIG_PM
  /* Initialize the power management subsystem.  This MCU-specific function
   * must be called *very* early in the initialization sequence *before* any
   * other device drivers are initialized (since they may attempt to register
   * with the power management subsystem).
   */

  up_pminitialize();
#endif

#ifdef CONFIG_ARCH_DMA
  /* Initialize the DMA subsystem if the weak function up_dma_initialize has
   * been brought into the build
   */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (up_dma_initialize)
#endif
    {
      up_dma_initialize();
    }
#endif

  /* Initialize the serial device driver */

  up_serialinit();

  /* Print OpenRISC CPU information */

  or1k_print_cpuinfo();

  /* Initialize the network */

  up_netinitialize();

  /* Initialize USB -- device and/or host */

  up_usbinitialize();

  /* Initialize the L2 cache if present and selected */

  up_l2ccinitialize();

#ifdef CONFIG_OR1K_ICACHE
  up_enable_icache();
#endif

#ifdef CONFIG_OR1K_DCACHE
  up_enable_dcache();
#endif

  board_autoled_on(LED_IRQSENABLED);
}
