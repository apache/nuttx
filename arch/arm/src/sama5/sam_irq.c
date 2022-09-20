/****************************************************************************
 * arch/arm/src/sama5/sam_irq.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "arm_internal.h"

#ifdef CONFIG_SAMA5_PIO_IRQ
#  include "sam_pio.h"
#endif

#include "chip.h"
#include "mmu.h"
#include "cp15_cacheops.h"
#include "sctlr.h"
#include "hardware/sam_aic.h"
#include "hardware/sam_matrix.h"
#include "hardware/sam_aximx.h"
#include "hardware/sam_sfr.h"

#include "sam_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private types
 ****************************************************************************/

typedef uint32_t *(*doirq_t)(int irq, uint32_t *regs);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Symbols defined via the linker script */

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_srctype[SCRTYPE_NTYPES] =
{
  0, 0, 1, 1, 2, 3
};

/* This is an array of bit maps that can be used to quickly determine is the
 * peripheral identified by its PID is served by H64MX or H32MX.  Then the
 * appropriate MATRIX SPSELR register can be consulted to determine if the
 * peripheral interrupts are secured or not.
 */

#if defined(CONFIG_SAMA5_SAIC)
static const uint32_t g_h64mxpids[3] =
{
  H64MX_SPSELR0_PIDS, H64MX_SPSELR1_PIDS, H64MX_SPSELR2_PIDS
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_dumpaic
 *
 * Description:
 *   Dump some interesting AIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void sam_dumpaic(const char *msg, uintptr_t base, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();
  irqinfo("AIC (%s, base=%08x irq=%d):\n", msg, base, irq);

  /* Select the register set associated with this irq */

  putreg32(irq, base + SAM_AIC_SSR_OFFSET);

  /* Then dump all of the (readable) register contents */

  irqinfo("  SSR: %08x  SMR: %08x  SVR: %08x  IVR: %08x\n",
          getreg32(base + SAM_AIC_SSR_OFFSET),
          getreg32(base + SAM_AIC_SMR_OFFSET),
          getreg32(base + SAM_AIC_SVR_OFFSET),
          getreg32(base + SAM_AIC_IVR_OFFSET));
  irqinfo("  FVR: %08x  ISR: %08x\n",
          getreg32(base + SAM_AIC_FVR_OFFSET),
          getreg32(base + SAM_AIC_ISR_OFFSET));
  irqinfo("  IPR: %08x       %08x       %08x       %08x\n",
          getreg32(base + SAM_AIC_IPR0_OFFSET),
          getreg32(base + SAM_AIC_IPR1_OFFSET),
          getreg32(base + SAM_AIC_IPR2_OFFSET),
          getreg32(base + SAM_AIC_IPR3_OFFSET));

  /* SAMA5D4 does not have the FFSR register */

#if defined(SAM_AIC_FFSR)
  irqinfo("  IMR: %08x CISR: %08x  SPU: %08x FFSR: %08x\n",
          getreg32(base + SAM_AIC_IMR_OFFSET),
          getreg32(base + SAM_AIC_CISR_OFFSET),
          getreg32(base + SAM_AIC_SPU_OFFSET),
          getreg32(base + SAM_AIC_FFSR_OFFSET));
#else
  irqinfo("  IMR: %08x CISR: %08x  SPU: %08x\n",
          getreg32(base + SAM_AIC_IMR_OFFSET),
          getreg32(base + SAM_AIC_CISR_OFFSET),
          getreg32(base + SAM_AIC_SPU_OFFSET));
#endif

  irqinfo("  DCR: %08x WPMR: %08x WPSR: %08x\n",
          getreg32(base + SAM_AIC_DCR_OFFSET),
          getreg32(base + SAM_AIC_WPMR_OFFSET),
          getreg32(base + SAM_AIC_WPSR_OFFSET));

  leave_critical_section(flags);
}
#else
#  define sam_dumpaic(msg, base, irq)
#endif

/****************************************************************************
 * Name: sam_vectorsize
 *
 * Description:
 *   Return the size of the vector data
 *
 ****************************************************************************/

static inline size_t sam_vectorsize(void)
{
  uintptr_t src;
  uintptr_t end;

  src  = (uintptr_t)&_vector_start;
  end  = (uintptr_t)&_vector_end;

  return (size_t)(end - src);
}

/****************************************************************************
 * Name: sam_spurious
 *
 * Description:
 *   Spurious interrupt handler.
 *
 *   Paragraph 17.8.6, Spurious Interrupt: "The Advanced Interrupt Controller
 *   features protection against spurious interrupts. A spurious interrupt is
 *   defined as being the assertion of an interrupt source long enough for
 *   the AIC to assert the nIRQ, but no longer present when AIC_IVR is read.
 *   This is most prone to occur when:
 *
 *     o An external interrupt source is programmed in level-sensitive mode
 *       and an active level occurs for only a short time.
 *     o An internal interrupt source is programmed in level sensitive and
 *       the output signal of the corresponding embedded peripheral is
 *       activated for a short time. (As in the case for the Watchdog.)
 *     o An interrupt occurs just a few cycles before the software begins to
 *       mask it, thus resulting in a pulse on the interrupt source.
 *
 *   "The AIC detects a spurious interrupt at the time the AIC_IVR is read
 *   while no enabled interrupt source is pending. When this happens, the AIC
 *   returns the value stored by the programmer in AIC_SPU (Spurious Vector
 *   Register). The programmer must store the address of a spurious interrupt
 *   handler in AIC_SPU as part of the application, to enable an as fast as
 *   possible return to the normal execution flow. This handler writes in
 *   AIC_EOICR and performs a return from interrupt."
 *
 ****************************************************************************/

static uint32_t *sam_spurious(int irq, uint32_t *regs)
{
  /* This is probably irrelevant since true vectored interrupts are not used
   * in this implementation.  The value of AIC_IVR is ignored.
   */

#if defined(CONFIG_DEBUG_IRQ_INFO)
  irqinfo("Spurious interrupt: IRQ %d\n", irq);
#endif
  return regs;
}

/****************************************************************************
 * Name: sam_fiqhandler
 *
 * Description:
 *   Default FIQ interrupt handler.
 *
 ****************************************************************************/

static uint32_t *sam_fiqhandler(int irq, uint32_t *regs)
{
  /* Dispatch the FIQ */

  return arm_doirq(SAM_IRQ_FIQ, regs);
}

/****************************************************************************
 * Name: sam_aic_issecure
 *
 * Description:
 *   Return true if the peripheral secure.
 *
 * Input Parameters:
 *   PID = IRQ number
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_SAIC)
static bool sam_aic_issecure(uint32_t irq)
{
  uintptr_t regaddr;
  uint32_t bit;
  unsigned int regndx;

  /* Get the register index and bit mask */

  regndx = (irq >> 5);
  bit    = ((uint32_t)1 << (irq & 0x1f));

  /* Get the SPSELR register address */

  DEBUGASSERT(regndx < 3);
  if ((g_h64mxpids[regndx] & bit) != 0)
    {
      /* H64MX.  Use Matrix 0 */

      regaddr = SAM_MATRIX0_SPSELR(regndx);
    }
  else
    {
      /* H32MX.  Use Matrix 1 */

      regaddr = SAM_MATRIX1_SPSELR(regndx);
    }

  /* Return true if the bit corresponding to this IRQ is zero */

  return (getreg32(regaddr) & bit) == 0;
}
#endif

/****************************************************************************
 * Name: sam_aic_redirection
 *
 * Description:
 *   Redirect all interrupts to the AIC.  This function is only compiled if
 *   (1) the architecture supports an SAIC (CONFIG_SAMA5_HAVE_SAIC), but (2)
 *   Use of the SAIC has not been selected (!CONFIG_SAMA5_SAIC).
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_HAVE_SAIC) && !defined(CONFIG_SAMA5_SAIC)
static void sam_aic_redirection(void)
{
  unsigned int regval;

  /* Check if interrupts are already redirected to the AIC */

  regval = getreg32(SAM_SFR_AICREDIR);
  if ((regval & SFR_AICREDIR_ENABLE) == 0)
    {
      /* Enable redirection of all interrupts to the AIC */

      regval  = getreg32(SAM_SFR_SN1);
      regval ^= SFR_AICREDIR_KEY;
      regval |= SFR_AICREDIR_ENABLE;
      putreg32(regval, SAM_SFR_AICREDIR);

#if defined(CONFIG_DEBUG_IRQ_INFO)
      /* Check if redirection was successfully enabled */

      regval = getreg32(SAM_SFR_AICREDIR);
      irqinfo("Interrupts %s redirected to the AIC\n",
              (regval & SFR_AICREDIR_ENABLE) != 0 ? "ARE" : "NOT");
#endif
    }
}
#else
#  define sam_aic_redirection()
#endif

/****************************************************************************
 * Name: sam_aic_initialize
 *
 * Description:
 *   Initialize the AIC or the SAIC.
 *
 ****************************************************************************/

static void sam_aic_initialize(uintptr_t base)
{
  int i;

  /* Unprotect SMR, SVR, SPU and DCR register */

  putreg32(AIC_WPMR_WPKEY, base + SAM_AIC_WPMR_OFFSET);

  /* Configure the FIQ and the IRQs. */

  for (i = 0; i < SAM_IRQ_NINT; i++)
    {
      /* Select the interrupt registers */

      putreg32(i, base + SAM_AIC_SSR_OFFSET);

      /* Disable the interrupt */

      putreg32(AIC_IDCR_INTD, base + SAM_AIC_IDCR_OFFSET);

      /* Set the (unused) FIQ/IRQ handler */

      if (i == SAM_PID_FIQ)
        {
          putreg32((uint32_t)sam_fiqhandler, base + SAM_AIC_SVR_OFFSET);
        }
      else
        {
          putreg32((uint32_t)arm_doirq, base + SAM_AIC_SVR_OFFSET);
        }

      /* Set the default interrupt priority */

      putreg32(SAM_DEFAULT_PRIOR, base + SAM_AIC_SMR_OFFSET);

      /* Clear any pending interrupt */

      putreg32(AIC_ICCR_INTCLR, base + SAM_AIC_ICCR_OFFSET);
    }

  /* Set the (unused) spurious interrupt handler */

  putreg32((uint32_t)sam_spurious, base + SAM_AIC_SPU_OFFSET);

  /* Configure debug register */

  putreg32(AIC_DCR_PROT, base + SAM_AIC_DCR_OFFSET);

  /* Perform 8 interrupt acknowledgements by writing any value to the
   * EOICR register.
   */

  for (i = 0; i < 8; i++)
    {
      putreg32(AIC_EOICR_ENDIT, base + SAM_AIC_EOICR_OFFSET);
    }

  /* Restore protection and the interrupt state */

  putreg32(AIC_WPMR_WPKEY | AIC_WPMR_WPEN, base + SAM_AIC_WPMR_OFFSET);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   This function is called by up_initialize() during the bring-up of the
 *   system.  It is the responsibility of this function to but the interrupt
 *   subsystem into the working and ready state.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
#if defined(CONFIG_SAMA5_BOOT_ISRAM) || defined(CONFIG_SAMA5_BOOT_CS0FLASH)
  size_t vectorsize;
#endif

  /* The following operations need to be atomic, but since this function is
   * called early in the initialization sequence, we expect to have exclusive
   * access to the AIC.
   */

  /* Redirect all interrupts to the AIC if so configured */

  sam_aic_redirection();

  /* Initialize the Advanced Interrupt Controller (AIC) */

  sam_aic_initialize(SAM_AIC_VBASE);

#if defined(CONFIG_SAMA5_SAIC)
  /* Initialize the Secure Advanced Interrupt Controller (SAIC) */

  sam_aic_initialize(SAM_SAIC_VBASE);

#endif

#if defined(CONFIG_ARCH_LOWVECTORS)
  /* If CONFIG_ARCH_LOWVECTORS is defined, then the vectors located at the
   * beginning of the .text region must appear at address at the address
   * specified in the VBAR.  There are three ways to accomplish this:
   *
   *   1. By explicitly mapping the beginning of .text region with a page
   *      table entry so that the virtual address zero maps to the beginning
   *      of the .text region.  VBAR == 0x0000:0000.
   *
   *   2. A second way is to map the use the AXI MATRIX remap register to
   *      map physical address zero to the beginning of the text region,
   *      either internal SRAM or EBI CS 0.  Then we can set an identity
   *      mapping to map the boot region at 0x0000:0000 to virtual address
   *      0x0000:00000.   VBAR == 0x0000:0000.
   *
   *      This method is used when booting from ISRAM or NOR FLASH.  In
   *      that case, vectors must lie at the beginning of NOFR FLASH.
   *
   *   3. Set the Cortex-A5 VBAR register so that the vector table address
   *      is moved to a location other than 0x0000:0000.
   *
   *      This is the method used when booting from SDRAM.
   *
   * - When executing from NOR FLASH, the first level bootloader is supposed
   *   to provide the AXI MATRIX mapping for us at boot time base on the
   *   state of the BMS pin.  However, I have found that in the test
   *   environments that I use, I cannot always be assured of that physical
   *   address mapping.
   *
   * - If we are executing out of ISRAM, then the SAMA5 primary bootloader
   *   probably copied us into ISRAM and set the AXI REMAP bit for us.
   *
   * - If we are executing from external SDRAM, then a secondary bootloader
   *   must have loaded us into SDRAM.  In this case, simply set the VBAR
   *   register to the address of the vector table (not necessary at the
   *   beginning or SDRAM).
   */

#if defined(CONFIG_SAMA5_BOOT_ISRAM) || defined(CONFIG_SAMA5_BOOT_CS0FLASH)
  /* Set the vector base address register to 0x0000:0000 */

  cp15_wrvbar(0);

#if 0 /* Disabled on reset */
  /* Disable MATRIX write protection */

  putreg32(MATRIX_WPMR_WPKEY, SAM_MATRIX_WPMR);
#endif

  /* Set remap state 0 if we are running from internal SRAM or from SDRAM.
   * If we booted into NOR FLASH, then the first level bootloader should
   * have already provided this mapping for us.
   *
   * This is done late in the boot sequence.  Any exceptions taken before
   * this point in time will be handled by the ROM code, not by the NuttX
   * interrupt since which was, up to this point, uninitialized.
   *
   *   Boot state:    ROM is seen at address 0x00000000
   *   Remap State 0: SRAM is seen at address 0x00000000 (through AHB slave
   *                  interface) instead of ROM.
   *   Remap State 1: HEBI is seen at address 0x00000000 (through AHB slave
   *                  interface) instead of ROM for external boot.
   *
   * Here we are assuming that vectors reside in the lower end of ISRAM.
   * Hmmm... this probably does not matter since we will map a page to
   * address 0x0000:0000 in that case anyway.
   */

#ifdef ATSAMA5D3
  putreg32(MATRIX_MRCR_RCB0, SAM_MATRIX_MRCR);   /* Enable Cortex-A5 remap */
#endif

#if defined(CONFIG_SAMA5_BOOT_ISRAM)
  putreg32(AXIMX_REMAP_REMAP0, SAM_AXIMX_REMAP); /* Remap SRAM */
#elif defined(ATSAMA5D3) /* && defined(CONFIG_SAMA5_BOOT_CS0FLASH) */
  putreg32(AXIMX_REMAP_REMAP1, SAM_AXIMX_REMAP); /* Remap NOR FLASH on CS0 */
#endif

  /* Make sure that there is no trace of any previous mapping (here we
   * that the L2 cache has not yet been enabled.
   */

  vectorsize = sam_vectorsize();
  cp15_invalidate_icache();
  cp15_invalidate_dcache(0, vectorsize);
  mmu_invalidate_region(0, vectorsize);

#if 0 /* Disabled on reset */
  /* Restore MATRIX write protection */

  putreg32(MATRIX_WPMR_WPKEY | MATRIX_WPMR_WPEN, SAM_MATRIX_WPMR);
#endif

#elif defined(CONFIG_SAMA5_BOOT_SDRAM)
  /* Set the VBAR register to the address of the vector table in SDRAM */

  DEBUGASSERT((((uintptr_t)&_vector_start) & ~VBAR_MASK) == 0);
  cp15_wrvbar((uint32_t)&_vector_start);

#endif /* CONFIG_SAMA5_BOOT_ISRAM || CONFIG_SAMA5_BOOT_CS0FLASH */
#endif /* CONFIG_ARCH_LOWVECTORS */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Initialize logic to support a second level of interrupt decoding for
   * PIO pins.
   */

#ifdef CONFIG_SAMA5_PIO_IRQ
  sam_pioirqinitialize();
#endif

  /* And finally, enable interrupts */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: arm_decodeirq, arm_decodefiq (and sam_decodeirq helper).
 *
 * Description:
 *   This function is called from the IRQ vector handler in arm_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call arm_doirq to dispatch
 *   the interrupt.
 *
 *  Input Parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

static uint32_t *sam_decodeirq(uintptr_t base, uint32_t *regs)
{
  uint32_t irqid;
  uint32_t ivr;

  /* Paragraph 17.8.5 Protect Mode: "The Protect Mode permits reading the
   *   Interrupt Vector Register without performing the associated automatic
   *   operations. ... Writing PROT in AIC_DCR (Debug Control Register) at
   *   0x1 enables the Protect Mode.
   *
   *  "When the Protect Mode is enabled, the AIC performs interrupt stacking
   *    only when a write access is performed on the AIC_IVR. Therefore, the
   *    Interrupt Service Routines must write (arbitrary data) to the AIC_IVR
   *    just after reading it. The new context of the AIC, including the
   *    value of the Interrupt Status Register (AIC_ISR), is updated with the
   *    current interrupt only when AIC_IVR is written. ..."
   *
   *  "To summarize, in normal operating mode, the read of AIC_IVR performs
   *   the following operations within the AIC:
   *
   *   1. Calculates active interrupt (higher than current or spurious).
   *   2. Determines and returns the vector of the active interrupt.
   *   3. Memorizes the interrupt.
   *   4. Pushes the current priority level onto the internal stack.
   *   5. Acknowledges the interrupt.
   *
   * "However, while the Protect Mode is activated, only operations 1 to 3
   *  are performed when AIC_IVR is read.  Operations 4 and 5 are only
   *  performed by the AIC when AIC_IVR is written.
   *
   * "Software that has been written and debugged using the Protect Mode
   *  runs correctly in Normal Mode without modification. However, in Normal
   *  Mode the AIC_IVR write has no effect and can be removed to optimize the
   *  code.
   */

  /* Write in the IVR to support Protect Mode */

  ivr = getreg32(base + SAM_AIC_IVR_OFFSET);
  putreg32(ivr, base + SAM_AIC_IVR_OFFSET);

  /* Get the IRQ number from the interrupt status register.  NOTE that the
   * IRQ number is the same is the peripheral ID (PID).
   */

  irqid = getreg32(base + SAM_AIC_ISR_OFFSET) & AIC_ISR_MASK;

  /* Dispatch the interrupt */

  regs = ((doirq_t)ivr)((int)irqid, regs);

  /* Acknowledge interrupt */

  putreg32(AIC_EOICR_ENDIT, base + SAM_AIC_EOICR_OFFSET);
  return regs;
}

/* This is the entry point from the ARM IRQ vector handler */

uint32_t *arm_decodeirq(uint32_t *regs)
{
  return sam_decodeirq(SAM_AIC_VBASE, regs);
}

#if defined(CONFIG_SAMA5_SAIC)
/* This is the entry point from the ARM FIQ vector handler */

uint32_t *arm_decodefiq(uint32_t *regs)
{
  uint32_t *ret;

  /* In order to distinguish a FIQ from a true secure interrupt we need to
   * check the state of the FIQ line in the SAIC_CISR register.
   */

  if ((getreg32(SAM_SAIC_CISR) & AIC_CISR_NFIQ) != 0)
    {
      /* Handle the FIQ */

      ret = arm_doirq(SAM_IRQ_FIQ, regs);

      /* Acknowledge interrupt */

      putreg32(AIC_EOICR_ENDIT, SAM_SAIC_EOICR);
    }
  else
    {
      /* Handle the IRQ */

      ret = sam_decodeirq(SAM_SAIC_VBASE, regs);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: up_disable_irq (and sam_disable_irq helper)
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

static void sam_disable_irq(uintptr_t base, int irq)
{
  irqstate_t flags;

  if (irq < SAM_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = enter_critical_section();

      /* Select the register set associated with this irq */

      putreg32(irq, base + SAM_AIC_SSR_OFFSET);

      /* Disable the interrupt */

      putreg32(AIC_IDCR_INTD, base + SAM_AIC_IDCR_OFFSET);
      sam_dumpaic("disable", base, irq);
      leave_critical_section(flags);
    }
#ifdef CONFIG_SAMA5_PIO_IRQ
  else
    {
      /* Maybe it is a (derived) PIO IRQ */

      sam_pioirqdisable(irq);
    }
#endif
}

void up_disable_irq(int irq)
{
#if defined(CONFIG_SAMA5_SAIC)
  if (sam_aic_issecure(irq))
    {
      sam_disable_irq(SAM_SAIC_VBASE, irq);
    }
  else
#endif
    {
      sam_disable_irq(SAM_AIC_VBASE, irq);
    }
}

/****************************************************************************
 * Name: up_enable_irq (and sam_enable_irq helper)
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

static void sam_enable_irq(uintptr_t base, int irq)
{
  irqstate_t flags;

  if (irq < SAM_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = enter_critical_section();

      /* Select the register set associated with this irq */

      putreg32(irq, base + SAM_AIC_SSR_OFFSET);

      /* Enable the interrupt */

      putreg32(AIC_IECR_INTEN, base + SAM_AIC_IECR_OFFSET);
      sam_dumpaic("enable", base, irq);
      leave_critical_section(flags);
    }
#ifdef CONFIG_SAMA5_PIO_IRQ
  else
    {
      /* Maybe it is a (derived) PIO IRQ */

      sam_pioirqenable(irq);
    }
#endif
}

void up_enable_irq(int irq)
{
#if defined(CONFIG_SAMA5_SAIC)
  if (sam_aic_issecure(irq))
    {
      sam_enable_irq(SAM_SAIC_VBASE, irq);
    }
  else
#endif
    {
      sam_enable_irq(SAM_AIC_VBASE, irq);
    }
}

/****************************************************************************
 * Name: up_prioritize_irq (and sam_prioritize_irq helper)
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
static int sam_prioritize_irq(uint32_t base, int irq, int priority)
{
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(irq < SAM_IRQ_NINT &&
             (unsigned)priority <= AIC_SMR_PRIOR_MASK);
  if (irq < SAM_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = enter_critical_section();

      /* Select the register set associated with this irq */

      putreg32(irq, base + SAM_AIC_SSR_OFFSET);

      /* Unprotect and write the SMR register */

      putreg32(AIC_WPMR_WPKEY, base + SAM_AIC_WPMR_OFFSET);

      /* Set the new priority, preserving the current srctype */

      regval  = getreg32(base + SAM_AIC_SMR_OFFSET);
      regval &= ~AIC_SMR_PRIOR_MASK;
      regval |= (uint32_t)priority << AIC_SMR_PRIOR_SHIFT;
      putreg32(regval, base + SAM_AIC_SMR_OFFSET);

      /* Restore protection and the interrupt state */

      putreg32(AIC_WPMR_WPKEY | AIC_WPMR_WPEN, base + SAM_AIC_WPMR_OFFSET);
      sam_dumpaic("prioritize", base, irq);
      leave_critical_section(flags);
    }

  return OK;
}

int up_prioritize_irq(int irq, int priority)
{
#if defined(CONFIG_SAMA5_SAIC)
  if (sam_aic_issecure(irq))
    {
      sam_prioritize_irq(SAM_SAIC_VBASE, irq, priority);
    }
  else
#endif
    {
      sam_prioritize_irq(SAM_AIC_VBASE, irq, priority);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: sam_irq_srctype (and _sam_irq_srctype helper)
 *
 * Description:
 *   irq     - Identifies the IRQ source to be configured
 *   srctype - IRQ source configuration
 *
 ****************************************************************************/

static void _sam_irq_srctype(uintptr_t base, int irq,
                             enum sam_srctype_e srctype)
{
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(irq < SAM_IRQ_NINT && (unsigned)srctype < SCRTYPE_NTYPES);

  /* These operations must be atomic */

  flags = enter_critical_section();

  /* Select the register set associated with this irq */

  putreg32(irq, base + SAM_AIC_SSR_OFFSET);

  /* Unprotect and write the SMR register */

  putreg32(AIC_WPMR_WPKEY, base + SAM_AIC_WPMR_OFFSET);

  /* Set the new srctype, preserving the current priority */

  regval  = getreg32(base + SAM_AIC_SMR_OFFSET);
  regval &= ~AIC_SMR_SRCTYPE_MASK;
  regval |= (uint32_t)g_srctype[srctype] << AIC_SMR_SRCTYPE_SHIFT;
  putreg32(regval, base + SAM_AIC_SMR_OFFSET);

  /* Restore protection and the interrupt state */

  putreg32(AIC_WPMR_WPKEY | AIC_WPMR_WPEN, base + SAM_AIC_WPMR_OFFSET);
  sam_dumpaic("srctype", base, irq);
  leave_critical_section(flags);
}

void sam_irq_srctype(int irq, enum sam_srctype_e srctype)
{
#if defined(CONFIG_SAMA5_SAIC)
  if (sam_aic_issecure(irq))
    {
      _sam_irq_srctype(SAM_SAIC_VBASE, irq, srctype);
    }
  else
#endif
    {
      _sam_irq_srctype(SAM_AIC_VBASE, irq, srctype);
    }
}
