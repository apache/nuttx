/****************************************************************************
 * arch/arm/src/samd5e5/sam_irq.c
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
#include <arch/armv7-m/nvicpri.h>

#include "nvic.h"
#include "ram_vectors.h"
#include "arm_internal.h"

#ifdef CONFIG_SAMD5E5_EIC
#  include "sam_eic.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
   NVIC_SYSH_PRIORITY_DEFAULT)

/* Given the address of a NVIC ENABLE register, this is the offset to
 * the corresponding CLEAR ENABLE register.
 */

#define NVIC_ENA_OFFSET    (0)
#define NVIC_CLRENA_OFFSET (NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];

/* This is the address of the  exception vector table (determined by the
 * linker script).
 */

extern uint32_t _vectors[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void sam_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();

  irqinfo("NVIC (%s, irq=%d):\n", msg, irq);
  irqinfo("  INTCTRL:    %08x VECTAB:  %08x\n",
          getreg32(NVIC_INTCTRL), getreg32(NVIC_VECTAB));
#if 0
  irqinfo("  SYSH ENABLE MEMFAULT: %08x BUSFAULT: %08x USGFAULT: %08x "
          "SYSTICK: %08x\n",
          getreg32(NVIC_SYSHCON_MEMFAULTENA),
          getreg32(NVIC_SYSHCON_BUSFAULTENA),
          getreg32(NVIC_SYSHCON_USGFAULTENA),
          getreg32(NVIC_SYSTICK_CTRL_ENABLE));
#endif
  irqinfo("  IRQ ENABLE: %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_31_ENABLE),
          getreg32(NVIC_IRQ32_63_ENABLE),
          getreg32(NVIC_IRQ64_95_ENABLE));
#if SAM_IRQ_NEXTINT > 95
  irqinfo("            : %08x %08x %08x\n",
          getreg32(NVIC_IRQ96_127_ENABLE),
          getreg32(NVIC_IRQ128_159_ENABLE),
          getreg32(NVIC_IRQ160_191_ENABLE));
#endif
#if SAM_IRQ_NEXTINT > 191
#  warning Missing logic
#endif

  irqinfo("  SYSH_PRIO:  %08x %08x %08x\n",
          getreg32(NVIC_SYSH4_7_PRIORITY),
          getreg32(NVIC_SYSH8_11_PRIORITY),
          getreg32(NVIC_SYSH12_15_PRIORITY));
  irqinfo("  IRQ PRIO:   %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_3_PRIORITY),
          getreg32(NVIC_IRQ4_7_PRIORITY),
          getreg32(NVIC_IRQ8_11_PRIORITY),
          getreg32(NVIC_IRQ12_15_PRIORITY));
#if SAM_IRQ_NEXTINT > 15
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ16_19_PRIORITY),
          getreg32(NVIC_IRQ20_23_PRIORITY),
          getreg32(NVIC_IRQ24_27_PRIORITY),
          getreg32(NVIC_IRQ28_31_PRIORITY));
#endif
#if SAM_IRQ_NEXTINT > 31
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ32_35_PRIORITY),
          getreg32(NVIC_IRQ36_39_PRIORITY),
          getreg32(NVIC_IRQ40_43_PRIORITY),
          getreg32(NVIC_IRQ44_47_PRIORITY));
#endif
#if SAM_IRQ_NEXTINT > 47
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ48_51_PRIORITY),
          getreg32(NVIC_IRQ52_55_PRIORITY),
          getreg32(NVIC_IRQ56_59_PRIORITY),
          getreg32(NVIC_IRQ60_63_PRIORITY));
#endif
#if SAM_IRQ_NEXTINT > 63
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ64_67_PRIORITY),
          getreg32(NVIC_IRQ68_71_PRIORITY),
          getreg32(NVIC_IRQ72_75_PRIORITY),
          getreg32(NVIC_IRQ76_79_PRIORITY));
#endif
#if SAM_IRQ_NEXTINT > 79
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ80_83_PRIORITY),
          getreg32(NVIC_IRQ84_87_PRIORITY),
          getreg32(NVIC_IRQ88_91_PRIORITY),
          getreg32(NVIC_IRQ92_95_PRIORITY));
#endif
#if SAM_IRQ_NEXTINT > 95
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ96_99_PRIORITY),
          getreg32(NVIC_IRQ100_103_PRIORITY),
          getreg32(NVIC_IRQ104_107_PRIORITY),
          getreg32(NVIC_IRQ108_111_PRIORITY));
#endif
#if SAM_IRQ_NEXTINT > 111
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ112_115_PRIORITY),
          getreg32(NVIC_IRQ116_119_PRIORITY),
          getreg32(NVIC_IRQ120_123_PRIORITY),
          getreg32(NVIC_IRQ124_127_PRIORITY));
#endif
#if SAM_IRQ_NEXTINT > 127
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ128_131_PRIORITY),
          getreg32(NVIC_IRQ132_135_PRIORITY),
          getreg32(NVIC_IRQ136_139_PRIORITY),
          getreg32(NVIC_IRQ140_143_PRIORITY));
#endif
#if SAM_IRQ_NEXTINT > 143
#  warning Missing logic
#endif

  leave_critical_section(flags);
}
#else
#  define sam_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: sam_nmi, sam_busfault, sam_usagefault, sam_pendsv, sam_dbgmonitor,
 *       sam_pendsv, sam_reserved
 *
 * Description:
 *   Handlers for various exceptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int sam_nmi(int irq, FAR void *context, FAR void *arg)
{
  up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int sam_busfault(int irq, FAR void *context, FAR void *arg)
{
  up_irq_save();
  _err("PANIC!!! Bus fault received: %08x\n", getreg32(NVIC_CFAULTS));
  PANIC();
  return 0;
}

static int sam_usagefault(int irq, FAR void *context, FAR void *arg)
{
  up_irq_save();
  _err("PANIC!!! Usage fault received: %08x\n", getreg32(NVIC_CFAULTS));
  PANIC();
  return 0;
}

static int sam_pendsv(int irq, FAR void *context, FAR void *arg)
{
  up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int sam_dbgmonitor(int irq, FAR void *context, FAR void *arg)
{
  up_irq_save();
  _err("PANIC!!! Debug Monitor received\n");
  PANIC();
  return 0;
}

static int sam_reserved(int irq, FAR void *context, FAR void *arg)
{
  up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: sam_prioritize_syscall
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_USEBASEPRI
static inline void sam_prioritize_syscall(int priority)
{
  uint32_t regval;

  /* SVCALL is system handler 11 */

  regval  = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}
#endif

/****************************************************************************
 * Name: sam_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int sam_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                       uintptr_t offset)
{
  unsigned int extint = irq - SAM_IRQ_EXTINT;

  DEBUGASSERT(irq >= SAM_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= SAM_IRQ_EXTINT)
    {
#if SAM_IRQ_NEXTINT <= 32
      if (extint < SAM_IRQ_NEXTINT)
        {
           *regaddr = (NVIC_IRQ0_31_ENABLE + offset);
           *bit     = 1 << extint;
        }
      else
#elif SAM_IRQ_NEXTINT <= 64
      if (extint < 32)
        {
           *regaddr = (NVIC_IRQ0_31_ENABLE + offset);
           *bit     = 1 << extint;
        }
      else if (extint < SAM_IRQ_NEXTINT)
        {
           *regaddr = (NVIC_IRQ32_63_ENABLE + offset);
           *bit     = 1 << (extint - 32);
        }
      else
#elif SAM_IRQ_NEXTINT <= 96
      if (extint < 32)
        {
           *regaddr = (NVIC_IRQ0_31_ENABLE + offset);
           *bit     = 1 << extint;
        }
      else if (extint < 64)
        {
           *regaddr = (NVIC_IRQ32_63_ENABLE + offset);
           *bit     = 1 << (extint - 32);
        }
      else if (extint < SAM_IRQ_NEXTINT)
        {
           *regaddr = (NVIC_IRQ64_95_ENABLE + offset);
           *bit     = 1 << (extint - 64);
        }
      else
#elif SAM_IRQ_NEXTINT <= 128
      if (extint < 32)
        {
           *regaddr = (NVIC_IRQ0_31_ENABLE + offset);
           *bit     = 1 << extint;
        }
      else if (extint < 64)
        {
           *regaddr = (NVIC_IRQ32_63_ENABLE + offset);
           *bit     = 1 << (extint - 32);
        }
      else if (extint < 96)
        {
           *regaddr = (NVIC_IRQ64_95_ENABLE + offset);
           *bit     = 1 << (extint - 64);
        }
      else if (extint < SAM_IRQ_NEXTINT)
        {
           *regaddr = (NVIC_IRQ96_127_ENABLE + offset);
           *bit     = 1 << (extint - 96);
        }
      else
#elif SAM_IRQ_NEXTINT <= 160
      if (extint < 32)
        {
           *regaddr = (NVIC_IRQ0_31_ENABLE + offset);
           *bit     = 1 << extint;
        }
      else if (extint < 64)
        {
           *regaddr = (NVIC_IRQ32_63_ENABLE + offset);
           *bit     = 1 << (extint - 32);
        }
      else if (extint < 96)
        {
           *regaddr = (NVIC_IRQ64_95_ENABLE + offset);
           *bit     = 1 << (extint - 64);
        }
      else if (extint < 128)
        {
           *regaddr = (NVIC_IRQ96_127_ENABLE + offset);
           *bit     = 1 << (extint - 96);
        }
      else if (extint < SAM_IRQ_NEXTINT)
        {
           *regaddr = (NVIC_IRQ128_159_ENABLE + offset);
           *bit     = 1 << (extint - 128);
        }
      else
#else
#  warning Missing logic
#endif
        {
          return ERROR; /* Invalid interrupt */
        }
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
      *regaddr = NVIC_SYSHCON;
      if (irq == SAM_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == SAM_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == SAM_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == SAM_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return ERROR; /* Invalid or unsupported exception */
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uintptr_t regaddr;
#if defined(CONFIG_DEBUG_SYMBOLS) && !defined(CONFIG_ARMV7M_USEBASEPRI)
  uint32_t regval;
#endif
  int nintlines;
  int i;

  /* The NVIC ICTR register (bits 0-4) holds the number of interrupt
   * lines that the NVIC supports, defined in groups of 32. That is,
   * the total number of interrupt lines is up to (32*(INTLINESNUM+1)).
   *
   *  0 -> 32 interrupt lines, 1 enable register,   8 priority registers
   *  1 -> 64 "       " "   ", 2 enable registers, 16 priority registers
   *  2 -> 96 "       " "   ", 3 enable registers, 24 priority registers
   *  ...
   */

  nintlines = (getreg32(NVIC_ICTR) & NVIC_ICTR_INTLINESNUM_MASK) + 1;

  /* Disable all interrupts.  There are nintlines interrupt enable
   * registers.
   */

  for (i = nintlines, regaddr = NVIC_IRQ0_31_CLEAR;
       i > 0;
       i--, regaddr += 4)
    {
      putreg32(0xffffffff, regaddr);
    }

  /* Make sure that we are using the correct vector table.  The default
   * vector address is 0x0000:0000 but if we are executing code that is
   * positioned in SRAM or in external FLASH, then we may need to reset
   * the interrupt vector so that it refers to the table in SRAM or in
   * external FLASH.
   */

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

#ifdef CONFIG_ARCH_RAMVECTORS
  /* If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
   * vector table that requires special initialization.
   */

  arm_ramvec_initialize();
#endif

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  /* Now set all of the interrupt lines to the default priority.  There are
   * nintlines * 8 priority registers.
   */

  for (i = (nintlines << 3), regaddr = NVIC_IRQ0_3_PRIORITY;
       i > 0;
       i--, regaddr += 4)
    {
      putreg32(DEFPRIORITY32, regaddr);
    }

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(SAM_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(SAM_IRQ_HARDFAULT, arm_hardfault, NULL);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO
  /* up_prioritize_irq(SAM_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */
#endif
#ifdef CONFIG_ARMV7M_USEBASEPRI
  sam_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARM_MPU
  irq_attach(SAM_IRQ_MEMFAULT, arm_memfault, NULL);
  up_enable_irq(SAM_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(SAM_IRQ_NMI, sam_nmi, NULL);
#ifndef CONFIG_ARM_MPU
  irq_attach(SAM_IRQ_MEMFAULT, arm_memfault, NULL);
#endif
  irq_attach(SAM_IRQ_BUSFAULT, sam_busfault, NULL);
  irq_attach(SAM_IRQ_USAGEFAULT, sam_usagefault, NULL);
  irq_attach(SAM_IRQ_PENDSV, sam_pendsv, NULL);
  irq_attach(SAM_IRQ_DBGMONITOR, sam_dbgmonitor, NULL);
  irq_attach(SAM_IRQ_RESERVED, sam_reserved, NULL);
#endif

  sam_dumpnvic("initial", SAM_IRQ_NIRQS);

  /* If a debugger is connected, try to prevent it from catching hardfaults.
   * If CONFIG_ARMV7M_USEBASEPRI, no hardfaults are expected in normal
   * operation.
   */

#if defined(CONFIG_DEBUG_SYMBOLS) && !defined(CONFIG_ARMV7M_USEBASEPRI)
  regval  = getreg32(NVIC_DEMCR);
  regval &= ~NVIC_DEMCR_VCHARDERR;
  putreg32(regval, NVIC_DEMCR);
#endif

#ifdef CONFIG_SAMD5E5_EIC
  /* Initialize the external interrupt controller. */

  sam_eic_initialize();
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable global interrupts */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (sam_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to disable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Clear Enable register.  For other exceptions, we need to
       * clear the bit in the System Handler Control and State Register.
       */

      if (irq >= SAM_IRQ_EXTINT)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval &= ~bit;
          putreg32(regval, regaddr);
        }
    }

#if 0 /* Might be useful in early bring-up */
  sam_dumpnvic("disable", irq);
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (sam_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to enable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Set Enable register.  For other exceptions, we need to
       * set the bit in the System Handler Control and State Register.
       */

      if (irq >= SAM_IRQ_EXTINT)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval |= bit;
          putreg32(regval, regaddr);
        }
    }

#if 0 /* Might be useful in early bring-up */
  sam_dumpnvic("enable", irq);
#endif
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq >= SAM_IRQ_MEMFAULT && irq < SAM_IRQ_NIRQS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < SAM_IRQ_EXTINT)
    {
      /* NVIC_SYSH_PRIORITY() maps {0..15} to one of three priority
       * registers (0-3 are invalid)
       */

      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq    -= 4;
    }
  else
    {
      /* NVIC_IRQ_PRIORITY() maps {0..} to one of many priority registers */

      irq    -= SAM_IRQ_EXTINT;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);

  sam_dumpnvic("prioritize", irq);
  return OK;
}
#endif
