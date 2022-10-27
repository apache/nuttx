/****************************************************************************
 * arch/sparc/src/s698pm/s698pm.h
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

#ifndef __ARCH_SPARC_SRC_S698PM_S698PM_H
#define __ARCH_SARRC_SRC_S698PM_S698PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "s698pm-config.h"
#include <stdint.h>
#include <stdbool.h>
#include "s698pm_irq.h"
#ifdef CONFIG_S698PM_GPIOIRQ
#include <nuttx/irq.h>
#include "s698pm_exti.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following defines the bits in Memory Configuration Register 1. */

#define S698PM_MEMORY_CONFIGURATION_PROM_SIZE_MASK  0x0003C000

/* The following defines the bits in Memory Configuration Register 1. */

#define S698PM_MEMORY_CONFIGURATION_RAM_SIZE_MASK  0x00001E00

/* The following defines the bits in the Timer Control Register. */

/* 1 = enable counting
 * 0 = hold scalar and counter
 */

#define S698PM_REG_TIMER_CONTROL_EN    0x00000001

/* 1 = reload at 0
 * 0 = stop at 0
 */

#define S698PM_REG_TIMER_CONTROL_RL    0x00000002

/* 1 = load counter
 * 0 = no function
 */

#define S698PM_REG_TIMER_CONTROL_LD    0x00000004

/* The following defines the bits in the UART Control Registers. */

#define S698PM_REG_UartCtrlRTD  0x000000FF /* RX/TX data */

/* The following defines the bits in the S698PM UART Status Registers. */

#define S698PM_REG_UART_STATUS_CLR  0x00000000 /* Clear all status bits */
#define S698PM_REG_UART_STATUS_DR   0x00000001 /* Data Ready */
#define S698PM_REG_UART_STATUS_TSE  0x00000002 /* TX Send Register Empty */
#define S698PM_REG_UART_STATUS_THE  0x00000004 /* TX Hold Register Empty */
#define S698PM_REG_UART_STATUS_BR   0x00000008 /* Break Error */
#define S698PM_REG_UART_STATUS_OE   0x00000010 /* RX Overrun Error */
#define S698PM_REG_UART_STATUS_PE   0x00000020 /* RX Parity Error */
#define S698PM_REG_UART_STATUS_FE   0x00000040 /* RX Framing Error */
#define S698PM_REG_UART_STATUS_ERR  0x00000078 /* Error Mask */

/* The following defines the bits in the S698PM UART Status Registers. */

#define S698PM_REG_UART_CTRL_RE     0x00000001 /* Receiver enable */
#define S698PM_REG_UART_CTRL_TE     0x00000002 /* Transmitter enable */
#define S698PM_REG_UART_CTRL_RI     0x00000004 /* Receiver interrupt enable */
#define S698PM_REG_UART_CTRL_TI     0x00000008 /* Transmitter interrupt enable */
#define S698PM_REG_UART_CTRL_PS     0x00000010 /* Parity select */
#define S698PM_REG_UART_CTRL_PE     0x00000020 /* Parity enable */
#define S698PM_REG_UART_CTRL_FL     0x00000040 /* Flow control enable */
#define S698PM_REG_UART_CTRL_LB     0x00000080 /* Loop Back enable */

#ifndef __ASSEMBLY__

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SMP
/* SPARC requires at least a 8-byte stack alignment  */

#define SMP_STACK_ALIGNMENT  8
#define SMP_STACK_MASK       7
#define SMP_STACK_SIZE       ((CONFIG_IDLETHREAD_STACKSIZE + 7) & ~7)
#define SMP_STACK_WORDS      (SMP_STACK_SIZE >> 2)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t g_cpu0_idlestack[SMP_STACK_WORDS];
#if CONFIG_SMP_NCPUS > 1
extern uint32_t g_cpu1_idlestack[SMP_STACK_WORDS];
#if CONFIG_SMP_NCPUS > 2
extern uint32_t g_cpu2_idlestack[SMP_STACK_WORDS];
#if CONFIG_SMP_NCPUS > 3
extern uint32_t g_cpu3_idlestack[SMP_STACK_WORDS];
#if CONFIG_SMP_NCPUS > 4
#  error This logic needs to extended for CONFIG_SMP_NCPUS > 4
#endif /* CONFIG_SMP_NCPUS > 4 */
#endif /* CONFIG_SMP_NCPUS > 3 */
#endif /* CONFIG_SMP_NCPUS > 2 */
#endif /* CONFIG_SMP_NCPUS > 1 */
#endif

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Read CPU ID(LEON3)
 ****************************************************************************/

#define LEON3_READ_CPUID(reg)   \
    RD      %asr17 , reg;       \
    SRL     reg    , 28  , reg;

/****************************************************************************
 * Read CPU ID (Now only support LEON3, LEON4!)
 ****************************************************************************/

#ifdef CONFIG_SMP
#define READ_CPUID(reg) LEON3_READ_CPUID(reg)
#else
#define READ_CPUID(reg) MOV     %g0 , reg
#endif

/*  Trap Types for on-chip peripherals
 *
 *  Source: Table 8 - Interrupt Trap Type and Default Priority Assignments
 *
 *  NOTE: The priority level for each source corresponds to the least
 *        significant nibble of the trap type.
 */

#define S698PM_TRAP_TYPE( _source ) SPARC_ASYNCHRONOUS_TRAP((_source) + 0x10)

#define S698PM_TRAP_SOURCE( _trap ) ((_trap) - 0x10)

#define S698PM_INT_TRAP( _trap ) \
  ( (_trap) >= S698PM_TRAP_TYPE( S698PM_IRQ_CORRECTABLE_MEMORY_ERROR ) && \
    (_trap) <= S698PM_TRAP_TYPE( S698PM_IRQ_EMPTY6 ) )

static __inline__ int bsp_irq_fixup(int irq)
{
       return irq;
}

/*  Macros to manipulate the Interrupt Clear, Interrupt Force, Interrupt Mask
 *  and the Interrupt Pending Registers.
 *
 *  NOTE: For operations which are not atomic, this code disables interrupts
 *        to guarantee there are no intervening accesses to the same register
 *        The operations which read the register, modify the value and then
 *        store the result back are vulnerable.
 */

#define S698PM_Clear_interrupt( _source ) \
  do { \
    S698PM_REG.Interrupt_Clear = (1 << (_source)); \
  } while (0)

#define S698PM_Force_interrupt( _source ) \
  do { \
    S698PM_REG.Interrupt_Force = (1 << (_source)); \
  } while (0)

#define S698PM_Is_interrupt_pending( _source ) \
  (S698PM_REG.Interrupt_Pending & (1 << (_source)))

#define S698PM_Is_interrupt_masked( _source ) \
  (!(S698PM_REG.Interrupt_Mask & (1 << (_source))))

#define S698PM_Mask_interrupt( _source ) \
  do { \
    uint32_t _level; \
    \
    _level = sparc_disable_interrupts(); \
      S698PM_REG.Interrupt_Mask &= ~(1 << (_source)); \
    sparc_enable_interrupts( _level ); \
  } while (0)

#define S698PM_Unmask_interrupt( _source ) \
  do { \
    uint32_t _level; \
    \
    _level = sparc_disable_interrupts(); \
      S698PM_REG.Interrupt_Mask |= (1 << (_source)); \
    sparc_enable_interrupts( _level ); \
  } while (0)

#define S698PM_Disable_interrupt( _source, _previous ) \
  do { \
    uint32_t _level; \
    uint32_t _mask = 1 << (_source); \
    \
    _level = sparc_disable_interrupts(); \
      (_previous) = S698PM_REG.Interrupt_Mask; \
      S698PM_REG.Interrupt_Mask = _previous & ~_mask; \
    sparc_enable_interrupts( _level ); \
    (_previous) &= _mask; \
  } while (0)

#define S698PM_Restore_interrupt( _source, _previous ) \
  do { \
    uint32_t _level; \
    uint32_t _mask = 1 << (_source); \
    \
    _level = sparc_disable_interrupts(); \
      S698PM_REG.Interrupt_Mask = \
        (S698PM_REG.Interrupt_Mask & ~_mask) | (_previous); \
    sparc_enable_interrupts( _level ); \
  } while (0)

/*  Each timer control register is organized as follows:
 *
 *    D0 - Enable
 *          1 = enable counting
 *          0 = hold scaler and counter
 *
 *    D1 - Counter Reload
 *          1 = reload counter at zero and restart
 *          0 = stop counter at zero
 *
 *    D2 - Counter Load
 *          1 = load counter with preset value
 *          0 = no function
 *
 */

#define S698PM_REG_TIMER_COUNTER_RELOAD_AT_ZERO     0x00000002
#define S698PM_REG_TIMER_COUNTER_STOP_AT_ZERO       0x00000000

#define S698PM_REG_TIMER_COUNTER_LOAD_COUNTER       0x00000004

#define S698PM_REG_TIMER_COUNTER_ENABLE_COUNTING    0x00000001
#define S698PM_REG_TIMER_COUNTER_DISABLE_COUNTING   0x00000000

#define S698PM_REG_TIMER_COUNTER_RELOAD_MASK        0x00000002
#define S698PM_REG_TIMER_COUNTER_ENABLE_MASK        0x00000001

#define S698PM_REG_TIMER_COUNTER_DEFINED_MASK       0x00000003
#define S698PM_REG_TIMER_COUNTER_CURRENT_MODE_MASK  0x00000003

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Load 32-bit word by forcing a cache-miss */

static inline unsigned int s698pm_r32_no_cache(uintptr_t addr)
{
  unsigned int tmp;
  __asm__ volatile (" lda [%1] 1, %0\n" : "=r"(tmp) : "r"(addr));
  return tmp;
}

/****************************************************************************
 * Name: up_clkinit
 *
 * Description:
 *   Initialiaze clock/PLL settings per the definitions in the board.h file.
 *
 ****************************************************************************/

void up_clkinitialize(void);

/****************************************************************************
 * Name: s698pm_consoleinit
 *
 * Description:
 *   Performs low level initialization of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot
 *   sequence.
 *
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
void s698pm_consoleinit(void);
#else
#  define s698pm_consoleinit()
#endif

/****************************************************************************
 * Name: s698pm_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void s698pm_uartreset(uintptr_t uart_base);
#endif

/****************************************************************************
 * Name: s698pm_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void s698pm_uartconfigure(uintptr_t uart_base, uint32_t baudrate,
                        unsigned int parity, unsigned int nbits, bool stop2);
#endif

/****************************************************************************
 * Name: s698pm_boardinitialize
 *
 * Description:
 *   This function must be provided by the board-specific logic in the
 *   directory boards/sparc/s698pm/<board-name>/src.
 *
 ****************************************************************************/

void s698pm_boardinitialize(void);

/****************************************************************************
 * Name: gpio_irqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 * Assumptions:
 *   Called during the early boot sequence before global interrupts have
 *   been enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SPARC_GPIOIRQ
void weak_function gpio_irqinitialize(void);
#endif

/****************************************************************************
 * Name: gpio_irqattach
 *
 * Description:
 *   Attach in GPIO interrupt to the provide 'isr'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 ****************************************************************************/

#ifdef CONFIG_SPARC_GPIOIRQ
int gpio_irqattach(int irq, xcpt_t newisr, xcpt_t *oldisr);
#endif

/****************************************************************************
 * Name: gpio_irqenable
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'irq'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 ****************************************************************************/

#ifdef CONFIG_SPARC_GPIOIRQ
void gpio_irqenable(int irq);
#endif

/****************************************************************************
 * Name: gpio_irqdisable
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'irq'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 ****************************************************************************/

#ifdef CONFIG_SPARC_GPIOIRQ
void gpio_irqdisable(int irq);
#endif

/****************************************************************************
 * Name: s698pm_pause_handler
 *
 * Description:
 *   Inter-CPU interrupt handler
 *
 * Input Parameters:
 *   Standard interrupt handler inputs
 *
 * Returned Value:
 *   Should always return OK
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
int s698pm_pause_handler(int irq, void *c, void *arg);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_SPARC_SRC_S698PM_S698PM_H */
