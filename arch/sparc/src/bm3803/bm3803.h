/****************************************************************************
 * arch/sparc/src/bm3803/bm3803.h
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

#ifndef __ARCH_SPARC_SRC_BM3803_BM3803_H
#define __ARCH_SARRC_SRC_BM3803_BM3803_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bm3803-config.h"

#ifdef CONFIG_BM3803_GPIOIRQ
#include <nuttx/irq.h>
#include "bm3803_exti.h"
#endif

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following defines the bits in Memory Configuration Register 1. */

#define BM3803_MEMORY_CONFIGURATION_PROM_SIZE_MASK  0x0003C000

/* The following defines the bits in Memory Configuration Register 1. */

#define BM3803_MEMORY_CONFIGURATION_RAM_SIZE_MASK  0x00001E00

/* The following defines the bits in the Timer Control Register. */

/* 1 = enable counting
 * 0 = hold scalar and counter
 */

#define BM3803_REG_TIMER_CONTROL_EN    0x00000001

/* 1 = reload at 0
 * 0 = stop at 0
 */

#define BM3803_REG_TIMER_CONTROL_RL    0x00000002

/* 1 = load counter
 * 0 = no function
 */

#define BM3803_REG_TIMER_CONTROL_LD    0x00000004

/* The following defines the bits in the UART Control Registers. */

#define BM3803_REG_UartCtrlRTD  0x000000FF /* RX/TX data */

/* The following defines the bits in the BM3803 UART Status Registers. */

#define BM3803_REG_UART_STATUS_CLR  0x00000000 /* Clear all status bits */
#define BM3803_REG_UART_STATUS_DR   0x00000001 /* Data Ready */
#define BM3803_REG_UART_STATUS_TSE  0x00000002 /* TX Send Register Empty */
#define BM3803_REG_UART_STATUS_THE  0x00000004 /* TX Hold Register Empty */
#define BM3803_REG_UART_STATUS_BR   0x00000008 /* Break Error */
#define BM3803_REG_UART_STATUS_OE   0x00000010 /* RX Overrun Error */
#define BM3803_REG_UART_STATUS_PE   0x00000020 /* RX Parity Error */
#define BM3803_REG_UART_STATUS_FE   0x00000040 /* RX Framing Error */
#define BM3803_REG_UART_STATUS_ERR  0x00000078 /* Error Mask */

/* The following defines the bits in the BM3803 UART Status Registers. */

#define BM3803_REG_UART_CTRL_RE     0x00000001 /* Receiver enable */
#define BM3803_REG_UART_CTRL_TE     0x00000002 /* Transmitter enable */
#define BM3803_REG_UART_CTRL_RI     0x00000004 /* Receiver interrupt enable */
#define BM3803_REG_UART_CTRL_TI     0x00000008 /* Transmitter interrupt enable */
#define BM3803_REG_UART_CTRL_PS     0x00000010 /* Parity select */
#define BM3803_REG_UART_CTRL_PE     0x00000020 /* Parity enable */
#define BM3803_REG_UART_CTRL_FL     0x00000040 /* Flow control enable */
#define BM3803_REG_UART_CTRL_LB     0x00000080 /* Loop Back enable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/*  The following constants are intended to be used ONLY in assembly
 *  language files.
 *
 *  NOTE:  The intended style of usage is to load the address of BM3803 REGS
 *         into a register and then use these as displacements from
 *         that register.
 */

#ifdef __ASSEMBLY__

#define  BM3803_REG_MEMCFG1_OFFSET                                  0x00
#define  BM3803_REG_MEMCFG2_OFFSET                                  0x04
#define  BM3803_REG_EDACCTRL_OFFSET                                 0x08
#define  BM3803_REG_FAILADDR_OFFSET                                 0x0C
#define  BM3803_REG_MEMSTATUS_OFFSET                                0x10
#define  BM3803_REG_CACHECTRL_OFFSET                                0x14
#define  BM3803_REG_POWERDOWN_OFFSET                                0x18
#define  BM3803_REG_WRITEPROT1_OFFSET                               0x1C
#define  BM3803_REG_WRITEPROT2_OFFSET                               0x20
#define  BM3803_REG_BM3803CONF_OFFSET                               0x24
#define  BM3803_REG_UNIMPLEMENTED_2_OFFSET                          0x28
#define  BM3803_REG_UNIMPLEMENTED_3_OFFSET                          0x2C
#define  BM3803_REG_UNIMPLEMENTED_4_OFFSET                          0x30
#define  BM3803_REG_UNIMPLEMENTED_5_OFFSET                          0x34
#define  BM3803_REG_UNIMPLEMENTED_6_OFFSET                          0x38
#define  BM3803_REG_UNIMPLEMENTED_7_OFFSET                          0x3C
#define  BM3803_REG_TIMERCNT1_OFFSET                                0x40
#define  BM3803_REG_TIMERLOAD1_OFFSET                               0x44
#define  BM3803_REG_TIMERCTRL1_OFFSET                               0x48
#define  BM3803_REG_WDOG_OFFSET                                     0x4C
#define  BM3803_REG_TIMERCNT2_OFFSET                                0x50
#define  BM3803_REG_TIMERLOAD2_OFFSET                               0x54
#define  BM3803_REG_TIMERCTRL2_OFFSET                               0x58
#define  BM3803_REG_UNIMPLEMENTED_8_OFFSET                          0x5C
#define  BM3803_REG_SCALERCNT_OFFSET                                0x60
#define  BM3803_REG_SCALER_LOAD_OFFSET                              0x64
#define  BM3803_REG_UNIMPLEMENTED_9_OFFSET                          0x68
#define  BM3803_REG_UNIMPLEMENTED_10_OFFSET                         0x6C
#define  BM3803_REG_UARTDATA1_OFFSET                                0x70
#define  BM3803_REG_UARTSTATUS1_OFFSET                              0x74
#define  BM3803_REG_UARTCTRL1_OFFSET                                0x78
#define  BM3803_REG_UARTSCALER1_OFFSET                              0x7C
#define  BM3803_REG_UARTDATA2_OFFSET                                0x80
#define  BM3803_REG_UARTSTATUS2_OFFSET                              0x84
#define  BM3803_REG_UARTCTRL2_OFFSET                                0x88
#define  BM3803_REG_UARTSCALER2_OFFSET                              0x8C
#define  BM3803_REG_IRQMASK_OFFSET                                  0x90
#define  BM3803_REG_IRQPEND_OFFSET                                  0x94
#define  BM3803_REG_IRQFORCE_OFFSET                                 0x98
#define  BM3803_REG_IRQCLEAR_OFFSET                                 0x9C
#define  BM3803_REG_PIODATA_OFFSET                                  0xA0
#define  BM3803_REG_PIODIR_OFFSET                                   0xA4
#define  BM3803_REG_PIOIRQ_OFFSET                                   0xA8
#define  BM3803_REG_SIM_RAM_SIZE_OFFSET                             0xF4
#define  BM3803_REG_SIM_ROM_SIZE_OFFSET                             0xF8

#else

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*  Trap Types for on-chip peripherals
 *
 *  Source: Table 8 - Interrupt Trap Type and Default Priority Assignments
 *
 *  NOTE: The priority level for each source corresponds to the least
 *        significant nibble of the trap type.
 */

#define BM3803_TRAP_TYPE( _source ) SPARC_ASYNCHRONOUS_TRAP((_source) + 0x10)

#define BM3803_TRAP_SOURCE( _trap ) ((_trap) - 0x10)

#define BM3803_INT_TRAP( _trap ) \
  ( (_trap) >= BM3803_TRAP_TYPE( BM3803_IRQ_CORRECTABLE_MEMORY_ERROR ) && \
    (_trap) <= BM3803_TRAP_TYPE( BM3803_IRQ_EMPTY6 ) )

/*  Structure for BM3803 memory mapped registers.
 *
 *  Source: Section 6.1 - On-chip registers
 *
 *  NOTE:  There is only one of these structures per CPU, its base address
 *         is 0x80000000, and the variable BM3803_REG is placed there by the
 *         linkcmds file.
 */

typedef struct
{
    volatile unsigned int mem_cfg1;      /* Memory Configuration Register 1 */
    volatile unsigned int mem_cfg2;      /* Memory Configuration Register 2 */
    volatile unsigned int mem_cfg3;      /* Memory Configuration Register 3 */
    volatile unsigned int fail_addr;

    volatile unsigned int mem_status;    /* 0x10 */
    volatile unsigned int cache_ctrl;    /* Cache control register */
    volatile unsigned int power_down;
    volatile unsigned int write_prot1;   /* Write protection 1 */

    volatile unsigned int write_prot2;   /* Write protection 2 0x20 */
    volatile unsigned int pcr;
    volatile unsigned int dummy2;
    volatile unsigned int dummy3;

    volatile unsigned int dummy4;         /* 0x30 */
    volatile unsigned int dummy5;
    volatile unsigned int dummy6;
    volatile unsigned int dummy7;

    volatile unsigned int timer_cnt1;     /* 0x40 */
    volatile unsigned int timer_load1;
    volatile unsigned int timer_ctrl1;
    volatile unsigned int wdog;

    volatile unsigned int timer_cnt2;   /* 0x50 */
    volatile unsigned int timer_load2;
    volatile unsigned int timer_ctrl2;
    volatile unsigned int dummy8;

    volatile unsigned int scaler_cnt;   /* 0x60 */
    volatile unsigned int scaler_load;
    volatile unsigned int dummy9;
    volatile unsigned int dummy10;

    volatile unsigned int uart_data1;   /* UA1 data */
    volatile unsigned int uart_status1; /* UA1 status register */
    volatile unsigned int uart_ctrl1;   /* UA1 control register */
    volatile unsigned int uart_scaler1; /* UA1 scaler register */

    volatile unsigned int uart_data2;   /* UA2 data */
    volatile unsigned int uart_status2; /* UA2 status register */
    volatile unsigned int uart_ctrl2;   /* UA2 control register */
    volatile unsigned int uart_scaler2; /* UA2 scaler register */

    /* Interrupt Mask and Priority Register register */

    volatile unsigned int int_mask;
    volatile unsigned int int_pend;  /* Interrupt Pending Register register */
    volatile unsigned int int_force;
    volatile unsigned int int_clear;   /* Interrupt Clear Register register */

    volatile unsigned int pio_data;     /* 0xA0 */
    volatile unsigned int pio_dir;
    volatile unsigned int pio_irq;
    volatile unsigned int dummy11;

    volatile unsigned int i_mask2;      /* 0xB0 */
    volatile unsigned int i_pend2;
    volatile unsigned int i_stat2;
    volatile unsigned int dummy12;

    volatile unsigned int dummy13;     /* 0xC0 */
    volatile unsigned int dcom_status;
    volatile unsigned int dcom_ctrl;
    volatile unsigned int dcom_scaler;

    volatile unsigned int pci_reset;     /* 0xD0 */
    volatile unsigned int dummy15;
    volatile unsigned int dummy16;
    volatile unsigned int dummy17;

    volatile unsigned int uart_data3;   /* 0xE0 */
    volatile unsigned int uart_status3;
    volatile unsigned int uart_ctrl3;
    volatile unsigned int uart_scaler3;

    volatile unsigned int reserved1;   /* 0xF0 */
    volatile unsigned int reserved2;
    volatile unsigned int reserved3;
    volatile unsigned int reserved4;

    volatile unsigned int mecfg1;      /* 0x100 */
    volatile unsigned int mecfg2;
    volatile unsigned int mecfg3;
    volatile unsigned int mecfg4;
} bm3803_register_map;

/*  This is used to manipulate the on-chip registers.
 *
 *  The following symbol must be defined in the linkcmds file and point
 *  to the correct location.
 */

extern bm3803_register_map BM3803_REG;

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

#define BM3803_Clear_interrupt( _source ) \
  do { \
    BM3803_REG.Interrupt_Clear = (1 << (_source)); \
  } while (0)

#define BM3803_Force_interrupt( _source ) \
  do { \
    BM3803_REG.Interrupt_Force = (1 << (_source)); \
  } while (0)

#define BM3803_Is_interrupt_pending( _source ) \
  (BM3803_REG.Interrupt_Pending & (1 << (_source)))

#define BM3803_Is_interrupt_masked( _source ) \
  (!(BM3803_REG.Interrupt_Mask & (1 << (_source))))

#define BM3803_Mask_interrupt( _source ) \
  do { \
    uint32_t _level; \
    \
    _level = sparc_disable_interrupts(); \
      BM3803_REG.Interrupt_Mask &= ~(1 << (_source)); \
    sparc_enable_interrupts( _level ); \
  } while (0)

#define BM3803_Unmask_interrupt( _source ) \
  do { \
    uint32_t _level; \
    \
    _level = sparc_disable_interrupts(); \
      BM3803_REG.Interrupt_Mask |= (1 << (_source)); \
    sparc_enable_interrupts( _level ); \
  } while (0)

#define BM3803_Disable_interrupt( _source, _previous ) \
  do { \
    uint32_t _level; \
    uint32_t _mask = 1 << (_source); \
    \
    _level = sparc_disable_interrupts(); \
      (_previous) = BM3803_REG.Interrupt_Mask; \
      BM3803_REG.Interrupt_Mask = _previous & ~_mask; \
    sparc_enable_interrupts( _level ); \
    (_previous) &= _mask; \
  } while (0)

#define BM3803_Restore_interrupt( _source, _previous ) \
  do { \
    uint32_t _level; \
    uint32_t _mask = 1 << (_source); \
    \
    _level = sparc_disable_interrupts(); \
      BM3803_REG.Interrupt_Mask = \
        (BM3803_REG.Interrupt_Mask & ~_mask) | (_previous); \
    sparc_enable_interrupts( _level ); \
  } while (0)

/* Make all SPARC BSPs have common macros for interrupt handling */
#define BSP_Clear_interrupt(_source) BM3803_Clear_interrupt(_source)
#define BSP_Force_interrupt(_source) BM3803_Force_interrupt(_source)
#define BSP_Is_interrupt_pending(_source) BM3803_Is_interrupt_pending(_source)
#define BSP_Is_interrupt_masked(_source) BM3803_Is_interrupt_masked(_source)
#define BSP_Unmask_interrupt(_source) BM3803_Unmask_interrupt(_source)
#define BSP_Mask_interrupt(_source) BM3803_Mask_interrupt(_source)
#define BSP_Disable_interrupt(_source, _previous) \
        BM3803_Disable_interrupt(_source, _prev)
#define BSP_Restore_interrupt(_source, _previous) \
        BM3803_Restore_interrupt(_source, _previous)

/* Make all SPARC BSPs have common macros for interrupt handling on any CPU */
#define BSP_Cpu_Is_interrupt_masked(_source, _cpu) \
        BSP_Is_interrupt_masked(_source)
#define BSP_Cpu_Unmask_interrupt(_source, _cpu) \
        BSP_Unmask_interrupt(_source)
#define BSP_Cpu_Mask_interrupt(_source, _cpu) \
        BSP_Mask_interrupt(_source)
#define BSP_Cpu_Disable_interrupt(_source, _previous, _cpu) \
        BSP_Disable_interrupt(_source, _prev)
#define BSP_Cpu_Restore_interrupt(_source, _previous, _cpu) \
        BSP_Cpu_Restore_interrupt(_source, _previous)

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

#define BM3803_REG_TIMER_COUNTER_RELOAD_AT_ZERO     0x00000002
#define BM3803_REG_TIMER_COUNTER_STOP_AT_ZERO       0x00000000

#define BM3803_REG_TIMER_COUNTER_LOAD_COUNTER       0x00000004

#define BM3803_REG_TIMER_COUNTER_ENABLE_COUNTING    0x00000001
#define BM3803_REG_TIMER_COUNTER_DISABLE_COUNTING   0x00000000

#define BM3803_REG_TIMER_COUNTER_RELOAD_MASK        0x00000002
#define BM3803_REG_TIMER_COUNTER_ENABLE_MASK        0x00000001

#define BM3803_REG_TIMER_COUNTER_DEFINED_MASK       0x00000003
#define BM3803_REG_TIMER_COUNTER_CURRENT_MODE_MASK  0x00000003

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Load 32-bit word by forcing a cache-miss */

static inline unsigned int bm3803_r32_no_cache(uintptr_t addr)
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
 * Name: usart0_reset and usart1_reset
 *
 * Description:
 *   Reset USART0 or USART1.
 *
 ****************************************************************************/

void usart1_reset(void);
void usart2_reset(void);
void usart3_reset(void);
/****************************************************************************
 * Name: usart0_configure and usart1_configure
 *
 * Description:
 *   Configure USART0 or USART1.
 *
 ****************************************************************************/

void uart1_configure(void);
void uart2_configure(void);
void uart3_configure(void);
/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the initialization sequence to configure the serial console
 *   uart (only).
 *
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
void up_consoleinit(void);
#else
#  define up_consoleinit()
#endif

/****************************************************************************
 * Name: bm3803_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void bm3803_uartconfigure(uintptr_t uart_base, uint32_t baudrate,
                        unsigned int parity, unsigned int nbits, bool stop2);
#endif

/****************************************************************************
 * Name: bm3803_boardinitialize
 *
 * Description:
 *   This function must be provided by the board-specific logic in the
 *   directory boards/sparc/bm3803/<board-name>/src.
 *
 ****************************************************************************/

void bm3803_boardinitialize(void);

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_SPARC_SRC_BM3803_BM3803_H */
