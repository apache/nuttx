/****************************************************************************
 * arch/risc-v/include/gap8/irq.h
 * GAP8 event system
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 *  GAP8 features a FC controller and a 8-core cluster. IRQ from peripherals
 *  have unique ID, which are dispatched to the FC or cluster by the SOC
 *  event unit, and then by the FC event unit or cluster event unit, and
 *  finally to FC or cluster. Peripherals share the same IRQ entry.
 ****************************************************************************/

#ifndef __ARCH_RISC_V_INCLUDE_GAP8_IRQ_H
#define __ARCH_RISC_V_INCLUDE_GAP8_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Unique ID in SOC domain */

/* uDMA data events.
 *  Each peripheral has a uDMA_ID.
 *  Each peripheral also has RX and TX event ID, which happen to be 2*uDMA_ID
 *  and 2*uDMA_ID+1.
 */

#define GAP8_EVENT_UDMA_LVDS_RX              0
#define GAP8_EVENT_UDMA_LVDS_TX              1
#define GAP8_EVENT_UDMA_SPIM0_RX             2
#define GAP8_EVENT_UDMA_SPIM0_TX             3
#define GAP8_EVENT_UDMA_SPIM1_RX             4
#define GAP8_EVENT_UDMA_SPIM1_TX             5
#define GAP8_EVENT_UDMA_HYPERBUS_RX          6
#define GAP8_EVENT_UDMA_HYPERBUS_TX          7
#define GAP8_EVENT_UDMA_UART_RX              8
#define GAP8_EVENT_UDMA_UART_TX              9
#define GAP8_EVENT_UDMA_I2C0_RX              10
#define GAP8_EVENT_UDMA_I2C0_TX              11
#define GAP8_EVENT_UDMA_I2C1_RX              12
#define GAP8_EVENT_UDMA_I2C1_TX              13
#define GAP8_EVENT_UDMA_TCDM_RX              14
#define GAP8_EVENT_UDMA_TCDM_TX              15
#define GAP8_EVENT_UDMA_SAI_CH0              16
#define GAP8_EVENT_UDMA_SAI_CH1              17
#define GAP8_EVENT_UDMA_CPI_RX               18

#define GAP8_UDMA_MAX_EVENT                  18

/* Other events of uDMA peripherals */

#define GAP8_EVENT_LVDS_GEN0                 20
#define GAP8_EVENT_LVDS_GEN1                 21
#define GAP8_EVENT_SPIM0_EOT                 22
#define GAP8_EVENT_SPIM1_EOT                 23
#define GAP8_EVENT_HYPERBUS_RESERVED         24
#define GAP8_EVENT_UART_RESERVED             25
#define GAP8_EVENT_I2C0_ERROR                26
#define GAP8_EVENT_I2C1_ERROR                27
#define GAP8_EVENT_I2S_RESERVED              28
#define GAP8_EVENT_CAM_RESERVED              29

/* PMU events */

#define GAP8_EVENT_PMU_CLUSTER_POWER_ON      31
#define GAP8_EVENT_PMU_CLUSTER_RESERVED0     32
#define GAP8_EVENT_PMU_CLUSTER_RESERVED1     33
#define GAP8_EVENT_PMU_CLUSTER_RESERVED2     34
#define GAP8_EVENT_PMU_CLUSTER_CLOCK_GATING  35
#define GAP8_EVENT_PMU_DLC_BRIDGE_PICL_OK    36
#define GAP8_EVENT_PMU_DLC_BRIDGE_SCU_OK     37

/* Other SOC domain peripheral events */

#define GAP8_EVENT_PWM0                      38
#define GAP8_EVENT_PWM1                      39
#define GAP8_EVENT_PWM2                      40
#define GAP8_EVENT_PWM3                      41
#define GAP8_EVENT_GPIO                      42    /* GPIO group interrupt */
#define GAP8_EVENT_RTC_APB                   43
#define GAP8_EVENT_RTC                       44
#define GAP8_EVENT_RESERVED0                 45
#define GAP8_EVENT_RESERVED1                 46
#define GAP8_EVENT_RESERVED2                 47
#define GAP8_EVENT_SOC_SW_0                  48    /* GAP8 SOC SW Event0 */
#define GAP8_EVENT_SOC_SW_1                  49    /* GAP8 SOC SW Event1 */
#define GAP8_EVENT_SOC_SW_2                  50    /* GAP8 SOC SW Event2 */
#define GAP8_EVENT_SOC_SW_3                  51    /* GAP8 SOC SW Event3 */
#define GAP8_EVENT_SOC_SW_4                  52    /* GAP8 SOC SW Event4 */
#define GAP8_EVENT_SOC_SW_5                  53    /* GAP8 SOC SW Event5 */
#define GAP8_EVENT_SOC_SW_6                  54    /* GAP8 SOC SW Event6 */
#define GAP8_EVENT_SOC_SW_7                  55    /* GAP8 SOC SW Event7 */
#define GAP8_EVENT_REF32K_CLK_RISE           56    /* Reference 32K Clock event */

/* FC domain IRQ ID */

#define GAP8_IRQ_FC_SW_0      0
#define GAP8_IRQ_FC_SW_1      1
#define GAP8_IRQ_FC_SW_2      2
#define GAP8_IRQ_FC_SW_3      3
#define GAP8_IRQ_FC_SW_4      4
#define GAP8_IRQ_FC_SW_5      5
#define GAP8_IRQ_FC_SW_6      6
#define GAP8_IRQ_FC_SW_7      7
#define GAP8_IRQ_FC_TIMER_LO  10
#define GAP8_IRQ_FC_TIMER_HI  11
#define GAP8_IRQ_FC_UDMA      27
#define GAP8_IRQ_FC_MPU       28
#define GAP8_IRQ_FC_UDMA_ERR  29
#define GAP8_IRQ_FC_HP_0      30
#define GAP8_IRQ_FC_HP_1      31

#define GAP8_IRQ_RESERVED     60

/* Cluster domain IRQ ID */

/* TODO */

/* RISCY core exception vectors */

#define GAP8_IRQ_RST       32
#define GAP8_IRQ_ILLEGAL   33
#define GAP8_IRQ_SYSCALL   34

/* Total number of IRQs.
 * 32 ISRs + reset-handler + illegal-instruction-handler +
 * system-call-handler
 */

#define NR_IRQS 35

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* SOC_EU - SOC domain event unit */

typedef struct
{
  volatile uint32_t EVENT;           /* event register               */
  volatile uint32_t FC_MASK_MSB;     /* fc mask MSB register         */
  volatile uint32_t FC_MASK_LSB;     /* fc mask LSB register         */
  volatile uint32_t CL_MASK_MSB;     /* cluster mask MSB register    */
  volatile uint32_t CL_MASK_LSB;     /* cluster mask LSB register    */
  volatile uint32_t PR_MASK_MSB;     /* propagate mask MSB register  */
  volatile uint32_t PR_MASK_LSB;     /* propagate mask LSB register  */
  volatile uint32_t ERR_MASK_MSB;    /* error mask MSB register      */
  volatile uint32_t ERR_MASK_LSB;    /* error mask LSB register      */
  volatile uint32_t TIMER_SEL_HI;    /* timer high register          */
  volatile uint32_t TIMER_SEL_LO;    /* timer low register           */
} soc_eu_reg_t;

#define SOC_EU  ((soc_eu_reg_t *)0x1A106000U)

/* FCEU - FC domain event unit */

typedef struct
{
  volatile uint32_t MASK;               /* mask register                    */
  volatile uint32_t MASK_AND;           /* mask-and(clr) register           */
  volatile uint32_t MASK_OR;            /* mask-or(set) register            */
  volatile uint32_t MASK_IRQ;           /* irq mask register                */
  volatile uint32_t MASK_IRQ_AND;       /* irq mask-and(clr) register       */
  volatile uint32_t MASK_IRQ_OR;        /* irq mask-or(set) register        */
  volatile uint32_t STATUS;             /* clock Status register            */
  volatile uint32_t BUFFER;             /* irq pending register             */
  volatile uint32_t BUFFER_MASKED;      /* buffer masked register           */
  volatile uint32_t BUFFER_IRQ_MASKED;  /* buffer irq masked register       */
  volatile uint32_t BUFFER_CLEAR;       /* clear irq pending                */
  volatile uint32_t SW_EVENTS_MASK;     /* software event mask register     */
  volatile uint32_t SW_EVENTS_MASK_AND; /* software event mask and register */
  volatile uint32_t SW_EVENTS_MASK_OR;  /* software event mask or register  */
  volatile uint32_t EVENT_WAIT;         /* event wait register              */
  volatile uint32_t EVENT_WAIT_CLEAR;   /* event wait clear register        */
  volatile uint32_t MASK_SEC_IRQ;       /* mask sec irq register            */
} fceu_reg_t;

#define FCEU  ((fceu_reg_t*)0x00204000U)

/* Current interrupt event ID */

typedef struct
{
  volatile uint32_t CURRENT_EVENT;  /* current event register */
} soc_event_reg_t;

#define SOC_EVENTS ((soc_event_reg_t*)0x00200F00UL)

/* event trigger and mask */

typedef struct
{
  volatile uint32_t TRIGGER_SET[8];  /* trigger set register */
  volatile uint32_t _reserved0[8];   /* Offset: 0x20 (R/W)  Empty Registers */
  volatile uint32_t TRIGGER_WAIT[8]; /* trigger wait register */
  volatile uint32_t _reserved1[8];   /* Offset: 0x60 (R/W)  Empty Registers */
  volatile uint32_t TRIGGER_CLR[8];  /* trigger clear register */
} eu_sw_events_trigger_reg_t;

#define EU_SW_EVNT_TRIG ((eu_sw_events_trigger_reg_t*)0x00204100UL)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_disable_event
 *
 * Description:
 *   Disable the specific event. Note that setting 1 means to disable an
 *   event...
 *
 ****************************************************************************/

static inline void up_disable_event(int event)
{
  if (event >= 32)
    {
      SOC_EU->FC_MASK_MSB |= (1 << (event - 32));
    }
  else
    {
      SOC_EU->FC_MASK_LSB |= (1 << event);
    }
}

/****************************************************************************
 * Name: up_enable_event
 *
 * Description:
 *   Enable the specific event. Note that setting 0 means to enable an
 *   event...
 *
 ****************************************************************************/

static inline void up_enable_event(int event)
{
  if (event >= 32)
    {
      SOC_EU->FC_MASK_MSB &= ~(1 << (event - 32));
    }
  else
    {
      SOC_EU->FC_MASK_LSB &= ~(1 << event);
    }
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'. Mind the Machine privilege.
 *
 ****************************************************************************/

static inline void up_disable_irq(int irq)
{
  FCEU->MASK_IRQ_AND = (1UL << irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'. Mind the Machine privilege.
 *
 ****************************************************************************/

static inline void up_enable_irq(int irq)
{
  FCEU->MASK_IRQ_OR = (1 << irq);
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

static inline void up_ack_irq(int irq)
{
}

/****************************************************************************
 * Name: _current_privilege
 *
 * Description:
 *   Get the current privilege mode. 0x0 for user mode, and 0x3 for machine
 *   mode.
 *
 ****************************************************************************/

static inline uint32_t _current_privilege(void)
{
  uint32_t result;

  asm volatile ("csrr %0, 0xC10" : "=r" (result));

  return result;
}

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable interrupt and return the current interrupt state.
 *
 ****************************************************************************/

static inline uint32_t up_irq_save(void)
{
  uint32_t oldstat;
  uint32_t newstat;

  if (_current_privilege())
    {
      /* Machine mode: Unset MIE and UIE */

      asm volatile ("csrr %0, 0x300": "=r" (oldstat));
      newstat = oldstat & ~(0x9);
      asm volatile("csrw 0x300, %0" : /* no output */ : "r" (newstat));
    }
  else
    {
      /* User mode: Unset UIE */

      asm volatile ("csrr %0, 0x000": "=r" (oldstat));
      newstat = oldstat & ~(1L << 0);
      asm volatile("csrw 0x000, %0" : /* no output */ : "r" (newstat));
    }

  return oldstat;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore previous IRQ mask state
 *
 ****************************************************************************/

static inline void up_irq_restore(uint32_t pri)
{
  if (_current_privilege())
    {
      /* Machine mode - mstatus */

      asm volatile("csrw 0x300, %0" : /* no output */ : "r" (pri));
    }
  else
    {
      /* User mode - ustatus */

      asm volatile("csrw 0x000, %0" : /* no output */ : "r" (pri));
    }
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

static inline uint32_t up_irq_enable(void)
{
  uint32_t oldstat;
  uint32_t newstat;

  if (_current_privilege())
    {
      /* Machine mode: Set MIE and UIE */

      asm volatile ("csrr %0, 0x300": "=r" (oldstat));
      newstat = oldstat | (0x9);
      asm volatile("csrw 0x300, %0" : /* no output */ : "r" (newstat));
    }
  else
    {
      /* User mode: Set UIE */

      asm volatile ("csrr %0, 0x000": "=r" (oldstat));
      newstat = oldstat | (1L << 0);
      asm volatile("csrw 0x000, %0" : /* no output */ : "r" (newstat));
    }
  return oldstat;
}

/****************************************************************************
 * Name: gap8_sleep_wait_sw_evnt
 *
 * Description:
 *   Sleep on specific event.
 *
 ****************************************************************************/

static inline void gap8_sleep_wait_sw_evnt(uint32_t event_mask)
{
  FCEU->MASK_OR = event_mask;
  __builtin_pulp_event_unit_read((void *)&FCEU->EVENT_WAIT_CLEAR, 0);
  FCEU->MASK_AND = event_mask;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_RISC_V_INCLUDE_GAP8_IRQ_H */
