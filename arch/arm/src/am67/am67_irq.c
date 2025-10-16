/****************************************************************************
 * arch/arm/src/am67/am67_irq.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <assert.h>

#include "arm_internal.h"
#include "irq/irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define INTR_MAX_INTERRUPTS (512u)
#define INTR_MAX_PRIORITY   (16u)
#define INTRC_BASE_ADDR     (0x2fff0000u)

#define VIM_BIT_POS(j)      ((j) & 0x1fu)
#define VIM_IRQVEC          (0x18u)
#define VIM_ACTIRQ          (0x20u)
#define VIM_RAW(j)          (0x400u + ((((j) >> 5) & 0xfu) * 0x20u))
#define VIM_STS(j)          (0x404u + ((((j) >> 5) & 0xfu) * 0x20u))
#define VIM_INT_EN(j)       (0x408u + ((((j) >> 5) & 0xfu) * 0x20u))
#define VIM_INT_DIS(j)      (0x40cu + ((((j) >> 5) & 0xfu) * 0x20u))
#define VIM_INT_MAP(j)      (0x418u + ((((j) >> 5) & 0xfu) * 0x20u))
#define VIM_INT_TYPE(j)     (0x41cu + ((((j) >> 5) & 0xfu) * 0x20u))
#define VIM_INT_PRI(j)      (0x1000u + ((j) * 0x4u))
#define VIM_INT_VEC(j)      (0x2000u + ((j) * 0x4u))

#define INTR_SUCCESS        ((int32_t)0)
#define INTR_FAILURE        ((int32_t)-1)
#define INTR_TIMEOUT        ((int32_t)-2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int32_t intr_get_irq(uint32_t *int_num);
static void intr_ack_irq(uint32_t int_num);
static void intr_clear_irq(uint32_t int_num);
static void intr_set_irq_pri(uint32_t int_num, uint32_t priority);
static uint32_t intr_get_irq_vec_addr(void);
static void intr_set_irq_vec_addr(uint32_t int_num, uintptr_t vec_addr);

static void utils_data_and_instruction_barrier(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: intr_get_irq
 *
 * Description:
 *   Return the interrupt status corresponding to the given int_num.
 *
 ****************************************************************************/

static int32_t intr_get_irq(uint32_t *int_num)
{
  int32_t status = INTR_FAILURE;
  uint32_t value;

  *int_num = 0;

  value = getreg32(INTRC_BASE_ADDR + VIM_ACTIRQ);

  if ((value & 0x80000000u) != 0U)
    {
      *int_num = (value & (INTR_MAX_INTERRUPTS - 1U));
      status = INTR_SUCCESS;
    }

  return status;
}

/****************************************************************************
 * Name: intr_ack_irq
 *
 * Description:
 *   Acknowledge a specific interrupt number by writing to the interrupt
 *   controller's IRQVEC register.
 *
 ****************************************************************************/

static void intr_ack_irq(uint32_t int_num)
{
  putreg32(int_num, INTRC_BASE_ADDR + VIM_IRQVEC);
}

/****************************************************************************
 * Name: intr_clear_irq
 *
 * Description:
 *   Clear a specific interrupt by setting the corresponding bit in the
 *   interrupt status register.
 *
 ****************************************************************************/

static void intr_clear_irq(uint32_t int_num)
{
  uint32_t bit_pos;

  bit_pos = VIM_BIT_POS(int_num);

  putreg32((0x1u << bit_pos), INTRC_BASE_ADDR + VIM_STS(int_num));
}

/****************************************************************************
 * Name: intr_set_irq_pri
 *
 * Description:
 *   Set the priority level (0–15) for a specified interrupt.
 *
 ****************************************************************************/

static void intr_set_irq_pri(uint32_t int_num, uint32_t priority)
{
  putreg32((priority & 0xfu), INTRC_BASE_ADDR + VIM_INT_PRI(int_num));
}

/****************************************************************************
 * Name: intr_get_irq_vec_addr
 *
 * Description:
 *   Get the interrupt vector address for a specific interrupt number from
 *   the corresponding interrupt controller register.
 *
 ****************************************************************************/

static uint32_t intr_get_irq_vec_addr(void)
{
  return getreg32(INTRC_BASE_ADDR + VIM_IRQVEC);
}

/****************************************************************************
 * Name: intr_set_irq_vec_addr
 *
 * Description:
 *   Set the interrupt vector address for a specific interrupt number into
 *   the corresponding interrupt controller register.
 *
 ****************************************************************************/

static void intr_set_irq_vec_addr(uint32_t int_num, uintptr_t vec_addr)
{
  putreg32(((uint32_t)vec_addr & 0xfffffffcu),
           INTRC_BASE_ADDR + VIM_INT_VEC(int_num));
}

/****************************************************************************
 * Name: utils_data_and_instruction_barrier
 *
 * Description:
 *   Enforces CPU memory ordering by executing an Instruction Synchronization
 *   Barrier (ISB) followed by a Data Synchronization Barrier (DSB),
 *   ensuring all previous instructions complete and memory accesses are
 *   synchronized before continuing execution.
 *
 ****************************************************************************/

static void utils_data_and_instruction_barrier(void)
{
  __asm__ __volatile__(
    " isb"
    "\n\t"
    :
    :
    : "memory");
  __asm__ __volatile__(
    " dsb"
    "\n\t"
    :
    :
    : "memory");
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern int irq_unexpected_isr(int irq, FAR void *context, FAR void *arg);
extern uint32_t *arm_doirq(int irq, uint32_t *regs);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable a specific interrupt by setting the corresponding bit in the
 *   interrupt enable register after executing a barrier.
 *
 ****************************************************************************/

void up_enable_irq(int int_num)
{
  uint32_t bit_pos;

  utils_data_and_instruction_barrier();

  bit_pos = VIM_BIT_POS(int_num);

  putreg32((0x1u << bit_pos), INTRC_BASE_ADDR + VIM_INT_EN(int_num));
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable a specific interrupt by setting the corresponding bit in the
 *   interrupt disable register and then executing a barrier.
 *
 ****************************************************************************/

void up_disable_irq(int int_num)
{
  uint32_t bit_pos;

  bit_pos = VIM_BIT_POS(int_num);

  putreg32(((uint32_t)0x1 << bit_pos),
           INTRC_BASE_ADDR + VIM_INT_DIS(int_num));

  utils_data_and_instruction_barrier();
}

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
  int i;

  for (i = 0; i < INTR_MAX_INTERRUPTS; i++)
    {
      intr_set_irq_pri(i, 0xf);
      intr_set_irq_vec_addr((uint32_t)i, (uintptr_t)arm_vectorirq);
    }

  for (i = 0; i < INTR_MAX_INTERRUPTS / 32; i++)
    {
      /* Disable all interrupts. */

      putreg32(0xffffffffu, INTRC_BASE_ADDR + VIM_INT_DIS(i * 32));

      /* Clear all pending interrupts. */

      putreg32(0xffffffffu, INTRC_BASE_ADDR + VIM_STS(i * 32));

      /* Make all as level. */

      putreg32(0x0u, INTRC_BASE_ADDR + VIM_INT_TYPE(i * 32));

      /* Make all as IRQ. */

      putreg32(0x0u, INTRC_BASE_ADDR + VIM_INT_MAP(i * 32));
    }

  /* Have to read vec addr, sets other registers. */

  (void)intr_get_irq_vec_addr();
  intr_ack_irq(0);

  for (i = 0; i < NR_IRQS; i++)
    {
      irq_attach(i, irq_unexpected_isr, NULL);
    }

  arm_color_intstack();
  up_irq_enable();
}

/****************************************************************************
 * Name: arm_decodeirq
 *
 * Description:
 *   This function is called from the IRQ vector handler in arm_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call arm_doirq to dispatch
 *   the interrupt.
 *
 *  Input parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
  uint32_t intr_num;

  /* Have to read vec addr, sets other registers. */

  (void)intr_get_irq_vec_addr();

  if (intr_get_irq(&intr_num) == 0)
    {
      regs = arm_doirq(intr_num, regs);
    }

  intr_clear_irq(intr_num);
  intr_ack_irq(intr_num);

  return regs;
}
