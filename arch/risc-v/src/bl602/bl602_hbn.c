/****************************************************************************
 * arch/risc-v/src/bl602/bl602_hbn.c
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

#include "nuttx/arch.h"
#include "hardware/bl602_hbn.h"
#include "bl602_hbn.h"
#include "bl602_rtc.h"
#include "riscv_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bl602_hbn_info_s
{
  bl602_hbn_cb_t out0_callback[3];
  void *out0_arg[3];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bl602_hbn_info_s g_bl602_hbn_info;

/****************************************************************************
 * Name: bl602_hbn_irq
 ****************************************************************************/

static int bl602_hbn_irq(int irq, void *context, void *p_arg)
{
  bl602_hbn_cb_t isr_cb;
  void *arg;

  /* GPIO7 GPIO8 and RTC */

  if (bl602_hbn_get_int_state(BL602_HBN_INT_GPIO7))
    {
      /* gpio7 sync/async mode */

      bl602_hbn_clear_irq(BL602_HBN_INT_GPIO7);

      isr_cb = g_bl602_hbn_info.out0_callback[BL602_HBN_OUT0_INT_GPIO7];
      arg = g_bl602_hbn_info.out0_arg[BL602_HBN_OUT0_INT_GPIO7];

      if (isr_cb)
        {
          isr_cb(arg);
        }
    }

  if (bl602_hbn_get_int_state(BL602_HBN_INT_GPIO8))
    {
      /* gpio8 sync/async mode */

      bl602_hbn_clear_irq(BL602_HBN_INT_GPIO8);

      isr_cb = g_bl602_hbn_info.out0_callback[BL602_HBN_OUT0_INT_GPIO8];
      arg = g_bl602_hbn_info.out0_arg[BL602_HBN_OUT0_INT_GPIO8];

      if (isr_cb)
        {
          isr_cb(arg);
        }
    }

  if (bl602_hbn_get_int_state(BL602_HBN_INT_RTC))
    {
      bl602_hbn_clear_irq(BL602_HBN_INT_RTC);
      bl602_hbn_clear_rtc_int();

      isr_cb = g_bl602_hbn_info.out0_callback[BL602_HBN_OUT0_INT_RTC];
      arg = g_bl602_hbn_info.out0_arg[BL602_HBN_OUT0_INT_RTC];

      if (isr_cb)
        {
          isr_cb(arg);
        }
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_hbn_set_uart_clk_sel
 *
 * Description:
 *   Select uart clock source.
 *
 * Input Parameters:
 *   clk_sel: uart clock type selection, 0 for FCLK or 1 for 160MHz CLK
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

void bl602_set_uart_clk_sel(int clk_sel)
{
  if (clk_sel)
    {
      modifyreg32(BL602_HBN_GLB, 0, HBN_GLB_HBN_UART_CLK_SEL);
    }
  else
    {
      modifyreg32(BL602_HBN_GLB, HBN_GLB_HBN_UART_CLK_SEL, 0);
    }
}

void bl602_aon_pad_iesmt_cfg(uint8_t pad_cfg)
{
  modifyreg32(BL602_HBN_IRQ_MODE, HBN_IRQ_MODE_REG_AON_PAD_IE_SMT,
      pad_cfg << 8);
}

/****************************************************************************
 * Name: bl602_hbn_get_int_state
 *
 * Description:
 *   HBN get interrupt status.
 *
 * Input Parameters:
 *   irq_type: HBN interrupt type
 *
 * Returned Value:
 *   true or false
 *
 ****************************************************************************/

bool bl602_hbn_get_int_state(uint8_t irq_type)
{
  uint32_t tmp_val;

  /* Check the parameters */

  tmp_val = getreg32(BL602_HBN_IRQ_STAT);

  if (tmp_val & (1 << irq_type))
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: bl602_hbn_clear_irq
 *
 * Description:
 *   HBN clear interrupt status.
 *
 * Input Parameters:
 *   hbn_int_type: HBN interrupt type
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl602_hbn_clear_irq(uint8_t hbn_int_type)
{
  modifyreg32(BL602_HBN_IRQ_CLR, 0, 1 << hbn_int_type);
  modifyreg32(BL602_HBN_IRQ_CLR, 1 << hbn_int_type, 0);
}

/****************************************************************************
 * Name: bl602_hbn_out0_int_register
 *
 * Description:
 *   HBN out0 interrupt cllback register.
 *
 * Input Parameters:
 *   irq_type: HBN interrupt type
 *   isr_cb: callback
 *   arg: callback arg
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int bl602_hbn_out0_int_register(uint8_t irq_type, bl602_hbn_cb_t isr_cb,
                                void *arg)
{
  irqstate_t flags;

  if (irq_type > 2)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  g_bl602_hbn_info.out0_callback[irq_type] = isr_cb;
  g_bl602_hbn_info.out0_arg[irq_type] = arg;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: bl602_hbn_out0_int_unregister
 *
 * Description:
 *   HBN out0 interrupt cllback unregister.
 *
 * Input Parameters:
 *   irq_type: HBN interrupt type
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int bl602_hbn_out0_int_unregister(uint8_t irq_type)
{
  irqstate_t flags;

  if (irq_type > 2)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  g_bl602_hbn_info.out0_callback[irq_type] = NULL;
  g_bl602_hbn_info.out0_arg[irq_type] = NULL;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: bl602_hbn_out0_int_enable
 *
 * Description:
 *   HBN out0 interrupt enable.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_out0_int_enable(void)
{
  irq_attach(BL602_IRQ_HBN_OUT0, bl602_hbn_irq, NULL);
  up_enable_irq(BL602_IRQ_HBN_OUT0);
}

/****************************************************************************
 * Name: bl602_hbn_out0_int_disable
 *
 * Description:
 *   HBN out0 interrupt disable.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_out0_int_disable(void)
{
  irq_detach(BL602_IRQ_HBN_OUT0);
  up_disable_irq(BL602_IRQ_HBN_OUT0);
}
