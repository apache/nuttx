/****************************************************************************
 * arch/arm64/src/imx9/imx9_mu.c
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <sys/types.h>

#include "imx9_mu.h"
#include "arm64_arch.h"
#include "arm64_internal.h"
#include "hardware/imx95/imx95_memorymap.h"
#include "hardware/imx9_mu.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx9_mu_dev_s
{
  uintptr_t mubase;
  uint32_t  irq;
  imx9_mu_msg_callback_t msg_callback;
  imx9_mu_gpi_callback_t gpi_callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imx9_mu_dev_s g_mu2_dev =
{
  .mubase = IMX9_MU2_MUA_BASE,
  .irq = IMX9_IRQ_MU2_A
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int imx9_mu_interrupt(int irq, void *context, void *args)
{
  struct imx9_mu_dev_s *dev = args;

  uint32_t sr  = getreg32(IMX9_MU_SR(dev->mubase));
  uint32_t rsr = getreg32(IMX9_MU_RSR(dev->mubase));
  uint32_t gsr = getreg32(IMX9_MU_GSR(dev->mubase));

  ipcinfo("MU irq=%d, SR=0x%04x, RSR=0x%04x, GSR=0x%04x\n", irq, sr, rsr,
          gsr);

  if (sr & IMX9_MU_SR_RFP_FLAG)
    {
      for (int i = 0; i < IMX9_MU_RR_REGARRAY_SIZE; i++)
        {
          if (rsr & (1 << i))
            {
              uint32_t msg = imx95_mu_receive_msg_non_blocking(dev, i);

              if (dev->msg_callback)
                {
                  dev->msg_callback(i, msg, dev);
                }
            }
        }

      for (int i = 0; i < IMX9_NR_OF_GPI; i++)
        {
          if (gsr & (1 << i))
            {
              putreg32((1 << i), IMX9_MU_GSR(dev->mubase));

              if (dev->gpi_callback)
                {
                  dev->gpi_callback(i, dev);
                }
            }
        }
    }

  /* Unknown interrupt flag which occurs when A55 shuts down */

  if (sr & 0x80)
    {
      /* TODO how to clear this flag? A W1C doesn't seem to work.. */

      putreg32(0x80, IMX9_MU_SR(dev->mubase));
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx95_mu_init
 *
 * Description:
 *   Initialize mu
 *
 * Input Parameters:
 *   index   - The index of mu
 *
 * Returned Value:
 *   imx9_mu_dev_s struct is returned on success. NULL is returned on
 *   failure.
 *
 ****************************************************************************/

struct imx9_mu_dev_s *imx95_mu_init(int index)
{
  struct imx9_mu_dev_s *priv;

  if (index == 2)
    {
      priv = &g_mu2_dev;
    }

  else
    {
      return NULL;
    }

  irq_attach(priv->irq, imx9_mu_interrupt, priv);
  up_enable_irq(priv->irq);

  return priv;
}

/****************************************************************************
 * Name: imx95_mu_subscribe_msg
 *
 * Description:
 *  Subscribe msg, when the irq occur,the msg callback will be called.
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   msg_int_bitfield - Enable correspond bit receive irq
 *   callback         - The call back will called when the irq occur
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx95_mu_subscribe_msg(struct imx9_mu_dev_s *priv,
                            uint32_t msg_int_bitfield,
                            imx9_mu_msg_callback_t callback)
{
  priv->msg_callback = callback;
  putreg32(msg_int_bitfield & IMX9_MSG_INT_MASK, IMX9_MU_RCR(priv->mubase));
}

/****************************************************************************
 * Name: imx95_mu_subscribe_gpi
 *
 * Description:
 *  Subscribe msg, when the irq occur,the msg callback will be called.
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   gpi_int_enable   - Enable correspond bit general purpose irq
 *   callback         - The call back will called when the irq occur
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx95_mu_subscribe_gpi(struct imx9_mu_dev_s *priv,
                            uint32_t gpi_int_enable,
                            imx9_mu_gpi_callback_t callback)
{
  priv->gpi_callback = callback;
  putreg32(gpi_int_enable & IMX9_GPI_INT_MASK, IMX9_MU_GIER(priv->mubase));
}

/****************************************************************************
 * Name: imx95_mu_deinit
 *
 * Description:
 *   Deinit mu
 *
 * Input Parameters:
 *   priv             - The mu dev will be deinit
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx95_mu_deinit(struct imx9_mu_dev_s *priv)
{
  up_disable_irq(priv->irq);
}

/****************************************************************************
 * Name: imx95_mu_send_msg_non_blocking
 *
 * Description:
 *  When the mu is busy, will return err
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   reg_index        - Which one transmit reg to be used
 *   msg              - The msgto be send
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx95_mu_send_msg_non_blocking(struct imx9_mu_dev_s *priv,
                                   uint32_t reg_index, uint32_t msg)
{
  assert(reg_index < IMX9_MU_TR_REGARRAY_SIZE);

  ipcinfo("MU send msg nonblocking idx=%d, msg=%d\n", reg_index, msg);

  if ((getreg32(IMX9_MU_TSR(priv->mubase)) & (1UL << reg_index)) == 0UL)
    {
      return -EBUSY;
    }

  putreg32(msg, IMX9_MU_TR1(priv->mubase) + (reg_index * sizeof(uint32_t)));
  return OK;
}

/****************************************************************************
 * Name: imx95_mu_send_msg
 *
 * Description:
 *  Send msg blocking
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   reg_index        - Which one transmit reg to be used
 *   msg              - The msgto be send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx95_mu_send_msg(struct imx9_mu_dev_s *priv, uint32_t reg_index,
                       uint32_t msg)
{
  assert(reg_index < IMX9_MU_TR_REGARRAY_SIZE);

  ipcinfo("MU send msg idx=%d, msg=%d\n", reg_index, msg);

  /* Wait TX register to be empty. */

  while ((getreg32(IMX9_MU_TSR(priv->mubase)) & (1UL << reg_index)) == 0UL);

  putreg32(msg, IMX9_MU_TR1(priv->mubase) + reg_index * sizeof(uint32_t));
}

/****************************************************************************
 * Name: imx95_mu_has_received_msg
 *
 * Description:
 *   Check Mu if has msg
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   reg_index        - Which one receive reg to be used
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx95_mu_has_received_msg(struct imx9_mu_dev_s *priv, uint32_t reg_index)
{
  return getreg32(IMX9_MU_RSR(priv->mubase)) & (1UL << reg_index) ? 0 :
                  -ENODATA;
}

/****************************************************************************
 * Name: imx95_mu_receive_msg_non_blocking
 *
 * Description:
 *  Non block receive msg
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   reg_index        - Which one receive reg to be used
 *
 * Returned Value:
 *  The value of index receive reg
 *
 ****************************************************************************/

uint32_t imx95_mu_receive_msg_non_blocking(struct imx9_mu_dev_s *priv,
                                           uint32_t reg_index)
{
  assert(reg_index < IMX9_MU_RR_REGARRAY_SIZE);

  return getreg32(IMX9_MU_RR1(priv->mubase) + reg_index * sizeof(uint32_t));
}

/****************************************************************************
 * Name: imx95_mu_receive_msg
 *
 * Description:
 *   Block receive msg
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   reg_index        - Which one receive reg to be used
 *
 * Returned Value:
 *  The value of index receive reg
 *
 ****************************************************************************/

uint32_t imx95_mu_receive_msg(struct imx9_mu_dev_s *priv, uint32_t reg_index)
{
  assert(reg_index < IMX9_MU_RR_REGARRAY_SIZE);

  /* Wait RX register to be not empty. */

  while (imx95_mu_has_received_msg(priv, reg_index) == -ENODATA);

  return getreg32(IMX9_MU_RR1(priv->mubase) + reg_index * sizeof(uint32_t));
}

/****************************************************************************
 * Name: imx95_mu_trigger_interrupts
 *
 * Description:
 *  Send irq to MUB
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   interrupts       - The number of interrupts
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx95_mu_trigger_interrupts(struct imx9_mu_dev_s *priv,
                                uint32_t interrupts)
{
  int ret      = -ECOMM;
  uint32_t gcr = getreg32(IMX9_MU_GCR(priv->mubase));

  /* Only if previous interrupts has been accepted. */

  if ((gcr & interrupts) == 0)
    {
      putreg32(gcr | interrupts, IMX9_MU_GCR(priv->mubase));
      ret = OK;
    }

  return ret;
}
