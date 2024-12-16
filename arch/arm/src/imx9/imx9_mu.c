/****************************************************************************
 * arch/arm/src/imx9/imx9_mu.c
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

#include "imx9_mu.h"
#include "arm_internal.h"
#include "hardware/imx95/imx95_memorymap.h"
#include "hardware/imx9_mu.h"
#include <debug.h>
#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NR_OF_GPI    4
#define MSG_INT_MASK ((1 << IMX9_MU_RR_REGARRAY_SIZE) - 1)
#define GPI_INT_MASK ((1 << NR_OF_GPI) - 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx9_mudev_s
{
  uint32_t mubase;
  uint32_t irq;
  imx9_mu_msg_callback_t msg_callback;
  imx9_mu_gpi_callback_t gpi_callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_IMX9_MU5
static struct imx9_mudev_s g_mu5_dev = /* clang-format off */
{
  .mubase = IMX9_MU5_MUA_BASE,
  .irq = IMX9_IRQ_MU5_A
}; /* clang-format on */
#endif

#ifdef CONFIG_IMX9_MU7
static struct imx9_mudev_s g_mu7_dev = /* clang-format off */
{
  .mubase = IMX9_MU7_MUB_BASE,
  .irq = IMX9_IRQ_MU7_B
}; /* clang-format on */
#endif

#ifdef CONFIG_IMX9_MU8
static struct imx9_mudev_s g_mu8_dev = /* clang-format off */
{
  .mubase = IMX9_MU8_MUB_BASE,
  .irq = IMX9_IRQ_MU8_B
}; /* clang-format on */
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int imx9_mu_interrupt(int irq, void *context, void *args)
{
  struct imx9_mudev_s *dev = args;

  uint32_t sr  = getreg32(IMX9_MU_SR(dev->mubase));
  uint32_t rsr = getreg32(IMX9_MU_RSR(dev->mubase));
  uint32_t gsr = getreg32(IMX9_MU_GSR(dev->mubase));

  ipcinfo("MU irq=%d, SR=0x%04lx, RSR=0x%04lx, GSR=0x%04lx\n", irq, sr, rsr,
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

      for (int i = 0; i < NR_OF_GPI; i++)
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

struct imx9_mudev_s *imx95_mu_init(int index)
{
  struct imx9_mudev_s *priv;

#ifdef CONFIG_IMX9_MU5
  if ((index == 5))
    {
      priv = &g_mu5_dev;
    }

  else
#endif
#ifdef CONFIG_IMX9_MU7
      if ((index == 7))
    {
      priv = &g_mu7_dev;
    }

  else
#endif
#ifdef CONFIG_IMX9_MU8
      if ((index == 8))
    {
      priv = &g_mu8_dev;
    }

  else
#endif
    {
      return NULL;
    }

  irq_attach(priv->irq, imx9_mu_interrupt, priv);
  up_enable_irq(priv->irq);

  priv->gpi_callback = NULL;
  priv->msg_callback = NULL;

  return priv;
}

void imx95_mu_subscribe_msg(struct imx9_mudev_s *priv,
                            uint32_t msg_int_bitfield,
                            imx9_mu_msg_callback_t callback)
{
  priv->msg_callback = callback;
  putreg32(msg_int_bitfield & MSG_INT_MASK, IMX9_MU_RCR(priv->mubase));
}

void imx95_mu_subscribe_gpi(struct imx9_mudev_s *priv,
                            uint32_t gpi_int_enable,
                            imx9_mu_gpi_callback_t callback)
{
  priv->gpi_callback = callback;
  putreg32(gpi_int_enable & GPI_INT_MASK, IMX9_MU_GIER(priv->mubase));
}

void imx95_mu_deinit(struct imx9_mudev_s *priv)
{
  up_disable_irq(priv->irq);
}

int imx95_mu_send_msg_non_blocking(struct imx9_mudev_s *priv,
                                   uint32_t reg_index, uint32_t msg)
{
  assert(reg_index < IMX9_MU_TR_REGARRAY_SIZE);

  ipcinfo("MU send msg nonblocking idx=%ld, msg=%ld\n", reg_index, msg);

  if ((getreg32(IMX9_MU_TSR(priv->mubase)) & (1UL << reg_index)) == 0UL)
    {
      return -EBUSY;
    }

  putreg32(msg, IMX9_MU_TR1(priv->mubase) + (reg_index * sizeof(uint32_t)));
  return OK;
}

void imx95_mu_send_msg(struct imx9_mudev_s *priv, uint32_t reg_index,
                       uint32_t msg)
{
  assert(reg_index < IMX9_MU_TR_REGARRAY_SIZE);

  ipcinfo("MU send msg idx=%ld, msg=%ld\n", reg_index, msg);

  /* Wait TX register to be empty. */

  while ((getreg32(IMX9_MU_TSR(priv->mubase)) & (1UL << reg_index)) == 0UL)
    ;

  putreg32(msg, IMX9_MU_TR1(priv->mubase) + (reg_index * sizeof(uint32_t)));
}

int imx95_mu_has_received_msg(struct imx9_mudev_s *priv, uint32_t reg_index)
{
  if ((getreg32(IMX9_MU_RSR(priv->mubase)) & (1UL << reg_index)) == 0UL)
    {
      return -ENODATA;
    }

  return 0;
}

uint32_t imx95_mu_receive_msg_non_blocking(struct imx9_mudev_s *priv,
                                           uint32_t reg_index)
{
  assert(reg_index < IMX9_MU_RR_REGARRAY_SIZE);

  return getreg32(IMX9_MU_RR1(priv->mubase) +
            (reg_index * sizeof(uint32_t)));
}

uint32_t imx95_mu_receive_msg(struct imx9_mudev_s *priv, uint32_t reg_index)
{
  assert(reg_index < IMX9_MU_RR_REGARRAY_SIZE);

  /* Wait RX register to be full. */

  while (imx95_mu_has_received_msg(priv, reg_index) == -ENODATA);

  return getreg32(IMX9_MU_RR1(priv->mubase) +
            (reg_index * sizeof(uint32_t)));
}

int imx95_mu_trigger_interrupts(struct imx9_mudev_s *priv,
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
