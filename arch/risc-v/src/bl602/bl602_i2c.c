/****************************************************************************
 * arch/risc-v/src/bl602/bl602_i2c.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "riscv_internal.h"
#include "hardware/bl602_glb.h"
#include "hardware/bl602_hbn.h"
#include "hardware/bl602_i2c.h"
#include "bl602_i2c.h"
#include "bl602_gpio.h"
#include "bl602_glb.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define I2C0 0

#define PUT_UINT32_LE(n, b, i) \
  { \
    (b)[(i)]     = (uint8_t)((n)); \
    (b)[(i) + 1] = (uint8_t)((n) >> 8); \
    (b)[(i) + 2] = (uint8_t)((n) >> 16); \
    (b)[(i) + 3] = (uint8_t)((n) >> 24); \
  }

#define I2C_DEFAULT_FREQUENCY 100000

/* I2C state */

#define EV_I2C_END_INT    0
#define EV_I2C_TXF_INT    1
#define EV_I2C_RXF_INT    3
#define EV_I2C_FER_INT    4
#define EV_I2C_ARB_INT    5
#define EV_I2C_NAK_INT    6
#define EV_I2C_UNKNOW_INT 0xff

/* I2C Device hardware configuration */

struct bl602_i2c_config_s
{
  uint32_t reg_base; /* I2C register base address */
  uint8_t  irq;      /* Interrupt ID */
  uint32_t clk_freq; /* i2c freq */
};

/* I2C Device Private Data */

struct bl602_i2c_priv_s
{
  const struct i2c_ops_s *ops; /* Standard I2C operations */

  /* Port configuration */

  const struct bl602_i2c_config_s *config;

  uint8_t  subflag;   /* Sub address flag */
  uint32_t subaddr;   /* Sub address */
  uint8_t  sublen;    /* Sub address length */
  mutex_t  lock;      /* Mutual exclusion mutex */
  sem_t    sem_isr;   /* Interrupt wait semaphore */

  /* I2C work state */

  uint8_t i2cstate;

  struct i2c_msg_s *msgv; /* Message list */

  uint8_t msgid; /* Current message ID */
  ssize_t bytes; /* Processed data bytes */
  int     refs;  /* Reference count */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bl602_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *   msgs,
                              int                      count);

#ifdef CONFIG_I2C_RESET
static int bl602_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_ops_s bl602_i2c_ops =
{
  .transfer = bl602_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = bl602_i2c_reset
#endif
};

/* I2C device structures */

#ifdef CONFIG_BL602_I2C0
static const struct bl602_i2c_config_s bl602_i2c0_config =
{
  .reg_base = BL602_I2C_BASE,
  .irq      = BL602_IRQ_I2C,
  .clk_freq = I2C_DEFAULT_FREQUENCY,
};

static struct bl602_i2c_priv_s bl602_i2c0_priv =
{
  .ops      = &bl602_i2c_ops,
  .config   = &bl602_i2c0_config,
  .subaddr  = 0,
  .sublen   = 0,
  .lock     = NXMUTEX_INITIALIZER,
  .sem_isr  = SEM_INITIALIZER(1),
  .i2cstate = EV_I2C_END_INT,
  .msgv     = NULL,
  .msgid    = 0,
  .bytes    = 0,
};
#endif /* CONFIG_BL602_I2C0 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_i2c_send_data
 *
 * Description:
 *   Send I2C data
 *
 ****************************************************************************/

static void bl602_i2c_send_data(struct bl602_i2c_priv_s *priv)
{
  uint32_t          temp = 0;
  uint32_t          val  = 0;
  int               i;
  int               count;
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  count = msg->length - priv->bytes;
  if (count >= 4)
    {
      count = 4;
    }

  for (i = 0; i < count; i++)
    {
      val = *(msg->buffer + priv->bytes + i);
      temp += val << i * 8;
    }

  putreg32(temp, BL602_I2C_FIFO_WDATA);
  priv->bytes += count;
}

/****************************************************************************
 * Name: bl602_i2c_recvdata
 *
 * Description:
 *   Receive I2C data
 *
 ****************************************************************************/

static void bl602_i2c_recvdata(struct bl602_i2c_priv_s *priv)
{
  uint32_t          temp = 0;
  int               i    = 0;
  int               count;
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  count = msg->length - priv->bytes;
  temp  = getreg32(BL602_I2C_FIFO_RDATA);
  if (count >= 4)
    {
      PUT_UINT32_LE(temp, msg->buffer, priv->bytes);
      count = 4;
    }
  else if (count < 4)
    {
      for (i = 0; i < count; i++)
        {
          msg->buffer[priv->bytes + i] = (temp & 0xff);
          temp                         = (temp >> 8);
        }
    }

  priv->bytes += count;
}

/****************************************************************************
 * Name: bl602_i2c_clear_status
 *
 * Description:
 *   clear i2c status
 *
 ****************************************************************************/

static void bl602_i2c_clear_status(int i2cx)
{
  if (i2cx == I2C0)
    {
      modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_END_CLR
              | I2C_INT_STS_CR_NAK_CLR | I2C_INT_STS_CR_ARB_CLR);
    }
  else
    {
      i2cerr("port error\n");
    }
}

/****************************************************************************
 * Name: bl602_i2c_config_para
 *
 * Description:
 *   config i2c param
 *
 ****************************************************************************/

static void bl602_i2c_config_para(struct bl602_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  if (msg->flags & I2C_M_READ)
    {
      modifyreg32(BL602_I2C_CONFIG, 0, I2C_CONFIG_CR_I2C_PKT_DIR);
    }
  else
    {
      modifyreg32(BL602_I2C_CONFIG, I2C_CONFIG_CR_I2C_PKT_DIR, 0);
    }

  modifyreg32(BL602_I2C_CONFIG,
              I2C_CONFIG_CR_I2C_SLV_ADDR_MASK,
              msg->addr << I2C_CONFIG_CR_I2C_SLV_ADDR_SHIFT);
  if (priv->subflag > 0)
    {
      modifyreg32(BL602_I2C_CONFIG, 0, I2C_CONFIG_CR_I2C_SUB_ADDR_EN);
      modifyreg32(BL602_I2C_CONFIG,
                  I2C_CONFIG_CR_I2C_SUB_ADDR_BC_MASK,
                  (priv->sublen - 1) << I2C_CONFIG_CR_I2C_SUB_ADDR_BC_SHIFT);
    }
  else
    {
      modifyreg32(BL602_I2C_CONFIG, I2C_CONFIG_CR_I2C_SUB_ADDR_EN, 0);
    }

  modifyreg32(BL602_I2C_CONFIG,
              I2C_CONFIG_CR_I2C_PKT_LEN_MASK,
              (msg->length - 1) << I2C_CONFIG_CR_I2C_PKT_LEN_SHIFT);

  if (priv->subflag > 0)
    {
      putreg32(priv->subaddr, BL602_I2C_SUB_ADDR);
    }
}

/****************************************************************************
 * Name: bl602_i2c_intmask
 *
 * Description:
 *   Mask/Unmask the I2C interrupt
 *
 ****************************************************************************/

static void bl602_i2c_intmask(uint8_t int_type, uint8_t int_mask)
{
  switch (int_type)
    {
    case I2C_TRANS_END_INT:
      if (int_mask == 0)
        {
          /* UNMASK(Enable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_END_EN);
          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_END_MASK, 0);
        }
      else
        {
          /* MASK(Disable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_END_EN, 0);
          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_END_MASK);
        }
      break;
    case I2C_TX_FIFO_READY_INT:
      if (int_mask == 0)
        {
          /* UNMASK(Enable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_TXF_EN);
          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_TXF_MASK, 0);
        }
      else
        {
          /* MASK(Disable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_TXF_EN, 0);
          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_TXF_MASK);
        }
      break;
    case I2C_RX_FIFO_READY_INT:
      if (int_mask == 0)
        {
          /* UNMASK(Enable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_RXF_EN);
          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_RXF_MASK, 0);
        }
      else
        {
          /* MASK(Disable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_RXF_EN, 0);
          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_RXF_MASK);
        }
      break;
    case I2C_NACK_RECV_INT:
      if (int_mask == 0)
        {
          /* UNMASK(Enable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_NAK_EN);
          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_NAK_MASK, 0);
        }
      else
        {
          /* MASK(Disable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_NAK_EN, 0);
          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_NAK_MASK);
        }
      break;
    case I2C_ARB_LOST_INT:
      if (int_mask == 0)
        {
          /* UNMASK(Enable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_ARB_EN);
          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_ARB_MASK, 0);
        }
      else
        {
          /* MASK(Disable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_ARB_EN, 0);
          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_ARB_MASK);
        }
      break;
    case I2C_FIFO_ERR_INT:
      if (int_mask == 0)
        {
          /* UNMASK(Enable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_FER_EN);
          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_FER_MASK, 0);
        }
      else
        {
          /* MASK(Disable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_FER_EN, 0);
          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_FER_MASK);
        }
      break;
    case I2C_INT_ALL:
      if (int_mask == 0)
        {
          /* UNMASK(Enable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_END_EN
                  | I2C_INT_STS_CR_TXF_EN | I2C_INT_STS_CR_RXF_EN
                  | I2C_INT_STS_CR_NAK_EN | I2C_INT_STS_CR_ARB_EN
                  | I2C_INT_STS_CR_FER_EN);

          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_END_MASK
                  | I2C_INT_STS_CR_TXF_MASK | I2C_INT_STS_CR_RXF_MASK
                  | I2C_INT_STS_CR_NAK_MASK | I2C_INT_STS_CR_ARB_MASK
                  | I2C_INT_STS_CR_FER_MASK, 0);
        }
      else
        {
          /* MASK(Disable) this interrupt */

          modifyreg32(BL602_I2C_INT_STS, I2C_INT_STS_CR_END_EN
                  | I2C_INT_STS_CR_TXF_EN | I2C_INT_STS_CR_RXF_EN
                  | I2C_INT_STS_CR_NAK_EN | I2C_INT_STS_CR_ARB_EN
                  | I2C_INT_STS_CR_FER_EN, 0);

          modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_END_MASK
                  | I2C_INT_STS_CR_TXF_MASK | I2C_INT_STS_CR_RXF_MASK
                  | I2C_INT_STS_CR_NAK_MASK | I2C_INT_STS_CR_ARB_MASK
                  | I2C_INT_STS_CR_FER_MASK);
        }
      break;

    default:
      break;
    }
}

/****************************************************************************
 * Name: bl602_i2c_enable
 *
 * Description:
 *   i2c enable
 *
 ****************************************************************************/

static void bl602_i2c_enable(void)
{
  modifyreg32(BL602_I2C_FIFO_CONFIG_0, 0, I2C_FIFO_CONFIG_0_TX_FIFO_CLR);
  modifyreg32(BL602_I2C_FIFO_CONFIG_0, 0, I2C_FIFO_CONFIG_0_RX_FIFO_CLR);
  modifyreg32(BL602_I2C_CONFIG, 0, I2C_CONFIG_CR_I2C_M_EN);
}

/****************************************************************************
 * Name: bl602_i2c_transfer_enable
 *
 * Description:
 *   i2c transfer enable
 *
 ****************************************************************************/

static void bl602_i2c_transfer_enable(struct bl602_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  if (msg->flags & I2C_M_READ)
    {
      bl602_i2c_intmask(I2C_RX_FIFO_READY_INT, 0);
    }
  else
    {
      bl602_i2c_intmask(I2C_TX_FIFO_READY_INT, 0);
    }

  bl602_i2c_intmask(I2C_TRANS_END_INT, 0);
  bl602_i2c_intmask(I2C_FIFO_ERR_INT, 0);
  bl602_i2c_intmask(I2C_ARB_LOST_INT, 0);
  bl602_i2c_intmask(I2C_NACK_RECV_INT, 0);

  bl602_i2c_enable();
}

/****************************************************************************
 * Name: bl602_i2c_start_transfer
 *
 * Description:
 *   Send I2C start signal
 *
 ****************************************************************************/

static void bl602_i2c_start_transfer(struct bl602_i2c_priv_s *priv)
{
  bl602_i2c_clear_status(I2C0);
  bl602_i2c_config_para(priv);
  bl602_i2c_transfer_enable(priv);
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_i2c_setsclsync
 *
 * Description:
 *   set i2c scl sync
 *
 ****************************************************************************/

static void bl602_i2c_setsclsync(uint8_t enable)
{
  if (enable)
    {
      modifyreg32(BL602_I2C_CONFIG, 0, I2C_CONFIG_CR_I2C_SCL_SYNC_EN);
    }
  else
    {
      modifyreg32(BL602_I2C_CONFIG, I2C_CONFIG_CR_I2C_SCL_SYNC_EN, 0);
    }
}

/****************************************************************************
 * Name: bl602_i2c_setprd
 *
 * Description:
 *   set i2c prd
 *
 ****************************************************************************/

static void bl602_i2c_setprd(uint8_t phase)
{
  modifyreg32(BL602_I2C_PRD_START, I2C_PRD_START_CR_PRD_S_PH_0_MASK, phase);
  modifyreg32(BL602_I2C_PRD_START,
              I2C_PRD_START_CR_PRD_S_PH_1_MASK,
              phase << I2C_PRD_START_CR_PRD_S_PH_1_SHIFT);
  modifyreg32(BL602_I2C_PRD_START,
              I2C_PRD_START_CR_PRD_S_PH_2_MASK,
              phase << I2C_PRD_START_CR_PRD_S_PH_2_SHIFT);
  modifyreg32(BL602_I2C_PRD_START,
              I2C_PRD_START_CR_PRD_S_PH_3_MASK,
              phase << I2C_PRD_START_CR_PRD_S_PH_3_SHIFT);

  modifyreg32(BL602_I2C_PRD_STOP, I2C_PRD_STOP_CR_PRD_P_PH_0_MASK, phase);
  modifyreg32(BL602_I2C_PRD_STOP,
              I2C_PRD_STOP_CR_PRD_P_PH_1_MASK,
              phase << I2C_PRD_STOP_CR_PRD_P_PH_1_SHIFT);
  modifyreg32(BL602_I2C_PRD_STOP,
              I2C_PRD_STOP_CR_PRD_P_PH_2_MASK,
              phase << I2C_PRD_STOP_CR_PRD_P_PH_2_SHIFT);
  modifyreg32(BL602_I2C_PRD_STOP,
              I2C_PRD_STOP_CR_PRD_P_PH_3_MASK,
              phase << I2C_PRD_STOP_CR_PRD_P_PH_3_SHIFT);

  modifyreg32(BL602_I2C_PRD_DATA, I2C_PRD_DATA_CR_PRD_D_PH_0_MASK, phase);
  modifyreg32(BL602_I2C_PRD_DATA,
              I2C_PRD_DATA_CR_PRD_D_PH_1_MASK,
              phase << I2C_PRD_DATA_CR_PRD_D_PH_1_SHIFT);
  modifyreg32(BL602_I2C_PRD_DATA,
              I2C_PRD_DATA_CR_PRD_D_PH_2_MASK,
              phase << I2C_PRD_DATA_CR_PRD_D_PH_2_SHIFT);
  modifyreg32(BL602_I2C_PRD_DATA,
              I2C_PRD_DATA_CR_PRD_D_PH_3_MASK,
              phase << I2C_PRD_DATA_CR_PRD_D_PH_3_SHIFT);
}

/****************************************************************************
 * Name: bl602_set_i2c_clk
 *
 * Description:
 *   set I2C clock.
 *
 ****************************************************************************/

void bl602_set_i2c_clk(uint8_t enable, uint8_t div)
{
  modifyreg32(BL602_CLK_CFG3,
              CLK_CFG3_I2C_CLK_DIV_MASK,
              div << CLK_CFG3_I2C_CLK_DIV_SHIFT);

  if (enable)
    {
      modifyreg32(BL602_CLK_CFG3, 0, CLK_CFG3_I2C_CLK_EN);
    }
  else
    {
      modifyreg32(BL602_CLK_CFG3, CLK_CFG3_I2C_CLK_EN, 0);
    }
}

/****************************************************************************
 * Name: bl602_i2c_clockset
 *
 * Description:
 *   set i2c clock
 *
 ****************************************************************************/

static void bl602_i2c_clockset(uint32_t clk)
{
  uint8_t bclk_div = 0;
  uint32_t sys_clock = 0;

  bclk_div = bl602_glb_get_bclk_div();
  sys_clock = getreg32(BL602_HBN_RSV2);

  if (clk >= 100000)
    {
      bl602_set_i2c_clk(1, 0);
      bl602_i2c_setprd((sys_clock / (bclk_div + 1)) / (clk * 4) -
                       1);
    }
  else if (clk >= 8000)
    {
      bl602_set_i2c_clk(1, 9);
      bl602_i2c_setprd(
        ((sys_clock / (bclk_div + 1)) / 10) / (clk * 4) - 1);
    }
  else if (clk >= 800)
    {
      bl602_set_i2c_clk(1, 99);
      bl602_i2c_setprd(
        ((sys_clock / (bclk_div + 1)) / 100) / (clk * 4) - 1);
    }
  else
    {
      bl602_set_i2c_clk(1, 255);
      bl602_i2c_setprd(
        ((sys_clock / (bclk_div + 1)) / 256) / (clk * 4) - 1);
    }
}

/****************************************************************************
 * Name: bl602_i2c_set_freq
 *
 * Description:
 *   set i2c freq
 *
 ****************************************************************************/

static void bl602_i2c_set_freq(int freq)
{
  bl602_i2c_setsclsync(0);
  bl602_i2c_clockset(freq);
}

/****************************************************************************
 * Name: bl602_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int bl602_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *   msgs,
                              int                      count)
{
  int                          i;
  int                          j;
  int                          ret  = OK;
  struct bl602_i2c_priv_s *priv = (struct bl602_i2c_priv_s *)dev;

  if (count <= 0)
    {
      i2cerr("count is error\n");
      return -1;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      i2cerr("lock error\n");
      return ret;
    }

  ret = nxsem_wait_uninterruptible(&priv->sem_isr);

  if (ret < 0)
    {
      i2cerr("take sem_irq error\n");
      return ret;
    }

  priv->msgv = msgs;

  for (i = 0; i < count; i++)
    {
      priv->bytes    = 0;
      priv->i2cstate = EV_I2C_END_INT;

      bl602_i2c_set_freq(msgs[i].frequency);

      /* if msgs[i].flag I2C_M_NOSTOP,means start i2c with subddr */

      if (msgs[i].flags & I2C_M_NOSTOP)
        {
          priv->subflag = 1;
          priv->subaddr = 0;
          for (j = 0; j < msgs[i].length; j++)
            {
              priv->subaddr += msgs[i].buffer[j] << (j * 8);
            }

          priv->sublen = msgs[i].length;
          i++;
        }
      else
        {
          priv->subflag = 0;
          priv->subaddr = 0;
          priv->sublen  = 0;
        }

      priv->msgid = i;
      bl602_i2c_start_transfer(priv);

      /* wait for transter finished */

      ret = nxsem_wait_uninterruptible(&priv->sem_isr);

      if (ret < 0)
        {
          i2cerr("transter error\n");
          return ret;
        }

      if (priv->i2cstate == EV_I2C_END_INT)
        {
          i2cinfo("i2c transfer success\n");
        }
      else
        {
          i2cerr("i2c transfer error, event = %d\n", priv->i2cstate);
        }

      nxsem_post(&priv->sem_isr);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: bl602_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int bl602_i2c_reset(struct i2c_master_s *dev)
{
  struct bl602_i2c_priv_s *priv = (struct bl602_i2c_priv_s *)dev;

  bl602_swrst_ahb_slave1(AHB_SLAVE1_I2C);
  bl602_i2c_set_freq(priv->config->clk_freq);
  bl602_i2c_disable();
  up_enable_irq(BL602_IRQ_I2C);
  bl602_i2c_intmask(I2C_INT_ALL, 1);
  priv->i2cstate = EV_I2C_END_INT;
  priv->msgid    = 0;
  priv->bytes    = 0;

  return OK;
}
#endif

/****************************************************************************
 * Name: bl602_i2c_transferbytes
 *
 * Description:
 *   i2c transfer bytes.
 *
 ****************************************************************************/

static void bl602_i2c_transferbytes(struct bl602_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  if (msg->flags & I2C_M_READ)
    {
      if (priv->i2cstate == EV_I2C_RXF_INT)
        {
          if (priv->bytes < msg->length)
            {
              bl602_i2c_recvdata(priv);
            }
          else if (priv->bytes == msg->length)
            {
              bl602_i2c_intmask(I2C_RX_FIFO_READY_INT, 1);
              return;
            }
          else
            {
            }
        }
    }
  else
    {
      if (priv->i2cstate == EV_I2C_TXF_INT)
        {
          if (msg->length > priv->bytes)
            {
              bl602_i2c_send_data(priv);
            }
          else if (priv->bytes == msg->length)
            {
              bl602_i2c_intmask(I2C_TX_FIFO_READY_INT, 1);
              return;
            }
          else
            {
            }
        }
    }
}

/****************************************************************************
 * Name: bl602_i2c_disable
 *
 * Description:
 *   disable i2c
 *
 ****************************************************************************/

static void bl602_i2c_disable(void)
{
  modifyreg32(BL602_I2C_CONFIG, I2C_CONFIG_CR_I2C_M_EN, 0);

  /* Clear I2C fifo */

  modifyreg32(BL602_I2C_FIFO_CONFIG_0, 0, I2C_FIFO_CONFIG_0_TX_FIFO_CLR);
  modifyreg32(BL602_I2C_FIFO_CONFIG_0, 0, I2C_FIFO_CONFIG_0_RX_FIFO_CLR);

  /* Clear I2C interrupt status */

  modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_END_CLR);
  modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_NAK_CLR);
  modifyreg32(BL602_I2C_INT_STS, 0, I2C_INT_STS_CR_ARB_CLR);
}

/****************************************************************************
 * Name: bl602_i2c_callback
 *
 * Description:
 *   callback function.
 *
 ****************************************************************************/

static void bl602_i2c_callback(struct bl602_i2c_priv_s *priv)
{
  bl602_i2c_disable();
  bl602_i2c_intmask(I2C_INT_ALL, 1);
  bl602_i2c_clear_status(I2C0);
  nxsem_post(&priv->sem_isr);
}

/****************************************************************************
 * Name: bl602_i2c_irq
 *
 * Description:
 *   This is the common I2C interrupt handler.  It will be invoked
 *   when an interrupt received on the device.
 *
 ****************************************************************************/

static int bl602_i2c_irq(int cpuint, void *context, void *arg)
{
  uint32_t tmp_val;

  struct bl602_i2c_priv_s *priv = (struct bl602_i2c_priv_s *)arg;

  tmp_val = getreg32(BL602_I2C_INT_STS);

  if (tmp_val & I2C_INT_STS_RXF_INT)
    {
      priv->i2cstate = EV_I2C_RXF_INT;
    }
  else if (tmp_val & I2C_INT_STS_END_INT)
    {
      priv->i2cstate = EV_I2C_END_INT;
      bl602_i2c_callback(priv);
      return 0;
    }
  else if (tmp_val & I2C_INT_STS_NAK_INT)
    {
      priv->i2cstate = EV_I2C_NAK_INT;
      bl602_i2c_callback(priv);
      return -1;
    }
  else if (tmp_val & I2C_INT_STS_TXF_INT)
    {
      priv->i2cstate = EV_I2C_TXF_INT;
    }
  else if (tmp_val & I2C_INT_STS_ARB_INT)
    {
      priv->i2cstate = EV_I2C_ARB_INT;
      bl602_i2c_callback(priv);
      return -1;
    }
  else if (tmp_val & I2C_INT_STS_FER_INT)
    {
      priv->i2cstate = EV_I2C_FER_INT;
      bl602_i2c_callback(priv);
      return -1;
    }
  else
    {
      i2cerr("other interrupt\n");
      priv->i2cstate = EV_I2C_UNKNOW_INT;
      bl602_i2c_callback(priv);
      return -1;
    }

  bl602_i2c_transferbytes(priv);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

struct i2c_master_s *bl602_i2cbus_initialize(int port)
{
  struct bl602_i2c_priv_s *        priv;
  const struct bl602_i2c_config_s *config;

  switch (port)
    {
#ifdef CONFIG_BL602_I2C0
    case 0:
      priv = (struct bl602_i2c_priv_s *)&bl602_i2c0_priv;
      break;
#endif
    default:
      return NULL;
    }

  config = priv->config;

  nxmutex_lock(&priv->lock);
  if (++priv->refs > 1)
  {
    nxmutex_unlock(&priv->lock);
    return (struct i2c_master_s *)priv;
  }

  bl602_configgpio(BOARD_I2C_SCL);
  bl602_configgpio(BOARD_I2C_SDA);

  bl602_i2c_set_freq(config->clk_freq);
  bl602_i2c_disable();
  up_enable_irq(BL602_IRQ_I2C);
  bl602_i2c_intmask(I2C_INT_ALL, 1);
  irq_attach(BL602_IRQ_I2C, bl602_i2c_irq, priv);

  nxmutex_unlock(&priv->lock);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: bl602_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int bl602_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct bl602_i2c_priv_s *priv = (struct bl602_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  bl602_swrst_ahb_slave1(AHB_SLAVE1_I2C);
  nxmutex_unlock(&priv->lock);

  return OK;
}

