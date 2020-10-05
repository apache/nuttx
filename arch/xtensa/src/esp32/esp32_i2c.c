/****************************************************************************
 * arch/xtensa/src/esp32/esp32_i2c.c
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

#ifdef CONFIG_ESP32_I2C

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "esp32_i2c.h"
#include "esp32_gpio.h"
#include "esp32_cpuint.h"

#include "xtensa.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_i2c.h"
#include "hardware/esp32_soc.h"
#include "rom/esp32_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Command format */

#define I2C_BASE_CMD(_cmd, _check_ack) (((_cmd) << 11) + \
                                        ((_check_ack) << 8))

#define I2C_SEND_CMD(_cmd, _check_ack, _bytes) (((_cmd) << 11) + \
                                                ((_check_ack) << 8) + \
                                                (_bytes))

#define I2C_RECV_CMD(_cmd, _ack_val, _bytes) (((_cmd) << 11) + \
                                              ((_ack_val) << 10) + \
                                              (_bytes))

/* Default option */

#define I2C_FIFO_SIZE (255)

#define I2C_FILTER_CYC_NUM_DEF (7)

#define I2C_CLK_FREQ_DEF (100 * 1000)

#define I2C_INT_ERR_EN_BITS (I2C_ACK_ERR_INT_ENA | \
                             I2C_TIME_OUT_INT_ENA | \
                             I2C_ARBITRATION_LOST_INT_ENA | \
                             I2C_RXFIFO_OVF_INT_ENA)

/* I2C state */

enum esp32_i2cstate_e
{
  I2CSTATE_IDLE = 0,
  I2CSTATE_PROC,
  I2CSTATE_STOP,
  I2CSTATE_FINISH
};

/* I2C hardware command */

enum i2c_opmode_e
{
  I2C_CMD_RESTART = 0,        /* I2C restart command */
  I2C_CMD_WRITE,              /* I2C write command */
  I2C_CMD_READ,               /* I2C read command */
  I2C_CMD_STOP,               /* I2C stop command */
  I2C_CMD_END                 /* I2C end command */
};

/* I2C Device hardware configuration */

struct esp32_i2c_config_s
{
  uint32_t reg_base;          /* I2C register base address */

  uint32_t clk_freq;          /* Clock frequency */

  uint8_t scl_pin;            /* GPIO configuration for SCL as SCL */
  uint8_t sda_pin;            /* GPIO configuration for SDA as SDA */

  uint8_t cpu;                /* CPU ID */
  uint8_t periph;             /* peripher ID */
  uint8_t irq;                /* Interrupt ID */

  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t rst_bit;           /* I2C reset bit */

  uint32_t scl_insig;         /* I2C SCL input signal index */
  uint32_t scl_outsig;        /* I2C SCL output signal index */

  uint32_t sda_insig;         /* I2C SDA input signal index */
  uint32_t sda_outsig;        /* I2C SDA output signal index */
};

/* I2C Device Private Data */

struct esp32_i2c_priv_s
{
  const struct i2c_ops_s *ops; /* Standard I2C operations */

  /* Port configuration */

  const struct esp32_i2c_config_s *config;
  int refs;                    /* Referernce count */
  sem_t sem_excl;              /* Mutual exclusion semaphore */
  sem_t sem_isr;               /* Interrupt wait semaphore */

  /* I2C work state (see enum esp32_i2cstate_e) */

  volatile enum esp32_i2cstate_e i2cstate;

  struct i2c_msg_s *msgv;      /* Message list */

  uint8_t msgid;               /* Current message ID */
  ssize_t bytes;               /* Processed data bytes */

  int     cpuint;              /* CPU interrupt assigned to this I2C */

  uint32_t error;              /* I2C transform error */

  bool ready_read;             /* If I2C has can read date */

  uint32_t clk_freq;           /* Current I2C Clock frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp32_i2c_init_clock(FAR struct esp32_i2c_priv_s *priv,
                                 uint32_t clock);
static void esp32_i2c_init(FAR struct esp32_i2c_priv_s *priv);
static void esp32_i2c_deinit(FAR struct esp32_i2c_priv_s *priv);
static int esp32_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs,
                              int count);
#ifdef CONFIG_I2C_RESET
static int esp32_i2c_reset(FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_ops_s esp32_i2c_ops =
{
  .transfer = esp32_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = esp32_i2c_reset
#endif
};

/* I2C device structures */

#ifdef CONFIG_ESP32_I2C0
static const struct esp32_i2c_config_s esp32_i2c0_config =
{
  .reg_base   = REG_I2C_BASE(0),
  .clk_freq   = I2C_CLK_FREQ_DEF,
  .scl_pin    = CONFIG_ESP32_I2C0_SCLPIN,
  .sda_pin    = CONFIG_ESP32_I2C0_SDAPIN,
  .cpu        = 0,
  .periph     = ESP32_PERIPH_I2C_EXT0,
  .irq        = ESP32_IRQ_I2C_EXT0,
  .clk_bit    = DPORT_I2C_EXT0_CLK_EN,
  .rst_bit    = DPORT_I2C_EXT0_RST,
  .scl_insig  = I2CEXT0_SCL_IN_IDX,
  .scl_outsig = I2CEXT0_SCL_OUT_IDX,
  .sda_insig  = I2CEXT0_SDA_IN_IDX,
  .sda_outsig = I2CEXT0_SDA_OUT_IDX
};

static struct esp32_i2c_priv_s esp32_i2c0_priv =
{
  .ops        = &esp32_i2c_ops,
  .config     = &esp32_i2c0_config,
  .refs       = 0,
  .i2cstate   = I2CSTATE_IDLE,
  .msgv       = NULL,
  .msgid      = 0,
  .bytes      = 0,
  .ready_read = 0
};
#endif /* CONFIG_ESP32_I2C1 */

#ifdef CONFIG_ESP32_I2C1
static const struct esp32_i2c_config_s esp32_i2c1_config =
{
  .reg_base   = REG_I2C_BASE(1),
  .clk_freq   = I2C_CLK_FREQ_DEF,
  .scl_pin    = CONFIG_ESP32_I2C1_SCLPIN,
  .sda_pin    = CONFIG_ESP32_I2C1_SDAPIN,
  .cpu        = 0,
  .periph     = ESP32_PERIPH_I2C_EXT1,
  .irq        = ESP32_IRQ_I2C_EXT1,
  .clk_bit    = DPORT_I2C_EXT1_CLK_EN,
  .rst_bit    = DPORT_I2C_EXT1_RST,
  .scl_insig  = I2CEXT1_SCL_IN_IDX,
  .scl_outsig = I2CEXT1_SCL_OUT_IDX,
  .sda_insig  = I2CEXT1_SDA_IN_IDX,
  .sda_outsig = I2CEXT1_SDA_OUT_IDX
};

static struct esp32_i2c_priv_s esp32_i2c1_priv =
{
  .ops        = &esp32_i2c_ops,
  .config     = &esp32_i2c1_config,
  .refs       = 0,
  .i2cstate   = I2CSTATE_IDLE,
  .msgv       = NULL,
  .msgid      = 0,
  .bytes      = 0,
  .ready_read = 0
};
#endif /* CONFIG_ESP32_I2C1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_i2c_set_reg
 ****************************************************************************/

static inline void esp32_i2c_set_reg(struct esp32_i2c_priv_s *priv,
                                     int offset,
                                     uint32_t value)
{
  putreg32(value, priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_i2c_get_reg
 ****************************************************************************/

static inline uint32_t esp32_i2c_get_reg(struct esp32_i2c_priv_s *priv,
                                         int offset)
{
  return getreg32(priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_i2c_set_reg_bits
 ****************************************************************************/

static inline void esp32_i2c_set_reg_bits(struct esp32_i2c_priv_s *priv,
                                          int offset,
                                          uint32_t bits)
{
  uint32_t tmp = getreg32(priv->config->reg_base + offset);

  putreg32(tmp | bits, priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_i2c_reset_reg_bits
 ****************************************************************************/

static inline void esp32_i2c_reset_reg_bits(struct esp32_i2c_priv_s *priv,
                                            int offset,
                                            uint32_t bits)
{
  uint32_t tmp = getreg32(priv->config->reg_base + offset);

  putreg32(tmp & (~bits), priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_i2c_reset_fifo
 *
 * Description:
 *   Reset I2C RX and TX hardware FiFO
 *
 ****************************************************************************/

static void esp32_i2c_reset_fifo(FAR struct esp32_i2c_priv_s *priv)
{
  esp32_i2c_set_reg_bits(priv, I2C_FIFO_CONF_OFFSET, I2C_TX_FIFO_RST);
  esp32_i2c_reset_reg_bits(priv, I2C_FIFO_CONF_OFFSET, I2C_TX_FIFO_RST);

  esp32_i2c_set_reg_bits(priv, I2C_FIFO_CONF_OFFSET, I2C_RX_FIFO_RST);
  esp32_i2c_reset_reg_bits(priv, I2C_FIFO_CONF_OFFSET, I2C_RX_FIFO_RST);
}

/****************************************************************************
 * Name: esp32_i2c_sendstart
 *
 * Description:
 *   Send I2C start signal
 *
 ****************************************************************************/

static void esp32_i2c_sendstart(FAR struct esp32_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  esp32_i2c_set_reg(priv, I2C_COMD0_OFFSET,
                    I2C_BASE_CMD(I2C_CMD_RESTART, 0));
  esp32_i2c_set_reg(priv, I2C_COMD1_OFFSET,
                    I2C_SEND_CMD(I2C_CMD_WRITE, 1, 1));
  esp32_i2c_set_reg(priv, I2C_COMD2_OFFSET,
                    I2C_BASE_CMD(I2C_CMD_END, 0));
  esp32_i2c_set_reg(priv, I2C_DATA_OFFSET, (msg->addr << 1) |
                                           (msg->flags & I2C_M_READ));
  esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, I2C_END_DETECT_INT_ENA |
                                              I2C_INT_ERR_EN_BITS);
  esp32_i2c_set_reg_bits(priv, I2C_CTR_OFFSET, I2C_TRANS_START_M);
}

/****************************************************************************
 * Name: esp32_i2c_senddata
 *
 * Description:
 *   Send I2C data
 *
 ****************************************************************************/

static void esp32_i2c_senddata(FAR struct esp32_i2c_priv_s *priv)
{
  int i;
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  int n = msg->length - priv->bytes;

  n = n < I2C_FIFO_SIZE ? n : I2C_FIFO_SIZE;

  esp32_i2c_set_reg(priv, I2C_COMD0_OFFSET,
                    I2C_SEND_CMD(I2C_CMD_WRITE, 1, n));
  esp32_i2c_set_reg(priv, I2C_COMD1_OFFSET,
                    I2C_BASE_CMD(I2C_CMD_END, 0));

  for (i = 0; i < n; i ++)
    {
      esp32_i2c_set_reg(priv, I2C_DATA_OFFSET,
                        msg->buffer[priv->bytes + i]);
    }

  priv->bytes += n;

  esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, I2C_END_DETECT_INT_ENA |
                                              I2C_INT_ERR_EN_BITS);
  esp32_i2c_set_reg_bits(priv, I2C_CTR_OFFSET, I2C_TRANS_START_M);
}

/****************************************************************************
 * Name: esp32_i2c_recvdata
 *
 * Description:
 *   Receive I2C data
 *
 ****************************************************************************/

static void esp32_i2c_recvdata(struct esp32_i2c_priv_s *priv)
{
  int i;
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  uint32_t cmd = esp32_i2c_get_reg(priv, I2C_COMD0_OFFSET);
  uint8_t n = cmd & 0xff;

  for (i = 0; i < n; i++)
    {
      msg->buffer[priv->bytes + i] = esp32_i2c_get_reg(priv,
                                                       I2C_DATA_OFFSET);
    }

  priv->bytes += n;
}

/****************************************************************************
 * Name: esp32_i2c_startrecv
 *
 * Description:
 *   Configure I2C to prepare receiving data and it will create an interrupt
 *   to receive real data
 *
 ****************************************************************************/

static void esp32_i2c_startrecv(struct esp32_i2c_priv_s *priv)
{
  int ack_value;
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  int n = msg->length - priv->bytes;

  if (n > 1)
    {
      n -= 1;
      n = n < I2C_FIFO_SIZE ? n : I2C_FIFO_SIZE;
      ack_value = 0;
    }
  else
    {
      ack_value = 1;
    }

  esp32_i2c_set_reg(priv, I2C_COMD0_OFFSET,
                    I2C_RECV_CMD(I2C_CMD_READ, ack_value, n));
  esp32_i2c_set_reg(priv, I2C_COMD1_OFFSET,
                    I2C_BASE_CMD(I2C_CMD_END, 0));

  esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, I2C_END_DETECT_INT_ENA |
                                              I2C_INT_ERR_EN_BITS);
  esp32_i2c_set_reg_bits(priv, I2C_CTR_OFFSET, I2C_TRANS_START_M);
}

/****************************************************************************
 * Name: esp32_i2c_sendstop
 *
 * Description:
 *   Send I2C STOP signal
 *
 ****************************************************************************/

static void esp32_i2c_sendstop(struct esp32_i2c_priv_s *priv)
{
  esp32_i2c_set_reg(priv, I2C_COMD0_OFFSET, I2C_BASE_CMD(I2C_CMD_STOP, 0));
  esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, I2C_TRANS_COMPLETE_INT_ENA |
                                              I2C_INT_ERR_EN_BITS);
  esp32_i2c_set_reg_bits(priv, I2C_CTR_OFFSET, I2C_TRANS_START_M);
}

/****************************************************************************
 * Name: esp32_i2c_init_clock
 *
 * Description:
 *   Initialize I2C hardware clock
 *
 ****************************************************************************/

static void esp32_i2c_init_clock(FAR struct esp32_i2c_priv_s *priv,
                                 uint32_t clk_freq)
{
  uint32_t half_cycles = APB_CLK_FREQ / clk_freq / 2;
  uint32_t timeout_cycles = half_cycles * 20;

  if (clk_freq == priv->clk_freq)
    {
      return ;
    }

  esp32_i2c_set_reg(priv, I2C_SCL_LOW_PERIOD_OFFSET, half_cycles);
  esp32_i2c_set_reg(priv, I2C_SCL_HIGH_PERIOD_OFFSET, half_cycles);
  esp32_i2c_set_reg(priv, I2C_SDA_HOLD_OFFSET, half_cycles / 2);
  esp32_i2c_set_reg(priv, I2C_SDA_SAMPLE_OFFSET, half_cycles / 2);
  esp32_i2c_set_reg(priv, I2C_SCL_RSTART_SETUP_OFFSET, half_cycles);
  esp32_i2c_set_reg(priv, I2C_SCL_STOP_SETUP_OFFSET, half_cycles);
  esp32_i2c_set_reg(priv, I2C_SCL_START_HOLD_OFFSET, half_cycles);
  esp32_i2c_set_reg(priv, I2C_SCL_STOP_HOLD_OFFSET, half_cycles);
  esp32_i2c_set_reg(priv, I2C_TO_OFFSET, timeout_cycles);

  priv->clk_freq = clk_freq;
}

/****************************************************************************
 * Name: esp32_i2c_init
 *
 * Description:
 *   Initialize I2C hardware
 *
 ****************************************************************************/

static void esp32_i2c_init(FAR struct esp32_i2c_priv_s *priv)
{
  const struct esp32_i2c_config_s *config = priv->config;

  esp32_gpiowrite(config->scl_pin, 1);
  esp32_gpiowrite(config->sda_pin, 1);

  esp32_configgpio(config->scl_pin, OUTPUT | OPEN_DRAIN | FUNCTION_3);
  gpio_matrix_out(config->scl_pin, config->scl_outsig, 0, 0);
  gpio_matrix_in(config->scl_pin, config->scl_insig, 0);

  esp32_configgpio(config->sda_pin, INPUT |
                                    OUTPUT |
                                    OPEN_DRAIN |
                                    FUNCTION_3);
  gpio_matrix_out(config->sda_pin, config->sda_outsig, 0, 0);
  gpio_matrix_in(config->sda_pin, config->sda_insig, 0);

  modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, config->clk_bit);
  modifyreg32(DPORT_PERIP_RST_EN_REG, config->rst_bit, 0);

  esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, 0);
  esp32_i2c_set_reg(priv, I2C_INT_CLR_OFFSET, UINT32_MAX);
  esp32_i2c_set_reg(priv, I2C_CTR_OFFSET, I2C_MS_MODE |
                                          I2C_SCL_FORCE_OUT |
                                          I2C_SDA_FORCE_OUT);
  esp32_i2c_reset_reg_bits(priv, I2C_FIFO_CONF_OFFSET,
                           I2C_NONFIFO_EN_M);

  esp32_i2c_reset_fifo(priv);

  esp32_i2c_set_reg(priv, I2C_SCL_FILTER_CFG_OFFSET, I2C_SCL_FILTER_EN_M |
                                                     I2C_FILTER_CYC_NUM_DEF);
  esp32_i2c_set_reg(priv, I2C_SDA_FILTER_CFG_OFFSET, I2C_SDA_FILTER_EN_M |
                                                     I2C_FILTER_CYC_NUM_DEF);

  esp32_i2c_init_clock(priv, config->clk_freq);
}

/****************************************************************************
 * Name: esp32_i2c_deinit
 *
 * Description:
 *   Disable I2C hardware
 *
 ****************************************************************************/

static void esp32_i2c_deinit(FAR struct esp32_i2c_priv_s *priv)
{
  const struct esp32_i2c_config_s *config = priv->config;

  modifyreg32(DPORT_PERIP_RST_EN_REG, 0, config->rst_bit);
  modifyreg32(DPORT_PERIP_CLK_EN_REG, config->clk_bit, 0);
}

/****************************************************************************
 * Name: esp32_i2c_reset_fsmc
 *
 * Description:
 *   Reset I2C hardware state machine and registers
 *
 ****************************************************************************/

static void esp32_i2c_reset_fsmc(FAR struct esp32_i2c_priv_s *priv)
{
  esp32_i2c_deinit(priv);
  esp32_i2c_init(priv);
}

/****************************************************************************
 * Name: esp32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

static int esp32_i2c_sem_waitdone(FAR struct esp32_i2c_priv_s *priv)
{
  int ret;
  struct timespec abstime;

  clock_gettime(CLOCK_REALTIME, &abstime);

  abstime.tv_sec += 10;
  abstime.tv_nsec += 0;

  ret = nxsem_timedwait_uninterruptible(&priv->sem_isr, &abstime);

  return ret;
}

/****************************************************************************
 * Name: esp32_i2c_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ****************************************************************************/

static int esp32_i2c_sem_wait(FAR struct esp32_i2c_priv_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->sem_excl);
}

/****************************************************************************
 * Name: esp32_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ****************************************************************************/

static void esp32_i2c_sem_post(struct esp32_i2c_priv_s *priv)
{
  nxsem_post(&priv->sem_excl);
}

/****************************************************************************
 * Name: esp32_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ****************************************************************************/

static void esp32_i2c_sem_destroy(FAR struct esp32_i2c_priv_s *priv)
{
  nxsem_destroy(&priv->sem_excl);
  nxsem_destroy(&priv->sem_isr);
}

/****************************************************************************
 * Name: esp32_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ****************************************************************************/

static inline void esp32_i2c_sem_init(FAR struct esp32_i2c_priv_s *priv)
{
  nxsem_init(&priv->sem_excl, 0, 1);

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->sem_isr, 0, 0);
  nxsem_set_protocol(&priv->sem_isr, SEM_PRIO_NONE);
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int esp32_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs,
                              int count)
{
  int i;
  int ret = OK;
  FAR struct esp32_i2c_priv_s *priv = (FAR struct esp32_i2c_priv_s *)dev;

  DEBUGASSERT(count > 0);

  ret = esp32_i2c_sem_wait(priv);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->i2cstate != I2CSTATE_IDLE)
    {
      esp32_i2c_reset_fsmc(priv);
      priv->i2cstate = I2CSTATE_IDLE;
    }

  priv->msgv = msgs;

  for (i = 0; i < count; i++)
    {
      esp32_i2c_reset_fifo(priv);

      priv->bytes = 0;
      priv->msgid = i;
      priv->ready_read = 0;
      priv->error = 0;
      priv->i2cstate = I2CSTATE_PROC;

      esp32_i2c_init_clock(priv, msgs[i].frequency);

      esp32_i2c_sendstart(priv);

      if (esp32_i2c_sem_waitdone(priv) < 0)
        {
          ret = -ETIMEDOUT;
          break;
        }
      else
        {
          if (priv->error)
            {
              ret = -EIO;
            }
          else
            {
              priv->i2cstate = I2CSTATE_IDLE;
              ret = OK;
            }
        }
    }

  esp32_i2c_sem_post(priv);

  return ret;
}

/****************************************************************************
 * Name: esp32_i2c_reset
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
static int esp32_i2c_reset(FAR struct i2c_master_s *dev)
{
  irqstate_t flags;
  FAR struct esp32_i2c_priv_s *priv = (FAR struct esp32_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  DEBUGASSERT(priv->refs > 0);

  flags = enter_critical_section();

  esp32_i2c_reset_fsmc(priv);

  priv->i2cstate = I2CSTATE_IDLE;
  priv->msgid = 0;
  priv->bytes = 0;
  priv->ready_read = 0;

  leave_critical_section(flags);

  return OK;
}
#endif

/****************************************************************************
 * Name: esp32_i2c_irq
 *
 * Description:
 *   This is the common I2C interrupt handler.  It will be invoked
 *   when an interrupt received on the device.
 *
 ****************************************************************************/

static int esp32_i2c_irq(int cpuint, void *context, FAR void *arg)
{
  struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)arg;
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  uint32_t status = esp32_i2c_get_reg(priv, I2C_INT_STATUS_OFFSET);

  esp32_i2c_set_reg(priv, I2C_INT_CLR_OFFSET, status);

  if (I2C_INT_ERR_EN_BITS & status)
    {
      priv->error = status & I2C_INT_ERR_EN_BITS;
      nxsem_post(&priv->sem_isr);
    }
  else
    {
      if (priv->i2cstate == I2CSTATE_PROC)
        {
          if (msg->flags & I2C_M_READ)
            {
              if (priv->ready_read)
                {
                  esp32_i2c_recvdata(priv);

                  priv->ready_read = 0;
                }

              if (priv->bytes == msg->length)
                {
                  esp32_i2c_sendstop(priv);

                  priv->i2cstate = I2CSTATE_FINISH;
                }
              else
                {
                  esp32_i2c_startrecv(priv);

                  priv->ready_read = 1;
                }
            }
          else
            {
              esp32_i2c_senddata(priv);

              if (priv->bytes == msg->length)
                {
                  priv->i2cstate = I2CSTATE_STOP;
                }
            }
        }
      else if (priv->i2cstate == I2CSTATE_STOP)
        {
          esp32_i2c_sendstop(priv);

          priv->i2cstate = I2CSTATE_FINISH;
        }
      else if (priv->i2cstate == I2CSTATE_FINISH)
        {
          nxsem_post(&priv->sem_isr);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

FAR struct i2c_master_s *esp32_i2cbus_initialize(int port)
{
  int ret;
  irqstate_t flags;
  struct esp32_i2c_priv_s *priv;
  const struct esp32_i2c_config_s *config;

  switch (port)
    {
#ifdef CONFIG_ESP32_I2C0
    case 0:
      priv = (struct esp32_i2c_priv_s *)&esp32_i2c0_priv;
      break;
#endif
#ifdef CONFIG_ESP32_I2C1
    case 1:
      priv = (struct esp32_i2c_priv_s *)&esp32_i2c1_priv;
      break;
#endif
    default:
      return NULL;
    }

  config = priv->config;

  flags = enter_critical_section();

  if ((volatile int)priv->refs++ != 0)
    {
      leave_critical_section(flags);

      return (struct i2c_master_s *)priv;
    }

  priv->cpuint = esp32_alloc_levelint(1);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type */

      leave_critical_section(flags);

      return NULL;
    }

  up_disable_irq(priv->cpuint);
  esp32_attach_peripheral(config->cpu, config->periph, priv->cpuint);

  ret = irq_attach(config->irq, esp32_i2c_irq, priv);
  if (ret != OK)
    {
      esp32_detach_peripheral(config->cpu, config->periph, priv->cpuint);
      esp32_free_cpuint(priv->cpuint);

      leave_critical_section(flags);

      return NULL;
    }

  up_enable_irq(priv->cpuint);

  esp32_i2c_sem_init(priv);

  esp32_i2c_init(priv);

  leave_critical_section(flags);

  return (FAR struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: esp32_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int esp32_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  irqstate_t flags;
  FAR struct esp32_i2c_priv_s *priv = (FAR struct esp32_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->refs == 0)
    {
      return ERROR;
    }

  flags = enter_critical_section();

  if (--priv->refs)
    {
      leave_critical_section(flags);
      return OK;
    }

  leave_critical_section(flags);

  esp32_i2c_deinit(priv);

  esp32_i2c_sem_destroy(priv);

  return OK;
}

#endif /* CONFIG_ESP32_I2C */
