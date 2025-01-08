/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_flexio_i2c.c
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

#include <sys/types.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "s32k1xx_edma.h"
#include "hardware/s32k1xx_dmamux.h"
#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_flexio_i2c.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_pin.h"

#include <arch/board/board.h>

/* At least one I2C peripheral must be enabled */
#define CONFIG_S32K1XX_FLEXIO_I2C
#ifdef CONFIG_S32K1XX_FLEXIO_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_MAX_FREQUENCY       400000
#define FLEXIO_TX_IRQ           (1 << priv->config->tx_inst)
#define FLEXIO_RX_IRQ           (1 << priv->config->rx_inst)
#define FLEXIO_ADDR_READ(addr)  ((addr << 1 ) | 1)
#define FLEXIO_ADDR_WRITE(addr) (addr << 1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

enum s32k1xx_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_RECV_DATA,
  INTSTATE_WRITE_DATA,
};

struct s32k1xx_flexio_i2c_config_s
{
  int              tx_inst;
  int              rx_inst;
  int              timer;
  int              two_word_timer;
  uint32_t         flexio_sda_pin;
  uint32_t         flexio_scl_pin;
  uint32_t         sda_pin;
  uint32_t         scl_pin;
};

struct s32k1xx_flexio_i2cdev_s
{
  struct i2c_master_s dev;     /* Generic I2C device */
  unsigned int     base;       /* Base address of registers */
  uint16_t         irqid;      /* IRQ for this device */
  int8_t           port;       /* Port number */
  uint32_t         base_freq;  /* branch frequency */

  mutex_t          lock;       /* Only one thread can access at a time */
  sem_t            wait;       /* Place to wait for transfer completion */
  uint32_t         frequency;  /* Current I2C frequency */
  int32_t          tx_count;
  int32_t          reg_buff_offset;
  int32_t          reg_buff_mod;
  uint32_t         tx_done;

  struct i2c_msg_s *msgs;
  int              total_bytes;

  /* Port configuration */

  const struct s32k1xx_flexio_i2c_config_s *config;

  enum s32k1xx_intstate_e state;

  int              error;      /* Error status of each transfers */
  int              refs;       /* Reference count */
};

static const struct s32k1xx_flexio_i2c_config_s s32k1xx_flexio_i2c0_config =
{
  .tx_inst = 0,
  .rx_inst = 1,
  .timer = 1,
  .two_word_timer = 0,
  .flexio_sda_pin = FLEXIO_I2C0_SDA,
  .flexio_scl_pin = FLEXIO_I2C0_SCL,
  .sda_pin = PIN_FLEXIO_I2C0_SCL,
  .scl_pin = PIN_FLEXIO_I2C0_SDA,
};

static struct s32k1xx_flexio_i2cdev_s g_i2c0dev =
{
  .port = 0,
  .base = S32K1XX_FLEXIO_BASE,
  .irqid = S32K1XX_IRQ_FLEXIO,
  .lock = NXMUTEX_INITIALIZER,
  .wait = SEM_INITIALIZER(0),
  .refs = 0,
  .config = &s32k1xx_flexio_i2c0_config,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void s32k1xx_flexio_i2c_init(struct s32k1xx_flexio_i2cdev_s *priv);
static int  s32k1xx_flexio_i2c_interrupt(int irq, void *context, void *arg);
static void s32k1xx_flexio_i2c_set_freq_and_size(
                                   struct s32k1xx_flexio_i2cdev_s *priv,
                                   uint32_t frequency, uint8_t num_bytes);
static int  s32k1xx_flexio_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count);

static inline uint32_t flexio_getreg32(struct s32k1xx_flexio_i2cdev_s *priv,
                                    uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void flexio_putreg32(struct s32k1xx_flexio_i2cdev_s *priv,
                                 uint32_t val, uint32_t offset)
{
  putreg32(val, priv->base + offset);
}

static inline void flexio_modifyreg32(struct s32k1xx_flexio_i2cdev_s *priv,
              unsigned int offset,
              uint32_t clearbits,
              uint32_t setbits)
{
  modifyreg32(priv->base + offset, clearbits, setbits);
}

static inline void
enable_shifter_status_interrupts(struct s32k1xx_flexio_i2cdev_s *priv,
                                                    uint32_t mask)
{
  flexio_modifyreg32(priv, S32K1XX_FLEXIO_SHIFTSIEN_OFFSET, 0, mask);
}

static inline void
disable_shifter_status_interrupts(struct s32k1xx_flexio_i2cdev_s *priv,
                                                    uint32_t mask)
{
  flexio_modifyreg32(priv, S32K1XX_FLEXIO_SHIFTSIEN_OFFSET, mask, 0);
}

static inline uint32_t
get_shifter_status_flags(struct s32k1xx_flexio_i2cdev_s *priv)
{
  return flexio_getreg32(priv, S32K1XX_FLEXIO_SHIFTSTAT_OFFSET);
}

static inline void
clear_shifter_status_flags(struct s32k1xx_flexio_i2cdev_s *priv,
                                                    uint32_t mask)
{
  flexio_putreg32(priv, mask, S32K1XX_FLEXIO_SHIFTSTAT_OFFSET);
}

static inline void
enable_timer_status_interrupts(struct s32k1xx_flexio_i2cdev_s *priv,
                                                    uint32_t mask)
{
  flexio_modifyreg32(priv, S32K1XX_FLEXIO_TIMIEN_OFFSET, 0, mask);
}

static inline void
disable_timer_status_interrupts(struct s32k1xx_flexio_i2cdev_s *priv,
                                                    uint32_t mask)
{
  flexio_modifyreg32(priv, S32K1XX_FLEXIO_TIMIEN_OFFSET, mask, 0);
}

static inline void
s32k1xx_flexio_i2c_set_nack(struct s32k1xx_flexio_i2cdev_s *priv)
{
  flexio_putreg32(priv, FLEXIO_SHIFTCFG_INSRC_PIN |
    FLEXIO_SHIFTCFG_SSTOP_ONE |
    FLEXIO_SHIFTCFG_SSTART_ZERO,
    S32K1XX_FLEXIO_SHIFTCFG0_OFFSET + priv->config->tx_inst * 0x4);
}

static inline void
s32k1xx_flexio_i2c_set_ack(struct s32k1xx_flexio_i2cdev_s *priv)
{
  flexio_putreg32(priv, FLEXIO_SHIFTCFG_INSRC_PIN |
    FLEXIO_SHIFTCFG_SSTOP_ZERO |
    FLEXIO_SHIFTCFG_SSTART_ZERO,
    S32K1XX_FLEXIO_SHIFTCFG0_OFFSET + priv->config->tx_inst * 0x4);
}

static inline void
s32k1xx_flexio_i2c_clear_errors(struct s32k1xx_flexio_i2cdev_s *priv)
{
  flexio_getreg32(priv, S32K1XX_FLEXIO_SHIFTBUFBIS0_OFFSET +
                        priv->config->rx_inst * 0x4);
  flexio_putreg32(priv, FLEXIO_TX_IRQ | FLEXIO_RX_IRQ,
                        S32K1XX_FLEXIO_SHIFTERR_OFFSET);
}

/****************************************************************************
 * I2C device operations
 ****************************************************************************/

struct i2c_ops_s s32k1xx_flexio_i2c_ops =
{
  .transfer = s32k1xx_flexio_i2c_transfer,
};

/****************************************************************************
 * Name: s32k1xx_flexio_i2c_set_freq_and_size
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void s32k1xx_flexio_i2c_set_freq_and_size(
                                   struct s32k1xx_flexio_i2cdev_s *priv,
                                   uint32_t frequency, uint8_t num_bytes)
{
  int divider = ((priv->base_freq / frequency) / 2) - 1;

  if (divider > 0xff)
    {
      divider = 0xff;
    }

  flexio_putreg32(priv, divider |
                  ((((num_bytes + 1) * 9 + 1) * 2 - 1) << 8),
                  S32K1XX_FLEXIO_TIMCMP0_OFFSET +
                  priv->config->two_word_timer * 0x4);
}

/****************************************************************************
 * Name: s32k1xx_flexio_i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int s32k1xx_flexio_i2c_interrupt(int irq, void *context, void *arg)
{
  struct s32k1xx_flexio_i2cdev_s *priv =
          (struct s32k1xx_flexio_i2cdev_s *)arg;
  struct i2c_msg_s *msg = priv->msgs;

  if ((priv->state == INTSTATE_IDLE))
    {
      disable_shifter_status_interrupts(priv, FLEXIO_RX_IRQ);
      disable_shifter_status_interrupts(priv, FLEXIO_TX_IRQ);
    }
  else if ((priv->state == INTSTATE_RECV_DATA))
    {
      if (!priv->tx_done && get_shifter_status_flags(priv) & FLEXIO_TX_IRQ)
        {
          if (priv->tx_count == msg->length)
            {
              /* Send NACK and Stop bit */

              s32k1xx_flexio_i2c_set_nack(priv);
              flexio_putreg32(priv, 0x00,
                              S32K1XX_FLEXIO_SHIFTBUFBBS0_OFFSET +
                              priv->config->tx_inst * 0x4);
              priv->tx_count++;
            }
          else if (priv->tx_count < msg->length)
            {
              flexio_putreg32(priv, 0xff,
                              S32K1XX_FLEXIO_SHIFTBUFBBS0_OFFSET +
                              priv->config->tx_inst * 0x4);
              priv->tx_count++;
            }
          else
            {
              priv->tx_done = 1;
              disable_shifter_status_interrupts(priv, FLEXIO_TX_IRQ);
            }
        }

      if (get_shifter_status_flags(priv) & FLEXIO_RX_IRQ)
        {
          if (priv->reg_buff_offset < 0)
            {
              flexio_getreg32(priv, S32K1XX_FLEXIO_SHIFTBUFBIS0_OFFSET +
                              priv->config->rx_inst * 0x4);

              /* If RX shifter reports an error then stop bit (ACK)
               * doesn't go low, thus we get a NACK
               */

              if (flexio_getreg32(priv, S32K1XX_FLEXIO_SHIFTERR_OFFSET) &
                  FLEXIO_RX_IRQ)
                {
                  priv->error = 1;
                  flexio_getreg32(priv, S32K1XX_FLEXIO_SHIFTBUFBIS0_OFFSET +
                                  priv->config->rx_inst * 0x4);
                  flexio_putreg32(priv, FLEXIO_RX_IRQ,
                                  S32K1XX_FLEXIO_SHIFTERR_OFFSET);
                  disable_shifter_status_interrupts(priv, FLEXIO_RX_IRQ);
                  disable_shifter_status_interrupts(priv, FLEXIO_TX_IRQ);
                  priv->state = INTSTATE_IDLE;
                  nxsem_post(&priv->wait);
                }
              else if (msg->length > 1)
                {
                  s32k1xx_flexio_i2c_set_ack(priv);
                }
            }
          else
            {
              msg->buffer[priv->reg_buff_offset] =
                  flexio_getreg32(priv, S32K1XX_FLEXIO_SHIFTBUFBIS0_OFFSET +
                                  priv->config->rx_inst * 0x4) & 0xff;
            }

          priv->reg_buff_offset++;
          if (priv->reg_buff_offset == msg->length)
            {
              disable_shifter_status_interrupts(priv, FLEXIO_RX_IRQ);
              disable_shifter_status_interrupts(priv, FLEXIO_TX_IRQ);
              priv->state = INTSTATE_IDLE;
              nxsem_post(&priv->wait);
            }
        }
    }
  else if ((priv->state == INTSTATE_WRITE_DATA))
    {
      if (!priv->tx_done && get_shifter_status_flags(priv) & FLEXIO_TX_IRQ)
        {
          flexio_putreg32(priv, msg->buffer[priv->reg_buff_mod++],
                          S32K1XX_FLEXIO_SHIFTBUFBBS0_OFFSET +
                              priv->config->tx_inst * 0x4);
          priv->tx_count++;

          if (priv->reg_buff_mod % msg->length == 0)
            {
              priv->reg_buff_mod = 0;
              priv->msgs++;
            }

          if (priv->tx_count == priv->total_bytes)
            {
              priv->tx_done = 1;
              disable_shifter_status_interrupts(priv, FLEXIO_TX_IRQ);
            }
        }

      if (get_shifter_status_flags(priv) & FLEXIO_RX_IRQ)
        {
          flexio_getreg32(priv, S32K1XX_FLEXIO_SHIFTBUFBIS0_OFFSET +
                                    priv->config->rx_inst * 0x4);

          /* If RX shifter reports an error then stop bit (ACK)
           * doesn't go low, thus we get a NACK
           */

          if (priv->reg_buff_offset == 0 &&
              (flexio_getreg32(priv, S32K1XX_FLEXIO_SHIFTERR_OFFSET) &
               FLEXIO_RX_IRQ))
            {
              priv->error = 1;
            }

          priv->reg_buff_offset++;
          if (priv->reg_buff_offset == priv->total_bytes - 1)
            {
              disable_shifter_status_interrupts(priv, FLEXIO_RX_IRQ);
              priv->state = INTSTATE_IDLE;
              nxsem_post(&priv->wait);
            }
        }
    }

  return OK;
}

static int set_expiretime(int expire_time, struct timespec *set_time)
{
  struct timespec curr_time;

  /* Get current time. */

  if (clock_gettime(CLOCK_REALTIME, &curr_time) != OK)
    {
      return ERROR;
    }

  set_time->tv_sec = expire_time / 1000;
  set_time->tv_nsec =
    (expire_time - (set_time->tv_sec * 1000)) * 1000 * 1000;

  set_time->tv_sec += curr_time.tv_sec;
  set_time->tv_nsec += curr_time.tv_nsec;

  /* Check more than 1 sec. */

  if (set_time->tv_nsec >= (1000 * 1000 * 1000))
    {
      set_time->tv_sec += 1;
      set_time->tv_nsec -= (1000 * 1000 * 1000);
    }

  return OK;
}

static void s32k1xx_flexio_i2c_init(struct s32k1xx_flexio_i2cdev_s *priv)
{
  /* Reset FlexIO peripheral */

  flexio_modifyreg32(priv, S32K1XX_FLEXIO_CTRL_OFFSET, 0,
         FLEXIO_CTRL_SWRST);
  flexio_putreg32(priv, 0, S32K1XX_FLEXIO_CTRL_OFFSET);

  /* Initialize FlexIO peripheral */

  flexio_modifyreg32(priv, S32K1XX_FLEXIO_CTRL_OFFSET,
         (FLEXIO_CTRL_DOZEN |
          FLEXIO_CTRL_DBGE |
          FLEXIO_CTRL_FASTACC |
          FLEXIO_CTRL_FLEXEN),
         (FLEXIO_CTRL_DBGE |
          FLEXIO_CTRL_FLEXEN_DIS));

  /* Start bit enabled (logic 0) and stop bit enabled (logic 1). */

  s32k1xx_flexio_i2c_set_nack(priv);

  /* Transmit mode, output to FXIO pin */

  flexio_putreg32(priv, FLEXIO_SHIFTCTL_TIMSEL(priv->config->timer) |
      FLEXIO_SHIFTCTL_TIMPOL_PE |
      FLEXIO_SHIFTCTL_PINCFG_OD |
      FLEXIO_SHIFTCTL_PINSEL(priv->config->flexio_sda_pin) |
      FLEXIO_SHIFTCTL_PINPOL_LO |
      FLEXIO_SHIFTCTL_SMOD_TX,
      S32K1XX_FLEXIO_SHIFTCTL0_OFFSET + priv->config->tx_inst * 0x4);

  /* Start bit disabled and stop bit enabled
   * (logic 0) for ACK/NACK detection..
   */

  flexio_putreg32(priv, FLEXIO_SHIFTCFG_INSRC_PIN |
      FLEXIO_SHIFTCFG_SSTOP_ZERO |
      FLEXIO_SHIFTCFG_SSTART_DIS,
      S32K1XX_FLEXIO_SHIFTCFG0_OFFSET + priv->config->rx_inst * 0x4);

  /* Configure receive using Timer X on falling edge
   * of clock with input data on Pin X
   */

  flexio_putreg32(priv, FLEXIO_SHIFTCTL_TIMSEL(priv->config->timer) |
      FLEXIO_SHIFTCTL_TIMPOL_NE |
      FLEXIO_SHIFTCTL_PINCFG_DIS |
      FLEXIO_SHIFTCTL_PINSEL(priv->config->flexio_sda_pin) |
      FLEXIO_SHIFTCTL_SMOD_RX,
      S32K1XX_FLEXIO_SHIFTCTL0_OFFSET + priv->config->rx_inst * 0x4);

  /* Configure start bit, stop bit, enable on trigger high,
   * disable on compare, reset if output equals pin.
   * Initial clock state is logic 0 and is not affected by reset.
   */

  flexio_putreg32(priv, FLEXIO_TIMCFG_TSTART_ENA |
      FLEXIO_TIMCFG_TSTOP_TIMDIS |
      FLEXIO_TIMCFG_TIMENA_TRGHI |
      FLEXIO_TIMCFG_TIMDIS_TIMCMP |
      FLEXIO_TIMCFG_TIMRST_PINOUT |
      FLEXIO_TIMCFG_TIMOUT_ZERO,
      S32K1XX_FLEXIO_TIMCFG0_OFFSET + priv->config->two_word_timer * 0x4);

  /* Configure dual 8-bit counter using Pin 1 output enable (SCL open drain),
   * with Shifter 0 flag as the inverted trigger.
   */

  flexio_putreg32(priv, FLEXIO_TIMCTL_TRGSEL_SHIFTER(priv->config->tx_inst) |
      FLEXIO_TIMCTL_TRGPOL_LO |
      FLEXIO_TIMCTL_TRGSRC_INT |
      FLEXIO_TIMCTL_PINCFG_OD |
      FLEXIO_TIMCTL_PINSEL(priv->config->flexio_scl_pin) |
      FLEXIO_TIMCTL_TIMOD_8BBAUD,
      S32K1XX_FLEXIO_TIMCTL0_OFFSET + priv->config->two_word_timer * 0x4);

  /* Configure 8-bit transfer. Set TIMCMP[15:0] = (number of bits x 2) - 1. */

  flexio_putreg32(priv, 0xf, S32K1XX_FLEXIO_TIMCMP0_OFFSET +
                             priv->config->timer * 0x4);

  /* Enable when Timer 0 is enabled, disable when Timer 0 is disabled,
   * enable start bit and stop bit at end of each word,
   * decrement on pin input..
   */

  flexio_putreg32(priv, FLEXIO_TIMCFG_TSTART_ENA |
      FLEXIO_TIMCFG_TSTOP_TIMCMP |
      FLEXIO_TIMCFG_TIMENA_TIMENA |
      FLEXIO_TIMCFG_TIMDIS_TIMDIS |
      FLEXIO_TIMCFG_TIMDEC_PINBOTHPIN,
      S32K1XX_FLEXIO_TIMCFG0_OFFSET + priv->config->timer * 0x4);

  /* Configure 16-bit counter using inverted Pin 1 input (SCL).. */

  flexio_putreg32(priv, FLEXIO_TIMCTL_TRGSEL_SHIFTER(priv->config->tx_inst) |
      FLEXIO_TIMCTL_TRGPOL_LO |
      FLEXIO_TIMCTL_TRGSRC_INT |
      FLEXIO_TIMCTL_PINSEL(priv->config->flexio_scl_pin) |
      FLEXIO_TIMCTL_PINPOL_LO |
      FLEXIO_TIMCTL_TIMOD_16BCNT,
      S32K1XX_FLEXIO_TIMCTL0_OFFSET + priv->config->timer * 0x4);

  flexio_modifyreg32(priv, S32K1XX_FLEXIO_CTRL_OFFSET, 0,
         FLEXIO_CTRL_FLEXEN);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_flexio_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *s32k1xx_flexio_i2cbus_initialize(int port)
{
  struct s32k1xx_flexio_i2cdev_s *priv;

  if (port == 0)
    {
      priv          = &g_i2c0dev;
      priv->dev.ops = &s32k1xx_flexio_i2c_ops;
    }
  else
    {
      i2cerr("I2C Only support 0\n");
      return NULL;
    }

  nxmutex_lock(&priv->lock);

  /* Test if already initialized or not */

  if (1 < ++priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return &priv->dev;
    }

  priv->port      = port;
  priv->frequency = 0;

  s32k1xx_get_pclkfreq(FLEXIO0_CLK, &priv->base_freq);

  /* Configure pins */

  s32k1xx_pinconfig(priv->config->scl_pin);
  s32k1xx_pinconfig(priv->config->sda_pin);

  s32k1xx_flexio_i2c_init(priv);

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, s32k1xx_flexio_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  nxmutex_unlock(&priv->lock);
  return &priv->dev;
}

/****************************************************************************
 * Name: s32k1xx_flexio_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int s32k1xx_flexio_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct s32k1xx_flexio_i2cdev_s *priv =
              (struct s32k1xx_flexio_i2cdev_s *)dev;

  /* Decrement reference count and check for underflow */

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

  up_disable_irq(priv->irqid);
  irq_detach(priv->irqid);

  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_flexio_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 * TODO: Multiple i2c_msg_s read operations with the same address are not
 * currently guaranteed.
 ****************************************************************************/

static int s32k1xx_flexio_i2c_transfer(struct i2c_master_s *dev,
                               struct i2c_msg_s *msgs, int count)
{
  struct s32k1xx_flexio_i2cdev_s *priv =
              (struct s32k1xx_flexio_i2cdev_s *)dev;
  int i;
  int ret    = 0;
  int semval = 0;
  struct timespec abs_time;

  if (msgs->frequency > I2C_MAX_FREQUENCY)
    {
      return -EINVAL;
    }

  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the I2C bus */

  nxmutex_lock(&priv->lock);

  /* Check wait semaphore value. If the value is not 0, the transfer can not
   * be performed normally.
   */

  ret = nxsem_get_value(&priv->wait, &semval);
  DEBUGASSERT(ret == OK && semval == 0);

  priv->total_bytes = 0;

  for (i = 0; i < count; i++)
    {
      priv->total_bytes += msgs[i].length;
    }

  /* FlexIO driver can only handle transfers of maximum 12 bytes */

  if (priv->total_bytes <= 12)
    {
      /* Pass msg descriptor via device context */

      priv->msgs  = msgs;
      priv->error = OK;
      set_expiretime(50, &abs_time);

      s32k1xx_flexio_i2c_set_freq_and_size(priv, msgs->frequency,
                                          priv->total_bytes);
      s32k1xx_flexio_i2c_set_nack(priv);
      s32k1xx_flexio_i2c_clear_errors(priv);

      priv->tx_done = 0;
      priv->tx_count = 0;

      if (msgs->flags & I2C_M_READ)
        {
          priv->reg_buff_offset = -1; /* Ignore readback addr */
          priv->state = INTSTATE_RECV_DATA;
          flexio_putreg32(priv, FLEXIO_ADDR_READ(msgs->addr),
                          S32K1XX_FLEXIO_SHIFTBUFBBS0_OFFSET +
                          priv->config->tx_inst * 0x4);
        }
      else
        {
          priv->reg_buff_offset = 0;
          priv->reg_buff_mod = 0;
          priv->state = INTSTATE_WRITE_DATA;
          flexio_putreg32(priv, FLEXIO_ADDR_WRITE(msgs->addr),
                          S32K1XX_FLEXIO_SHIFTBUFBBS0_OFFSET +
                          priv->config->tx_inst * 0x4);
        }

      enable_shifter_status_interrupts(priv, FLEXIO_TX_IRQ | FLEXIO_RX_IRQ);
      nxsem_timedwait_uninterruptible(&priv->wait, &abs_time);

      if (priv->error != OK)
        {
          ret = priv->error;
        }
    }
  else
    {
      /* FlexIO driver can only handle transfers of maximum 12 bytes */

      ret = -EINVAL;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

#endif /* CONFIG_S32K1XX_FLEXIO_I2C */
