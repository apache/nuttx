/*****************************************************************************
 * arch/arm/src/xmc4/xmc4_i2c.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/mutex.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/xmc4_pinmux.h"
#include "hardware/xmc4_memorymap.h"
#include "hardware/xmc4_usic.h"
#include "xmc4_i2c.h"
#include "xmc4_usic.h"
#include "xmc4_gpio.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_XMC4_USCI_I2C)

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define XMC_I2C_CLOCK_OVERSAMPLING_STANDARD (10U)
#define XMC_I2C_CLOCK_OVERSAMPLING_FAST (25U)
#define XMC_I2C_MAX_SPEED_STANDARD 100000UL
#define XMC_I2C_MAX_SPEED_FAST 400000
#define I2C_DEFAULT_FREQUENCY 400000

#define I2C_INPUT_DX_SDA (0U) /* I2C uses DX0 input stage as SDA */
#define I2C_INPUT_DX_SCL (1U) /* I2C uses DX1 input stage as SCL */

#define XMC_I2C_TDF_MASTER_SEND (0U)
#define XMC_I2C_TDF_SLAVE_SEND (1U << 8)
#define XMC_I2C_TDF_MASTER_RECEIVE_ACK (2U << 8)
#define XMC_I2C_TDF_MASTER_RECEIVE_NACK (3U << 8)
#define XMC_I2C_TDF_MASTER_START (4U << 8)
#define XMC_I2C_TDF_MASTER_RESTART (5U << 8)
#define XMC_I2C_TDF_MASTER_STOP (6U << 8)

#define XMC_I2C_CMD_WRITE (0U)
#define XMC_I2C_CMD_READ (1U)

#define I2C_WORDLENGTH (7U) /* 8 bits word length */
#define I2C_TRM_MODE (3U)   /* Shift and transfert config */

#define I2C_TDV_SET (1U) /* A transmission of data in TBUF \
                          * can be started if TDV = 1 */

/*****************************************************************************
 * Private Types
 *****************************************************************************/

typedef enum xmc_usic_ch_input
{
  XMC_USIC_CH_INPUT_DX0, /* DX0 input */
  XMC_USIC_CH_INPUT_DX1, /* DX1 input */
  XMC_USIC_CH_INPUT_DX2, /* DX2 input */
  XMC_USIC_CH_INPUT_DX3, /* DX3 input */
  XMC_USIC_CH_INPUT_DX4, /* DX4 input */
  XMC_USIC_CH_INPUT_DX5  /* DX5 input */
} xmc_usic_ch_input_t;

typedef enum xmc_usic_ch_source
{
  XMC_USIC_CH_INPUT_DXNA, /* DXnA source signal */
  XMC_USIC_CH_INPUT_DXNB, /* DXnB source signal */
  XMC_USIC_CH_INPUT_DXNC, /* DXnC source signal */
  XMC_USIC_CH_INPUT_DXND, /* DXnD source signal */
  XMC_USIC_CH_INPUT_DXNE, /* DXnE source signal */
  XMC_USIC_CH_INPUT_DXNF, /* DXnF source signal */
  XMC_USIC_CH_INPUT_DXNG, /* DXnG source signal */
  XMC_USIC_CH_INPUT_DXN1  /* Signal always high */
} xmc_usic_ch_source_t;

/* This structure represents the state of one i2c channel */

struct xmc4_i2cdev_s
{
  const struct i2c_ops_s *ops;       /* Generic I2C device operation */
  const uint32_t base;               /* Base address of registers */
  const enum usic_channel_e channel; /* USIC channel */

  uint32_t frequency; /* Current I2C frequency */

  xmc_usic_ch_source_t dx0_sda; /* Source signal of SDA */
  xmc_usic_ch_source_t dx1_scl; /* Source signal of SCL */
  uint32_t sda_gpio;            /* GPIO config of SDA */
  uint32_t scl_gpio;            /* GPIO config of SCL */

  mutex_t lock; /* Only one thread can access at a time */
  int refs;     /* Reference count */
};

/*****************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

static inline void xmc4_i2c_putreg32(struct xmc4_i2cdev_s *priv,
                                     uint32_t offset,
                                     uint32_t value);
static inline uint32_t xmc4_i2c_getreg32(struct xmc4_i2cdev_s *priv,
                                         uint32_t offset);
static inline void xmc4_i2c_modifyreg32(struct xmc4_i2cdev_s *priv,
                                        uint32_t offset,
                                        uint32_t clearbits,
                                        uint32_t setbits);

static int i2c_set_baudrate(struct xmc4_i2cdev_s *priv);
static int i2c_set_input_source(struct xmc4_i2cdev_s *priv);
static void i2c_start_channel(struct xmc4_i2cdev_s *priv);
static void i2c_stop_channel(struct xmc4_i2cdev_s *priv);
static bool i2c_get_status_flag(struct xmc4_i2cdev_s *priv,
                                const uint32_t flag_mask);
static int i2c_clear_status_flag(struct xmc4_i2cdev_s *priv,
                                  const uint32_t flag_mask);
static bool i2c_get_transmit_buffer_status(struct xmc4_i2cdev_s *priv);
static void i2c_fill_transmit_buffer(struct xmc4_i2cdev_s *priv,
                                      const uint32_t data);
static uint8_t i2c_get_receive_buffer(struct xmc4_i2cdev_s *priv);

static void i2c_start(struct xmc4_i2cdev_s *priv,
                      const uint16_t addr,
                      const bool read,
                      const bool restart);
static void i2c_stop(struct xmc4_i2cdev_s *priv);
static void i2c_master_transmit(struct xmc4_i2cdev_s *priv,
                                const uint8_t data);
static void i2c_master_ack(struct xmc4_i2cdev_s *priv);
static void i2c_master_nack(struct xmc4_i2cdev_s *priv);
static void i2c_wait_for_slave_ack(struct xmc4_i2cdev_s *priv);
static void i2c_wait_for_received_data_ready(struct xmc4_i2cdev_s *priv);

static int i2c_initialize(struct xmc4_i2cdev_s *priv);
static int i2c_uninitialize(struct xmc4_i2cdev_s *priv);

static int i2c_transfer(struct i2c_master_s *dev,
                        struct i2c_msg_s *msgs,
                        int count);
#ifdef CONFIG_I2C_RESET
static int i2c_reset(struct i2c_master_s *dev);
#endif
/*****************************************************************************
 * Private Data
 *****************************************************************************/

struct i2c_ops_s xmc4_i2c_ops =
{
  .transfer = i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = i2c_reset,
#endif
};

#ifdef CONFIG_XMC4_I2C0
static struct xmc4_i2cdev_s g_i2c0 =
{
  .ops = &xmc4_i2c_ops,
  .base = XMC4_USIC0_CH0_BASE,
  .channel = USIC0_CHAN0,
  .dx0_sda = BOARD_I2C0_SDA_DX, /* See i2c_set_input_source */
  .dx1_scl = BOARD_I2C0_SCL_DX, /* See i2c_set_input_source */
  .sda_gpio = GPIO_I2C0_SDA,    /* See i2c_set_input_source */
  .scl_gpio = GPIO_I2C0_SCL,    /* See i2c_set_input_source */
  .frequency = (uint32_t)I2C_DEFAULT_FREQUENCY,
  .lock = NXMUTEX_INITIALIZER,
  .refs = 0,
};
#endif

#ifdef CONFIG_XMC4_I2C1
static struct xmc4_i2cdev_s g_i2c1 =
{
  .ops = &xmc4_i2c_ops,
  .base = XMC4_USIC0_CH1_BASE,
  .channel = USIC0_CHAN1,
  .dx0_sda = BOARD_I2C1_SDA_DX, /* See i2c_set_input_source */
  .dx1_scl = BOARD_I2C1_SCL_DX, /* See i2c_set_input_source */
  .sda_gpio = GPIO_I2C1_SDA,    /* See i2c_set_input_source */
  .scl_gpio = GPIO_I2C1_SCL,    /* See i2c_set_input_source */
  .frequency = (uint32_t)I2C_DEFAULT_FREQUENCY,
  .lock = NXMUTEX_INITIALIZER,
  .refs = 0,
};
#endif

#ifdef CONFIG_XMC4_I2C2
static struct xmc4_i2cdev_s g_i2c2 =
{
  .ops = &xmc4_i2c_ops,
  .base = XMC4_USIC1_CH0_BASE,
  .channel = USIC1_CHAN0,
  .dx0_sda = BOARD_I2C2_SDA_DX, /* See i2c_set_input_source */
  .dx1_scl = BOARD_I2C2_SCL_DX, /* See i2c_set_input_source */
  .sda_gpio = GPIO_I2C2_SDA,    /* See i2c_set_input_source */
  .scl_gpio = GPIO_I2C2_SCL,    /* See i2c_set_input_source */
  .frequency = (uint32_t)I2C_DEFAULT_FREQUENCY,
  .lock = NXMUTEX_INITIALIZER,
  .refs = 0,
};
#endif

#ifdef CONFIG_XMC4_I2C3
static struct xmc4_i2cdev_s g_i2c3 =
{
  .ops = &xmc4_i2c_ops,
  .base = XMC4_USIC1_CH1_BASE,
  .channel = USIC1_CHAN1,
  .dx0_sda = BOARD_I2C3_SDA_DX, /* See i2c_set_input_source */
  .dx1_scl = BOARD_I2C3_SCL_DX, /* See i2c_set_input_source */
  .sda_gpio = GPIO_I2C3_SDA,    /* See i2c_set_input_source */
  .scl_gpio = GPIO_I2C3_SCL,    /* See i2c_set_input_source */
  .frequency = (uint32_t)I2C_DEFAULT_FREQUENCY,
  .lock = NXMUTEX_INITIALIZER,
  .refs = 0,
};
#endif

#ifdef CONFIG_XMC4_I2C4
static struct xmc4_i2cdev_s g_i2c4 =
{
  .ops = &xmc4_i2c_ops,
  .base = XMC4_USIC2_CH0_BASE,
  .channel = USIC2_CHAN0,
  .dx0_sda = BOARD_I2C4_SDA_DX, /* See i2c_set_input_source */
  .dx1_scl = BOARD_I2C4_SCL_DX, /* See i2c_set_input_source */
  .sda_gpio = GPIO_I2C4_SDA,    /* See i2c_set_input_source */
  .scl_gpio = GPIO_I2C4_SCL,    /* See i2c_set_input_source */
  .frequency = (uint32_t)I2C_DEFAULT_FREQUENCY,
  .lock = NXMUTEX_INITIALIZER,
  .refs = 0,
};
#endif

#ifdef CONFIG_XMC4_I2C5
static struct xmc4_i2cdev_s g_i2c5 =
{
  .ops = &xmc4_i2c_ops,
  .base = XMC4_USIC2_CH1_BASE,
  .channel = USIC2_CHAN1,
  .dx0_sda = BOARD_I2C5_SDA_DX, /* See i2c_set_input_source */
  .dx1_scl = BOARD_I2C5_SCL_DX, /* See i2c_set_input_source */
  .sda_gpio = GPIO_I2C5_SDA,    /* See i2c_set_input_source */
  .scl_gpio = GPIO_I2C5_SCL,    /* See i2c_set_input_source */
  .frequency = (uint32_t)I2C_DEFAULT_FREQUENCY,
  .lock = NXMUTEX_INITIALIZER,
  .refs = 0,
};
#endif

/*****************************************************************************
 * Private Function
 *****************************************************************************/

/* I2C register related functions */

static inline void xmc4_i2c_putreg32(struct xmc4_i2cdev_s *priv,
                                     uint32_t offset,
                                     uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static inline uint32_t xmc4_i2c_getreg32(struct xmc4_i2cdev_s *priv,
                                         uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void xmc4_i2c_modifyreg32(struct xmc4_i2cdev_s *priv,
                                 uint32_t offset,
                                 uint32_t clearbits,
                                 uint32_t setbits)
{
  modifyreg32(priv->base + offset, clearbits, setbits);
}

/*****************************************************************************
 * Name: i2c_initialize
 *
 * Description:
 *   Initialize USIC channel for I2C operations.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int i2c_initialize(struct xmc4_i2cdev_s *priv)
{
  int ret;

  ret = xmc4_enable_usic_channel(priv->channel);

  /* Data format configuration */

  uint32_t sctr = ((uint32_t)I2C_TRM_MODE << (uint32_t)USIC_SCTR_TRM_SHIFT) |
                ((uint32_t)I2C_WORDLENGTH << (uint32_t)USIC_SCTR_WLE_SHIFT) |
                  (uint32_t)USIC_SCTR_FLE_MASK | /* unlimited data flow */
                  (uint32_t)USIC_SCTR_SDIR |     /* MSB shifted first */
                  (uint32_t)USIC_SCTR_PDL;       /* Passive Data Level */
  xmc4_i2c_putreg32(priv, XMC4_USIC_SCTR_OFFSET, sctr);

  /* Set baudrate */

  ret |= i2c_set_baudrate(priv);

  /* Enable transfer buffer */

  uint32_t tcsr = ((uint32_t)I2C_TDV_SET << (uint32_t)USIC_TCSR_TDEN_SHIFT) |
                                            (uint32_t)USIC_TCSR_TDSSM;
  xmc4_i2c_putreg32(priv, XMC4_USIC_TCSR_OFFSET, tcsr);

  /* Clear status flags */

  ret |= i2c_clear_status_flag(priv, 0x1ffff);

  /* Disable parity generation */

  xmc4_i2c_putreg32(priv, XMC4_USIC_CCR_OFFSET, 0);

  /* Set DX0m and DX1n as input sources for SDA & SCL */

  ret |= i2c_set_input_source(priv);

  /* Set USIC channel in I2C mode */

  i2c_start_channel(priv);

  /* Configure signal outputs pin */

  xmc4_gpio_config((gpioconfig_t)priv->sda_gpio); /* GPIO_I2C_SDA is DOUT0 */
  xmc4_gpio_config((gpioconfig_t)priv->scl_gpio); /* GPIO_I2C_SCL is SCLKOUT */

  return ret;
}

/*****************************************************************************
 * Name: i2c_uninitialize
 *
 * Description:
 *   Reset USIC channel.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int i2c_uninitialize(struct xmc4_i2cdev_s *priv)
{
  xmc4_i2c_putreg32(priv, XMC4_USIC_PSCR_OFFSET, 0);
  xmc4_i2c_putreg32(priv, XMC4_USIC_PCR_OFFSET, 0);
  xmc4_i2c_putreg32(priv, XMC4_USIC_CCR_OFFSET, 0);

  return xmc4_disable_usic_channel(priv->channel);
}

/*****************************************************************************
 * Name: i2c_set_baudrate
 *
 * Description:
 *   Set the baudrate of the USIC channel. Baudrate should be in [100-400k]Hz.
 *   Baudrate can't be changed while the USIC is running.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int i2c_set_baudrate(struct xmc4_i2cdev_s *priv)
{
  int ret;

  if (priv->frequency <= (uint32_t)XMC_I2C_MAX_SPEED_STANDARD)
    {
      xmc4_i2c_modifyreg32(priv, XMC4_USIC_PCR_OFFSET, USIC_PCR_IICMODE_STIM,
                          ~USIC_PCR_IICMODE_STIM);

      ret = xmc4_usic_baudrate(priv->channel, priv->frequency,
                              XMC_I2C_CLOCK_OVERSAMPLING_STANDARD);
    }
  else if (priv->frequency <= (uint32_t)XMC_I2C_MAX_SPEED_FAST)
    {
      xmc4_i2c_modifyreg32(priv, XMC4_USIC_PCR_OFFSET, USIC_PCR_IICMODE_STIM,
                          USIC_PCR_IICMODE_STIM);

      ret = xmc4_usic_baudrate(priv->channel, priv->frequency,
                              XMC_I2C_CLOCK_OVERSAMPLING_FAST);
    }
  else
    {
      ret = -EINVAL;
      i2cerr("Selected baudrate isn't supported : [100-400k]Hz\n");
    }

  return ret;
}

/*****************************************************************************
 * Name: i2c_set_input_source
 *
 * Description:
 *   Sets the input source for I2C. Defines the input stage for the
 *   corresponding input line. BOARD_I2Cx_SDA_DX, BOARD_I2Cx_SCL_DX,
 *   GPIO_I2Cx_SDA and GPIO_I2Cx_SCL must be defined in board.h
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int i2c_set_input_source(struct xmc4_i2cdev_s *priv)
{
  /* Configure I2C pins and input sources.
   *
   * NOTE that the board must provide the definitions in the board.h header
   * file of the form like:  GPIO_I2Cx_SDA and GPIO_I2Cx_SCL where x is
   * the I2C port number (U0C0 = 0, U0C1 = 1, U1C0 = 3 ... U2C1 = 5)
   *
   * In additional, the board.h must provide the definition of
   * BOARD_I2Cx_SDA_DX and BOARD_I2Cx_SCL_DX which indicates which input
   * source signal is selected, (0=DX_A, 1=DX_B, ... 6=DX_G).
   * Use enum uart_dx_e.
   */

  if ((priv->dx0_sda > XMC_USIC_CH_INPUT_DXN1) ||
      (priv->dx1_scl > XMC_USIC_CH_INPUT_DXN1))
    {
      return -EINVAL;
    }

  /* Set DX0 config */

  xmc4_i2c_modifyreg32(priv, XMC4_USIC_DX0CR_OFFSET,
                       USIC_DXCR_INSW, USIC_DXCR_DSEN);

  /* Set DX0 source [G:A] */

  xmc4_i2c_modifyreg32(priv, XMC4_USIC_DX0CR_OFFSET,
                       USIC_DXCR_DSEL_MASK, USIC_DXCR_DSEL_DX(priv->dx0_sda));

  /* Set DX1 config */

  xmc4_i2c_modifyreg32(priv, XMC4_USIC_DX1CR_OFFSET,
                       USIC_DXCR_INSW, USIC_DXCR_DSEN);

  /* Set DX1 source [G:A] */

  xmc4_i2c_modifyreg32(priv, XMC4_USIC_DX1CR_OFFSET,
                       USIC_DXCR_DSEL_MASK, USIC_DXCR_DSEL_DX(priv->dx1_scl));

  return OK;
}

/*****************************************************************************
 * Name: i2c_start_channel
 *
 * Description:
 *   Leave USIC channel from idle mode and set I2C mode.
 *
 *****************************************************************************/

static void i2c_start_channel(struct xmc4_i2cdev_s *priv)
{
  /* Sets the USIC input operation mode to I2C mode using CCR register. */

  xmc4_i2c_modifyreg32(priv, XMC4_USIC_CCR_OFFSET,
                       USIC_CCR_MODE_MASK, USIC_CCR_MODE_I2C);
}

/*****************************************************************************
 * Name: i2c_stop_channel
 *
 * Description:
 *   Wait for current transmit to end and set the USIC channel in idle mode.
 *
 *****************************************************************************/

static void i2c_stop_channel(struct xmc4_i2cdev_s *priv)
{
  /* Sets the USIC input operation mode to idle mode using CCR register. */

  while (i2c_get_transmit_buffer_status(priv) == 1)
    {
      /* wait for transmit to end */
    }

  xmc4_i2c_modifyreg32(priv, XMC4_USIC_CCR_OFFSET, USIC_CCR_MODE_MASK, 0);
}

/*****************************************************************************
 * Name: i2c_get_status_flag
 *
 * Description:
 *   Return the flags from PSR register.
 *
 * Returned Value:
 *   FALSE if selected flags are 0
 *   else TRUE if one of the selected flags isn't 0.
 *
 *****************************************************************************/

static bool i2c_get_status_flag(struct xmc4_i2cdev_s *priv,
                             const uint32_t flag_mask)
{
  uint32_t psr = xmc4_i2c_getreg32(priv, XMC4_USIC_PSR_OFFSET);

  return ((psr & flag_mask) != 0);
}

/*****************************************************************************
 * Name: i2c_clear_status_flag
 *
 * Description:
 *   Clear the flags from PSR register by writting in PSCR register.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int i2c_clear_status_flag(struct xmc4_i2cdev_s *priv,
                          const uint32_t flag_mask)
{
  if (flag_mask > 0x1ffff)
    {
      return -EINVAL;
    }

  xmc4_i2c_modifyreg32(priv, XMC4_USIC_PSCR_OFFSET, 0, flag_mask);

  return OK;
}

/*****************************************************************************
 * Name: i2c_get_transmit_buffer_status
 *
 * Description:
 *   Get the status of the transmit buffer.
 *
 * Returned Value:
 *   ZERO if idle, else ONE if busy.
 *
 *****************************************************************************/

static bool i2c_get_transmit_buffer_status(struct xmc4_i2cdev_s *priv)
{
  uint32_t tbuf_stat = xmc4_i2c_getreg32(priv, XMC4_USIC_TCSR_OFFSET);

  return (tbuf_stat & (uint32_t)USIC_TCSR_TDV);
}

/*****************************************************************************
 * Name: i2c_fill_transmit_buffer
 *
 * Description:
 *   Wait for transmit buffer to be ready and fill the transmit buffer
 *   with given word data.
 *   TODO : enable FIFO mechanism so you don't have to wait.
 *
 *****************************************************************************/

static void i2c_fill_transmit_buffer(struct xmc4_i2cdev_s *priv,
                                      const uint32_t data)
{
  /* Check FIFO size */

  uint32_t tbctr = xmc4_i2c_getreg32(priv, XMC4_USIC_TBCTR_OFFSET);

  if ((tbctr & USIC_TBCTR_SIZE_MASK) == 0)
    {
      /* FIFO mechanism is disabled */

      while (i2c_get_transmit_buffer_status(priv) == 1)
        {
          /* check TDV, wait until TBUF is ready */
        }

      /* Clear Status Flag */

      i2c_clear_status_flag(priv, (uint32_t)USIC_PSR_IICMODE_TBIF);

      /* Put data in TBUF[0] */

      xmc4_i2c_putreg32(priv, XMC4_USIC_TBUF_OFFSET, data);
    }
  else
    {
      /* Put data in IN[0] */

      xmc4_i2c_putreg32(priv, XMC4_USIC_IN_OFFSET, data);
    }
}

/*****************************************************************************
 * Name: i2c_get_receive_buffer
 *
 * Description:
 *   Get the data word from receive buffer.
 *
 * Returned Value:
 *   Recived data word in uint8_t.
 *
 *****************************************************************************/

static uint8_t i2c_get_receive_buffer(struct xmc4_i2cdev_s *priv)
{
  uint8_t retval;

  /* Check FIFO size */

  uint32_t rbctr = xmc4_i2c_getreg32(priv, XMC4_USIC_RBCTR_OFFSET);
  if ((rbctr & USIC_RBCTR_SIZE_SHIFT) == 0U)
    {
      retval = (uint8_t)xmc4_i2c_getreg32(priv, XMC4_USIC_RBUF_OFFSET);
    }
  else
    {
      retval = (uint8_t)xmc4_i2c_getreg32(priv, XMC4_USIC_OUTR_OFFSET);
    }

  return retval;
}

/*****************************************************************************
 * Name: i2c_start
 *
 * Description:
 *   Start and send an I2C frame with the given slave adress.
 *   Send the following on bus : [START/RESTART A6 A5 A4 A3 A2 A1 A0 R/W]
 *
 *****************************************************************************/

static void i2c_start(struct xmc4_i2cdev_s *priv,
               const uint16_t addr,
               const bool read,
               const bool restart)
{
  uint32_t sent_data = (addr << 1);

  /* Add start transition */

  if (restart)
    {
      sent_data |= XMC_I2C_TDF_MASTER_RESTART;
    }
  else
    {
      sent_data |= XMC_I2C_TDF_MASTER_START;
    }

  if (read)
    {
      sent_data |= 0x01;
    }

  i2c_fill_transmit_buffer(priv, sent_data);
}

/*****************************************************************************
 * Name: i2c_stop
 *
 * Description:
 *   Stop an I2C frame.
 *   Send the following on bus : [STOP]
 *
 *****************************************************************************/

static void i2c_stop(struct xmc4_i2cdev_s *priv)
{
  uint32_t buff_data = (uint32_t)XMC_I2C_TDF_MASTER_STOP;
  i2c_fill_transmit_buffer(priv, buff_data);
}

/*****************************************************************************
 * Name: i2c_master_transmit
 *
 * Description:
 *   Transmit data on bus.
 *   Send the following on bus : [D7 D6 D5 D4 D3 D2 D1 D0]
 *
 *****************************************************************************/

static void i2c_master_transmit(struct xmc4_i2cdev_s *priv,
                                const uint8_t data)
{
  uint32_t buff_data = (uint32_t)(data | (uint32_t)XMC_I2C_TDF_MASTER_SEND);
  i2c_fill_transmit_buffer(priv, buff_data);
}

/*****************************************************************************
 * Name: i2c_master_ack
 *
 * Description:
 *   Answer with a master acknowledge.
 *   Send the following on bus : [ACK]
 *
 *****************************************************************************/

static void i2c_master_ack(struct xmc4_i2cdev_s *priv)
{
  uint32_t buff_data = (uint32_t)XMC_I2C_TDF_MASTER_RECEIVE_ACK;
  i2c_fill_transmit_buffer(priv, buff_data);
}

/*****************************************************************************
 * Name: i2c_master_nack
 *
 * Description:
 *   Answer with a master not-acknowledge.
 *   Send the following on bus : [NACK]
 *
 *****************************************************************************/

static void i2c_master_nack(struct xmc4_i2cdev_s *priv)
{
  uint32_t buff_data = (uint32_t)XMC_I2C_TDF_MASTER_RECEIVE_NACK;
  i2c_fill_transmit_buffer(priv, buff_data);
}

/*****************************************************************************
 * Name: i2c_wait_for_slave_ack
 *
 * Description:
 *   Wait for the slave on bus to acknowledge.
 *
 *****************************************************************************/

static void i2c_wait_for_slave_ack(struct xmc4_i2cdev_s *priv)
{
  while (i2c_get_status_flag(priv, (uint32_t)USIC_PSR_IICMODE_ACK) == false)
    {
      /* Wait for ACK */
    }

  i2c_clear_status_flag(priv, (uint32_t)USIC_PSR_IICMODE_ACK);
}

/*****************************************************************************
 * Name: i2c_wait_for_received_data_ready
 *
 * Description:
 *   Wait for the received data buffer to be ready to be read.
 *
 *****************************************************************************/

static void i2c_wait_for_received_data_ready(struct xmc4_i2cdev_s *priv)
{
  while (i2c_get_status_flag(priv, (uint32_t)(USIC_PSR_IICMODE_RIF |
                                              USIC_PSR_IICMODE_AIF)) == false)
    {
      /* Wait for RIF */
    }

  i2c_clear_status_flag(priv, (uint32_t)(USIC_PSR_IICMODE_RIF |
                                         USIC_PSR_IICMODE_AIF));
}

/*****************************************************************************
 * Name: i2c_transfer
 *
 * Description:
 *   Generic I2C transfer.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int i2c_transfer(struct i2c_master_s *dev,
                        struct i2c_msg_s *msgs,
                        int count)
{
  struct xmc4_i2cdev_s *priv = (struct xmc4_i2cdev_s *)dev;

  int ret = OK;

  bool previous_msg_no_stop = false;

  /* Get exclusive access to the I2C bus */

  nxmutex_lock(&priv->lock);

  /* Enter critical section to avoid interrupts during i2c transfert */

  irqstate_t state = enter_critical_section();

  for (int i = 0; i < count; i++)
    {
      /* Check if frequency must be changed */

      if (msgs[i].frequency != priv->frequency)
        {
          priv->frequency = msgs[i].frequency;

          if (!previous_msg_no_stop)
            {
              i2c_stop_channel(priv);

              i2c_set_baudrate(priv);

              i2c_start_channel(priv);
            }
          else
            {
              i2cerr("Can't update frequency between Start & Stop symbols\n");
              leave_critical_section(state);
              nxmutex_unlock(&priv->lock);
              return -EINVAL;
            }
        }

      /* R/W bit */

      bool read = ((msgs[i].flags & I2C_M_READ) != 0);

      /* Check if you should start or restart a new i2c frame */

      if ((msgs[i].flags & I2C_M_NOSTART) == 0)
        {
          if (previous_msg_no_stop)
            {
              /* Repeated start */

              i2c_start(priv, msgs[i].addr, read, true);
              i2c_wait_for_slave_ack(priv);
            }
          else
            {
              /*  Normal start */

              i2c_start(priv, msgs[i].addr, read, false);
              i2c_wait_for_slave_ack(priv);
            }
        }

      /* Parse buffer data */

      for (int j = 0; j < (msgs[i].length); j++)
        {
          if (read)
            {
              /* Answer ack (or nack if last buffer) */

              if (j == ((msgs[i].length) - 1))
                {
                  i2c_master_nack(priv);
                }
              else
                {
                  i2c_master_ack(priv);
                }

              i2c_wait_for_received_data_ready(priv);

              /* Save received data */

              msgs[i].buffer[j] = i2c_get_receive_buffer(priv);
            }
          else
            {
              i2c_master_transmit(priv, msgs[i].buffer[j]);
              i2c_wait_for_slave_ack(priv);
            }
        }

      /* Check if you should not stop the i2c frame
      * msg has NO_STOP or next msg has NO_START flag
      */

      bool message_has_no_stop = ((msgs[i].flags & I2C_M_NOSTOP) != 0);

      bool next_message_has_no_start = false;

      if (i < count)
        {
          next_message_has_no_start =
                                ((msgs[i + 1].flags & I2C_M_NOSTART) != 0);
        }

      bool should_stop = !(message_has_no_stop || next_message_has_no_start);

      /* TODO : Should you stop when last msg and overide flags ? */

      if (should_stop)
        {
          i2c_stop(priv);
        }
      else
        {
          previous_msg_no_stop = true;
        }
    }

  leave_critical_section(state);
  nxmutex_unlock(&priv->lock);
  return ret;
}

#ifdef CONFIG_I2C_RESET
int i2c_reset(struct i2c_master_s *dev)
{
#error not implemented
}
#endif

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: xmc4_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 *****************************************************************************/

struct i2c_master_s *xmc4_i2cbus_initialize(int port)
{
  struct xmc4_i2cdev_s *priv = NULL;

  switch (port)
    {
#ifdef CONFIG_XMC4_I2C0
      case 0:
      {
        priv = (struct xmc4_i2cdev_s *)&g_i2c0;
        break;
      }
#endif

#ifdef CONFIG_XMC4_I2C1
      case 1:
      {
        priv = (struct xmc4_i2cdev_s *)&g_i2c1;
        break;
      }
#endif

#ifdef CONFIG_XMC4_I2C2
      case 2:
      {
        priv = (struct xmc4_i2cdev_s *)&g_i2c2;
        break;
      }
#endif

#ifdef CONFIG_XMC4_I2C3
      case 3:
      {
        priv = (struct xmc4_i2cdev_s *)&g_i2c3;
        break;
      }
#endif

#ifdef CONFIG_XMC4_I2C4
      case 4:
      {
        priv = (struct xmc4_i2cdev_s *)&g_i2c4;
        break;
      }
#endif

#ifdef CONFIG_XMC4_I2C5
      case 5:
      {
        priv = (struct xmc4_i2cdev_s *)&g_i2c5;
        break;
      }
#endif

      default:
      {
        i2cerr("No such i2c module.\n");
        return NULL;
      }
    }

  nxmutex_lock(&priv->lock);

  if (priv->refs++ == 0)
    {
      int ret = i2c_initialize(priv);
      if (ret < 0)
        {
          nxmutex_unlock(&priv->lock);
          return NULL;
        }
    }

  nxmutex_unlock(&priv->lock);

  return (struct i2c_master_s *)priv;
}

/*****************************************************************************
 * Name: xmc4_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 *****************************************************************************/

int xmc4_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct xmc4_i2cdev_s *priv = (struct xmc4_i2cdev_s *)dev;

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);

  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      kmm_free(dev);
      return OK;
    }

  i2c_uninitialize(priv);

  nxmutex_unlock(&priv->lock);

  kmm_free(dev);

  return OK;
}

#endif /* CONFIG_XMC4_USCI_I2C */
