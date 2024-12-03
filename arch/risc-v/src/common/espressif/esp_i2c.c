/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_i2c.c
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

#ifdef CONFIG_ESPRESSIF_I2C_PERIPH

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <time.h>
#include <sys/time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "esp_i2c.h"
#include "esp_irq.h"
#include "esp_gpio.h"
#include "riscv_internal.h"

#include "periph_ctrl.h"
#include "hal/i2c_hal.h"
#include "hal/i2c_types.h"
#include "hal/i2c_ll.h"
#include "soc/system_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2c_periph.h"
#if defined(CONFIG_ESPRESSIF_ESP32H2) || defined(CONFIG_ESPRESSIF_ESP32C6)
#  include "soc/pcr_reg.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_ESP32H2
#  define SYSTEM_I2C_EXT0_CLK_EN PCR_I2C0_CLK_EN
#  define SYSTEM_I2C_EXT0_RST    PCR_I2C0_RST_EN
#  define SYSTEM_I2C_EXT1_CLK_EN PCR_I2C1_CLK_EN
#  define SYSTEM_I2C_EXT1_RST    PCR_I2C1_RST_EN
#endif

#ifdef CONFIG_ESPRESSIF_ESP32C6
#  define SYSTEM_I2C_EXT0_CLK_EN PCR_I2C_CLK_EN
#  define SYSTEM_I2C_EXT0_RST    PCR_I2C_RST_EN
#endif

#define GET_STATUS(hw) hw->sr.val

#define ESPRESSIF_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_ESPRESSIF_I2CTIMEOSEC) + MSEC2TICK(CONFIG_ESPRESSIF_I2CTIMEOMS))

/* I2C hardware FIFO depth */

#define I2C_FIFO_SIZE (32)

/* Number of bus cycles filtered by default */

#define I2C_FILTER_CYC_NUM_DEF (7)

/* Number of bus cycles for the master to generate when the slave is in
 * deadlock
 */

#define I2C_SCL_CYC_NUM_DEF (9)

/* I2C default clock frequency */

#define I2C_CLK_FREQ_DEF (100 * 1000)

/* Mask for the interrupt errors */

#define I2C_INT_ERR_MASK (I2C_NACK_INT_ENA_M | \
                          I2C_TIME_OUT_INT_ENA_M | \
                          I2C_ARBITRATION_LOST_INT_ENA_M)

/* I2C event trace logic.
 * NOTE: trace uses the internal, non-standard, low-level debug interface
 * syslog() but does not require that any other debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define esp_i2c_tracereset(p)
#  define esp_i2c_tracenew(p,s)
#  define esp_i2c_traceevent(p,e,a,s)
#  define esp_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C state */

enum esp_i2cstate_e
{
  I2CSTATE_IDLE = 0,
  I2CSTATE_PROC,
  I2CSTATE_STOP,
#ifndef CONFIG_I2C_POLLED
  I2CSTATE_FINISH,
#endif
  I2CSTATE_ERROR
};

/* I2C hardware command */

enum esp_i2c_opmode_e
{
  I2C_CMD_RESTART = 6, /* I2C restart command */
  I2C_CMD_WRITE = 1,   /* I2C write command */
  I2C_CMD_READ = 3,    /* I2C read command */
  I2C_CMD_STOP = 2,    /* I2C stop command */
  I2C_CMD_END = 4      /* I2C end command */
};

#ifdef CONFIG_I2C_TRACE

/* Trace events */

enum esp_trace_e
{
  I2CEVENT_NONE = 0,  /* No events have occurred with this status */
  I2CEVENT_SENDADDR,  /* Start/Master bit set and address sent, param = addr */
  I2CEVENT_SENDBYTE,  /* Send byte, param = bytes */
  I2CEVENT_RCVMODEEN, /* Receive mode enabled, param = 0 */
  I2CEVENT_RCVBYTE,   /* Read more dta, param = bytes */
  I2CEVENT_STOP,      /* Last byte sten, send stop, param = length */
  I2CEVENT_ERROR      /* Error occurred, param = 0 */
};

/* Trace data */

struct esp_trace_s
{
  uint32_t status;           /* I2C 32-bit SR status */
  uint32_t count;            /* Interrupt count when status change */
  enum esp_i2cstate_e event; /* Last event that occurred with this status */
  uint32_t parm;             /* Parameter associated with the event */
  clock_t time;              /* First of event or first status */
};

#endif /* CONFIG_I2C_TRACE */

/* I2C Device hardware configuration */

struct esp_i2c_config_s
{
  uint32_t clk_freq;    /* Clock frequency */

  uint8_t scl_pin;      /* GPIO configuration for SCL as SCL */
  uint8_t sda_pin;      /* GPIO configuration for SDA as SDA */

#ifndef CONFIG_I2C_POLLED
  uint8_t periph;      /* Peripheral ID */
  uint8_t irq;         /* Interrupt ID */
#endif

  uint32_t clk_bit;    /* Clock enable bit */
  uint32_t rst_bit;    /* I2C reset bit */

  uint32_t scl_insig;  /* I2C SCL input signal index */
  uint32_t scl_outsig; /* I2C SCL output signal index */

  uint32_t sda_insig;  /* I2C SDA input signal index */
  uint32_t sda_outsig; /* I2C SDA output signal index */
};

/* I2C Device Private Data */

struct esp_i2c_priv_s
{
  const struct i2c_ops_s *ops; /* Standard I2C operations */

  uint32_t id;                 /* I2C instance */

  /* Port configuration */

  const struct esp_i2c_config_s *config;
  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion mutex */

#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif

  /* I2C work state (see enum esp_i2cstate_e) */

  volatile enum esp_i2cstate_e i2cstate;

  struct i2c_msg_s *msgv;      /* Message list */

  uint8_t msgid;               /* Current message ID */
  ssize_t bytes;               /* Processed data bytes */

#ifndef CONFIG_I2C_POLLED
  int cpuint;                  /* CPU interrupt assigned to this I2C */
#endif

  uint32_t error;              /* I2C transform error */

  bool ready_read;             /* If I2C is ready for receiving data */

  uint32_t clk_freq;           /* Current I2C Clock frequency */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct esp_trace_s trace[CONFIG_I2C_NTRACE];

#endif
  i2c_hal_context_t *ctx;     /* Common layer struct */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp_i2c_reset_fifo(struct esp_i2c_priv_s *priv);
static void esp_i2c_intr_enable(struct esp_i2c_priv_s *priv);
static void esp_i2c_intr_enable(struct esp_i2c_priv_s *priv);
static void esp_i2c_intr_disable(struct esp_i2c_priv_s *priv);
static void esp_i2c_sendstart(struct esp_i2c_priv_s *priv);
static void esp_i2c_senddata(struct esp_i2c_priv_s *priv);
static void esp_i2c_recvdata(struct esp_i2c_priv_s *priv);
static void esp_i2c_startrecv(struct esp_i2c_priv_s *priv);
static void esp_i2c_sendstop(struct esp_i2c_priv_s *priv);
static void esp_i2c_init_clock(struct esp_i2c_priv_s *priv,
                               uint32_t bus_freq);
static void esp_i2c_init(struct esp_i2c_priv_s *priv);
static void esp_i2c_deinit(struct esp_i2c_priv_s *priv);
static void esp_i2c_reset_fsmc(struct esp_i2c_priv_s *priv);
static int esp_i2c_sem_waitdone(struct esp_i2c_priv_s *priv);
#ifdef CONFIG_I2C_POLLED
static int esp_i2c_polling_waitdone(struct esp_i2c_priv_s *priv);
#endif /* CONFIG_I2C_POLLED */
static int  esp_i2c_transfer(struct i2c_master_s *dev,
                             struct i2c_msg_s *msgs,
                             int count);
#ifdef CONFIG_I2C_RESET
static void esp_i2c_clear_bus(struct esp_i2c_priv_s *priv);
static int  esp_i2c_reset(struct i2c_master_s *dev);
#endif /* CONFIG_I2C_RESET */
#ifdef CONFIG_I2C_TRACE
static void esp_i2c_traceclear(struct esp_i2c_priv_s *priv);
static void esp_i2c_tracereset(struct esp_i2c_priv_s *priv);
static void esp_i2c_tracenew(struct esp_i2c_priv_s *priv,
                             uint32_t status);
static void esp_i2c_traceevent(struct esp_i2c_priv_s *priv,
                               enum esp_trace_e event,
                               uint32_t parm,
                               uint32_t status);
static void esp_i2c_tracedump(struct esp_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */
#ifndef CONFIG_I2C_POLLED
static int esp_i2c_irq(int cpuint, void *context, void *arg);
#endif /* CONFIG_I2C_POLLED */
static inline void esp_i2c_process(struct esp_i2c_priv_s *priv,
                                   uint32_t status);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_ops_s esp_i2c_ops =
{
  .transfer = esp_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = esp_i2c_reset
#endif
};

#ifdef CONFIG_ESPRESSIF_I2C0

i2c_hal_context_t i2c0_ctx =
{
  0
};

/* I2C device structure */

static const struct esp_i2c_config_s esp_i2c0_config =
{
  .clk_freq   = I2C_CLK_FREQ_DEF,
  .scl_pin    = CONFIG_ESPRESSIF_I2C0_SCLPIN,
  .sda_pin    = CONFIG_ESPRESSIF_I2C0_SDAPIN,
#ifndef CONFIG_I2C_POLLED
  .periph     = ETS_I2C_EXT0_INTR_SOURCE,
  .irq        = ESP_IRQ_I2C_EXT0,
#endif
  .clk_bit    = SYSTEM_I2C_EXT0_CLK_EN,
  .rst_bit    = SYSTEM_I2C_EXT0_RST,
  .scl_insig  = I2CEXT0_SCL_IN_IDX,
  .scl_outsig = I2CEXT0_SCL_OUT_IDX,
  .sda_insig  = I2CEXT0_SDA_IN_IDX,
  .sda_outsig = I2CEXT0_SDA_OUT_IDX
};

static struct esp_i2c_priv_s esp_i2c0_priv =
{
  .ops        = &esp_i2c_ops,
  .id         = 0,
  .config     = &esp_i2c0_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .i2cstate   = I2CSTATE_IDLE,
  .msgv       = NULL,
  .msgid      = 0,
  .bytes      = 0,
#ifndef CONFIG_I2C_POLLED
  .cpuint     = -ENOMEM,
#endif
  .error      = 0,
  .ready_read = false,
  .clk_freq   = 0,
  .ctx = &i2c0_ctx
};
#endif

#ifdef CONFIG_ESPRESSIF_I2C1

i2c_hal_context_t i2c1_ctx =
{
  0
};

/* I2C device structure */

static const struct esp_i2c_config_s esp_i2c1_config =
{
  .clk_freq   = I2C_CLK_FREQ_DEF,
  .scl_pin    = CONFIG_ESPRESSIF_I2C1_SCLPIN,
  .sda_pin    = CONFIG_ESPRESSIF_I2C1_SDAPIN,
#ifndef CONFIG_I2C_POLLED
  .periph     = ETS_I2C_EXT1_INTR_SOURCE,
  .irq        = ESP_IRQ_I2C_EXT1,
#endif
  .clk_bit    = SYSTEM_I2C_EXT1_CLK_EN,
  .rst_bit    = SYSTEM_I2C_EXT1_RST,
  .scl_insig  = I2CEXT1_SCL_IN_IDX,
  .scl_outsig = I2CEXT1_SCL_OUT_IDX,
  .sda_insig  = I2CEXT1_SDA_IN_IDX,
  .sda_outsig = I2CEXT1_SDA_OUT_IDX
};

static struct esp_i2c_priv_s esp_i2c1_priv =
{
  .ops        = &esp_i2c_ops,
  .id         = 1,
  .config     = &esp_i2c1_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .i2cstate   = I2CSTATE_IDLE,
  .msgv       = NULL,
  .msgid      = 0,
  .bytes      = 0,
#ifndef CONFIG_I2C_POLLED
  .cpuint     = -ENOMEM,
#endif
  .error      = 0,
  .ready_read = false,
  .clk_freq   = 0,
  .ctx        = &i2c1_ctx
};
#endif /* CONFIG_ESPRESSIF_I2C1 */

/* Trace events strings */

#ifdef CONFIG_I2C_TRACE
static const char *g_trace_names[] =
{
  "NONE      ",
  "SENDADDR  ",
  "SENDBYTE  ",
  "RCVMODEEN ",
  "RCVBYTE   ",
  "STOP      ",
  "ERROR     "
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_i2c_reset_fifo
 *
 * Description:
 *   Reset I2C RX and TX hardware FIFO.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_reset_fifo(struct esp_i2c_priv_s *priv)
{
  i2c_ll_txfifo_rst(priv->ctx->dev);
  i2c_ll_rxfifo_rst(priv->ctx->dev);
}

/****************************************************************************
 * Name: esp_i2c_intr_enable
 *
 * Description:
 *   Enable I2C interrupts.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_intr_enable(struct esp_i2c_priv_s *priv)
{
  i2c_ll_clear_intr_mask(priv->ctx->dev, I2C_LL_INTR_MASK);
  i2c_ll_enable_intr_mask(priv->ctx->dev, I2C_LL_MASTER_EVENT_INTR);
}

/****************************************************************************
 * Name: esp_i2c_intr_disable
 *
 * Description:
 *   Disable I2C interrupts.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_intr_disable(struct esp_i2c_priv_s *priv)
{
  i2c_ll_disable_intr_mask(priv->ctx->dev, I2C_LL_INTR_MASK);
  i2c_ll_clear_intr_mask(priv->ctx->dev, I2C_LL_INTR_MASK);
}

/****************************************************************************
 * Name: esp_i2c_sendstart
 *
 * Description:
 *   Send I2C start signal.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_sendstart(struct esp_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  uint32_t fifo_val = 0;
  i2c_ll_hw_cmd_t restart_cmd;
  i2c_ll_hw_cmd_t write_cmd;
  i2c_ll_hw_cmd_t end_cmd;

  /* Write I2C command registers */

  restart_cmd.op_code = I2C_LL_CMD_RESTART;

  write_cmd.byte_num = 1;
  write_cmd.ack_en = 1;
  write_cmd.op_code = I2C_LL_CMD_WRITE;

  end_cmd.op_code = I2C_LL_CMD_END;

  i2c_ll_write_cmd_reg(priv->ctx->dev, restart_cmd, 0);
  i2c_ll_write_cmd_reg(priv->ctx->dev, write_cmd, 1);
  i2c_ll_write_cmd_reg(priv->ctx->dev, end_cmd, 2);

  /* Write data to FIFO register */

  fifo_val = (msg->addr << 1) | (msg->flags & I2C_M_READ);
  i2c_ll_write_txfifo(priv->ctx->dev, (uint8_t *)&fifo_val, 1);

  /* Enable I2C master TX interrupt */

  esp_i2c_intr_enable(priv);

  /* Update I2C configuration */

  /* Configure the I2C to trigger a transaction */

  i2c_hal_master_trans_start(priv->ctx);
}

/****************************************************************************
 * Name: esp_i2c_senddata
 *
 * Description:
 *   Send I2C data.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_senddata(struct esp_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  int n = msg->length - priv->bytes;
  i2c_ll_hw_cmd_t write_cmd =
    {
      .ack_en = 1,
      .op_code = I2C_LL_CMD_WRITE
    };

  i2c_ll_hw_cmd_t end_cmd =
    {
      .op_code = I2C_LL_CMD_END
    };

  n = n < I2C_FIFO_SIZE ? n : I2C_FIFO_SIZE;

  write_cmd.byte_num = n;
  i2c_ll_write_cmd_reg(priv->ctx->dev, write_cmd, 0);
  i2c_ll_write_cmd_reg(priv->ctx->dev, end_cmd, 1);
  i2c_ll_write_txfifo(priv->ctx->dev, &msg->buffer[priv->bytes], n);
  i2c_ll_master_enable_tx_it(priv->ctx->dev);

  priv->bytes += n;

  /* Enable I2C master TX interrupt */

  esp_i2c_intr_enable(priv);

  /* Update I2C configuration */

  /* Configure the I2C to trigger a transaction */

  i2c_hal_master_trans_start(priv->ctx);
}

/****************************************************************************
 * Name: esp_i2c_recvdata
 *
 * Description:
 *   Transfer data from the FIFO to the driver buffer.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_recvdata(struct esp_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  uint32_t cmd = 0;
  uint8_t n = 0;
  uint32_t data = 0;

  i2c_ll_get_rxfifo_cnt(priv->ctx->dev, &cmd);
  n = cmd & 0xff;

  i2c_ll_read_rxfifo(priv->ctx->dev, &msg->buffer[priv->bytes], n);
  i2c_ll_master_enable_rx_it(priv->ctx->dev);

  priv->bytes += n;
}

/****************************************************************************
 * Name: esp_i2c_startrecv
 *
 * Description:
 *   Configure I2C to prepare receiving data and it will create an interrupt
 *   to receive real data.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_startrecv(struct esp_i2c_priv_s *priv)
{
  int ack_value = 0;
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  int n = msg->length - priv->bytes;
  i2c_ll_hw_cmd_t read_cmd;
  i2c_ll_hw_cmd_t end_cmd;

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

  read_cmd.byte_num = n;
  read_cmd.ack_val = ack_value;
  read_cmd.op_code = I2C_LL_CMD_READ;
  i2c_ll_write_cmd_reg(priv->ctx->dev, read_cmd, 0);

  end_cmd.op_code = I2C_LL_CMD_END;
  i2c_ll_write_cmd_reg(priv->ctx->dev, end_cmd, 1);

  /* Enable I2C master RX interrupt */

  esp_i2c_intr_enable(priv);

  /* Update I2C configuration */

  /* Configure the I2C to trigger a transaction */

  i2c_hal_master_trans_start(priv->ctx);
}

/****************************************************************************
 * Name: esp_i2c_sendstop
 *
 * Description:
 *   Send I2C STOP signal.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_sendstop(struct esp_i2c_priv_s *priv)
{
  i2c_ll_hw_cmd_t stop_cmd =
    {
      .op_code = I2C_LL_CMD_STOP
    };

  i2c_ll_write_cmd_reg(priv->ctx->dev, stop_cmd, 0);

  /* Enable I2C master TX interrupt */

  esp_i2c_intr_enable(priv);

  /* Update I2C configuration */

  /* Configure the I2C to trigger a transaction */

  i2c_hal_master_trans_start(priv->ctx);
}

/****************************************************************************
 * Name: esp_i2c_init_clock
 *
 * Description:
 *   Initialize I2C hardware clock.
 *
 * Parameters:
 *   priv     - Pointer to the internal driver state structure.
 *   bus_freq - Clock frequency of the I2C bus in Hz.
 *
 ****************************************************************************/

static void esp_i2c_init_clock(struct esp_i2c_priv_s *priv,
                               uint32_t bus_freq)
{
  if (bus_freq == priv->clk_freq)
    {
      return ;
    }

  i2c_clock_source_t src_clk = I2C_CLK_SRC_DEFAULT;
  i2c_hal_set_bus_timing(priv->ctx, priv->config->clk_freq,
                         src_clk, XTAL_CLK_FREQ);
  i2c_ll_update(priv->ctx->dev);
  priv->clk_freq = bus_freq;
}

/****************************************************************************
 * Name: esp_i2c_init
 *
 * Description:
 *   Initialize I2C hardware.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_init(struct esp_i2c_priv_s *priv)
{
  const struct esp_i2c_config_s *config = priv->config;

  /* Configure GPIO signals for I2C SCL and SDA pins */

  esp_gpiowrite(config->scl_pin, 1);
  esp_gpiowrite(config->sda_pin, 1);

  esp_configgpio(config->scl_pin, INPUT_PULLUP | OUTPUT_OPEN_DRAIN);
  esp_gpio_matrix_out(config->scl_pin, config->scl_outsig, 0, 0);
  esp_gpio_matrix_in(config->scl_pin, config->scl_insig, 0);

  esp_configgpio(config->sda_pin, INPUT_PULLUP | OUTPUT_OPEN_DRAIN);
  esp_gpio_matrix_out(config->sda_pin, config->sda_outsig, 0, 0);
  esp_gpio_matrix_in(config->sda_pin, config->sda_insig, 0);

  /* Enable I2C hardware */

  periph_module_enable(i2c_periph_signal[priv->id].module);

  i2c_hal_init(priv->ctx, priv->id);

  /* Disable I2C interrupts */

  esp_i2c_intr_disable(priv);

  /* Initialize I2C Master  */

  i2c_hal_master_init(priv->ctx);

  /* Configure the hardware filter function */

  i2c_ll_set_filter(priv->ctx->dev, I2C_FILTER_CYC_NUM_DEF);

  /* Initialize I2C bus clock */

  esp_i2c_init_clock(priv, config->clk_freq);
}

/****************************************************************************
 * Name: esp_i2c_deinit
 *
 * Description:
 *   Disable I2C hardware.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_deinit(struct esp_i2c_priv_s *priv)
{
  const struct esp_i2c_config_s *config = priv->config;

  priv->clk_freq = 0;
  i2c_hal_deinit(priv->ctx);
  periph_module_disable(i2c_periph_signal[priv->id].module);
}

/****************************************************************************
 * Name: esp_i2c_reset_fsmc
 *
 * Description:
 *   Reset I2C hardware state machine and registers.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_reset_fsmc(struct esp_i2c_priv_s *priv)
{
  /* Reset FSM machine */

  i2c_hal_master_fsm_rst(priv->ctx);
}

/****************************************************************************
 * Name: esp_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/
#ifndef CONFIG_I2C_POLLED
static int esp_i2c_sem_waitdone(struct esp_i2c_priv_s *priv)
{
  return nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                        ESPRESSIF_I2CTIMEOTICKS);
}
#endif

/****************************************************************************
 * Name: esp_i2c_polling_waitdone
 *
 * Description:
 *   Wait for a transfer to complete by polling status interrupt registers,
 *   which indicates the status of the I2C operations. This function is only
 *   used in polling driven mode.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 * Returned Values:
 *   Zero (OK) is returned on successfull transfer. -ETIMEDOUT is returned
 *   in case a transfer didn't finish within the timeout interval. And ERROR
 *   is returned in case of any I2C error during the transfer has happened.
 *
 ****************************************************************************/
#ifdef CONFIG_I2C_POLLED
static int esp_i2c_polling_waitdone(struct esp_i2c_priv_s *priv)
{
  int ret;
  clock_t current;
  clock_t timeout;
  uint32_t status = 0;
  i2c_intr_event_t event = 0;

  /* Get the current absolute time and add an offset as timeout.
   * Preferable to use monotonic, so in case the time changes,
   * the time reference is kept, i.e., current time can't jump
   * forward and backwards.
   */

  current = clock_systime_ticks();
  timeout = current + SEC2TICK(10);

  /* Loop while a transfer is in progress
   * and an error didn't occur within the timeout
   */

  while ((sclock_t)(current - timeout) < 0 && (priv->error == 0))
    {
      /* Check if any interrupt triggered, clear them
       * process the operation.
       */

      i2c_ll_get_intr_mask(priv->ctx->dev, &status);
      if (status != 0)
        {
          i2c_ll_master_get_event(priv->ctx->dev, &event);
          /* Check if the stop operation ended. Don't use
           * I2CSTATE_FINISH, because it is set before the stop
           * signal really ends. This works for interrupts because
           * the i2c_state is checked in the next interrupt when
           * stop signal has concluded. This is not the case of
           * polling.
           */

          if (event == I2C_INTR_EVENT_TRANS_DONE)
            {
              i2c_ll_clear_intr_mask(priv->ctx->dev, status);
              break;
            }

          i2c_ll_clear_intr_mask(priv->ctx->dev, status);
          esp_i2c_process(priv, status);
        }

      /* Update current time */

      current = clock_systime_ticks();
    }

  /* Return a negated value in case of timeout, and in the other scenarios
   * return a positive value.
   * The transfer function will check the status of priv to check the other
   * scenarios.
   */

  if (current >= timeout)
    {
      ret = -ETIMEDOUT;
    }
  else if (priv->error)
    {
      ret = ERROR;
    }
  else
    {
      ret = OK;
    }

  /* Disable all interrupts */

  esp_i2c_intr_disable(priv);

  return ret;
}
#endif

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: esp_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function.
 *
 * Parameters:
 *   dev   - Device-specific state data
 *   msgs  - A pointer to a set of message descriptors
 *   count - The number of transfers to perform
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int esp_i2c_transfer(struct i2c_master_s *dev,
                            struct i2c_msg_s *msgs,
                            int count)
{
  int ret = OK;
  struct esp_i2c_priv_s *priv = (struct esp_i2c_priv_s *)dev;
#ifdef CONFIG_I2C_TRACE
  uint32_t status = 0;
#endif

  i2cinfo("Starting transfer request of %d message(s):\n", count);

  DEBUGASSERT(count > 0);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->i2cstate != I2CSTATE_IDLE)
    {
      esp_i2c_reset_fsmc(priv);
      priv->i2cstate = I2CSTATE_IDLE;
    }

  priv->msgv = msgs;

  for (int i = 0; i < count; i++)
    {
      esp_i2c_reset_fifo(priv);

      priv->bytes = 0;
      priv->msgid = i;
      priv->ready_read = false;
      priv->error = 0;
      priv->i2cstate = I2CSTATE_PROC;

      i2cinfo("Sending message %" PRIu8 "...\n", priv->msgid);

      esp_i2c_init_clock(priv, msgs[i].frequency);
#ifdef CONFIG_I2C_TRACE
      status = GET_STATUS(priv->ctx->dev);
#endif
#ifndef CONFIG_I2C_POLLED
      if ((msgs[i].flags & I2C_M_NOSTART) != 0)
        {
          esp_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->bytes, status);
          esp_i2c_senddata(priv);

          if (priv->bytes == msgs[i].length)
            {
              priv->i2cstate = I2CSTATE_STOP;
              if ((msgs[i].flags & I2C_M_NOSTOP) != 0)
                {
                  priv->i2cstate = I2CSTATE_FINISH;
                }
            }
        }
      else
#endif
        {
          /* Reset I2C trace logic */

          esp_i2c_tracereset(priv);

          esp_i2c_traceevent(priv, I2CEVENT_SENDADDR, msgs[i].addr, status);

          esp_i2c_sendstart(priv);
        }

#ifndef CONFIG_I2C_POLLED
      if (esp_i2c_sem_waitdone(priv) < 0)
        {
          i2cerr("Message %" PRIu8 " timed out.\n", priv->msgid);
          ret = -ETIMEDOUT;
          break;
        }
      else
        {
          if (priv->error != 0)
            {
              i2cerr("Transfer error %" PRIu32 "\n", priv->error);
              ret = -EIO;
              break;
            }
          else
            {
              priv->i2cstate = I2CSTATE_IDLE;
              ret = OK;
            }
        }
#else
      ret = esp_i2c_polling_waitdone(priv);
      if (ret < 0)
        {
          if (ret == -ETIMEDOUT)
            {
              break;
            }
          else
            {
              ret = -EIO;
              break;
            }
        }
      else
        {
          /* Successful transfer, update the I2C state to idle */

          priv->i2cstate = I2CSTATE_IDLE;
          ret = OK;
        }
#endif

        i2cinfo("Message %" PRIu8 " transfer complete.\n", priv->msgid);
    }

  /* Dump the trace result */

  esp_i2c_tracedump(priv);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: esp_i2c_clear_bus
 *
 * Description:
 *   Clear I2C bus, when the slave is stuck in a deadlock and keeps pulling
 *   the bus low, master can control the SCL bus to generate 9 CLKs.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static void esp_i2c_clear_bus(struct esp_i2c_priv_s *priv)
{
  i2c_ll_master_clr_bus(priv->ctx->dev);
}
#endif

/****************************************************************************
 * Name: esp_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int esp_i2c_reset(struct i2c_master_s *dev)
{
  irqstate_t flags;
  struct esp_i2c_priv_s *priv = (struct esp_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  DEBUGASSERT(priv->refs > 0);

  flags = enter_critical_section();

  esp_i2c_reset_fsmc(priv);

  /* Clear bus */

  esp_i2c_clear_bus(priv);

  priv->i2cstate   = I2CSTATE_IDLE;
  priv->msgid      = 0;
  priv->bytes      = 0;
  priv->ready_read = false;

  leave_critical_section(flags);

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_i2c_traceclear
 *
 * Description:
 *   Set I2C trace fields to default value.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void esp_i2c_traceclear(struct esp_i2c_priv_s *priv)
{
  struct esp_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;
  trace->count  = 0;
  trace->event  = I2CEVENT_NONE;
  trace->parm   = 0;
  trace->time   = 0;
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: esp_i2c_tracereset
 *
 * Description:
 *   Reset the trace info for a new data collection.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void esp_i2c_tracereset(struct esp_i2c_priv_s *priv)
{
  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  esp_i2c_traceclear(priv);
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: esp_i2c_tracenew
 *
 * Description:
 *   Create a new trace entry.
 *
 * Parameters:
 *   priv   - Pointer to the internal driver state structure.
 *   status - Current value of I2C status register.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void esp_i2c_tracenew(struct esp_i2c_priv_s *priv,
                             uint32_t status)
{
  struct esp_trace_s *trace = &priv->trace[priv->tndx];

  /* Check if the current entry is already initialized or if its status had
   * already changed
   */

  if (trace->count == 0 || status != trace->status)
    {
      /* Check whether the status changed  */

      if (trace->count != 0)
        {
          /* Bump up the trace index (unless we are out of trace entries) */

          if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
            {
              i2cerr("ERROR: Trace table overflow\n");
              return;
            }

          priv->tndx++;
          trace = &priv->trace[priv->tndx];
        }

      /* Initialize the new trace entry */

      esp_i2c_traceclear(priv);
      trace->status = status;
      trace->count  = 1;
      trace->time   = clock_systime_ticks();
    }
  else
    {
      /* Just increment the count of times that we have seen this status */

      trace->count++;
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: esp_i2c_traceevent
 *
 * Description:
 *   Record a new trace event.
 *
 * Parameters:
 *   priv   - Pointer to the internal driver state structure.
 *   event  - Event to be recorded on the trace.
 *   parm   - Parameter associated with the event.
 *   status - Current value of I2C status register.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void esp_i2c_traceevent(struct esp_i2c_priv_s *priv,
                               enum esp_trace_e event,
                               uint32_t parm,
                               uint32_t status)
{
  /* Check for new trace setup */

  esp_i2c_tracenew(priv, status);

  if (event != I2CEVENT_NONE)
    {
      struct esp_trace_s *trace = &priv->trace[priv->tndx];

      /* Initialize the new trace entry */

      trace->event  = event;
      trace->parm   = parm;

      /* Bump up the trace index (unless we are out of trace entries) */

      if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
        {
          i2cerr("ERROR: Trace table overflow\n");
          return;
        }

      priv->tndx++;
      esp_i2c_traceclear(priv);
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: esp_i2c_tracedump
 *
 * Description:
 *   Dump the trace results.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void esp_i2c_tracedump(struct esp_i2c_priv_s *priv)
{
  struct esp_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %" PRIu32 "\n",
         (clock_systime_ticks() - priv->start_time));

  for (i = 0; i < priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog(LOG_DEBUG,
             "%2d. STATUS: %08" PRIx32 " COUNT: %3" PRIu32 " EVENT: %s(%2d)"
             " PARM: %08" PRIx32 " TIME: %" PRIu32 "\n",
             i + 1, trace->status, trace->count, g_trace_names[trace->event],
             trace->event, trace->parm, trace->time - priv->start_time);
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: esp_i2c_irq
 *
 * Description:
 *   This is the common I2C interrupt handler. It will be invoked when an
 *   interrupt is received on the device.
 *
 * Parameters:
 *   cpuint  - CPU interrupt index
 *   context - Context data from the ISR
 *   arg     - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/
#ifndef CONFIG_I2C_POLLED
static int esp_i2c_irq(int cpuint, void *context, void *arg)
{
  struct esp_i2c_priv_s *priv = (struct esp_i2c_priv_s *)arg;
  uint32_t irq_status = 0;

  i2c_ll_get_intr_mask(priv->ctx->dev, &irq_status);
  i2c_ll_clear_intr_mask(priv->ctx->dev, irq_status);
  esp_i2c_process(priv , irq_status);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_i2c_process
 *
 * Description:
 *   This routine manages the transfer. It's called after some specific
 *   commands from the I2C controller are executed or in case of errors.
 *   It's responsible for writing/reading operations and transferring data
 *   from/to FIFO.
 *   It's called in the interrupt and polled driven mode.
 *
 * Parameters:
 *   priv   - Pointer to the internal driver state structure.
 *   status - The current interrupt status register.
 *
 ****************************************************************************/

static inline void esp_i2c_process(struct esp_i2c_priv_s *priv,
                                   uint32_t irq_status)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
#ifdef CONFIG_I2C_TRACE
  uint32_t status = 0;
  status = GET_STATUS(priv->ctx->dev);
#endif
  /* Check for any errors */

  if (I2C_INT_ERR_MASK & irq_status)
    {
      priv->error = irq_status & I2C_INT_ERR_MASK;
      priv->i2cstate = I2CSTATE_ERROR;
      esp_i2c_traceevent(priv, I2CEVENT_ERROR, priv->error, status);
      esp_i2c_intr_disable(priv);
#ifndef CONFIG_I2C_POLLED
      nxsem_post(&priv->sem_isr);
#endif
    }
  else
    {
      if (priv->i2cstate == I2CSTATE_PROC)
        {
          if (msg->flags & I2C_M_READ)
            {
              if (priv->ready_read)
                {
                  esp_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->bytes,
                                     status);
                  esp_i2c_recvdata(priv);

                  priv->ready_read = false;
                }

              if (priv->bytes == msg->length)
                {
                  esp_i2c_traceevent(priv, I2CEVENT_STOP, msg->length,
                                     status);
                  esp_i2c_sendstop(priv);
#ifndef CONFIG_I2C_POLLED
                  priv->i2cstate = I2CSTATE_FINISH;
#endif
                }
              else
                {
                  esp_i2c_traceevent(priv, I2CEVENT_RCVMODEEN, 0,
                                     status);
                  esp_i2c_startrecv(priv);

                  priv->ready_read = true;
                }
            }
          else
            {
              esp_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->bytes,
                                 status);
              esp_i2c_senddata(priv);

              if (priv->bytes == msg->length)
                {
                  priv->i2cstate = I2CSTATE_STOP;
#ifndef CONFIG_I2C_POLLED
                  if ((msg->flags & I2C_M_NOSTOP) != 0)
                    {
                      priv->i2cstate = I2CSTATE_FINISH;
                    }
#endif
                }
            }
        }
      else if (priv->i2cstate == I2CSTATE_STOP)
        {
          esp_i2c_traceevent(priv, I2CEVENT_STOP, msg->length,
                             status);
          esp_i2c_sendstop(priv);

#ifndef CONFIG_I2C_POLLED
          priv->i2cstate = I2CSTATE_FINISH;
#endif
        }
#ifndef CONFIG_I2C_POLLED
      else if (priv->i2cstate == I2CSTATE_FINISH)
        {
          esp_i2c_intr_disable(priv);
          nxsem_post(&priv->sem_isr);
        }
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a pointer to an unique
 *   instance of struct i2c_master_s. This function may be called to obtain
 *   multiple instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Parameters:
 *   port - Port number of the I2C interface to be initialized.
 *
 * Returned Value:
 *   Pointer to valid I2C device structure is returned on success.
 *   A NULL pointer is returned on failure.
 *
 ****************************************************************************/

struct i2c_master_s *esp_i2cbus_initialize(int port)
{
  struct esp_i2c_priv_s *priv;
#ifndef CONFIG_I2C_POLLED
  const struct esp_i2c_config_s *config;
  int ret;
#endif

  switch (port)
    {
#ifdef CONFIG_ESPRESSIF_I2C0
    case ESPRESSIF_I2C0:
      priv = &esp_i2c0_priv;
      break;
#endif
#ifdef CONFIG_ESPRESSIF_I2C1
    case ESPRESSIF_I2C1:
      priv = &esp_i2c1_priv;
      break;
#endif
    default:
      return NULL;
    }

  nxmutex_lock(&priv->lock);

  if (priv->refs++ != 0)
    {
      nxmutex_unlock(&priv->lock);

      i2cinfo("Returning previously initialized I2C bus. "
              "Handler: %p\n", priv);

      return (struct i2c_master_s *)priv;
    }

#ifndef CONFIG_I2C_POLLED
  config = priv->config;
  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the previous IRQ */

      up_disable_irq(config->irq);
      esp_teardown_irq(config->periph, priv->cpuint);
    }

  priv->cpuint = esp_setup_irq(config->periph,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      priv->refs--;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  ret = irq_attach(config->irq, esp_i2c_irq, priv);
  if (ret != OK)
    {
      /* Failed to attach IRQ, free the allocated CPU interrupt */

      esp_teardown_irq(config->periph, priv->cpuint);
      priv->cpuint = -ENOMEM;
      priv->refs--;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  /* Enable the CPU interrupt that is linked to the I2C device. */

  up_enable_irq(config->irq);
#endif

  esp_i2c_init(priv);
  nxmutex_unlock(&priv->lock);

  i2cinfo("I2C bus initialized! Handler: %p\n", priv);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: esp_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port and power down the device.
 *
 * Parameters:
 *   dev - Device structure as returned by
 *         esp_i2cbus_initialize()
 *
 * Returned Value:
 *   OK is returned on success. ERROR is returned when internal reference
 *   count mismatches or dev points to invalid hardware device.
 *
 ****************************************************************************/

int esp_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct esp_i2c_priv_s *priv = (struct esp_i2c_priv_s *)dev;

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

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->irq);
  esp_teardown_irq(priv->config->periph, priv->cpuint);
  priv->cpuint = -ENOMEM;
#endif

  esp_i2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_ESPRESSIF_I2C_PERIPH */
