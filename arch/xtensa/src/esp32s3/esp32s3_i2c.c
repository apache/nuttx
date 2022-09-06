/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_i2c.c
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

#ifdef CONFIG_ESP32S3_I2C

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>

#include "esp32s3_gpio.h"
#include "esp32s3_i2c.h"
#include "esp32s3_irq.h"

#include "xtensa.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_i2c.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_system.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Command format */

#define I2C_BASE_CMD(_cmd, _check_ack) (((_cmd) << 11) | \
                                        ((_check_ack) << 8))

#define I2C_SEND_CMD(_cmd, _check_ack, _bytes) (((_cmd) << 11) | \
                                                ((_check_ack) << 8) | \
                                                (_bytes))

#define I2C_RECV_CMD(_cmd, _ack_val, _bytes) (((_cmd) << 11) | \
                                              ((_ack_val) << 10) | \
                                              (_bytes))

/* Helper */

#ifdef CONFIG_I2C_POLLED
#define TIMESPEC_TO_US(sec, nano)  ((sec) * USEC_PER_SEC + (nano) / NSEC_PER_USEC)
#endif

#define ESP32S3_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_ESP32S3_I2CTIMEOSEC) + MSEC2TICK(CONFIG_ESP32S3_I2CTIMEOMS))

/* Default option */

#define I2C_FIFO_SIZE (32)

#define I2C_FILTER_CYC_NUM_DEF (7)

#define I2C_CLK_FREQ_DEF (100 * 1000)

#define I2C_INT_ERR_MASK (I2C_NACK_INT_ENA | \
                          I2C_TIME_OUT_INT_ENA | \
                          I2C_ARBITRATION_LOST_INT_ENA)

#define I2C_SCL_CYC_NUM_DEF 9

/* I2C event trace logic.
 * NOTE: trace uses the internal, non-standard, low-level debug interface
 * syslog() but does not require that any other debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define i2c_tracereset(p)
#  define i2c_tracenew(p,s)
#  define i2c_traceevent(p,e,a,s)
#  define i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C state */

enum esp32s3_i2cstate_e
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

enum i2c_opmode_e
{
  I2C_CMD_RESTART = 6,    /* I2C restart command */
  I2C_CMD_WRITE   = 1,    /* I2C write command */
  I2C_CMD_READ    = 3,    /* I2C read command */
  I2C_CMD_STOP    = 2,    /* I2C stop command */
  I2C_CMD_END     = 4     /* I2C end command */
};

#ifdef CONFIG_I2C_TRACE

/* Trace events */

enum esp32s3_trace_e
{
  I2CEVENT_NONE = 0,      /* No events have occurred with this status */
  I2CEVENT_SENDADDR,      /* Start/Master bit set and address sent, param = addr */
  I2CEVENT_SENDBYTE,      /* Send byte, param = bytes */
  I2CEVENT_RCVMODEEN,     /* Receive mode enabled, param = 0 */
  I2CEVENT_RCVBYTE,       /* Read more dta, param = bytes */
  I2CEVENT_STOP,          /* Last byte sten, send stop, param = length */
  I2CEVENT_ERROR          /* Error occurred, param = 0 */
};

/* Trace data */

struct esp32s3_trace_s
{
  uint32_t status;            /* I2C 32-bit SR status */
  uint32_t count;             /* Interrupt count when status change */
  enum esp32s3_trace_e event; /* Last event that occurred with this status */
  uint32_t parm;              /* Parameter associated with the event */
  clock_t time;               /* First of event or first status */
};

#endif /* CONFIG_I2C_TRACE */

/* I2C Device hardware configuration */

struct esp32s3_i2c_config_s
{
  uint32_t clk_freq;          /* Clock frequency */

  uint8_t scl_pin;            /* GPIO configuration for SCL as SCL */
  uint8_t sda_pin;            /* GPIO configuration for SDA as SDA */

#ifndef CONFIG_I2C_POLLED
  uint8_t periph;             /* Peripheral ID */
  uint8_t irq;                /* Interrupt ID */
#endif

  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t rst_bit;           /* I2C reset bit */

  uint32_t scl_insig;         /* I2C SCL input signal index */
  uint32_t scl_outsig;        /* I2C SCL output signal index */

  uint32_t sda_insig;         /* I2C SDA input signal index */
  uint32_t sda_outsig;        /* I2C SDA output signal index */
};

/* I2C Device Private Data */

struct esp32s3_i2c_priv_s
{
  const struct i2c_ops_s *ops; /* Standard I2C operations */

  uint32_t id;                 /* I2C instance */

  /* Port configuration */

  const struct esp32s3_i2c_config_s *config;
  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion mutex */

#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
  int cpuint;                  /* CPU interrupt assigned to this I2C */
  uint8_t cpu;                 /* CPU ID */
#endif

  /* I2C work state (see enum esp32s3_i2cstate_e) */

  volatile enum esp32s3_i2cstate_e i2cstate;

  struct i2c_msg_s *msgv;      /* Message list */

  uint8_t msgid;               /* Current message ID */
  ssize_t bytes;               /* Processed data bytes */

  uint32_t error;              /* I2C transform error */

  bool ready_read;             /* If I2C has read data */

  uint32_t clk_freq;           /* Current I2C Clock frequency */

#ifdef CONFIG_I2C_TRACE
  /* I2C trace support */

  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct esp32s3_trace_s trace[CONFIG_I2C_NTRACE];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void i2c_init_clock(struct esp32s3_i2c_priv_s *priv,
                           uint32_t clock);
static void i2c_init(struct esp32s3_i2c_priv_s *priv);
static void i2c_deinit(struct esp32s3_i2c_priv_s *priv);
static int i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs,
                        int count);
static inline void i2c_process(struct esp32s3_i2c_priv_s *priv,
                               uint32_t status);
#ifndef CONFIG_I2C_POLLED
static int i2c_sem_waitdone(struct esp32s3_i2c_priv_s *priv);
#endif
#ifdef CONFIG_I2C_POLLED
static int i2c_polling_waitdone(struct esp32s3_i2c_priv_s *priv);
#endif
static void i2c_clear_bus(struct esp32s3_i2c_priv_s *priv);
static void i2c_reset_fsmc(struct esp32s3_i2c_priv_s *priv);
#ifdef CONFIG_I2C_RESET
static int i2c_reset(struct i2c_master_s *dev);
#endif

#ifdef CONFIG_I2C_TRACE
static void i2c_tracereset(struct esp32s3_i2c_priv_s *priv);
static void i2c_tracenew(struct esp32s3_i2c_priv_s *priv, uint32_t status);
static void i2c_traceevent(struct esp32s3_i2c_priv_s *priv,
                           enum esp32s3_trace_e event,
                           uint32_t parm,
                           uint32_t status);
static void i2c_tracedump(struct esp32s3_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_ops_s g_esp32s3_i2c_ops =
{
  .transfer = i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = i2c_reset
#endif
};

/* I2C device structures */

#ifdef CONFIG_ESP32S3_I2C0
static const struct esp32s3_i2c_config_s g_esp32s3_i2c0_config =
{
  .clk_freq   = I2C_CLK_FREQ_DEF,
  .scl_pin    = CONFIG_ESP32S3_I2C0_SCLPIN,
  .sda_pin    = CONFIG_ESP32S3_I2C0_SDAPIN,
#ifndef CONFIG_I2C_POLLED
  .periph     = ESP32S3_PERIPH_I2C_EXT0,
  .irq        = ESP32S3_IRQ_I2C_EXT0,
#endif
  .clk_bit    = SYSTEM_I2C_EXT0_CLK_EN,
  .rst_bit    = SYSTEM_I2C_EXT0_RST,
  .scl_insig  = I2CEXT0_SCL_IN_IDX,
  .scl_outsig = I2CEXT0_SCL_OUT_IDX,
  .sda_insig  = I2CEXT0_SDA_IN_IDX,
  .sda_outsig = I2CEXT0_SDA_OUT_IDX
};

static struct esp32s3_i2c_priv_s g_esp32s3_i2c0_priv =
{
  .ops        = &g_esp32s3_i2c_ops,
  .id         = ESP32S3_I2C0,
  .config     = &g_esp32s3_i2c0_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .i2cstate   = I2CSTATE_IDLE,
  .msgv       = NULL,
  .msgid      = 0,
  .bytes      = 0,
  .ready_read = false
};
#endif /* CONFIG_ESP32S3_I2C0 */

#ifdef CONFIG_ESP32S3_I2C1
static const struct esp32s3_i2c_config_s g_esp32s3_i2c1_config =
{
  .clk_freq   = I2C_CLK_FREQ_DEF,
  .scl_pin    = CONFIG_ESP32S3_I2C1_SCLPIN,
  .sda_pin    = CONFIG_ESP32S3_I2C1_SDAPIN,
#ifndef CONFIG_I2C_POLLED
  .periph     = ESP32S3_PERIPH_I2C_EXT1,
  .irq        = ESP32S3_IRQ_I2C_EXT1,
#endif
  .clk_bit    = SYSTEM_I2C_EXT1_CLK_EN,
  .rst_bit    = SYSTEM_I2C_EXT1_RST,
  .scl_insig  = I2CEXT1_SCL_IN_IDX,
  .scl_outsig = I2CEXT1_SCL_OUT_IDX,
  .sda_insig  = I2CEXT1_SDA_IN_IDX,
  .sda_outsig = I2CEXT1_SDA_OUT_IDX
};

static struct esp32s3_i2c_priv_s g_esp32s3_i2c1_priv =
{
  .ops        = &g_esp32s3_i2c_ops,
  .id         = ESP32S3_I2C1,
  .config     = &g_esp32s3_i2c1_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
#endif
  .i2cstate   = I2CSTATE_IDLE,
  .msgv       = NULL,
  .msgid      = 0,
  .bytes      = 0,
  .ready_read = false
};
#endif /* CONFIG_ESP32S3_I2C1 */

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
 * Name: i2c_reset_fifo
 *
 * Description:
 *   Reset I2C RX and TX hardware FIFO.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_reset_fifo(struct esp32s3_i2c_priv_s *priv)
{
  uint32_t bits = I2C_TX_FIFO_RST | I2C_RX_FIFO_RST;

  modifyreg32(I2C_FIFO_CONF_REG(priv->id), 0, bits);
  modifyreg32(I2C_FIFO_CONF_REG(priv->id), bits, 0);
}

/****************************************************************************
 * Name: i2c_intr_enable
 *
 * Description:
 *   Enable I2C interrupts.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_intr_enable(struct esp32s3_i2c_priv_s *priv)
{
  putreg32(UINT32_MAX, I2C_INT_CLR_REG(priv->id));

  putreg32(I2C_TRANS_COMPLETE_INT_ENA | I2C_END_DETECT_INT_ENA |
           I2C_INT_ERR_MASK, I2C_INT_ENA_REG(priv->id));
}

/****************************************************************************
 * Name: i2c_intr_disable
 *
 * Description:
 *   Disable I2C interrupts.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_intr_disable(struct esp32s3_i2c_priv_s *priv)
{
  putreg32(0, I2C_INT_ENA_REG(priv->id));

  putreg32(UINT32_MAX, I2C_INT_CLR_REG(priv->id));
}

/****************************************************************************
 * Name: i2c_sendstart
 *
 * Description:
 *   Send I2C start signal.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_sendstart(struct esp32s3_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  /* Write I2C command registers */

  putreg32(I2C_BASE_CMD(I2C_CMD_RESTART, 0), I2C_COMD0_REG(priv->id));
  putreg32(I2C_SEND_CMD(I2C_CMD_WRITE, 1, 1), I2C_COMD1_REG(priv->id));
  putreg32(I2C_BASE_CMD(I2C_CMD_END, 0), I2C_COMD2_REG(priv->id));

  /* Write data to FIFO register */

  if ((msg->flags & I2C_M_READ) == 0)
    {
      putreg32(I2C_WRITEADDR8(msg->addr), I2C_DATA_REG(priv->id));
    }
  else
    {
      putreg32(I2C_READADDR8(msg->addr), I2C_DATA_REG(priv->id));
    }

  /* Enable I2C master TX interrupt */

  i2c_intr_enable(priv);

  /* Update I2C configuration */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_CONF_UPGATE);

  /* Configure the I2C to trigger a transaction */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_TRANS_START);
}

/****************************************************************************
 * Name: i2c_senddata
 *
 * Description:
 *   Send I2C data.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_senddata(struct esp32s3_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  int n = msg->length - priv->bytes;

  n = n < I2C_FIFO_SIZE ? n : I2C_FIFO_SIZE;

  putreg32(I2C_SEND_CMD(I2C_CMD_WRITE, 1, n), I2C_COMD0_REG(priv->id));
  putreg32(I2C_BASE_CMD(I2C_CMD_END, 0), I2C_COMD1_REG(priv->id));

  for (int i = 0; i < n; i++)
    {
      putreg32(msg->buffer[priv->bytes + i], I2C_DATA_REG(priv->id));
    }

  priv->bytes += n;

  /* Enable I2C master TX interrupt */

  i2c_intr_enable(priv);

  /* Update I2C configuration */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_CONF_UPGATE);

  /* Configure the I2C to trigger a transaction */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_TRANS_START);
}

/****************************************************************************
 * Name: i2c_recvdata
 *
 * Description:
 *   Transfer data from the FIFO to the driver buffer.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_recvdata(struct esp32s3_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  uint32_t cmd = getreg32(I2C_COMD0_REG(priv->id));
  uint8_t n = cmd & 0xff;
  uint32_t data = 0;

  for (int i = 0; i < n; i++)
    {
      data = getreg32(I2C_DATA_REG(priv->id));
      msg->buffer[priv->bytes + i] = data & 0xff;
    }

  priv->bytes += n;
}

/****************************************************************************
 * Name: i2c_startrecv
 *
 * Description:
 *   Configure I2C to prepare receiving data and it will create an interrupt
 *   to receive real data.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_startrecv(struct esp32s3_i2c_priv_s *priv)
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

  putreg32(I2C_RECV_CMD(I2C_CMD_READ, ack_value, n),
           I2C_COMD0_REG(priv->id));
  putreg32(I2C_BASE_CMD(I2C_CMD_END, 0), I2C_COMD1_REG(priv->id));

  /* Enable I2C master RX interrupt */

  i2c_intr_enable(priv);

  /* Update I2C configuration */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_CONF_UPGATE);

  /* Configure the I2C to trigger a transaction */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_TRANS_START);
}

/****************************************************************************
 * Name: i2c_sendstop
 *
 * Description:
 *   Send I2C STOP signal.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_sendstop(struct esp32s3_i2c_priv_s *priv)
{
  putreg32(I2C_BASE_CMD(I2C_CMD_STOP, 0), I2C_COMD0_REG(priv->id));

  /* Enable I2C master TX interrupt */

  i2c_intr_enable(priv);

  /* Update I2C configuration */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_CONF_UPGATE);

  /* Configure the I2C to trigger a transaction */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_TRANS_START);
}

/****************************************************************************
 * Name: i2c_init_clock
 *
 * Description:
 *   Initialize I2C hardware clock.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *   bus_freq      - Clock frequency of the I2C bus in Hz.
 *
 ****************************************************************************/

static void i2c_init_clock(struct esp32s3_i2c_priv_s *priv,
                           uint32_t bus_freq)
{
  if (bus_freq == priv->clk_freq)
    {
      return;
    }

  uint32_t reg_value = 0;
  uint32_t scl_low = 0;
  uint32_t scl_high = 0;
  uint32_t scl_wait_high = 0;
  uint32_t sda_hold = 0;
  uint32_t sda_sample = 0;
  uint32_t setup = 0;
  uint32_t hold = 0;
  uint32_t timeout = 0;
  uint32_t source_clk = XTAL_CLK_FREQ;
  uint32_t clkm_div = source_clk / (bus_freq * 1024) + 1;
  uint32_t sclk_freq = source_clk / clkm_div;
  uint32_t half_cycle = sclk_freq / bus_freq / 2;

  modifyreg32(I2C_CLK_CONF_REG(priv->id), I2C_SCLK_DIV_NUM_M,
              VALUE_TO_FIELD((clkm_div - 1), I2C_SCLK_DIV_NUM));

  /* According to the Technical Reference Manual, the following timings must
   * be subtracted by 1.
   * Moreover, the frequency calculation also shows that we must subtract 3
   * to the total SCL.
   */

  scl_low       = half_cycle;
  putreg32(scl_low - 1 - 2, I2C_SCL_LOW_PERIOD_REG(priv->id));

  /* By default, scl_wait_high must be less than scl_high.
   * A time compensation is needed for when the bus frequency is higher
   * than 50K.
   */

  scl_high      = (bus_freq <= 50000) ? half_cycle :
                                        (half_cycle / 5 * 4 + 4);
  scl_wait_high = half_cycle - scl_high;

  reg_value     = VALUE_TO_FIELD(scl_high - 1 - 1, I2C_SCL_HIGH_PERIOD);
  reg_value    |= VALUE_TO_FIELD(scl_wait_high - 1 - 1,
                                 I2C_SCL_WAIT_HIGH_PERIOD);
  putreg32(reg_value, I2C_SCL_HIGH_PERIOD_REG(priv->id));

  sda_hold      = half_cycle / 2;
  putreg32(sda_hold - 1, I2C_SDA_HOLD_REG(priv->id));

  /* scl_wait_high < sda_sample <= scl_high */

  sda_sample    = half_cycle / 2;
  putreg32(sda_sample - 1, I2C_SDA_SAMPLE_REG(priv->id));

  setup         = half_cycle;
  putreg32(setup - 1, I2C_SCL_RSTART_SETUP_REG(priv->id));
  putreg32(setup - 1, I2C_SCL_STOP_SETUP_REG(priv->id));

  hold          = half_cycle;
  putreg32(hold - 1, I2C_SCL_START_HOLD_REG(priv->id));
  putreg32(hold - 1, I2C_SCL_STOP_HOLD_REG(priv->id));

  /* By default, we set the timeout value to about 10 bus cycles
   * log(20*half_cycle)/log(2) = log(half_cycle)/log(2) +  log(20)/log(2)
   */

  timeout  = sizeof(half_cycle) * 8;
  timeout -= __builtin_clz(5 * half_cycle);
  timeout += 2;

  reg_value  = I2C_TIME_OUT_EN;
  reg_value |= VALUE_TO_FIELD(timeout, I2C_TIME_OUT_VALUE);
  putreg32(reg_value, I2C_TO_REG(priv->id));

  priv->clk_freq = bus_freq;
}

/****************************************************************************
 * Name: i2c_init
 *
 * Description:
 *   Initialize I2C hardware.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_init(struct esp32s3_i2c_priv_s *priv)
{
  const struct esp32s3_i2c_config_s *config = priv->config;

  esp32s3_gpiowrite(config->scl_pin, 1);
  esp32s3_configgpio(config->scl_pin, INPUT_PULLUP | OUTPUT_OPEN_DRAIN);
  esp32s3_gpio_matrix_out(config->scl_pin, config->scl_outsig, 0, 0);
  esp32s3_gpio_matrix_in(config->scl_pin, config->scl_insig, 0);

  esp32s3_gpiowrite(config->sda_pin, 1);
  esp32s3_configgpio(config->sda_pin, INPUT_PULLUP | OUTPUT_OPEN_DRAIN);
  esp32s3_gpio_matrix_out(config->sda_pin, config->sda_outsig, 0, 0);
  esp32s3_gpio_matrix_in(config->sda_pin, config->sda_insig, 0);

  /* Enable I2C hardware */

  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, config->clk_bit);
  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, config->rst_bit, 0);

  /* Disable I2C interrupts */

  i2c_intr_disable(priv);

  /* Initialize I2C Master */

  putreg32(I2C_MS_MODE | I2C_CLK_EN | I2C_SCL_FORCE_OUT | I2C_SDA_FORCE_OUT,
           I2C_CTR_REG(priv->id));

  /* Set FIFO mode */

  modifyreg32(I2C_FIFO_CONF_REG(priv->id), I2C_NONFIFO_EN, 0);

  /* Ensure I2C data mode is set to MSB */

  modifyreg32(I2C_CTR_REG(priv->id), I2C_TX_LSB_FIRST | I2C_RX_LSB_FIRST, 0);

  i2c_reset_fifo(priv);

  /* Configure the hardware filter function */

  putreg32(I2C_SCL_FILTER_EN | I2C_SDA_FILTER_EN |
           VALUE_TO_FIELD(I2C_FILTER_CYC_NUM_DEF, I2C_SCL_FILTER_THRES) |
             VALUE_TO_FIELD(I2C_FILTER_CYC_NUM_DEF, I2C_SDA_FILTER_THRES),
           I2C_FILTER_CFG_REG(priv->id));

  /* Set I2C source clock */

  modifyreg32(I2C_CLK_CONF_REG(priv->id), I2C_SCLK_SEL, 0);

  /* Configure I2C bus frequency */

  i2c_init_clock(priv, config->clk_freq);

  /* Update I2C configuration */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_CONF_UPGATE);
}

/****************************************************************************
 * Name: i2c_deinit
 *
 * Description:
 *   Disable I2C hardware.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_deinit(struct esp32s3_i2c_priv_s *priv)
{
  const struct esp32s3_i2c_config_s *config = priv->config;

  priv->clk_freq = 0;

  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, config->rst_bit);
  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, config->clk_bit, 0);
}

/****************************************************************************
 * Name: i2c_reset_fsmc
 *
 * Description:
 *   Reset I2C hardware state machine and registers.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_reset_fsmc(struct esp32s3_i2c_priv_s *priv)
{
  /* Reset FSM machine */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_FSM_RST);

  i2c_clear_bus(priv);
}

/****************************************************************************
 * Name: i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int i2c_sem_waitdone(struct esp32s3_i2c_priv_s *priv)
{
  /* Wait on ISR semaphore */

  return nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                        ESP32S3_I2CTIMEOTICKS);
}
#endif

/****************************************************************************
 * Name: i2c_polling_waitdone
 *
 * Description:
 *   Wait for a transfer to complete by polling status interrupt registers,
 *   which indicates the status of the I2C operations. This function is only
 *   used in polling driven mode.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 * Returned Values:
 *   Zero (OK) is returned on successfull transfer. -ETIMEDOUT is returned
 *   in case a transfer didn't finish within the timeout interval. And ERROR
 *   is returned in case of any I2C error during the transfer has happened.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_POLLED
static int i2c_polling_waitdone(struct esp32s3_i2c_priv_s *priv)
{
  int ret;
  clock_t current;
  clock_t timeout;
  uint32_t status = 0;

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

  while ((current < timeout) && (priv->error == 0))
    {
      /* Check if any interrupt triggered, clear them
       * process the operation.
       */

      status = getreg32(I2C_INT_STATUS_REG(priv->id));
      if (status != 0)
        {
          /* Check if the stop operation ended. Don't use
           * I2CSTATE_FINISH, because it is set before the stop
           * signal really ends. This works for interrupts because
           * the i2c_state is checked in the next interrupt when
           * stop signal has concluded. This is not the case of
           * polling.
           */

          if ((status & I2C_TRANS_COMPLETE_INT_ST) != 0)
            {
              putreg32(status, I2C_INT_CLR_REG(priv->id));
              break;
            }

          putreg32(status, I2C_INT_CLR_REG(priv->id));
          i2c_process(priv, status);
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
  else if (priv->error != 0)
    {
      ret = ERROR;
    }
  else
    {
      ret = OK;
    }

  /* Disable all interrupts */

  i2c_intr_disable(priv);

  return ret;
}
#endif

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function.
 *
 * Parameters:
 *   dev           - Device-specific state data
 *   msgs          - A pointer to a set of message descriptors
 *   count         - The number of transfers to perform
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs,
                        int count)
{
  int ret = OK;
  struct esp32s3_i2c_priv_s *priv = (struct esp32s3_i2c_priv_s *)dev;

  DEBUGASSERT(count > 0);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* If previous state is different than idle,
   * reset the FSMC to the idle state.
   */

  if (priv->i2cstate != I2CSTATE_IDLE)
    {
      i2c_reset_fsmc(priv);
      priv->i2cstate = I2CSTATE_IDLE;
    }

  /* Transfer the messages to the internal struct
   * and loop count times to make all transfers.
   */

  priv->msgv = msgs;

  for (int i = 0; i < count; i++)
    {
      /* Clear TX and RX FIFOs. */

      i2c_reset_fifo(priv);

      priv->bytes      = 0;
      priv->msgid      = i;
      priv->ready_read = false;
      priv->error      = 0;
      priv->i2cstate   = I2CSTATE_PROC;

      /* Set the SCLK frequency for the current msg. */

      i2c_init_clock(priv, msgs[i].frequency);

      if ((msgs[i].flags & I2C_M_NOSTART) != 0)
        {
          i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->bytes,
                         getreg32(I2C_SR_REG(priv->id)));
          i2c_senddata(priv);

          if (priv->bytes == msgs[i].length)
            {
              if ((msgs[i].flags & I2C_M_NOSTOP) == 0)
                {
                  priv->i2cstate = I2CSTATE_STOP;
                }
#ifndef CONFIG_I2C_POLLED
              else
                {
                  priv->i2cstate = I2CSTATE_FINISH;
                }
#endif
            }
        }
      else
        {
          /* Reset I2C trace logic */

          i2c_tracereset(priv);

          i2c_traceevent(priv, I2CEVENT_SENDADDR, msgs[i].addr,
                         getreg32(I2C_SR_REG(priv->id)));

          i2c_sendstart(priv);
        }

#ifndef CONFIG_I2C_POLLED
      if (i2c_sem_waitdone(priv) < 0)
        {
          /* Timed out - transfer was not completed within the timeout */

          i2cerr("Message %" PRIu8 " timed out.\n", priv->msgid);
          ret = -ETIMEDOUT;
          break;
        }
      else
        {
          if (priv->error != 0)
            {
              /* An error occurred */

              i2cerr("Transfer error %" PRIu32 "\n", priv->error);
              ret = -EIO;
              break;
            }
          else
            {
              /* Successful transfer, update the I2C state to idle */

              priv->i2cstate = I2CSTATE_IDLE;
              ret = OK;
            }
        }
#else
      ret = i2c_polling_waitdone(priv);
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

  i2c_tracedump(priv);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: i2c_clear_bus
 *
 * Description:
 *   Clear I2C bus, when the slave is stuck in a deadlock and keeps pulling
 *   the bus low, master can control the SCL bus to generate 9 CLKs.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void i2c_clear_bus(struct esp32s3_i2c_priv_s *priv)
{
  modifyreg32(I2C_SCL_SP_CONF_REG(priv->id),
              I2C_SCL_RST_SLV_EN | I2C_SCL_RST_SLV_NUM_M,
              VALUE_TO_FIELD(I2C_SCL_CYC_NUM_DEF, I2C_SCL_RST_SLV_NUM));

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_CONF_UPGATE);

  modifyreg32(I2C_SCL_SP_CONF_REG(priv->id), 0, I2C_SCL_RST_SLV_EN);
}

/****************************************************************************
 * Name: i2c_reset
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
static int i2c_reset(struct i2c_master_s *dev)
{
  struct esp32s3_i2c_priv_s *priv = (struct esp32s3_i2c_priv_s *)dev;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(priv->refs > 0);

  nxmutex_lock(&priv->lock);

  i2c_reset_fsmc(priv);

  /* Clear bus */

  i2c_clear_bus(priv);

  priv->i2cstate   = I2CSTATE_IDLE;
  priv->msgid      = 0;
  priv->bytes      = 0;
  priv->ready_read = false;

  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: i2c_traceclear
 *
 * Description:
 *   Set I2C trace fields to default value.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void i2c_traceclear(struct esp32s3_i2c_priv_s *priv)
{
  struct esp32s3_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;
  trace->count  = 0;
  trace->event  = I2CEVENT_NONE;
  trace->parm   = 0;
  trace->time   = 0;
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: i2c_tracereset
 *
 * Description:
 *   Reset the trace info for a new data collection.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void i2c_tracereset(struct esp32s3_i2c_priv_s *priv)
{
  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  i2c_traceclear(priv);
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: i2c_tracenew
 *
 * Description:
 *   Create a new trace entry.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *   status        - Current value of I2C status register.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void i2c_tracenew(struct esp32s3_i2c_priv_s *priv, uint32_t status)
{
  struct esp32s3_trace_s *trace = &priv->trace[priv->tndx];

  /* Check if the current entry is already initialized or if its status had
   * already changed
   */

  if (trace->count == 0 || status != trace->status)
    {
      /* Check whether the status changed */

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

      i2c_traceclear(priv);
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
 * Name: i2c_traceevent
 *
 * Description:
 *   Record a new trace event.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *   event         - Event to be recorded on the trace.
 *   parm          - Parameter associated with the event.
 *   status        - Current value of I2C status register.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void i2c_traceevent(struct esp32s3_i2c_priv_s *priv,
                           enum esp32s3_trace_e event,
                           uint32_t parm,
                           uint32_t status)
{
  /* Check for new trace setup */

  i2c_tracenew(priv, status);

  if (event != I2CEVENT_NONE)
    {
      struct esp32s3_trace_s *trace = &priv->trace[priv->tndx];

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
      i2c_traceclear(priv);
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: i2c_tracedump
 *
 * Description:
 *   Dump the trace results.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void i2c_tracedump(struct esp32s3_i2c_priv_s *priv)
{
  syslog(LOG_DEBUG, "Elapsed time: %" PRIu32 "\n",
         clock_systime_ticks() - priv->start_time);

  for (int i = 0; i < priv->tndx; i++)
    {
      struct esp32s3_trace_s *trace = &priv->trace[i];
      syslog(LOG_DEBUG,
             "%2d. STATUS: %08" PRIx32 " COUNT: %3" PRIu32 " EVENT: %s(%2d)"
             " PARM: %08" PRIx32 " TIME: %" PRIu32 "\n",
             i + 1, trace->status, trace->count, g_trace_names[trace->event],
             trace->event, trace->parm, trace->time - priv->start_time);
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: i2c_irq
 *
 * Description:
 *   This is the common I2C interrupt handler. It will be invoked when an
 *   interrupt is received on the device.
 *
 * Parameters:
 *   cpuint        - CPU interrupt index
 *   context       - Context data from the ISR
 *   arg           - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int i2c_irq(int cpuint, void *context, void *arg)
{
  struct esp32s3_i2c_priv_s *priv = (struct esp32s3_i2c_priv_s *)arg;

  /* Get the interrupt status and clear the interrupts that
   * triggered.
   */

  uint32_t irq_status = getreg32(I2C_INT_STATUS_REG(priv->id));
  putreg32(irq_status, I2C_INT_CLR_REG(priv->id));

  i2c_process(priv, irq_status);

  return OK;
}
#endif

/****************************************************************************
 * Name: i2c_process
 *
 * Description:
 *   This routine manages the transfer. It's called after some specific
 *   commands from the I2C controller are executed or in case of errors.
 *   It's responsible for writing/reading operations and transferring data
 *   from/to FIFO.
 *   It's called in the interrupt and polled driven mode.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *   status        - The current interrupt status register.
 *
 ****************************************************************************/

static void i2c_process(struct esp32s3_i2c_priv_s *priv, uint32_t status)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  /* Check for any errors */

  if ((I2C_INT_ERR_MASK & status) != 0)
    {
      /* Save the errors, register the error event, disable interrupts
       * and release the semaphore to conclude the transfer.
       */

      priv->error = status & I2C_INT_ERR_MASK;
      priv->i2cstate = I2CSTATE_ERROR;
      i2c_traceevent(priv, I2CEVENT_ERROR, priv->error,
                     getreg32(I2C_SR_REG(priv->id)));
      i2c_intr_disable(priv);
#ifndef CONFIG_I2C_POLLED
      nxsem_post(&priv->sem_isr);
#endif
    }
  else /* If no error */
    {
      /* If a transfer has just initialized */

      if (priv->i2cstate == I2CSTATE_PROC)
        {
          /* Check the flags to perform a read or write operation */

          if ((msg->flags & I2C_M_READ) != 0)
            {
              /* RX FIFO has available data */

              if (priv->ready_read)
                {
                  i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->bytes,
                                 getreg32(I2C_SR_REG(priv->id)));
                  i2c_recvdata(priv);

                  priv->ready_read = false;
                }

              /* Received all data. Finish the transaction
               * and update the I2C state.
               */

              if (priv->bytes == msg->length)
                {
                  i2c_traceevent(priv, I2CEVENT_STOP, msg->length,
                                 getreg32(I2C_SR_REG(priv->id)));
                  i2c_sendstop(priv);
#ifndef CONFIG_I2C_POLLED
                  priv->i2cstate = I2CSTATE_FINISH;
#endif
                }
              else /* Start a receive operation */
                {
                  i2c_traceevent(priv, I2CEVENT_RCVMODEEN, 0,
                                 getreg32(I2C_SR_REG(priv->id)));
                  i2c_startrecv(priv);

                  priv->ready_read = true;
                }
            }
          else /* Write operation */
            {
              i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->bytes,
                             getreg32(I2C_SR_REG(priv->id)));
              i2c_senddata(priv);

              /* Finally sent the entire message. Update the I2C state to
               * send a stop signal in the next interrupt.
               */

              if (priv->bytes == msg->length)
                {
                  if ((msg->flags & I2C_M_NOSTOP) == 0)
                    {
                      priv->i2cstate = I2CSTATE_STOP;
                    }
#ifndef CONFIG_I2C_POLLED
                  else
                    {
                      priv->i2cstate = I2CSTATE_FINISH;
                    }
#endif
                }
            }
        }
      else if (priv->i2cstate == I2CSTATE_STOP)
        {
          /* Transmitted all data. Finish the transaction sending a stop
           * and update the I2C state.
           */

          i2c_traceevent(priv, I2CEVENT_STOP, msg->length,
                         getreg32(I2C_SR_REG(priv->id)));
          i2c_sendstop(priv);
#ifndef CONFIG_I2C_POLLED
          priv->i2cstate = I2CSTATE_FINISH;
#endif
        }
#ifndef CONFIG_I2C_POLLED
      else if (priv->i2cstate == I2CSTATE_FINISH)
        {
          /* Disable all interrupts and release the semaphore */

          i2c_intr_disable(priv);
          nxsem_post(&priv->sem_isr);
        }
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a pointer to an unique
 *   instance of struct i2c_master_s. This function may be called to obtain
 *   multiple instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Parameters:
 *   port          - Port number of the I2C interface to be initialized.
 *
 * Returned Value:
 *   Pointer to valid I2C device structure is returned on success.
 *   A NULL pointer is returned on failure.
 *
 ****************************************************************************/

struct i2c_master_s *esp32s3_i2cbus_initialize(int port)
{
  struct esp32s3_i2c_priv_s *priv;
#ifndef CONFIG_I2C_POLLED
  const struct esp32s3_i2c_config_s *config;
  int ret;
#endif

  switch (port)
    {
#ifdef CONFIG_ESP32S3_I2C0
      case ESP32S3_I2C0:
        priv = &g_esp32s3_i2c0_priv;
        break;
#endif
#ifdef CONFIG_ESP32S3_I2C1
      case ESP32S3_I2C1:
        priv = &g_esp32s3_i2c1_priv;
        break;
#endif
      default:
        return NULL;
    }

  nxmutex_lock(&priv->lock);
  if (priv->refs++ != 0)
    {
      nxmutex_unlock(&priv->lock);
      return (struct i2c_master_s *)priv;
    }

#ifndef CONFIG_I2C_POLLED
  config = priv->config;

  /* Set up to receive peripheral interrupts on the current CPU */

  priv->cpu = up_cpu_index();
  priv->cpuint = esp32s3_setup_irq(priv->cpu, config->periph,
                                   1, ESP32S3_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type */

      priv->refs--;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  ret = irq_attach(config->irq, i2c_irq, priv);
  if (ret != OK)
    {
      esp32s3_teardown_irq(priv->cpu, config->periph, priv->cpuint);
      priv->refs--;

      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  up_enable_irq(config->irq);
#endif

  i2c_init(priv);
  nxmutex_unlock(&priv->lock);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: esp32s3_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port and power down the device.
 *
 * Parameters:
 *   dev           - Device structure as returned by
 *                   esp32s3_i2cbus_initialize()
 *
 * Returned Value:
 *   OK is returned on success. ERROR is returned when internal reference
 *   count mismatches or dev points to invalid hardware device.
 *
 ****************************************************************************/

int esp32s3_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct esp32s3_i2c_priv_s *priv = (struct esp32s3_i2c_priv_s *)dev;

  DEBUGASSERT(dev != NULL);

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
  esp32s3_teardown_irq(priv->cpu, priv->config->periph, priv->cpuint);
#endif

  i2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_ESP32S3_I2C */
