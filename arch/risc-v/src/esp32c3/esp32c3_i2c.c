/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_i2c.c
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

#ifdef CONFIG_ESP32C3_I2C

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

#include "esp32c3_i2c.h"
#include "esp32c3_irq.h"
#include "esp32c3_gpio.h"
#include "riscv_internal.h"
#include "hardware/esp32c3_gpio_sigmap.h"
#include "hardware/esp32c3_i2c.h"
#include "hardware/esp32c3_soc.h"
#include "hardware/esp32c3_system.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helper for applying the mask for a given register field.
 * Mask is determined by the macros suffixed with _V and _S from the
 * peripheral register description.
 */

#define I2C_VALUE_MASK(_val, _field) ((_val & (_field##_V)) << (_field##_S))

/* Command format */

#define I2C_BASE_CMD(_cmd, _check_ack) (((_cmd) << 11) + \
                                        ((_check_ack) << 8))

#define I2C_SEND_CMD(_cmd, _check_ack, _bytes) (((_cmd) << 11) + \
                                                ((_check_ack) << 8) + \
                                                (_bytes))

#define I2C_RECV_CMD(_cmd, _ack_val, _bytes) (((_cmd) << 11) + \
                                              ((_ack_val) << 10) + \
                                              (_bytes))

/* Helper */

#ifdef CONFIG_I2C_POLLED
#define TIMESPEC_TO_US(sec, nano)  ((sec * USEC_PER_SEC) + (nano / NSEC_PER_USEC))
#endif

#define ESP32C3_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_ESP32C3_I2CTIMEOSEC) + MSEC2TICK(CONFIG_ESP32C3_I2CTIMEOMS))

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
#  define esp32c3_i2c_tracereset(p)
#  define esp32c3_i2c_tracenew(p,s)
#  define esp32c3_i2c_traceevent(p,e,a,s)
#  define esp32c3_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C state */

enum esp32c3_i2cstate_e
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

enum esp32c3_i2c_opmode_e
{
  I2C_CMD_RESTART = 6,        /* I2C restart command */
  I2C_CMD_WRITE = 1,          /* I2C write command */
  I2C_CMD_READ = 3,           /* I2C read command */
  I2C_CMD_STOP = 2,           /* I2C stop command */
  I2C_CMD_END = 4             /* I2C end command */
};

#ifdef CONFIG_I2C_TRACE

/* Trace events */

enum esp32c3_trace_e
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

struct esp32c3_trace_s
{
  uint32_t status;               /* I2C 32-bit SR status */
  uint32_t count;                /* Interrupt count when status change */
  enum esp32c3_i2cstate_e event; /* Last event that occurred with this status */
  uint32_t parm;                 /* Parameter associated with the event */
  clock_t time;                  /* First of event or first status */
};

#endif /* CONFIG_I2C_TRACE */

/* I2C Device hardware configuration */

struct esp32c3_i2c_config_s
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

struct esp32c3_i2c_priv_s
{
  const struct i2c_ops_s *ops; /* Standard I2C operations */

  uint32_t id;                 /* I2C instance */

  /* Port configuration */

  const struct esp32c3_i2c_config_s *config;
  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion mutex */

#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif

  /* I2C work state (see enum esp32c3_i2cstate_e) */

  volatile enum esp32c3_i2cstate_e i2cstate;

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

  struct esp32c3_trace_s trace[CONFIG_I2C_NTRACE];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp32c3_i2c_init_clock(struct esp32c3_i2c_priv_s *priv,
                                   uint32_t clock);
static void esp32c3_i2c_init(struct esp32c3_i2c_priv_s *priv);
static void esp32c3_i2c_deinit(struct esp32c3_i2c_priv_s *priv);
static int  esp32c3_i2c_transfer(struct i2c_master_s *dev,
                                 struct i2c_msg_s *msgs,
                                 int count);
static inline void esp32c3_i2c_process(struct esp32c3_i2c_priv_s *priv,
                                       uint32_t status);
#ifdef CONFIG_I2C_POLLED
static int esp32c3_i2c_polling_waitdone(struct esp32c3_i2c_priv_s *priv);
#endif

#ifdef CONFIG_I2C_RESET
static int  esp32c3_i2c_reset(struct i2c_master_s *dev);
#endif /* CONFIG_I2C_RESET */

#ifdef CONFIG_I2C_TRACE
static void esp32c3_i2c_tracereset(struct esp32c3_i2c_priv_s *priv);
static void esp32c3_i2c_tracenew(struct esp32c3_i2c_priv_s *priv,
                                 uint32_t status);
static void esp32c3_i2c_traceevent(struct esp32c3_i2c_priv_s *priv,
                                   enum esp32c3_trace_e event,
                                   uint32_t parm,
                                   uint32_t status);
static void esp32c3_i2c_tracedump(struct esp32c3_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_ops_s esp32c3_i2c_ops =
{
  .transfer = esp32c3_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = esp32c3_i2c_reset
#endif
};

#ifdef CONFIG_ESP32C3_I2C0

/* I2C device structure */

static const struct esp32c3_i2c_config_s esp32c3_i2c0_config =
{
  .clk_freq   = I2C_CLK_FREQ_DEF,
  .scl_pin    = CONFIG_ESP32C3_I2C0_SCLPIN,
  .sda_pin    = CONFIG_ESP32C3_I2C0_SDAPIN,
#ifndef CONFIG_I2C_POLLED
  .periph     = ESP32C3_PERIPH_I2C_EXT0,
  .irq        = ESP32C3_IRQ_I2C_EXT0,
#endif
  .clk_bit    = SYSTEM_I2C_EXT0_CLK_EN,
  .rst_bit    = SYSTEM_I2C_EXT0_RST,
  .scl_insig  = I2CEXT0_SCL_IN_IDX,
  .scl_outsig = I2CEXT0_SCL_OUT_IDX,
  .sda_insig  = I2CEXT0_SDA_IN_IDX,
  .sda_outsig = I2CEXT0_SDA_OUT_IDX
};

static struct esp32c3_i2c_priv_s esp32c3_i2c0_priv =
{
  .ops        = &esp32c3_i2c_ops,
  .id         = 0,
  .config     = &esp32c3_i2c0_config,
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
  .clk_freq   = 0
};

#endif

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
 * Name: esp32c3_i2c_reset_fifo
 *
 * Description:
 *   Reset I2C RX and TX hardware FIFO.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp32c3_i2c_reset_fifo(struct esp32c3_i2c_priv_s *priv)
{
  uint32_t bits = (I2C_TX_FIFO_RST_M | I2C_RX_FIFO_RST_M);

  modifyreg32(I2C_FIFO_CONF_REG(priv->id), 0, bits);
  modifyreg32(I2C_FIFO_CONF_REG(priv->id), bits, 0);
}

/****************************************************************************
 * Name: esp32c3_i2c_intr_enable
 *
 * Description:
 *   Enable I2C interrupts.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp32c3_i2c_intr_enable(struct esp32c3_i2c_priv_s *priv)
{
  putreg32(UINT32_MAX, I2C_INT_CLR_REG(priv->id));

  putreg32(I2C_TRANS_COMPLETE_INT_ENA_M | I2C_END_DETECT_INT_ENA_M |
           I2C_INT_ERR_MASK, I2C_INT_ENA_REG(priv->id));
}

/****************************************************************************
 * Name: esp32c3_i2c_intr_disable
 *
 * Description:
 *   Disable I2C interrupts.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp32c3_i2c_intr_disable(struct esp32c3_i2c_priv_s *priv)
{
  putreg32(0, I2C_INT_ENA_REG(priv->id));

  putreg32(UINT32_MAX, I2C_INT_CLR_REG(priv->id));
}

/****************************************************************************
 * Name: esp32c3_i2c_sendstart
 *
 * Description:
 *   Send I2C start signal.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp32c3_i2c_sendstart(struct esp32c3_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  /* Write I2C command registers */

  putreg32(I2C_BASE_CMD(I2C_CMD_RESTART, 0), I2C_COMD0_REG(priv->id));
  putreg32(I2C_SEND_CMD(I2C_CMD_WRITE, 1, 1), I2C_COMD1_REG(priv->id));
  putreg32(I2C_BASE_CMD(I2C_CMD_END, 0), I2C_COMD2_REG(priv->id));

  /* Write data to FIFO register */

  putreg32((msg->addr << 1) | (msg->flags & I2C_M_READ),
           I2C_DATA_REG(priv->id));

  /* Enable I2C master TX interrupt */

  esp32c3_i2c_intr_enable(priv);

  /* Update I2C configuration */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_CONF_UPGATE_M);

  /* Configure the I2C to trigger a transaction */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_TRANS_START_M);
}

/****************************************************************************
 * Name: esp32c3_i2c_senddata
 *
 * Description:
 *   Send I2C data.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp32c3_i2c_senddata(struct esp32c3_i2c_priv_s *priv)
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

  esp32c3_i2c_intr_enable(priv);

  /* Update I2C configuration */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_CONF_UPGATE_M);

  /* Configure the I2C to trigger a transaction */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_TRANS_START_M);
}

/****************************************************************************
 * Name: esp32c3_i2c_recvdata
 *
 * Description:
 *   Transfer data from the FIFO to the driver buffer.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp32c3_i2c_recvdata(struct esp32c3_i2c_priv_s *priv)
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
 * Name: esp32c3_i2c_startrecv
 *
 * Description:
 *   Configure I2C to prepare receiving data and it will create an interrupt
 *   to receive real data.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp32c3_i2c_startrecv(struct esp32c3_i2c_priv_s *priv)
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

  esp32c3_i2c_intr_enable(priv);

  /* Update I2C configuration */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_CONF_UPGATE_M);

  /* Configure the I2C to trigger a transaction */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_TRANS_START_M);
}

/****************************************************************************
 * Name: esp32c3_i2c_sendstop
 *
 * Description:
 *   Send I2C STOP signal.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp32c3_i2c_sendstop(struct esp32c3_i2c_priv_s *priv)
{
  putreg32(I2C_BASE_CMD(I2C_CMD_STOP, 0), I2C_COMD0_REG(priv->id));

  /* Enable I2C master TX interrupt */

  esp32c3_i2c_intr_enable(priv);

  /* Update I2C configuration */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_CONF_UPGATE_M);

  /* Configure the I2C to trigger a transaction */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_TRANS_START_M);
}

/****************************************************************************
 * Name: esp32c3_i2c_init_clock
 *
 * Description:
 *   Initialize I2C hardware clock.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *   bus_freq      - Clock frequency of the I2C bus in Hz.
 *
 ****************************************************************************/

static void esp32c3_i2c_init_clock(struct esp32c3_i2c_priv_s *priv,
                                   uint32_t bus_freq)
{
  if (bus_freq == priv->clk_freq)
    {
      return ;
    }

  uint32_t reg_value = 0;
  uint32_t scl_wait_high = 0;
  uint32_t scl_high = 0;
  uint32_t timeout_cycles = 0;
  uint32_t source_clk = XTAL_CLK_FREQ;
  uint32_t clkm_div = (source_clk / (bus_freq * 1024)) + 1;
  uint32_t sclk_freq = source_clk / clkm_div;
  uint32_t half_cycle = sclk_freq / bus_freq / 2;

  reg_value = I2C_VALUE_MASK((clkm_div - 1), I2C_SCLK_DIV_NUM);
  modifyreg32(I2C_CLK_CONF_REG(priv->id),
              I2C_SCLK_SEL_M | I2C_SCLK_DIV_NUM_M, reg_value);

  putreg32(half_cycle - 1, I2C_SCL_LOW_PERIOD_REG(priv->id));

  /* By default, scl_wait_high must be less than scl_high.
   * A time compensation is needed for when the bus frequency is higher
   * than 50K.
   */

  scl_wait_high = (bus_freq <= 50000) ? 0 : (half_cycle / 8);
  scl_high      = half_cycle - scl_wait_high;

  reg_value  = I2C_VALUE_MASK(scl_high, I2C_SCL_HIGH_PERIOD);
  reg_value |= I2C_VALUE_MASK(scl_wait_high, I2C_SCL_WAIT_HIGH_PERIOD);
  putreg32(reg_value, I2C_SCL_HIGH_PERIOD_REG(priv->id));

  putreg32(half_cycle / 4, I2C_SDA_HOLD_REG(priv->id));

  /* scl_wait_high < sda_sample <= scl_high */

  putreg32(half_cycle / 2, I2C_SDA_SAMPLE_REG(priv->id));

  putreg32(half_cycle, I2C_SCL_RSTART_SETUP_REG(priv->id));
  putreg32(half_cycle, I2C_SCL_STOP_SETUP_REG(priv->id));
  putreg32(half_cycle - 1, I2C_SCL_START_HOLD_REG(priv->id));
  putreg32(half_cycle, I2C_SCL_STOP_HOLD_REG(priv->id));

  /* By default, we set the timeout value to about 10 bus cycles
   * log(20*half_cycle)/log(2) = log(half_cycle)/log(2) +  log(20)/log(2)
   */

  timeout_cycles  = sizeof(half_cycle) * 8;
  timeout_cycles -= __builtin_clz(5 * half_cycle);
  timeout_cycles += 2;

  reg_value  = I2C_TIME_OUT_EN_M;
  reg_value |= I2C_VALUE_MASK(timeout_cycles, I2C_TIME_OUT_REG);
  putreg32(reg_value, I2C_TO_REG(priv->id));

  priv->clk_freq = bus_freq;
}

/****************************************************************************
 * Name: esp32c3_i2c_init
 *
 * Description:
 *   Initialize I2C hardware.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp32c3_i2c_init(struct esp32c3_i2c_priv_s *priv)
{
  const struct esp32c3_i2c_config_s *config = priv->config;

  /* Configure GPIO signals for I2C SCL and SDA pins */

  esp32c3_gpiowrite(config->scl_pin, 1);
  esp32c3_gpiowrite(config->sda_pin, 1);

  esp32c3_configgpio(config->scl_pin, INPUT_PULLUP | OUTPUT_OPEN_DRAIN);
  esp32c3_gpio_matrix_out(config->scl_pin, config->scl_outsig, 0, 0);
  esp32c3_gpio_matrix_in(config->scl_pin, config->scl_insig, 0);

  esp32c3_configgpio(config->sda_pin, INPUT_PULLUP | OUTPUT_OPEN_DRAIN);
  esp32c3_gpio_matrix_out(config->sda_pin, config->sda_outsig, 0, 0);
  esp32c3_gpio_matrix_in(config->sda_pin, config->sda_insig, 0);

  /* Enable I2C hardware */

  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, config->clk_bit);
  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, config->rst_bit, 0);

  /* Disable I2C interrupts */

  esp32c3_i2c_intr_disable(priv);

  /* Initialize I2C Master  */

  putreg32(I2C_MS_MODE_M | I2C_CLK_EN_M |
           I2C_SCL_FORCE_OUT_M | I2C_SDA_FORCE_OUT_M,
           I2C_CTR_REG(priv->id));

  /* Set FIFO mode */

  modifyreg32(I2C_FIFO_CONF_REG(priv->id), I2C_NONFIFO_EN_M, 0);

  /* Ensure I2C data mode is set to MSB */

  modifyreg32(I2C_CTR_REG(priv->id),
              (I2C_TX_LSB_FIRST_M | I2C_RX_LSB_FIRST_M), 0);

  esp32c3_i2c_reset_fifo(priv);

  /* Configure the hardware filter function */

  putreg32(I2C_SCL_FILTER_EN_M | I2C_SDA_FILTER_EN_M |
           I2C_VALUE_MASK(I2C_FILTER_CYC_NUM_DEF,
                          I2C_SCL_FILTER_THRES) |
           I2C_VALUE_MASK(I2C_FILTER_CYC_NUM_DEF,
                          I2C_SDA_FILTER_THRES),
           I2C_FILTER_CFG_REG(priv->id));

  /* Initialize I2C bus clock */

  esp32c3_i2c_init_clock(priv, config->clk_freq);
}

/****************************************************************************
 * Name: esp32c3_i2c_deinit
 *
 * Description:
 *   Disable I2C hardware.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp32c3_i2c_deinit(struct esp32c3_i2c_priv_s *priv)
{
  const struct esp32c3_i2c_config_s *config = priv->config;

  priv->clk_freq = 0;

  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, config->rst_bit);
  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, config->clk_bit, 0);
}

/****************************************************************************
 * Name: esp32c3_i2c_reset_fsmc
 *
 * Description:
 *   Reset I2C hardware state machine and registers.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp32c3_i2c_reset_fsmc(struct esp32c3_i2c_priv_s *priv)
{
  /* Reset FSM machine */

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_FSM_RST_M);
}

/****************************************************************************
 * Name: esp32c3_i2c_sem_waitdone
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
static int esp32c3_i2c_sem_waitdone(struct esp32c3_i2c_priv_s *priv)
{
  return nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                        ESP32C3_I2CTIMEOTICKS);
}
#endif

/****************************************************************************
 * Name: esp32c3_i2c_polling_waitdone
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
static int esp32c3_i2c_polling_waitdone(struct esp32c3_i2c_priv_s *priv)
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

  while ((sclock_t)(current - timeout) < 0 && (priv->error == 0))
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

          if (status & I2C_TRANS_COMPLETE_INT_ST_M)
            {
              putreg32(status, I2C_INT_CLR_REG(priv->id));
              break;
            }

          putreg32(status, I2C_INT_CLR_REG(priv->id));
          esp32c3_i2c_process(priv, status);
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

  esp32c3_i2c_intr_disable(priv);

  return ret;
}
#endif

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_i2c_transfer
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

static int esp32c3_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs,
                                int count)
{
  int ret = OK;
  struct esp32c3_i2c_priv_s *priv = (struct esp32c3_i2c_priv_s *)dev;

  i2cinfo("Starting transfer request of %d message(s):\n", count);

  DEBUGASSERT(count > 0);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->i2cstate != I2CSTATE_IDLE)
    {
      esp32c3_i2c_reset_fsmc(priv);
      priv->i2cstate = I2CSTATE_IDLE;
    }

  priv->msgv = msgs;

  for (int i = 0; i < count; i++)
    {
      esp32c3_i2c_reset_fifo(priv);

      priv->bytes = 0;
      priv->msgid = i;
      priv->ready_read = false;
      priv->error = 0;
      priv->i2cstate = I2CSTATE_PROC;

      i2cinfo("Sending message %" PRIu8 "...\n", priv->msgid);

      esp32c3_i2c_init_clock(priv, msgs[i].frequency);

#ifndef CONFIG_I2C_POLLED
      if ((msgs[i].flags & I2C_M_NOSTART) != 0)
        {
          esp32c3_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->bytes,
                                 getreg32(I2C_SR_REG(priv->id)));
          esp32c3_i2c_senddata(priv);

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

          esp32c3_i2c_tracereset(priv);

          esp32c3_i2c_traceevent(priv, I2CEVENT_SENDADDR, msgs[i].addr,
                                 getreg32(I2C_SR_REG(priv->id)));

          esp32c3_i2c_sendstart(priv);
        }

#ifndef CONFIG_I2C_POLLED
      if (esp32c3_i2c_sem_waitdone(priv) < 0)
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
      ret = esp32c3_i2c_polling_waitdone(priv);
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

  esp32c3_i2c_tracedump(priv);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: esp32c3_i2c_clear_bus
 *
 * Description:
 *   Clear I2C bus, when the slave is stuck in a deadlock and keeps pulling
 *   the bus low, master can control the SCL bus to generate 9 CLKs.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static void esp32c3_i2c_clear_bus(struct esp32c3_i2c_priv_s *priv)
{
  uint32_t value = VALUE_TO_FIELD(I2C_SCL_CYC_NUM_DEF, I2C_SCL_RST_SLV_NUM);
  modifyreg32(I2C_SCL_SP_CONF_REG(priv->id), I2C_SCL_RST_SLV_NUM_M, value);

  modifyreg32(I2C_SCL_SP_CONF_REG(priv->id), 0, I2C_SCL_RST_SLV_EN_M);

  modifyreg32(I2C_CTR_REG(priv->id), 0, I2C_CONF_UPGATE_M);
}
#endif

/****************************************************************************
 * Name: esp32c3_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Parameters:
 *   dev           - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int esp32c3_i2c_reset(struct i2c_master_s *dev)
{
  irqstate_t flags;
  struct esp32c3_i2c_priv_s *priv = (struct esp32c3_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  DEBUGASSERT(priv->refs > 0);

  flags = enter_critical_section();

  esp32c3_i2c_reset_fsmc(priv);

  /* Clear bus */

  esp32c3_i2c_clear_bus(priv);

  priv->i2cstate   = I2CSTATE_IDLE;
  priv->msgid      = 0;
  priv->bytes      = 0;
  priv->ready_read = false;

  leave_critical_section(flags);

  return OK;
}
#endif

/****************************************************************************
 * Name: esp32c3_i2c_traceclear
 *
 * Description:
 *   Set I2C trace fields to default value.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void esp32c3_i2c_traceclear(struct esp32c3_i2c_priv_s *priv)
{
  struct esp32c3_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;
  trace->count  = 0;
  trace->event  = I2CEVENT_NONE;
  trace->parm   = 0;
  trace->time   = 0;
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: esp32c3_i2c_tracereset
 *
 * Description:
 *   Reset the trace info for a new data collection.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void esp32c3_i2c_tracereset(struct esp32c3_i2c_priv_s *priv)
{
  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  esp32c3_i2c_traceclear(priv);
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: esp32c3_i2c_tracenew
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
static void esp32c3_i2c_tracenew(struct esp32c3_i2c_priv_s *priv,
                                 uint32_t status)
{
  struct esp32c3_trace_s *trace = &priv->trace[priv->tndx];

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

      esp32c3_i2c_traceclear(priv);
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
 * Name: esp32c3_i2c_traceevent
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
static void esp32c3_i2c_traceevent(struct esp32c3_i2c_priv_s *priv,
                                   enum esp32c3_trace_e event,
                                   uint32_t parm,
                                   uint32_t status)
{
  /* Check for new trace setup */

  esp32c3_i2c_tracenew(priv, status);

  if (event != I2CEVENT_NONE)
    {
      struct esp32c3_trace_s *trace = &priv->trace[priv->tndx];

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
      esp32c3_i2c_traceclear(priv);
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: esp32c3_i2c_tracedump
 *
 * Description:
 *   Dump the trace results.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void esp32c3_i2c_tracedump(struct esp32c3_i2c_priv_s *priv)
{
  struct esp32c3_trace_s *trace;
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
 * Name: esp32c3_i2c_irq
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
static int esp32c3_i2c_irq(int cpuint, void *context, void *arg)
{
  struct esp32c3_i2c_priv_s *priv = (struct esp32c3_i2c_priv_s *)arg;
  uint32_t irq_status = getreg32(I2C_INT_STATUS_REG(priv->id));

  putreg32(irq_status, I2C_INT_CLR_REG(priv->id));
  esp32c3_i2c_process(priv , irq_status);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp32c3_i2c_process
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

static inline void esp32c3_i2c_process(struct esp32c3_i2c_priv_s *priv,
                                       uint32_t irq_status)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  /* Check for any errors */

  if (I2C_INT_ERR_MASK & irq_status)
    {
      priv->error = irq_status & I2C_INT_ERR_MASK;
      priv->i2cstate = I2CSTATE_ERROR;
      esp32c3_i2c_traceevent(priv, I2CEVENT_ERROR, priv->error,
                             getreg32(I2C_SR_REG(priv->id)));
      esp32c3_i2c_intr_disable(priv);
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
                  esp32c3_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->bytes,
                                         getreg32(I2C_SR_REG(priv->id)));
                  esp32c3_i2c_recvdata(priv);

                  priv->ready_read = false;
                }

              if (priv->bytes == msg->length)
                {
                  esp32c3_i2c_traceevent(priv, I2CEVENT_STOP, msg->length,
                                         getreg32(I2C_SR_REG(priv->id)));
                  esp32c3_i2c_sendstop(priv);
#ifndef CONFIG_I2C_POLLED
                  priv->i2cstate = I2CSTATE_FINISH;
#endif
                }
              else
                {
                  esp32c3_i2c_traceevent(priv, I2CEVENT_RCVMODEEN, 0,
                                         getreg32(I2C_SR_REG(priv->id)));
                  esp32c3_i2c_startrecv(priv);

                  priv->ready_read = true;
                }
            }
          else
            {
              esp32c3_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->bytes,
                                     getreg32(I2C_SR_REG(priv->id)));
              esp32c3_i2c_senddata(priv);

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
          esp32c3_i2c_traceevent(priv, I2CEVENT_STOP, msg->length,
                                 getreg32(I2C_SR_REG(priv->id)));
          esp32c3_i2c_sendstop(priv);

#ifndef CONFIG_I2C_POLLED
          priv->i2cstate = I2CSTATE_FINISH;
#endif
        }
#ifndef CONFIG_I2C_POLLED
      else if (priv->i2cstate == I2CSTATE_FINISH)
        {
          esp32c3_i2c_intr_disable(priv);
          nxsem_post(&priv->sem_isr);
        }
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_i2cbus_initialize
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

struct i2c_master_s *esp32c3_i2cbus_initialize(int port)
{
  struct esp32c3_i2c_priv_s *priv;
#ifndef CONFIG_I2C_POLLED
  const struct esp32c3_i2c_config_s *config;
  int ret;
#endif

  switch (port)
    {
#ifdef CONFIG_ESP32C3_I2C0
    case 0:
      priv = &esp32c3_i2c0_priv;
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
              "Handler: %" PRIxPTR "\n", (uintptr_t)priv);

      return (struct i2c_master_s *)priv;
    }

#ifndef CONFIG_I2C_POLLED
  config = priv->config;
  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(priv->cpuint);
    }

  priv->cpuint = esp32c3_request_irq(config->periph,
                                     ESP32C3_INT_PRIO_DEF,
                                     ESP32C3_INT_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      priv->refs--;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  ret = irq_attach(config->irq, esp32c3_i2c_irq, priv);
  if (ret != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp32c3_free_cpuint(config->periph);
      priv->cpuint = -ENOMEM;
      priv->refs--;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  /* Enable the CPU interrupt that is linked to the I2C device. */

  up_enable_irq(priv->cpuint);
#endif

  esp32c3_i2c_init(priv);
  nxmutex_unlock(&priv->lock);

  i2cinfo("I2C bus initialized! Handler: %" PRIxPTR "\n", (uintptr_t)priv);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: esp32c3_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port and power down the device.
 *
 * Parameters:
 *   dev           - Device structure as returned by
 *                   esp32c3_i2cbus_initialize()
 *
 * Returned Value:
 *   OK is returned on success. ERROR is returned when internal reference
 *   count mismatches or dev points to invalid hardware device.
 *
 ****************************************************************************/

int esp32c3_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct esp32c3_i2c_priv_s *priv = (struct esp32c3_i2c_priv_s *)dev;

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
  up_disable_irq(priv->cpuint);
  esp32c3_free_cpuint(priv->config->periph);
  priv->cpuint = -ENOMEM;
#endif

  esp32c3_i2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_ESP32C3_I2C */
