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

#include "esp32_i2c.h"
#include "esp32_gpio.h"
#include "esp32_irq.h"

#include "xtensa.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_i2c.h"
#include "hardware/esp32_soc.h"

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

/* Helper */

#ifdef CONFIG_I2C_POLLED
#define TIMESPEC_TO_US(sec, nano)  ((sec * USEC_PER_SEC) + (nano / NSEC_PER_USEC))
#endif

#define ESP32_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_ESP32_I2CTIMEOSEC) + MSEC2TICK(CONFIG_ESP32_I2CTIMEOMS))

/* Default option */

#define I2C_FIFO_SIZE (32)

#define I2C_FILTER_CYC_NUM_DEF (7)

#define I2C_CLK_FREQ_DEF (100 * 1000)

#define I2C_INT_ERR_EN_BITS (I2C_ACK_ERR_INT_ENA | \
                             I2C_TIME_OUT_INT_ENA | \
                             I2C_ARBITRATION_LOST_INT_ENA | \
                             I2C_RXFIFO_OVF_INT_ENA)

#define I2C_SCL_CYC_NUM_DEF 9

/* I2C event trace logic.
 * NOTE: trace uses the internal, non-standard, low-level debug interface
 * syslog() but does not require that any other debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define esp32_i2c_tracereset(p)
#  define esp32_i2c_tracenew(p,s)
#  define esp32_i2c_traceevent(p,e,a,s)
#  define esp32_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/* I2C state */

enum esp32_i2cstate_e
{
  I2CSTATE_IDLE = 0,
  I2CSTATE_PROC,
  I2CSTATE_STOP,
#ifndef CONFIG_I2C_POLLED
  I2CSTATE_FINISH
#endif
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

#ifdef CONFIG_I2C_TRACE

/* Trace events */

enum esp32_trace_e
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

struct esp32_trace_s
{
  uint32_t status;               /* I2C 32-bit SR status */
  uint32_t count;                /* Interrupt count when status change */
  enum esp32_i2cstate_e event;   /* Last event that occurred with this status */
  uint32_t parm;                 /* Parameter associated with the event */
  clock_t time;                  /* First of event or first status */
};

#endif /* CONFIG_I2C_TRACE */

/* I2C Device hardware configuration */

struct esp32_i2c_config_s
{
  uint32_t reg_base;          /* I2C register base address */

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

struct esp32_i2c_priv_s
{
  const struct i2c_ops_s *ops; /* Standard I2C operations */

  /* Port configuration */

  const struct esp32_i2c_config_s *config;
  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion mutex */

#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
  int     cpuint;              /* CPU interrupt assigned to this I2C */
  uint8_t cpu;                 /* CPU ID */
#endif

  /* I2C work state (see enum esp32_i2cstate_e) */

  volatile enum esp32_i2cstate_e i2cstate;

  struct i2c_msg_s *msgv;      /* Message list */

  uint8_t msgid;               /* Current message ID */
  ssize_t bytes;               /* Processed data bytes */

  uint32_t error;              /* I2C transform error */

  bool ready_read;             /* If I2C has read data */

  uint32_t clk_freq;           /* Current I2C Clock frequency */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct esp32_trace_s trace[CONFIG_I2C_NTRACE];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp32_i2c_init_clock(struct esp32_i2c_priv_s *priv,
                                 uint32_t clock);
static void esp32_i2c_init(struct esp32_i2c_priv_s *priv);
static void esp32_i2c_deinit(struct esp32_i2c_priv_s *priv);
static int esp32_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs,
                              int count);
static inline void esp32_i2c_process(struct esp32_i2c_priv_s *priv,
                                     uint32_t status);
#ifdef CONFIG_I2C_POLLED
static int esp32_i2c_polling_waitdone(struct esp32_i2c_priv_s *priv);
#endif
#ifdef CONFIG_I2C_RESET
static int esp32_i2c_reset(struct i2c_master_s *dev);
#endif

#ifdef CONFIG_I2C_TRACE
static void esp32_i2c_tracereset(struct esp32_i2c_priv_s *priv);
static void esp32_i2c_tracenew(struct esp32_i2c_priv_s *priv,
                               uint32_t status);
static void esp32_i2c_traceevent(struct esp32_i2c_priv_s *priv,
                                 enum esp32_trace_e event,
                                 uint32_t parm,
                                 uint32_t status);
static void esp32_i2c_tracedump(struct esp32_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

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
#ifndef CONFIG_I2C_POLLED
  .periph     = ESP32_PERIPH_I2C_EXT0,
  .irq        = ESP32_IRQ_I2C_EXT0,
#endif
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
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
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
#ifndef CONFIG_I2C_POLLED
  .periph     = ESP32_PERIPH_I2C_EXT1,
  .irq        = ESP32_IRQ_I2C_EXT1,
#endif
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
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .i2cstate   = I2CSTATE_IDLE,
  .msgv       = NULL,
  .msgid      = 0,
  .bytes      = 0,
  .ready_read = 0
};
#endif /* CONFIG_ESP32_I2C1 */

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

static void esp32_i2c_reset_fifo(struct esp32_i2c_priv_s *priv)
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

static void esp32_i2c_sendstart(struct esp32_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  /* Load a start cmd, write and end in controller. */

  esp32_i2c_set_reg(priv, I2C_COMD0_OFFSET,
                    I2C_BASE_CMD(I2C_CMD_RESTART, 0));
  esp32_i2c_set_reg(priv, I2C_COMD1_OFFSET,
                    I2C_SEND_CMD(I2C_CMD_WRITE, 1, 1));
  esp32_i2c_set_reg(priv, I2C_COMD2_OFFSET,
                    I2C_BASE_CMD(I2C_CMD_END, 0));

  /* Load the slave address and the flags to the FIFO. */

  esp32_i2c_set_reg(priv, I2C_DATA_OFFSET, (msg->addr << 1) |
                                           (msg->flags & I2C_M_READ));

  /* Enable interrupts to detect the execution of an end cmd and
   * multiple errors.
   */

  esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, I2C_END_DETECT_INT_ENA |
                                              I2C_INT_ERR_EN_BITS);

  /* Start sending the slave address and hold line. */

  esp32_i2c_set_reg_bits(priv, I2C_CTR_OFFSET, I2C_TRANS_START_M);
}

/****************************************************************************
 * Name: esp32_i2c_senddata
 *
 * Description:
 *   Send I2C data
 *
 ****************************************************************************/

static void esp32_i2c_senddata(struct esp32_i2c_priv_s *priv)
{
  int i;
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  int n = msg->length - priv->bytes;

  n = n < I2C_FIFO_SIZE ? n : I2C_FIFO_SIZE;

  /* Load a write operation */

  esp32_i2c_set_reg(priv, I2C_COMD0_OFFSET,
                    I2C_SEND_CMD(I2C_CMD_WRITE, 1, n));
  esp32_i2c_set_reg(priv, I2C_COMD1_OFFSET,
                    I2C_BASE_CMD(I2C_CMD_END, 0));

  /* Transfer the data from the msg buffer to the TX FIFO */

  for (i = 0; i < n; i++)
    {
      esp32_i2c_set_reg(priv, I2C_DATA_OFFSET,
                        msg->buffer[priv->bytes + i]);
    }

  priv->bytes += n;

  /* Enable interrupts to detect the execution of an end cmd and
   * multiple errors.
   */

  esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, I2C_END_DETECT_INT_ENA |
                                              I2C_INT_ERR_EN_BITS);
  esp32_i2c_set_reg_bits(priv, I2C_CTR_OFFSET, I2C_TRANS_START_M);
}

/****************************************************************************
 * Name: esp32_i2c_recvdata
 *
 * Description:
 *   Transfer the received data from the RX FIFO to the message buffer.
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
 *   Configure I2C to prepare receiving data and it will enable an interrupt
 *   to detect the end of the receiving operation and to load the content
 *   into the RX buffer.
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

  /* Load read and end cmds */

  esp32_i2c_set_reg(priv, I2C_COMD0_OFFSET,
                    I2C_RECV_CMD(I2C_CMD_READ, ack_value, n));
  esp32_i2c_set_reg(priv, I2C_COMD1_OFFSET,
                    I2C_BASE_CMD(I2C_CMD_END, 0));

  /* Enable interrupts to detect the execution of an end cmd and
   * multiple errors.
   */

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
  /* Load a stop cmd and enable the interrupts that indicate end of a stop
   * cmd or error.
   */

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

static void esp32_i2c_init_clock(struct esp32_i2c_priv_s *priv,
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

static void esp32_i2c_init(struct esp32_i2c_priv_s *priv)
{
  const struct esp32_i2c_config_s *config = priv->config;

  esp32_gpiowrite(config->scl_pin, 1);
  esp32_gpiowrite(config->sda_pin, 1);

  esp32_configgpio(config->scl_pin, INPUT |
                                    OUTPUT |
                                    OPEN_DRAIN |
                                    FUNCTION_3);
  esp32_gpio_matrix_out(config->scl_pin, config->scl_outsig, 0, 0);
  esp32_gpio_matrix_in(config->scl_pin, config->scl_insig, 0);

  esp32_configgpio(config->sda_pin, INPUT |
                                    OUTPUT |
                                    OPEN_DRAIN |
                                    FUNCTION_3);
  esp32_gpio_matrix_out(config->sda_pin, config->sda_outsig, 0, 0);
  esp32_gpio_matrix_in(config->sda_pin, config->sda_insig, 0);

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

static void esp32_i2c_deinit(struct esp32_i2c_priv_s *priv)
{
  const struct esp32_i2c_config_s *config = priv->config;

  priv->clk_freq = 0;

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

static void esp32_i2c_reset_fsmc(struct esp32_i2c_priv_s *priv)
{
  esp32_i2c_deinit(priv);
  esp32_i2c_init(priv);
}

/****************************************************************************
 * Name: esp32_i2c_sem_waitdone
 *
 * Description:
 *    Wait for a transfer to complete using an interruptible semaphore which
 *    in unlocked from within the ISR context. This function is only used in
 *    interrupt driven mode.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 ****************************************************************************/
#ifndef CONFIG_I2C_POLLED
static int esp32_i2c_sem_waitdone(struct esp32_i2c_priv_s *priv)
{
  /* Wait on ISR semaphore */

  return nxsem_tickwait_uninterruptible(&priv->sem_isr, ESP32_I2CTIMEOTICKS);
}
#endif

/****************************************************************************
 * Name: esp32_i2c_polling_waitdone
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
static int esp32_i2c_polling_waitdone(struct esp32_i2c_priv_s *priv)
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

      status = esp32_i2c_get_reg(priv, I2C_INT_STATUS_OFFSET);
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
              esp32_i2c_set_reg(priv, I2C_INT_CLR_OFFSET, status);
              break;
            }

          esp32_i2c_set_reg(priv, I2C_INT_CLR_OFFSET, status);
          esp32_i2c_process(priv, status);
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

  esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, 0);

  return ret;
}
#endif

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

static int esp32_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs,
                              int count)
{
  int i;
  int ret = OK;
  struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;

  DEBUGASSERT(count > 0);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* If previous state is different than idle,
   * reset the fsmc to the idle state.
   */

  if (priv->i2cstate != I2CSTATE_IDLE)
    {
      esp32_i2c_reset_fsmc(priv);
      priv->i2cstate = I2CSTATE_IDLE;
    }

  /* Transfer the messages to the internal struct
   * and loop count times to make all transfers.
   */

  priv->msgv = msgs;

  for (i = 0; i < count; i++)
    {
      /* Clear TX and RX FIFOs. */

      esp32_i2c_reset_fifo(priv);

      priv->bytes = 0;
      priv->msgid = i;
      priv->ready_read = 0;
      priv->error = 0;
      priv->i2cstate = I2CSTATE_PROC;

      /* Set the SCLK frequency for the current msg. */

      esp32_i2c_init_clock(priv, msgs[i].frequency);

      if ((msgs[i].flags & I2C_M_NOSTART) != 0)
        {
          esp32_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->bytes,
                               esp32_i2c_get_reg(priv, I2C_SR_OFFSET));
          esp32_i2c_senddata(priv);

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

          esp32_i2c_tracereset(priv);

          esp32_i2c_traceevent(priv, I2CEVENT_SENDADDR, msgs[i].addr,
                               esp32_i2c_get_reg(priv, I2C_SR_OFFSET));

          /* Send the start cmd */

          esp32_i2c_sendstart(priv);
        }

#ifndef CONFIG_I2C_POLLED
      if (esp32_i2c_sem_waitdone(priv) < 0)
        {
          /* Timed out - transfer was not completed within the timeout */

          ret = -ETIMEDOUT;
          break;
        }
      else
        {
          if (priv->error)
            {
              /* An error occurred */

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
      ret = esp32_i2c_polling_waitdone(priv);
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
    }

  /* Dump the trace result */

  esp32_i2c_tracedump(priv);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: esp32_i2c_clear_bus
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
static int esp32_i2c_clear_bus(struct esp32_i2c_priv_s *priv)
{
  uint32_t clock_count;
  uint32_t stretch_count;
  int ret;
  const struct esp32_i2c_config_s *config = priv->config;

  /* Use GPIO configuration to un-wedge the bus */

  esp32_configgpio(config->scl_pin, INPUT_PULLUP | OUTPUT_OPEN_DRAIN);
  esp32_gpio_matrix_out(config->scl_pin, SIG_GPIO_OUT_IDX, 0, 0);
  esp32_configgpio(config->sda_pin, INPUT_PULLUP | OUTPUT_OPEN_DRAIN);
  esp32_gpio_matrix_out(config->sda_pin, SIG_GPIO_OUT_IDX, 0, 0);

  /* Set SDA to high */

  esp32_gpiowrite(config->sda_pin, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!esp32_gpioread(config->sda_pin))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ >= I2C_SCL_CYC_NUM_DEF)
        {
          ret = -EIO;
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!esp32_gpioread(config->scl_pin))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              ret = -EIO;
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      esp32_gpiowrite(config->scl_pin, 0);
      up_udelay(10);

      /* Drive SCL high again */

      esp32_gpiowrite(config->scl_pin, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  esp32_gpiowrite(config->sda_pin, 0);
  up_udelay(10);
  esp32_gpiowrite(config->scl_pin, 0);
  up_udelay(10);
  esp32_gpiowrite(config->scl_pin, 1);
  up_udelay(10);
  esp32_gpiowrite(config->sda_pin, 1);
  up_udelay(10);

  ret = OK;

out:
  return ret;
}
#endif

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
static int esp32_i2c_reset(struct i2c_master_s *dev)
{
  struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv->refs > 0);

  nxmutex_lock(&priv->lock);

  esp32_i2c_deinit(priv);
  esp32_i2c_clear_bus(priv);
  esp32_i2c_init(priv);

  priv->i2cstate = I2CSTATE_IDLE;
  priv->msgid = 0;
  priv->bytes = 0;
  priv->ready_read = 0;

  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp32_i2c_traceclear
 *
 * Description:
 *   Set I2C trace fields to default value.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void esp32_i2c_traceclear(struct esp32_i2c_priv_s *priv)
{
  struct esp32_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;
  trace->count  = 0;
  trace->event  = I2CEVENT_NONE;
  trace->parm   = 0;
  trace->time   = 0;
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: esp32_i2c_tracereset
 *
 * Description:
 *   Reset the trace info for a new data collection.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void esp32_i2c_tracereset(struct esp32_i2c_priv_s *priv)
{
  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  esp32_i2c_traceclear(priv);
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: esp32_i2c_tracenew
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
static void esp32_i2c_tracenew(struct esp32_i2c_priv_s *priv,
                               uint32_t status)
{
  struct esp32_trace_s *trace = &priv->trace[priv->tndx];

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

      esp32_i2c_traceclear(priv);
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
 * Name: esp32_i2c_traceevent
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
static void esp32_i2c_traceevent(struct esp32_i2c_priv_s *priv,
                                 enum esp32_trace_e event,
                                 uint32_t parm,
                                 uint32_t status)
{
  /* Check for new trace setup */

  esp32_i2c_tracenew(priv, status);

  if (event != I2CEVENT_NONE)
    {
      struct esp32_trace_s *trace = &priv->trace[priv->tndx];

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
      esp32_i2c_traceclear(priv);
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: esp32_i2c_tracedump
 *
 * Description:
 *   Dump the trace results.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void esp32_i2c_tracedump(struct esp32_i2c_priv_s *priv)
{
  struct esp32_trace_s *trace;
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
 * Name: esp32_i2c_irq
 *
 * Description:
 *   This is the common I2C interrupt handler.  It will be invoked
 *   when an interrupt received on the device.
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int esp32_i2c_irq(int cpuint, void *context, void *arg)
{
  struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)arg;

  /* Get the interrupt status and clear the interrupts that
   * triggered.
   */

  uint32_t status = esp32_i2c_get_reg(priv, I2C_INT_STATUS_OFFSET);
  esp32_i2c_set_reg(priv, I2C_INT_CLR_OFFSET, status);
  esp32_i2c_process(priv, status);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp32_i2c_process
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

static inline void esp32_i2c_process(struct esp32_i2c_priv_s *priv,
                                     uint32_t status)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];

  /* Check for any errors */

  if (I2C_INT_ERR_EN_BITS & status)
    {
      /* Save the errors, register the error event, disable interrupts
       * and release the semaphore to conclude the transfer.
       */

      priv->error = status & I2C_INT_ERR_EN_BITS;
      esp32_i2c_traceevent(priv, I2CEVENT_ERROR, priv->error,
                           esp32_i2c_get_reg(priv, I2C_SR_OFFSET));
      esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, 0);
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

          if (msg->flags & I2C_M_READ)
            {
              /* RX FIFO has available data */

              if (priv->ready_read)
                {
                  esp32_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->bytes,
                                     esp32_i2c_get_reg(priv, I2C_SR_OFFSET));
                  esp32_i2c_recvdata(priv);

                  priv->ready_read = 0;
                }

              /* Received all data. Finish the transaction
               * and update the state I2C state.
               */

              if (priv->bytes == msg->length)
                {
                  esp32_i2c_traceevent(priv, I2CEVENT_STOP, msg->length,
                                     esp32_i2c_get_reg(priv, I2C_SR_OFFSET));
                  esp32_i2c_sendstop(priv);
#ifndef CONFIG_I2C_POLLED
                  priv->i2cstate = I2CSTATE_FINISH;
#endif
                }
              else /* Start a receive operation */
                {
                  esp32_i2c_traceevent(priv, I2CEVENT_RCVMODEEN, 0,
                                     esp32_i2c_get_reg(priv, I2C_SR_OFFSET));
                  esp32_i2c_startrecv(priv);

                  priv->ready_read = 1;
                }
            }
          else /* Write operation */
            {
              esp32_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->bytes,
                                   esp32_i2c_get_reg(priv, I2C_SR_OFFSET));
              esp32_i2c_senddata(priv);

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
           * and update the state I2C state.
           */

          esp32_i2c_traceevent(priv, I2CEVENT_STOP, msg->length,
                               esp32_i2c_get_reg(priv, I2C_SR_OFFSET));
          esp32_i2c_sendstop(priv);
#ifndef CONFIG_I2C_POLLED
          priv->i2cstate = I2CSTATE_FINISH;
#endif
        }
#ifndef CONFIG_I2C_POLLED
      else if (priv->i2cstate == I2CSTATE_FINISH)
        {
          /* Disable all interrupts and release the semaphore */

          esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, 0);
          nxsem_post(&priv->sem_isr);
        }
#endif
    }
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

struct i2c_master_s *esp32_i2cbus_initialize(int port)
{
  struct esp32_i2c_priv_s *priv;
#ifndef CONFIG_I2C_POLLED
  const struct esp32_i2c_config_s *config;
  int ret;
#endif

  switch (port)
    {
#ifdef CONFIG_ESP32_I2C0
    case 0:
      priv = &esp32_i2c0_priv;
      break;
#endif
#ifdef CONFIG_ESP32_I2C1
    case 1:
      priv = &esp32_i2c1_priv;
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
  priv->cpuint = esp32_setup_irq(priv->cpu, config->periph,
                                 1, ESP32_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type */

      priv->refs--;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  ret = irq_attach(config->irq, esp32_i2c_irq, priv);
  if (ret != OK)
    {
      esp32_teardown_irq(priv->cpu, config->periph, priv->cpuint);
      priv->refs--;

      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  up_enable_irq(config->irq);
#endif

  esp32_i2c_init(priv);
  nxmutex_unlock(&priv->lock);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: esp32_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int esp32_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct esp32_i2c_priv_s *priv = (struct esp32_i2c_priv_s *)dev;

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
  esp32_teardown_irq(priv->cpu, priv->config->periph, priv->cpuint);
#endif

  esp32_i2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_ESP32_I2C */
