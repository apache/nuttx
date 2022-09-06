/****************************************************************************
 * arch/arm/src/kinetis/kinetis_i2c.c
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
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "kinetis_config.h"
#include "chip.h"
#include "hardware/kinetis_i2c.h"
#include "hardware/kinetis_sim.h"
#include "hardware/kinetis_pinmux.h"
#include "kinetis.h"
#include "kinetis_i2c.h"

#if defined(CONFIG_KINETIS_I2C0) || defined(CONFIG_KINETIS_I2C1) || \
    defined(CONFIG_KINETIS_I2C2) || defined(CONFIG_KINETIS_I2C3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_TIMEOUT  USEC2TICK(20*1000)   /* 20 mS */

#define I2C_DEFAULT_FREQUENCY 400000

#define STATE_OK                0
#define STATE_ARBITRATION_ERROR 1
#define STATE_TIMEOUT           2
#define STATE_NAK               3

#define MKI2C_OUTPUT(p) (((p) & (_PIN_PORT_MASK | _PIN_MASK)) | \
        GPIO_OPENDRAIN | GPIO_OUTPUT_ONE)

#define MKI2C_INPUT(p) (((p) & (_PIN_PORT_MASK | _PIN_MASK)) | \
        PIN_ANALOG)

/* TODO:
 * - revisar tamanio de todos los registros (getreg/putreg)
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C Device hardware configuration */

struct kinetis_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  uint32_t clk_reg;           /* Clock Register */
  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
  uint16_t irqid;             /* IRQ for this device */
};

/* I2C device state structure */

struct kinetis_i2cdev_s
{
  /* Generic I2C device */

  struct i2c_master_s dev;

  /* Port configuration */

  const struct kinetis_i2c_config_s *config;

  uint32_t frequency;         /* Current I2C frequency */
  uint16_t nmsg;              /* Number of transfer remaining */
  uint16_t wrcnt;             /* number of bytes sent to tx fifo */
  uint16_t rdcnt;             /* number of bytes read from rx fifo */
  int      refs;              /* Reference count */
  volatile uint8_t state;     /* State of state machine */
  bool restart;               /* Should next transfer restart or not */
  mutex_t lock;               /* Only one thread can access at a time */
  sem_t wait;                 /* Place to wait for state machine completion */
  struct wdog_s timeout;      /* watchdog to timeout when bus hung */
  struct i2c_msg_s *msgs;     /* Remaining transfers - first one is in
                               * progress */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register access */

static uint8_t kinetis_i2c_getreg(struct kinetis_i2cdev_s *priv,
                                  uint8_t offset);
static void kinetis_i2c_putreg(struct kinetis_i2cdev_s *priv,
                               uint8_t value, uint8_t offset);

/* Signal Helper */

static inline void kinetis_i2c_endwait(struct kinetis_i2cdev_s *priv);
static inline void kinetis_i2c_wait(struct kinetis_i2cdev_s *priv);

/* I2C helpers */

static int kinetis_i2c_init(struct kinetis_i2cdev_s *priv);
static int kinetis_i2c_deinit(struct kinetis_i2cdev_s *priv);

static void kinetis_i2c_setfrequency(struct kinetis_i2cdev_s *priv,
                                     uint32_t frequency);
static int  kinetis_i2c_start(struct kinetis_i2cdev_s *priv);
static void kinetis_i2c_stop(struct kinetis_i2cdev_s *priv);
static int kinetis_i2c_interrupt(int irq, void *context, void *arg);
static void kinetis_i2c_timeout(wdparm_t arg);
static void kinetis_i2c_setfrequency(struct kinetis_i2cdev_s *priv,
                                     uint32_t frequency);

/* I2C lower half driver methods */

static int  kinetis_i2c_transfer(struct i2c_master_s *dev,
                                 struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int  kinetis_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C lower half driver operations */

static const struct i2c_ops_s kinetis_i2c_ops =
{
  .transfer = kinetis_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = kinetis_i2c_reset
#endif
};

/* I2C device structures */

#ifdef CONFIG_KINETIS_I2C0
static const struct kinetis_i2c_config_s kinetis_i2c0_config =
{
  .base       = KINETIS_I2C0_BASE,
  .clk_reg    = KINETIS_SIM_SCGC4,
  .clk_bit    = SIM_SCGC4_I2C0,
  .scl_pin    = PIN_I2C0_SCL,
  .sda_pin    = PIN_I2C0_SDA,
  .irqid      = KINETIS_IRQ_I2C0,
};

static struct kinetis_i2cdev_s g_i2c0_dev =
{
  .dev.ops    = &kinetis_i2c_ops,
  .config     = &kinetis_i2c0_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
  .state      = STATE_OK,
  .msgs       = NULL,
};
#endif

#ifdef CONFIG_KINETIS_I2C1
static const struct kinetis_i2c_config_s kinetis_i2c1_config =
{
  .base       = KINETIS_I2C1_BASE,
  .clk_reg    = KINETIS_SIM_SCGC4,
  .clk_bit    = SIM_SCGC4_I2C1,
  .scl_pin    = PIN_I2C1_SCL,
  .sda_pin    = PIN_I2C1_SDA,
  .irqid      = KINETIS_IRQ_I2C1,
};

static struct kinetis_i2cdev_s g_i2c1_dev =
{
  .dev.ops    = &kinetis_i2c_ops,
  .config     = &kinetis_i2c1_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
  .state      = STATE_OK,
  .msgs       = NULL,
};
#endif

#ifdef CONFIG_KINETIS_I2C2
static const struct kinetis_i2c_config_s kinetis_i2c2_config =
{
  .base       = KINETIS_I2C2_BASE,
  .clk_reg    = KINETIS_SIM_SCGC1,
  .clk_bit    = SIM_SCGC1_I2C2,
  .scl_pin    = PIN_I2C2_SCL,
  .sda_pin    = PIN_I2C2_SDA,
  .irqid      = KINETIS_IRQ_I2C2,
};

static struct kinetis_i2cdev_s g_i2c2_dev =
{
  .dev.ops    = &kinetis_i2c_ops,
  .config     = &kinetis_i2c2_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
  .state      = STATE_OK,
  .msgs       = NULL,
};
#endif

#ifdef CONFIG_KINETIS_I2C3
static const struct kinetis_i2c_config_s kinetis_i2c3_config =
{
  .base       = KINETIS_I2C3_BASE,
  .clk_reg    = KINETIS_SIM_SCGC1,
  .clk_bit    = SIM_SCGC1_I2C3,
  .scl_pin    = PIN_I2C3_SCL,
  .sda_pin    = PIN_I2C3_SDA,
  .irqid      = KINETIS_IRQ_I2C3,
};

static struct kinetis_i2cdev_s g_i2c3_dev =
{
  .dev.ops    = &kinetis_i2c_ops,
  .config     = &kinetis_i2c3_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
  .state      = STATE_OK,
  .msgs       = NULL,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static uint8_t kinetis_i2c_getreg(struct kinetis_i2cdev_s *priv,
                                  uint8_t offset)
{
  return getreg8(priv->config->base + offset);
}

/****************************************************************************
 * Name: kinetis_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ****************************************************************************/

static void kinetis_i2c_putreg(struct kinetis_i2cdev_s *priv, uint8_t value,
                               uint8_t offset)
{
  putreg8(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: kinetis_i2c_wait
 *
 * Description:
 *   Wait on the signaling semaphore.  May be interrupted by a signal.
 *
 ****************************************************************************/

static inline void kinetis_i2c_wait(struct kinetis_i2cdev_s *priv)
{
  nxsem_wait(&priv->wait);
}

/****************************************************************************
 * Name: kinetis_i2c_endwait
 *
 * Description:
 *   Release the signaling semaphore
 *
 ****************************************************************************/

static inline void kinetis_i2c_endwait(struct kinetis_i2cdev_s *priv)
{
  nxsem_post(&priv->wait);
}

/****************************************************************************
 * Name: kinetis_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int kinetis_i2c_init(struct kinetis_i2cdev_s *priv)
{
  uint32_t regval;

  /* Enable the clock to peripheral */

  modifyreg32(priv->config->clk_reg, 0, priv->config->clk_bit);

  /* Disable while configuring */

  kinetis_i2c_putreg(priv, 0, KINETIS_I2C_C1_OFFSET);

  /* Configure pins */

  if (kinetis_pinconfig(priv->config->scl_pin) < 0)
    {
      return ERROR;
    }

  if (kinetis_pinconfig(priv->config->sda_pin) < 0)
    {
      kinetis_pinconfig(MKI2C_INPUT(priv->config->scl_pin));
      return ERROR;
    }

  /* High-drive select */

  regval = kinetis_i2c_getreg(priv, KINETIS_I2C_C2_OFFSET);
  regval |= I2C_C2_HDRS;
  kinetis_i2c_putreg(priv, regval, KINETIS_I2C_C2_OFFSET);

  /* Attach Interrupt Handler */

  irq_attach(priv->config->irqid, kinetis_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->config->irqid);

  /* Force a frequency update */

  priv->frequency = 0;

  /* Set the default I2C frequency */

  kinetis_i2c_setfrequency(priv, I2C_DEFAULT_FREQUENCY);

  /* Enable I2C */

  kinetis_i2c_putreg(priv, I2C_C1_IICEN, KINETIS_I2C_C1_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: kinetis_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int kinetis_i2c_deinit(struct kinetis_i2cdev_s *priv)
{
  /* Disable I2C */

  kinetis_i2c_putreg(priv, 0, KINETIS_I2C_C1_OFFSET);

  /* Unconfigure GPIO pins */

  kinetis_pinconfig(MKI2C_INPUT(priv->config->scl_pin));
  kinetis_pinconfig(MKI2C_INPUT(priv->config->sda_pin));

  /* Disable and detach interrupts */

  up_disable_irq(priv->config->irqid);
  irq_detach(priv->config->irqid);

  /* Disable clocking */

  modifyreg32(priv->config->clk_reg, priv->config->clk_bit, 0);
  return OK;
}

/****************************************************************************
 * Name: kinetis_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void kinetis_i2c_setfrequency(struct kinetis_i2cdev_s *priv,
                                     uint32_t frequency)
{
  i2cinfo("frequency=%lu\n", (unsigned long)frequency);

  if (frequency == priv->frequency)
    {
      return;
    }

#if BOARD_BUS_FREQ == 120000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV1152, KINETIS_I2C_F_OFFSET);   /* 104 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV288, KINETIS_I2C_F_OFFSET);    /* 416 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV128, KINETIS_I2C_F_OFFSET);    /* 0.94 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 108000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV1024, KINETIS_I2C_F_OFFSET);   /* 105 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV256, KINETIS_I2C_F_OFFSET);    /* 422 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV112, KINETIS_I2C_F_OFFSET);    /* 0.96 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 96000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV960, KINETIS_I2C_F_OFFSET);    /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV240, KINETIS_I2C_F_OFFSET);    /* 400 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV96, KINETIS_I2C_F_OFFSET);     /* 1.0 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 90000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV896, KINETIS_I2C_F_OFFSET);    /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV224, KINETIS_I2C_F_OFFSET);    /* 402 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV88, KINETIS_I2C_F_OFFSET);     /* 1.02 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 80000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV768, KINETIS_I2C_F_OFFSET);    /* 104 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV192, KINETIS_I2C_F_OFFSET);    /* 416 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV80, KINETIS_I2C_F_OFFSET);     /* 1.0 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 72000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV640, KINETIS_I2C_F_OFFSET);    /* 112 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV192, KINETIS_I2C_F_OFFSET);    /* 375 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV72, KINETIS_I2C_F_OFFSET);     /* 1.0 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 64000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV640, KINETIS_I2C_F_OFFSET);    /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV160, KINETIS_I2C_F_OFFSET);    /* 400 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV64, KINETIS_I2C_F_OFFSET);     /* 1.0 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 60000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV576, KINETIS_I2C_F_OFFSET);    /* 104 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV144, KINETIS_I2C_F_OFFSET);    /* 416 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV64, KINETIS_I2C_F_OFFSET);     /* 938 kHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 56000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV512, KINETIS_I2C_F_OFFSET);    /* 109 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV144, KINETIS_I2C_F_OFFSET);    /* 389 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV56_1, KINETIS_I2C_F_OFFSET);   /* 1 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 54000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV512, KINETIS_I2C_F_OFFSET);    /* 105 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV128, KINETIS_I2C_F_OFFSET);    /* 422 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV56, KINETIS_I2C_F_OFFSET);     /* 0.96 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 48000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV480, KINETIS_I2C_F_OFFSET);    /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV112, KINETIS_I2C_F_OFFSET);    /* 400 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV48_1, KINETIS_I2C_F_OFFSET);   /* 1 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(4), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 40000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV384_2, KINETIS_I2C_F_OFFSET);  /* 104 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV96, KINETIS_I2C_F_OFFSET);     /* 416 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV40_2, KINETIS_I2C_F_OFFSET);   /* 1 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(3), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 36000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV320_2, KINETIS_I2C_F_OFFSET);  /* 113 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV96, KINETIS_I2C_F_OFFSET);     /* 375 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV36, KINETIS_I2C_F_OFFSET);     /* 1 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(3), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 24000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV240, KINETIS_I2C_F_OFFSET);    /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV64, KINETIS_I2C_F_OFFSET);     /* 375 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV24, KINETIS_I2C_F_OFFSET);     /* 1 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(2), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 16000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV160_2, KINETIS_I2C_F_OFFSET);  /* 100 kHz */
    }
  else if (frequency < 1000000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV40_1, KINETIS_I2C_F_OFFSET);   /* 400 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV20, KINETIS_I2C_F_OFFSET);     /* 800 MHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(1), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 8000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV80_1, KINETIS_I2C_F_OFFSET);   /* 100 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV20, KINETIS_I2C_F_OFFSET);     /* 400 kHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(1), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 4000000
  if (frequency < 400000)
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV40_1, KINETIS_I2C_F_OFFSET);   /* 100 kHz */
    }
  else
    {
      kinetis_i2c_putreg(priv, I2C_F_DIV20, KINETIS_I2C_F_OFFSET);     /* 200 kHz */
    }

  kinetis_i2c_putreg(priv, I2C_FLT(1), KINETIS_I2C_FLT_OFFSET);

#elif BOARD_BUS_FREQ == 2000000
  kinetis_i2c_putreg(priv, I2C_F_DIV20, KINETIS_I2C_F_OFFSET);         /* 100 kHz */
  kinetis_i2c_putreg(priv, I2C_FLT(1), KINETIS_I2C_FLT_OFFSET);

#else
#  error "F_BUS must be 120, 108, 96, 9, 80, 72, 64, 60, 56, 54, 48, 40, 36, 24, 16, 8, 4 or 2 MHz"
#endif

  priv->frequency = frequency;
}

/****************************************************************************
 * Name: kinetis_i2c_start
 *
 * Description:
 *   Initiate I2C transfer (START/RSTART + address)
 *
 ****************************************************************************/

static int kinetis_i2c_start(struct kinetis_i2cdev_s *priv)
{
  struct i2c_msg_s *msg;
  clock_t start;

  i2cinfo("START msg=%p\n", priv->msgs);
  msg = priv->msgs;

  /* Now take control of the bus */

  if (kinetis_i2c_getreg(priv, KINETIS_I2C_C1_OFFSET) & I2C_C1_MST)
    {
      /* We are already the bus master, so send a repeated start */

      kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST |
                               I2C_C1_RSTA | I2C_C1_TX,
                         KINETIS_I2C_C1_OFFSET);
    }
  else
    {
      /* We are not currently the bus master, wait for bus ready or timeout */

      start = clock_systime_ticks();

      while (kinetis_i2c_getreg(priv, KINETIS_I2C_S_OFFSET) & I2C_S_BUSY)
        {
          if (clock_systime_ticks() - start > I2C_TIMEOUT)
            {
              priv->state = STATE_TIMEOUT;
              return -EIO;
            }
        }

      /* Become the bus master in transmit mode (send start) */

      kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST |
                               I2C_C1_TX,
                         KINETIS_I2C_C1_OFFSET);
    }

  if (I2C_M_READ & msg->flags)  /* DEBUG: should happen always */
    {
      /* Wait until start condition establishes control of the bus or
       * a timeout occurs
       */

      start = clock_systime_ticks();

      while ((kinetis_i2c_getreg(priv, KINETIS_I2C_S_OFFSET) & I2C_S_BUSY)
             == 0)
        {
          if (clock_systime_ticks() - start > I2C_TIMEOUT)
            {
              priv->state = STATE_TIMEOUT;
              return -EIO;
            }
        }
    }

  /* Initiate actual transfer (send address) */

  kinetis_i2c_putreg(priv, (I2C_M_READ & msg->flags) == I2C_M_READ ?
                     I2C_READADDR8(msg->addr) : I2C_WRITEADDR8(msg->addr),
                     KINETIS_I2C_D_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: kinetis_i2c_stop
 *
 * Description:
 *   Perform a I2C transfer stop
 *
 ****************************************************************************/

static void kinetis_i2c_stop(struct kinetis_i2cdev_s *priv)
{
  i2cinfo("STOP msg=%p\n", priv->msgs);

  kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE,
                     KINETIS_I2C_C1_OFFSET);
  kinetis_i2c_endwait(priv);
}

/****************************************************************************
 * Name: kinetis_i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 ****************************************************************************/

static void kinetis_i2c_timeout(wdparm_t arg)
{
  struct kinetis_i2cdev_s *priv = (struct kinetis_i2cdev_s *)arg;

  DEBUGASSERT(priv != NULL);
  i2cinfo("Timeout msg=%p\n", priv->msgs);

  irqstate_t flags = enter_critical_section();
  priv->state = STATE_TIMEOUT;
  kinetis_i2c_endwait(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: kinetis_i2c_nextmsg
 *
 * Description:
 *   Setup for the next message.
 *
 ****************************************************************************/

void kinetis_i2c_nextmsg(struct kinetis_i2cdev_s *priv)
{
  priv->nmsg--;
  i2cinfo("nmsg=%u\n", priv->nmsg);

  if (priv->nmsg > 0)
    {
      priv->msgs++;
      i2cinfo("msg=%p\n", priv->msgs);

      priv->wrcnt = 0;
      priv->rdcnt = 0;

      if (priv->restart)
        {
          kinetis_i2c_endwait(priv);
        }
    }
  else
    {
      kinetis_i2c_stop(priv);
    }
}

/****************************************************************************
 * Name: kinetis_i2c_interrupt
 *
 * Description:
 *   The I2C common interrupt handler
 *
 ****************************************************************************/

static int kinetis_i2c_interrupt(int irq, void *context, void *arg)
{
  struct kinetis_i2cdev_s *priv = (struct kinetis_i2cdev_s *)arg;
  struct i2c_msg_s *msg;
  uint32_t state;
  int regval;
  int dummy;
  UNUSED(dummy);

  DEBUGASSERT(priv != NULL);

  /* Get current state */

  state = kinetis_i2c_getreg(priv, KINETIS_I2C_S_OFFSET);
  msg = priv->msgs;

  /* Arbitration lost */

  if (state & I2C_S_ARBL)
    {
      kinetis_i2c_putreg(priv, I2C_S_IICIF | I2C_S_ARBL,
                         KINETIS_I2C_S_OFFSET);
      priv->state = STATE_ARBITRATION_ERROR;
      kinetis_i2c_stop(priv);
    }
  else
    {
      /* Clear interrupt */

      kinetis_i2c_putreg(priv, I2C_S_IICIF, KINETIS_I2C_S_OFFSET);
      regval = kinetis_i2c_getreg(priv, KINETIS_I2C_C1_OFFSET);

      /* TX mode */

      if (regval & I2C_C1_TX)
        {
          /* Last write was not acknowledged */

          if (state & I2C_S_RXAK)
            {
              priv->state = STATE_NAK;  /* Set error flag */
              kinetis_i2c_stop(priv);   /* Send STOP */
            }
          else
            {
              /* Actually intending to write */

              if ((I2C_M_READ & msg->flags) == 0)
                {
                  /* Wrote everything */

                  if (priv->wrcnt == msg->length)
                    {
                      /* Continue with next message */

                      kinetis_i2c_nextmsg(priv);

                      if (!priv->restart)
                        {
                          /* Initiate transfer of following message */

                          kinetis_i2c_putreg(priv,
                                             priv->msgs->buffer[priv->wrcnt],
                                             KINETIS_I2C_D_OFFSET);
                          priv->wrcnt++;

                          kinetis_i2c_endwait(priv);
                        }
                    }
                  else
                    {
                      /* Put next byte */

                      kinetis_i2c_putreg(priv, msg->buffer[priv->wrcnt],
                                         KINETIS_I2C_D_OFFSET);
                      priv->wrcnt++;
                    }
                }

              /* Actually intending to read (address was just sent) */

              else
                {
                  if (msg->length == 1 && priv->restart)
                    {
                      /* Go to RX mode, do not send ACK */

                      kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE |
                                         I2C_C1_MST | I2C_C1_TXAK,
                                         KINETIS_I2C_C1_OFFSET);
                    }
                  else
                    {
                      /* Go to RX mode */

                      kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE |
                                         I2C_C1_MST, KINETIS_I2C_C1_OFFSET);
                    }

                  /* TODO: handle zero-length reads */

                  /* Dummy read to initiate reception */

                  dummy = kinetis_i2c_getreg(priv, KINETIS_I2C_D_OFFSET);
                }
            }
        }

      /* RX: mode */

      else
        {
          /* If last receiving byte */

          if (priv->rdcnt == (msg->length - 1))
            {
              if (priv->restart)
                {
                  /* Go to TX mode before last read, otherwise a new read is
                   * triggered.
                   */

                  /* Go to TX mode */

                  kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE |
                                     I2C_C1_MST | I2C_C1_TX,
                                     KINETIS_I2C_C1_OFFSET);
                }
              else if ((priv->msgs + 1)->length == 1)
                {
                  /* We will continue reception on next message.
                   * if next message is length == 1, this is actually the
                   * 2nd to last byte, so do not send ACK.
                   */

                  /* Do not ACK any more */

                  kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE |
                                     I2C_C1_MST | I2C_C1_TXAK,
                                     KINETIS_I2C_C1_OFFSET);
                }

              msg->buffer[priv->rdcnt] =
                kinetis_i2c_getreg(priv, KINETIS_I2C_D_OFFSET);
              priv->rdcnt++;

              kinetis_i2c_nextmsg(priv);
            }

          /* Second to last receiving byte */

          else if (priv->rdcnt == (msg->length - 2))
            {
              if (priv->restart)
                {
                  /* Do not ACK any more */

                  kinetis_i2c_putreg(priv, I2C_C1_IICEN | I2C_C1_IICIE |
                                           I2C_C1_MST | I2C_C1_TXAK,
                                     KINETIS_I2C_C1_OFFSET);
                }

              msg->buffer[priv->rdcnt] =
                kinetis_i2c_getreg(priv, KINETIS_I2C_D_OFFSET);
              priv->rdcnt++;
            }
          else
            {
              msg->buffer[priv->rdcnt] =
                kinetis_i2c_getreg(priv, KINETIS_I2C_D_OFFSET);
              priv->rdcnt++;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: kinetis_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int kinetis_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count)
{
  struct kinetis_i2cdev_s *priv = (struct kinetis_i2cdev_s *)dev;
  int msg_n;
  int ret;

  i2cinfo("msgs=%p count=%d\n", msgs, count);
  DEBUGASSERT(dev != NULL && msgs != NULL && (unsigned)count <= UINT16_MAX);

  /* Get exclusive access to the I2C bus */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set up for the transfer */

  msg_n       = 0;
  priv->msgs  = msgs;
  priv->nmsg  = count;
  priv->state = STATE_OK;
  priv->wrcnt = 0;
  priv->rdcnt = 0;

  /* Configure the I2C frequency. REVISIT: Note that the frequency is set
   * only on the first message. This could be extended to support
   * different transfer frequencies for each message segment.
   */

  kinetis_i2c_setfrequency(priv, msgs->frequency);

  /* Clear the status flags */

  kinetis_i2c_putreg(priv, I2C_S_IICIF | I2C_S_ARBL, KINETIS_I2C_S_OFFSET);

  /* Process every message */

  while (priv->nmsg > 0 && priv->state == STATE_OK)
    {
      priv->restart = true;

      /* Process NORESTART flag */

      if (priv->nmsg > 1)
        {
          struct i2c_msg_s *nextmsg = (priv->msgs + 1);

          /* If there is a following message with "norestart" flag of
           * the same type as the current one, we can avoid the restart
           */

          if ((nextmsg->flags & I2C_M_NOSTART) &&
              nextmsg->addr == priv->msgs->addr &&
              nextmsg->frequency == priv->msgs->frequency &&
              (nextmsg->flags & I2C_M_READ) ==
               (priv->msgs->flags & I2C_M_READ))
            {
              /* "no restart" can be performed */

              priv->restart = false;
            }
        }

      /* Only send start when required (we are trusting the flags setting to
       * be correctly used here).
       */

      if (!(priv->msgs->flags & I2C_M_NOSTART))
        {
          /* Initiate the transfer, in case restart is required */

          if (kinetis_i2c_start(priv) < 0)
            {
              goto timeout;
            }
        }

      /* Wait for transfer complete */

      wd_start(&priv->timeout, I2C_TIMEOUT,
               kinetis_i2c_timeout, (wdparm_t)priv);
      kinetis_i2c_wait(priv);

      wd_cancel(&priv->timeout);

      msg_n++;
    }

  /* Disable interrupts */

timeout:
  kinetis_i2c_putreg(priv, I2C_C1_IICEN, KINETIS_I2C_C1_OFFSET);

  /* Get the result before releasing the bus  */

  ret  = (priv->state != STATE_OK) ? -EIO : 0;

  /* Release access to I2C bus */

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: kinetis_i2c_reset
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
static int kinetis_i2c_reset(struct i2c_master_s *dev)
{
  struct kinetis_i2cdev_s *priv = (struct kinetis_i2cdev_s *)dev;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  uint32_t frequency;
  int ret;

  DEBUGASSERT(dev);

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EIO;

  /* Save the current frequency */

  frequency = priv->frequency;

  /* De-init the port */

  kinetis_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  kinetis_pinconfig(scl_gpio);
  kinetis_pinconfig(sda_gpio);

  /* Let SDA go high */

  kinetis_gpiowrite(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!kinetis_gpioread(sda_gpio))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ > 10)
        {
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!kinetis_gpioread(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      kinetis_gpiowrite(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      kinetis_gpiowrite(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  kinetis_gpiowrite(sda_gpio, 0);
  up_udelay(10);
  kinetis_gpiowrite(scl_gpio, 0);
  up_udelay(10);
  kinetis_gpiowrite(scl_gpio, 1);
  up_udelay(10);
  kinetis_gpiowrite(sda_gpio, 1);
  up_udelay(10);

  /* Revert the GPIO configuration. */

  kinetis_pinconfig(MKI2C_INPUT(sda_gpio));
  kinetis_pinconfig(MKI2C_INPUT(scl_gpio));
  ret = OK;

  /* Re-init the port (even on error to enable clock) */

out:
  kinetis_i2c_init(priv);

  /* Restore the frequency */

  kinetis_i2c_setfrequency(priv, frequency);

  /* Release the port for re-use by other clients */

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *kinetis_i2cbus_initialize(int port)
{
  struct kinetis_i2cdev_s *priv;

  i2cinfo("port=%d\n", port);

  switch (port)
    {
#ifdef CONFIG_KINETIS_I2C0
    case 0:
      priv = &g_i2c0_dev;
      break;
#endif

#ifdef CONFIG_KINETIS_I2C1
    case 1:
      priv = &g_i2c1_dev;
      break;
#endif

#ifdef CONFIG_KINETIS_I2C2
    case 2:
      priv = &g_i2c2_dev;
      break;
#endif

#ifdef CONFIG_KINETIS_I2C3
    case 3:
      priv = &g_i2c3_dev;
      break;
#endif

    default:
      i2cerr("ERROR: Unsupported I2C port %d\n", port);
      return NULL;
    }

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      kinetis_i2c_init(priv);
    }

  nxmutex_unlock(&priv->lock);
  return &priv->dev;
}

/****************************************************************************
 * Name: kinetis_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int kinetis_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct kinetis_i2cdev_s *priv = (struct kinetis_i2cdev_s *)dev;

  DEBUGASSERT(priv != NULL);

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

  /* Disable power and other HW resource (GPIO's) */

  kinetis_i2c_deinit(priv);
  wd_cancel(&priv->timeout);

  nxmutex_unlock(&priv->lock);
  return OK;
}

#endif /* CONFIG_KINETIS_I2C0 || CONFIG_KINETIS_I2C1 || CONFIG_KINETIS_I2C2 */
