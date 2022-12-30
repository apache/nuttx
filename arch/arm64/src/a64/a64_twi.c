/****************************************************************************
 * arch/arm64/src/a64/a64_twi.c
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

/* References: Allwinner_A64_User_Manual_V1.0. */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm64_internal.h"
#include "hardware/a64_pio.h"

#include "a64_pio.h"
#include "a64_pinmux.h"
#include "a64_twi.h"
#include "arm64_gic.h"

#if defined(CONFIG_A64_TWI0) || defined(CONFIG_A64_TWI1) || \
    defined(CONFIG_A64_TWI2) || defined(CONFIG_A64_RTWI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bus Clock Gating Register 3 (A64 Page 105) */

#define BUS_CLK_GATING_REG3 (A64_CCU_ADDR + 0x006c)

/* Bus Software Reset Register 4 (A64 Page 143) */

#define BUS_SOFT_RST_REG4 (A64_CCU_ADDR + 0x02d8)

/* Configuration ************************************************************/

#ifndef CONFIG_A64_TWI0_FREQUENCY
#  define CONFIG_A64_TWI0_FREQUENCY 100000
#endif

#ifndef CONFIG_A64_TWI1_FREQUENCY
#  define CONFIG_A64_TWI1_FREQUENCY 100000
#endif

#ifndef CONFIG_A64_TWI2_FREQUENCY
#  define CONFIG_A64_TWI2_FREQUENCY 100000
#endif

#ifndef CONFIG_A64_RTWI_FREQUENCY
#  define CONFIG_A64_RTWI_FREQUENCY 100000
#endif

#ifndef CONFIG_DEBUG_I2C_INFO
#  undef CONFIG_A64_TWI_REGDEBUG
#endif

/* Driver internal definitions **********************************************/

/* If verbose I2C debug output is enabled, then allow more time before we
 * declare a timeout.  The debug output from twi_interrupt will really slow
 * things down!
 *
 * With a very slow clock (say 100,000 Hz), less than 100 usec would be
 * required to transfer on byte.  So these define a "long" timeout.
 */

#ifdef CONFIG_DEBUG_I2C_INFO
#  define TWI_TIMEOUT_MSPB (50)  /* 50 msec/byte */
#else
#  define TWI_TIMEOUT_MSPB (5)   /* 5 msec/byte */
#endif

#ifdef CONFIG_I2C_SLAVE
#  error I2C slave logic is not supported yet for A64
#endif

#define I2C_MASTER      1
#define I2C_SLAVE       2

/* Ugly formula to convert m and n values to a frequency comes from
 * TWI specifications
 */

#ifndef CONFIG_APB2_CLK
#  define CONFIG_APB2_CLK 24000000
#endif

#define TWI_FREQUENCY(m, n) (CONFIG_APB2_CLK / ((10 * ((m) + 1)) << (n)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum a64_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* I2C Device hardware configuration */

struct a64_twi_config_s
{
  uint64_t base;        /* I2C base address */
  pio_pinset_t scl_pin; /* GPIO configuration for SCL as SCL */
  pio_pinset_t sda_pin; /* GPIO configuration for SDA as SDA */
  uint8_t mode;         /* Master or Slave mode */
#ifndef CONFIG_I2C_POLLED
  uint32_t irq;         /* Event IRQ */
#endif

  uint8_t twi;          /* TWI peripheral number (for debug output) */
};

/* I2C Device Private Data */

struct a64_twi_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  /* Port configuration */

  const struct a64_twi_config_s *config;

  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion mutex */
  sem_t            waitsem;    /* Wait for TWI transfer completion */
  struct wdog_s    timeout;    /* Watchdog to recover from bus hangs */
  volatile int     result;     /* The result of the transfer */
  volatile uint8_t intstate;   /* Interrupt handshake (see enum a64_intstate_e) */

  int msgc;                    /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  int dcnt;                    /* Current message length */
  uint16_t flags;              /* Current message flags */
  uint32_t frequency;          /* Current I2C frequency */

  uint32_t status;             /* End of transfer SR2|SR1 status */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t a64_twi_getreg(struct a64_twi_priv_s *priv,
                                      uint16_t offset);
static inline void a64_twi_putreg(struct a64_twi_priv_s *priv,
                                  uint16_t offset, uint32_t value);
static inline void a64_twi_modifyreg(struct a64_twi_priv_s *priv,
                                     uint16_t offset, uint32_t clearbits,
                                     uint32_t setbits);

/* Low-level helper functions */

static inline void twi_disable(struct a64_twi_priv_s *priv);
static inline void twi_enable(struct a64_twi_priv_s *priv);
static inline void twi_enable_irq(struct a64_twi_priv_s *priv);
static inline void twi_disable_irq(struct a64_twi_priv_s *priv);
static inline void twi_set_start(struct a64_twi_priv_s *priv);
static inline unsigned int twi_get_start(struct a64_twi_priv_s *priv);
static inline void twi_set_stop(struct a64_twi_priv_s *priv);
static inline unsigned int twi_get_stop(struct a64_twi_priv_s *priv);
static inline unsigned int twi_get_int_flag(struct a64_twi_priv_s *priv);
static inline void twi_clear_irq_flag(struct a64_twi_priv_s *priv);
static inline unsigned int twi_get_status(struct a64_twi_priv_s *priv);
static inline void twi_softreset(struct a64_twi_priv_s *priv);
static inline void twi_disable_ack(struct a64_twi_priv_s *priv);
static inline void twi_enable_ack(struct a64_twi_priv_s *priv);
static void twi_set_efr(struct a64_twi_priv_s *priv, unsigned int efr);

#ifdef CONFIG_I2C_RESET
static void twi_enable_scl(struct a64_twi_priv_s *priv);
static void twi_enable_sda(struct a64_twi_priv_s *priv);
static void twi_disable_scl(struct a64_twi_priv_s *priv);
static void twi_disable_sda(struct a64_twi_priv_s *priv);
static unsigned int twi_get_scl(struct a64_twi_priv_s *priv);
static unsigned int twi_get_sda(struct a64_twi_priv_s *priv);
static void twi_set_scl(struct a64_twi_priv_s *priv, unsigned int hi_lo);
static void twi_set_sda(struct a64_twi_priv_s *priv, unsigned int hi_lo);
#endif

static inline unsigned int twi_get_lcr(struct a64_twi_priv_s *priv);
static inline unsigned char twi_get_byte(struct a64_twi_priv_s *priv);
static inline void twi_put_byte(struct a64_twi_priv_s *priv, uint8_t ch);
static int twi_stop(struct a64_twi_priv_s *priv);

/* I2C transfer helper functions */

static int  twi_wait(struct a64_twi_priv_s *priv, unsigned int size);
static void twi_wakeup(struct a64_twi_priv_s *priv, int result);
static int  twi_interrupt(int irq, void *context, void *arg);
static void twi_timeout(wdparm_t arg);
static void twi_put_addr(struct a64_twi_priv_s *priv, uint16_t address);

/* I2C device operations */

static int twi_transfer(struct i2c_master_s *dev,
                        struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int twi_reset(struct i2c_master_s *dev);
static int twi_release(struct a64_twi_priv_s *priv);
#endif

/* Initialization */

static void twi_setclock(struct a64_twi_priv_s *priv, uint32_t freq);
static void twi_hw_initialize(struct a64_twi_priv_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_ops_s a64_twi_ops =
{
  .transfer = twi_transfer,
#ifdef CONFIG_I2C_RESET
  .reset  = twi_reset
#endif
};

#ifdef CONFIG_A64_TWI0
static const struct a64_twi_config_s a64_twi0_config =
{
  .base       = A64_TWI0_ADDR,
  .scl_pin    = PIO_TWI0_SCK,
  .sda_pin    = PIO_TWI0_SDA,
#ifndef CONFIG_I2C_SLAVE
  .mode       = I2C_MASTER,
#else
  .mode       = I2C_SLAVE,
#endif

#ifndef CONFIG_I2C_POLLED
  .irq        = A64_IRQ_TWI0,
#endif

  .twi        = 0
};

static struct a64_twi_priv_s a64_twi0_priv =
{
  .ops        = &a64_twi_ops,
  .config     = &a64_twi0_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .waitsem    = SEM_INITIALIZER(0),
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

#ifdef CONFIG_A64_TWI1
static const struct a64_twi_config_s a64_twi1_config =
{
  .base       = A64_TWI1_ADDR,
  .scl_pin    = PIO_TWI1_SCK,
  .sda_pin    = PIO_TWI1_SDA,
#ifndef CONFIG_I2C_SLAVE
  .mode       = I2C_MASTER,
#else
  .mode       = I2C_SLAVE,
#endif

#ifndef CONFIG_I2C_POLLED
  .irq        = A64_IRQ_TWI1,
#endif

  .twi        = 1
};

static struct a64_twi_priv_s a64_twi1_priv =
{
  .ops        = &a64_twi_ops,
  .config     = &a64_twi1_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .waitsem    = SEM_INITIALIZER(0),
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

#ifdef CONFIG_A64_TWI2
static const struct a64_twi_config_s a64_twi2_config =
{
  .base       = A64_TWI2_ADDR,
  .scl_pin    = PIO_TWI2_SCK,
  .sda_pin    = PIO_TWI2_SDA,
#ifndef CONFIG_I2C_SLAVE
  .mode       = I2C_MASTER,
#else
  .mode       = I2C_SLAVE,
#endif

#ifndef CONFIG_I2C_POLLED
  .irq        = A64_IRQ_TWI2,
#endif

  .twi        = 2
};

static struct a64_twi_priv_s a64_twi2_priv =
{
  .ops        = &a64_twi_ops,
  .config     = &a64_twi2_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .waitsem    = SEM_INITIALIZER(0),
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

#ifdef CONFIG_A64_RTWI
static const struct a64_twi_config_s a64_rtwi_config =
{
  .base       = A64_RTWI_ADDR,
  .scl_pin    = PIO_RTWI_SCK,
  .sda_pin    = PIO_RTWI_SDA,
#ifndef CONFIG_I2C_SLAVE
  .mode       = I2C_MASTER,
#else
  .mode       = I2C_SLAVE,
#endif

#ifndef CONFIG_I2C_POLLED
  .irq        = A64_IRQ_R_TWI,
#endif

  .twi        = 3
};

static struct a64_twi_priv_s a64_rtwi_priv =
{
  .ops        = &a64_twi_ops,
  .config     = &a64_rtwi_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .waitsem    = SEM_INITIALIZER(0),
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a64_twi_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t a64_twi_getreg(struct a64_twi_priv_s *priv,
                                      uint16_t offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: a64_twi_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void a64_twi_putreg(struct a64_twi_priv_s *priv,
                                  uint16_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: a64_twi_modifyreg
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *   modreg32(v,m,a) defined as:
 *   putreg32((getreg32(a) & ~(m)) | ((v) & (m)), (a))
 *
 ****************************************************************************/

static inline void a64_twi_modifyreg(struct a64_twi_priv_s *priv,
                                     uint16_t offset, uint32_t clearbits,
                                     uint32_t setbits)
{
  modreg32(setbits, clearbits, priv->config->base + offset);
}

/****************************************************************************
 * I2C transfer helper functions
 ****************************************************************************/

/****************************************************************************
 * Name: twi_disable
 *
 * Description:
 *   Disable i2c bus by A64_TWI_CNTR_REG
 *
 ****************************************************************************/

static inline void twi_disable(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);
  reg_val &= ~TWI_CNTR_BUSEN;
  a64_twi_putreg(priv, A64_TWI_CNTR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_enable
 *
 * Description:
 *   Enable i2c bus by A64_TWI_CNTR_REG
 *
 *   1’b0: The TWI bus inputs ISDA/ISCL are ignored and the TWI Controller
 *   will not respond to any address on the bus
 *   1’b1: The TWI will respond to calls to its slave address – and to
 *   the general call address if the GCE bit in the ADDR register is set.
 *   Notes: In master operation mode, this bit should be set to ‘1’
 *
 ****************************************************************************/

static inline void twi_enable(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);
  reg_val |= TWI_CNTR_BUSEN;
  a64_twi_putreg(priv, A64_TWI_CNTR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_enable_irq
 *
 * Description:
 *   Enable i2c bus interrupt
 *   Interrupt Enable
 *   bit7: 1’b0: The interrupt line always low
 *   bit7: 1’b1: The interrupt line will go high when INT_FLAG is set.
 *
 ****************************************************************************/

static inline void twi_enable_irq(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);

  /* 1 when enable irq for next operation, set intflag to 0 to prevent to
   * clear it by a mistake (intflag bit is write-1-to-clear bit)
   * 2 Similarly, mask START bit and STOP bit to prevent to set it twice by
   * a mistake (START bit and STOP bit are self-clear-to-0 bits)
   */

  reg_val |= TWI_CNTR_INTEN;
  reg_val &= ~(TWI_CNTR_STA | TWI_CNTR_STP | TWI_CNTR_INTFLG);
  a64_twi_putreg(priv, A64_TWI_CNTR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_disable_irq
 *
 * Description:
 *   Disable i2c bus interrupt
 *
 *   bit7: 1’b0: The interrupt line always low
 *   bit7: 1’b1: The interrupt line will go high when INT_FLAG is set.
 *
 ****************************************************************************/

static inline void twi_disable_irq(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);
  reg_val &= ~TWI_CNTR_INTEN;
  reg_val &= ~(TWI_CNTR_STA | TWI_CNTR_STP | TWI_CNTR_INTFLG);
  a64_twi_putreg(priv, A64_TWI_CNTR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_set_start
 *
 * Description:
 *   Trigger start signal, the start bit will be cleared automatically
 *   Master Mode Start
 *   When M_STA is set to ‘1’, TWI Controller enters master mode and will
 *   transmit a START condition on the bus when the bus is free.
 *   If the M_STA bit is set to ‘1’ when the TWI Controller is already
 *   in master mode and one or more bytes have been transmitted, then a
 *   repeated START condition will be sent. If the M_STA bit is set to '1'
 *   when the TWI is being accessed in slave mode, the TWI will complete the
 *   data transfer in slave mode then enter master mode when the bus has been
 *   released.The M_STA bit is cleared automatically after a START condition
 *   has been sent,writing a ‘0’ to this bit has no effect.
 *
 ****************************************************************************/

static inline void twi_set_start(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);
  reg_val |= TWI_CNTR_STA;
  reg_val &= ~TWI_CNTR_INTFLG;
  a64_twi_putreg(priv, A64_TWI_CNTR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_get_start
 *
 * Description:
 *   Get start bit status, poll if start signal is sent
 *   The M_STA bit is cleared automatically after a START condition has
 *   been sent
 *
 ****************************************************************************/

static inline unsigned int twi_get_start(struct a64_twi_priv_s *priv)
{
  unsigned int reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);
  reg_val >>= 5;
  return reg_val & 0x1;
}

/****************************************************************************
 * Name: twi_set_stop
 *
 * Description:
 *   Trigger stop signal, the stop bit will be cleared automatically
 *
 *   If M_STP is set to ‘1’ in master mode, a STOP condition is
 *   transmitted on the TWI bus. If the M_STP bit is set to ‘1’ in slave
 *   mode, the TWI will behave as if a STOP condition has been received,
 *   but no STOP condition will be transmitted on the TWI bus. If both
 *   M_STA and M_STP bits are set, the TWI will first transmit the STOP
 *   condition (if in master mode) then transmit the START condition.
 *   The M_STP bit is cleared automatically,writing a ‘0’ to this bit has
 *   no effect.
 *
 ****************************************************************************/

static inline void twi_set_stop(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);
  reg_val |= TWI_CNTR_STP;
  reg_val &= ~TWI_CNTR_INTFLG;
  a64_twi_putreg(priv, A64_TWI_CNTR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_get_stop
 *
 * Description:
 *   Get stop bit status, poll if stop signal is sent
 *
 ****************************************************************************/

static inline unsigned int twi_get_stop(struct a64_twi_priv_s *priv)
{
  unsigned int reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);
  reg_val >>= 4;
  return reg_val & 1;
}

/****************************************************************************
 * Name: twi_get_int_flag
 *
 * Description:
 *   Get the interrupt flag
 *
 ****************************************************************************/

static inline unsigned int twi_get_int_flag(struct a64_twi_priv_s *priv)
{
  unsigned int reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);
  return (reg_val & TWI_CNTR_INTFLG);
}

/****************************************************************************
 * Name: twi_clear_irq_flag
 *
 * Description:
 *   clear the interrupt flag
 *
 *   INT_FLAG is automatically set to '1' when any of 28 (out of the
 *   possible 29) states is entered (see ‘STAT Register’ below). The only
 *   state that does not set INT_FLAG is state F8h. If the INT_EN bit is
 *   set, the interrupt line goes high when IFLG is set to '1'. If the TWI
 *   is operating in slave mode, data transfer is suspended when INT_FLAG
 *   is set and the low period of the TWI bus clock line (SCL) is stretched
 *   until '1' is written to INT_FLAG.
 *   The TWI clock line is then released and the interrupt line goes low.
 *
 ****************************************************************************/

static inline void twi_clear_irq_flag(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);

  /* start and stop bit should be 0 */

  reg_val |= TWI_CNTR_INTFLG;
  reg_val &= ~(TWI_CNTR_STA | TWI_CNTR_STP);
  a64_twi_putreg(priv, A64_TWI_CNTR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_get_status
 *
 * Description:
 *   Get the twi status bit 7~0
 *
 ****************************************************************************/

static inline unsigned int twi_get_status(struct a64_twi_priv_s *priv)
{
  unsigned int reg_val = a64_twi_getreg(priv, A64_TWI_STAT_OFFSET);
  return (reg_val & TWI_STAT_MASK);
}

/****************************************************************************
 * Name: twi_softreset
 *
 * Description:
 *   Soft reset twi by A64_TWI_SRST_REG
 *
 *   Write ‘1’ to this bit to reset the TWI and clear to ‘0’ when
 *   completing Soft Reset operation.
 *
 ****************************************************************************/

static inline void twi_softreset(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_SRST_OFFSET);
  reg_val |= TWI_SRST_SRST;
  a64_twi_putreg(priv, A64_TWI_SRST_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_disable_ack
 *
 * Description:
 *   Disable TWI Ack
 *
 *   If A_ACK is cleared to '0' in slave transmit mode, the byte in the
 *   DATA register is assumed to be the 'last byte'. After this byte has
 *   been transmitted, the TWI will enter state C8h then return to the
 *   idle state (status code F8h) when INT_FLAG is cleared.
 *   The TWI will not respond as a slave unless A_ACK is set.
 *
 ****************************************************************************/

static inline void twi_disable_ack(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);
  reg_val &= ~TWI_CNTR_ACK;
  reg_val &= ~TWI_CNTR_INTFLG;
  a64_twi_putreg(priv, A64_TWI_CNTR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_enable_ack
 *
 * Description:
 *   When sending ack or nack, it will send ack automatically
 *
 *   When A_ACK is set to ‘1’, an Acknowledge (low level on SDA) will be
 *   sent during the acknowledge clock pulse on the TWI bus if:
 *   1. Either the whole of a matching 7-bit slave address or the first or
 *   the second byte of a matching 10-bit slave address has been received.
 *   2. The general call address has been received and the GCE bit in the
 *   ADDR register is set to ‘1’.
 *   3. A data byte has been received in master or slave mode.
 *   When A_ACK is '0', a Not Acknowledge (high level on SDA) will be sent
 *   when a data byte is received in master or slave mode.
 *
 ****************************************************************************/

static inline void twi_enable_ack(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_CNTR_OFFSET);
  reg_val |= TWI_CNTR_ACK;
  reg_val &= ~TWI_CNTR_INTFLG;
  a64_twi_putreg(priv, A64_TWI_CNTR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_set_efr
 *
 * Description:
 *   Set Enhanced Feature Register
 *
 *   Data Byte number follow Read Command Control
 *   0— No Data Byte to be written after read command
 *   1— Only 1 byte data to be written after read command
 *   2— 2 bytes data can be written after read command
 *   3— 3 bytes data can be written after read command
 *
 ****************************************************************************/

static void twi_set_efr(struct a64_twi_priv_s *priv, unsigned int efr)
{
  a64_twi_modifyreg(priv, A64_TWI_EFR_OFFSET, TWI_EFR_MASK, efr);
}

#ifdef CONFIG_I2C_RESET
/****************************************************************************
 * Name: twi_enable_scl
 *
 * Description:
 *   enable SCL by A64_TWI_LCR_REG
 *
 *   TWI_SCL line state control enable
 *   When this bit is set, the state of TWI_SCL is control by the value
 *   of bit[3].
 *   0-disable TWI_SCL line control mode
 *   1-enable TWI_SCL line control mode
 *
 ****************************************************************************/

static void twi_enable_scl(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_LCR_OFFSET);
  reg_val |= TWI_LCR_SCL_EN; /* enable scl line control */
  a64_twi_putreg(priv, A64_TWI_LCR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_enable_sda
 *
 * Description:
 *   enable SDA by A64_TWI_LCR_REG
 *
 *   TWI_SDA line state control enable
 *   When this bit is set, the state of TWI_SDA is control by the value
 *   of bit[1].
 *   0-disable TWI_SDA line control mode
 *   1-enable TWI_SDA line control mode
 *
 ****************************************************************************/

static void twi_enable_sda(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_LCR_OFFSET);
  reg_val |= TWI_LCR_SDA_EN; /* enable sda line control */
  a64_twi_putreg(priv, A64_TWI_LCR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_disable_scl
 *
 * Description:
 *   disable SCL by A64_TWI_LCR_REG
 *
 *   TWI_SCL line state control enable
 *   When this bit is set, the state of TWI_SCL is control by the value
 *   of bit[3].
 *   0-disable TWI_SCL line control mode
 *   1-enable TWI_SCL line control mode
 *
 ****************************************************************************/

static void twi_disable_scl(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_LCR_OFFSET);
  reg_val &= ~TWI_LCR_SCL_EN; /* disable scl line control */
  a64_twi_putreg(priv, A64_TWI_LCR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_disable_sda
 *
 * Description:
 *   disable SDA by A64_TWI_LCR_REG
 *
 *   TWI_SDA line state control enable
 *   When this bit is set, the state of TWI_SDA is control by the value
 *   of bit[1].
 *   0-disable TWI_SDA line control mode
 *   1-enable TWI_SDA line control mode
 *
 ****************************************************************************/

static void twi_disable_sda(struct a64_twi_priv_s *priv)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_LCR_OFFSET);
  reg_val &= ~TWI_LCR_SDA_EN; /* disable sda line control */
  a64_twi_putreg(priv, A64_TWI_LCR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_get_scl
 *
 * Description:
 *   get SCL state
 *
 ****************************************************************************/

static unsigned int twi_get_scl(struct a64_twi_priv_s *priv)
{
  unsigned int status = a64_twi_getreg(priv, A64_TWI_LCR_OFFSET);
  status = TWI_LCR_SCL_STATE_MASK & status;
  status >>= 5;
  return (status & 0x1);
}

/****************************************************************************
 * Name: twi_get_sda
 *
 * Description:
 *   get SDA state
 *
 ****************************************************************************/

static unsigned int twi_get_sda(struct a64_twi_priv_s *priv)
{
  unsigned int status = a64_twi_getreg(priv, A64_TWI_LCR_OFFSET);
  status = TWI_LCR_SDA_STATE_MASK & status;
  status >>= 4;
  return (status & 0x1);
}

/****************************************************************************
 * Name: twi_set_scl
 *
 * Description:
 *   set SCL level(high/low), only when SCL enable
 *
 ****************************************************************************/

static void twi_set_scl(struct a64_twi_priv_s *priv, unsigned int hi_lo)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_LCR_OFFSET);
  reg_val &= ~TWI_LCR_SCL_CTL;
  hi_lo   &= 0x01;
  reg_val |= (hi_lo << 3);
  a64_twi_putreg(priv, A64_TWI_LCR_OFFSET, reg_val);
}

/****************************************************************************
 * Name: twi_set_sda
 *
 * Description:
 *   set SDA level(high/low), only when SDA enable
 *
 ****************************************************************************/

static void twi_set_sda(struct a64_twi_priv_s *priv, unsigned int hi_lo)
{
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_LCR_OFFSET);
  reg_val &= ~TWI_LCR_SDA_CTL;
  hi_lo   &= 0x01;
  reg_val |= (hi_lo << 1);
  a64_twi_putreg(priv, A64_TWI_LCR_OFFSET, reg_val);
}
#endif

/****************************************************************************
 * Name: twi_get_lcr
 *
 * Description:
 *   Get the twi lcr register bit 5~0
 *
 ****************************************************************************/

static inline unsigned int twi_get_lcr(struct a64_twi_priv_s *priv)
{
  unsigned int reg_val = a64_twi_getreg(priv, A64_TWI_LCR_OFFSET);
  return (reg_val & TWI_LCR_MASK);
}

/****************************************************************************
 * Name: twi_get_byte
 *
 * Description:
 *   Get data first, then clear flag
 *
 ****************************************************************************/

static inline unsigned char twi_get_byte(struct a64_twi_priv_s *priv)
{
  unsigned int reg_val = a64_twi_getreg(priv, A64_TWI_DATA_OFFSET);
  twi_clear_irq_flag(priv);
  return (unsigned char)reg_val;
}

/****************************************************************************
 * Name: twi_put_byte
 *
 * Description:
 *   Write data and clear irq flag to trigger send flow
 *
 ****************************************************************************/

static inline void twi_put_byte(struct a64_twi_priv_s *priv, uint8_t ch)
{
  a64_twi_putreg(priv, A64_TWI_DATA_OFFSET, ch);

  /* clear any pending interrupt -- that'll cause sending */

  twi_clear_irq_flag(priv);
}

/****************************************************************************
 * Name: twi_stop
 *
 * Description:
 *   Stop TWI and clear the irq flag
 *
 ****************************************************************************/

static int twi_stop(struct a64_twi_priv_s *priv)
{
  int i;

  twi_set_stop(priv);
  twi_clear_irq_flag(priv);

  /* it must delay 1 nop to check stop bit */

  twi_get_stop(priv);
  for (i = 0; i < 0xff; i++)
    {
      if (!twi_get_stop(priv))
        {
          break;
        }
    }

  if (i >= 0xff)
    {
      i2cerr("ERROR: TWI STOP can't sendout!\n");
      return -ETIMEDOUT;
    }

  for (i = 0; i < 0xff; i++)
    {
      if (TWI_STAT_IDLE == twi_get_status(priv))
        {
          break;
        }
    }

  if (i >= 0xff)
    {
      i2cerr("ERROR: TWI state(0x%08x) isn't idle(0xf8)\n",
             twi_get_status(priv));
      return -ETIMEDOUT;
    }

  for (i = 0; i < 0xff; i++)
    {
      if (TWI_LCR_IDLE_STATUS == twi_get_lcr(priv))
        {
          break;
        }
    }

  if (i >= 0xff)
    {
      i2cerr("ERROR: TWI lcr(0x%08x) isn't idle(0x3a)\n",
             twi_get_lcr(priv));
      return -ETIMEDOUT;
    }

  return OK;
}

/****************************************************************************
 * Name: twi_wait
 *
 * Description:
 *   Perform a I2C transfer start
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static int twi_wait(struct a64_twi_priv_s *priv, unsigned int size)
{
  uint32_t timeout;
  int ret;

  /* Calculate a timeout value based on the size of the transfer
   *
   *   ticks = msec-per-byte * bytes / msec-per-tick
   *
   * There is no concern about arithmetic overflow for reasonable transfer
   * sizes.
   */

  timeout = MSEC2TICK(size * TWI_TIMEOUT_MSPB);
  if (timeout < 1)
    {
      timeout = 1;
    }

  /* Then start the timeout.  This timeout is needed to avoid hangs if/when
   * a TWI transfer stalls.
   */

  wd_start(&priv->timeout, timeout, twi_timeout, (wdparm_t)priv);

  /* Wait for either the TWI transfer or the timeout to complete */

  do
    {
      i2cinfo("TWI%d Waiting...\n", priv->config->twi);
      ret = nxsem_wait(&priv->waitsem);
      i2cinfo("TWI%d Awakened with result: %d\n",
              priv->config->twi, priv->result);

      if (ret < 0)
        {
          wd_cancel(&priv->timeout);
          return ret;
        }
    }
  while (priv->result == -EBUSY);

  /* We get here via twi_wakeup.  The watchdog timer has been disabled and
   * all further interrupts for the TWI have been disabled.
   */

  return priv->result;
}

/****************************************************************************
 * Name: twi_wakeup
 *
 * Description:
 *   A terminal event has occurred.  Wake-up the waiting thread
 *
 ****************************************************************************/

static void twi_wakeup(struct a64_twi_priv_s *priv, int result)
{
  /* Cancel any pending timeout */

  wd_cancel(&priv->timeout);

  /* Disable any further TWI interrupts */

  twi_disable_irq(priv);

  /* Wake up the waiting thread with the result of the transfer */

  priv->result = result;
  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: a64_twi_isr_process
 *
 * Description:
 *   Common Interrupt Service Routine, Status Information Byte Code Status:
 *   0x00: Bus error
 *   0x08: START condition transmitted
 *   0x10: Repeated START condition transmitted
 *   0x18: Address + Write bit transmitted, ACK received
 *   0x20: Address + Write bit transmitted, ACK not received
 *   0x28: Data byte transmitted in master mode, ACK received
 *   0x30: Data byte transmitted in master mode, ACK not received
 *   0x38: Arbitration lost in address or data byte
 *   0x40: Address + Read bit transmitted, ACK received
 *   0x48: Address + Read bit transmitted, ACK not received
 *   0x50: Data byte received in master mode, ACK transmitted
 *   0x58: Data byte received in master mode, not ACK transmitted
 *   0x60: Slave address + Write bit received, ACK transmitted
 *   0x68: Arbitration lost in address as master, slave address + Write bit
 *         received, ACK transmitted
 *   0x70: General Call address received, ACK transmitted
 *   0x78: Arbitration lost in address as master, General Call address
 *         received, ACK transmitted
 *   0x80: Data byte received after slave address received, ACK transmitted
 *   0x88: Data byte received after slave address received, not ACK
 *         transmitted
 *   0x90: Data byte received after General Call received, ACK transmitted
 *   0x98: Data byte received after General Call received, not ACK
 *         transmitted
 *   0xA0: STOP or repeated START condition received in slave mode
 *   0xA8: Slave address + Read bit received, ACK transmitted
 *   0xB0: Arbitration lost in address as master, slave address + Read bit
 *         received, ACK transmitted
 *   0xB8: Data byte transmitted in slave mode, ACK received
 *   0xC0: Data byte transmitted in slave mode, ACK not received
 *   0xC8: Last byte transmitted in slave mode, ACK received
 *   0xD0: Second Address byte + Write bit transmitted, ACK received
 *   0xD8: Second Address byte + Write bit transmitted, ACK not received
 *   0xF8: No relevant status information, INT_FLAG=0
 *   Others: Reserved
 *
 ****************************************************************************/

static int a64_twi_isr_process(struct a64_twi_priv_s *priv)
{
  unsigned char addr = 0;
  int err_code   = 0;
  int ret;
  uint32_t status = twi_get_status(priv);

  switch (status)
    {
      case 0xf8: /* On reset or stop the bus is idle, used only on polled */
        err_code = 0xf8;
        goto errout;
      case 0x08: /* A START condition has been transmitted */
      case 0x10: /* A repeated start condition has been transmitted */

        twi_put_addr(priv, priv->msgv->addr); /* send slave address */
        break;

      case 0xd8: /* second addr has transmitted, ACK not received! */
      case 0x20: /* SLA+W has been transmitted; NOT ACK has been received */
        err_code = 0x20;
        goto errout;
      case 0x18: /* SLA+W has been transmitted; ACK has been received */

        /* if 10bit address mode, send second part of 10 bits addr */

        if (priv->flags & I2C_M_TEN)
          {
            /* the remaining 8 bits of address */

            addr = priv->msgv->addr & 0xff;
            twi_put_byte(priv, addr); /* case 0xd0: */
            break;
          }

      /* for 7 bit addr, then directly send data byte--case 0xd0:  */

      case 0xd0: /* second addr has transmitted,ACK received! */

      /* Data byte in DATA REG has been transmitted; ACK has been received */

      case 0x28:

        if (priv->dcnt <= 0)
          {
            priv->msgc--;
            if (priv->msgc > 0)
              {
                priv->msgv++;
                priv->flags = priv->msgv->flags;
                priv->ptr   = priv->msgv->buffer;
                priv->dcnt  = priv->msgv->length;
                if ((priv->flags & I2C_M_NOSTART) == 0)
                  {
                    twi_set_start(priv);
                    twi_clear_irq_flag(priv);
                    break;
                  }
              }
          }

        /* after send register address then START send write data  */

        if (priv->dcnt > 0)
          {
            /* No interrupts or context switches should occur in the
             * following sequence. Otherwise, additional bytes may be
             * sent by the device
             */

  #ifdef CONFIG_I2C_POLLED
            irqstate_t flags = enter_critical_section();
  #endif

            /* Transmit a byte */

            twi_put_byte(priv, *priv->ptr++);
            priv->dcnt--;

  #ifdef CONFIG_I2C_POLLED
            leave_critical_section(flags);
  #endif

            break;
          }
        else
          {
            goto okout;
          }

      case 0x30: /* Data byte has been transmitted; NO ACK has been received */

        err_code = 0x30; /* err,wakeup the thread */
        goto errout;

      case 0x38: /* Arbitration lost during SLA+W, SLA+R or data bytes */

        err_code = 0x38; /* err,wakeup the thread */
        goto errout;

      case 0x40: /* SLA+R has been transmitted; ACK has been received */

        /* with Restart,needn't to send second part of 10 bits addr
         * refer-"I2C-SPEC v2.1" enable A_ACK need it(receive data len)
         * more than 1.
         */

        if (priv->dcnt > 1)
          {
            /* send register addr complete,then enable the A_ACK and
             * get ready for receiving data
             */

            twi_enable_ack(priv);
            twi_clear_irq_flag(priv); /* jump to case 0x50 */
          }
        else if (priv->dcnt == 1)
          {
            twi_clear_irq_flag(priv); /* jump to case 0x58 */
          }
        else
          {
            err_code = 0x40;
            goto errout;
          }
        break;

      case 0x48: /* SLA+R has been transmitted; NOT ACK has been received */

        err_code = 0x48; /* err,wakeup the thread */
        goto errout;

      /* Data bytes has been received; ACK has been transmitted */

      case 0x50:

        /* receive first data byte */

        if (priv->dcnt > 0)
          {
            /* get data then clear flag,then next data comming */

            *priv->ptr++ = twi_get_byte(priv);
            priv->dcnt--;

            /* the last byte need not to send ACK but send NACK */

            if (priv->dcnt == 1)
              {
                twi_disable_ack(priv); /* last byte no ACK */
              }
            break;
          }

        /* err process, the last byte should processed at case 0x58 */

        err_code = 0x50; /* err, wakeup */
        goto errout;

      /* Data byte has been received; NACK has been transmitted */

      case 0x58:

        /* received the last byte */

        if (priv->dcnt == 1)
          {
            *priv->ptr++ = (uint8_t)a64_twi_getreg(priv,
                                                   A64_TWI_DATA_OFFSET);
            priv->msgc--;
            if (priv->msgc > 0)
              {
                priv->msgv++;
                priv->flags = priv->msgv->flags;
                priv->ptr   = priv->msgv->buffer;
                priv->dcnt  = priv->msgv->length;
                twi_set_start(priv);
                twi_clear_irq_flag(priv);
                break;
              }
            else
              {
                priv->dcnt--;
                goto okout;
              }
          }
        else
          {
            err_code = 0x58;
            goto errout;
          }

      case 0x00:

        /* Bus error during master or slave mode due to illegal level
         * condition
         */

        err_code = 0xff;
        goto errout;
      default:
        err_code = status;
        goto errout;
    }

  return OK;

errout:
  ret = -EIO;
  i2cerr("TWI%d error code = 0x%x\n", priv->config->twi, err_code);

okout:
  if (twi_stop(priv) < 0)
    {
      i2cerr("TWI%d STOP failed!\n", priv->config->twi);
      ret = -EIO;
    }
  else
    {
      ret = OK;
    }

  priv->intstate = INTSTATE_DONE;

  /* Wake up the thread with an I/O error indication */

  twi_wakeup(priv, ret);
  return ret;
}

/****************************************************************************
 * Name: twi_interrupt
 *
 * Description:
 *   The TWI Interrupt Handler
 *
 ****************************************************************************/

static int twi_interrupt(int irq, void *context, void *arg)
{
  struct a64_twi_priv_s *priv = (struct a64_twi_priv_s *)arg;
  int ret;

  DEBUGASSERT(priv != NULL);

  if (!twi_get_int_flag(priv))
    {
      i2cerr("unknown interrupt!\n");
      return OK;
    }

  /* disable irq */

  twi_disable_irq(priv);

  /* interrupt service routine */

  ret = a64_twi_isr_process(priv);

  /* enable irq only when twi is transfering, otherwise disable irq */

  if (priv->intstate != INTSTATE_IDLE)
    {
      twi_enable_irq(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: twi_timeout
 *
 * Description:
 *   Watchdog timer for timeout of TWI operation
 *
 * Assumptions:
 *   Called from the timer interrupt handler with interrupts disabled.
 *
 ****************************************************************************/

static void twi_timeout(wdparm_t arg)
{
  struct a64_twi_priv_s *priv = (struct a64_twi_priv_s *)arg;

  i2cerr("ERROR: TWI%d Timeout!\n", priv->config->twi);
  twi_wakeup(priv, -ETIMEDOUT);
}

/****************************************************************************
 * Name: twi_put_addr
 *
 * Description:
 *   7bits addr: 7-1bits addr+0 bit r/w
 *   10bits addr: 1111_11xx_xxxx_xxxx-->1111_0xx_rw,xxxx_xxxx
 *   send the 7 bits addr,or the first part of 10 bits addr
 *
 ****************************************************************************/

static void twi_put_addr(struct a64_twi_priv_s *priv, uint16_t address)
{
  unsigned char addr = 0;
  unsigned char tmp  = 0;

  if (priv->flags & I2C_M_TEN)
    {
      /* 0111_10xx,ten bits address--9:8bits */

      tmp = 0x78 | ((address >> 8) & 0x03);

      /* the second part of ten bits addr deal at a64_twi_isr_process */

      addr = tmp << 1; /* 1111_0xx0 */
    }
  else
    {
      /* 7-1bits addr, xxxx_xxx0 */

      addr = (address & 0x7f) << 1;
    }

  /* read, default value is write */

  if (priv->flags & I2C_M_READ)
    {
      addr |= 1;
    }

  if (priv->flags & I2C_M_TEN)
    {
      i2cinfo("TWI address first part of 10bits = 0x%x\n", addr);
    }

  i2cinfo("TWI address 7bits+r/w = 0x%x\n", addr);

  /* send 7bits+r/w or the first part of 10bits */

  twi_put_byte(priv, addr);
}

/****************************************************************************
 * I2C device operations
 ****************************************************************************/

/****************************************************************************
 * Name: twi_transfer
 *
 * Description:
 *   Receive a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 * Returned Value:
 *   Returns zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int twi_transfer(struct i2c_master_s *dev,
                        struct i2c_msg_s *msgs, int count)
{
  struct a64_twi_priv_s *priv = (struct a64_twi_priv_s *)dev;
  irqstate_t flags;
  unsigned int size;
  int i;
  int ret;

  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);
  i2cinfo("TWI%d count: %d\n", priv->config->twi, count);

  /* Calculate the total transfer size so that we can calculate a
   * reasonable timeout value.
   */

  size = 0;
  for (i = 0; i < count; i++)
    {
      size += msgs[i].length;
    }

  DEBUGASSERT(size > 0);

  /* Get exclusive access to the device */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Setup the message transfer */

  priv->msgv  = msgs;
  priv->msgc  = count;
  priv->flags = priv->msgv->flags;
  priv->ptr   = priv->msgv->buffer;
  priv->dcnt  = priv->msgv->length;

  /* Configure the I2C frequency.
   * REVISIT: Note that the frequency is set only on the first message.
   * This could be extended to support different transfer frequencies for
   * each message segment.
   */

  twi_setclock(priv, msgs->frequency);

  flags = enter_critical_section();

  /* Initiate the transfer.  The rest will be handled from interrupt
   * logic.  Interrupts must be disabled to prevent re-entrance from the
   * interrupt level.
   */

  twi_enable_irq(priv);  /* enable irq */
  twi_disable_ack(priv); /* disabe ACK */

  /* No Data Byte to be written after read command */

  twi_set_efr(priv, 0);  /* set the special function register,default:0 */

  priv->intstate = INTSTATE_WAITING;

  /* START signal, needn't clear int flag */

  twi_set_start(priv);
  for (i = 0; i < 0xff; i++)
    {
      if (!twi_get_start(priv))
        {
          break;
        }
    }

  if (i >= 0xff)
    {
      twi_softreset(priv);
      twi_disable_irq(priv); /* disable irq */
      i2cerr("ERROR: START can't sendout\n");
      ret = -ETIMEDOUT;
      goto out;
    }

  /* And wait for the transfers to complete. Interrupts will be re-enabled
   * while we are waiting.
   */

  ret = twi_wait(priv, size);
  if (ret < 0)
    {
      i2cerr("ERROR: Transfer failed: %d\n", ret);
    }

out:

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  /* Release the port for re-use by other clients */

  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);
  return ret;
}

#ifdef CONFIG_I2C_RESET
/****************************************************************************
 * Name: twi_reset
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

static int twi_reset(struct i2c_master_s *dev)
{
  struct a64_twi_priv_s *priv = (struct a64_twi_priv_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv);

  /* Get exclusive access to the TWI device */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Disable TWI interrupts */

  up_disable_irq(priv->config->irq);

  /* send 9 clock to release sda */

  ret = twi_release(priv);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  /* enable scl sda control */

  twi_enable_scl(priv);
  twi_enable_sda(priv);

  /* Generate a start followed by a stop to reset slave state machines */

  twi_set_sda(priv, 0);
  up_udelay(10);
  twi_set_scl(priv, 0);
  up_udelay(10);

  twi_set_scl(priv, 1);
  up_udelay(10);
  twi_set_sda(priv, 1);
  up_udelay(10);

  /* disable scl sda control */

  twi_disable_scl(priv);
  twi_disable_sda(priv);

  /* Re-initialize the port hardware */

  twi_hw_initialize(priv);

  /* Restore the frequency */

  twi_setclock(priv, priv->frequency);
  ret = OK;

errout_with_lock:

  /* Release our lock on the bus */

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: twi_release
 *
 * Description:
 *   send 9 clock to release sda
 *
 ****************************************************************************/

static int twi_release(struct a64_twi_priv_s *priv)
{
  int i = 0;

  /* enable scl control */

  twi_enable_scl(priv);

  for (i = 0; i < 9; i++)
    {
      if (twi_get_sda(priv) && twi_get_sda(priv) && twi_get_sda(priv))
        {
          break;
        }

      /* twi_scl -> low */

      twi_set_scl(priv, 0);

      /* Wait 1 millisecond */

      up_mdelay(1);

      /* twi_scl -> high */

      twi_set_scl(priv, 1);

      /* Wait 1 millisecond */

      up_mdelay(1);
    }

  if (twi_get_sda(priv))
    {
      twi_disable_scl(priv);
      return OK;
    }
  else
    {
      i2cerr("ERROR: SDA is still stuck low, failed. \n");
      twi_disable_scl(priv);
      return -EIO;
    }
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Initialization
 ****************************************************************************/

/****************************************************************************
 * Name: twi_setclock
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 *   Fin is APB CLOCK INPUT;
 *   Fsample = F0 = Fin/2^CLK_N;
 *   F1 = F0/(CLK_M+1);
 *   Foscl = F1/10 = Fin/(2^CLK_N * (CLK_M+1)*10);
 *   Foscl is clock SCL;100KHz or 400KHz
 *
 *   clk_in: apb clk clock
 *   sclk_req: freqence to set in HZ
 ****************************************************************************/

static void twi_setclock(struct a64_twi_priv_s *priv, uint32_t freq)
{
  uint32_t tmp_clk;
  uint32_t n;
  uint32_t m;
  uint32_t baud = 0x44; /* baudrate at controller reset */
  uint32_t reg_val = a64_twi_getreg(priv, A64_TWI_CCR_OFFSET);

  if (freq != priv->frequency)
    {
      /* compute m, n setting for highest speed not above requested speed */

      for (n = 0; n < 8; n++)
        {
          for (m = 0; m < 16; m++)
            {
              tmp_clk = TWI_FREQUENCY(m, n);
              if (tmp_clk <= freq)
                {
                  baud = (m << 3) | n;
                  goto set_clk;
                }
            }
        }

set_clk:
      reg_val &= ~(TWI_CLK_DIV_M | TWI_CLK_DIV_N);
      reg_val |= baud;
      a64_twi_putreg(priv, A64_TWI_CCR_OFFSET, reg_val);

      /* Save the requested frequency */

      priv->frequency = freq;
    }
}

/****************************************************************************
 * Name: twi_hw_initialize
 *
 * Description:
 *   Initialize/Re-initialize the TWI peripheral.  This logic performs only
 *   repeatable initialization after either (1) the one-time initialization,
 *   or (2) after each bus reset.
 *
 ****************************************************************************/

static void twi_hw_initialize(struct a64_twi_priv_s *priv)
{
  irqstate_t flags = enter_critical_section();

  i2cinfo("TWI%d Initializing\n", priv->config->twi);

  /* Configure pins */

  a64_pio_config(priv->config->scl_pin);
  a64_pio_config(priv->config->sda_pin);

  /* Enable i2c bus */

  twi_enable(priv);

  /* Set the initial TWI data transfer frequency */

  priv->frequency = 0;
  twi_setclock(priv, 100000);

#ifndef CONFIG_I2C_POLLED
  /* Attach ISRs */

  irq_attach(priv->config->irq, twi_interrupt, priv);

  /* Set Interrupt Priority in Generic Interrupt Controller v2 */

  arm64_gic_irq_set_priority(priv->config->irq, IRQ_TYPE_LEVEL, 0);

  /* Enable TWI Interrupt */

  up_enable_irq(priv->config->irq);
#endif

  /* soft reset twi */

  twi_softreset(priv);

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a64_i2cbus_initialize
 *
 * Description:
 *   Initialize a TWI device for I2C operation
 *
 ****************************************************************************/

struct i2c_master_s *a64_i2cbus_initialize(int bus)
{
  struct a64_twi_priv_s *priv = NULL;

  switch (bus)
    {
#ifdef CONFIG_A64_TWI0
      case 0:
        priv = (struct a64_twi_priv_s *)&a64_twi0_priv;
        break;
#endif

#ifdef CONFIG_A64_TWI1
      case 1:
        priv = (struct a64_twi_priv_s *)&a64_twi1_priv;
        break;
#endif

#ifdef CONFIG_A64_TWI2
      case 2:
        priv = (struct a64_twi_priv_s *)&a64_twi2_priv;
        break;
#endif

#ifdef CONFIG_A64_RTWI
      case 3:
        priv = (struct a64_twi_priv_s *)&a64_rtwi_priv;
        break;
#endif

      default:
        return NULL;
    }

  /* TODO: R_TWI is not on APB2 but APBS */

  if (bus < 3)
    {
      /* Bus Clock Gating Register 3 (A64 Page 105) */

      modreg32(1 << bus, 1 << bus, BUS_CLK_GATING_REG3);

      /* Bus Software Reset Register 4 (A64 Page 143) */

      modreg32(1 << bus, 1 << bus, BUS_SOFT_RST_REG4);
    }

  /* Initialize private data for the first time, increment reference count,
   * power-up hardware and configure GPIOs.
   */

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      twi_hw_initialize(priv);
    }

  nxmutex_unlock(&priv->lock);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: a64_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C device
 *
 ****************************************************************************/

int a64_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct a64_twi_priv_s *priv = (struct a64_twi_priv_s *)dev;

  i2cinfo("TWI%d Un-initializing\n", priv->config->twi);

  /* Disable TWI interrupts */

  up_disable_irq(priv->config->irq);

  /* Disable i2c bus */

  twi_disable(priv);

  /* Detach Interrupt Handler */

  irq_detach(priv->config->irq);
  return OK;
}

#endif /* CONFIG_A64_TWI0 || ... || CONFIG_A64_RTWI */
