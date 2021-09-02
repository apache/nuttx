/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_riic.c
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

#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/semaphore.h>
#include <nuttx/kthread.h>
#include "rx65n_riic.h"
#include "rx65n_definitions.h"
#include <arch/board/board.h>
#include <arch/board/rx65n_gpio.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include "up_internal.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

# define rx65n_getreg(addr)      getreg8(addr)
# define rx65n_putreg(val,addr)  putreg8(val,addr)

#if defined(CONFIG_RX65N_RIIC0) || defined(CONFIG_RX65N_RIIC1) || \
    defined(CONFIG_RX65N_RIIC2)

/****************************************************************************
 * Typedef definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum rx65n_i2c_mode_e
{
  RIIC_NONE = 0,             /* Uninitialized state */
  RIIC_READY,                /* Ready for operation */
  RIIC_M_TRANSMIT,           /* Master transmission mode */
  RIIC_M_RECEIVE,            /* Master reception mode */
  RIIC_FINISH,               /* Successful operation */
};

enum rx65n_i2c_dev_sts_e
{
  RIIC_STS_NO_INIT = 0,        /* None initialization state */
  RIIC_STS_IDLE,               /* Idle state */
  RIIC_STS_ST_COND_WAIT,       /* Start condition generation completion wait state */
  RIIC_STS_SEND_SLVADR_W_WAIT, /* Slave address [Write] transmission completion wait state */
  RIIC_STS_SEND_SLVADR_R_WAIT, /* Slave address [Read] transmission completion wait state */
  RIIC_STS_SEND_DATA_WAIT,     /* Data transmission completion wait state */
  RIIC_STS_RECEIVE_DATA_WAIT,  /* Data reception completion wait state */
  RIIC_STS_SP_COND_WAIT,       /* Stop condition generation completion wait state */
  RIIC_STS_AL,                 /* Detect Arbitration Lost */
  RIIC_STS_TMO,                /* Detect Time out */
  RIIC_STS_MAX,                /* Prohibition of setup above here */
};

enum rx65n_i2c_event_e
{
  RIIC_EV_NONE = 0,
  RIIC_EV_GEN_START_COND,  /* Called function of Start condition generation */
  RIIC_EV_INT_START,       /* Interrupted start codition generation */
  RIIC_EV_INT_ADD,         /* Interrupted address sending */
  RIIC_EV_INT_SEND,        /* Interrupted data sending */
  RIIC_EV_INT_RECEIVE,     /* Interrupted data receiving */
  RIIC_EV_INT_STOP,        /* Interrupted Stop condition generation */
  RIIC_EV_INT_AL,          /* Interrupted Arbitration-Lost */
  RIIC_EV_INT_NACK,        /* Interrupted No Acknowledge */
  RIIC_EV_INT_TXI,         /* Interrupted transmitted buffer empty */
  RIIC_EV_INT_TMO,         /* Interrupted Time out */
  RIIC_EV_MAX,             /* Prohibition of setup above here */
};

struct rx65n_i2c_dev_s
{
  uint32_t  base;       /* Base address of registers */
  uint32_t  frequency;  /* Configured kbps */
  uint8_t   txi_irq;    /* TXI IRQ Number */
  uint8_t   rxi_irq;    /* RXI IRQ Number */
  uint8_t   tei_irq;    /* TEI IRQ Number */
  uint8_t   eei_irq;    /* EEI IRQ Number */
  uint32_t  grpbase;
  uint32_t  teimask;
  uint32_t  eeimask;
};

struct rx65n_i2c_priv_s
{
  const struct      i2c_ops_s *ops;
  const struct      rx65n_i2c_dev_s *dev;
  int refs;                          /* Referernce count */
  int bus;                           /* Bus number */
  volatile uint8_t  mode;            /* See enum rx65n_i2c_mode_e */
  volatile uint8_t  dev_sts;         /* See enum rx65n_i2c_dev_sts_e */
  volatile uint8_t  event;           /* See enum rx65n_i2c_event_e */
  sem_t             sem_excl;        /* Mutual exclusion semaphore */
  sem_t             sem_isr;         /* Interrupt wait semaphore */
  uint8_t           msgc;            /* Number of Messages */
  struct            i2c_msg_s *msgv; /* Message list */
  uint8_t           *ptr;            /* Current message buffer */
  int               dcnt;            /* Bytes remaining to transfer */
  uint16_t          flags;           /* Current message flags */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rx65n_riic_iicrst(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_setclock(FAR struct rx65n_i2c_priv_s *priv,
                                 uint32_t frequency);
static void rx65n_riic_init(FAR struct rx65n_i2c_priv_s *priv);
static int rx65n_riic_startcond(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_restartcond(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_send_slv_addr(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_after_send_slvadr(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_master_transmit(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_master_receive(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_stopcond(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_advance(FAR struct rx65n_i2c_priv_s *priv);
static uint8_t rx65n_riic_read_data(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_wait_set(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_pre_end_set(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_end_set(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_nack(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_set_sending_data(FAR struct rx65n_i2c_priv_s *priv, \
                                          uint8_t data);
static int rx65n_riic_after_stop(FAR struct rx65n_i2c_priv_s *priv);
static bool rx65n_riic_check_bus_busy(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_set_icier(FAR struct rx65n_i2c_priv_s *priv, \
                                          uint8_t value);
static void rx65n_riic_timeout(FAR struct rx65n_i2c_priv_s *priv);

/* RIIC Interrupt Handling */

static void rx65n_riic_int_disable(FAR struct rx65n_i2c_priv_s *priv);
static void rx65n_riic_int_enable(FAR struct rx65n_i2c_priv_s *priv);
static int rx65n_riic_irq_init(FAR struct rx65n_i2c_priv_s *priv);

/* RIIC0 Channel Interrupt Handling */

static int rx65n_riic_rxi0interrupt(int irq,
  FAR void *context, FAR void *arg);
static int rx65n_riic_txi0interrupt(int irq,
  FAR void *context, FAR void *arg);
static int rx65n_riic_eei0interrupt(int irq,
  FAR void *context, FAR void *arg);
static int rx65n_riic_tei0interrupt(int irq,
  FAR void *context, FAR void *arg);

/* RIIC1 Channel Interrupt Handling */

static int rx65n_riic_rxi1interrupt(int irq,
  FAR void *context, FAR void *arg);
static int rx65n_riic_txi1interrupt(int irq,
  FAR void *context, FAR void *arg);
static int rx65n_riic_eei1interrupt(int irq,
  FAR void *context, FAR void *arg);
static int rx65n_riic_tei1interrupt(int irq,
  FAR void *context, FAR void *arg);

/* RIIC2 Channel Interrupt Handling */

static int rx65n_riic_rxi2interrupt(int irq,
  FAR void *context, FAR void *arg);
static int rx65n_riic_txi2interrupt(int irq,
  FAR void *context, FAR void *arg);
static int rx65n_riic_eei2interrupt(int irq,
  FAR void *context, FAR void *arg);
static int rx65n_riic_tei2interrupt(int irq,
  FAR void *context, FAR void *arg);

/* I2C operations */

static int rx65n_i2c_transfer(FAR struct i2c_master_s *dev, \
                                FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int rx65n_i2c_reset(FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void up_enable_irq(int irq);
void up_disable_irq(int irq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Device Structures, Instantiation */

static const struct i2c_ops_s rx65n_i2c_ops =
{
  .transfer = rx65n_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = rx65n_i2c_reset
#endif
};

#ifdef CONFIG_RX65N_RIIC0
static const struct rx65n_i2c_dev_s rx65n_riic0_dev =
{
  .base       = RX65N_RIIC0_BASE,
  .frequency  = CONFIG_RX65N_RIIC0_BITRATE,
  .txi_irq    = RX65N_RIIC0_TXI0_IRQ,
  .rxi_irq    = RX65N_RIIC0_RXI0_IRQ,
  .tei_irq    = RX65N_RIIC0_TEI0_IRQ,
  .eei_irq    = RX65N_RIIC0_EEI0_IRQ,
  .grpbase    = RX65N_GRPBL1_ADDR,
  .teimask    = RX65N_GRPBL1_TEI0_MASK,
  .eeimask    = RX65N_GRPBL1_EEI0_MASK,
};

static struct rx65n_i2c_priv_s rx65n_riic0_priv =
{
  .ops       = &rx65n_i2c_ops,
  .dev       = &rx65n_riic0_dev,
  .refs      = 0,
  .bus       = 0,
  .mode      = RIIC_NONE,
  .dev_sts   = RIIC_STS_NO_INIT,
  .event     = RIIC_EV_NONE,
  .msgc      = 0,
  .msgv      = NULL,
  .ptr       = NULL,
  .dcnt      = 0,
  .flags     = 0,
};
#endif

#ifdef CONFIG_RX65N_RIIC1
static const struct rx65n_i2c_dev_s rx65n_riic1_dev =
{
  .base      = RX65N_RIIC1_BASE,
  .frequency = CONFIG_RX65N_RIIC1_BITRATE,
  .txi_irq   = RX65N_RIIC1_TXI1_IRQ,
  .rxi_irq   = RX65N_RIIC1_RXI1_IRQ,
  .tei_irq   = RX65N_RIIC1_TEI1_IRQ,
  .eei_irq   = RX65N_RIIC1_EEI1_IRQ,
  .grpbase   = RX65N_GRPBL1_ADDR,
  .teimask   = RX65N_GRPBL1_TEI1_MASK,
  .eeimask   = RX65N_GRPBL1_EEI1_MASK,
};

static struct rx65n_i2c_priv_s rx65n_riic1_priv =
{
  .ops       = &rx65n_i2c_ops,
  .dev       = &rx65n_riic1_dev,
  .refs      = 0,
  .bus       = 1,
  .mode      = RIIC_NONE,
  .dev_sts   = RIIC_STS_NO_INIT,
  .event     = RIIC_EV_NONE,
  .msgc      = 0,
  .msgv      = NULL,
  .ptr       = NULL,
  .dcnt      = 0,
  .flags     = 0,
};
#endif

#ifdef CONFIG_RX65N_RIIC2
static const struct rx65n_i2c_dev_s rx65n_riic2_dev =
{
  .base      = RX65N_RIIC2_BASE,
  .frequency = CONFIG_RX65N_RIIC2_BITRATE,
  .txi_irq   = RX65N_RIIC2_TXI2_IRQ,
  .rxi_irq   = RX65N_RIIC2_RXI2_IRQ,
  .tei_irq   = RX65N_RIIC2_TEI2_IRQ,
  .eei_irq   = RX65N_RIIC2_EEI2_IRQ,
  .grpbase   = RX65N_GRPBL1_ADDR,
  .teimask   = RX65N_GRPBL1_TEI2_MASK,
  .eeimask   = RX65N_GRPBL1_EEI2_MASK,
};

static struct rx65n_i2c_priv_s rx65n_riic2_priv =
{
  .ops       = &rx65n_i2c_ops,
  .dev       = &rx65n_riic2_dev,
  .refs      = 0,
  .bus       = 2,
  .mode      = RIIC_NONE,
  .dev_sts   = RIIC_STS_NO_INIT,
  .event     = RIIC_EV_NONE,
  .msgc      = 0,
  .msgv      = NULL,
  .ptr       = NULL,
  .dcnt      = 0,
  .flags     = 0,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riic_mpc_enable
 *
 * Description:
 * Enable writing to registers
 ****************************************************************************/

static void riic_mpc_enable(void)
{
  /* Enable writing to registers related to operating modes,
   * LPC, CGC and software reset
   */

  SYSTEM.PRCR.WORD = 0xa50b;

  /* Enable writing to MPC pin function control registers */

  MPC.PWPR.BIT.B0WI = 0;
  MPC.PWPR.BIT.PFSWE = 1;
}

/****************************************************************************
 * Name: riic_mpc_disable
 *
 * Description:
 * Disable writing to registers
 ****************************************************************************/

static void riic_mpc_disable(void)
{
  /* Disable writing to MPC pin function control registers */

  MPC.PWPR.BIT.PFSWE = 0;
  MPC.PWPR.BIT.B0WI = 1;

  /* Enable protection */

  SYSTEM.PRCR.WORD = 0xa500;
}

/****************************************************************************
 * Name: rx65n_riic_iicrst
 *
 * Description:
 *   Disable writing to registers
 ****************************************************************************/

static int rx65n_riic_iicrst(FAR struct rx65n_i2c_priv_s *priv)
{
  uint8_t regval;

  if (priv->bus == 0)
    {
      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);
      regval |= RX65N_RIIC_ICCR1_IICRST_SET;
      rx65n_putreg(regval, RX65N_RIIC0_ICCR1);
      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);

      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);
      regval &= ~(RX65N_RIIC_ICCR1_IICRST_SET);
      rx65n_putreg(regval, RX65N_RIIC0_ICCR1);
      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);
    }

  else if (priv->bus == 1)
    {
      regval = rx65n_getreg(RX65N_RIIC1_ICCR1);
      regval |= RX65N_RIIC_ICCR1_IICRST_SET;
      rx65n_putreg(regval, RX65N_RIIC1_ICCR1);
      regval = rx65n_getreg(RX65N_RIIC1_ICCR1);

      regval = rx65n_getreg(RX65N_RIIC1_ICCR1);
      regval &= ~(RX65N_RIIC_ICCR1_IICRST_SET);
      rx65n_putreg(regval, RX65N_RIIC1_ICCR1);
      regval = rx65n_getreg(RX65N_RIIC1_ICCR1);
    }

  else
    {
      regval = rx65n_getreg(RX65N_RIIC2_ICCR1);
      regval |= RX65N_RIIC_ICCR1_IICRST_SET;
      rx65n_putreg(regval, RX65N_RIIC2_ICCR1);
      regval = rx65n_getreg(RX65N_RIIC2_ICCR1);

      regval = rx65n_getreg(RX65N_RIIC2_ICCR1);
      regval &= ~(RX65N_RIIC_ICCR1_IICRST_SET);
      rx65n_putreg(regval, RX65N_RIIC2_ICCR1);
      regval = rx65n_getreg(RX65N_RIIC2_ICCR1);
    }

  return OK;
}

/****************************************************************************
 * Name: rx65n_riic_set_icier
 *
 * Description:
 *   Sets interrupt enable register ICIER for RIIC communication
 *
 ****************************************************************************/

static void rx65n_riic_set_icier(FAR struct rx65n_i2c_priv_s *priv, \
                                             uint8_t value)
{
  uint8_t regval;
  regval = (value | RX65N_RIIC_ICIER_TMO);

  if (0 == priv->bus)
    {
      rx65n_putreg(regval, RX65N_RIIC0_ICIER);
      regval = rx65n_getreg(RX65N_RIIC0_ICIER);
    }

  else if (1 == priv->bus)
    {
      rx65n_putreg(regval, RX65N_RIIC1_ICIER);
      regval = rx65n_getreg(RX65N_RIIC1_ICIER);
    }

  else
    {
      rx65n_putreg(regval, RX65N_RIIC2_ICIER);
      regval = rx65n_getreg(RX65N_RIIC2_ICIER);
    }
}

/****************************************************************************
 * Name: rx65n_riic_setclock
 *
 * Description:
 *   Sets the I2C bus clock frequency – frequency for the transfer
 *
 ****************************************************************************/

static void rx65n_riic_setclock(FAR struct rx65n_i2c_priv_s *priv,
                                 uint32_t frequency)
{
  /* Divider array of RIIC clock  */

  uint8_t regval;
  const uint8_t d_cks[RIIC_MAX_DIV] =
    { 1, 2, 4, 8, 16, 32, 64, 128
    };

  volatile double l_time;     /* L Width period */
  volatile double h_time;     /* H Width period */
  volatile double calc_val;   /* Using for Calculation */
  double calc_val_tmp;
  volatile uint8_t cnt;

  /* Calculation of L width time */

  l_time = (1.0 / (2.0 * frequency)); /* Half period of frequency */
  h_time = l_time;

  /* Check I2C mode of Speed */

  if (frequency > I2C_SPEED_FAST)
    {
      i2cinfo("Fast Plus Mode Selected - Transmission Rate: 1 Mbps\n");
      if (l_time < 0.5E-6)
        {
          /* Wnen L width less than 0.5us, subtract Rise up and down
           * time for SCL from H/L width
           */

          l_time = 0.5E-6;
          h_time = (((1.0 / frequency) - l_time) - SCL_RISE_TIME_FASTPLUS) \
                 - SCL_FALL_TIME_FASTPLUS;
        }

      else
        {
          /* Subtract Rise up and down time for SCL from H/L width */

          l_time -= SCL_FALL_TIME_FASTPLUS;
          h_time -= SCL_RISE_TIME_FASTPLUS;
        }
    }

  else if (frequency > I2C_SPEED_STANDARD)
    {
      i2cinfo("Fast Mode Selected - Transmission Rate: 400 kbps\n");
      if (l_time < 1.3E-6)
        {
          /* Wnen L width less than 1.3us, subtract Rise up and down
           * time for SCL from H/L width
           */

          l_time = 1.3E-6;
          h_time = (((1.0 / frequency) - l_time) - SCL_RISE_TIME_FAST) \
                    - SCL_FALL_TIME_FAST;
        }

      else
        {
          /* Subtract Rise up and down time for SCL from H/L width */

          l_time -= SCL_FALL_TIME_FAST;
          h_time -= SCL_RISE_TIME_FAST;
        }
    }

  else
    {
      i2cinfo("Standard Mode Selected - Transmission Rate: 100 kbps\n");

      /* Subtract Rise up and down time for SCL from H/L width */

      l_time -= SCL_FALL_TIME_STANDARD;
      h_time -= SCL_RISE_TIME_STANDARD;
    }

  /* Calculate ICBRL value */

  for (calc_val = RIIC_RATE_CALC, cnt = 0; RIIC_CALC_MAX < calc_val; cnt++)
    {
      calc_val = l_time; /* Set L width time */

      /* Check the range of divider of CKS */

      if (cnt >= RIIC_MAX_DIV)
        {
          /* Cannot set bps */
        }

      calc_val_tmp = calc_val;

      /* Calculation of ICBRL value */

      calc_val = (calc_val_tmp / (d_cks[cnt] / RX_PCLKB));
      calc_val = calc_val + 0.5; /* round off */
    }

  if (priv->bus == 0)
    {
      /* Set ICMR1.CKS bits. */

      regval = rx65n_getreg(RX65N_RIIC0_ICMR1);
      regval &= RX65N_RIIC_ICMR1_CKS_MASK;
      regval |= (uint8_t) ((cnt - 1) << 4);
      rx65n_putreg(regval, RX65N_RIIC0_ICMR1);

      /* Set value to ICBRL register */

      regval = (uint8_t) ((uint8_t)(calc_val - 1) | RX65N_RIIC_ICBRL_MASK);
      rx65n_putreg(regval, RX65N_RIIC0_ICBRL);
    }

  else if (priv->bus == 1)
    {
      /* Set ICMR1.CKS bits. */

      regval = rx65n_getreg(RX65N_RIIC1_ICMR1);
      regval &= RX65N_RIIC_ICMR1_CKS_MASK;
      regval |= (uint8_t) ((cnt - 1) << 4);
      rx65n_putreg(regval, RX65N_RIIC1_ICMR1);

      /* Set value to ICBRL register */

      regval = (uint8_t) ((uint8_t)(calc_val - 1) | RX65N_RIIC_ICBRL_MASK);
      rx65n_putreg(regval, RX65N_RIIC1_ICBRL);
    }

  else
    {
      /* Set ICMR1.CKS bits. */

      regval = rx65n_getreg(RX65N_RIIC2_ICMR1);
      regval &= RX65N_RIIC_ICMR1_CKS_MASK;
      regval |= (uint8_t) ((cnt - 1) << 4);
      rx65n_putreg(regval, RX65N_RIIC2_ICMR1);

      /* Set value to ICBRL register */

      regval = (uint8_t) ((uint8_t)(calc_val - 1) | RX65N_RIIC_ICBRL_MASK);
      rx65n_putreg(regval, RX65N_RIIC2_ICBRL);
    }

  /* Calculate ICBRH value */

  calc_val = h_time;
  calc_val_tmp = calc_val;

  /* Calculation ICBRH value */

  calc_val = (calc_val_tmp / (d_cks[cnt - 1] / RX_PCLKB));
  calc_val = (uint8_t) (calc_val + 0.5); /* Round off */

  /* If the calculated value is less than 1, it rounded up to 1. */

  if (1 > calc_val)
    {
      calc_val = 1;
    }

  if (priv->bus == 0)
    {
      /* Set value to ICBRH register */

      regval = (uint8_t) ((uint8_t) (calc_val - 1) | RX65N_RIIC_ICBRH_MASK);
      rx65n_putreg(regval, RX65N_RIIC0_ICBRH);
    }

  else if (priv->bus == 1)
    {
      /* Set value to ICBRH register */

      regval = (uint8_t) ((uint8_t) (calc_val - 1) | RX65N_RIIC_ICBRH_MASK);
      rx65n_putreg(regval, RX65N_RIIC1_ICBRH);
    }

  else
    {
      /* Set value to ICBRH register */

      regval = (uint8_t) ((uint8_t) (calc_val - 1) | RX65N_RIIC_ICBRH_MASK);
      rx65n_putreg(regval, RX65N_RIIC2_ICBRH);
    }
}

/****************************************************************************
 * Name: rx65n_riic_int_enable
 *
 * Description:
 *   Enable Interrupts
 *
 ****************************************************************************/

static void rx65n_riic_int_enable(FAR struct rx65n_i2c_priv_s *priv)
{
  up_enable_irq(priv->dev->txi_irq);
  up_enable_irq(priv->dev->rxi_irq);
  up_enable_irq(priv->dev->tei_irq);
  up_enable_irq(priv->dev->eei_irq);
}

/****************************************************************************
 * Name: rx65n_riic_int_disable
 *
 * Description:
 *   Disable Interrupts
 *
 ****************************************************************************/

static void rx65n_riic_int_disable(FAR struct rx65n_i2c_priv_s *priv)
{
  up_disable_irq(priv->dev->txi_irq);
  up_disable_irq(priv->dev->rxi_irq);
  up_disable_irq(priv->dev->tei_irq);
  up_disable_irq(priv->dev->eei_irq);
}

/****************************************************************************
 * Name: rx65n_riic_irq_init
 *
 * Description:
 *   Setup the initial conditions of I2C interrupt handling
 *
 ****************************************************************************/

static int rx65n_riic_irq_init(FAR struct rx65n_i2c_priv_s *priv)
{
  int ret;

  IEN(ICU, GROUPBL1) = 0;     /* Disable Group BL1 interrupts */
  IR(ICU, GROUPBL1)  = 0;     /* Clear interrupt flag */
  IPR(ICU, GROUPBL1) = RIIC_INTERRUPT_PRIO;
  IEN(ICU, GROUPBL1) = 1;     /* Enable Group BL1 interrupt */

  if (priv->bus == 0)
    {
      IR(RIIC0, RXI0)  = 0;
      IPR(RIIC0, RXI0) = RIIC_INTERRUPT_PRIO;

      IR(RIIC0, TXI0)  = 0;
      IPR(RIIC0, TXI0) = RIIC_INTERRUPT_PRIO;

      ret = irq_attach(priv->dev->rxi_irq, rx65n_riic_rxi0interrupt, priv);
      ret = irq_attach(priv->dev->txi_irq, rx65n_riic_txi0interrupt, priv);
      ret = irq_attach(priv->dev->tei_irq, rx65n_riic_tei0interrupt, priv);
      ret = irq_attach(priv->dev->eei_irq, rx65n_riic_eei0interrupt, priv);
    }

  else if (priv->bus == 1)
    {
      IR(RIIC1, RXI1)  = 0;
      IPR(RIIC1, RXI1) = RIIC_INTERRUPT_PRIO;

      IR(RIIC1, TXI1)  = 0;
      IPR(RIIC1, TXI1) = RIIC_INTERRUPT_PRIO;

      ret = irq_attach(priv->dev->rxi_irq, rx65n_riic_rxi1interrupt, priv);
      ret = irq_attach(priv->dev->txi_irq, rx65n_riic_txi1interrupt, priv);
      ret = irq_attach(priv->dev->tei_irq, rx65n_riic_tei1interrupt, priv);
      ret = irq_attach(priv->dev->eei_irq, rx65n_riic_eei1interrupt, priv);
    }

  else
    {
      IR(RIIC2, RXI2)  = 0;
      IPR(RIIC2, RXI2) = RIIC_INTERRUPT_PRIO;

      IR(RIIC2, TXI2)  = 0;
      IPR(RIIC2, TXI2) = RIIC_INTERRUPT_PRIO;

      ret = irq_attach(priv->dev->rxi_irq, rx65n_riic_rxi2interrupt, priv);
      ret = irq_attach(priv->dev->txi_irq, rx65n_riic_txi2interrupt, priv);
      ret = irq_attach(priv->dev->tei_irq, rx65n_riic_tei2interrupt, priv);
      ret = irq_attach(priv->dev->eei_irq, rx65n_riic_eei2interrupt, priv);
    }

  rx65n_riic_int_enable(priv);

  return ret;
}

/****************************************************************************
 * Name: rx65n_riic_init
 *
 * Description:
 *   Setup the initial conditions of I2C hardware and be ready for operation
 *
 ****************************************************************************/

static void rx65n_riic_init(FAR struct rx65n_i2c_priv_s *priv)
{
  uint8_t regval;

  /* Disable RXI, TXI,TEI and EEI interrupts */

  rx65n_riic_int_disable(priv);

  /* RIIC registers initialization */

  if (priv->bus == 0)
    {
      /* Release from the module-stop state */

      MSTP_RIIC0 = 0;

      /* SCLn and SDAn pins not driven - RIIC disabled */

      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);
      regval &= RX65N_RIIC_ICCR1_ICE_RST;
      rx65n_putreg(regval, RX65N_RIIC0_ICCR1);

      /* RIIC Reset */

      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);
      regval |= RX65N_RIIC_ICCR1_IICRST_SET;
      rx65n_putreg(regval, RX65N_RIIC0_ICCR1);

      /* Internal reset, SCLn and SDAn pins in active state */

      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);
      regval |= ~(RX65N_RIIC_ICCR1_ICE_RST);
      rx65n_putreg(regval, RX65N_RIIC0_ICCR1);

      /* Set SARLy and SARUy */

      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC0_SARL0);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC0_SARU0);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC0_SARL1);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC0_SARU1);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC0_SARL2);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC0_SARU2);

      /* Set I2C bus frequency */

      rx65n_riic_setclock(priv, priv->dev->frequency);

      regval = rx65n_getreg(RX65N_RIIC0_ICSER);
      regval &= RX65N_RIIC_ICSER_SET;
      rx65n_putreg(regval, RX65N_RIIC0_ICSER);

#ifndef CONFIG_RX65N_RIIC0_NF
      RIIC0.ICFER.BIT.NFE = 0;
#else
      regval = rx65n_getreg(RX65N_RIIC0_ICMR3);
      switch (CONFIG_RX65N_RIIC0_NF_STAGE)
        {
          case 1:
                  regval |= RX65N_RIIC_ICMR3_NF1;
               break;

          case 2:
                  regval |= RX65N_RIIC_ICMR3_NF2;
               break;

          case 3:
                  regval |= RX65N_RIIC_ICMR3_NF3;
               break;

          case 4:
               regval |= RX65N_RIIC_ICMR3_NF4;
            break;
        }

      rx65n_putreg(regval, RX65N_RIIC0_ICMR3);
#endif

      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC0_ICIER);

      regval = rx65n_getreg(RX65N_RIIC0_ICFER);
      regval |= RX65N_RIIC_ICFER_TMOE_SET;
      rx65n_putreg(regval, RX65N_RIIC0_ICFER);

#ifdef CONFIG_RX65N_RIIC0_SDA_DELAY
      RIIC0.ICMR2.BIT.DLCS = 1;
      regval = rx65n_getreg(RX65N_RIIC0_ICMR2);
      switch (CONFIG_RX65N_RIIC0_DELAY_CNT)
        {
          case 0:
            regval |= RX65N_RIIC_ICMR2_SDDL0;
            break;

          case 1:
            regval |= RX65N_RIIC_ICMR2_SDDL1;
            break;

          case 2:
            regval |= RX65N_RIIC_ICMR2_SDDL2;
            break;

          case 3:
            regval |= RX65N_RIIC_ICMR2_SDDL3;
            break;

          case 4:
            regval |= RX65N_RIIC_ICMR2_SDDL4;
            break;

          case 5:
            regval |= RX65N_RIIC_ICMR2_SDDL5;
            break;

          case 6:
            regval |= RX65N_RIIC_ICMR2_SDDL6;
            break;

          case 7:
            regval |= RX65N_RIIC_ICMR2_SDDL7;
            break;
        }

      rx65n_putreg(regval, RX65N_RIIC0_ICMR2);
#endif

      rx65n_riic_irq_init(priv);

      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);
      regval &= ~(RX65N_RIIC_ICCR1_IICRST_SET);
      rx65n_putreg(regval, RX65N_RIIC0_ICCR1);
    }

  else if (priv->bus == 1)
    {
      /* Release from the module-stop state */

      MSTP_RIIC1 = 0;

      /* SCLn and SDAn pins not driven - RIIC disabled */

      regval = rx65n_getreg(RX65N_RIIC1_ICCR1);
      regval &= RX65N_RIIC_ICCR1_ICE_RST;
      rx65n_putreg(regval, RX65N_RIIC1_ICCR1);

      /* RIIC Reset */

      regval = rx65n_getreg(RX65N_RIIC1_ICCR1);
      regval |= RX65N_RIIC_ICCR1_IICRST_SET;
      rx65n_putreg(regval, RX65N_RIIC1_ICCR1);

      /* Internal reset, SCLn and SDAn pins in active state */

      regval = rx65n_getreg(RX65N_RIIC1_ICCR1);
      regval |= ~(RX65N_RIIC_ICCR1_ICE_RST);
      rx65n_putreg(regval, RX65N_RIIC1_ICCR1);

      /* Set SARLy and SARUy */

      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC1_SARL0);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC1_SARU0);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC1_SARL1);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC1_SARU1);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC1_SARL2);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC1_SARU2);

      /* Set I2C bus frequency */

      rx65n_riic_setclock(priv, priv->dev->frequency);

      regval = rx65n_getreg(RX65N_RIIC1_ICSER);
      regval &= RX65N_RIIC_ICSER_SET;
      rx65n_putreg(regval, RX65N_RIIC1_ICSER);

#ifndef CONFIG_RX65N_RIIC1_NF
      RIIC1.ICFER.BIT.NFE = 0;
#else
      regval = rx65n_getreg(RX65N_RIIC1_ICMR3);
      switch (CONFIG_RX65N_RIIC1_NF_STAGE)
        {
          case 1:
                  regval |= RX65N_RIIC_ICMR3_NF1;
               break;

          case 2:
                  regval |= RX65N_RIIC_ICMR3_NF2;
               break;

          case 3:
                  regval |= RX65N_RIIC_ICMR3_NF3;
               break;

          case 4:
               regval |= RX65N_RIIC_ICMR3_NF4;
            break;
        }

      rx65n_putreg(regval, RX65N_RIIC1_ICMR3);
#endif

      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC1_ICIER);

      regval = rx65n_getreg(RX65N_RIIC1_ICFER);
      regval |= RX65N_RIIC_ICFER_TMOE_SET;
      rx65n_putreg(regval, RX65N_RIIC1_ICFER);

#ifdef CONFIG_RX65N_RIIC1_SDA_DELAY
      RIIC1.ICMR2.BIT.DLCS = 1;
      regval = rx65n_getreg(RX65N_RIIC1_ICMR2);
      switch (CONFIG_RX65N_RIIC1_DELAY_CNT)
        {
          case 0:
            regval |= RX65N_RIIC_ICMR2_SDDL0;
            break;

          case 1:
            regval |= RX65N_RIIC_ICMR2_SDDL1;
            break;

          case 2:
            regval |= RX65N_RIIC_ICMR2_SDDL2;
            break;

          case 3:
            regval |= RX65N_RIIC_ICMR2_SDDL3;
            break;

          case 4:
            regval |= RX65N_RIIC_ICMR2_SDDL4;
            break;

          case 5:
            regval |= RX65N_RIIC_ICMR2_SDDL5;
            break;

          case 6:
            regval |= RX65N_RIIC_ICMR2_SDDL6;
            break;

          case 7:
            regval |= RX65N_RIIC_ICMR2_SDDL7;
            break;
        }

      rx65n_putreg(regval, RX65N_RIIC1_ICMR2);
#endif

      rx65n_riic_irq_init(priv);

      regval = rx65n_getreg(RX65N_RIIC1_ICCR1);
      regval &= ~(RX65N_RIIC_ICCR1_IICRST_SET);
      rx65n_putreg(regval, RX65N_RIIC1_ICCR1);
    }

  else
    {
      /* Release from the module-stop state */

      MSTP_RIIC2 = 0;

      /* SCLn and SDAn pins not driven - RIIC disabled */

      regval = rx65n_getreg(RX65N_RIIC2_ICCR1);
      regval &= RX65N_RIIC_ICCR1_ICE_RST;
      rx65n_putreg(regval, RX65N_RIIC2_ICCR1);

      /* RIIC Reset */

      regval = rx65n_getreg(RX65N_RIIC2_ICCR1);
      regval |= RX65N_RIIC_ICCR1_IICRST_SET;
      rx65n_putreg(regval, RX65N_RIIC2_ICCR1);

      /* Internal reset, SCLn and SDAn pins in active state */

      regval = rx65n_getreg(RX65N_RIIC2_ICCR1);
      regval |= ~(RX65N_RIIC_ICCR1_ICE_RST);
      rx65n_putreg(regval, RX65N_RIIC2_ICCR1);

      /* Set SARLy and SARUy */

      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC2_SARL0);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC2_SARU0);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC2_SARL1);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC2_SARU1);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC2_SARL2);
      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC2_SARU2);

      /* Set I2C bus frequency */

      rx65n_riic_setclock(priv, priv->dev->frequency);

      regval = rx65n_getreg(RX65N_RIIC2_ICSER);
      regval &= RX65N_RIIC_ICSER_SET;
      rx65n_putreg(regval, RX65N_RIIC2_ICSER);

#ifndef CONFIG_RX65N_RIIC2_NF
      RIIC2.ICFER.BIT.NFE = 0;
#else
      regval = rx65n_getreg(RX65N_RIIC2_ICMR3);
      switch (CONFIG_RX65N_RIIC2_NF_STAGE)
        {
          case 1:
                  regval |= RX65N_RIIC_ICMR3_NF1;
               break;

          case 2:
                  regval |= RX65N_RIIC_ICMR3_NF2;
               break;

          case 3:
                  regval |= RX65N_RIIC_ICMR3_NF3;
               break;

          case 4:
                  regval |= RX65N_RIIC_ICMR3_NF4;
               break;
           }

      rx65n_putreg(regval, RX65N_RIIC2_ICMR3);
#endif

      rx65n_putreg(RIIC_REG_INIT, RX65N_RIIC2_ICIER);

      regval = rx65n_getreg(RX65N_RIIC2_ICFER);
      regval |= RX65N_RIIC_ICFER_TMOE_SET;
      rx65n_putreg(regval, RX65N_RIIC2_ICFER);

#ifdef CONFIG_RX65N_RIIC2_SDA_DELAY
      RIIC2.ICMR2.BIT.DLCS = 1;
      regval = rx65n_getreg(RX65N_RIIC2_ICMR2);
      switch (CONFIG_RX65N_RIIC2_DELAY_CNT)
        {
          case 0:
            regval |= RX65N_RIIC_ICMR2_SDDL0;
            break;

          case 1:
            regval |= RX65N_RIIC_ICMR2_SDDL1;
            break;

          case 2:
            regval |= RX65N_RIIC_ICMR2_SDDL2;
            break;

          case 3:
            regval |= RX65N_RIIC_ICMR2_SDDL3;
            break;

          case 4:
            regval |= RX65N_RIIC_ICMR2_SDDL4;
            break;

          case 5:
            regval |= RX65N_RIIC_ICMR2_SDDL5;
            break;

          case 6:
            regval |= RX65N_RIIC_ICMR2_SDDL6;
            break;

          case 7:
            regval |= RX65N_RIIC_ICMR2_SDDL7;
            break;
        }

      rx65n_putreg(regval, RX65N_RIIC2_ICMR2);
#endif

      rx65n_riic_irq_init(priv);

      regval = rx65n_getreg(RX65N_RIIC2_ICCR1);
      regval &= ~(RX65N_RIIC_ICCR1_IICRST_SET);
      rx65n_putreg(regval, RX65N_RIIC2_ICCR1);
    }

  priv->mode = RIIC_READY;
  priv->dev_sts = RIIC_STS_IDLE;

  /* RIIC registers initialized */
}

/****************************************************************************
 * Function Name: rx65n_riic_set_sending_data
 * Description  : Transmit Data Processing.
 *                Sets transmission data to ICDRT register.
 * Arguments    : rx65n_i2c_priv_s *priv      ; IIC Information
 * Return Value : None
 ****************************************************************************/

static void rx65n_riic_set_sending_data (FAR struct rx65n_i2c_priv_s *priv, \
                                                  uint8_t data)
{
  if (0 == priv->bus)
    {
      /* Clears TXI interrupt request register. */

      IR(RIIC0, TXI0) = 0;

      /* Sets the transmitting data. */

      rx65n_putreg(data, RX65N_RIIC0_ICDRT);
      data = rx65n_getreg(RX65N_RIIC0_ICDRT);
    }

  else if (1 == priv->bus)
    {
      /* Clears TXI interrupt request register. */

      IR(RIIC1, TXI1) = 0;

      /* Sets the transmitting data. */

      rx65n_putreg(data, RX65N_RIIC1_ICDRT);
      data = rx65n_getreg(RX65N_RIIC1_ICDRT);
    }

  else
    {
      /* Clears TXI interrupt request register. */

      IR(RIIC2, TXI2) = 0;

      /* Sets the transmitting data. */

      rx65n_putreg(data, RX65N_RIIC2_ICDRT);
      data = rx65n_getreg(RX65N_RIIC2_ICDRT);
    }
}

/****************************************************************************
 * Name: rx65n_riic_advance
 *
 * Description:
 *  Check if RIIC Bus is busy
 *
 ****************************************************************************/

static void rx65n_riic_advance(FAR struct rx65n_i2c_priv_s *priv)
{
  int ret;

  if ((RIIC_EV_INT_TMO == priv->event)  || (RIIC_STS_TMO == priv->dev_sts))
    {
      i2cerr("RIIC Transfer Timed Out\n");
      rx65n_riic_timeout(priv);
    }

  if (RIIC_EV_INT_AL == priv->event)
    {
      rx65n_riic_iicrst(priv);
    }

  if (RIIC_EV_INT_START == priv->event)
    {
      rx65n_riic_send_slv_addr(priv);
    }

  if (RIIC_EV_INT_ADD == priv->event)
    {
      rx65n_riic_after_send_slvadr(priv);
    }

  if (RIIC_EV_INT_SEND == priv->event)
    {
      rx65n_riic_master_transmit(priv);
    }

  if (RIIC_EV_INT_RECEIVE == priv->event)
    {
      if (RIIC_STS_RECEIVE_DATA_WAIT == priv->dev_sts)
        {
          rx65n_riic_master_receive(priv);
        }

      else
        {
          rx65n_riic_after_send_slvadr(priv);
        }
    }

  if (RIIC_EV_INT_NACK == priv->event)
    {
      rx65n_riic_nack(priv);
    }

  if (RIIC_EV_INT_STOP == priv->event)
    {
      ret = rx65n_riic_after_stop(priv);
      if (RIIC_SUCCESS == ret)
        {
          priv->mode = RIIC_FINISH;
        }
    }
}

/****************************************************************************
 * Name: rx65n_riic_nack
 *
 * Description:
 *  NACK reception handler
 *
 ****************************************************************************/

static void rx65n_riic_nack(FAR struct rx65n_i2c_priv_s *priv)
{
  rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_SP_AL);

  priv->dev_sts = RIIC_STS_SP_COND_WAIT;
  rx65n_riic_stopcond(priv);

  if (RIIC_M_RECEIVE == priv->mode)
    {
      rx65n_riic_read_data(priv);
    }
}

/****************************************************************************
 * Name: rx65n_riic_timeout
 *
 * Description:
 *  After Timeout condition processing
 *
 ****************************************************************************/

static void rx65n_riic_timeout(FAR struct rx65n_i2c_priv_s *priv)
{
  priv->mode = RIIC_FINISH;
}

/****************************************************************************
 * Name: rx65n_riic_after_stop
 *
 * Description:
 *  After stop condition processing
 *
 ****************************************************************************/

static int rx65n_riic_after_stop(FAR struct rx65n_i2c_priv_s *priv)
{
  int ret;
  bool bus;

  bus = rx65n_riic_check_bus_busy(priv);

  if (bus == RIIC_BUS_BUSY)
    {
      ret = RIIC_ERR_BUS_BUSY;
    }

  else
    {
      if (RIIC_EV_INT_TMO == priv->event)
        {
          ret = RIIC_ERR_TMO;
        }

      else
        {
          ret = RIIC_SUCCESS;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rx65n_riic_check_bus_busy
 *
 * Description:
 *  Check if RIIC Bus is busy
 *
 ****************************************************************************/

static bool rx65n_riic_check_bus_busy(FAR struct rx65n_i2c_priv_s *priv)
{
  int i;
  bool bus_state = RIIC_BUS_BUSY;

  if (priv->bus == 0)
    {
      for (i = BUS_CHECK_COUNTER; i >= 0; i--)
        {
          if ((0U == RIIC0.ICCR2.BIT.BBSY) && (1U == RIIC0.ICCR1.BIT.SCLI) &&
             (1U == RIIC0.ICCR1.BIT.SDAI))
            {
              bus_state = RIIC_BUS_FREE;
              break;
            }
        }
    }

  if (priv->bus == 1)
    {
      for (i = BUS_CHECK_COUNTER; i >= 0; i--)
        {
          if ((0U == RIIC1.ICCR2.BIT.BBSY) && (1U == RIIC1.ICCR1.BIT.SCLI) &&
             (1U == RIIC1.ICCR1.BIT.SDAI))
            {
              bus_state = RIIC_BUS_FREE;
              break;
            }
        }
    }

  else
    {
      for (i = BUS_CHECK_COUNTER; i >= 0; i--)
        {
          if ((0U == RIIC2.ICCR2.BIT.BBSY) && (1U == RIIC2.ICCR1.BIT.SCLI) &&
             (1U == RIIC2.ICCR1.BIT.SDAI))
            {
              bus_state = RIIC_BUS_FREE;
              break;
            }
        }
    }

  return bus_state;
}

/****************************************************************************
 * Name: rx65n_riic_startcond
 *
 * Description:
 *  Issue the start condition for Master transmission/reception
 *
 ****************************************************************************/

static int rx65n_riic_startcond(FAR struct rx65n_i2c_priv_s *priv)
{
  uint8_t regval;
  int ret = RIIC_SUCCESS;
  bool bus;

  if (priv->bus == 0)
    {
      bus = rx65n_riic_check_bus_busy(priv);

      if (bus == RIIC_BUS_FREE)
        {
          /* Clear stop */

          regval = rx65n_getreg(RX65N_RIIC0_ICSR2);
          regval &= ~(RX65N_RIIC_ICSR2_STOP_SET);
          rx65n_putreg(regval, RX65N_RIIC0_ICSR2);

          regval = rx65n_getreg(RX65N_RIIC0_ICSR2);
          while (0x00 != ((regval & RX65N_RIIC_ICSR2_START_SET) || \
            (regval & RX65N_RIIC_ICSR2_STOP_SET)))
            {
              regval = rx65n_getreg(RX65N_RIIC0_ICSR2);
            }

          /* Set the internal status */

          priv->dev_sts = RIIC_STS_ST_COND_WAIT;

          /* Clear ALIE bit */

          regval = rx65n_getreg(RX65N_RIIC0_ICIER);
          regval &= ~(RX65N_RIIC_ICIER_ALIE);
          rx65n_putreg(regval, RX65N_RIIC0_ICIER);

          /* Interrupt setting */

          rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_ST_NAK_AL);

          /* Generate the start condition */

          /* Clear the start condition detection flag */

          regval = rx65n_getreg(RX65N_RIIC0_ICSR2);
          if (0x00 != ((regval & RX65N_RIIC_ICSR2_START_SET) >> 2U))
            {
              /* Clears start condition detection flag. */

              regval &= ~(RX65N_RIIC_ICSR2_START_SET);
              rx65n_putreg(regval, RX65N_RIIC0_ICSR2);
              regval = rx65n_getreg(RX65N_RIIC0_ICSR2);
            }

          /* Generate start condition */

          regval = rx65n_getreg(RX65N_RIIC0_ICCR2);
          regval |= RX65N_RIIC_ICCR2_ST_SET;
          rx65n_putreg(regval, RX65N_RIIC0_ICCR2);
          regval = rx65n_getreg(RX65N_RIIC0_ICCR2);
        }

      else
        {
          ret = RIIC_ERR_BUS_BUSY;
        }
    }

  else if (priv->bus == 1)
    {
      bus = rx65n_riic_check_bus_busy(priv);

      if (bus == RIIC_BUS_FREE)
        {
          /* Clear stop */

          regval = rx65n_getreg(RX65N_RIIC1_ICSR2);
          regval &= ~(RX65N_RIIC_ICSR2_STOP_SET);
          rx65n_putreg(regval, RX65N_RIIC1_ICSR2);

          regval = rx65n_getreg(RX65N_RIIC1_ICSR2);
          while (0x00 != ((regval & RX65N_RIIC_ICSR2_START_SET) || \
               (regval & RX65N_RIIC_ICSR2_STOP_SET)))
            {
              regval = rx65n_getreg(RX65N_RIIC1_ICSR2);
            }

          priv->dev_sts = RIIC_STS_ST_COND_WAIT;

          regval = rx65n_getreg(RX65N_RIIC1_ICIER);
          regval &= ~(RX65N_RIIC_ICIER_ALIE);
          rx65n_putreg(regval, RX65N_RIIC1_ICIER);

          rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_ST_NAK_AL);

          regval = rx65n_getreg(RX65N_RIIC1_ICSR2);
          if (0x00 != ((regval & RX65N_RIIC_ICSR2_START_SET) >> 2U))
            {
              /* Clears start condition detection flag. */

              regval &= ~(RX65N_RIIC_ICSR2_START_SET);
              rx65n_putreg(regval, RX65N_RIIC1_ICSR2);
            }

          /* Generate start condition */

          regval = rx65n_getreg(RX65N_RIIC1_ICCR2);
          regval |= RX65N_RIIC_ICCR2_ST_SET;
          rx65n_putreg(regval, RX65N_RIIC1_ICCR2);
          regval = rx65n_getreg(RX65N_RIIC1_ICCR2);
        }

      else
        {
          ret = RIIC_ERR_BUS_BUSY;
        }
    }

  else
    {
      bus = rx65n_riic_check_bus_busy(priv);

      if (bus == RIIC_BUS_FREE)
        {
          /* Clear stop */

          regval = rx65n_getreg(RX65N_RIIC2_ICSR2);
          regval &= ~(RX65N_RIIC_ICSR2_STOP_SET);
          rx65n_putreg(regval, RX65N_RIIC2_ICSR2);

          regval = rx65n_getreg(RX65N_RIIC2_ICSR2);
          while (0x00 != ((regval & RX65N_RIIC_ICSR2_START_SET) || \
               (regval & RX65N_RIIC_ICSR2_STOP_SET)))
            {
              regval = rx65n_getreg(RX65N_RIIC2_ICSR2);
            }

          priv->dev_sts = RIIC_STS_ST_COND_WAIT;

          regval = rx65n_getreg(RX65N_RIIC2_ICIER);
          regval &= ~(RX65N_RIIC_ICIER_ALIE);
          rx65n_putreg(regval, RX65N_RIIC2_ICIER);

          /* Timeout Detection */

          rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_ST_NAK_AL);

          regval = rx65n_getreg(RX65N_RIIC2_ICSR2);
          if (0x00 != ((regval & RX65N_RIIC_ICSR2_START_SET) >> 2U))
            {
              /* Clears start condition detection flag. */

              regval &= ~(RX65N_RIIC_ICSR2_START_SET);
              rx65n_putreg(regval, RX65N_RIIC2_ICSR2);
            }

          /* Generate start condition */

          regval = rx65n_getreg(RX65N_RIIC2_ICCR2);
          regval |= RX65N_RIIC_ICCR2_ST_SET;
          rx65n_putreg(regval, RX65N_RIIC2_ICCR2);
          regval = rx65n_getreg(RX65N_RIIC2_ICCR2);
        }

      else
        {
          ret = RIIC_ERR_BUS_BUSY;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rx65n_riic_restartcond
 *
 * Description:
 *  Issue Re-start condition for Master Transmission/Reception
 *
 ****************************************************************************/

static void rx65n_riic_restartcond(FAR struct rx65n_i2c_priv_s *priv)
{
  uint8_t regval;

  if (priv->bus == 0)
    {
      /* Clears start condition detection flag. */

      regval = getreg8(RX65N_RIIC0_ICSR2);
      if (0x00 != ((regval & RX65N_RIIC_ICSR2_START_SET) >> 2U))
        {
          regval &= ~(RX65N_RIIC_ICSR2_START_SET);
          rx65n_putreg(regval, RX65N_RIIC0_ICSR2);
        }

      /* Generates a restart condition. */

      regval = getreg8(RX65N_RIIC0_ICCR2);
      regval |= RX65N_RIIC_ICCR2_RS_SET; /* Sets ICCR2.RS bit. */
      rx65n_putreg(regval, RX65N_RIIC0_ICCR2);
    }

  else if (priv->bus == 1)
    {
      /* Clears start condition detection flag. */

      regval = getreg8(RX65N_RIIC1_ICSR2);
      if (0x00 != ((regval & RX65N_RIIC_ICSR2_START_SET) >> 2U))
        {
          regval &= ~(RX65N_RIIC_ICSR2_START_SET);
          rx65n_putreg(regval, RX65N_RIIC1_ICSR2);
        }

      /* Generates a restart condition. */

      regval = getreg8(RX65N_RIIC1_ICCR2);
      regval |= RX65N_RIIC_ICCR2_RS_SET; /* Sets ICCR2.RS bit. */
      rx65n_putreg(regval, RX65N_RIIC1_ICCR2);
    }

  else
    {
      /* Clears start condition detection flag. */

      regval = getreg8(RX65N_RIIC2_ICSR2);
      if (0x00 != ((regval & RX65N_RIIC_ICSR2_START_SET) >> 2U))
        {
          regval &= ~(RX65N_RIIC_ICSR2_START_SET);
          rx65n_putreg(regval, RX65N_RIIC2_ICSR2);
        }

      /* Generates a restart condition. */

      regval = getreg8(RX65N_RIIC2_ICCR2);
      regval |= RX65N_RIIC_ICCR2_RS_SET; /* Sets ICCR2.RS bit. */
      rx65n_putreg(regval, RX65N_RIIC2_ICCR2);
    }
}

/****************************************************************************
 * Name: rx65n_riic_send_slv_addr
 *
 * Description:
 *  Transmit slave address for communication
 *
 ****************************************************************************/

static void rx65n_riic_send_slv_addr(FAR struct rx65n_i2c_priv_s *priv)
{
  uint16_t bit_addr;
  uint8_t regval;

  if (priv->msgv->flags == I2C_M_TEN)
    {
      /* 10 bit slave address handling */

      bit_addr = priv->msgv->addr;
      bit_addr &= RX65N_RIIC_10BIT_SARU_MASK;
      bit_addr >>= 8;
      regval = (uint8_t) bit_addr;

      rx65n_riic_set_sending_data(priv, regval);

      /* Check ACK/NACK reception */

      bit_addr = priv->msgv->addr;
      bit_addr &= RX65N_RIIC_10BIT_SARL_MASK;
      regval = (uint8_t) bit_addr;
    }

  else
    {
      if (priv->flags == I2C_M_READ)
        {
          priv->mode = RIIC_M_RECEIVE;
          rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_RX_NAK_AL);

          /* 7-bit slave address with READ code */

          regval = (priv->msgv->addr);
          regval <<= 1U;
          regval |= RX65N_RIIC_READ_MASK;

          priv->dev_sts = RIIC_STS_SEND_SLVADR_R_WAIT;
        }

      else
        {
          priv->mode = RIIC_M_TRANSMIT;
          rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_TEND_NAK_AL);

          /* 7-bit slave address with WRITE code */

          regval = (priv->msgv->addr);
          regval <<= 1U;
          regval &= ~(RX65N_RIIC_READ_MASK);

          priv->dev_sts = RIIC_STS_SEND_SLVADR_W_WAIT;
        }
    }

  /* Send slave address */

  rx65n_riic_set_sending_data(priv, regval);
}

/****************************************************************************
 * Name: rx65n_riic_after_send_slvadr
 *
 * Description:
 *  Processing after sending slave address
 *
 ****************************************************************************/

static void rx65n_riic_after_send_slvadr(FAR struct rx65n_i2c_priv_s *priv)
{
  uint8_t regval;
  uint8_t *data;

  if (RIIC_M_TRANSMIT == priv->mode)
    {
      /* Pattern write 1: msg[0] is memory addressand msg[1] is data byte */

      if (2U >= priv->msgc)
        {
          priv->dev_sts = RIIC_STS_SEND_DATA_WAIT;
          rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_TEND_NAK_AL);

          /* Transmit first byte (memory address) */

          if (0U != priv->dcnt)
            {
              data = priv->ptr;
              regval = *data;
              rx65n_riic_set_sending_data(priv, regval);

              priv->dcnt--;
              priv->ptr++;
            }

          else
            {
              /* Do nothing */
            }
        }

      /* Pattern write 2: msg[0] is data byte */

      else if (1U >= priv->msgc)
        {
          priv->dev_sts = RIIC_STS_SEND_DATA_WAIT;
          rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_TEND_NAK_AL);

          /* Transmit data byte */

          if (0U != priv->dcnt)
            {
              data = priv->ptr;
              regval = *data;
              rx65n_riic_set_sending_data(priv, regval);

              priv->dcnt--;
              priv->ptr++;
            }

          else
            {
              /* Do nothing */
            }
        }

      /* Pattern write 3: transmit only slave address */

      else if (0 == priv->msgc)
        {
          priv->dev_sts = RIIC_STS_SP_COND_WAIT;
          rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_SP_NAK_AL);

          /* Generate stop condition */

          rx65n_riic_stopcond(priv);
        }

      else
        {
          /* Do nothing */
        }
    }

  if (RIIC_M_RECEIVE == priv->mode)
    {
      priv->dev_sts = RIIC_STS_RECEIVE_DATA_WAIT;

      rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_RX_NAK_AL);

      if (2 >= priv->dcnt)
        {
          rx65n_riic_wait_set(priv);
        }

      if (0x01 >= priv->dcnt)
        {
          rx65n_riic_pre_end_set(priv);
        }

      /* Dummy read */

      regval = rx65n_riic_read_data(priv);
    }
}

/****************************************************************************
 * Name: rx65n_riic_stopcond
 *
 * Description:
 *  Issue stop condition for Master Transmission/Reception
 *
 ****************************************************************************/

static void rx65n_riic_stopcond(FAR struct rx65n_i2c_priv_s *priv)
{
  uint8_t regval;

  if (priv->bus == 0)
    {
      regval = rx65n_getreg(RX65N_RIIC0_ICSR2);
      if (0x00 != ((regval & RX65N_RIIC_ICSR2_STOP_SET) >> 3U))
        {
          regval &= ~(RX65N_RIIC_ICSR2_STOP_SET);
          rx65n_putreg(regval, RX65N_RIIC0_ICSR2);
          regval = rx65n_getreg(RX65N_RIIC0_ICSR2);
        }

      /* Generates a stop condition. */

      regval = rx65n_getreg(RX65N_RIIC0_ICCR2);
      regval |= RX65N_RIIC_ICCR2_SP_SET; /* Sets ICCR2.SP bit. */
      rx65n_putreg(regval, RX65N_RIIC0_ICCR2);

      regval = rx65n_getreg(RX65N_RIIC0_ICCR2);
    }

  else if (priv->bus == 1)
    {
      /* Clears stop condition detection flag. */

      regval = rx65n_getreg(RX65N_RIIC1_ICSR2);
      if (0x00 != ((regval & RX65N_RIIC_ICSR2_STOP_SET) >> 3U))
        {
          regval &= ~(RX65N_RIIC_ICSR2_STOP_SET);
          rx65n_putreg(regval, RX65N_RIIC1_ICSR2);
        }

      /* Generates a stop condition. */

      regval = rx65n_getreg(RX65N_RIIC1_ICCR2);
      regval |= RX65N_RIIC_ICCR2_SP_SET; /* Sets ICCR2.SP bit. */
      rx65n_putreg(regval, RX65N_RIIC1_ICCR2);
    }

  else
    {
      /* Clears stop condition detection flag. */

      regval = rx65n_getreg(RX65N_RIIC2_ICSR2);
      if (0x00 != ((regval & RX65N_RIIC_ICSR2_STOP_SET) >> 3U))
        {
          regval &= ~(RX65N_RIIC_ICSR2_STOP_SET);
          rx65n_putreg(regval, RX65N_RIIC2_ICSR2);
        }

      /* Generates a stop condition. */

      regval = rx65n_getreg(RX65N_RIIC2_ICCR2);
      regval |= RX65N_RIIC_ICCR2_SP_SET; /* Sets ICCR2.SP bit. */
      rx65n_putreg(regval, RX65N_RIIC2_ICCR2);
    }
}

/****************************************************************************
 * Name: rx65n_riic_master_transmit
 *
 * Description:
 *  Transmission of data from Master to Slave
 *
 ****************************************************************************/

static void rx65n_riic_master_transmit(FAR struct rx65n_i2c_priv_s *priv)
{
  uint8_t regval;
  uint8_t *data;

  priv->dev_sts = RIIC_STS_SEND_DATA_WAIT;

  if (0U == priv->dcnt)
    {
      /* Moving to the next message */

      priv->msgc--;

      if (0x00 == priv->msgc)
        {
          rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_SP_NAK_AL);

          priv->dev_sts = RIIC_STS_SP_COND_WAIT;
          rx65n_riic_stopcond(priv);
        }

      else
        {
          priv->msgv++;
          priv->ptr = priv->msgv->buffer;
          priv->flags = priv->msgv->flags;
          priv->dcnt = priv->msgv->length;

          if (priv->flags == I2C_M_READ)
            {
              priv->mode = RIIC_M_RECEIVE;
              priv->event = RIIC_EV_GEN_START_COND;

              rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_ST_NAK_AL);

              rx65n_riic_restartcond(priv);
            }
        }
    }

  if (0U != priv->dcnt && priv->flags != I2C_M_READ)
    {
      rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_TEND_NAK_AL);

      data = priv->ptr;
      regval = *data;

      /* send the byte */

      rx65n_riic_set_sending_data(priv, regval);

      /* decrement the count */

      priv->dcnt--;
      priv->ptr++;
    }
}

/****************************************************************************
 * Name: rx65n_riic_read_data
 *
 * Description:
 *  Read received data from ICDRR
 *
 ****************************************************************************/

static uint8_t rx65n_riic_read_data(FAR struct rx65n_i2c_priv_s *priv)
{
  volatile uint8_t * regval;

  if (0 == priv->bus)
    {
      regval = (uint8_t *) (RX65N_RIIC0_ICDRR);
    }

  else if (1 == priv->bus)
    {
      regval = (uint8_t *) (RX65N_RIIC1_ICDRR);
    }

  else
    {
      regval = (uint8_t *) (RX65N_RIIC2_ICDRR);
    }

  return *regval;
}

/****************************************************************************
 * Name: rx65n_riic_wait_set
 *
 * Description:
 *  Receive "last byte - 2bytes" Setting Proccesing.
 *  Sets ICMR3.WAIT bit.
 *
 ****************************************************************************/

static void rx65n_riic_wait_set(FAR struct rx65n_i2c_priv_s *priv)
{
  uint8_t regval;

  if (0 == priv->bus)
    {
#ifdef CONFIG_RX65N_RIIC0_RCV_IN_BYTE_UNITS
      regval = rx65n_getreg(RX65N_RIIC0_ICMR3);
      regval |= RX65N_RIIC_ICMR3_WAIT_SET;
      rx65n_putreg(regval, RX65N_RIIC0_ICMR3);
      regval = rx65n_getreg(RX65N_RIIC0_ICMR3);
#endif
    }

  else if (1 == priv->bus)
    {
#ifdef CONFIG_RX65N_RIIC1_RCV_IN_BYTE_UNITS
      regval = rx65n_getreg(RX65N_RIIC1_ICMR3);
      regval |= RX65N_RIIC_ICMR3_WAIT_SET;
      rx65n_putreg(regval, RX65N_RIIC1_ICMR3);
      regval = rx65n_getreg(RX65N_RIIC1_ICMR3);
#endif
    }

  else
    {
#ifdef CONFIG_RX65N_RIIC2_RCV_IN_BYTE_UNITS
      regval = rx65n_getreg(RX65N_RIIC2_ICMR3);
      regval |= RX65N_RIIC_ICMR3_WAIT_SET;
      rx65n_putreg(regval, RX65N_RIIC2_ICMR3);
      regval = rx65n_getreg(RX65N_RIIC2_ICMR3);
#endif
    }
}

/****************************************************************************
 * Name: rx65n_riic_pre_end_set
 *
 * Description:
 *  Receive "last byte - 1byte" Setting Processing.
 *  Sets ICMR3.RDRFS bit and ACKBT bit.
 *
 ****************************************************************************/

static void rx65n_riic_pre_end_set(FAR struct rx65n_i2c_priv_s *priv)
{
  if (0 == priv->bus)
    {
#ifdef CONFIG_RX65N_RIIC0_RCV_IN_BYTE_UNITS
      RIIC0.ICMR3.BIT.RDRFS = 1;
#endif
      RIIC0.ICMR3.BIT.ACKWP = 1;
      RIIC0.ICMR3.BIT.ACKBT = 1;
      RIIC0.ICMR3.BIT.ACKWP = 0;

      rx65n_getreg(RX65N_RIIC0_ICMR3);
    }

  else if (1 == priv->bus)
    {
#ifdef CONFIG_RX65N_RIIC1_RCV_IN_BYTE_UNITS
      RIIC1.ICMR3.BIT.RDRFS = 1;
#endif
      RIIC1.ICMR3.BIT.ACKWP = 1;
      RIIC1.ICMR3.BIT.ACKBT = 1;
      RIIC1.ICMR3.BIT.ACKWP = 0;

      rx65n_getreg(RX65N_RIIC1_ICMR3);
    }

  else
    {
#ifdef CONFIG_RX65N_RIIC2_RCV_IN_BYTE_UNITS
      RIIC2.ICMR3.BIT.RDRFS = 1;
#endif
      RIIC2.ICMR3.BIT.ACKWP = 1;
      RIIC2.ICMR3.BIT.ACKBT = 1;
      RIIC2.ICMR3.BIT.ACKWP = 0;

      rx65n_getreg(RX65N_RIIC2_ICMR3);
    }
}

/****************************************************************************
 * Name: rx65n_riic_end_set
 *
 * Description:
 *  Receive End Setting Processing.
 *  Sets ICMR3.ACKBT bit and clears ICMR3.WAIT bit.
 *
 ****************************************************************************/

static void rx65n_riic_end_set(FAR struct rx65n_i2c_priv_s *priv)
{
  if (0 == priv->bus)
    {
      RIIC0.ICMR3.BIT.ACKWP = 1;
      RIIC0.ICMR3.BIT.ACKBT = 1;
      RIIC0.ICMR3.BIT.ACKWP = 0;
#ifdef CONFIG_RX65N_RIIC0_RCV_IN_BYTE_UNITS
      RIIC0.ICMR3.BIT.WAIT = 0;
#endif

      rx65n_getreg(RX65N_RIIC0_ICMR3);
    }

  else if (1 == priv->bus)
    {
      RIIC1.ICMR3.BIT.ACKWP = 1;
      RIIC1.ICMR3.BIT.ACKBT = 1;
      RIIC1.ICMR3.BIT.ACKWP = 0;
#ifdef CONFIG_RX65N_RIIC1_RCV_IN_BYTE_UNITS
      RIIC1.ICMR3.BIT.WAIT = 0;
#endif

      rx65n_getreg(RX65N_RIIC1_ICMR3);
    }

  else
    {
      RIIC2.ICMR3.BIT.ACKWP = 1;
      RIIC2.ICMR3.BIT.ACKBT = 1;
      RIIC2.ICMR3.BIT.ACKWP = 0;
#ifdef CONFIG_RX65N_RIIC2_RCV_IN_BYTE_UNITS
      RIIC2.ICMR3.BIT.WAIT = 0;
#endif

      rx65n_getreg(RX65N_RIIC2_ICMR3);
    }
}

/****************************************************************************
 * Name: rx65n_riic_master_receive
 *
 * Description:
 *  Transmission of data from Master to Slave
 *
 ****************************************************************************/

static void rx65n_riic_master_receive(FAR struct rx65n_i2c_priv_s *priv)
{
  uint8_t regval;
  priv->dev_sts = RIIC_STS_RECEIVE_DATA_WAIT;

  /* The period between the ninth clock cycle and the first clock cycle
   * is held low. Low-hold is released by reading the ICDRR register.
   */

  if (0x03 >= priv->dcnt)
    {
      rx65n_riic_wait_set(priv);
    }

  if (0x02 >= priv->dcnt)
    {
      rx65n_riic_pre_end_set(priv);
    }

  if (0x01 >= priv->dcnt)
    {
      rx65n_riic_set_icier(priv, RX65N_RIIC_ICIER_SP_NAK_AL);

      priv->dev_sts = RIIC_STS_SP_COND_WAIT;
      rx65n_riic_stopcond(priv);

      if (0x00 != priv->dcnt)
        {
          regval = rx65n_riic_read_data(priv);
          *(priv->msgv->buffer) = regval;
          priv->msgv->buffer++;
          *(priv->ptr) = regval;
          priv->dcnt--;
        }

      else
        {
          regval = rx65n_riic_read_data(priv);
        }

      rx65n_riic_end_set(priv);
    }

  else
    {
      regval = rx65n_riic_read_data(priv);
      *(priv->msgv->buffer) = regval;
      priv->msgv->buffer++;
      *(priv->ptr) = regval;
      priv->dcnt--;
    }
}

/****************************************************************************
 * Name: rx65n_riic_rxi0interrupt
 *
 * Description:
 *  Interrupt RIIC0 RXI0 handler – Received Data Full Interrupt Handler
 *
 * Occurs under following conditions:
 *  - Address/data transmission completed in Master Receive Mode
 *  - Reception of (last data – 1) completed in Master Receive Mode
 *  - Reception of last data completed in Master Receive Mode
 *
 ****************************************************************************/

static int rx65n_riic_rxi0interrupt(int irq, FAR void *context, \
                                      FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;

  priv->event = RIIC_EV_INT_RECEIVE;

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_riic_rxi1interrupt
 *
 * Description:
 *  Interrupt RIIC1 RXI1 handler – Received Data Full Interrupt Handler
 *
 * Occurs under following conditions:
 *  - Address/data transmission completed in Master Receive Mode
 *  - Reception of (last data – 1) completed in Master Receive Mode
 *  - Reception of last data completed in Master Receive Mode
 *
 ****************************************************************************/

static int rx65n_riic_rxi1interrupt(int irq, FAR void *context, \
                                                  FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;

  priv->event = RIIC_EV_INT_RECEIVE;

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_i2c_rxi2interrupt
 *
 * Description:
 *  Interrupt RIIC2 RXI2 handler – Received Data Full Interrupt Handler
 *
 * Occurs under following conditions:
 *  - Address/data transmission completed in Master Receive Mode
 *  - Reception of (last data – 1) completed in Master Receive Mode
 *  - Reception of last data completed in Master Receive Mode
 *
 ****************************************************************************/

static int rx65n_riic_rxi2interrupt(int irq, FAR void *context, \
                                        FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;
  priv->event = RIIC_EV_INT_RECEIVE;

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_riic_txi0interrupt
 *
 * Description:
 *  Interrupt RIIC0 TXI0 handler – Transmit Data Empty Interrupt Handler
 *
 * Occurs under following conditions:
 *  - Transmit Buffer is empty
 *
 ****************************************************************************/

static int rx65n_riic_txi0interrupt(int irq, FAR void *context, \
                                       FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;

  /* Ideally, should never get here as this interrupt only occurs during
   * Multi-master mode of operation and
   * Slave transmission/recption mode
   */

  priv->event = RIIC_EV_INT_TXI;

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_riic_txi1interrupt
 *
 * Description:
 *  Interrupt RIIC1 TXI1 handler – Transmit Data Empty Interrupt Handler
 *
 * Occurs under following conditions:
 *  - Transmit Buffer is empty
 *
 ****************************************************************************/

static int rx65n_riic_txi1interrupt(int irq, FAR void *context, \
                                         FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;

  /* Ideally, should never get here as this interrupt only occurs during
   * Multi-master mode of operation and
   * Slave transmission/recption mode
   */

  priv->event = RIIC_EV_INT_TXI;

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_riic_txi2interrupt
 *
 * Description:
 *  Interrupt RIIC2 TXI2 handler – Transmit Data Empty Interrupt Handler
 *
 * Occurs under following conditions:
 *  - Transmit Buffer is empty
 *
 ****************************************************************************/

static int rx65n_riic_txi2interrupt(int irq, FAR void *context, \
                                       FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;

  /* Ideally, should never get here as this interrupt only occurs during
   * Multi-master mode of operation and
   * Slave transmission/recption mode
   */

  priv->event = RIIC_EV_INT_TXI;

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_riic_tei0interrupt
 *
 * Description:
 *  Interrupt RIIC0 TEI0 handler – Transmission End Interrupt Handler
 *
 * Occurs under following conditions:
 *  - Address/data transmission completed
 *
 ****************************************************************************/

static int rx65n_riic_tei0interrupt(int irq, FAR void *context, \
                                    FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;

  RIIC0.ICSR2.BIT.TEND = 0U;
  while (0U != RIIC0.ICSR2.BIT.TEND)
    {
      /* Do Nothing */
    }

  if ((RIIC_STS_SEND_SLVADR_W_WAIT == priv->dev_sts) ||
     (RIIC_STS_SEND_SLVADR_R_WAIT == priv->dev_sts))
    {
      /* Sets interrupted address sending - slave address transmitted */

      priv->event = RIIC_EV_INT_ADD;
    }

  else if (RIIC_STS_SEND_DATA_WAIT == priv->dev_sts)
    {
      /* Sets interrupted data sending */

      priv->event = RIIC_EV_INT_SEND;
    }

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_riic_tei1interrupt
 *
 * Description:
 *  Interrupt RIIC1 TEI1 handler – Transmission End Interrupt Handler
 *
 * Occurs under following conditions:
 *  - Address/data transmission completed
 *
 ****************************************************************************/

static int rx65n_riic_tei1interrupt(int irq, FAR void *context, \
                                    FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;

  RIIC1.ICSR2.BIT.TEND = 0U;
  while (0U != RIIC1.ICSR2.BIT.TEND)
    {
      /* Do Nothing */
    }

  if ((RIIC_STS_SEND_SLVADR_W_WAIT == priv->dev_sts) ||
     (RIIC_STS_SEND_SLVADR_R_WAIT == priv->dev_sts))
    {
      priv->event = RIIC_EV_INT_ADD;
    }

  else if (RIIC_STS_SEND_DATA_WAIT == priv->dev_sts)
    {
      priv->event = RIIC_EV_INT_SEND;
    }

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_riic_tei2interrupt
 *
 * Description:
 *  Interrupt RIIC2 TEI2 handler – Transmission End Interrupt Handler
 *
 * Occurs under following conditions:
 * - Address/data transmission completed
 *
 ****************************************************************************/

static int rx65n_riic_tei2interrupt(int irq, FAR void *context, \
                                    FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;

  RIIC2.ICSR2.BIT.TEND = 0U;
  while (0U != RIIC2.ICSR2.BIT.TEND)
    {
      /* Do Nothing */
    }

  if ((RIIC_STS_SEND_SLVADR_W_WAIT == priv->dev_sts) ||
     (RIIC_STS_SEND_SLVADR_R_WAIT == priv->dev_sts))
    {
      priv->event = RIIC_EV_INT_ADD;
    }

  else if (RIIC_STS_SEND_DATA_WAIT == priv->dev_sts)
    {
      priv->event = RIIC_EV_INT_SEND;
    }

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_riic_eei0interrupt
 *
 * Description:
 *  Interrupt RIIC0 EEI0 handler – Event/Error Generation Interrupt Handler
 *
 * Occurs under following conditions:
 *  - START condition detected
 *  - RESTART condition detected
 *  - STOP condition detected
 *  - NACK detected
 *  - AL (arbitration-lost) detected
 *  - TMO (timeout detection)
 *
 ****************************************************************************/

static int rx65n_riic_eei0interrupt(int irq, FAR void *context, \
                                    FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;

  /* Check Timeout Condition */

  if ((0U != RIIC0.ICSR2.BIT.TMOF) && (0U != RIIC0.ICIER.BIT.TMOIE))
    {
      RIIC0.ICIER.BIT.TMOIE = 0U;
      while (0U != RIIC0.ICIER.BIT.TMOIE)
        {
          /* Wait for reset to complete */
        }

      priv->event = RIIC_EV_INT_TMO;
    }

  /* Check Arbitration-Lost Condition */

  if ((0U != RIIC0.ICSR2.BIT.AL) && (0U != RIIC0.ICIER.BIT.ALIE))
    {
      RIIC0.ICIER.BIT.ALIE = 0U;
      while (0U != RIIC0.ICIER.BIT.ALIE)
        {
          /* Do Nothing */
        }

      priv->event = RIIC_EV_INT_AL;
    }

  /* Check Stop Condition */

  if ((0U != RIIC0.ICSR2.BIT.STOP) && (0U != RIIC0.ICIER.BIT.SPIE))
    {
      RIIC0.ICIER.BIT.SPIE = 0U;

      RIIC0.ICMR3.BIT.RDRFS = 0U;
      RIIC0.ICMR3.BIT.ACKWP = 1U;
      RIIC0.ICMR3.BIT.ACKBT = 0U;
      RIIC0.ICMR3.BIT.ACKWP = 0U;

      while ((0U != RIIC0.ICMR3.BIT.RDRFS) || \
        (0U != RIIC0.ICMR3.BIT.ACKBT))
        {
          /* Do Nothing */
        }

      RIIC0.ICSR2.BIT.STOP = 0U;
      while (0U != RIIC0.ICSR2.BIT.STOP)
        {
          /* Do Nothing */
        }

      priv->event = RIIC_EV_INT_STOP;
    }

  /* Check NACK reception. */

  if ((0U != RIIC0.ICSR2.BIT.NACKF) && (0U != RIIC0.ICIER.BIT.NAKIE))
    {
      /* Prohibits NACK interrupt to generate stop condition. */

      RIIC0.ICIER.BIT.NAKIE = 0U;

      /* Prohibits these interrupt.
       * After NACK interrupt, these interrupts will occur
       * when they do not stop the following interrupts.
       */

      RIIC0.ICIER.BIT.TEIE = 0U;
      RIIC0.ICIER.BIT.TIE = 0U;
      RIIC0.ICIER.BIT.RIE = 0U;

      while (((0U != RIIC0.ICIER.BIT.TEIE) || \
        (0U != RIIC0.ICIER.BIT.TIE)) || (0U != RIIC0.ICIER.BIT.RIE))
        {
          /* Do Nothing */
        }

      if (0U == RIIC0.ICCR2.BIT.TRS)
        {
          IR(RIIC0, RXI0) = 0;
        }

      priv->event = RIIC_EV_INT_NACK;
    }

  /* Check Start condition detection. */

  if ((0U != RIIC0.ICSR2.BIT.START) && (0U != RIIC0.ICIER.BIT.STIE))
    {
      RIIC0.ICIER.BIT.STIE = 0U;
      RIIC0.ICSR2.BIT.START = 0U;
      while ((0U != RIIC0.ICSR2.BIT.START) || \
       (0U != RIIC0.ICIER.BIT.STIE))
        {
          /* Wait till reset is completed */
        }

      /* Sets event flag. */

      priv->event = RIIC_EV_INT_START;
    }

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_riic_eei1interrupt
 *
 * Description:
 *  Interrupt RIIC1 EEI1 handler – Event/Error Generation Interrupt Handler
 *
 * Occurs under following conditions:
 *  - START condition detected
 *  - RESTART condition detected
 *  - STOP condition detected
 *  - NACK detected
 *  - AL (arbitration-lost) detected
 *  - TMO (timeout detection)
 *
 ****************************************************************************/

static int rx65n_riic_eei1interrupt(int irq, FAR void *context, \
                                    FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;

  /* Check Timeout Condition */

  if ((0U != RIIC1.ICSR2.BIT.TMOF) && (0U != RIIC1.ICIER.BIT.TMOIE))
    {
      RIIC1.ICIER.BIT.TMOIE = 0U;
      while (0U != RIIC1.ICIER.BIT.TMOIE)
        {
          /* Wait for reset to complete */
        }

      priv->event = RIIC_EV_INT_TMO;
    }

  /* Check Arbitration-Lost Condition */

  if ((0U != RIIC1.ICSR2.BIT.AL) && (0U != RIIC1.ICIER.BIT.ALIE))
    {
      RIIC1.ICIER.BIT.ALIE = 0U;
      while (0U != RIIC1.ICIER.BIT.ALIE)
        {
          /* Do Nothing */
        }

      priv->event = RIIC_EV_INT_AL;
    }

  /* Check Stop Condition */

  if ((0U != RIIC1.ICSR2.BIT.STOP) && (0U != RIIC1.ICIER.BIT.SPIE))
    {
      RIIC1.ICIER.BIT.SPIE = 0U;
      RIIC1.ICMR3.BIT.RDRFS = 0U;
      RIIC1.ICMR3.BIT.ACKWP = 1U;
      RIIC1.ICMR3.BIT.ACKBT = 0U;
      RIIC1.ICMR3.BIT.ACKWP = 0U;

      while ((0U != RIIC1.ICMR3.BIT.RDRFS) || \
          (0U != RIIC1.ICMR3.BIT.ACKBT))
        {
          /* Do Nothing */
        }

      RIIC1.ICSR2.BIT.STOP = 0U;
      while (0U != RIIC1.ICSR2.BIT.STOP)
        {
          /* Do Nothing */
        }

      priv->event = RIIC_EV_INT_STOP;
    }

  /* Check NACK reception. */

  if ((0U != RIIC1.ICSR2.BIT.NACKF) && (0U != RIIC1.ICIER.BIT.NAKIE))
    {
      /* Prohibits NACK interrupt to generate stop condition. */

      RIIC1.ICIER.BIT.NAKIE = 0U;

      /* Prohibits these interrupt.
       * After NACK interrupt, these interrupts will occur
       * when they do not stop the following interrupts.
       */

      RIIC1.ICIER.BIT.TEIE = 0U;
      RIIC1.ICIER.BIT.TIE = 0U;
      RIIC1.ICIER.BIT.RIE = 0U;

      while (((0U != RIIC1.ICIER.BIT.TEIE) || \
          (0U != RIIC1.ICIER.BIT.TIE)) || (0U != RIIC1.ICIER.BIT.RIE))
        {
          /* Do Nothing */
        }

      if (0U == RIIC1.ICCR2.BIT.TRS)
        {
          IR(RIIC1, RXI1) = 0;
        }

      priv->event = RIIC_EV_INT_NACK;
    }

  /* Check Start condition detection. */

  if ((0U != RIIC1.ICSR2.BIT.START) && (0U != RIIC1.ICIER.BIT.STIE))
    {
      RIIC1.ICIER.BIT.STIE = 0U;
      RIIC1.ICSR2.BIT.START = 0U;
      while ((0U != RIIC1.ICSR2.BIT.START) || (0U != RIIC1.ICIER.BIT.STIE))
        {
          /* Wait till reset is completed */
        }

      priv->event = RIIC_EV_INT_START;
    }

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_riic_eei2interrupt
 *
 * Description:
 *  Interrupt RIIC2 EEI2 handler – Event/Error Generation Interrupt Handler
 *
 * Occurs under following conditions:
 *  - START condition detected
 *  - RESTART condition detected
 *  - STOP condition detected
 *  - NACK detected
 *  - AL (arbitration-lost) detected
 *  - TMO (timeout detection)
 *
 ****************************************************************************/

static int rx65n_riic_eei2interrupt(int irq, FAR void *context, \
                                    FAR void *arg)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)arg;

  /* Check Timeout Condition */

  if ((0U != RIIC2.ICSR2.BIT.TMOF) && (0U != RIIC2.ICIER.BIT.TMOIE))
    {
      RIIC2.ICIER.BIT.TMOIE = 0U;
      while (0U != RIIC2.ICIER.BIT.TMOIE)
        {
          /* Wait for reset to complete */
        }

      priv->event = RIIC_EV_INT_TMO;
    }

  /* Check Arbitration-Lost Condition */

  if ((0U != RIIC2.ICSR2.BIT.AL) && (0U != RIIC2.ICIER.BIT.ALIE))
    {
      RIIC2.ICIER.BIT.ALIE = 0U;
      while (0U != RIIC2.ICIER.BIT.ALIE)
        {
          /* Do Nothing */
        }

      priv->event = RIIC_EV_INT_AL;
    }

  /* Check Stop Condition */

  if ((0U != RIIC2.ICSR2.BIT.STOP) && (0U != RIIC2.ICIER.BIT.SPIE))
    {
      RIIC2.ICIER.BIT.SPIE = 0U;

      RIIC2.ICMR3.BIT.RDRFS = 0U;
      RIIC2.ICMR3.BIT.ACKWP = 1U;
      RIIC2.ICMR3.BIT.ACKBT = 0U;
      RIIC2.ICMR3.BIT.ACKWP = 0U;

      while ((0U != RIIC2.ICMR3.BIT.RDRFS) || \
        (0U != RIIC2.ICMR3.BIT.ACKBT))
        {
          /* Do Nothing */
        }

      RIIC2.ICSR2.BIT.STOP = 0U;
      while (0U != RIIC2.ICSR2.BIT.STOP)
        {
          /* Do Nothing */
        }

      priv->event = RIIC_EV_INT_STOP;
    }

  /* Check NACK reception. */

  if ((0U != RIIC2.ICSR2.BIT.NACKF) && (0U != RIIC2.ICIER.BIT.NAKIE))
    {
      /* Prohibits NACK interrupt to generate stop condition. */

      RIIC2.ICIER.BIT.NAKIE = 0U;

      /* Prohibits these interrupt.
       * After NACK interrupt, these interrupts will occur
       * when they do not stop the following interrupts.
       */

      RIIC2.ICIER.BIT.TEIE = 0U;
      RIIC2.ICIER.BIT.TIE = 0U;
      RIIC2.ICIER.BIT.RIE = 0U;

      while (((0U != RIIC2.ICIER.BIT.TEIE) || \
        (0U != RIIC2.ICIER.BIT.TIE)) || (0U != RIIC2.ICIER.BIT.RIE))
        {
          /* Do Nothing */
        }

      if (0U == RIIC2.ICCR2.BIT.TRS)
        {
          IR(RIIC2, RXI2) = 0;
        }

      priv->event = RIIC_EV_INT_NACK;
    }

  /* Check Start condition detection. */

  if ((0U != RIIC2.ICSR2.BIT.START) && (0U != RIIC2.ICIER.BIT.STIE))
    {
      RIIC2.ICIER.BIT.STIE = 0U;
      RIIC2.ICSR2.BIT.START = 0U;
      while ((0U != RIIC2.ICSR2.BIT.START) || \
       (0U != RIIC2.ICIER.BIT.STIE))
        {
          /* Wait till reset is completed */
        }

      priv->event = RIIC_EV_INT_START;
    }

  rx65n_riic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: rx65n_i2c_transfer
 *
 * Description:
 *   This function initializes the RIIC channel
 *
 ****************************************************************************/

static int rx65n_i2c_transfer(FAR struct i2c_master_s *dev, \
            FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)dev;
  int ret = 0;

  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);

  /* Get exclusive access to the I2C bus */

  nxsem_wait(&priv->sem_excl);

  priv->mode = RIIC_READY;
  priv->dev_sts = RIIC_STS_IDLE;
  priv->event = RIIC_EV_NONE;

  priv->dcnt = 0;
  priv->ptr = NULL;

  priv->msgv = msgs;
  priv->msgc = count;

  priv->ptr = priv->msgv->buffer;
  priv->dcnt = priv->msgv->length;
  priv->flags = priv->msgv->flags;

  /* Set reference clock for I2C bus */

  rx65n_riic_setclock(priv, msgs->frequency);

  if ((priv->flags == 0) || (priv->flags == I2C_M_NOSTOP)
     || (priv->flags == I2C_M_TEN))
    {
      /* MASTER TRANSMISSION MODE
       * No communication is in progress
       * Initiate START condition and proceed with transfer
       */

      if (priv->mode == RIIC_READY)
        {
          /* Device is ready for communication */

          priv->event = RIIC_EV_GEN_START_COND;
          ret = rx65n_riic_startcond(priv);
          if (ret == RIIC_ERR_BUS_BUSY)
            {
              priv->mode = RIIC_NONE;
              priv->dev_sts = RIIC_STS_NO_INIT;
              priv->event = RIIC_EV_NONE;

              /* Disable interrupts */

              rx65n_riic_int_disable(priv);
            }
        }
    }

  else if (priv->flags == I2C_M_READ)
    {
      /* MASTER RECEPTION MODE
       * No communication is in progress
       * Initiate START condition and proceed with transfer
       */

      if (priv->mode == RIIC_READY)
        {
          /* Device is ready for communication */

          priv->event = RIIC_EV_GEN_START_COND;
          ret = rx65n_riic_startcond(priv);
          if (ret == RIIC_ERR_BUS_BUSY)
            {
              priv->mode = RIIC_NONE;
              priv->dev_sts = RIIC_STS_NO_INIT;
              priv->event = RIIC_EV_NONE;

              /* Disable interrupts */

              rx65n_riic_int_disable(priv);
            }
        }
    }

  else if (priv->flags == I2C_M_NOSTART)
    {
      /* COMMUNICATION IS IN PROGRESS */

      if (priv->mode == RIIC_M_TRANSMIT)
        {
          priv->ptr = priv->msgv->buffer;
          priv->dcnt = priv->msgv->length;
          rx65n_riic_master_transmit(priv);
        }

      else if (priv->mode == RIIC_M_RECEIVE)
        {
          rx65n_riic_master_receive(priv);
        }
    }

  nxsem_post(&priv->sem_excl);

  while (RIIC_FINISH != priv->mode && RIIC_NONE != priv->mode);

  if (0 == priv->bus)
    {
      if (1U == RIIC0.ICSR2.BIT.NACKF)
        {
          ret = -ENXIO;
        }

      else if ((1U == RIIC0.ICCR2.BIT.BBSY) || (RIIC_ERR_BUS_BUSY == ret))
        {
          ret = -EBUSY;
        }

      else
        {
          ret = RIIC_SUCCESS;
        }
    }

  else if (1 == priv->bus)
    {
      if (1U == RIIC1.ICSR2.BIT.NACKF)
        {
          ret = -ENXIO;
        }

      else if ((1U == RIIC1.ICCR2.BIT.BBSY) || (RIIC_ERR_BUS_BUSY == ret))
        {
          ret = -EBUSY;
        }

      else
        {
          ret = RIIC_SUCCESS;
        }
    }

  else
    {
      if (1U == RIIC2.ICSR2.BIT.NACKF)
        {
          ret = -ENXIO;
        }

      else if ((1U == RIIC2.ICCR2.BIT.BBSY) || (RIIC_ERR_BUS_BUSY == ret))
        {
          ret = -EBUSY;
        }

      else
        {
          ret = RIIC_SUCCESS;
        }
    }

  rx65n_riic_iicrst(priv);
  return ret;
}

/****************************************************************************
 * Name: rx65n_i2c_reset
 *
 * Description:
 *   Resets the RIIC channel
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int rx65n_i2c_reset(FAR struct i2c_master_s *dev)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)dev;
  uint8_t regval;

  DEBUGASSERT(dev);

  DEBUGASSERT(priv->refs > 0);

  nxsem_wait(&priv->sem_excl);

  if (priv->bus == 0)
    {
      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);
      regval |= RX65N_RIIC_ICCR1_IICRST_SET; /* ICCR1.IICRST bit to 1. */
      rx65n_putreg(regval, RX65N_RIIC0_ICCR1);
      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);

      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);
      regval &= ~(RX65N_RIIC_ICCR1_IICRST_SET); /* ICCR1.IICRST bit to 0. */
      rx65n_putreg(regval, RX65N_RIIC0_ICCR1);
      regval = rx65n_getreg(RX65N_RIIC0_ICCR1);
    }

  else if (priv->bus == 1)
    {
      regval = rx65n_getreg(RX65N_RIIC1_ICCR1);
      regval |= RX65N_RIIC_ICCR1_IICRST_SET; /* ICCR1.IICRST bit to 1. */
      rx65n_putreg(regval, RX65N_RIIC1_ICCR1);

      regval = rx65n_getreg(RX65N_RIIC1_ICCR1);
      regval &= ~(RX65N_RIIC_ICCR1_IICRST_SET); /* ICCR1.IICRST bit to 0. */
      rx65n_putreg(regval, RX65N_RIIC1_ICCR1);
    }

  else
    {
      regval = rx65n_getreg(RX65N_RIIC2_ICCR1);
      regval |= RX65N_RIIC_ICCR1_IICRST_SET; /* ICCR1.IICRST bit to 1. */
      rx65n_putreg(regval, RX65N_RIIC2_ICCR1);

      regval = rx65n_getreg(RX65N_RIIC2_ICCR1);
      regval &= ~(RX65N_RIIC_ICCR1_IICRST_SET); /* ICCR1.IICRST bit to 0. */
      rx65n_putreg(regval, RX65N_RIIC2_ICCR1);
    }

  nxsem_post(&priv->sem_excl);
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Name: rx65n_i2cbus_initialize
 *
 * Description:
 *   This function initializes the RIIC channel
 *
 ****************************************************************************/

FAR struct i2c_master_s *rx65n_i2cbus_initialize(int channel)
{
  struct rx65n_i2c_priv_s * priv = NULL;
  irqstate_t irqs;

  /* Get I2C private structure */

  i2cinfo("RX65N RIIC Bus Initialization:\n");
  riic_mpc_enable();
  switch (channel)
    {
#ifdef CONFIG_RX65N_RIIC0
      case 0:
        riic0_init_port();
        i2cinfo("RX65N RIIC0 Channel Initial Setup\n");
        priv = (struct rx65n_i2c_priv_s *)&rx65n_riic0_priv;
        break;
#endif
#ifdef CONFIG_RX65N_RIIC1
      case 1:
        riic1_init_port();
        i2cinfo("RX65N RIIC1 Channel Initial Setup\n");
        priv = (struct rx65n_i2c_priv_s *)&rx65n_riic1_priv;
        break;
#endif
#ifdef CONFIG_RX65N_RIIC2
      case 2:
        riic2_init_port();
        i2cinfo("RX65N RIIC2 Channel Initial Setup\n");
        priv = (struct rx65n_i2c_priv_s *)&rx65n_riic2_priv;
        break;
#endif
      default:
        i2cerr("Channel %d is not supported by RX65N\n", channel);
        riic_mpc_disable();
        return NULL;
    }

  /* Initialize private data for the first time, increment reference count,
   * initialize RIIC registers and attach IRQs
   */

  irqs = enter_critical_section();

  if ((volatile int)priv->refs++ == 0)
    {
      /* Initialize semaphores */

      nxsem_init(&priv->sem_excl, 0, 1);
      nxsem_init(&priv->sem_isr, 0, 0);

      /* Initialize the RIIC registers */

      rx65n_riic_init(priv);
    }

  leave_critical_section(irqs);
  riic_mpc_disable();
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: rx65n_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an RIIC device
 *
 ****************************************************************************/

int rx65n_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  FAR struct rx65n_i2c_priv_s *priv = (struct rx65n_i2c_priv_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

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

  /* Disable power and other HW resource (GPIO's) */

  rx65n_riic_int_disable(priv);

  irq_detach(priv->dev->txi_irq);
  irq_detach(priv->dev->rxi_irq);
  irq_detach(priv->dev->tei_irq);
  irq_detach(priv->dev->eei_irq);

  /* Release unused resources */

  nxsem_destroy(&priv->sem_excl);
  nxsem_destroy(&priv->sem_isr);

  return OK;
}

#endif /* CONFIG_RX65N_RIIC0 || CONFIG_RX65N_RIIC1 || CONFIG_RX65N_RIIC2 */
