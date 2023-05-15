/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_i2c.c
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "mpfs_i2c.h"
#include "riscv_internal.h"
#include "hardware/mpfs_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_SYSREG_SOFT_RESET_CR   (MPFS_SYSREG_BASE + \
                                     MPFS_SYSREG_SOFT_RESET_CR_OFFSET)
#define MPFS_SYSREG_SUBBLK_CLOCK_CR (MPFS_SYSREG_BASE + \
                                     MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET)

#define MPFS_I2C_CTRL_OFFSET        0x00
#define MPFS_I2C_STATUS_OFFSET      0x04
#define MPFS_I2C_DATA_OFFSET        0x08
#define MPFS_I2C_SLAVE0ADR_OFFSET   0x0C
#define MPFS_I2C_SMBUS_OFFSET       0x10
#define MPFS_I2C_FREQ_OFFSET        0x14

#define MPFS_I2C_CTRL             (priv->hw_base + MPFS_I2C_CTRL_OFFSET)
#define MPFS_I2C_STATUS           (priv->hw_base + MPFS_I2C_STATUS_OFFSET)
#define MPFS_I2C_DATA             (priv->hw_base + MPFS_I2C_DATA_OFFSET)
#define MPFS_I2C_ADDR             (priv->hw_base + MPFS_I2C_SLAVE0ADR_OFFSET)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum mpfs_i2c_status
{
  MPFS_I2C_SUCCESS = 0u,
  MPFS_I2C_IN_PROGRESS,
  MPFS_I2C_FAILED,
  MPFS_I2C_TIMED_OUT
} mpfs_i2c_status_t;

typedef enum mpfs_i2c_clock_divider
{
  MPFS_I2C_PCLK_DIV_256 = 0u,
  MPFS_I2C_PCLK_DIV_224,
  MPFS_I2C_PCLK_DIV_192,
  MPFS_I2C_PCLK_DIV_160,
  MPFS_I2C_PCLK_DIV_960,
  MPFS_I2C_PCLK_DIV_120,
  MPFS_I2C_PCLK_DIV_60,
  MPFS_I2C_BCLK_DIV_8, /* FPGA generated BCLK */
  MPFS_I2C_NUMBER_OF_DIVIDERS
} mpfs_i2c_clk_div_t;

static const uint32_t mpfs_i2c_freqs[MPFS_I2C_NUMBER_OF_DIVIDERS] =
{
  MPFS_MSS_APB_AHB_CLK / 256,
  MPFS_MSS_APB_AHB_CLK / 224,
  MPFS_MSS_APB_AHB_CLK / 192,
  MPFS_MSS_APB_AHB_CLK / 160,
  MPFS_MSS_APB_AHB_CLK / 960,
  MPFS_MSS_APB_AHB_CLK / 120,
  MPFS_MSS_APB_AHB_CLK / 60,
  MPFS_FPGA_BCLK / 8
};

static const uint32_t mpfs_i2c_freqs_fpga[MPFS_I2C_NUMBER_OF_DIVIDERS] =
{
  MPFS_FPGA_PERIPHERAL_CLK / 256,
  MPFS_FPGA_PERIPHERAL_CLK / 224,
  MPFS_FPGA_PERIPHERAL_CLK / 192,
  MPFS_FPGA_PERIPHERAL_CLK / 160,
  MPFS_FPGA_PERIPHERAL_CLK / 960,
  MPFS_FPGA_PERIPHERAL_CLK / 120,
  MPFS_FPGA_PERIPHERAL_CLK / 60,
  MPFS_FPGA_BCLK / 8
};

static int mpfs_i2c_transfer(struct i2c_master_s *dev,
                             struct i2c_msg_s *msgs,
                             int count);

#ifdef CONFIG_I2C_RESET
static int mpfs_i2c_reset(struct i2c_master_s *dev);
#endif

static const struct i2c_ops_s mpfs_i2c_ops =
{
  .transfer = mpfs_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset    = mpfs_i2c_reset,
#endif
};

struct mpfs_i2c_priv_s
{
  const struct i2c_ops_s *ops;        /* Standard I2C operations */
  uint32_t               id;          /* I2C hardware identification */
  uintptr_t              hw_base;     /* I2C bus base address */
  uint16_t               plic_irq;    /* Platform PLIC irq */
  struct i2c_msg_s       *msgv;       /* Message list */
  uint32_t               frequency;   /* Current I2C frequency */

  uint8_t                msgid;       /* Current message ID */
  ssize_t                bytes;       /* Processed data bytes */

  uint8_t                ser_address; /* Own i2c address */
  uint8_t                target_addr; /* Target i2c address */
  mutex_t                lock;        /* Mutual exclusion mutex */
  sem_t                  sem_isr;     /* Interrupt wait semaphore */
  int                    refs;        /* Reference count */

  const uint8_t          *tx_buffer;  /* Tx buffer location */
  uint16_t               tx_size;     /* Tx buffer size */
  uint16_t               tx_idx;      /* Currently accessed index */

  uint8_t                *rx_buffer;  /* Rx buffer location */
  uint16_t               rx_size;     /* Rx buffer size */
  uint16_t               rx_idx;      /* Currently accessed index */

  mpfs_i2c_status_t      status;      /* Bus driver status */

  bool                   initialized; /* Bus initialization status */
  bool                   fpga;        /* FPGA i2c */
};

#ifdef CONFIG_MPFS_I2C0
static struct mpfs_i2c_priv_s g_mpfs_i2c0_lo_priv =
{
  .ops            = &mpfs_i2c_ops,
  .id             = 0,
  .hw_base        = MPFS_I2C0_LO_BASE,
  .plic_irq       = MPFS_IRQ_I2C0_MAIN,
  .msgv           = NULL,
  .frequency      = 0,
  .ser_address    = 0x21,
  .target_addr    = 0,
  .lock           = NXMUTEX_INITIALIZER,
  .sem_isr        = SEM_INITIALIZER(0),
  .refs           = 0,
  .tx_size        = 0,
  .tx_idx         = 0,
  .rx_size        = 0,
  .rx_idx         = 0,
  .status         = MPFS_I2C_SUCCESS,
  .initialized    = false,
  .fpga           = false
};
#endif /* CONFIG_MPFS_I2C0 */

#ifdef CONFIG_MPFS_I2C1
static struct mpfs_i2c_priv_s g_mpfs_i2c1_lo_priv =
{
  .ops            = &mpfs_i2c_ops,
  .id             = 1,
  .hw_base        = MPFS_I2C1_LO_BASE,
  .plic_irq       = MPFS_IRQ_I2C1_MAIN,
  .msgv           = NULL,
  .frequency      = 0,
  .ser_address    = 0x21,
  .target_addr    = 0,
  .lock           = NXMUTEX_INITIALIZER,
  .sem_isr        = SEM_INITIALIZER(0),
  .refs           = 0,
  .tx_size        = 0,
  .tx_idx         = 0,
  .rx_size        = 0,
  .rx_idx         = 0,
  .status         = MPFS_I2C_SUCCESS,
  .initialized    = false,
  .fpga           = false
};
#endif /* CONFIG_MPFS_I2C1 */

#ifdef CONFIG_MPFS_COREI2C0
static struct mpfs_i2c_priv_s g_mpfs_corei2c0_priv =
{
  .ops            = &mpfs_i2c_ops,
  .id             = 0,
  .hw_base        = 0x4b000000,
  .plic_irq       = MPFS_IRQ_FABRIC_F2H_6,
  .msgv           = NULL,
  .frequency      = 0,
  .ser_address    = 0,
  .target_addr    = 0,
  .lock           = NXMUTEX_INITIALIZER,
  .sem_isr        = SEM_INITIALIZER(0),
  .refs           = 0,
  .tx_size        = 0,
  .tx_idx         = 0,
  .rx_size        = 0,
  .rx_idx         = 0,
  .status         = MPFS_I2C_SUCCESS,
  .initialized    = false,
  .fpga           = true
};
#endif /* CONFIG_MPFS_COREI2C0 */

#ifdef CONFIG_MPFS_COREI2C1
static struct mpfs_i2c_priv_s g_mpfs_corei2c1_priv =
{
  .ops            = &mpfs_i2c_ops,
  .id             = 1,
  .hw_base        = 0x4b001000,
  .plic_irq       = MPFS_IRQ_FABRIC_F2H_7,
  .msgv           = NULL,
  .frequency      = 0,
  .ser_address    = 0,
  .target_addr    = 0,
  .lock           = NXMUTEX_INITIALIZER,
  .sem_isr        = SEM_INITIALIZER(0),
  .refs           = 0,
  .tx_size        = 0,
  .tx_idx         = 0,
  .rx_size        = 0,
  .rx_idx         = 0,
  .status         = MPFS_I2C_SUCCESS,
  .initialized    = false,
  .fpga           = true
};
#endif /* CONFIG_MPFS_COREI2C1 */

#ifdef CONFIG_MPFS_COREI2C2
static struct mpfs_i2c_priv_s g_mpfs_corei2c2_priv =
{
  .ops            = &mpfs_i2c_ops,
  .id             = 2,
  .hw_base        = 0x4b002000,
  .plic_irq       = MPFS_IRQ_FABRIC_F2H_8,
  .msgv           = NULL,
  .frequency      = 0,
  .ser_address    = 0,
  .target_addr    = 0,
  .lock           = NXMUTEX_INITIALIZER,
  .sem_isr        = SEM_INITIALIZER(0),
  .refs           = 0,
  .tx_size        = 0,
  .tx_idx         = 0,
  .rx_size        = 0,
  .rx_idx         = 0,
  .status         = MPFS_I2C_SUCCESS,
  .initialized    = false,
  .fpga           = true
};
#endif /* CONFIG_MPFS_COREI2C2 */

static int mpfs_i2c_setfrequency(struct mpfs_i2c_priv_s *priv,
                                  uint32_t frequency);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_i2c_init
 *
 * Description:
 *   Initialize and enable an I2C bus
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int mpfs_i2c_init(struct mpfs_i2c_priv_s *priv)
{
  if (!priv->initialized)
    {
      if (priv->fpga)
        {
          /* FIC3 is used by many, don't reset it here, or many
           * FPGA based modules will stop working right here. Just
           * bring out of reset instead.
           */

          modifyreg32(MPFS_SYSREG_SOFT_RESET_CR,
                      SYSREG_SOFT_RESET_CR_FIC3 | SYSREG_SOFT_RESET_CR_FPGA,
                      0);

          modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0,
                      SYSREG_SUBBLK_CLOCK_CR_FIC3);
        }
      else if (priv->id == 0)
        {
          modifyreg32(MPFS_SYSREG_SOFT_RESET_CR,
                      0, SYSREG_SOFT_RESET_CR_I2C0);

          modifyreg32(MPFS_SYSREG_SOFT_RESET_CR,
                      SYSREG_SOFT_RESET_CR_I2C0, 0);

          modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR,
                      0, SYSREG_SUBBLK_CLOCK_CR_I2C0);
        }
      else if (priv->id == 1)
        {
          modifyreg32(MPFS_SYSREG_SOFT_RESET_CR,
                      0, SYSREG_SOFT_RESET_CR_I2C1);

          modifyreg32(MPFS_SYSREG_SOFT_RESET_CR,
                      SYSREG_SOFT_RESET_CR_I2C1, 0);

          modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR,
                      0, SYSREG_SUBBLK_CLOCK_CR_I2C1);
        }
      else
        {
          /* Don't know which one, let's panic */

          PANIC();
        }

      /* Divider is zero after I2C reset */

      if (priv->fpga)
        {
          priv->frequency = mpfs_i2c_freqs_fpga[0];
        }
      else
        {
          priv->frequency = mpfs_i2c_freqs[0];
        }

      /* This is our own address, not the target chip */

      putreg32(priv->ser_address, MPFS_I2C_ADDR);

      /* Enable i2c bus */

      modifyreg32(MPFS_I2C_CTRL, MPFS_I2C_CTRL_ENS1_MASK,
                  MPFS_I2C_CTRL_ENS1_MASK);

      priv->initialized = true;
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_i2c_deinit
 *
 * Description:
 *   Disable I2C hardware.
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void mpfs_i2c_deinit(struct mpfs_i2c_priv_s *priv)
{
  up_disable_irq(priv->plic_irq);
  irq_detach(priv->plic_irq);

  modifyreg32(MPFS_I2C_CTRL, MPFS_I2C_CTRL_ENS1_MASK,
              ~MPFS_I2C_CTRL_ENS1_MASK);

  priv->initialized = false;
}

/****************************************************************************
 * Name: mpfs_i2c_sem_waitdone
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

static int mpfs_i2c_sem_waitdone(struct mpfs_i2c_priv_s *priv)
{
  return nxsem_tickwait_uninterruptible(&priv->sem_isr, SEC2TICK(1));
}

/****************************************************************************
 * Name: mpfs_i2c_irq
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

static int mpfs_i2c_irq(int cpuint, void *context, void *arg)
{
  struct mpfs_i2c_priv_s *priv = (struct mpfs_i2c_priv_s *)arg;
  struct i2c_msg_s *msg = &priv->msgv[priv->msgid];
  volatile uint32_t status;
  uint8_t clear_irq = 1u;

  DEBUGASSERT(msg != NULL);

  status = getreg32(MPFS_I2C_STATUS);

  switch (status)
    {
      case MPFS_I2C_ST_START:
      case MPFS_I2C_ST_RESTART:
        modifyreg32(MPFS_I2C_CTRL, MPFS_I2C_CTRL_STA_MASK, 0);
        putreg32((priv->target_addr << 1) | (msg->flags & I2C_M_READ),
                 MPFS_I2C_DATA);

        if (msg->flags & I2C_M_READ)
          {
            priv->rx_idx = 0u;
          }
        else
          {
            priv->tx_idx = 0u;
          }
        break;

      case MPFS_I2C_ST_LOST_ARB:

        /* Clear interrupt. */

        modifyreg32(MPFS_I2C_CTRL, MPFS_I2C_CTRL_SI_MASK, 0);
        clear_irq = 0u;
        modifyreg32(MPFS_I2C_CTRL, 0, MPFS_I2C_CTRL_STA_MASK);
        break;

      case MPFS_I2C_ST_SLAW_NACK:
        modifyreg32(MPFS_I2C_CTRL, 0, MPFS_I2C_CTRL_STO_MASK);
        priv->status = MPFS_I2C_FAILED;
        break;

      case MPFS_I2C_ST_SLAW_ACK:
      case MPFS_I2C_ST_TX_DATA_ACK:
        if (priv->tx_idx < priv->tx_size)
          {
            DEBUGASSERT(priv->tx_buffer != NULL);
            putreg32(priv->tx_buffer[priv->tx_idx], MPFS_I2C_DATA);
            priv->tx_idx++;
          }
        else if (msg->flags & I2C_M_NOSTOP)
          {
            /* Clear interrupt. */

            modifyreg32(MPFS_I2C_CTRL, MPFS_I2C_CTRL_SI_MASK, 0);
            clear_irq = 0u;
            modifyreg32(MPFS_I2C_CTRL, 0, MPFS_I2C_CTRL_STA_MASK);

          /* Jump to the next message */

            priv->msgid++;
          }
        else
          {
            /* Send stop condition */

            if (priv->fpga && (priv->rx_idx == 0 && priv->rx_size > 0))
              {
                /* This is a known bug: FPGA IP sends data twice after
                 * sending the address occasionally. Instead of sending
                 * STOP, send repeated start instead.
                 */

                modifyreg32(MPFS_I2C_CTRL, MPFS_I2C_CTRL_SI_MASK, 0);
                clear_irq = 0u;
                modifyreg32(MPFS_I2C_CTRL, 0, MPFS_I2C_CTRL_STA_MASK);

               /* Jump to the next message */

                priv->msgid++;
              }
            else
              {
                modifyreg32(MPFS_I2C_CTRL, 0, MPFS_I2C_CTRL_STO_MASK);
              }

            priv->status = MPFS_I2C_SUCCESS;
          }
        break;

      case MPFS_I2C_ST_TX_DATA_NACK:
        modifyreg32(MPFS_I2C_CTRL, 0, MPFS_I2C_CTRL_STO_MASK);
        priv->status = MPFS_I2C_FAILED;
        break;

      case MPFS_I2C_ST_SLAR_ACK: /* SLA+R tx'ed. */
        if (priv->rx_size > 1u)
          {
            modifyreg32(MPFS_I2C_CTRL, 0, MPFS_I2C_CTRL_AA_MASK);
          }
        else if (priv->rx_size == 1u)
          {
            modifyreg32(MPFS_I2C_CTRL, MPFS_I2C_CTRL_AA_MASK, 0);
          }
        else /* priv->rx_size == 0u */
          {
            modifyreg32(MPFS_I2C_CTRL, 0, MPFS_I2C_CTRL_AA_MASK);
            modifyreg32(MPFS_I2C_CTRL, 0, MPFS_I2C_CTRL_STO_MASK);
            priv->status = MPFS_I2C_SUCCESS;
          }
        break;

      case MPFS_I2C_ST_SLAR_NACK: /* SLA+R tx'ed; send a stop condition */
        modifyreg32(MPFS_I2C_CTRL, 0, MPFS_I2C_CTRL_STO_MASK);
        priv->status = MPFS_I2C_FAILED;
        break;

      case MPFS_I2C_ST_RX_DATA_ACK:

        /* Data byte received, ACK returned */

        DEBUGASSERT(priv->rx_buffer != NULL);
        priv->rx_buffer[priv->rx_idx] = (uint8_t)getreg32(MPFS_I2C_DATA);
        priv->rx_idx++;

        if (priv->rx_idx >= (priv->rx_size - 1u))
          {
            modifyreg32(MPFS_I2C_CTRL, MPFS_I2C_CTRL_AA_MASK, 0);
          }
        break;

      case MPFS_I2C_ST_RX_DATA_NACK:

        /* Data byte received, NACK returned */

        DEBUGASSERT(priv->rx_buffer != NULL);
        DEBUGASSERT(priv->rx_idx < priv->rx_size);
        priv->rx_buffer[priv->rx_idx] = (uint8_t)getreg32(MPFS_I2C_DATA);
        priv->rx_idx++;

        priv->status = MPFS_I2C_SUCCESS;
        modifyreg32(MPFS_I2C_CTRL, 0, MPFS_I2C_CTRL_STO_MASK);
        break;

      case MPFS_I2C_ST_IDLE:
      case MPFS_I2C_ST_STOP_SENT:

        /* No activity, bus idle */

        break;

      case MPFS_I2C_ST_RESET_ACTIVATED:
      case MPFS_I2C_ST_BUS_ERROR: /* Bus errors */
      default:

        modifyreg32(MPFS_I2C_CTRL, MPFS_I2C_CTRL_STA_MASK, 0);

        if (priv->status == MPFS_I2C_IN_PROGRESS)
          {
            priv->status = MPFS_I2C_FAILED;
          }

        break;
    }

  if (priv->fpga)
    {
      /* FPGA driver terminates all transactions with STOP sent irq */

      if (status == MPFS_I2C_ST_STOP_SENT)
        {
          /* Don't post on a new request, STOPs possible initially */

          if (!((priv->rx_idx == 0 && priv->rx_size > 0) ||
             (priv->tx_idx == 0 && priv->tx_size > 0)))
            {
              nxsem_post(&priv->sem_isr);
            }
          else if (priv->status == MPFS_I2C_FAILED)
            {
              nxsem_post(&priv->sem_isr);
            }
        }
    }
  else if (priv->status != MPFS_I2C_IN_PROGRESS)
    {
      /* MSS I2C has no STOP SENT irq */

      nxsem_post(&priv->sem_isr);
    }

  if (clear_irq)
    {
      /* Clear interrupt. */

      modifyreg32(MPFS_I2C_CTRL, MPFS_I2C_CTRL_SI_MASK, 0);
    }

  /* Read back the status register to ensure the last I2C registers write
   * took place in a system built around a bus making use of posted writes.
   */

  status = getreg32(MPFS_I2C_STATUS);

  return 0;
}

/****************************************************************************
 * Name: mpfs_i2c_sendstart
 *
 * Description:
 *   Send I2C start condition and enable the PLIC irq
 *
 * Parameters:
 *   priv          - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void mpfs_i2c_sendstart(struct mpfs_i2c_priv_s *priv)
{
  up_enable_irq(priv->plic_irq);
  modifyreg32(MPFS_I2C_CTRL, MPFS_I2C_CTRL_STA_MASK, MPFS_I2C_CTRL_STA_MASK);
}

static int mpfs_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs,
                                int count)
{
  struct mpfs_i2c_priv_s *priv = (struct mpfs_i2c_priv_s *)dev;
  int ret = OK;

  i2cinfo("Starting transfer request of %d message(s):\n", count);
  DEBUGASSERT(count > 0);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->status != MPFS_I2C_SUCCESS)
    {
      priv->status = MPFS_I2C_SUCCESS;
    }

  priv->msgv = msgs;

  for (int i = 0; i < count; i++)
    {
      priv->bytes = 0;
      priv->msgid = i;
      priv->status = MPFS_I2C_IN_PROGRESS;
      priv->target_addr = msgs[i].addr;

      if (msgs[i].flags & I2C_M_READ)
        {
          priv->rx_buffer = msgs[i].buffer;
          priv->rx_size = msgs[i].length;
          priv->rx_idx = 0;

          /* Clear tx_idx as well for combined transactions */

          priv->tx_idx = 0;
          priv->tx_size = 0;
        }
      else
        {
          priv->tx_buffer = msgs[i].buffer;
          priv->tx_size = msgs[i].length;
          priv->tx_idx = 0;

          /* Clear rx_idx as well for combined transactions */

          priv->rx_idx = 0;
          priv->rx_size = 0;

          if (msgs[i].flags & I2C_M_NOSTOP)
            {
              /* Support only write + read combinations */

              DEBUGASSERT(!(msgs[i].flags & I2C_M_READ));

              /* Combine write + read transaction into one */

              if (((i + 1) < count) && (msgs[i + 1].flags & I2C_M_READ))
                {
                  priv->rx_buffer = msgs[i + 1].buffer;
                  priv->rx_size = msgs[i + 1].length;
                  priv->rx_idx = 0;
                  i++;
                }
            }
        }

      ret = mpfs_i2c_setfrequency(priv, msgs[i].frequency);
      if (ret != OK)
        {
          break;
        }

      i2cinfo("Sending message %" PRIu8 "...\n", priv->msgid);

      mpfs_i2c_sendstart(priv);

      if (mpfs_i2c_sem_waitdone(priv) < 0)
        {
          i2cinfo("Message %" PRIu8 " timed out.\n", priv->msgid);
          ret = -ETIMEDOUT;
          break;
        }
      else
        {
          if (priv->status != MPFS_I2C_SUCCESS)
            {
              i2cinfo("Transfer error %" PRIu32 "\n", priv->status);
              ret = -EIO;
              break;
            }
          else
            {
              priv->status = MPFS_I2C_SUCCESS;
              ret = OK;
            }
        }

        i2cinfo("Message %" PRIu8 " transfer complete.\n", priv->msgid);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: mpfs_i2c_reset
 *
 * Description:
 *   Performs an I2C bus reset. This may be used to recover from a buggy
 *   situation.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; this should not fail.
 *
 ****************************************************************************/
#ifdef CONFIG_I2C_RESET
static int mpfs_i2c_reset(struct i2c_master_s *dev)
{
  struct mpfs_i2c_priv_s *priv = (struct mpfs_i2c_priv_s *)dev;
  int ret;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  flags = enter_critical_section();

  /* Disabling I2C interrupts.
   * NOTE: up_enable_irq() will be called at mpfs_i2c_sendstart()
   */

  up_disable_irq(priv->plic_irq);

  priv->initialized = false;

  ret = mpfs_i2c_init(priv);
  if (ret != OK)
    {
      leave_critical_section(flags);
      return ret;
    }

  priv->tx_size = 0;
  priv->tx_idx  = 0;
  priv->rx_size = 0;
  priv->rx_idx  = 0;

  leave_critical_section(flags);

  return OK;
}
#endif

/****************************************************************************
 * Name: mpfs_i2c_setfrequency
 *
 * Description:
 *   Sets the closest possible frequency for the next transfers.
 *
 * Input Parameters:
 *   priv      - Pointer to the internal driver state structure.
 *   frequency - Requested frequency
 *
 * Returned Value:
 *   Zero (OK) on success;
 *
 ****************************************************************************/

static int mpfs_i2c_setfrequency(struct mpfs_i2c_priv_s *priv,
                                  uint32_t frequency)
{
  uint32_t new_freq = 0;
  uint32_t clock_div = 0;

  if (priv->frequency != frequency)
    {
      /* Select the closest possible frequency
       * which is smaller than or equal to requested
       */

      if (priv->fpga)
        {
          /* FPGA clk differs from the others */

          for (uint8_t i = 0; i < MPFS_I2C_NUMBER_OF_DIVIDERS; i++)
            {
              if (frequency >= mpfs_i2c_freqs_fpga[i]
                  && mpfs_i2c_freqs_fpga[i] > new_freq)
                {
                  new_freq = mpfs_i2c_freqs_fpga[i];
                  clock_div = i;
                }
            }
        }
      else
        {
          for (uint8_t i = 0; i < MPFS_I2C_NUMBER_OF_DIVIDERS; i++)
            {
              if (frequency >= mpfs_i2c_freqs[i]
                  && mpfs_i2c_freqs[i] > new_freq)
                {
                  new_freq = mpfs_i2c_freqs[i];
                  clock_div = i;
                }
            }
        }

      if (new_freq == 0)
        {
          i2cerr("ERROR: Requested frequency %" PRIu32 " for I2C bus %" PRIu8
                 " is not supported by hardware.\n", frequency, priv->id);
          return -EINVAL;
        }

      i2cinfo("Changing I2Cbus %" PRIu8 " clock speed to %" PRIu32
              " (div %" PRIx32 ")\n", priv->id, new_freq, clock_div);

      /* Update the clock divider */

      modifyreg32(MPFS_I2C_CTRL,
                  MPFS_I2C_CTRL_CR2_MASK |
                  MPFS_I2C_CTRL_CR1_MASK |
                  MPFS_I2C_CTRL_CR0_MASK,
                  (((clock_div >> 2u) & 0x01u) << MPFS_I2C_CTRL_CR2) |
                  (((clock_div >> 1u) & 0x01u) << MPFS_I2C_CTRL_CR1) |
                  (((clock_div & 0x01u) << MPFS_I2C_CTRL_CR0)));

      priv->frequency = frequency;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a pointer to an unique
 *   instance of struct i2c_master_s. This function may be called to obtain
 *   multiple instances of the interface.
 *
 * Parameters:
 *   port          - Port number of the I2C interface to be initialized.
 *
 * Returned Value:
 *   Pointer to valid I2C device structure is returned on success.
 *   A NULL pointer is returned on failure.
 *
 ****************************************************************************/

struct i2c_master_s *mpfs_i2cbus_initialize(int port)
{
  struct mpfs_i2c_priv_s *priv;
  int ret;

  switch (port)
    {
#ifdef CONFIG_MPFS_I2C0
      case 0:
        priv = &g_mpfs_i2c0_lo_priv;
        break;
#endif /* CONFIG_MPFS_I2C0 */
#ifdef CONFIG_MPFS_I2C1
      case 1:
        priv = &g_mpfs_i2c1_lo_priv;
        break;
#endif /* CONFIG_MPFS_I2C1 */
#ifdef CONFIG_MPFS_COREI2C0
      case 0:
        priv = &g_mpfs_corei2c0_priv;
        break;
#endif
#ifdef CONFIG_MPFS_COREI2C1
      case 1:
        priv = &g_mpfs_corei2c1_priv;
        break;
#endif
#ifdef CONFIG_MPFS_COREI2C2
      case 2:
        priv = &g_mpfs_corei2c2_priv;
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

  ret = irq_attach(priv->plic_irq, mpfs_i2c_irq, priv);
  if (ret != OK)
    {
      priv->refs--;
      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  ret = mpfs_i2c_init(priv);
  if (ret != OK)
    {
      priv->refs--;
      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  nxmutex_unlock(&priv->lock);

  i2cinfo("I2C bus initialized! Handler: %" PRIxPTR "\n", (uintptr_t)priv);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: mpfs_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C bus.
 *
 * Parameters:
 *   dev           - Device structure as returned by
 *                   mpfs_i2cbus_initialize()
 *
 * Returned Value:
 *   OK is returned on success. ERROR is returned when internal reference
 *   count mismatches or dev points to invalid hardware device.
 *
 ****************************************************************************/

int mpfs_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct mpfs_i2c_priv_s *priv = (struct mpfs_i2c_priv_s *)dev;

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

  mpfs_i2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}
