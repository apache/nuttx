/****************************************************************************
 * arch/risc-v/src/bl808/bl808_i2c.c
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
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/ioctl.h>

#include "hardware/bl808_i2c.h"
#include "hardware/bl808_glb.h"
#include "hardware/bl808_mm_glb.h"
#include "riscv_internal.h"
#include "chip.h"
#include "bl808_gpio.h"
#include "bl808_i2c.h"

#if defined(CONFIG_BL808_I2C0) || defined(CONFIG_BL808_I2C1)    \
  || defined(CONFIG_BL808_I2C2) || defined(CONFIG_BL808_I2C3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XCLK 40000000 // 40 MHz crystal oscillator

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bl808_i2c_s
{
  struct i2c_ops_s *ops;

  uint8_t irq;
  uint8_t idx;
  uint8_t scl_pin;
  uint8_t sda_pin;
  int refs;
  mutex_t lock;
  sem_t sem_isr;

  struct i2c_msg_s *msgs;
  int error;
  uint8_t msgidx;
  uint32_t bytes;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

int bl808_i2c_transfer(FAR struct i2c_master_s *dev,
                       FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
int bl808_i2c_reset(FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct i2c_ops_s bl808_i2c_ops =
{
  .transfer = bl808_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = bl808_i2c_reset
#endif
};

#ifdef CONFIG_BL808_I2C0
static struct bl808_i2c_s bl808_i2c0 =
{
  .ops = &bl808_i2c_ops,
  .irq = BL808_IRQ_I2C0,
  .idx = 0,
  .scl_pin = CONFIG_BL808_I2C0_SCL,
  .sda_pin = CONFIG_BL808_I2C0_SDA,
  .refs = 0,
  .lock = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs = NULL,
  .error = OK,
  .msgidx = 0,
  .bytes = 0
};
#endif

#ifdef CONFIG_BL808_I2C1
static struct bl808_i2c_s bl808_i2c1 =
{
  .ops = &bl808_i2c_ops,
  .irq = BL808_IRQ_I2C1,
  .idx = 1,
  .scl_pin = CONFIG_BL808_I2C1_SCL,
  .sda_pin = CONFIG_BL808_I2C1_SDA,
  .refs = 0,
  .lock = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs = NULL,
  .error = OK,
  .msgidx = 0,
  .bytes = 0
};
#endif

#ifdef CONFIG_BL808_I2C2
static struct bl808_i2c_s bl808_i2c2 =
{
  .ops = &bl808_i2c_ops,
  .irq = BL808_IRQ_I2C2,
  .idx = 2,
  .scl_pin = CONFIG_BL808_I2C2_SCL,
  .sda_pin = CONFIG_BL808_I2C2_SDA,
  .refs = 0,
  .lock = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs = NULL,
  .error = OK,
  .msgidx = 0,
  .bytes = 0
};
#endif

#ifdef CONFIG_BL808_I2C3
static struct bl808_i2c_s bl808_i2c3 =
{
  .ops = &bl808_i2c_ops,
  .irq = BL808_IRQ_I2C3,
  .idx = 3,
  .scl_pin = CONFIG_BL808_I2C3_SCL,
  .sda_pin = CONFIG_BL808_I2C3_SDA,
  .refs = 0,
  .lock = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs = NULL,
  .error = OK,
  .msgidx = 0,
  .bytes = 0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_i2c_find_clock_dividers
 *
 * Description:
 *   Finds values for the clock divison registers to give the requested
 *   frequency. Tries to keep period values small for better accuracy.
 *   Warns when fails to match freq.
 *
 * Input Parameters:
 *   module_div - Return location for I2C module clock divider
 *   period_div - Return location for internal cycle count divider.
 *   freq       - Requested frequency
 *
 ****************************************************************************/

void bl808_i2c_find_clock_dividers(uint8_t *module_div,
                                   uint8_t *period_div,
                                   uint32_t freq)
{
  int best_module_div = 1;
  int best_period_div = 2;
  int min_error = abs(XCLK - freq);

  /* This function will try to keep the value that goes into the prd fields
   * as small as possible because I found through testing that this yields
   * clocks that are more accurate to the target. Writing 0 to the period
   * fields causes all transmissions to fail, so start with a divider of 2.
   */

  for (int period_iter = 2; period_iter < 256; period_iter++)
    {
      int calc_module = ((XCLK / (period_iter * 4)) / freq);
      if (calc_module > 256) /* Discard non viable values */
        {
          continue;
        }
      else if (calc_module == 0) /* Avoid division by 0 */
        {
          /* All subsequent values of period_iter will result in 0 again */

          break;
        }

      int freq_result = (XCLK / (4*period_iter)) / (calc_module);
      if (freq_result == freq)
        {
          best_period_div = period_iter;
          best_module_div = calc_module;
          min_error = 0;
          break;
        }
      else if (abs(freq_result - freq) < min_error)
        {
          best_module_div = period_iter;
          best_period_div = calc_module;
          min_error = abs(freq_result - freq);
        }
    }

  if (min_error > 0)
    {
      i2cwarn("Couldn't match requested I2C frequency. Requested %d, got %d",
              freq, XCLK / (4*best_module_div) / (best_period_div));
    }

  *module_div = best_module_div - 1;
  *period_div = best_period_div - 1;
}

/****************************************************************************
 * Name: bl808_i2c_set_freq
 *
 * Description:
 *   Set I2C module frequency to freq.
 *
 * Input Parameters:
 *   priv - Private I2C device structure
 *   freq - Requested frequency
 *
 * Returned Value:
 *   Error status. Always returns OK.
 *
 ****************************************************************************/

int bl808_i2c_set_freq(struct bl808_i2c_s *priv, uint32_t freq)
{
  uint8_t idx = priv->idx;

  uint8_t module_div;
  uint8_t period_div;
  bl808_i2c_find_clock_dividers(&module_div, &period_div, freq);

  /* Now we have our dividers and we can write config registers */

  if ((idx == 0) || (idx == 1))
    {
      /* Set clock source to xtal and set module clk division */

      modifyreg32(BL808_GLB_I2C_CFG0, I2C_CFG_CLK_DIV_MASK,
                  I2C_CFG_CLK_EN
                  | I2C_CFG_CLK_XTAL
                  | (module_div << I2C_CFG_CLK_DIV_SHIFT));
    }
  else if (idx == 2)
    {
      modifyreg32(BL808_MM_GLB_CLK_CTRL_CPU, 0,
                  CLK_CTRL_CPU_I2C_CLK_XTAL);
      modifyreg32(BL808_MM_GLB_CLK_CTRL_PERI,
                  CLK_CTRL_PERI_I2C0_DIV_MASK,
                  CLK_CTRL_PERI_I2C0_EN
                  | CLK_CTRL_PERI_I2C0_DIV_EN
                  | (module_div << CLK_CTRL_PERI_I2C0_DIV_SHIFT));
    }
  else if (idx == 3)
    {
      modifyreg32(BL808_MM_GLB_CLK_CTRL_CPU, 0,
                  CLK_CTRL_CPU_I2C_CLK_XTAL);
      modifyreg32(BL808_MM_GLB_CLK_CTRL_PERI3,
                  CLK_CTRL_PERI_I2C1_DIV_MASK,
                  CLK_CTRL_PERI_I2C1_EN
                  | CLK_CTRL_PERI_I2C1_DIV_EN
                  | (module_div << CLK_CTRL_PERI_I2C1_DIV_SHIFT));
    }

  modifyreg32(BL808_I2C_PRD_START(idx), 0xffffffff,
              period_div << I2C_PRD_PH0_SHIFT
              | period_div << I2C_PRD_PH1_SHIFT
              | period_div << I2C_PRD_PH2_SHIFT
              | period_div << I2C_PRD_PH3_SHIFT);
  modifyreg32(BL808_I2C_PRD_STOP(idx), 0xffffffff,
              period_div << I2C_PRD_PH0_SHIFT
              | period_div << I2C_PRD_PH1_SHIFT
              | period_div << I2C_PRD_PH2_SHIFT
              | period_div << I2C_PRD_PH3_SHIFT);
  modifyreg32(BL808_I2C_PRD_DATA(idx), 0xffffffff,
              period_div << I2C_PRD_PH0_SHIFT
              | period_div << I2C_PRD_PH1_SHIFT
              | period_div << I2C_PRD_PH2_SHIFT
              | period_div << I2C_PRD_PH3_SHIFT);

  return OK;
}

/****************************************************************************
 * Name: bl808_i2c_write
 *
 * Description:
 *   Writes one message into the TX FIFO.
 *
 * Input Parameters:
 *   priv - Private I2C device structure
 *
 ****************************************************************************/

void bl808_i2c_write(struct bl808_i2c_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgs[priv->msgidx];

  uint8_t count = msg->length - priv->bytes;
  if (count > 4)
    {
      count = 4; /* Only handle 4 bytes at a time */
    }

  uint32_t regval = 0;
  for (int i = 0; i < count; i++)
    {
      uint8_t byte = msg->buffer[priv->bytes + i];
      regval |= byte << (i * 8);
    }

  putreg32(regval, BL808_I2C_WDATA(priv->idx));
  priv->bytes += count;
}

/****************************************************************************
 * Name: bl808_i2c_read
 *
 * Description:
 *   Reads one message from the RX FIFO.
 *
 * Input Parameters:
 *   priv - Private I2C device structure
 *
 ****************************************************************************/

void bl808_i2c_read(struct bl808_i2c_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgs[priv->msgidx];
  uint8_t count = msg->length - priv->bytes;
  uint32_t regval = getreg32(BL808_I2C_RDATA(priv->idx));

  if (count > 4)
    {
      count = 4; /* Only handle 4 bytes at a time */
    }

  for (int i = 0; i < count; i++)
    {
      msg->buffer[priv->bytes + i] = regval & 0xff;
      regval = regval >> 8;
    }

  priv->bytes += count;
}

/****************************************************************************
 * Name: bl808_i2c_transferbytes
 *
 * Description:
 *   Determines a message should be written or read. Disables FIFO
 *   interrupts all bytes are handled.
 *
 * Input Parameters:
 *   priv - Private I2C device structure
 *
 ****************************************************************************/

void bl808_i2c_transferbytes(struct bl808_i2c_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgs[priv->msgidx];

  if (msg->flags & I2C_M_READ)
    {
      if (priv->bytes < msg->length)
        {
          bl808_i2c_read(priv);
        }
      else
        {
          modifyreg32(BL808_I2C_INT_STS(priv->idx), 0,
                      I2C_RX_FIFO_RDY_MASK);
          modifyreg32(BL808_I2C_FIFO_CONFIG_0(priv->idx), 0,
                      RX_FIFO_CLR);
        }
    }
  else
    {
      if (priv->bytes < msg->length)
        {
          bl808_i2c_write(priv);
        }
      else
        {
          modifyreg32(BL808_I2C_INT_STS(priv->idx), 0,
                      I2C_TX_FIFO_RDY_MASK);
        }
    }
}

/****************************************************************************
 * Name: bl808_i2c_start_transfer
 *
 * Description:
 *   Configures I2C module according to message, enables relevant
 *   interrupts and intitiates the transaction.
 *
 * Input Parameters:
 *   priv - Private I2C device structure
 *
 ****************************************************************************/

void bl808_i2c_start_transfer(struct bl808_i2c_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgs[priv->msgidx];

  /* Set frequency and slave address */

  bl808_i2c_set_freq(priv, msg->frequency);
  modifyreg32(BL808_I2C_CONFIG(priv->idx), I2C_SLV_ADDR_MASK,
              (msg->addr << I2C_SLV_ADDR_SHIFT));

  modifyreg32(BL808_I2C_FIFO_CONFIG_0(priv->idx), 0,
              TX_FIFO_CLR | RX_FIFO_CLR);
  modifyreg32(BL808_I2C_CONFIG(priv->idx),
              I2C_PKT_LEN_MASK,
              ((msg->length - 1) << I2C_PKT_LEN_SHIFT));

  modifyreg32(BL808_I2C_INT_STS(priv->idx),
              I2C_END_MASK | I2C_NAK_MASK
              | I2C_ARB_MASK
              | I2C_FIFO_ERROR_MASK, 0);

  /* Unmask relevant interrupts */

  if (msg->flags & I2C_M_READ)
    {
      modifyreg32(BL808_I2C_INT_STS(priv->idx),
                  I2C_RX_FIFO_RDY_MASK, 0);
    }
  else
    {
      modifyreg32(BL808_I2C_INT_STS(priv->idx),
                  I2C_TX_FIFO_RDY_MASK, 0);
    }

  modifyreg32(BL808_I2C_CONFIG(priv->idx), 0,
              I2C_M_EN);
}

/****************************************************************************
 * Name: bl808_transfer_finished
 *
 * Description:
 *   Disables I2C and clears FIFOs and interrupts.
 *
 * Input Parameters:
 *   priv - Private I2C device structure
 *
 ****************************************************************************/

void bl808_i2c_transfer_finished(struct bl808_i2c_s *priv)
{
  /* Disable i2c */

  modifyreg32(BL808_I2C_CONFIG(priv->idx), I2C_M_EN, 0);

  /* Clear FIFOs */

  modifyreg32(BL808_I2C_FIFO_CONFIG_0(priv->idx), 0,
              TX_FIFO_CLR | RX_FIFO_CLR);

  /* Mask all interrupts */

  modifyreg32(BL808_I2C_INT_STS(priv->idx), 0,
              I2C_END_MASK | I2C_TX_FIFO_RDY_MASK
              | I2C_RX_FIFO_RDY_MASK | I2C_NAK_MASK
              | I2C_ARB_MASK | I2C_FIFO_ERROR_MASK);

  /* Clear interrupts */

  modifyreg32(BL808_I2C_INT_STS(priv->idx), 0,
              I2C_END_CLR | I2C_NAK_CLR | I2C_ARB_CLR);

  nxsem_post(&priv->sem_isr);
}

/****************************************************************************
 * Name: i2c_interrupt
 *
 * Description:
 *   I2C interrupt handler. Handles transfer-related errors, as well as
 *   loading or reading from FIFOs when ready.
 *
 * Returned Value:
 *   Error status. Returns -EIO for transfer errors, OK otherwise.
 *
 ****************************************************************************/

int i2c_interrupt(int irq, void *context, void *arg)
{
  struct bl808_i2c_s *priv = (struct bl808_i2c_s *)arg;
  uint32_t int_status = getreg32(BL808_I2C_INT_STS(priv->idx));

  if (int_status & I2C_ARB_INT)
    {
      bl808_i2c_transfer_finished(priv);
      i2cerr("Bus arbitration error\n");
      priv->error = -EIO;
      return -EIO;
    }
  else if (int_status & I2C_FIFO_ERROR_INT)
    {
      bl808_i2c_transfer_finished(priv);
      i2cerr("FIFO error");
      priv->error = -EIO;
      return -EIO;
    }
  else if (int_status & I2C_NAK_INT)
    {
      bl808_i2c_transfer_finished(priv);
      i2cwarn("Transfer not acknowledged\n");
      priv->error = -EIO;
      return -EIO;
    }
  else if (int_status & I2C_RX_FIFO_RDY_INT)
    {
      bl808_i2c_transferbytes(priv);
      return OK;
    }
  else if (int_status & I2C_END_INT)
    {
      bl808_i2c_transfer_finished(priv);
      return OK;
    }
  else if (int_status & I2C_TX_FIFO_RDY_INT)
    {
      bl808_i2c_transferbytes(priv);
      return OK;
    }

  /* Should not get here */

  i2cerr("Unidentified interrupt\n");
  return -EIO;
}

/* Driver methods */

/****************************************************************************
 * Name: bl808_i2c_transfer
 *
 * Description:
 *   Method accessed by the I2C master driver. Configures hardware for
 *   each message to be transferred, then initiates each transfer.
 *
 * Input Parameters:
 *   dev   - Public I2C device structure
 *   msgs  - Message array to transfer
 *   count - Number of messages to transfer
 *
 * Returned Value:
 *   Error status. Returns errors that may propagate from nxmutex_lock()
 *   or -EIO. Otherwise returns OK.
 *
 ****************************************************************************/

int bl808_i2c_transfer(FAR struct i2c_master_s *dev,
                       struct i2c_msg_s *msgs, int count)
{
  struct bl808_i2c_s *priv = (struct bl808_i2c_s *)dev;
  int ret;
  priv->msgs = msgs;
  priv->error = OK;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      i2cerr("Lock error\n");
      return ret;
    }

  modifyreg32(BL808_I2C_CONFIG(priv->idx),
              I2C_SUB_ADDR_EN, 0);

  for (int idx = 0; idx < count; idx++)
    {
      priv->bytes = 0;

      /* Handle message flags */

      if (msgs[idx].flags & I2C_M_READ)
        {
          modifyreg32(BL808_I2C_CONFIG(priv->idx),
                      0, I2C_DIR_R);
        }
      else
        {
          modifyreg32(BL808_I2C_CONFIG(priv->idx),
                      I2C_DIR_R, 0);
        }

      if (msgs[idx].flags & I2C_M_TEN)
        {
          modifyreg32(BL808_I2C_CONFIG(priv->idx),
                      0, I2C_10B_ADDR_EN);
        }
      else
        {
          modifyreg32(BL808_I2C_CONFIG(priv->idx),
                      I2C_10B_ADDR_EN, 0);
        }

      if (msgs[idx].flags & I2C_M_NOSTOP)
        {
          if (msgs[idx].length > 4)
            {
              return -EIO;
            }

          modifyreg32(BL808_I2C_CONFIG(priv->idx), I2C_SUB_ADDR_LEN_MASK,
                      (I2C_SUB_ADDR_EN) | ((msgs[idx].length - 1)
                                           << I2C_SUB_ADDR_LEN_SHIFT));

          uint32_t subaddr_regval = 0;
          for (uint8_t subaddr_idx = 0;
               subaddr_idx < msgs[idx].length;
               subaddr_idx++)
            {
              subaddr_regval |= msgs[idx].buffer[subaddr_idx]
                << (subaddr_idx * 8);
            }

          putreg32(subaddr_regval, BL808_I2C_SUB_ADDR(priv->idx));

          continue;
        }

      priv->msgidx = idx;

      bl808_i2c_start_transfer(priv);

      ret = nxsem_wait_uninterruptible(&priv->sem_isr);
      if (ret < 0)
        {
          i2cerr("Transfer error\n");
          return ret;
        }
    }

  /* If something went wrong with the transfer (e.g. NAK),
   * the interrupt will have set priv->error.
   */

  ret = priv->error;

  nxmutex_unlock(&priv->lock);
  return ret;
}

#ifdef CONFIG_I2C_RESET

/****************************************************************************
 * Name: bl808_i2c_reset
 *
 * Description:
 *   Method accessed by the I2C master driver. Calls transfer_finished to
 *   clear FIFOs and interrupts.
 *
 * Input Parameters:
 *   dev - Public I2C device structure
 *
 * Returned Value:
 *   Error status. Always returns OK.
 *
 ****************************************************************************/

int bl808_i2c_reset(struct i2c_master_s *dev)
{
  struct bl808_i2c_s *priv = (struct bl808_i2c_s *)dev;
  bl808_i2c_transfer_finished(priv);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_master_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameter:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *bl808_i2cbus_initialize(int port)
{
  struct bl808_i2c_s *priv;
  gpio_pinattr_t gpio_attr = GPIO_INPUT | GPIO_PULLUP;

  switch (port)
    {
#ifdef CONFIG_BL808_I2C0
    case 0:
      priv = &bl808_i2c0;
      gpio_attr |= GPIO_FUNC_I2C0;
      break;
#endif

#ifdef CONFIG_BL808_I2C1
    case 1:
      priv = &bl808_i2c1;
      gpio_attr |= GPIO_FUNC_I2C1;
      break;
#endif

#ifdef CONFIG_BL808_I2C2
    case 2:
      priv = &bl808_i2c2;
      gpio_attr |= GPIO_FUNC_I2C2;
      break;
#endif

#ifdef CONFIG_BL808_I2C3
    case 3:
      priv = &bl808_i2c3;
      gpio_attr |= GPIO_FUNC_I2C3;
      break;
#endif

    default:
      i2cerr("Invalid port number\n");
      return NULL;
    }

  nxmutex_lock(&priv->lock);
  priv->refs++;
  if (priv->refs > 1)
    {
      nxmutex_unlock(&priv->lock);
      return (struct i2c_master_s *)priv;
    }

  /* Initialize GPIO */

  bl808_configgpio(priv->scl_pin, gpio_attr);
  bl808_configgpio(priv->sda_pin, gpio_attr);

  /* Enable interrupt */

  int ret = irq_attach(priv->irq, i2c_interrupt, (void *)priv);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }
  else
    {
      i2cerr("Error attaching ISR\n");
    }

  nxmutex_unlock(&priv->lock);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: bl808_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port.
 *
 * Input Parameter:
 *   Device structure as returned by the i2c_i2cbus_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int bl808_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct bl808_i2c_s *priv = (struct bl808_i2c_s *)dev;

  if (priv->refs == 0)
    {
      i2cerr("Attempt to deinit bus with no references\n");
      return -EIO;
    }

  nxmutex_lock(&priv->lock);
  priv->refs--;
  if (priv->refs > 0)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  irq_detach(priv->irq);
  up_disable_irq(priv->irq);
  modifyreg32(BL808_I2C_INT_STS(priv->idx), 0,
              I2C_END_MASK
              | I2C_TX_FIFO_RDY_MASK
              | I2C_RX_FIFO_RDY_MASK
              | I2C_NAK_MASK
              | I2C_ARB_MASK
              | I2C_FIFO_ERROR_MASK);

  nxmutex_unlock(&priv->lock);
  return OK;
}

#endif
