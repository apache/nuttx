/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_i2c.c
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include "arm64_arch.h"
#include "arm64_gic.h"
#include "bcm2711_gpio.h"
#include "bcm2711_i2c.h"
#include "chip.h"
#include "hardware/bcm2711_bsc.h"

#if defined(CONFIG_BCM2711_I2C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default bus frequency at 400kbps. */

#define I2C_DEFAULT_FREQUENCY 400000

/* Core clock nominal frequency in Hz */

#define CORE_CLOCK_FREQUENCY 150000000

/* Get divisor for desired I2C bus frequency */

#define CLK_DIVISOR(freq) (CORE_CLOCK_FREQUENCY / (freq))

/* The FIFO buffer length of the I2C interfaces */

#define FIFO_DEPTH 16

/* Logic for determining which alternate function should be used to achieve
 * the user selected I2C pins for each interface
 */

#ifdef CONFIG_BCM2711_I2C0

#if CONFIG_BCM2711_I2C0_SDA == 0 && CONFIG_BCM2711_I2C0_SCL == 1
#define BCM2711_I2C0_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_I2C0_SDA == 28 && CONFIG_BCM2711_I2C0_SCL == 29
#define BCM2711_I2C0_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_I2C0_SDA == 44 && CONFIG_BCM2711_I2C0_SCL == 45
#define BCM2711_I2C0_ALT BCM_GPIO_FUNC1
#elif CONFIG_BCM2711_I2C0_SDA == 46 && CONFIG_BCM2711_I2C0_SCL == 47
#error "BCM2711 datasheet does not declare the alternate function needed for this pin pair."
#else
#error "Invalid I2C0 pin selections. Valid SDA/SCL pairs are 0/1,28/29,44/45"
#endif

#endif /* CONFIG_BCM2711_I2C0 */

#ifdef CONFIG_BCM2711_I2C1

#if CONFIG_BCM2711_I2C1_SDA == 2 && CONFIG_BCM2711_I2C1_SCL == 3
#define BCM2711_I2C1_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_I2C1_SDA == 44 && CONFIG_BCM2711_I2C1_SCL == 45
#define BCM2711_I2C1_ALT BCM_GPIO_FUNC2
#else
#error "Invalid I2C1 pin selections. Valid SDA/SCL pairs are 2/3,44/45"
#endif

#endif /* CONFIG_BCM2711_I2C1 */

#ifdef CONFIG_BCM2711_I2C3

#if CONFIG_BCM2711_I2C3_SDA == 4 && CONFIG_BCM2711_I2C3_SCL == 5
#define BCM2711_I2C3_ALT BCM_GPIO_FUNC5
#elif CONFIG_BCM2711_I2C3_SDA == 2 && CONFIG_BCM2711_I2C3_SCL == 3
#define BCM2711_I2C3_ALT BCM_GPIO_FUNC5
#else
#error "Invalid I2C3 pin selections. Valid SDA/SCL pairs are 4/5,2/3"
#endif

#endif /* CONFIG_BCM2711_I2C3 */

#ifdef CONFIG_BCM2711_I2C4

#if CONFIG_BCM2711_I2C4_SDA == 8 && CONFIG_BCM2711_I2C4_SCL == 9
#define BCM2711_I2C4_ALT BCM_GPIO_FUNC5
#elif CONFIG_BCM2711_I2C4_SDA == 6 && CONFIG_BCM2711_I2C4_SCL == 7
#define BCM2711_I2C4_ALT BCM_GPIO_FUNC5
#else
#error "Invalid I2C4 pin selections. Valid SDA/SCL pairs are 8/9,6/7"
#endif

#endif /* CONFIG_BCM2711_I2C4 */

#ifdef CONFIG_BCM2711_I2C5

#if CONFIG_BCM2711_I2C5_SDA == 12 && CONFIG_BCM2711_I2C5_SCL == 13
#define BCM2711_I2C5_ALT BCM_GPIO_FUNC5
#elif CONFIG_BCM2711_I2C5_SDA == 10 && CONFIG_BCM2711_I2C5_SCL == 11
#define BCM2711_I2C5_ALT BCM_GPIO_FUNC5
#else
#error "Invalid I2C5 pin selections. Valid SDA/SCL pairs are 12/13,10/11"
#endif

#endif /* CONFIG_BCM2711_I2C5 */

#ifdef CONFIG_BCM2711_I2C6

#if CONFIG_BCM2711_I2C6_SDA == 22 && CONFIG_BCM2711_I2C6_SCL == 23
#define BCM2711_I2C6_ALT BCM_GPIO_FUNC5
#elif CONFIG_BCM2711_I2C6_SDA == 0 && CONFIG_BCM2711_I2C6_SCL == 1
#define BCM2711_I2C6_ALT BCM_GPIO_FUNC5
#else
#error "Invalid I2C6 pin selections. Valid SDA/SCL pairs are 12/13,10/11"
#endif

/* Verify that no two interfaces have overlapping pin pairs. This is only
 * possible with some interfaces.
 */

/* 0 and 1 pins are common between I2C6 and I2C0 */

#if defined(CONFIG_BCM2711_I2C6) && defined(CONFIG_BCM2711_I2C0)
#if CONFIG_BCM2711_I2C6_SDA == CONFIG_BCM2711_I2C0_SDA
#error "Same pin pair selected for I2C6 and I2C0."
#endif
#endif

/* 2 and 3 pins are common between I2C1 and I2C3 */

#if defined(CONFIG_BCM2711_I2C1) && defined(CONFIG_BCM2711_I2C3)
#if CONFIG_BCM2711_I2C1_SDA == CONFIG_BCM2711_I2C3_SDA
#error "Same pin pair selected for I2C1 and I2C3."
#endif
#endif

#endif /* CONFIG_BCM2711_I2C6 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* BCM2711 I2C device. */

struct bcm2711_i2cdev_s
{
  struct i2c_master_s dev;      /* Generic I2C device */
  mutex_t lock;                 /* Exclusive access */
  sem_t wait;                   /* Wait for transfer completion */
  uint32_t base;                /* Base address of I2C interface */
  uint32_t frequency;           /* I2C bus frequency */
  uint32_t sda;                 /* SDA GPIO number */
  uint32_t scl;                 /* SCL GPIO number */
  enum bcm2711_gpio_func_e alt; /* Alternate func. to use pins as I2C */
  int err;                      /* Error status of transfers */
  int refs;                     /* Reference count */
  uint8_t port;                 /* Port number */
  bool done;                    /* Transfer had DONE condition. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void bcm2711_i2c_clearfifos(struct bcm2711_i2cdev_s *priv);
static void bcm2711_i2c_setfrequency(struct bcm2711_i2cdev_s *priv,
                                     uint32_t frequency);
static void bcm2711_i2c_setaddr(struct bcm2711_i2cdev_s *priv,
                                uint16_t addr);
static void bcm2711_i2c_starttransfer(struct bcm2711_i2cdev_s *priv);
static bool bcm2711_i2c_transferactive(struct bcm2711_i2cdev_s *priv);
static int bcm2711_i2c_interrupted(struct bcm2711_i2cdev_s *priv);
static void bcm2711_i2c_disable(struct bcm2711_i2cdev_s *priv);
static void bcm2711_i2c_enable(struct bcm2711_i2cdev_s *priv);
static size_t bcm2711_i2c_drainrxfifo(struct bcm2711_i2cdev_s *priv,
                                      FAR uint8_t *buf, size_t n);
static size_t bcm2711_i2c_filltxfifo(struct bcm2711_i2cdev_s *priv,
                                     FAR uint8_t *buf, size_t n);
static int bcm2711_i2c_send(struct bcm2711_i2cdev_s *priv, FAR void *buf,
                            size_t n);
static int bcm2711_i2c_receive(struct bcm2711_i2cdev_s *priv, FAR void *buf,
                               size_t n);
static int bcm2711_i2c_secondary_handler(struct bcm2711_i2cdev_s *priv);

static int bcm2711_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
  #warning "Not implemented"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* True if the IRQ for all I2C interrupts has been set up. */

static bool g_i2c_irqinit = false;

/* The number of I2C interfaces currently initialized. */

static uint8_t g_i2c_devsinit = 0;

/* I2C operations for BCM2711 I2C interfaces. */

struct i2c_ops_s bcm2711_i2c_ops =
{
  .transfer = bcm2711_i2c_transfer,
};

#ifdef CONFIG_BCM2711_I2C0
static struct bcm2711_i2cdev_s g_i2c0dev =
{
  .base = BCM_BSC0,
  .sda = CONFIG_BCM2711_I2C0_SDA,
  .scl = CONFIG_BCM2711_I2C0_SCL,
  .alt = BCM2711_I2C0_ALT,
  .lock = NXMUTEX_INITIALIZER,
  .wait = SEM_INITIALIZER(0),
  .port = 0,
  .refs = 0,
  .err = 0,
  .done = false,
};
#endif /* CONFIG_BCM2711_I2C0 */

#ifdef CONFIG_BCM2711_I2C1
static struct bcm2711_i2cdev_s g_i2c1dev =
{
  .base = BCM_BSC1,
  .sda = CONFIG_BCM2711_I2C1_SDA,
  .scl = CONFIG_BCM2711_I2C1_SCL,
  .alt = BCM2711_I2C1_ALT,
  .lock = NXMUTEX_INITIALIZER,
  .wait = SEM_INITIALIZER(0),
  .port = 1,
  .refs = 0,
  .err = 0,
  .done = false,
};
#endif /* CONFIG_BCM2711_I2C1 */

#ifdef CONFIG_BCM2711_I2C3
static struct bcm2711_i2cdev_s g_i2c3dev =
{
  .base = BCM_BSC3,
  .sda = CONFIG_BCM2711_I2C3_SDA,
  .scl = CONFIG_BCM2711_I2C3_SCL,
  .alt = BCM2711_I2C3_ALT,
  .lock = NXMUTEX_INITIALIZER,
  .wait = SEM_INITIALIZER(0),
  .port = 3,
  .refs = 0,
  .err = 0,
  .done = false,
};
#endif /* CONFIG_BCM2711_I2C3 */

#ifdef CONFIG_BCM2711_I2C4
static struct bcm2711_i2cdev_s g_i2c4dev =
{
  .base = BCM_BSC4,
  .sda = CONFIG_BCM2711_I2C4_SDA,
  .scl = CONFIG_BCM2711_I2C4_SCL,
  .alt = BCM2711_I2C4_ALT,
  .lock = NXMUTEX_INITIALIZER,
  .wait = SEM_INITIALIZER(0),
  .port = 4,
  .refs = 0,
  .err = 0,
  .done = false,
};
#endif /* CONFIG_BCM2711_I2C4 */

#ifdef CONFIG_BCM2711_I2C5
static struct bcm2711_i2cdev_s g_i2c5dev =
{
  .base = BCM_BSC5,
  .sda = CONFIG_BCM2711_I2C5_SDA,
  .scl = CONFIG_BCM2711_I2C5_SCL,
  .alt = BCM2711_I2C5_ALT,
  .lock = NXMUTEX_INITIALIZER,
  .wait = SEM_INITIALIZER(0),
  .port = 5,
  .refs = 0,
  .err = 0,
  .done = false,
};
#endif /* CONFIG_BCM2711_I2C5 */

#ifdef CONFIG_BCM2711_I2C6
static struct bcm2711_i2cdev_s g_i2c6dev =
{
  .base = BCM_BSC6,
  .sda = CONFIG_BCM2711_I2C6_SDA,
  .scl = CONFIG_BCM2711_I2C6_SCL,
  .alt = BCM2711_I2C6_ALT,
  .lock = NXMUTEX_INITIALIZER,
  .wait = SEM_INITIALIZER(0),
  .port = 6,
  .refs = 0,
  .err = 0,
  .done = false,
};
#endif /* CONFIG_BCM2711_I2C6 */

/* I2C interfaces */

static struct bcm2711_i2cdev_s *g_i2c_devices[BCM_BSCS_NUM] =
{
#ifdef CONFIG_BCM2711_I2C0
  [0] = &g_i2c0dev,
#else
  [0] = NULL,
#endif /* CONFIG_BCM2711_I2C0 */

#ifdef CONFIG_BCM2711_I2C1
  [1] = &g_i2c1dev,
#else
  [1] = NULL,
#endif /* CONFIG_BCM2711_I2C1 */

  [2] = NULL, /* I2C2 is for HDMI */

#ifdef CONFIG_BCM2711_I2C3
  [3] = &g_i2c3dev,
#else
  [3] = NULL,
#endif /* CONFIG_BCM2711_I2C3 */

#ifdef CONFIG_BCM2711_I2C4
  [4] = &g_i2c4dev,
#else
  [4] = NULL,
#endif /* CONFIG_BCM2711_I2C4 */

#ifdef CONFIG_BCM2711_I2C5
  [5] = &g_i2c5dev,
#else
  [5] = NULL,
#endif /* CONFIG_BCM2711_I2C5 */

#ifdef CONFIG_BCM2711_I2C6
  [6] = &g_i2c6dev,
#else
  [6] = NULL,
#endif /* CONFIG_BCM2711_I2C6 */

  [7] = NULL, /* I2C7 is for HDMI */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_i2c_clearfifos
 *
 * Description:
 *   Clear the FIFOs of the I2C interface.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to clear the FIFOs of.
 *
 ****************************************************************************/

static void bcm2711_i2c_clearfifos(struct bcm2711_i2cdev_s *priv)
{
  modreg32(BCM_BSC_C_CLEAR, BCM_BSC_C_CLEAR, BCM_BSC_C(priv->base));
}

/****************************************************************************
 * Name: bcm2711_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to set the frequency of.
 *     frequency - The new frequency.
 *
 ****************************************************************************/

static void bcm2711_i2c_setfrequency(struct bcm2711_i2cdev_s *priv,
                                     uint32_t frequency)
{
  putreg32(CLK_DIVISOR(frequency), BCM_BSC_DIV(priv->base));
  priv->frequency = frequency;
}

/****************************************************************************
 * Name: bcm2711_i2c_setaddr
 *
 * Description:
 *   Set the slave address for the next transfer.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to set the slave address on.
 *     addr - The slave address.
 *
 ****************************************************************************/

static void bcm2711_i2c_setaddr(struct bcm2711_i2cdev_s *priv, uint16_t addr)
{
  /* TODO: handle 10-bit addresses */

  putreg32(addr & 0x7f, BCM_BSC_A(priv->base));
}

/****************************************************************************
 * Name: bcm2711_i2c_starttransfer
 *
 * Description:
 *   Start the next transfer.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to start the transfer on.
 *
 ****************************************************************************/

static void bcm2711_i2c_starttransfer(struct bcm2711_i2cdev_s *priv)
{
  modreg32(BCM_BSC_C_ST, BCM_BSC_C_ST, BCM_BSC_C(priv->base));
}

/****************************************************************************
 * Name: bcm2711_i2c_starttransfer
 *
 * Description:
 *   Check the status of a transfer.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to check the transfer status of.
 *
 * Returns: True if the transfer is active, false otherwise.
 ****************************************************************************/

static bool bcm2711_i2c_transferactive(struct bcm2711_i2cdev_s *priv)
{
  return getreg32(BCM_BSC_S(priv->base)) & BCM_BSC_S_TA;
}

/****************************************************************************
 * Name: bcm2711_i2c_interrupted
 *
 * Description:
 *   Checks if the I2C device has a pending interrupt.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to check for a pending interrupt.
 *
 * Returns:
 *     0 if no interrupt pending, non-zero if interrupt pending.
 *
 ****************************************************************************/

static int bcm2711_i2c_interrupted(struct bcm2711_i2cdev_s *priv)
{
  return getreg32(BCM_BSC_S(priv->base)) &
         (BCM_BSC_S_DONE | BCM_BSC_S_RXR | BCM_BSC_S_TXW);
}

/****************************************************************************
 * Name: bcm2711_i2c_disable
 *
 * Description:
 *   Disable the I2C interface.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to disable.
 *
 ****************************************************************************/

static void bcm2711_i2c_disable(struct bcm2711_i2cdev_s *priv)
{
  /* Disable interface */

  modreg32(0, BCM_BSC_C_I2CEN, BCM_BSC_C(priv->base));

  /* Disable interrupts */

  modreg32(0, (BCM_BSC_C_INTD | BCM_BSC_C_INTR | BCM_BSC_C_INTT),
           BCM_BSC_C(priv->base));
}

/****************************************************************************
 * Name: bcm2711_i2c_enable
 *
 * Description:
 *   Enable the I2C interface.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to enable.
 *
 ****************************************************************************/

static void bcm2711_i2c_enable(struct bcm2711_i2cdev_s *priv)
{
  /* Enable interrupts */

  modreg32((BCM_BSC_C_INTD | BCM_BSC_C_INTR | BCM_BSC_C_INTT),
           (BCM_BSC_C_INTD | BCM_BSC_C_INTR | BCM_BSC_C_INTT),
           BCM_BSC_C(priv->base));

  /* Enable interface */

  modreg32(BCM_BSC_C_I2CEN, BCM_BSC_C_I2CEN, BCM_BSC_C(priv->base));
}

/****************************************************************************
 * Name: bcm2711_i2c_drainrxfifo
 *
 * Description:
 *   Drain the RX FIFO into the receive message buffer of the I2C device.
 *   NOTE: this function assumes that `buf` is always a valid pointer.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to receive on.
 *     buf - The buffer to receive into
 *     n - The buffer size in bytes
 *
 * Returns the number of bytes that were read from the RX FIFO.
 ****************************************************************************/

static size_t bcm2711_i2c_drainrxfifo(struct bcm2711_i2cdev_s *priv,
                                      FAR uint8_t *buf, size_t n)
{
  uint8_t to_read = n < FIFO_DEPTH ? n : FIFO_DEPTH;
  uint8_t i = 0;

  /* While the RX FIFO contains data to be received, drain it into the
   * message buffer. Do not read more than the `to_read` maximum to avoid
   * overflowing the message buffer.
   */

  while ((i < to_read) && (getreg32(BCM_BSC_S(priv->base)) & BCM_BSC_S_RXD))
    {
      *buf = getreg32(BCM_BSC_FIFO(priv->base)) & 0xff;
      buf++;
      i++;
    }

  return i;
}

/****************************************************************************
 * Name: bcm2711_i2c_receive
 *
 * Description:
 *   Receive I2C data.
 *
 * Input Parameters:
 *     dev - The I2C interface to receive on.
 *     buf - The buffer to receive into
 *     n - The buffer size in bytes
 *     stop - Whether to send a stop condition at the end of the transfer.
 ****************************************************************************/

static int bcm2711_i2c_receive(struct bcm2711_i2cdev_s *priv, FAR void *buf,
                               size_t n)
{
  size_t remaining = n;
  size_t bread;
  int ret = 0;

  /* Start transfer. */

  bcm2711_i2c_starttransfer(priv);

  /* Continuously read until message has been completely read and transfer is
   * indicated as done.
   */

  while (remaining > 0 || !priv->done)
    {
      /* Wait here for interrupt handler to signal that RX FIFO has data
       * (RXR). We can then continue reading.
       */

      ret = nxsem_wait_uninterruptible(&priv->wait);
      DEBUGASSERT(ret == 0);

      /* Check for device errors. */

      if (priv->err != 0)
        {
          return priv->err;
        }

      /* Check if we are done and there's no data left to read. */

      if (priv->done && remaining == 0)
        {
          break; /* Leave loop and return */
        }

      /* The remaining message length is the current remainder minus how far
       * into the message we are. We also move forward the buffer pointer by
       * how many bytes were just read so it starts at the correct location
       * next iteration.
       *
       * We still have to read this if we received the "DONE" indicator but
       * have the tail-end of the data in the RX FIFO.
       */

      bread = bcm2711_i2c_drainrxfifo(priv, buf, remaining);
      remaining -= bread;
      buf += bread;
    }

  priv->done = false; /* Reset done indicator */
  return ret;
}

/****************************************************************************
 * Name: bcm2711_i2c_filltxfifo
 *
 * Description:
 *   Fill the TX FIFO with data to be sent.
 *   NOTE: this function assumes that `buf` is a valid pointer.
 *
 * Input Parameters:
 *     priv - The BCM2711 I2C interface to send on.
 *     buf - The buffer of data to be transmitted
 *     n - The length of `buf` in bytes
 *
 * Returns the number of bytes successfully written to the TX FIFO.
 ****************************************************************************/

static size_t bcm2711_i2c_filltxfifo(struct bcm2711_i2cdev_s *priv,
                                     FAR uint8_t *buf, size_t n)
{
  uint8_t to_send = n < FIFO_DEPTH ? n : FIFO_DEPTH;
  uint8_t i = 0;

  /* While there is data to be sent, and the TX FIFO is not full, write the
   * data to the TX FIFO. Stop when full or data stream is over.
   */

  while (i < to_send && (getreg32(BCM_BSC_S(priv->base)) & BCM_BSC_S_TXD))
    {
      putreg32(*buf & 0xff, BCM_BSC_FIFO(priv->base));
      buf++;
      i++;
    }

  return i;
}

/****************************************************************************
 * Name: bcm2711_i2c_send
 *
 * Description:
 *   Send I2C data.
 *
 * Input Parameters:
 *     dev - The I2C interface to send on.
 *     buf - The buffer to send data from.
 *     n - The size of the buffer in bytes.
 *     stop - Whether to send a stop condition at the end of the transfer.
 ****************************************************************************/

static int bcm2711_i2c_send(struct bcm2711_i2cdev_s *priv, FAR void *buf,
                            size_t n)
{
  size_t remaining = n;
  size_t bwrote;
  int ret = OK;

  DEBUGASSERT(buf != NULL || (buf == NULL && n == 0));

  while (remaining > 0 || !priv->done)
    {
      /* Write data to FIFO. The remaining message length is the total
       * remaining before, minus how far into the message we are. The `buf`
       * pointer is incremented to point to data that hasn't been sent yet.
       */

      bwrote = bcm2711_i2c_filltxfifo(priv, buf, remaining);
      remaining -= bwrote;
      buf += bwrote;

      /* Start the transfer if it's not running already */

      if (!bcm2711_i2c_transferactive(priv))
        {
          bcm2711_i2c_starttransfer(priv);
        }

      /* Here we wait for an interrupt. There are two scenarios:
       *
       * 1) If there is still message data left to write AND we receive an
       * interrupt, then we can continue on another iteration of the outer
       * do-while loop to keep writing. This interrupt was a TXW.
       *
       * 2) If there is no more message data left to write, then we need to
       * get a DONE interrupt to indicate the end of the transfer.
       */

      ret = nxsem_wait_uninterruptible(&priv->wait);
      DEBUGASSERT(ret == 0);

      /* First check for IO error because it's more important to report
       * that.
       */

      if (priv->err != 0)
        {
          return priv->err;
        }

      /* Now check if the received interrupt was a done condition. Otherwise
       * we loop again.
       */

      if (priv->done)
        {
          /* Debug assertion does not allow us to have received a done
           * interrupt if we still have data to write, as that is a logic
           * failure.
           */

          DEBUGASSERT(remaining == 0);
          break;
        }
    }

  priv->done = false; /* Reset done indicator */
  return ret;
}

/****************************************************************************
 * Name: bcm2711_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers.
 *
 * Input Parameters:
 *     dev - The I2C master interface to transfer on.
 *     msgs - The messages to transfer.
 *     count - The number of messages to transfer.
 ****************************************************************************/

static int bcm2711_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count)
{
  struct bcm2711_i2cdev_s *priv = (struct bcm2711_i2cdev_s *)dev;
  int i;
  int ret;
  bool stop = true;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(msgs != NULL);

  /* Get exclusive access before doing a transfer */

  ret = nxmutex_lock(&priv->lock);
  DEBUGASSERT(ret == 0);

  /* Perform send/receive operations for each message */

  for (i = 0; i < count; i++, msgs++)
    {
      priv->err = 0; /* No errors yet */
      priv->done = false;

      /* Configure I2C interface according to message. */

      i2cinfo("Transfer %zu bytes %s 0x%02x.", msgs->length,
              msgs->flags & I2C_M_READ ? "from" : "to", msgs->addr);

      bcm2711_i2c_disable(priv);
      bcm2711_i2c_clearfifos(priv);
      bcm2711_i2c_setfrequency(priv, msgs->frequency);
      bcm2711_i2c_setaddr(priv, msgs->addr);
      putreg32(msgs->length, BCM_BSC_DLEN(priv->base)); /* Set transfer len */
      bcm2711_i2c_enable(priv);

      /* TODO: do I need to support I2C_M_NOSTART? */

      /* TODO: Support restart condition (no stop) */

      if (msgs->flags & I2C_M_NOSTOP)
        {
          stop = false;
        }
      else
        {
          stop = true;
        }

      /* Perform the operation corresponding to whether the user wanted to
       * send or receive on this transfer.
       */

      if (msgs->flags & I2C_M_READ)
        {
          modreg32(BCM_BSC_C_READ, BCM_BSC_C_READ, BCM_BSC_C(priv->base));
          ret = bcm2711_i2c_receive(priv, msgs->buffer, msgs->length);
        }
      else
        {
          modreg32(0, BCM_BSC_C_READ, BCM_BSC_C(priv->base));
          ret = bcm2711_i2c_send(priv, msgs->buffer, msgs->length);
        }

      /* Check if there was an error during the send/receive operation and
       * return early if so.
       */

      if (ret < 0)
        {
          break;
        }

      if (priv->err != 0)
        {
          ret = priv->err;
          break;
        }
    }

  /* If our last message had a stop condition, we can safely disable this I2C
   * interface until it's used again.
   */

  if (stop)
    {
      bcm2711_i2c_disable(priv);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: bcm2711_i2c_primary_handler
 *
 * Description:
 *   Handle I2C interrupts.
 *
 * Input Parameters:
 *     irq - The IRQ number
 *     context - The interrupt context
 *     arg - The argument passed to the interrupt handler
 ****************************************************************************/

static int bcm2711_i2c_primary_handler(int irq, void *context, void *arg)
{
  int ret = 0;
  int tempret = 0;
  struct bcm2711_i2cdev_s *priv;

  /* Check all I2C interfaces for an interrupt */

  for (int i = 0; i < BCM_BSCS_NUM; i++)
    {
      priv = g_i2c_devices[i];
      if (priv == NULL)
        {
          continue;
        }

      /* If interface had an interrupt triggered, call its handler. */

      if (bcm2711_i2c_interrupted(priv))
        {
          tempret = bcm2711_i2c_secondary_handler(priv);

          /* If there was an error, record it. Otherwise don't overwrite the
           * ret value, it may contain an error from a previous iteration
           * that we still need to report.
           */

          if (tempret < 0)
            {
              ret = tempret;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bcm2711_i2c_secondary_handler
 *
 * Description:
 *   Handle I2C interrupts on a per-device basis.
 *
 * Input Parameters:
 *     priv - The I2C interface device which has an interrupt triggered.
 ****************************************************************************/

static int bcm2711_i2c_secondary_handler(struct bcm2711_i2cdev_s *priv)
{
  int ret = OK;
  uint32_t status;
  uint32_t status_addr;
  bool post_sem = false;

  /* Get interrupt status for this device. */

  status_addr = BCM_BSC_S(priv->base);
  status = getreg32(status_addr);

  /* Decide what to do with the status */

  /* There was an ACK error */

  if (status & BCM_BSC_S_ERR)
    {
      modreg32(BCM_BSC_S_ERR, BCM_BSC_S_ERR,
               status_addr); /* Acknowledge err */
      priv->err = -EIO;
      ret = -EIO;
      post_sem = true;
    }

  /* There was a clock stretch timeout */

  if (status & BCM_BSC_S_CLKT)
    {
      modreg32(BCM_BSC_S_CLKT, BCM_BSC_S_CLKT,
               status_addr); /* Acknowledge err */
      priv->err = -EIO;
      ret = -EIO;
      post_sem = true;
    }

  /* RX FIFO needs reading */

  if (status & BCM_BSC_S_RXR)
    {
      post_sem = true;
    }

  /* TX FIFO needs writing */

  if (status & BCM_BSC_S_TXW)
    {
      post_sem = true;
    }

  /* Transfer is done */

  if (status & BCM_BSC_S_DONE)
    {
      priv->done = true;
      modreg32(BCM_BSC_S_DONE, BCM_BSC_S_DONE, status_addr);
      post_sem = true;
    }

  if (post_sem)
    {
      nxsem_post(&priv->wait);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device for the BCM2711.
 *
 * Input parameters:
 *     port - The bus number for the I2C interface.
 *
 ****************************************************************************/

struct i2c_master_s *bcm2711_i2cbus_initialize(int port)
{
  int ret;
  struct bcm2711_i2cdev_s *priv;

  DEBUGASSERT(0 <= port && port <= 6); /* Only up to 6 is user controllable */

  /* Initialize selected port */

  switch (port)
    {
#if defined(CONFIG_BCM2711_I2C0)
    case 0:
      priv = &g_i2c0dev;
      break;
#endif
#if defined(CONFIG_BCM2711_I2C1)
    case 1:
      priv = &g_i2c1dev;
      break;
#endif
#if defined(CONFIG_BCM2711_I2C3)
    case 3:
      priv = &g_i2c3dev;
      break;
#endif
#if defined(CONFIG_BCM2711_I2C4)
    case 4:
      priv = &g_i2c4dev;
      break;
#endif
#if defined(CONFIG_BCM2711_I2C5)
    case 5:
      priv = &g_i2c5dev;
      break;
#endif
#if defined(CONFIG_BCM2711_I2C6)
    case 6:
      priv = &g_i2c6dev;
      break;
#endif
    default:
      i2cerr("Port %d is unsupported.\n", port);
      return NULL;
    }

  /* Exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      i2cerr("Couldn't lock mutex for I2C%u\n", port);
      return NULL;
    }

  /* Test for previous initialization. If already initialized, nothing more
   * needs to be done.
   */

  if (priv->refs > 0)
    {
      priv->refs++;
      nxmutex_unlock(&priv->lock);
      return &priv->dev;
    }

  /* Not yet initialized, little more work to do. */

  i2cinfo("Initializing I2C%u\n", port);

  priv->dev.ops = &bcm2711_i2c_ops;

  bcm2711_gpio_set_pulls(priv->sda, false, false);
  bcm2711_gpio_set_pulls(priv->scl, false, false);
  bcm2711_gpio_set_func(priv->sda, priv->alt);
  bcm2711_gpio_set_func(priv->scl, priv->alt);

  bcm2711_i2c_disable(priv); /* Only enabled when used. */
  bcm2711_i2c_setfrequency(priv, I2C_DEFAULT_FREQUENCY);

  /* Attach interrupt handler if it hasn't been already */

  if (!g_i2c_irqinit)
    {
      ret = irq_attach(BCM_IRQ_VC_I2C, bcm2711_i2c_primary_handler, NULL);
      if (ret < 0)
        {
          i2cerr("Could not attach interrupt handler for port %d: %d\n",
                 port, ret);
          return NULL;
        }

      i2cinfo("I2C interrupt handler attached\n");

      /* Enable interrupt handler */

      arm64_gic_irq_set_priority(BCM_IRQ_VC_I2C, 0, IRQ_TYPE_EDGE);
      up_enable_irq(BCM_IRQ_VC_I2C);
      g_i2c_irqinit = true; /* Mark IRQ handler as initialized */
      i2cinfo("I2C IRQ enabled\n");
    }

  /* Another device initialized */

  priv->refs++;
  g_i2c_devsinit++;
  nxmutex_unlock(&priv->lock);
  return &priv->dev;
}

/****************************************************************************
 * Name: bcm2711_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C device on the BCM2711.
 *
 * Input parameters;
 *     dev - The device to uninitialize.
 *
 ****************************************************************************/

int bcm2711_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  int ret = 0;
  struct bcm2711_i2cdev_s *priv = (struct bcm2711_i2cdev_s *)dev;

  if (priv->refs == 0)
    {
      return -1;
    }

  /* If there are still references to the device, just decrement references.
   */

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  /* This was the last reference to the I2C device. */

  bcm2711_i2c_disable(priv);
  g_i2c_devsinit--; /* One less device initialized */

  /* If there are no more I2C devices initialized, turn off interrupts. */

  if (g_i2c_devsinit == 0)
    {
      up_disable_irq(BCM_IRQ_VC_I2C);
      irq_detach(BCM_IRQ_VC_I2C);
      g_i2c_irqinit = false;
      i2cinfo("Detached I2C interrupt handler and disabled IRQ.\n");
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

#endif // defined(CONFIG_BCM2711_I2C)
