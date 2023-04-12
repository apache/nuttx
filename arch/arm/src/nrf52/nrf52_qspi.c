/****************************************************************************
 * arch/arm/src/nrf52/nrf52_qspi.c
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

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <arch/board/board.h>

#include "arm_internal.h"

#include "hardware/nrf52_qspi.h"

#include "nrf52_gpio.h"
#include "nrf52_qspi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Instruction handled by the NRF52 QSPI peripheral */

#define QSPI_SECTOR_ERASE 0x20
#define QSPI_BLOCK_ERASE  0xd8
#define QSPI_ALL_ERASE    0xc7

/* QSPI memory synchronization */

#define MEMORY_SYNC()     do { ARM_DSB(); ARM_ISB(); } while (0)

/* Ensure that the DMA buffers are word-aligned. */

#define ALIGN_SHIFT       2
#define ALIGN_MASK        3
#define ALIGN_UP(n)       (((n)+ALIGN_MASK) & ~ALIGN_MASK)
#define IS_ALIGNED(n)     (((uint32_t)(n) & ALIGN_MASK) == 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_qspidev_s
{
  struct qspi_dev_s qspi;           /* Externally visible part of the QSPI interface */
  mutex_t           lock;           /* Assures mutually exclusive access to QSPI */
  uint32_t          base;           /* QSPI base address */
  uint32_t          actual;         /* Actual clock frequency */
  uint32_t          frequency;      /* Requested clock frequency */
  bool              initialized;    /* TRUE: Controller has been initialized */
  uint8_t           intf;           /* QSPI controller number (0) */
  uint8_t           mode;           /* Mode 0,3 */
  sem_t             op_sem;         /* Block until complete */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline void nrf52_qspi_putreg(struct nrf52_qspidev_s *priv,
                                     uint32_t offset,
                                     uint32_t value);
static inline uint32_t nrf52_qspi_getreg(struct nrf52_qspidev_s *priv,
                                         uint32_t offset);
static void nrf52_qspi_cinstrdata_get(struct nrf52_qspidev_s *priv,
                                      struct qspi_cmdinfo_s *cmdinfo);
static void nrf52_qspi_cinstrdata_put(struct nrf52_qspidev_s *priv,
                                      struct qspi_cmdinfo_s *cmdinfo);

/* QSPI operations */

static int nrf52_qspi_lock(struct qspi_dev_s *dev, bool lock);

static int nrf52_qspi_lock(struct qspi_dev_s *dev, bool lock);
static uint32_t nrf52_qspi_setfrequency(struct qspi_dev_s *dev,
                                        uint32_t frequency);
static void nrf52_qspi_setmode(struct qspi_dev_s *dev,
                               enum qspi_mode_e mode);
static void nrf52_qspi_setbits(struct qspi_dev_s *dev, int nbits);
static int nrf52_qspi_command(struct qspi_dev_s *dev,
                             struct qspi_cmdinfo_s *cmdinfo);
static int nrf52_qspi_memory(struct qspi_dev_s *dev,
                             struct qspi_meminfo_s *meminfo);
static void *nrf52_qspi_alloc(struct qspi_dev_s *dev, size_t buflen);
static void nrf52_qspi_free(struct qspi_dev_s *dev, void *buffer);

static int nrf52_qspi_interrupt(int irq, void *context, void *arg);
static int nrf52_qspi_hw_initialize(struct nrf52_qspidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qspi_ops_s g_qspi_ops =
{
  .lock              = nrf52_qspi_lock,
  .setfrequency      = nrf52_qspi_setfrequency,
  .setmode           = nrf52_qspi_setmode,
  .setbits           = nrf52_qspi_setbits,
  .command           = nrf52_qspi_command,
  .memory            = nrf52_qspi_memory,
  .alloc             = nrf52_qspi_alloc,
  .free              = nrf52_qspi_free,
};

static struct nrf52_qspidev_s g_qspi0_dev =
{
  .qspi      =
  {
    .ops     = &g_qspi_ops,
  },
  .lock      = NXMUTEX_INITIALIZER,
  .op_sem    = SEM_INITIALIZER(0),
  .base      = NRF52_QSPI_BASE,
  .intf      = 0
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_qspi_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void nrf52_qspi_putreg(struct nrf52_qspidev_s *priv,
                                     uint32_t offset,
                                     uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_qspi_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t nrf52_qspi_getreg(struct nrf52_qspidev_s *priv,
                                        uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_qspi_lock
 *
 * Description:
 *   On QSPI buses where there are multiple devices, it will be necessary to
 *   lock QSPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the QSPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the QSPI is properly
 *   configured for the device.  If the QSPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock QSPI bus, false: unlock QSPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int nrf52_qspi_lock(struct qspi_dev_s *dev, bool lock)
{
  struct nrf52_qspidev_s *priv = (struct nrf52_qspidev_s *)dev;
  int ret;

  spiinfo("lock=%d\n", lock);
  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_qspi_setfrequency
 *
 * Description:
 *   Set the QSPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The QSPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t nrf52_qspi_setfrequency(struct qspi_dev_s *dev,
                                        uint32_t frequency)
{
  struct nrf52_qspidev_s *priv    = (struct nrf52_qspidev_s *)dev;
  uint32_t                sckfreq = 0;
  uint32_t                actual  = 0;
  uint32_t                regval  = 0;

  spiinfo("frequency=%ld\n", frequency);
  DEBUGASSERT(priv);

  /* Check if the requested frequency is the same as the frequency
   * selection
   */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* Get prescaler */

  if (frequency <= 2000000)
    {
      sckfreq = 15;
    }
  else if (frequency <= 32000000)
    {
      sckfreq = (32000000 / frequency) - 1;
    }
  else
    {
      sckfreq = 0;
    }

  /* Modify register */

  regval = nrf52_qspi_getreg(priv, NRF52_QSPI_IFCONFIG1_OFFSET);
  regval &= ~QSPI_IFCONFIG1_SCKFREQ_MASK;
  regval |= sckfreq << QSPI_IFCONFIG1_SCKFREQ_SHIFT;
  nrf52_qspi_putreg(priv, NRF52_QSPI_IFCONFIG1_OFFSET, regval);

  /* Calculate the new actual frequency */

  actual = 32000000 / (sckfreq + 1);

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("Frequency %ld->%ld\n", frequency, actual);

  return actual;
}

/****************************************************************************
 * Name: nrf52_qspi_setmode
 *
 * Description:
 *   Set the QSPI mode. Optional.  See enum qspi_mode_e for mode definitions.
 *   NOTE:  the NRF52 QSPI supports only modes 0 and 3.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The QSPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void nrf52_qspi_setmode(struct qspi_dev_s *dev, enum qspi_mode_e mode)
{
  struct nrf52_qspidev_s *priv    = (struct nrf52_qspidev_s *)dev;
  uint32_t                regval  = 0;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      regval = nrf52_qspi_getreg(priv, NRF52_QSPI_IFCONFIG1_OFFSET);

      switch (mode)
        {
          case QSPIDEV_MODE0:
            {
              regval |= (QSPI_IFCONFIG1_SPIMODE_0);
              break;
            }

          case QSPIDEV_MODE3:
            {
              regval |= (QSPI_IFCONFIG1_SPIMODE_3);
              break;
            }

          case QSPIDEV_MODE1:
          case QSPIDEV_MODE2:
            {
              spiinfo("unsupported mode=%d\n", mode);

              /* No break here */
            }
          default:
            {
              DEBUGASSERT(0);
              return;
            }
        }

      /* Write new mode */

      nrf52_qspi_putreg(priv, NRF52_QSPI_IFCONFIG1_OFFSET, regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: nrf52_qspi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *   NOTE:  the NRF52 QSPI only supports 8 bits, so this does nothing.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void nrf52_qspi_setbits(struct qspi_dev_s *dev, int nbits)
{
  if (nbits != 8)
    {
      spiinfo("unsupported nbits=%d\n", nbits);
      DEBUGASSERT(FALSE);
    }
}

/****************************************************************************
 * Name: nrf52_qspi_cinstrdata_get
 *
 * Description:
 *   Get command data
 *
 * Input Parameters:
 *   priv - Device state structure.
 *   cmdinfo - Describes the command transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static void nrf52_qspi_cinstrdata_get(struct nrf52_qspidev_s *priv,
                                      struct qspi_cmdinfo_s *cmdinfo)
{
  uint32_t regval = 0;
  int      i      = 0;
  uint8_t *buffer = cmdinfo->buffer;

  DEBUGASSERT(cmdinfo->buflen <= 8);

  /* Get Bytes 0-3 */

  regval = nrf52_qspi_getreg(priv, NRF52_QSPI_CINSTRDAT0_OFFSET);

  if (cmdinfo->buflen > 0)
    {
      for (i = 0; (i < 4) && (i < cmdinfo->buflen); i += 1)
        {
          buffer[i] = (regval >> (i * 8)) & 0xff;
        }
    }

  /* Write Bytes 4-7 */

  regval = nrf52_qspi_getreg(priv, NRF52_QSPI_CINSTRDAT1_OFFSET);

  if (cmdinfo->buflen > 4)
    {
      for (i = 4; (i < 8) && (i < cmdinfo->buflen); i += 1)
        {
          buffer[i] = (regval >> ((i - 4) * 8)) & 0xff;
        }
    }
}

/****************************************************************************
 * Name: nrf52_qspi_cinstrdata_put
 *
 * Description:
 *   Put command data
 *
 * Input Parameters:
 *   priv - Device state structure.
 *   cmdinfo - Describes the command transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static void nrf52_qspi_cinstrdata_put(struct nrf52_qspidev_s *priv,
                                      struct qspi_cmdinfo_s *cmdinfo)
{
  uint32_t regval = 0;
  int      i      = 0;
  uint8_t *buffer = cmdinfo->buffer;

  DEBUGASSERT(cmdinfo->buflen <= 8);

  regval = 0;

  if (cmdinfo->buflen > 0)
    {
      for (i = 0; (i < 4) && (i < cmdinfo->buflen); i += 1)
        {
          regval |= (buffer[i] << (i * 8));
        }
    }

  /* Write Bytes 0-3 */

  nrf52_qspi_putreg(priv, NRF52_QSPI_CINSTRDAT0_OFFSET, regval);

  regval = 0;
  if (cmdinfo->buflen > 4)
    {
      for (i = 4; (i < 8) && (i < cmdinfo->buflen); i += 1)
        {
          regval |= (buffer[i] << ((i - 4) * 8));
        }
    }

  /* Write Bytes 4-7 */

  nrf52_qspi_putreg(priv, NRF52_QSPI_CINSTRDAT1_OFFSET, regval);
}

/****************************************************************************
 * Name: nrf52_qspi_command
 *
 * Description:
 *   Perform one QSPI data transfer
 *
 *   TODO: long frame mode not supported
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   cmdinfo - Describes the command transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int nrf52_qspi_command(struct qspi_dev_s *dev,
                              struct qspi_cmdinfo_s *cmdinfo)
{
  struct nrf52_qspidev_s *priv   = (struct nrf52_qspidev_s *)dev;
  uint32_t                regval = 0;

  DEBUGASSERT(cmdinfo->cmd < 256);

  if (QSPICMD_ISADDRESS(cmdinfo->flags))
    {
      /* Only ERASE commands supported */

      switch (cmdinfo->cmd)
        {
          case QSPI_SECTOR_ERASE:
            {
              regval = QSPI_ERASE_SECTOR;
              break;
            }

          case QSPI_BLOCK_ERASE:
            {
              regval = QSPI_ERASE_PAGE;
              break;
            }

          case QSPI_ALL_ERASE:
            {
              regval = QSPI_ERASE_ALL;
              break;
            }

          default:
            {
              /* Not supported addressed command */

              DEBUGASSERT(0);
              return -EINVAL;
            }
        }

      /* Configure erase length */

      nrf52_qspi_putreg(priv, NRF52_QSPI_ERASE_LEN_OFFSET, regval);

      /* Configure erase address */

      regval = cmdinfo->addr;
      nrf52_qspi_putreg(priv, NRF52_QSPI_ERASE_PTR_OFFSET, regval);

      /* Start erase operation */

      nrf52_qspi_putreg(priv, NRF52_QSPI_TASKS_ERASESTART_OFFSET, 1);

      /* Wait for the READY event.
       * TODO: add timeout.
       *
       * NOTE: READ event only signals that the erase operation
       *       has been started.
       */

      nxsem_wait(&priv->op_sem);

      return OK;
    }

  if (QSPICMD_ISWRITE(cmdinfo->flags))
    {
      /* Write data to CINSTRDAT registers */

      nrf52_qspi_cinstrdata_put(priv, cmdinfo);
    }

  /* Configure custom instruction */

  regval = cmdinfo->cmd << QSPI_ADDRCONF_OPCODE_SHIFT;
  regval |= QSPI_CINSTRCONF_LENGTH(cmdinfo->buflen + 1);

  if (QSPICMD_ISWRITE(cmdinfo->flags))
    {
      /* Write request */

      regval |= QSPI_CINSTRCONF_WREN;

      /* Wait for write complete before sending command */

      regval |= QSPI_CINSTRCONF_WIPWAIT;
    }

  /* IO2 and IO3 high during transmission of custom instruction */

  regval |= QSPI_CINSTRCONF_LIO2 | QSPI_CINSTRCONF_LIO3;

  /* Write CINSTRCONF register to initiate transfer */

  nrf52_qspi_putreg(priv, NRF52_QSPI_CINSTRCONF_OFFSET, regval);

  /* Wait for the READY event.
   * TODO: add timeout.
   */

  nxsem_wait(&priv->op_sem);

  if (QSPICMD_ISREAD(cmdinfo->flags))
    {
      /* Get response */

      nrf52_qspi_cinstrdata_get(priv, cmdinfo);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_qspi_memory
 *
 * Description:
 *   Perform one QSPI memory transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   meminfo - Describes the memory transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int nrf52_qspi_memory(struct qspi_dev_s *dev,
                             struct qspi_meminfo_s *meminfo)
{
  struct nrf52_qspidev_s *priv   = (struct nrf52_qspidev_s *)dev;

  DEBUGASSERT(meminfo->buffer != NULL && meminfo->buflen > 0);

  if (QSPIMEM_ISWRITE(meminfo->flags))
    {
      /* Configure data transfer */

      nrf52_qspi_putreg(priv, NRF52_QSPI_WRITE_SRC_OFFSET,
                        meminfo->addr);
      nrf52_qspi_putreg(priv, NRF52_QSPI_WRITE_DST_OFFSET,
                        (uint32_t) meminfo->buffer);
      nrf52_qspi_putreg(priv, NRF52_QSPI_WRITE_CNT_OFFSET,
                        meminfo->buflen);

      /* Start WRITE task */

      nrf52_qspi_putreg(priv, NRF52_QSPI_TASKS_WRITESTART_OFFSET, 1);
    }
  else
    {
      /* Configure data transfer */

      nrf52_qspi_putreg(priv, NRF52_QSPI_READ_SRC_OFFSET,
                        meminfo->addr);
      nrf52_qspi_putreg(priv, NRF52_QSPI_READ_DST_OFFSET,
                        (uint32_t) meminfo->buffer);
      nrf52_qspi_putreg(priv, NRF52_QSPI_READ_CNT_OFFSET,
                        meminfo->buflen);

      /* Start READ task */

      nrf52_qspi_putreg(priv, NRF52_QSPI_TASKS_READSTART_OFFSET, 1);
    }

  /* Wait for the READY event.
   * TODO: add timeout.
   */

  nxsem_wait(&priv->op_sem);

  return OK;
}

/****************************************************************************
 * Name: nrf52_qspi_alloc
 *
 * Description:
 *   Allocate a buffer suitable for DMA data transfer
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buflen - Buffer length to allocate in bytes
 *
 * Returned Value:
 *   Address of the allocated memory on success; NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

static void *nrf52_qspi_alloc(struct qspi_dev_s *dev, size_t buflen)
{
  return kmm_malloc(ALIGN_UP(buflen));
}

/****************************************************************************
 * Name: nrf52_qspi_free
 *
 * Description:
 *   Free memory returned by QSPI_ALLOC
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - Buffer previously allocated via QSPI_ALLOC
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void nrf52_qspi_free(struct qspi_dev_s *dev, void *buffer)
{
  if (buffer)
    {
      kmm_free(buffer);
    }
}

/****************************************************************************
 * Name: nrf52_qspi_interrupt
 *
 * Description:
 *   Interrupt handler; we handle all QSPI cases -- reads, writes,
 *   automatic status polling, etc.
 *
 * Input Parameters:
 *   irq  -
 *   context  -
 *   qrg  -
 *
 * Returned Value:
 *   OK means we handled it
 *
 ****************************************************************************/

static int nrf52_qspi_interrupt(int irq, void *context, void *arg)
{
  struct nrf52_qspidev_s *priv = arg;

  /* Clear READY event */

  nrf52_qspi_putreg(priv, NRF52_QSPI_EVENTS_READY_OFFSET, 0);

  /* Signal TASK complete */

  nxsem_post(&g_qspi0_dev.op_sem);

  return OK;
}

/****************************************************************************
 * Name: nrf52_qspi_hw_initialize
 *
 * Description:
 *   Initialize the QSPI peripheral from hardware reset.
 *
 * Input Parameters:
 *   priv - Device state structure.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int nrf52_qspi_hw_initialize(struct nrf52_qspidev_s *priv)
{
  uint32_t regval = 0;
  int      pin    = 0;
  int      port   = 0;
  int      ret    = 0;

  /* Only for QSPI0 */

  DEBUGASSERT(priv->intf == 0);

  /* Attach the interrupt handler */

  ret = irq_attach(NRF52_IRQ_QSPI, nrf52_qspi_interrupt, priv);
  if (ret < 0)
    {
      spierr("ERROR: Failed to attach QSPI irq\n");
      return ret;
    }

  /* SCK pin */

  nrf52_gpio_config(NRF52_QSPI0_SCK_PIN | GPIO_DRIVE_H0H1);
  pin = GPIO_PIN_DECODE(NRF52_QSPI0_SCK_PIN);
  port = GPIO_PORT_DECODE(NRF52_QSPI0_SCK_PIN);
  regval = (pin << QSPI_PSEL_PIN_SHIFT) | (port << QSPI_PSEL_PORT_SHIFT);
  nrf52_qspi_putreg(priv, NRF52_QSPI_PSEL_SCK_OFFSET, regval);

  /* CSN pin */

  nrf52_gpio_config(NRF52_QSPI0_CSN_PIN | GPIO_DRIVE_H0H1);
  pin = GPIO_PIN_DECODE(NRF52_QSPI0_CSN_PIN);
  port = GPIO_PORT_DECODE(NRF52_QSPI0_CSN_PIN);
  regval = (pin << QSPI_PSEL_PIN_SHIFT) | (port << QSPI_PSEL_PORT_SHIFT);
  nrf52_qspi_putreg(priv, NRF52_QSPI_PSEL_CSN_OFFSET, regval);

  /* IO0 pin */

  nrf52_gpio_config(NRF52_QSPI0_IO0_PIN | GPIO_DRIVE_H0H1);
  pin = GPIO_PIN_DECODE(NRF52_QSPI0_IO0_PIN);
  port = GPIO_PORT_DECODE(NRF52_QSPI0_IO0_PIN);
  regval = (pin << QSPI_PSEL_PIN_SHIFT) | (port << QSPI_PSEL_PORT_SHIFT);
  nrf52_qspi_putreg(priv, NRF52_QSPI_PSEL_IO0_OFFSET, regval);

  /* IO1 pin */

  nrf52_gpio_config(NRF52_QSPI0_IO1_PIN | GPIO_DRIVE_H0H1);
  pin = GPIO_PIN_DECODE(NRF52_QSPI0_IO1_PIN);
  port = GPIO_PORT_DECODE(NRF52_QSPI0_IO1_PIN);
  regval = (pin << QSPI_PSEL_PIN_SHIFT) | (port << QSPI_PSEL_PORT_SHIFT);
  nrf52_qspi_putreg(priv, NRF52_QSPI_PSEL_IO1_OFFSET, regval);

  /* IO2 pin */

  nrf52_gpio_config(NRF52_QSPI0_IO2_PIN | GPIO_DRIVE_H0H1);
  pin = GPIO_PIN_DECODE(NRF52_QSPI0_IO2_PIN);
  port = GPIO_PORT_DECODE(NRF52_QSPI0_IO2_PIN);
  regval = (pin << QSPI_PSEL_PIN_SHIFT) | (port << QSPI_PSEL_PORT_SHIFT);
  nrf52_qspi_putreg(priv, NRF52_QSPI_PSEL_IO2_OFFSET, regval);

  /* IO3 pin */

  nrf52_gpio_config(NRF52_QSPI0_IO3_PIN | GPIO_DRIVE_H0H1);
  pin = GPIO_PIN_DECODE(NRF52_QSPI0_IO3_PIN);
  port = GPIO_PORT_DECODE(NRF52_QSPI0_IO3_PIN);
  regval = (pin << QSPI_PSEL_PIN_SHIFT) | (port << QSPI_PSEL_PORT_SHIFT);
  nrf52_qspi_putreg(priv, NRF52_QSPI_PSEL_IO3_OFFSET, regval);

  /* Configure quad data line SPI */

  regval = (QSPI_IFCONFIG0_READOC_READ4IO | QSPI_IFCONFIG0_WRITEOC_PP4IO);
  regval |= QSPI_IFCONFIG0_PPSIZE_512;
  nrf52_qspi_putreg(priv, NRF52_QSPI_IFCONFIG0_OFFSET, regval);

  /* Enable READY interrupt */

  nrf52_qspi_putreg(priv, NRF52_QSPI_INTENSET_OFFSET, QSPI_INT_READY);

  /* Enable QSPI interrupts */

  up_enable_irq(NRF52_IRQ_QSPI);

  /* Enable QSPI */

  nrf52_qspi_putreg(priv, NRF52_QSPI_ENABLE_OFFSET, 1);

  /* Activate QSPI */

  nrf52_qspi_putreg(priv, NRF52_QSPI_TASKS_ACTIVATE_OFFSET, 1);

  /* Wait for READY event.
   * TODO: add timeout.
   */

  nxsem_wait(&priv->op_sem);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_qspi_initialize
 *
 * Description:
 *   Initialize the selected QSPI port in master mode
 *
 * Input Parameters:
 *   intf - Interface number(must be zero)
 *
 * Returned Value:
 *   Valid QSPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct qspi_dev_s *nrf52_qspi_initialize(int intf)
{
  struct nrf52_qspidev_s *priv = NULL;
  int                     ret  = OK;

  /* The NRF52 has only a single QSPI port */

  spiinfo("intf: %d\n", intf);
  DEBUGASSERT(intf == 0);

  /* Select the QSPI interface */

  if (intf == 0)
    {
      /* Select QSPI0 */

      priv = &g_qspi0_dev;
    }
  else
    {
      spierr("ERROR: QSPI%d not supported\n", intf);
      return NULL;
    }

  /* Has the QSPI hardware been initialized? */

  if (!priv->initialized)
    {
      /* Perform hardware initialization.  Puts the QSPI into an active
       * state.
       */

      ret = nrf52_qspi_hw_initialize(priv);
      if (ret < 0)
        {
          spierr("ERROR: Failed to initialize QSPI hardware\n");
          irq_detach(NRF52_IRQ_QSPI);
          return NULL;
        }

      /* Enable interrupts at the NVIC */

      priv->initialized = true;
    }

  return &priv->qspi;
}
