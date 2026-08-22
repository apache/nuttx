/****************************************************************************
 * arch/arm/src/stm32h7/stm32_xspi.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <errno.h>
#include <stdlib.h>
#include <arch/board/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/qspi.h>

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "stm32_xspi.h"
#include "hardware/stm32h7rsxx_pwr.h"
#include "hardware/stm32h7rsxx_rcc.h"
#include "hardware/stm32h7rsxx_sbs.h"
#include "hardware/stm32h7rsxx_xspi.h"

#ifdef CONFIG_STM32_XSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XSPI_TIMEOUT 1000000

/* MTYP=5 is a reserved encoding in the DCR1 register */

#define XSPI_MEMTYPE_RESERVED 5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_xspidev_s
{
  struct qspi_dev_s qspi;
  uint32_t base;
  mutex_t lock;
  uint8_t intf;
  bool initialized;
  bool memmap;
  bool sample_shift;
};

struct stm32_xspi_cmd_s
{
  uint32_t instruction;
  uint32_t address;
  void *buffer;
  size_t buflen;
  uint8_t instruction_lines;
  uint8_t address_lines;
  uint8_t data_lines;
  uint8_t instruction_bits;
  uint8_t address_bits;
  uint8_t dummy_cycles;
  bool dtr;
  bool write;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int xspi_lock(struct qspi_dev_s *dev, bool lock);
static uint32_t xspi_setfrequency(struct qspi_dev_s *dev,
                                  uint32_t frequency);
static void xspi_setmode(struct qspi_dev_s *dev, enum qspi_mode_e mode);
static void xspi_setbits(struct qspi_dev_s *dev, int nbits);
static int xspi_command(struct qspi_dev_s *dev,
                        struct qspi_cmdinfo_s *cmdinfo);
static int xspi_memory(struct qspi_dev_s *dev,
                       struct qspi_meminfo_s *meminfo);
static void *xspi_alloc(struct qspi_dev_s *dev, size_t buflen);
static void xspi_free(struct qspi_dev_s *dev, void *buffer);
static int xspi_mmap_cmd(struct stm32_xspidev_s *priv,
                         const struct qspi_meminfo_s *meminfo,
                         bool write);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qspi_ops_s g_xspi_ops =
{
  .lock         = xspi_lock,
  .setfrequency = xspi_setfrequency,
  .setmode      = xspi_setmode,
  .setbits      = xspi_setbits,
  .command      = xspi_command,
  .memory       = xspi_memory,
  .alloc        = xspi_alloc,
  .free         = xspi_free,
};

static struct stm32_xspidev_s g_xspi1dev =
{
  .qspi =
  {
    .ops = &g_xspi_ops,
  },
  .base = STM32_XSPI1_R_BASE,
  .lock = NXMUTEX_INITIALIZER,
  .intf = 1,
};

static struct stm32_xspidev_s g_xspi2dev =
{
  .qspi =
  {
    .ops = &g_xspi_ops,
  },
  .base = STM32_XSPI2_R_BASE,
  .lock = NXMUTEX_INITIALIZER,
  .intf = 2,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xspi_getreg
 *
 * Description:
 *   Read an XSPI register.
 *
 ****************************************************************************/

static inline uint32_t xspi_getreg(struct stm32_xspidev_s *priv,
                                   uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: xspi_putreg
 *
 * Description:
 *   Write a value to an XSPI register.
 *
 ****************************************************************************/

static inline void xspi_putreg(struct stm32_xspidev_s *priv,
                               uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: xspi_device
 *
 * Description:
 *   Map an XSPI interface number to its device state structure.
 *
 ****************************************************************************/

static struct stm32_xspidev_s *xspi_device(int intf)
{
  if (intf == 1)
    {
      return &g_xspi1dev;
    }

  if (intf == 2)
    {
      return &g_xspi2dev;
    }

  return NULL;
}

/****************************************************************************
 * Name: xspi_wait
 *
 * Description:
 *   Poll until the masked status register bits equal the given value.
 *
 ****************************************************************************/

static int xspi_wait(struct stm32_xspidev_s *priv, uint32_t mask,
                     uint32_t value)
{
  int timeout;

  for (timeout = XSPI_TIMEOUT; timeout > 0; timeout--)
    {
      uint32_t status = xspi_getreg(priv, STM32_XSPI_SR_OFFSET);

      if ((status & XSPI_SR_TEF) != 0)
        {
          xspi_putreg(priv, STM32_XSPI_FCR_OFFSET, XSPI_FCR_CTEF);
          return -EIO;
        }

      if ((status & mask) == value)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: xspi_wait_any
 *
 * Description:
 *   Poll until any of the masked status register bits is set.
 *
 ****************************************************************************/

static int xspi_wait_any(struct stm32_xspidev_s *priv, uint32_t mask)
{
  int timeout;

  for (timeout = XSPI_TIMEOUT; timeout > 0; timeout--)
    {
      uint32_t status = xspi_getreg(priv, STM32_XSPI_SR_OFFSET);

      if ((status & XSPI_SR_TEF) != 0)
        {
          xspi_putreg(priv, STM32_XSPI_FCR_OFFSET, XSPI_FCR_CTEF);
          return -EIO;
        }

      if ((status & mask) != 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: xspi_size
 *
 * Description:
 *   Convert a bit width (8/16/24/32) to the CCR SIZE field encoding.
 *
 ****************************************************************************/

static int xspi_size(uint8_t bits)
{
  if (bits == 8 || bits == 16 || bits == 24 || bits == 32)
    {
      return (bits >> 3) - 1;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: xspi_lines
 *
 * Description:
 *   Convert a line count (0/1/2/4/8) to the CCR MODE field encoding.
 *
 ****************************************************************************/

static int xspi_lines(uint8_t lines)
{
  switch (lines)
    {
      case 0:
        return 0;
      case 1:
        return 1;
      case 2:
        return 2;
      case 4:
        return 3;
      case 8:
        return 4;
      default:
        return -EINVAL;
    }
}

/****************************************************************************
 * Name: xspi_ccr
 *
 * Description:
 *   Build the CCR register value for a transfer command.
 *
 ****************************************************************************/

static int xspi_ccr(const struct stm32_xspi_cmd_s *cmd, uint32_t *ccr)
{
  uint32_t reg = 0;
  int lines;
  int size;

  lines = xspi_lines(cmd->instruction_lines);
  if (lines < 0)
    {
      return lines;
    }

  if (lines != 0)
    {
      size = xspi_size(cmd->instruction_bits);
      if (size < 0)
        {
          return size;
        }

      reg |= XSPI_CCR_IMODE(lines) | XSPI_CCR_ISIZE(size);
      if (cmd->dtr)
        {
          reg |= XSPI_CCR_IDTR;
        }
    }

  lines = xspi_lines(cmd->address_lines);
  if (lines < 0)
    {
      return lines;
    }

  if (lines != 0)
    {
      size = xspi_size(cmd->address_bits);
      if (size < 0)
        {
          return size;
        }

      reg |= XSPI_CCR_ADMODE(lines) | XSPI_CCR_ADSIZE(size);
      if (cmd->dtr)
        {
          reg |= XSPI_CCR_ADDTR;
        }
    }

  lines = xspi_lines(cmd->data_lines);
  if (lines < 0)
    {
      return lines;
    }

  reg |= XSPI_CCR_DMODE(lines);
  if (cmd->dtr && lines != 0)
    {
      reg |= XSPI_CCR_DDTR;
    }

  if (cmd->dummy_cycles > 31)
    {
      return -EINVAL;
    }

  *ccr = reg;
  return OK;
}

/****************************************************************************
 * Name: xspi_iom_config
 *
 * Description:
 *   Configure the XSPI I/O manager port and chip select routing.
 *
 ****************************************************************************/

static void xspi_iom_config(struct stm32_xspidev_s *priv,
                            const struct stm32_xspi_config_s *config)
{
  enum stm32_xspi_iomport_e port1;
  enum stm32_xspi_iomport_e port2;
  enum stm32_xspi_cs_e cs1;
  enum stm32_xspi_cs_e cs2;
  uint32_t xspi1cr;
  uint32_t xspi2cr;
  uint32_t reg;

  reg = getreg32(STM32_XSPIM_CR);
  port1 = (reg & XSPIM_CR_MODE) != 0 ? STM32_XSPI_IOM_PORT2 :
                                      STM32_XSPI_IOM_PORT1;
  port2 = ((reg & XSPIM_CR_MUXEN) != 0) !=
          ((reg & XSPIM_CR_MODE) != 0) ? STM32_XSPI_IOM_PORT1 :
                                        STM32_XSPI_IOM_PORT2;

  if ((reg & XSPIM_CR_CSSEL_OVR_EN) == 0)
    {
      cs1 = STM32_XSPI_CS_DISABLED;
      cs2 = STM32_XSPI_CS_DISABLED;
    }
  else
    {
      cs1 = (reg & XSPIM_CR_CSSEL_OVR_O1) != 0 ? STM32_XSPI_CS_NCS2 :
                                                 STM32_XSPI_CS_NCS1;
      cs2 = (reg & XSPIM_CR_CSSEL_OVR_O2) != 0 ? STM32_XSPI_CS_NCS2 :
                                                 STM32_XSPI_CS_NCS1;
    }

  if (priv->intf == 1)
    {
      port1 = config->iomport;
      cs1   = config->cs_override;
    }
  else
    {
      port2 = config->iomport;
      cs2   = config->cs_override;
    }

  reg = XSPIM_CR_REQ2ACK(config->req2ack);
  if (port1 == port2)
    {
      reg |= XSPIM_CR_MUXEN;
    }

  if (port1 == STM32_XSPI_IOM_PORT2)
    {
      reg |= XSPIM_CR_MODE;
    }

  if (cs1 != STM32_XSPI_CS_DISABLED || cs2 != STM32_XSPI_CS_DISABLED)
    {
      reg |= XSPIM_CR_CSSEL_OVR_EN;
    }

  if (cs1 == STM32_XSPI_CS_NCS2)
    {
      reg |= XSPIM_CR_CSSEL_OVR_O1;
    }

  if (cs2 == STM32_XSPI_CS_NCS2)
    {
      reg |= XSPIM_CR_CSSEL_OVR_O2;
    }

  xspi1cr = getreg32(STM32_XSPI1_CR);
  xspi2cr = getreg32(STM32_XSPI2_CR);
  modifyreg32(STM32_XSPI1_CR, XSPI_CR_EN, 0);
  modifyreg32(STM32_XSPI2_CR, XSPI_CR_EN, 0);
  putreg32(reg, STM32_XSPIM_CR);
  putreg32(xspi1cr, STM32_XSPI1_CR);
  putreg32(xspi2cr, STM32_XSPI2_CR);
}

/****************************************************************************
 * Name: xspi_compensation_enable
 *
 * Description:
 *   Enable the XSPI I/O compensation cell and wait until it is ready.
 *
 ****************************************************************************/

static int xspi_compensation_enable(struct stm32_xspidev_s *priv)
{
  uint32_t enable;
  uint32_t ready;
  int timeout;

  modifyreg32(STM32_RCC_APB4ENR, 0, RCC_APB4ENR_SBSEN);
  modifyreg32(STM32_RCC_CR, 0, RCC_CR_CSION);

  for (timeout = XSPI_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CR) & RCC_CR_CSIRDY) != 0)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }

  if (priv->intf == 1)
    {
      enable = SBS_CCCSR_XSPI1_COMP_EN | SBS_CCCSR_XSPI1_IOHSLV;
      ready  = SBS_CCCSR_XSPI1_COMP_RDY;
    }
  else
    {
      enable = SBS_CCCSR_XSPI2_COMP_EN | SBS_CCCSR_XSPI2_IOHSLV;
      ready  = SBS_CCCSR_XSPI2_COMP_RDY;
    }

  modifyreg32(STM32_SBS_CCCSR, 0, enable);
  for (timeout = XSPI_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_SBS_CCCSR) & ready) != 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: xspi_hw_initialize
 *
 * Description:
 *   One-time hardware initialization: power, clocking and GPIO pins.
 *
 ****************************************************************************/

static int xspi_hw_initialize(struct stm32_xspidev_s *priv)
{
  if (priv->intf == 1)
    {
      modifyreg32(STM32_PWR_CSR2, 0, PWR_CSR2_EN_XSPIM1);
      modifyreg32(STM32_RCC_CCIPR1, RCC_CCIPR1_XSPI1SEL_MASK,
                  RCC_CCIPR1_XSPI1SEL_HCLK);
      modifyreg32(STM32_RCC_AHB5ENR, 0, RCC_AHB5ENR_XSPI1EN |
                   RCC_AHB5ENR_XSPIMEN);

#ifdef GPIO_XSPI1_CLK
      stm32_configgpio(GPIO_XSPI1_CLK);
#endif
#ifdef GPIO_XSPI1_DQS
      stm32_configgpio(GPIO_XSPI1_DQS);
#endif
#ifdef GPIO_XSPI1_NCS
      stm32_configgpio(GPIO_XSPI1_NCS);
#endif
#ifdef GPIO_XSPI1_IO0
      stm32_configgpio(GPIO_XSPI1_IO0);
#endif
#ifdef GPIO_XSPI1_IO1
      stm32_configgpio(GPIO_XSPI1_IO1);
#endif
#ifdef GPIO_XSPI1_IO2
      stm32_configgpio(GPIO_XSPI1_IO2);
#endif
#ifdef GPIO_XSPI1_IO3
      stm32_configgpio(GPIO_XSPI1_IO3);
#endif
#ifdef GPIO_XSPI1_IO4
      stm32_configgpio(GPIO_XSPI1_IO4);
#endif
#ifdef GPIO_XSPI1_IO5
      stm32_configgpio(GPIO_XSPI1_IO5);
#endif
#ifdef GPIO_XSPI1_IO6
      stm32_configgpio(GPIO_XSPI1_IO6);
#endif
#ifdef GPIO_XSPI1_IO7
      stm32_configgpio(GPIO_XSPI1_IO7);
#endif
    }
  else
    {
      modifyreg32(STM32_PWR_CSR2, 0, PWR_CSR2_EN_XSPIM2);
      modifyreg32(STM32_RCC_CCIPR1, RCC_CCIPR1_XSPI2SEL_MASK,
                  RCC_CCIPR1_XSPI2SEL_HCLK);
      modifyreg32(STM32_RCC_AHB5ENR, 0, RCC_AHB5ENR_XSPI2EN |
                   RCC_AHB5ENR_XSPIMEN);

#ifdef GPIO_XSPI2_CLK
      stm32_configgpio(GPIO_XSPI2_CLK);
#endif
#ifdef GPIO_XSPI2_DQS
      stm32_configgpio(GPIO_XSPI2_DQS);
#endif
#ifdef GPIO_XSPI2_NCS
      stm32_configgpio(GPIO_XSPI2_NCS);
#endif
#ifdef GPIO_XSPI2_IO0
      stm32_configgpio(GPIO_XSPI2_IO0);
#endif
#ifdef GPIO_XSPI2_IO1
      stm32_configgpio(GPIO_XSPI2_IO1);
#endif
#ifdef GPIO_XSPI2_IO2
      stm32_configgpio(GPIO_XSPI2_IO2);
#endif
#ifdef GPIO_XSPI2_IO3
      stm32_configgpio(GPIO_XSPI2_IO3);
#endif
#ifdef GPIO_XSPI2_IO4
      stm32_configgpio(GPIO_XSPI2_IO4);
#endif
#ifdef GPIO_XSPI2_IO5
      stm32_configgpio(GPIO_XSPI2_IO5);
#endif
#ifdef GPIO_XSPI2_IO6
      stm32_configgpio(GPIO_XSPI2_IO6);
#endif
#ifdef GPIO_XSPI2_IO7
      stm32_configgpio(GPIO_XSPI2_IO7);
#endif
    }

  return xspi_compensation_enable(priv);
}

/****************************************************************************
 * Name: xspi_lock
 *
 * Description:
 *   Lock or unlock the XSPI bus for exclusive access.
 *
 ****************************************************************************/

static int xspi_lock(struct qspi_dev_s *dev, bool lock)
{
  struct stm32_xspidev_s *priv = (struct stm32_xspidev_s *)dev;

  return lock ? nxmutex_lock(&priv->lock) : nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: xspi_setfrequency
 *
 * Description:
 *   Set the XSPI frequency and return the actual frequency selected.
 *
 ****************************************************************************/

static uint32_t xspi_setfrequency(struct qspi_dev_s *dev,
                                  uint32_t frequency)
{
  struct stm32_xspidev_s *priv = (struct stm32_xspidev_s *)dev;
  uint32_t prescaler;

  if (frequency == 0)
    {
      return 0;
    }

  prescaler = (STM32_HCLK_FREQUENCY + frequency - 1) / frequency;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > 256)
    {
      prescaler = 256;
    }

  modifyreg32(priv->base + STM32_XSPI_DCR2_OFFSET,
              XSPI_DCR2_PRESCALER_MASK,
              XSPI_DCR2_PRESCALER(prescaler - 1));
  return STM32_HCLK_FREQUENCY / prescaler;
}

/****************************************************************************
 * Name: xspi_setmode
 *
 * Description:
 *   Set the XSPI clock mode (mode 0 or mode 3).
 *
 ****************************************************************************/

static void xspi_setmode(struct qspi_dev_s *dev, enum qspi_mode_e mode)
{
  struct stm32_xspidev_s *priv = (struct stm32_xspidev_s *)dev;

  DEBUGASSERT(mode == QSPIDEV_MODE0 || mode == QSPIDEV_MODE3);
  modifyreg32(priv->base + STM32_XSPI_DCR1_OFFSET, XSPI_DCR1_CKMODE,
              mode == QSPIDEV_MODE3 ? XSPI_DCR1_CKMODE : 0);
}

/****************************************************************************
 * Name: xspi_setbits
 *
 * Description:
 *   Set the number of bits per word (only 8 bits are supported).
 *
 ****************************************************************************/

static void xspi_setbits(struct qspi_dev_s *dev, int nbits)
{
  UNUSED(dev);
  DEBUGASSERT(nbits == 8);
}

/****************************************************************************
 * Name: xspi_configure
 *
 * Description:
 *   Validate the configuration and program the XSPI device registers.
 *
 ****************************************************************************/

static int xspi_configure(struct stm32_xspidev_s *priv,
                          const struct stm32_xspi_config_s *config)
{
  uint32_t dcr1;
  uint32_t cr;
  int ret;

  if (config == NULL || config->memtype == XSPI_MEMTYPE_RESERVED ||
      config->memtype > STM32_XSPI_MEMTYPE_APMEM_16BIT ||
      config->iomport > STM32_XSPI_IOM_PORT2 ||
      config->cs_override > STM32_XSPI_CS_NCS2 || config->devsize > 31 ||
      config->csht < 1 || config->csht > 64 || config->fthres < 1 ||
      config->fthres > 32 || config->wrapsize > 7 ||
      config->csbound > 31 || config->req2ack < 1 ||
      config->req2ack > 256)
    {
      return -EINVAL;
    }

  ret = xspi_wait(priv, XSPI_SR_BUSY, 0);
  if (ret < 0)
    {
      return ret;
    }

  cr = xspi_getreg(priv, STM32_XSPI_CR_OFFSET);
  xspi_putreg(priv, STM32_XSPI_CR_OFFSET, cr & ~XSPI_CR_EN);
  xspi_iom_config(priv, config);

  dcr1 = XSPI_DCR1_CSHT(config->csht) |
         XSPI_DCR1_DEVSIZE(config->devsize) |
         XSPI_DCR1_MTYP(config->memtype);
  if (config->clock_mode3)
    {
      dcr1 |= XSPI_DCR1_CKMODE;
    }

  if (config->free_running)
    {
      dcr1 |= XSPI_DCR1_FRCK;
    }

  xspi_putreg(priv, STM32_XSPI_DCR1_OFFSET, dcr1);
  xspi_putreg(priv, STM32_XSPI_DCR2_OFFSET,
              XSPI_DCR2_PRESCALER(config->prescaler) |
              XSPI_DCR2_WRAPSIZE(config->wrapsize));
  xspi_putreg(priv, STM32_XSPI_DCR3_OFFSET,
              XSPI_DCR3_MAXTRAN(config->maxtran) |
              XSPI_DCR3_CSBOUND(config->csbound));
  xspi_putreg(priv, STM32_XSPI_DCR4_OFFSET, config->refresh);

  priv->sample_shift = config->sample_shift;
  cr &= ~(XSPI_CR_FTHRES_MASK | XSPI_CR_FMODE_MASK);
  cr |= XSPI_CR_FTHRES(config->fthres) | XSPI_CR_EN;
  xspi_putreg(priv, STM32_XSPI_CR_OFFSET, cr);
  return OK;
}

/****************************************************************************
 * Name: xspi_transfer
 *
 * Description:
 *   Execute a single polled indirect-mode transfer.
 *
 ****************************************************************************/

static int xspi_transfer(struct stm32_xspidev_s *priv,
                         const struct stm32_xspi_cmd_s *cmd)
{
  uint8_t *buffer;
  uint32_t ccr;
  uint32_t tcr;
  uint32_t fmode;
  size_t remaining;
  int ret;

  if (cmd == NULL || (cmd->buflen > 0 && cmd->buffer == NULL) ||
      (cmd->buflen > 0 && cmd->data_lines == 0) ||
      (cmd->buflen == 0 && cmd->data_lines != 0))
    {
      return -EINVAL;
    }

  ret = xspi_ccr(cmd, &ccr);
  if (ret < 0)
    {
      return ret;
    }

  ret = xspi_wait(priv, XSPI_SR_BUSY, 0);
  if (ret < 0)
    {
      return ret;
    }

  xspi_putreg(priv, STM32_XSPI_FCR_OFFSET, XSPI_FCR_ALL);
  if (cmd->buflen > 0)
    {
      xspi_putreg(priv, STM32_XSPI_DLR_OFFSET, cmd->buflen - 1);
    }

  tcr = XSPI_TCR_DCYC(cmd->dummy_cycles);
  if (priv->sample_shift && !cmd->dtr)
    {
      tcr |= XSPI_TCR_SSHIFT;
    }

  xspi_putreg(priv, STM32_XSPI_CCR_OFFSET, ccr);
  xspi_putreg(priv, STM32_XSPI_TCR_OFFSET, tcr);
  fmode = cmd->write ? XSPI_CR_FMODE_WRITE : XSPI_CR_FMODE_READ;
  modifyreg32(priv->base + STM32_XSPI_CR_OFFSET, XSPI_CR_FMODE_MASK,
              fmode);

  xspi_putreg(priv, STM32_XSPI_IR_OFFSET, cmd->instruction);
  if (cmd->address_lines != 0)
    {
      xspi_putreg(priv, STM32_XSPI_AR_OFFSET, cmd->address);
    }

  if (cmd->buflen == 0)
    {
      ret = xspi_wait(priv, XSPI_SR_BUSY, 0);
      xspi_putreg(priv, STM32_XSPI_FCR_OFFSET, XSPI_FCR_CTCF);
      return ret;
    }

  buffer = cmd->buffer;
  remaining = cmd->buflen;
  while (remaining > 0)
    {
      uint32_t flags = cmd->write ? XSPI_SR_FTF :
                                    (XSPI_SR_FTF | XSPI_SR_TCF);

      ret = xspi_wait_any(priv, flags);
      if (ret < 0)
        {
          break;
        }

      if (cmd->write)
        {
          putreg8(*buffer, priv->base + STM32_XSPI_DR_OFFSET);
        }
      else
        {
          *buffer = getreg8(priv->base + STM32_XSPI_DR_OFFSET);
        }

      buffer++;
      remaining--;
    }

  if (ret == OK)
    {
      ret = xspi_wait(priv, XSPI_SR_TCF, XSPI_SR_TCF);
    }

  xspi_putreg(priv, STM32_XSPI_FCR_OFFSET,
              XSPI_FCR_CTEF | XSPI_FCR_CTCF);
  return ret;
}

/****************************************************************************
 * Name: xspi_instruction_lines
 *
 * Description:
 *   Get the instruction line count from command or memory transfer flags.
 *
 ****************************************************************************/

static uint8_t xspi_instruction_lines(uint16_t flags, bool memory)
{
  if (memory ? QSPIMEM_ISIOCTAL(flags) : QSPICMD_ISIOCTAL(flags))
    {
      return 8;
    }

  if (memory ? QSPIMEM_ISIQUAD(flags) : QSPICMD_ISIQUAD(flags))
    {
      return 4;
    }

  if (memory ? QSPIMEM_ISIDUAL(flags) : QSPICMD_ISIDUAL(flags))
    {
      return 2;
    }

  return 1;
}

/****************************************************************************
 * Name: xspi_command
 *
 * Description:
 *   Perform one QSPI command transaction.
 *
 ****************************************************************************/

static int xspi_command(struct qspi_dev_s *dev,
                        struct qspi_cmdinfo_s *cmdinfo)
{
  struct stm32_xspidev_s *priv = (struct stm32_xspidev_s *)dev;
  struct stm32_xspi_cmd_s cmd;

  if (cmdinfo == NULL || cmdinfo->addrlen > 4 ||
      (QSPICMD_ISDATA(cmdinfo->flags) && cmdinfo->buffer == NULL))
    {
      return -EINVAL;
    }

  cmd.instruction       = cmdinfo->cmd;
  cmd.address           = cmdinfo->addr;
  cmd.buffer            = cmdinfo->buffer;
  cmd.buflen            = QSPICMD_ISDATA(cmdinfo->flags) ?
                          cmdinfo->buflen : 0;
  cmd.instruction_lines = xspi_instruction_lines(cmdinfo->flags, false);
  cmd.address_lines     = QSPICMD_ISADDRESS(cmdinfo->flags) ?
                          (QSPICMD_ISOCTALIO(cmdinfo->flags) ? 8 : 1) : 0;
  cmd.data_lines        = QSPICMD_ISDATA(cmdinfo->flags) ?
                          (QSPICMD_ISOCTALIO(cmdinfo->flags) ? 8 : 1) : 0;
  cmd.instruction_bits  = QSPICMD_ISIOCTAL(cmdinfo->flags) ? 16 : 8;
  cmd.address_bits      = cmdinfo->addrlen * 8;
  cmd.dummy_cycles      = 0;
  cmd.dtr               = QSPICMD_ISDTR(cmdinfo->flags);
  cmd.write             = !QSPICMD_ISREAD(cmdinfo->flags);
  return xspi_transfer(priv, &cmd);
}

/****************************************************************************
 * Name: xspi_memory
 *
 * Description:
 *   Perform one QSPI memory transfer.
 *
 ****************************************************************************/

static int xspi_memory(struct qspi_dev_s *dev,
                       struct qspi_meminfo_s *meminfo)
{
  struct stm32_xspidev_s *priv = (struct stm32_xspidev_s *)dev;
  struct stm32_xspi_cmd_s cmd;

  if (meminfo == NULL || meminfo->addrlen > 4 ||
      meminfo->buflen == 0 || meminfo->buffer == NULL ||
      meminfo->dummies > 31)
    {
      return -EINVAL;
    }

  cmd.instruction       = meminfo->cmd;
  cmd.address           = meminfo->addr;
  cmd.buffer            = meminfo->buffer;
  cmd.buflen            = meminfo->buflen;
  cmd.instruction_lines = xspi_instruction_lines(meminfo->flags, true);
  cmd.address_lines     = meminfo->addrlen == 0 ? 0 :
                          QSPIMEM_ISOCTALIO(meminfo->flags) ? 8 :
                          QSPIMEM_ISQUADIO(meminfo->flags) ? 4 :
                          QSPIMEM_ISDUALIO(meminfo->flags) ? 2 : 1;
  cmd.data_lines        = QSPIMEM_ISOCTALIO(meminfo->flags) ? 8 :
                          (QSPIMEM_ISQUADIO(meminfo->flags) ||
                           QSPIMEM_ISQUADDATA(meminfo->flags)) ? 4 :
                          QSPIMEM_ISDUALIO(meminfo->flags) ? 2 : 1;
  cmd.instruction_bits  = QSPIMEM_ISIOCTAL(meminfo->flags) ? 16 : 8;
  cmd.address_bits      = meminfo->addrlen * 8;
  cmd.dummy_cycles      = meminfo->dummies;
  cmd.dtr               = QSPIMEM_ISDTR(meminfo->flags);
  cmd.write             = QSPIMEM_ISWRITE(meminfo->flags);
  return xspi_transfer(priv, &cmd);
}

/****************************************************************************
 * Name: xspi_alloc
 *
 * Description:
 *   Allocate a buffer suitable for XSPI transfers.
 *
 ****************************************************************************/

static void *xspi_alloc(struct qspi_dev_s *dev, size_t buflen)
{
  UNUSED(dev);
  return kmm_malloc(buflen);
}

/****************************************************************************
 * Name: xspi_free
 *
 * Description:
 *   Free a buffer allocated by xspi_alloc.
 *
 ****************************************************************************/

static void xspi_free(struct qspi_dev_s *dev, void *buffer)
{
  UNUSED(dev);
  kmm_free(buffer);
}

/****************************************************************************
 * Name: xspi_mmap_cmd
 *
 * Description:
 *   Program the read or write command registers for memory-mapped mode.
 *
 ****************************************************************************/

static int xspi_mmap_cmd(struct stm32_xspidev_s *priv,
                         const struct qspi_meminfo_s *meminfo,
                         bool write)
{
  struct stm32_xspi_cmd_s cmd;
  uint32_t ccr_offset = write ? STM32_XSPI_WCCR_OFFSET :
                                STM32_XSPI_CCR_OFFSET;
  uint32_t tcr_offset = write ? STM32_XSPI_WTCR_OFFSET :
                                STM32_XSPI_TCR_OFFSET;
  uint32_t ir_offset  = write ? STM32_XSPI_WIR_OFFSET :
                                STM32_XSPI_IR_OFFSET;
  uint32_t ccr;
  uint32_t tcr;
  int ret;

  if (meminfo == NULL || meminfo->addrlen == 0 || meminfo->addrlen > 4 ||
      meminfo->dummies > 31)
    {
      return -EINVAL;
    }

  cmd.instruction       = meminfo->cmd;
  cmd.address           = 0;
  cmd.buffer            = NULL;
  cmd.buflen            = 0;
  cmd.instruction_lines = xspi_instruction_lines(meminfo->flags, true);
  cmd.address_lines     = QSPIMEM_ISOCTALIO(meminfo->flags) ? 8 :
                          QSPIMEM_ISQUADIO(meminfo->flags) ? 4 :
                          QSPIMEM_ISDUALIO(meminfo->flags) ? 2 : 1;
  cmd.data_lines        = QSPIMEM_ISOCTALIO(meminfo->flags) ? 8 :
                          (QSPIMEM_ISQUADIO(meminfo->flags) ||
                           QSPIMEM_ISQUADDATA(meminfo->flags)) ? 4 :
                          QSPIMEM_ISDUALIO(meminfo->flags) ? 2 : 1;
  cmd.instruction_bits  = QSPIMEM_ISIOCTAL(meminfo->flags) ? 16 : 8;
  cmd.address_bits      = meminfo->addrlen * 8;
  cmd.dummy_cycles      = meminfo->dummies;
  cmd.dtr               = QSPIMEM_ISDTR(meminfo->flags);
  cmd.write             = write;

  ret = xspi_ccr(&cmd, &ccr);
  if (ret < 0)
    {
      return ret;
    }

  tcr = XSPI_TCR_DCYC(cmd.dummy_cycles);
  if (priv->sample_shift && !cmd.dtr)
    {
      tcr |= XSPI_TCR_SSHIFT;
    }

  xspi_putreg(priv, ccr_offset, ccr);
  xspi_putreg(priv, tcr_offset, tcr);
  xspi_putreg(priv, ir_offset, cmd.instruction);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_xspi_initialize
 *
 * Description:
 *   Initialize an XSPI controller.
 *
 * Input Parameters:
 *   intf   - XSPI controller number (1 or 2)
 *   config - XSPI controller configuration
 *
 * Returned Value:
 *   Valid QSPI device structure on success; NULL on failure.
 *
 ****************************************************************************/

struct qspi_dev_s *
stm32_xspi_initialize(int intf, const struct stm32_xspi_config_s *config)
{
  struct stm32_xspidev_s *priv = xspi_device(intf);
  int ret;

  if (priv == NULL || config == NULL)
    {
      return NULL;
    }

  if (!priv->initialized)
    {
      ret = xspi_hw_initialize(priv);
      if (ret < 0)
        {
          return NULL;
        }

      priv->initialized = true;
    }

  if (xspi_configure(priv, config) < 0)
    {
      return NULL;
    }

  return &priv->qspi;
}

/****************************************************************************
 * Name: stm32_xspi_enter_memorymapped
 *
 * Description:
 *   Configure the XSPI controller for memory-mapped access.
 *
 * Input Parameters:
 *   dev       - QSPI device
 *   readinfo  - Memory transfer parameters used for reading
 *   writeinfo - Memory transfer parameters used for writing; may be NULL
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_xspi_enter_memorymapped(
  struct qspi_dev_s *dev,
  const struct qspi_meminfo_s *readinfo,
  const struct qspi_meminfo_s *writeinfo)
{
  struct stm32_xspidev_s *priv = (struct stm32_xspidev_s *)dev;
  uint32_t cr;
  int ret;

  if (dev == NULL || dev->ops != &g_xspi_ops)
    {
      return -EINVAL;
    }

  ret = xspi_lock(dev, true);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->memmap)
    {
      xspi_lock(dev, false);
      return OK;
    }

  ret = xspi_wait(priv, XSPI_SR_BUSY, 0);
  if (ret < 0)
    {
      goto errout;
    }

  cr = xspi_getreg(priv, STM32_XSPI_CR_OFFSET);
  xspi_putreg(priv, STM32_XSPI_CR_OFFSET, cr & ~XSPI_CR_EN);

  ret = xspi_mmap_cmd(priv, readinfo, false);
  if (ret == OK && writeinfo != NULL)
    {
      ret = xspi_mmap_cmd(priv, writeinfo, true);
    }

  if (ret < 0)
    {
      xspi_putreg(priv, STM32_XSPI_CR_OFFSET, cr);
      goto errout;
    }

  cr &= ~XSPI_CR_FMODE_MASK;
  cr |= XSPI_CR_EN | XSPI_CR_FMODE_MEMORY;
  xspi_putreg(priv, STM32_XSPI_CR_OFFSET, cr);
  priv->memmap = true;
  xspi_lock(dev, false);
  return OK;

errout:
  xspi_lock(dev, false);
  return ret;
}

/****************************************************************************
 * Name: stm32_xspi_exit_memorymapped
 *
 * Description:
 *   Take the XSPI device out of memory-mapped mode.
 *
 * Input Parameters:
 *   dev - QSPI device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_xspi_exit_memorymapped(struct qspi_dev_s *dev)
{
  struct stm32_xspidev_s *priv = (struct stm32_xspidev_s *)dev;

  if (dev == NULL || dev->ops != &g_xspi_ops)
    {
      return;
    }

  xspi_lock(dev, true);
  modifyreg32(priv->base + STM32_XSPI_CR_OFFSET, 0, XSPI_CR_ABORT);
  xspi_wait(priv, XSPI_SR_BUSY, 0);
  modifyreg32(priv->base + STM32_XSPI_CR_OFFSET,
              XSPI_CR_ABORT | XSPI_CR_FMODE_MASK,
              XSPI_CR_FMODE_WRITE);
  priv->memmap = false;
  xspi_lock(dev, false);
}

#endif /* CONFIG_STM32_XSPI */
