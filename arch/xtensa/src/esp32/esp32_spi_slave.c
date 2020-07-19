/****************************************************************************
 * arch/xtensa/src/esp32/esp32_spislv_slave.c
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

#ifdef CONFIG_ESP32_SPI

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/slave.h>

#include <arch/board/board.h>

#include "esp32_spi.h"
#include "esp32_gpio.h"
#include "esp32_cpuint.h"

#include "xtensa.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_spi.h"
#include "hardware/esp32_soc.h"
#include "rom/esp32_gpio.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifndef CONFIG_ESP_SPI_SLAVE_QSIZE
  #define CONFIG_ESP_SPI_SLAVE_QSIZE 1024
#endif

/* SPI Device hardware configuration */

struct esp32_spislv_config_s
{
  uint32_t reg_base;          /* SPI register base address */

  enum spi_mode_e mode;       /* SPI default mode */

  uint8_t cs_pin;             /* GPIO configuration for CS */
  uint8_t mosi_pin;           /* GPIO configuration for MOSI */
  uint8_t miso_pin;           /* GPIO configuration for MISO */
  uint8_t clk_pin;            /* GPIO configuration for CLK */

  uint8_t cpu;                /* CPU ID */
  uint8_t periph;             /* peripher ID */
  uint8_t irq;                /* Interrupt ID */

  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t rst_bit;           /* I2C reset bit */

  uint32_t cs_insig;          /* SPI CS input signal index */
  uint32_t cs_outsig;         /* SPI CS output signal index */
  uint32_t mosi_insig;        /* SPI MOSI input signal index */
  uint32_t mosi_outsig;       /* SPI MOSI output signal index */
  uint32_t miso_insig;        /* SPI MISO input signal index */
  uint32_t miso_outsig;       /* SPI MISO output signal index */
  uint32_t clk_insig;         /* SPI CLK input signal index */
  uint32_t clk_outsig;        /* SPI CLK output signal index */
};

struct esp32_spislv_priv_s
{
  /* Externally visible part of the SPI slave controller interface */

  struct spi_sctrlr_s sctrlr;

  struct spi_sdev_s   *sdev;    /* Externally visible part of the SPI interface */

  const struct esp32_spislv_config_s *config; /* Port configuration */

  uint32_t         cpuint;      /* SPI interrupt ID */

  enum spi_mode_e  mode;        /* Actual SPI hardware mode */
  uint8_t          nbits;       /* Actual SPI send/receive bits once transmission */
  int              refs;        /* Check if it is initialized */

  uint32_t         head;        /* Location of next value */
  uint32_t         tail;        /* Index of first value */

  /* SPI slave output queue buffer */

  uint16_t         outq[CONFIG_ESP_SPI_SLAVE_QSIZE];

  uint32_t         outval;      /* Default shift-out value */

  bool             process;     /* If SPI Slave process */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp32_spislv_setmode(FAR struct spi_sctrlr_s *dev,
                                 enum spi_mode_e mode);
static void esp32_spislv_setbits(FAR struct spi_sctrlr_s *dev, int nbits);
static int esp32_spislv_interrupt(int irq, void *context, FAR void *arg);
static void esp32_spislv_initialize(FAR struct spi_sctrlr_s *dev);
static void esp32_spislv_bind(struct spi_sctrlr_s *sctrlr,
                              struct spi_sdev_s *sdev,
                              enum spi_smode_e mode,
                              int nbits);
static void esp32_spislv_unbind(struct spi_sctrlr_s *sctrlr);
static int esp32_spislv_enqueue(struct spi_sctrlr_s *sctrlr,
                                FAR const void *data,
                                size_t nwords);
static bool esp32_spislv_qfull(struct spi_sctrlr_s *sctrlr);
static void esp32_spislv_qflush(struct spi_sctrlr_s *sctrlr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32_SPI2
static const struct esp32_spislv_config_s esp32_spi2_config =
{
  .reg_base     = REG_SPI_BASE(2),
  .mode         = SPIDEV_MODE0,
  .cs_pin       = CONFIG_ESP32_SPI2_CSPIN,
  .mosi_pin     = CONFIG_ESP32_SPI2_MOSIPIN,
  .miso_pin     = CONFIG_ESP32_SPI2_MISOPIN,
  .clk_pin      = CONFIG_ESP32_SPI2_CLKPIN,
  .cpu          = 0,
  .periph       = ESP32_PERIPH_SPI2,
  .irq          = ESP32_IRQ_SPI2,
  .clk_bit      = DPORT_SPI_CLK_EN_2,
  .rst_bit      = DPORT_SPI_RST_2,
  .cs_insig     = HSPICS0_IN_IDX,
  .cs_outsig    = HSPICS0_OUT_IDX,
  .mosi_insig   = HSPID_IN_IDX,
  .mosi_outsig  = HSPID_OUT_IDX,
  .miso_insig   = HSPIQ_IN_IDX,
  .miso_outsig  = HSPIQ_OUT_IDX,
  .clk_insig    = HSPICLK_IN_IDX,
  .clk_outsig   = HSPICLK_OUT_IDX
};

static const struct spi_sctrlrops_s esp32_spi2slv_ops =
{
  .bind     = esp32_spislv_bind,
  .unbind   = esp32_spislv_unbind,
  .enqueue  = esp32_spislv_enqueue,
  .qfull    = esp32_spislv_qfull,
  .qflush   = esp32_spislv_qflush
};

static struct esp32_spislv_priv_s esp32_spi2slv_priv =
{
  .sctrlr =
              {
                .ops = &esp32_spi2slv_ops
              },
  .config = &esp32_spi2_config,
  .mode = SPIDEV_MODE3
};
#endif /* CONFIG_ESP32_SPI2 */

#ifdef CONFIG_ESP32_SPI3
static const struct esp32_spislv_config_s esp32_spi3_config =
{
  .reg_base     = REG_SPI_BASE(3),
  .mode         = SPIDEV_MODE0,
  .cs_pin       = CONFIG_ESP32_SPI3_CSPIN,
  .mosi_pin     = CONFIG_ESP32_SPI3_MOSIPIN,
  .miso_pin     = CONFIG_ESP32_SPI3_MISOPIN,
  .clk_pin      = CONFIG_ESP32_SPI3_CLKPIN,
  .cpu          = 0,
  .periph       = ESP32_PERIPH_SPI3,
  .irq          = ESP32_IRQ_SPI3,
  .clk_bit      = DPORT_SPI_CLK_EN,
  .rst_bit      = DPORT_SPI_RST,
  .cs_insig     = VSPICS0_IN_IDX,
  .cs_outsig    = VSPICS0_OUT_IDX,
  .mosi_insig   = VSPID_IN_IDX,
  .mosi_outsig  = VSPID_OUT_IDX,
  .miso_insig   = VSPIQ_IN_IDX,
  .miso_outsig  = VSPIQ_OUT_IDX,
  .clk_insig    = VSPICLK_IN_IDX,
  .clk_outsig   = VSPICLK_OUT_MUX_IDX
};

static const struct spi_sctrlrops_s esp32_spi3slv_ops =
{
  .bind     = esp32_spislv_bind,
  .unbind   = esp32_spislv_unbind,
  .enqueue  = esp32_spislv_enqueue,
  .qfull    = esp32_spislv_qfull,
  .qflush   = esp32_spislv_qflush
};

static struct esp32_spislv_priv_s esp32_spi3slv_priv =
{
  .sctrlr =
              {
                .ops = &esp32_spi3slv_ops
              },
  .config = &esp32_spi3_config,
  .mode = SPIDEV_MODE3
};
#endif /* CONFIG_ESP32_SPI3 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_spi_set_reg
 *
 * Description:
 *   Set the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *   value  - Value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32_spi_set_reg(struct esp32_spislv_priv_s *priv,
                                     int offset,
                                     uint32_t value)
{
  putreg32(value, priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_spi_get_reg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *
 * Returned Value:
 *   The contents of the register
 *
 ****************************************************************************/

static inline uint32_t esp32_spi_get_reg(struct esp32_spislv_priv_s *priv,
                                         int offset)
{
  return getreg32(priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_spi_set_regbits
 *
 * Description:
 *   Set the bits of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *   bits   - Bits to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32_spi_set_regbits(struct esp32_spislv_priv_s *priv,
                                         int offset,
                                         uint32_t bits)
{
  uint32_t tmp = getreg32(priv->config->reg_base + offset);

  putreg32(tmp | bits, priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_spi_reset_regbits
 *
 * Description:
 *   Clear the bits of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *   bits   - Bits to be cleared
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32_spi_reset_regbits(struct esp32_spislv_priv_s *priv,
                                           int offset,
                                           uint32_t bits)
{
  uint32_t tmp = getreg32(priv->config->reg_base + offset);

  putreg32(tmp & (~bits), priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_spislv_setmode
 *
 * Description:
 *   Set the SPI mode.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void esp32_spislv_setmode(FAR struct spi_sctrlr_s *dev,
                                 enum spi_mode_e mode)
{
  uint32_t ck_idle_edge;
  uint32_t ck_in_edge;
  uint32_t miso_delay_mode;
  uint32_t miso_delay_num;
  uint32_t mosi_delay_mode;
  uint32_t mosi_delay_num;
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)dev;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          ck_idle_edge = 1;
          ck_in_edge = 0;
          miso_delay_mode = 0;
          miso_delay_num = 0;
          mosi_delay_mode = 2;
          mosi_delay_num = 2;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          ck_idle_edge = 1;
          ck_in_edge = 1;
          miso_delay_mode = 2;
          miso_delay_num = 0;
          mosi_delay_mode = 0;
          mosi_delay_num = 0;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          ck_idle_edge = 0;
          ck_in_edge = 1;
          miso_delay_mode = 0;
          miso_delay_num = 0;
          mosi_delay_mode = 1;
          mosi_delay_num = 2;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          ck_idle_edge = 0;
          ck_in_edge = 0;
          miso_delay_mode = 1;
          miso_delay_num = 0;
          mosi_delay_mode = 0;
          mosi_delay_num = 0;
          break;

        default:
          return;
        }

      esp32_spi_reset_regbits(priv,
                              SPI_PIN_OFFSET,
                              SPI_CK_IDLE_EDGE_M);
      esp32_spi_set_regbits(priv,
                            SPI_PIN_OFFSET,
                            (ck_idle_edge << SPI_CK_IDLE_EDGE_S));

      esp32_spi_reset_regbits(priv,
                              SPI_USER_OFFSET,
                              SPI_CK_I_EDGE_M);
      esp32_spi_set_regbits(priv,
                            SPI_USER_OFFSET,
                            (ck_in_edge << SPI_CK_I_EDGE_S));

      esp32_spi_reset_regbits(priv,
                              SPI_CTRL2_OFFSET,
                              SPI_MISO_DELAY_MODE_M |
                              SPI_MISO_DELAY_NUM_M |
                              SPI_MOSI_DELAY_NUM_M |
                              SPI_MOSI_DELAY_MODE_M);
      esp32_spi_set_regbits(priv,
                            SPI_CTRL2_OFFSET,
                            (miso_delay_mode << SPI_MISO_DELAY_MODE_S) |
                            (miso_delay_num  << SPI_MISO_DELAY_NUM_S) |
                            (mosi_delay_mode << SPI_MOSI_DELAY_MODE_S) |
                            (mosi_delay_num  << SPI_MOSI_DELAY_NUM_S));

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: esp32_spislv_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits in an SPI word.
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void esp32_spislv_setbits(FAR struct spi_sctrlr_s *dev, int nbits)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  if (nbits != priv->nbits)
    {
      priv->nbits = nbits;

      esp32_spi_set_reg(priv, SPI_SLV_WRBUF_DLEN_OFFSET, nbits - 1);
      esp32_spi_set_reg(priv, SPI_SLV_RDBUF_DLEN_OFFSET, nbits - 1);
    }
}

/****************************************************************************
 * Name: spi_dequeue
 *
 * Description:
 *   Get the next queued output value.
 *
 * Input Parameters:
 *   priv  - SPI controller CS state
 *   pdata - output data buffer
 *
 * Returned Value:
 *   true if there is data or fail
 *
 * Assumptions:
 *   Called only from the SPI interrupt handler so all interrupts are
 *   disabled.
 *
 ****************************************************************************/

static int spi_dequeue(struct esp32_spislv_priv_s *priv, uint32_t *pdata)
{
  int ret = false;
  int next;

  if (priv->head != priv->tail)
    {
      *pdata = priv->outq[priv->tail];

      next = priv->tail + 1;
      if (next >= CONFIG_ESP_SPI_SLAVE_QSIZE)
        {
          next = 0;
        }

      priv->tail = next;

      ret = true;
    }

  return ret;
}

/****************************************************************************
 * Name: esp32_io_interrupt
 *
 * Description:
 *   Common I/O interrupt handler
 *
 * Input Parameters:
 *   arg - I/O controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int esp32_io_interrupt(int irq, void *context, FAR void *arg)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)arg;

  priv->process = false;
  SPI_SDEV_SELECT(priv->sdev, false);

  return 0;
}

/****************************************************************************
 * Name: esp32_spislv_interrupt
 *
 * Description:
 *   Common SPI interrupt handler
 *
 * Input Parameters:
 *   arg - SPI controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int esp32_spislv_interrupt(int irq, void *context, FAR void *arg)
{
  uint32_t rd;
  uint32_t wd;
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)arg;
  irqstate_t flags;

  flags = enter_critical_section();

  esp32_spi_reset_regbits(priv, SPI_SLAVE_OFFSET, SPI_TRANS_DONE_M);

  if (!priv->process)
    {
      SPI_SDEV_SELECT(priv->sdev, true);
      priv->process = true;
    }

  rd = esp32_spi_get_reg(priv, SPI_W0_OFFSET);
  if (spi_dequeue(priv, &wd))
    {
      esp32_spi_set_reg(priv, SPI_W0_OFFSET, wd);
      esp32_spi_set_regbits(priv, SPI_CMD_OFFSET, SPI_USR_M);
    }

  SPI_SDEV_RECEIVE(priv->sdev, &rd, 1);

  if (priv->process == false)
    {
      SPI_SDEV_SELECT(priv->sdev, false);
    }

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: esp32_spislv_initialize
 *
 * Description:
 *   Initialize ESP32 SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spislv_initialize(FAR struct spi_sctrlr_s *dev)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)dev;
  const struct esp32_spislv_config_s *config = priv->config;

  esp32_gpiowrite(config->cs_pin, 1);
  esp32_gpiowrite(config->mosi_pin, 1);
  esp32_gpiowrite(config->miso_pin, 1);
  esp32_gpiowrite(config->clk_pin, 1);

  esp32_configgpio(config->cs_pin, INPUT | PULLUP | FUNCTION_2);
  gpio_matrix_out(config->cs_pin, config->cs_outsig, 0, 0);
  gpio_matrix_in(config->cs_pin, config->cs_insig, 0);

  esp32_configgpio(config->mosi_pin, INPUT | PULLUP | FUNCTION_2);
  gpio_matrix_out(config->mosi_pin, config->mosi_outsig, 0, 0);
  gpio_matrix_in(config->mosi_pin, config->mosi_insig, 0);

  esp32_configgpio(config->miso_pin, OUTPUT | PULLUP | FUNCTION_2);
  gpio_matrix_out(config->miso_pin, config->miso_outsig, 0, 0);
  gpio_matrix_in(config->miso_pin, config->miso_insig, 0);

  esp32_configgpio(config->clk_pin, INPUT | PULLUP | FUNCTION_2);
  gpio_matrix_out(config->clk_pin, config->clk_outsig, 0, 0);
  gpio_matrix_in(config->clk_pin, config->clk_insig, 0);

  modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, config->clk_bit);
  modifyreg32(DPORT_PERIP_RST_EN_REG, config->rst_bit, 0);

  esp32_spi_set_reg(priv, SPI_USER_OFFSET, SPI_DOUTDIN_M |
                                           SPI_USR_MOSI_M |
                                           SPI_USR_MISO_M);
  esp32_spi_set_reg(priv, SPI_USER1_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_PIN_OFFSET, SPI_CS1_DIS_M | SPI_CS2_DIS_M);
  esp32_spi_set_reg(priv, SPI_CTRL_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_CTRL2_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_USER2_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_CLOCK_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_SLAVE_OFFSET, SPI_SLAVE_MODE_M |
                                            SPI_SLV_WR_RD_BUF_EN_M |
                                            SPI_INT_EN_M);

  esp32_spislv_setmode(dev, config->mode);
  esp32_spislv_setbits(dev, 8);

  esp32_spi_set_regbits(priv, SPI_SLAVE_OFFSET, SPI_SYNC_RESET_M);
  esp32_spi_reset_regbits(priv, SPI_SLAVE_OFFSET, SPI_SYNC_RESET_M);

  esp32_gpioirqenable(ESP32_PIN2IRQ(config->cs_pin), RISING);
}

/****************************************************************************
 * Name: esp32_spislv_deinit
 *
 * Description:
 *   Deinitialize ESP32 SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spislv_deinit(FAR struct spi_sctrlr_s *dev)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)dev;

  esp32_gpioirqdisable(ESP32_PIN2IRQ(priv->config->cs_pin));
  esp32_spi_reset_regbits(priv, SPI_SLAVE_OFFSET, SPI_INT_EN_M);
  modifyreg32(DPORT_PERIP_CLK_EN_REG, priv->config->clk_bit, 0);
}

/****************************************************************************
 * Name: esp32_spislv_bind
 *
 * Description:
 *   Bind the SPI slave device interface to the SPI slave controller
 *   interface and configure the SPI interface.  Upon return, the SPI
 *   slave controller driver is fully operational and ready to perform
 *   transfers.
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *   sdev   - SPI slave device interface instance
 *   mode   - The SPI mode requested
 *   nbits  - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void esp32_spislv_bind(struct spi_sctrlr_s *sctrlr,
                              struct spi_sdev_s *sdev,
                              enum spi_smode_e mode,
                              int nbits)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)sctrlr;
  irqstate_t flags;
  FAR const void *data;
  int ret;

  spiinfo("sdev=%p mode=%d nbits=%d\n", sdv, mode, nbits);

  DEBUGASSERT(priv != NULL && priv->sdev == NULL && sdev != NULL);

  flags = enter_critical_section();

  priv->sdev = sdev;

  SPI_SDEV_SELECT(sdev, false);

  SPI_SDEV_CMDDATA(sdev, false);

  esp32_spislv_initialize(sctrlr);

  esp32_spislv_setmode(sctrlr, mode);
  esp32_spislv_setbits(sctrlr, nbits);

  priv->head  = 0;
  priv->tail  = 0;

  ret = SPI_SDEV_GETDATA(sdev, &data);
  if (ret == 1)
    {
      priv->outval = *(const uint16_t *)data;
    }

  esp32_spi_set_reg(priv, SPI_W0_OFFSET, priv->outval);
  esp32_spi_set_regbits(priv, SPI_CMD_OFFSET, SPI_USR_M);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32_spislv_unbind
 *
 * Description:
 *   Un-bind the SPI slave device interface from the SPI slave controller
 *   interface.  Reset the SPI interface and restore the SPI slave
 *   controller driver to its initial state,
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void esp32_spislv_unbind(struct spi_sctrlr_s *sctrlr)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)sctrlr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  spiinfo("Unbinding %p\n", priv->sdev);

  DEBUGASSERT(priv->sdev != NULL);

  flags = enter_critical_section();

  priv->sdev = NULL;

  esp32_gpioirqdisable(ESP32_PIN2IRQ(priv->config->cs_pin));
  esp32_spi_reset_regbits(priv, SPI_SLAVE_OFFSET, SPI_INT_EN_M);
  modifyreg32(DPORT_PERIP_CLK_EN_REG, priv->config->clk_bit, 0);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32_spislv_enqueue
 *
 * Description:
 *   Enqueue the next value to be shifted out from the interface.  This adds
 *   the word the controller driver for a subsequent transfer but has no
 *   effect on anyin-process or currently "committed" transfers
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *   data   - Command/data mode data value to be shifted out.  The width of
 *            the data must be the same as the nbits parameter previously
 *            provided to the bind() methods.
 *
 * Returned Value:
 *   Zero if the word was successfully queue; A negated errno valid is
 *   returned on any failure to enqueue the word (such as if the queue is
 *   full).
 *
 ****************************************************************************/

static int esp32_spislv_enqueue(struct spi_sctrlr_s *sctrlr,
                                FAR const void *data,
                                size_t nwords)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)sctrlr;
  irqstate_t flags;
  uint32_t next;
  int ret;

  spiinfo("spi_enqueue(sctrlr=%p, data=%p, nwords=%d)\n",
          sctrlr, data, nwords);
  DEBUGASSERT(priv != NULL && priv->sdev != NULL);

  if (!nwords || nwords > 1)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  next = priv->head + 1;
  if (next >= CONFIG_ESP_SPI_SLAVE_QSIZE)
    {
      next = 0;
    }

  if (next == priv->tail)
    {
      ret = -ENOSPC;
    }
  else
    {
      priv->outq[priv->head] = *(uint16_t *)data;
      priv->head = next;
      ret = OK;

      if (!priv->process)
        {
          esp32_spi_set_regbits(priv, SPI_SLAVE_OFFSET, SPI_TRANS_DONE_M);
        }
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: esp32_spislv_qfull
 *
 * Description:
 *   Return true if the queue is full or false if there is space to add an
 *   additional word to the queue.
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   true if the output wueue is full
 *
 ****************************************************************************/

static bool esp32_spislv_qfull(struct spi_sctrlr_s *sctrlr)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)sctrlr;
  irqstate_t flags;
  uint32_t next;
  bool ret;

  DEBUGASSERT(priv != NULL && priv->sdev != NULL);

  spiinfo("spi_qfull(sctrlr=%p)\n", sctrlr);

  flags = enter_critical_section();
  next = priv->head + 1;
  if (next >= CONFIG_ESP_SPI_SLAVE_QSIZE)
    {
      next = 0;
    }

  ret = (next == priv->tail);
  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: esp32_spislv_qflush
 *
 * Description:
 *   Discard all saved values in the output queue.  On return from this
 *   function the output queue will be empty.  Any in-progress or otherwise
 *   "committed" output values may not be flushed.
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spislv_qflush(struct spi_sctrlr_s *sctrlr)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)sctrlr;
  irqstate_t flags;

  spiinfo("data=%04x\n", data);

  DEBUGASSERT(priv != NULL && priv->sdev != NULL);

  flags = enter_critical_section();
  priv->head = 0;
  priv->tail = 0;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32_spislv_sctrlr_initialize
 *
 * Description:
 *   Initialize the selected SPI slave bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI slave interfaces)
 *
 * Returned Value:
 *   Valid SPI slave device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_sctrlr_s *esp32_spislv_sctrlr_initialize(int port)
{
  int ret;
  FAR struct spi_sctrlr_s *spislv_dev;
  FAR struct esp32_spislv_priv_s *priv;
  irqstate_t flags;

  switch (port)
    {
#ifdef CONFIG_ESP32_SPI2
      case 2:
        priv = &esp32_spi2slv_priv;
        break;
#endif
#ifdef CONFIG_ESP32_SPI3
      case 3:
        priv = &esp32_spi3slv_priv;
        break;
#endif
      default:
        return NULL;
    }

  spislv_dev = (FAR struct spi_sctrlr_s *)priv;

  flags = enter_critical_section();

  if ((volatile int)priv->refs++ != 0)
    {
      leave_critical_section(flags);

      return spislv_dev;
    }

  DEBUGVERIFY(irq_attach(ESP32_PIN2IRQ(priv->config->cs_pin),
                         esp32_io_interrupt,
                         priv));

  priv->cpuint = esp32_alloc_levelint(1);
  if (priv->cpuint < 0)
    {
      leave_critical_section(flags);

      return NULL;
    }

  up_disable_irq(priv->cpuint);
  esp32_attach_peripheral(priv->config->cpu,
                          priv->config->periph,
                          priv->cpuint);

  ret = irq_attach(priv->config->irq, esp32_spislv_interrupt, priv);
  if (ret != OK)
    {
      esp32_detach_peripheral(priv->config->cpu,
                              priv->config->periph,
                              priv->cpuint);
      esp32_free_cpuint(priv->cpuint);

      leave_critical_section(flags);

      return NULL;
    }

  up_enable_irq(priv->cpuint);

  leave_critical_section(flags);

  return spislv_dev;
}

/****************************************************************************
 * Name: esp32_spislv_sctrlr_uninitialize
 *
 * Description:
 *   Uninitialize an SPI slave bus
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   OK if success or fail
 *
 ****************************************************************************/

int esp32_spislv_sctrlr_uninitialize(FAR struct spi_sctrlr_s *sctrlr)
{
  irqstate_t flags;
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)sctrlr;

  DEBUGASSERT(sctrlr);

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

  up_disable_irq(priv->cpuint);

  esp32_spislv_deinit(sctrlr);

  leave_critical_section(flags);

  return OK;
}

#endif /* CONFIG_ESP32_SPI */
