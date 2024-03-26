/****************************************************************************
 * boards/risc-v/esp32c3-legacy/common/src/esp32c3_board_twai.c
 *
 * Board-specific layer for TWAI CAN controller
 * on ESP32 C3 (via TWAI interface).
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arch/board/board.h>
#include <nuttx/can/can.h>
#include <nuttx/irq.h>

#include "chip.h"

#include <nuttx/can/sja1000.h>

#include <nuttx/signal.h>

#include "riscv_internal.h"

#include "esp32c3_clockconfig.h"
#include "esp32c3_gpio.h"
#include "esp32c3_irq.h"

#include "hardware/esp32c3_gpio_sigmap.h"
#include "hardware/esp32c3_system.h"

#include "esp32c3_board_twai.h"

#ifdef CONFIG_ESP32C3_TWAI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sampling of BUS 3 times */

#  ifndef CONFIG_CAN_SJA1000_SAM
#    define CONFIG_CAN_SJA1000_SAM  false
#  endif

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing SJA1000 */

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_CANBUS_REGDEBUG
#endif

#if defined(CONFIG_CAN_SJA1000_DEBUG)
#  define cantrace _info
#else
#  define cantrace _none
#endif /* CONFIG_CAN_SJA1000_DEBUG */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32c3_twai_config_s
{
  /* Configuration structure as seen by the SJA1000 driver */

  struct sja1000_config_s config;

  /* Additional private definitions only known to this driver */

  struct sja1000_dev_s *handle; /* The SJA1000 driver handle */
  sja1000_handler_t handler;    /* The SJA1000 interrupt handler */
  void *arg;                    /* Argument to pass interrupt handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ access callbacks.  These operations all hidden behind callbacks
 * to isolate the SJA1000 driver from differences in interrupt handling
 * by varying boards and MCUs.
 *
 *   twai_attach  - Attach the TWAI interrupt handler to the interrupt
 *   twai_detach  - Detach the TWAI interrupt handler from interrupt
 */

static int twai_attach(struct sja1000_config_s *config,
                           sja1000_handler_t handler, void *arg);

static int twai_detach(struct sja1000_config_s *config);

static uint32_t twai_getreg(struct sja1000_dev_s *sja_priv,
                               uint32_t reg);

static void twai_putreg(struct sja1000_dev_s *sja_priv,
                           uint32_t reg, uint32_t value);

static void twai_modifyreg32(struct sja1000_dev_s *sja_priv,
      uint32_t reg, uint32_t clearbits, uint32_t setbits);

static int twai_init_gpio(FAR struct sja1000_config_s *config);

static int board_twai_initialise(int devno);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* CAN hardware-dependent bit-timing constant
 * Used for calculating and checking bit-timing parameters
 */

static const struct can_bittiming_const_s esp32c3_twai_bittiming_const =
{
  .tseg1_min = 1,
  .tseg1_max = 16,
  .tseg2_min = 1,
  .tseg2_max = 8,
  .sjw_max   = 3,
  .brp_min   = 1,
  .brp_max   = 64,
  .brp_inc   = 1,
};

/* A reference to a structure of this type must be passed to the SJA1000
 * driver.  This structure provides information about the configuration
 * of the SJA1000 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct esp32c3_twai_config_s
    g_esp32c3_twai_config[] =
{
#ifdef CONFIG_ESP32C3_TWAI0
    {
      .config =
        {
          .bittiming_const = &esp32c3_twai_bittiming_const,
          .port     = 0,
          .periph   = ESP32C3_PERIPH_TWAI,
          .irq      = ESP32C3_IRQ_TWAI,
          .cpuint   = -ENOMEM,
          .bitrate  = CONFIG_CAN_SJA1000_BITRATE,
          .samplep  = CONFIG_CAN_SJA1000_SAMPLEP,
          .sjw      = CONFIG_CAN_SJA1000_SJW,
#ifdef CONFIG_CAN_LOOPBACK
          .loopback = true,
#else
          .loopback = false,
#endif /* #ifdef CONFIG_CAN_LOOPBACK */
          .triple_sample = CONFIG_CAN_SJA1000_SAM,
          .attach        = twai_attach,
          .detach        = twai_detach,
        },
    },
#endif /* #ifdef CONFIG_ESP32C3_TWAI0 */
#ifdef CONFIG_ESP32C3_TWAI1
#  error "TODO - TWAI1 not yet defined"
#endif /* #ifdef CONFIG_ESP32C3_TWAI1 */
};

static struct sja1000_dev_s g_twai_priv[] =
{
#ifdef CONFIG_ESP32C3_TWAI0
    {
      .getreg = twai_getreg,
      .putreg = twai_putreg,
      .modifyreg32 = twai_modifyreg32,
      .config = &g_esp32c3_twai_config[0].config,
      .base = DR_REG_TWAI_BASE,
    },
#endif /* #ifdef CONFIG_ESP32C3_TWAI0 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_twai_interrupt
 *
 * Description:
 *   This is the TWAI Interrupt handler.
 *
 * Input Parameters:
 *   irq - Interrupt number
 *   context - interrupt context
 *   arg - pointer to sja1000_dev_s (lower CAN driver) context.
 *
 * Returned Value:
 *   OK on success or negated error value.
 *
 ****************************************************************************/

int esp32c3_twai_interrupt(int irq, void *context, void *arg)
{
  struct sja1000_dev_s *priv = (struct sja1000_dev_s *)arg;
  struct esp32c3_twai_config_s *esp32c3_twai_config
      = &g_esp32c3_twai_config[priv->config->port];

  cantrace("Entered for CAN port %" PRId8 ".\n", priv->config->port);
  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(esp32c3_twai_config != NULL);
  DEBUGASSERT(esp32c3_twai_config->handler != NULL);
  DEBUGASSERT(priv->config != NULL);
  DEBUGASSERT(esp32c3_twai_config->arg);

  esp32c3_twai_config->handler(priv->config, esp32c3_twai_config->arg);

  return OK;
}

/****************************************************************************
 * Name: twai_detach
 *
 * Description:
 *   Attach SJA1000 instance to interrupt.
 *
 * Input Parameters:
 *   config - Configuration instance of the SJA1000 driver to attach
 *   handler - interrupt callback hander
 *   arg - argument to register for passing to interrupt handler.
 *
 * Returned Value:
 *   OK on success or negated error value.
 *
 ****************************************************************************/

static int twai_attach(struct sja1000_config_s *config,
                          sja1000_handler_t handler, void *arg)
{
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  struct sja1000_dev_s *priv = dev->cd_priv;
  irqstate_t flags;
  int devno = config->port;
  DEBUGASSERT(devno < ESP32C3_TWAI_NUM_PORTS);
  int ret;
  struct esp32c3_twai_config_s *esp32c3_twai_config =
      &g_esp32c3_twai_config[devno];

  DEBUGASSERT(esp32c3_twai_config);

  flags = enter_critical_section();

  cantrace("Saving handler %p, arg %p for port %" PRId8 ".\n",
      handler, arg, devno);

  esp32c3_twai_config->handler = handler;
  esp32c3_twai_config->arg = arg;

  if (config->cpuint >= 0)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(config->irq);
    }

  config->cpuint = esp32c3_setup_irq(
      config->periph, ESP32C3_INT_PRIO_DEF, ESP32C3_INT_LEVEL);
  if (config->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      ret = config->cpuint;
      leave_critical_section(flags);

      return ret;
    }

  ret = irq_attach(config->irq, esp32c3_twai_interrupt, priv);

  if (ret != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      canerr("Failed to initialize device %d.\n", devno);
      esp32c3_teardown_irq(config->periph, config->cpuint);
      config->cpuint = -ENOMEM;
      leave_critical_section(flags);

      return ret;
    }

  /* Enable the CPU interrupt that is linked to the TWAI device. */

  up_enable_irq(config->irq);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: twai_detach
 *
 * Description:
 *   Detach from interrupt.
 *
 * Input Parameters:
 *   config - Configuration instance of the SJA1000 driver to detach
 *
 * Returned Value:
 *   OK on success or negated error value.
 *
 ****************************************************************************/

static int twai_detach(struct sja1000_config_s *config)
{
  irqstate_t flags;
  int devno = config->port;
  DEBUGASSERT(devno < ESP32C3_TWAI_NUM_PORTS);
  struct esp32c3_twai_config_s *esp32c3_twai_config =
      &g_esp32c3_twai_config[devno];

  DEBUGASSERT(esp32c3_twai_config);

  flags = enter_critical_section();

  cantrace("Detaching interrupts for port %" PRId8 ".\n", devno);

  esp32c3_twai_config->handler = NULL;
  esp32c3_twai_config->arg     = NULL;

  if (config->cpuint >= 0)
    {
      /* Disable cpu interrupt */

      up_disable_irq(config->irq);

      /* Dissociate the IRQ from the ISR */

      irq_detach(config->irq);

      config->cpuint = -ENOMEM;
    }

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: twai_getreg
 *
 * Description:
 *   Read the value of an TWAI register.
 *
 * Input Parameters:
 *   priv - sja1000 lower-half driver context
 *   reg - The register address to read
 *
 * Returned Value:
 *
 ****************************************************************************/

static uint32_t twai_getreg(struct sja1000_dev_s *sja_priv,
      uint32_t reg)
{
  uint32_t value;
  uint32_t mem_addr = (sja_priv->base + (reg << 2));

  /* Read the value from the register */

  value = getreg32(mem_addr);
#ifdef CONFIG_CANBUS_REGDEBUG
  cantrace("%08" PRIx32 " -> %08" PRIx32 "\n", mem_addr, value);
#endif /* CONFIG_CANBUS_REGDEBUG */
  return value;
}

/****************************************************************************
 * Name: twai_putreg
 *
 * Description:
 *   Set the value of an TWAI register.
 *
 * Input Parameters:
 *   priv - sja1000 lower-half driver context
 *   reg - The register address to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void twai_putreg(
    struct sja1000_dev_s *sja_priv, uint32_t reg, uint32_t value)
{
  uint32_t mem_addr = (sja_priv->base + (reg << 2));

#ifdef CONFIG_CANBUS_REGDEBUG
  /* Show the register value being written */

  cantrace("%08" PRIx32 " <- %08" PRIx32 "\n", mem_addr, value);
#endif /* CONFIG_CANBUS_REGDEBUG */

  /* Write the value */

  putreg32(value, mem_addr);
}

/****************************************************************************
 * Name: twai_modifyreg32
 *
 * Description:
 *   Modify the value of an TWAI register.
 *
 * Input Parameters:
 *   priv - sja1000 lower-half driver context
 *   reg - The address to the register to modify
 *   clearbits - Bitmask of the bits to clear in a register
 *   setbits - Bitmask of the bits to set in a register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void twai_modifyreg32(
      struct sja1000_dev_s *sja_priv, uint32_t reg, uint32_t clearbits,
      uint32_t setbits)
{
  uint32_t mem_addr = (sja_priv->base + (reg << 2));

#ifdef CONFIG_CANBUS_REGDEBUG
  /* Show the register value being modified */

  cantrace("%08" PRIx32" <- %08" PRIx32 ", %08" PRIx32 "\n",
          mem_addr, clearbits, setbits);
#endif /* CONFIG_CANBUS_REGDEBUG */

  /* Write the value */

  modifyreg32(mem_addr, clearbits, setbits);
}

/****************************************************************************
 * Name: twai_init_gpio
 *
 * Description:
 *   Initialise GPIO pin/pad settings of TWAI controller for this platform
 *
 * Input Parameters:
 *   config - configuration context for SJA1000/TWAI driver
 *
 * Returned Value:
 *   OK on success, else negated error value.
 *
 ****************************************************************************/

static int twai_init_gpio(FAR struct sja1000_config_s *config)
{
  int ret = OK;

  irqstate_t flags;

  caninfo("TWAI%" PRIu8 "\n", config->port);

  flags = enter_critical_section();

  if (config->port == 0)
    {
      /* Enable power to the TWAI module and
       * Enable clocking to the TWAI module
       */

      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, SYSTEM_TWAI_RST_M);
      modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_TWAI_CLK_EN_M, 0);

      nxsig_usleep(1);

      modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, SYSTEM_TWAI_CLK_EN_M);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_TWAI_RST_M, 0);

      /* Configure CAN GPIO pins */

      esp32c3_gpio_matrix_out(
          CONFIG_ESP32C3_TWAI0_TXPIN, TWAI_TX_IDX, 0, 0);
      esp32c3_configgpio(CONFIG_ESP32C3_TWAI0_TXPIN, OUTPUT_FUNCTION_1);

      esp32c3_configgpio(CONFIG_ESP32C3_TWAI0_RXPIN, INPUT_FUNCTION_1);
      esp32c3_gpio_matrix_in(
          CONFIG_ESP32C3_TWAI0_RXPIN, TWAI_RX_IDX, 0);
    }
  else
    {
      canerr("ERROR: Unsupported port: %d\n", config->port);
      ret = -EIO;
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: board_twai_initialise
 *
 * Description:
 *  Initialize specific TWAI device and register the TWAI device
 *
 * Input Parameters:
 *   devno - Peripheral  SJA1000/TWAI device index to initialise
 *
 * Returned Value:
 *   OK on success, else negated error value.
 *
 ****************************************************************************/

static int board_twai_initialise(int devno)
{
  struct esp32c3_twai_config_s *twai_config;
  struct sja1000_dev_s *priv = &g_twai_priv[devno];
  int ret;

  DEBUGASSERT(&g_esp32c3_twai_config[devno] != NULL);

  char devpath[16] =
    {
      0
    };

  snprintf(devpath, sizeof(devpath), "/dev/can%d", devno);

  twai_config = &g_esp32c3_twai_config[devno];

  /* Check if we are already initialized */

  if (!twai_config->handle)
    {
      caninfo("Initializing device %" PRId8 ".\n", devno);

      twai_config->config.clk_freq = esp32c3_clk_apb_freq(),
      twai_config->config.cpu      = up_cpu_index(),

      twai_config->handle = priv;

      /* Initialise GPIOs */

      ret = twai_init_gpio(&twai_config->config);
      if (ret != OK)
        {
          canerr("ERROR:  Failed to initialise TWAI GPIOs.");
          return -EIO;
        }

      /* Instantiate the SJA1000 CAN Driver */

      struct can_dev_s *dev = sja1000_instantiate(priv);
      if (dev == NULL)
        {
          canerr("ERROR:  Failed to get SJA1000 Driver Loaded");
          return -ENODEV;
        }

      dev->cd_priv = priv;

      /* Register the CAN driver at "/dev/can0" */

      ret = can_register(devpath, dev);
      if (ret < 0)
        {
          canerr("ERROR: can_register failed: %d.", ret);
          return ret;
        }
    }
  else
    {
      caninfo("Device %s already initialised.", devpath);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_twai_setup
 *
 * Description:
 *  Initialize TWAI and register the TWAI devices
 *
 * Returned Value:
 *   OK on success, else negated error value.
 *
 ****************************************************************************/

int board_twai_setup(void)
{
  int ret = OK;
  int devno;

#ifdef CONFIG_ESP32C3_TWAI0
  devno = 0;

  ret = board_twai_initialise(devno);
#endif /* CONFIG_ESP32C3_TWAI0 */

#ifdef CONFIG_ESP32C3_TWAI1
  devno = 1;

  ret = board_twai_initialise(devno);
#endif /* CONFIG_ESP32C3_TWAI1 */

  return ret;
}

#endif /* CONFIG_ESP32C3_TWAI */
