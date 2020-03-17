/****************************************************************************
 * boards/arm/stm32/nucleo-f4x1re/src/stm32_mcp2515.c
 *
 *   Copyright (C) 2017 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/can/mcp2515.h>

#include "stm32.h"
#include "stm32_spi.h"
#include "nucleo-f4x1re.h"

#if defined(CONFIG_SPI) && defined(CONFIG_STM32_SPI1) && \
    defined(CONFIG_CAN_MCP2515)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MCP2515_SPI_PORTNO 1   /* On SPI1 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_mcp2515config_s
{
  /* Configuration structure as seen by the MCP2515 driver */

  struct mcp2515_config_s config;

  /* Additional private definitions only known to this driver */

  FAR struct mcp2515_can_s *handle; /* The MCP2515 driver handle */
  mcp2515_handler_t handler;        /* The MCP2515 interrupt handler */
  FAR void *arg;                    /* Argument to pass to the interrupt handler */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the MCP2515 driver from differences in GPIO interrupt handling
 * by varying boards and MCUs.
 *
 *   attach  - Attach the MCP2515 interrupt handler to the GPIO interrupt
 */

static int  mcp2515_attach(FAR struct mcp2515_config_s *state,
                           mcp2515_handler_t handler, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the MCP2515
 * driver.  This structure provides information about the configuration
 * of the MCP2515 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct stm32_mcp2515config_s g_mcp2515config =
{
  .config =
  {
    .spi        = NULL,
    .baud       = 0,     /* REVISIT.  Probably broken by commit eb7373cedfa */
    .btp        = 0,     /* REVISIT.  Probably broken by commit eb7373cedfa */
    .devid      = 0,
    .mode       = 0,     /* REVISIT.  Probably broken by commit eb7373cedfa */
    .nfilters   = 6,
#ifdef MCP2515_LOOPBACK
    .loopback   = false;
#endif
    .attach     = mcp2515_attach,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* This is the MCP2515 Interrupt handler */

int mcp2515_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct stm32_mcp2515config_s *priv =
             (FAR struct stm32_mcp2515config_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Verify that we have a handler attached */

  if (priv->handler)
    {
      /* Yes.. forward with interrupt along with its argument */

      priv->handler(&priv->config, priv->arg);
    }

  return OK;
}

static int mcp2515_attach(FAR struct mcp2515_config_s *state,
                          mcp2515_handler_t handler, FAR void *arg)
{
  FAR struct stm32_mcp2515config_s *priv =
             (FAR struct stm32_mcp2515config_s *)state;
  irqstate_t flags;

  caninfo("Saving handler %p\n", handler);

  flags = enter_critical_section();

  priv->handler = handler;
  priv->arg = arg;

  /* Configure the interrupt for falling edge */

  stm32_gpiosetevent(GPIO_MCP2515_IRQ, false, true, false,
                     mcp2515_interrupt, priv);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mcp2515initialize
 *
 * Description:
 *   Initialize and register the MCP2515 RFID driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/rfid0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_mcp2515initialize(FAR const char *devpath)
{
  FAR struct spi_dev_s     *spi;
  FAR struct can_dev_s     *can;
  FAR struct mcp2515_can_s *mcp2515;
  int ret;

  /* Check if we are already initialized */

  if (!g_mcp2515config.handle)
    {
      sninfo("Initializing\n");

      /* Configure the MCP2515 interrupt pin as an input */

      stm32_configgpio(GPIO_MCP2515_IRQ);

      spi = stm32_spibus_initialize(MCP2515_SPI_PORTNO);

      if (!spi)
        {
          return -ENODEV;
        }

      /* Save the SPI instance in the mcp2515_config_s structure */

      g_mcp2515config.config.spi = spi;

      /* Instantiate the MCP2515 CAN Driver */

      mcp2515 = mcp2515_instantiate(&g_mcp2515config.config);
      if (mcp2515 == NULL)
        {
          canerr("ERROR:  Failed to get MCP2515 Driver Loaded\n");
          return -ENODEV;
        }

      /* Save the opaque structure */

      g_mcp2515config.handle = mcp2515;

      /* Initialize the CAN Device with the MCP2515 operations */

      can = mcp2515_initialize(mcp2515);
      if (can == NULL)
        {
          canerr("ERROR:  Failed to get CAN interface\n");
          return -ENODEV;
        }

      /* Register the CAN driver at "/dev/can0" */

      ret = can_register(devpath, can);
      if (ret < 0)
        {
          canerr("ERROR: can_register failed: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_SPI && CONFIG_CAN_MCP2515 */
