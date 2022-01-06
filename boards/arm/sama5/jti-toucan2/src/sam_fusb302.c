/****************************************************************************
 * boards/arm/sama5/jti-toucan2/src/sam_fusb302.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <arch/board/board.h>
#include "jti-toucan2.h"
#include <nuttx/usb/fusb302.h>
#include <nuttx/irq.h>
//#include <nuttx/kthread.h>


#include "arm_arch.h"
#include "sam_pio.h"
#include "sam_usbhost.h"




#if defined(HAVE_FUSB302) && defined (CONFIG_DEBUG_FUSB302)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int fusb302_irq_attach(FAR struct fusb302_config_s *state,
                                  xcpt_t isr, FAR void *arg);
static void fusb302_irq_enable(FAR struct fusb302_config_s *state, bool enable);

static int fusb302_irq_clear(FAR struct fusb302_config_s *state);                              

/****************************************************************************
 * Private Data
 ****************************************************************************/
//static xcpt_t g_fusb302handler;

/* A reference to a structure of this type must be passed to the FUSB302
 * driver. This structure provides information about the configuration
 * of the FUSB302 and provides some board-specific hooks
 * 
 * memory for this structure is provided by the caller. It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify parameters.
 */
static struct sam_fusb302config_s
{

  /* Configuration as seen by the FUSB302 driver */

  struct fusb302_config_s config;

  /* Additional private definitions only know to this driver */

  FAR void *arg;  /* Argument to pass to the interrupt handler */
  FAR xcpt_t isr; /* Interrupt handler */
};

static struct sam_fusb302config_s g_fusb302config = 
{
  .config =
    {
      .irq_attach = fusb302_irq_attach,
      .irq_enable = fusb302_irq_enable,
      .irq_clear =  fusb302_irq_clear,
    }
};

/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fusb302_irq_attach
 *
 * Description:
 *   Attach the FUSB302 interrupt handler to the GPIO interrupt
 *
 ****************************************************************************/

static int fusb302_irq_attach(FAR struct fusb302_config_s *state,
                                  xcpt_t isr, FAR void *arg)
{
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();
  /* Configure the interrupt */

  sam_pioirq(PIO_FUSB302_INT);
  irq_attach(IRQ_FUSB302_INT, isr, arg);
  sam_pioirqenable(IRQ_FUSB302_INT);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: fusb302_irq_enable
 *
 * Description:
 *   Enable/disable the FUSB302 interrupt handler to the GPIO 
 *
 ****************************************************************************/

static void fusb302_irq_enable(FAR struct fusb302_config_s *state, bool enable)
{
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  if (enable)
    {
      sam_pioirqenable(IRQ_FUSB302_INT);
    }
  else
    {
      sam_pioirqdisable(IRQ_FUSB302_INT);
    }

  leave_critical_section(flags);
}

/* Do not believce we need to do anything to clear the interrupt
/****************************************************************************
 * Name: fusb302_irq_clear
 *
 * Description:
 *   Clear the FUSB302 interrupt 
 *
 ****************************************************************************/

static int fusb302_irq_clear(FAR struct fusb302_config_s *state)
{
  /* nothing to do here */
  return OK;
} 

/****************************************************************************
 * Public Functions
 ****************************************************************************/


/****************************************************************************
 * Name: sam_fusb302init
 *
 * Description:
 *   Called from sam_bringup if there's an FUSB302 device on the board
 *   to set up GPIO pins for the JTI Toucan2 board.
 *
 ****************************************************************************/

int sam_fusb302init(int busno)
//int sam_fusb302init(FAR struct i2c_master_s *i2c, uint8_t fusb302addr)
{
  int ret;
  FAR struct i2c_master_s *i2c;
  uinfo("initialising FUSB302\n");

  //if (i2c == NULL)
    //return -ENODEV;
  /* configure PIO */
  sam_configpio(PIO_FUSB302_INT);

  /* initialise I2C */

  i2c = sam_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  g_fusb302config.config.i2c      = i2c;
  g_fusb302config.config.i2c_addr = FUSB302_I2C_ADDR;

  ret = fusb302_register("/dev/fusb302", &g_fusb302config.config);
  if (ret < 0)
    {
      _err("ERROR: Failed registering FUSB302\n");
    }
  else
    {
      _info("INFO: FUSB302 registered OK\n");
    }
  return ret;
}



#endif /* CONFIG_SAMA5_PIOD_IRQ ... */
