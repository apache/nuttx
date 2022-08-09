/****************************************************************************
 * boards/arm/stm32/stm32f429i-disco/src/stm32_stmpe811.c
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

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/stmpe811.h>

#include <nuttx/irq.h>

#include "stm32.h"
#include "stm32f429i-disco.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_INPUT_STMPE811
#ifndef CONFIG_INPUT
#  error "STMPE811 support requires CONFIG_INPUT"
#endif

#ifndef CONFIG_STM32_I2C3
#  error "STMPE811 support requires CONFIG_STM32_I2C3"
#endif

#ifndef CONFIG_STMPE811_I2C
#  error "Only the STMPE811 I2C interface is supported"
#endif

#ifdef CONFIG_STMPE811_SPI
#  error "Only the STMPE811 SPI interface is supported"
#endif

#ifndef CONFIG_STMPE811_FREQUENCY
#  define CONFIG_STMPE811_FREQUENCY 100000
#endif

#ifndef CONFIG_STMPE811_I2CDEV
#  define CONFIG_STMPE811_I2CDEV 3
#endif

#if CONFIG_STMPE811_I2CDEV != 3
#  error "CONFIG_STMPE811_I2CDEV must be three"
#endif

#ifndef CONFIG_STMPE811_DEVMINOR
#  define CONFIG_STMPE811_DEVMINOR 0
#endif

/* Board definitions ********************************************************/

/* The STM3240G-EVAL has two STMPE811QTR I/O expanders on board both
 * connected to the STM32 via I2C1.  They share a common interrupt line: PI2.
 *
 * STMPE811 U24, I2C address 0x41 (7-bit)
 * ------ ---- ---------------- --------------------------------------------
 * STPE11 PIN  BOARD SIGNAL     BOARD CONNECTION
 * ------ ---- ---------------- --------------------------------------------
 *   Y-        TouchScreen_Y-   LCD Connector XL
 *   X-        TouchScreen_X-   LCD Connector XR
 *   Y+        TouchScreen_Y+   LCD Connector XD
 *   X+        TouchScreen_X+   LCD Connector XU
 *   IN3       EXP_IO9
 *   IN2       EXP_IO10
 *   IN1       EXP_IO11
 *   IN0       EXP_IO12
 *
 * STMPE811 U29, I2C address 0x44 (7-bit)
 * ------ ---- ---------------- --------------------------------------------
 * STPE11 PIN  BOARD SIGNAL     BOARD CONNECTION
 * ------ ---- ---------------- --------------------------------------------
 *   Y-        EXP_IO1
 *   X-        EXP_IO2
 *   Y+        EXP_IO3
 *   X+        EXP_IO4
 *   IN3       EXP_IO5
 *   IN2       EXP_IO6
 *   IN1       EXP_IO7
 *   IN0       EXP_IO8
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_stmpe811config_s
{
  /* Configuration structure as seen by the STMPE811 driver */

  struct stmpe811_config_s config;

  /* Additional private definitions only known to this driver */

  STMPE811_HANDLE handle;   /* The STMPE811 driver handle */
  xcpt_t          handler;  /* The STMPE811 interrupt handler */
  void           *arg;      /* Interrupt handler argument */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the STMPE811 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.* so that contact and loss-
 * of-contact events can be detected.
 *
 * attach  - Attach the STMPE811 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int  stmpe811_attach(struct stmpe811_config_s *state, xcpt_t isr,
                            void *arg);
static void stmpe811_enable(struct stmpe811_config_s *state,
                            bool enable);
static void stmpe811_clear(struct stmpe811_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the STMPE811
 * driver.  This structure provides information about the configuration
 * of the STMPE811 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

#ifndef CONFIG_STMPE811_TSC_DISABLE
static struct stm32_stmpe811config_s g_stmpe811config =
{
  .config =
  {
#ifdef CONFIG_STMPE811_I2C
    .address   = STMPE811_ADDR1,
#endif
    .frequency = CONFIG_STMPE811_FREQUENCY,

#ifdef CONFIG_STMPE811_MULTIPLE
    .irq       = STM32_IRQ_EXTI2,
#endif
    .ctrl1     = (ADC_CTRL1_SAMPLE_TIME_80 | ADC_CTRL1_MOD_12B),
    .ctrl2     = ADC_CTRL2_ADC_FREQ_3p25,

    .attach    = stmpe811_attach,
    .enable    = stmpe811_enable,
    .clear     = stmpe811_clear,
  },
  .handler     = NULL,
  .arg         = NULL,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the STMPE811 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.
 *
 * attach  - Attach the STMPE811 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int stmpe811_attach(struct stmpe811_config_s *state, xcpt_t isr,
                           void *arg)
{
  struct stm32_stmpe811config_s *priv =
    (struct stm32_stmpe811config_s *)state;

  iinfo("Saving handler %p\n", isr);
  DEBUGASSERT(priv);

  /* Just save the handler.  We will use it when EXTI interruptsare enabled */

  priv->handler = isr;
  priv->arg     = arg;
  return OK;
}

static void stmpe811_enable(struct stmpe811_config_s *state, bool enable)
{
  struct stm32_stmpe811config_s *priv =
    (struct stm32_stmpe811config_s *)state;
  irqstate_t flags;

  /* Attach and enable, or detach and disable.  Enabling and disabling GPIO
   * interrupts is a multi-step process so the safest thing is to keep
   * interrupts disabled during the reconfiguration.
   */

  flags = enter_critical_section();
  if (enable)
    {
      /* Configure the EXTI interrupt using the SAVED handler */

      stm32_gpiosetevent(GPIO_IO_EXPANDER, true, true, true,
                         priv->handler, priv->arg);
    }
  else
    {
      /* Configure the EXTI interrupt with a NULL handler to disable it */

     stm32_gpiosetevent(GPIO_IO_EXPANDER, false, false, false,
                        NULL, NULL);
    }

  leave_critical_section(flags);
}

static void stmpe811_clear(struct stmpe811_config_s *state)
{
  /* Does nothing */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_tsc_setup(int minor)
{
#ifndef CONFIG_STMPE811_TSC_DISABLE
  struct i2c_master_s *dev;
  int ret;

  iinfo("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Check if we are already initialized */

  if (!g_stmpe811config.handle)
    {
      iinfo("Initializing\n");

      /* Configure the STMPE811 interrupt pin as an input */

      stm32_configgpio(GPIO_IO_EXPANDER);

      /* Get an instance of the I2C interface */

      dev = stm32_i2cbus_initialize(CONFIG_STMPE811_I2CDEV);
      if (!dev)
        {
          ierr("ERROR: Failed to initialize I2C bus %d\n",
               CONFIG_STMPE811_I2CDEV);
          return -ENODEV;
        }

      /* Instantiate the STMPE811 driver */

      g_stmpe811config.handle =
        stmpe811_instantiate(dev,
                          (struct stmpe811_config_s *)&g_stmpe811config);
      if (!g_stmpe811config.handle)
        {
          ierr("ERROR: Failed to instantiate the STMPE811 driver\n");
          return -ENODEV;
        }

      /* Initialize and register the I2C touchscreen device */

      ret = stmpe811_register(g_stmpe811config.handle,
                              CONFIG_STMPE811_DEVMINOR);
      if (ret < 0)
        {
          ierr("ERROR: Failed to register STMPE driver: %d\n", ret);

          /* stm32_i2cbus_uninitialize(dev); */

          return -ENODEV;
        }
    }

  return OK;
#else
  return -ENOSYS;
#endif
}

#endif /* CONFIG_INPUT_STMPE811 */
