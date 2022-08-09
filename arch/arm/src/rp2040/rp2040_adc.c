/****************************************************************************
 * arch/arm/src/rp2040/rp2040_adc.c
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
 * Notes:
 *    The ADC driver upper-half returns signed values of up to 32-bits to
 *    the user and expects the high-order bits in any result to be
 *    significant.  So, while the RP2040 hardware returns an unsigned value
 *    in the low-order 12 bits of the result register, we shift this
 *    value to the high-order bits.
 *
 *    The result is that to convert a 32-bit value returned from the ADC
 *    driver, you should use   V = ADC_AVDD * value / (2^31)  where ADC_AVDD
 *    is the analogue reference voltage supplied to the RP2040 chip.  If
 *    8 or 16 bit values are returned the divisor would be (2^15) or (2^7)
 *    respectively.
 *
 *    Also, if the conversion error bit was set for a particular sample,
 *    the return value will be negated.  Any negative return value should
 *    be treated as erroneous.
 *
 *    -------------
 *
 *    This lower-half supports multiple drivers (/dev/adc0, /dav/dca1, etc.)
 *    that each may read data from any of the ADC ports.  The driver reads
 *    whichever ADC ports are needed by ANY of ther drivers in strict
 *    round-robin fashion, passing the converted values to the drivers that
 *    needed it.  Data is only passed if the driver is open.
 *
 *    --------------
 *
 *    This code reads the ADC ports at full speed.  At the time this comment
 *    was written, the upper-half will throw away any converted values it
 *    receives when the buffer is full; therefor, if the data is not read
 *    for a while, the returned values may be stale when finally read. You
 *    can use the ANIOC_RESET_FIFO ioctl call to flush this stale data.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <rp2040_adc.h>

#include <stdlib.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "arm_internal.h"

#include <rp2040_gpio.h>

#include <hardware/rp2040_adc.h>

#include <nuttx/analog/adc.h>
#include <nuttx/kmalloc.h>

#ifdef CONFIG_RP2040_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_CHANNEL_COUNT  5
#define ADC_TEMP_CHANNEL   4

/* Get the private data pointer from a device pointer */

#define PRIV(x) ((FAR struct private_s *)(x)->ad_priv)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct private_s
{
  FAR const struct adc_callback_s *callback;     /* ADC Callback Structure */
  FAR FAR struct   adc_dev_s      *next_device;
  FAR FAR struct   adc_dev_s      *prior_device;
  bool                             has_channel[ADC_CHANNEL_COUNT];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int  interrupt_handler(int       irq,
                              FAR void *context,
                              FAR void *arg);

static void get_next_channel(void);
static void add_device(FAR struct adc_dev_s *dev);
static void remove_device(FAR struct adc_dev_s *dev);

/* ADC methods */

static int  my_bind(FAR struct adc_dev_s            *dev,
                   FAR const struct adc_callback_s *callback);

static void my_reset(FAR struct adc_dev_s *dev);
static int  my_setup(FAR struct adc_dev_s *dev);
static void my_shutdown(FAR struct adc_dev_s *dev);
static void my_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  my_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = my_bind,     /*  Called first  during initialization. */
  .ao_reset    = my_reset,    /*  Called second during intialization.  */
  .ao_setup    = my_setup,    /*  Called during first open.            */
  .ao_shutdown = my_shutdown, /*  Called during last close.            */
  .ao_rxint    = my_rxint,    /*  Called to enable/disable interrupts. */
  .ao_ioctl    = my_ioctl,    /*  Called for custom ioctls.            */
};

static const int8_t g_gpio_map[ADC_CHANNEL_COUNT] =
{
  26, 27, 28, 29, -1
};

static FAR struct adc_dev_s *g_first_device     = NULL;

static uint8_t               g_current_channel  = 0xf0; /* too big */
static uint8_t               g_active_count     = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: interrupt_handler
 *
 * Description:
 *   ADC interrupt handler.  Note that this one handler is shared between
 *   all ADC devices.
 *
 *  Note: This is called from inside an interrupt service routine.
 ****************************************************************************/

static int interrupt_handler(int irq, void *context, void *arg)
{
  FAR struct adc_dev_s            *a_device;
  FAR const struct adc_callback_s *a_callback;
  int32_t                          value;
  bool                             error_bit_set;

  if (g_active_count == 0)
    {
      /* Last device has been removed -- turn off ADC */

      putreg32(0, RP2040_ADC_CS);

      putreg32(0, RP2040_ADC_INTE);

      up_disable_irq(RP2040_ADC_IRQ_FIFO);

      irq_detach(RP2040_ADC_IRQ_FIFO);

      /* Flush the FIFO */

      while (getreg32(RP2040_ADC_FCS) & RP2040_ADC_FCS_LEVEL_MASK)
        {
          getreg32(RP2040_ADC_FIFO);
        }

      return OK;
    }

  /* Fetch the data from the FIFO register */

  value         = getreg32(RP2040_ADC_FIFO);
  error_bit_set = (value & RP2040_ADC_FIFO_ERR) != 0;

  /* Shift value to top of signed 32-bit word for upper-halfs benefit. */

  value <<= 19;

  if (error_bit_set)
    {
      value = -value;
    }

  for (a_device = g_first_device;
       a_device != NULL;
       a_device = PRIV(a_device)->next_device)
    {
      a_callback = PRIV(a_device)->callback;

      if (a_callback != NULL  &&  a_callback->au_receive != NULL)
        {
          if (PRIV(a_device)->has_channel[g_current_channel])
            {
              if (a_callback->au_receive(a_device,
                                         g_current_channel,
                                         value) != OK)
                {
                  /* ### TODO ### Upper half buffer overflow */
                }
            }
        }
    }

  /* Start next channel read */

  get_next_channel();

  return OK;
}

/****************************************************************************
 * Name: get_next_channel
 *
 * Description:
 *   Update g_current_channel to point to next channel in use and start
 *   the conversion.
 *
 *  Note: This is called from inside a critical section.
 ****************************************************************************/

static void get_next_channel(void)
{
  FAR struct adc_dev_s *a_device;
  uint8_t               next = g_current_channel + 1;
  uint32_t              value;

  while (true)
    {
      if (next >= ADC_CHANNEL_COUNT)
        {
          next = 0;
        }

      for (a_device = g_first_device;
           a_device != NULL;
           a_device = PRIV(a_device)->next_device)
        {
          if (PRIV(a_device)->has_channel[next])
            {
              g_current_channel = next;

              while (getreg32(RP2040_ADC_FCS)
                     & RP2040_ADC_FCS_LEVEL_MASK)
                {
                  getreg32(RP2040_ADC_FIFO);
                }

              /* Enable Interrupt on ADC Completion */

              putreg32(RP2040_ADC_INTE_FIFO, RP2040_ADC_INTE);

              /* Configure CS to read one value from current channel */

              value =   (g_current_channel << RP2040_ADC_CS_AINSEL_SHIFT)
                      | RP2040_ADC_CS_EN;

              if (g_current_channel == ADC_TEMP_CHANNEL)
                {
                  value |= RP2040_ADC_CS_TS_ENA;
                }

              putreg32(value, RP2040_ADC_CS);

              while ((getreg32(RP2040_ADC_CS)
                      & RP2040_ADC_CS_READY) == 0)
                {
                  /* Wait for ready to go high.  The rp2040 docs
                   * say this is only a few clock cycles so we'll
                   * just busy wait.
                   */
                }

              /* Start the conversion */

              value += RP2040_ADC_CS_START_ONCE;

              putreg32(value, RP2040_ADC_CS);

              return;
            }
        }

      /* if we've looped all the way we're in trouble */

      ASSERT(next != g_current_channel);

      /* try another */

      next += 1;
    }

  return;
}

/****************************************************************************
 * Name: add_device
 *
 * Description:
 *   This function is called to link the device int the device list.
 *   It also makes sure ADC reads are taking place
 *
 *  Note: This is called from inside a critical section.
 ****************************************************************************/

static void add_device(FAR struct adc_dev_s *dev)
{
  uint32_t   value;

  g_active_count += 1;

  if (g_first_device != NULL)
    {
      PRIV(g_first_device)->prior_device = dev;
    }

  PRIV(dev)->next_device  = g_first_device;
  PRIV(dev)->prior_device = NULL;

  g_first_device = dev;

  if (PRIV(g_first_device)->next_device == NULL)
    {
      /* We just added first device */

      /* Make sure ADC interrupts are disabled */

      putreg32(0, RP2040_ADC_INTE);

      /* Configure FCS to use FIFO and interrupt on first value */

      value =   (1 << RP2040_ADC_FCS_THRESH_SHIFT)
              | RP2040_ADC_FCS_OVER
              | RP2040_ADC_FCS_UNDER
              | RP2040_ADC_FCS_ERR
              | RP2040_ADC_FCS_EN;

      putreg32(value, RP2040_ADC_FCS);

      /* Set up for interrupts */

      irq_attach(RP2040_ADC_IRQ_FIFO, interrupt_handler, NULL);

      up_enable_irq(RP2040_ADC_IRQ_FIFO);

      /* Start conversions on first required channel.  */

      get_next_channel();

      ainfo("new cur %d\n", g_current_channel);
    }
}

/****************************************************************************
 * Name: remove_device
 *
 * Description:
 *   This function is called to unlink the device from the device list.
 *
 *  Note: This is called from inside a critical section.
 ****************************************************************************/

void remove_device(FAR struct adc_dev_s *dev)
{
  FAR struct adc_dev_s *a_device;

  if (dev == g_first_device)
    {
      /* Special handling for first device */

      g_first_device = PRIV(g_first_device)->next_device;

      if (g_first_device != NULL)
        {
          PRIV(g_first_device)->prior_device = NULL;
        }

      g_active_count -= 1;
    }
  else
    {
      /* Make sure dev is on the change, and unlink if it is. */

      for (a_device = g_first_device;
           a_device != NULL;
           a_device = PRIV(a_device)->next_device)
        {
          if (a_device == dev)
            {
              PRIV(PRIV(dev)->prior_device)->next_device =
              PRIV(dev)->next_device;

              PRIV(PRIV(dev)->next_device)->prior_device =
                PRIV(dev)->prior_device;

              g_active_count -= 1;

              break;
            }
        }
    }
}

/****************************************************************************
 * Name: my_bind
 *
 * Description:
 *   This function is called when a driver is registered.  It give us a
 *   chance to bind the upper-half callbacks to our private data structure so
 *   they can be accessed later.
 *
 ****************************************************************************/

static int my_bind(struct adc_dev_s            *dev,
                   const struct adc_callback_s *callback)
{
  DEBUGASSERT(PRIV(dev) != NULL);

  ainfo("entered\n");

  PRIV(dev)->callback = callback;

  return OK;
}

/****************************************************************************
 * Name: my_reset
 *
 * Description:
 *   This is called by the upper-half as part of the driver registration
 *   process.  The upper half documentation also claims that it may
 *   be called as part of an error condition.
 *
 *   Set the pin for the ADC's to standard GPIO input with no pulls.
 *
 ****************************************************************************/

static void my_reset(struct adc_dev_s *dev)
{
  int a_gpio;

  ainfo("entered\n");

  for (int i = 0; i < ADC_CHANNEL_COUNT; ++i)
    {
      a_gpio = g_gpio_map[i];

      if (a_gpio >= 0)
        {
          rp2040_gpio_setdir(a_gpio, false);
          rp2040_gpio_set_function(a_gpio, RP2040_GPIO_FUNC_NULL);
          rp2040_gpio_set_pulls(a_gpio, false, false);
        }
    }
}

/****************************************************************************
 * Name: my_setup
 *
 * Description:
 *   This is called when a particular ADC driver is first opened.
 *
 *   We don't do anything here.
 *
 *  Note: This is called from inside a critical section.
 ****************************************************************************/

static int my_setup(struct adc_dev_s *dev)
{
  int        ret;

  ainfo("entered: 0x%08lX\n", dev);

  /* Note: We check g_active_count here so we can return an error
   *       in the, probably impossible, case we have too many.
   */

  if (g_active_count >= 200)
    {
      aerr("Too many active devices.");
      ret = -EBUSY;
    }
  else
    {
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: my_shutdown
 *
 * Description:
 *  This is called to shutdown an ADC device.  It unlinks the
 *  device from out local chain and turns off ADC interrupts if no
 *  more devices are active.
 *
 *  Note: This is called from inside a critical section.
 ****************************************************************************/

static void my_shutdown(FAR struct adc_dev_s *dev)
{
  ainfo("entered: 0x%08lX\n", dev);

  /* Remove adc_dev_s structure from the list */

  remove_device(dev);
}

/****************************************************************************
 * Name: my_rxint
 *
 * Description:
 *   Call to enable or disable ADC RX interrupts
 *
 *  Note: This is called from inside a critical section.
 ****************************************************************************/

static void my_rxint(struct adc_dev_s *dev, bool enable)
{
  if (enable)
    {
      ainfo("entered: enable: 0x%08lX\n", dev);

      add_device(dev);
    }
  else
    {
      ainfo("entered: disable: 0x%08lX\n", dev);

      remove_device(dev);
    }
}

/****************************************************************************
 * Name: my_ioctl
 *
 * Description:
 *  All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int my_ioctl(struct adc_dev_s *dev,
                    int               cmd,
                    unsigned long     arg)
{
  /* No ioctl commands supported */

  ainfo("entered\n");

  return -ENOTTY;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

#ifdef CONFIG_ADC

/****************************************************************************
 * Name: my_setup
 *
 * Description:
 *   Initialize and register the ADC driver.
 *
 * Input Parameters:
 *   path      - Path to the ws2812 device  (e.g. "/dev/adc0")
 *   read_adc0 - This device reads ADC0
 *   read_adc1 - This device reads ADC1
 *   read_adc2 - This device reads ADC3
 *   read_adc3 - This device reads ADC4
 *   read_temp - This device reads the chip temperature.
 *
 * Returned Value:
 *   An opaque pointer that can be passed to rp2040_adc_release on
 *   success or NULL (with errno set) on failure
 ****************************************************************************/

int rp2040_adc_setup(FAR const char *path,
                     bool            read_adc0,
                     bool            read_adc1,
                     bool            read_adc2,
                     bool            read_adc3,
                     bool            read_temp)
{
  FAR struct adc_dev_s *dev;
  FAR struct private_s *priv;
  int                   ret;

  ainfo("entered\n");

  if (!read_adc0  && !read_adc1 && !read_adc2 && !read_adc3 && !read_temp)
    {
      aerr("No ADC inputs selected.\n");
      return -EINVAL;
    }

  dev = kmm_zalloc(sizeof(struct adc_dev_s));

  if (dev == NULL)
    {
      aerr("Failed to allocate adc_dev_s.\n");
      return -ENOMEM;
    }

  priv = kmm_zalloc(sizeof(struct private_s));

  if (priv == NULL)
    {
      aerr("Failed to allocate private_s.\n");
      kmm_free(dev);
      return -ENOMEM;
    }

  priv->has_channel[0] = read_adc0;
  priv->has_channel[1] = read_adc1;
  priv->has_channel[2] = read_adc2;
  priv->has_channel[3] = read_adc3;
  priv->has_channel[4] = read_temp;

  dev->ad_ops  = &g_adcops;
  dev->ad_priv = priv;

  ret = adc_register(path, dev);

  return ret;
}

#endif /* if CONFIG_ADC */
#endif /* if CONFIG_RP2040_ADC */