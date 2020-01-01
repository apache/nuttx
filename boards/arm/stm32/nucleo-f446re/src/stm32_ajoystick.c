/****************************************************************************
 * boards/arm/stm32/nucleo-f446re/src/stm32_ajoystick.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/input/ajoystick.h>

#include "stm32_gpio.h"
#include "stm32_adc.h"
#include "hardware/stm32_adc.h"
#include "nucleo-f446re.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check for pre-requisites and pin conflicts */

#ifdef CONFIG_AJOYSTICK
#  if !defined(CONFIG_ADC)
#    error CONFIG_ADC is required for the Itead joystick
#    undef CONFIG_AJOYSTICK
#  elif !defined(CONFIG_STM32_ADC1)
#    error CONFIG_STM32_ADC1 is required for Itead joystick
#    undef CONFIG_AJOYSTICK
#  endif
#endif /* CONFIG_AJOYSTICK */

#ifdef CONFIG_AJOYSTICK

/* A no-ADC, buttons only version can be built for testing */

#undef NO_JOYSTICK_ADC

/* Maximum number of ADC channels */

#define MAX_ADC_CHANNELS 8

/* Dual channel ADC support requires DMA */

#ifdef CONFIG_ADC_DMA
#  define NJOYSTICK_CHANNELS 2
#else
#  define NJOYSTICK_CHANNELS 1
#endif

#ifdef CONFIG_NUCLEO_F401RE_AJOY_MINBUTTONS
/* Number of Joystick buttons */

#  define AJOY_NGPIOS  3

/* Bitset of supported Joystick buttons */

#  define AJOY_SUPPORTED (AJOY_BUTTON_1_BIT | AJOY_BUTTON_2_BIT | \
                          AJOY_BUTTON_3_BIT)
#else
/* Number of Joystick buttons */

#  define AJOY_NGPIOS  7

/* Bitset of supported Joystick buttons */

#  define AJOY_SUPPORTED (AJOY_BUTTON_1_BIT | AJOY_BUTTON_2_BIT | \
                          AJOY_BUTTON_3_BIT | AJOY_BUTTON_4_BIT | \
                          AJOY_BUTTON_5_BIT | AJOY_BUTTON_6_BIT | \
                          AJOY_BUTTON_7_BIT )
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ajoy_buttonset_t ajoy_supported(FAR const struct ajoy_lowerhalf_s *lower);
static int ajoy_sample(FAR const struct ajoy_lowerhalf_s *lower,
                       FAR struct ajoy_sample_s *sample);
static ajoy_buttonset_t ajoy_buttons(FAR const struct ajoy_lowerhalf_s *lower);
static void ajoy_enable(FAR const struct ajoy_lowerhalf_s *lower,
                         ajoy_buttonset_t press, ajoy_buttonset_t release,
                         ajoy_handler_t handler, FAR void *arg);

static void ajoy_disable(void);
static int ajoy_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Pin configuration for each Itead joystick button.  Index using AJOY_*
 * button definitions in include/nuttx/input/ajoystick.h.
 */

#ifdef CONFIG_NUCLEO_F401RE_AJOY_MINBUTTONS
static const uint32_t g_joygpio[AJOY_NGPIOS] =
{
  GPIO_BUTTON_1, GPIO_BUTTON_2, GPIO_BUTTON_3
};
#else
static const uint32_t g_joygpio[AJOY_NGPIOS] =
{
  GPIO_BUTTON_1, GPIO_BUTTON_2, GPIO_BUTTON_3, GPIO_BUTTON_4,
  GPIO_BUTTON_5, GPIO_BUTTON_6, GPIO_BUTTON_7
};
#endif

/* This is the button joystick lower half driver interface */

static const struct ajoy_lowerhalf_s g_ajoylower =
{
  .al_supported  = ajoy_supported,
  .al_sample     = ajoy_sample,
  .al_buttons    = ajoy_buttons,
  .al_enable     = ajoy_enable,
};

#ifndef NO_JOYSTICK_ADC
/* Thread-independent file structure for the open ADC driver */

static struct file g_adcfile;
#endif

/* Current interrupt handler and argument */

static ajoy_handler_t g_ajoyhandler;
static FAR void *g_ajoyarg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ajoy_supported
 *
 * Description:
 *   Return the set of buttons supported on the button joystick device
 *
 ****************************************************************************/

static ajoy_buttonset_t ajoy_supported(FAR const struct ajoy_lowerhalf_s *lower)
{
  iinfo("Supported: %02x\n", AJOY_SUPPORTED);
  return (ajoy_buttonset_t)AJOY_SUPPORTED;
}

/****************************************************************************
 * Name: ajoy_sample
 *
 * Description:
 *   Return the current state of all button joystick buttons
 *
 ****************************************************************************/

static int ajoy_sample(FAR const struct ajoy_lowerhalf_s *lower,
                       FAR struct ajoy_sample_s *sample)
{
#ifndef NO_JOYSTICK_ADC
  struct adc_msg_s adcmsg[MAX_ADC_CHANNELS];
  FAR struct adc_msg_s *ptr;
  ssize_t nread;
  ssize_t offset;
  int have;
  int i;

  /* Read all of the available samples (handling the case where additional
   * channels are enabled).
   */

  nread = file_read(&g_adcfile, adcmsg,
                    MAX_ADC_CHANNELS * sizeof(struct adc_msg_s));
  if (nread < 0)
    {
      if (nread != -EINTR)
        {
          ierr("ERROR: read failed: %d\n", (int)nread);
        }

      return nread;
    }
  else if (nread < NJOYSTICK_CHANNELS * sizeof(struct adc_msg_s))
    {
      ierr("ERROR: read too small: %ld\n", (long)nread);
      return -EIO;
    }

  /* Sample and the raw analog inputs */

#ifdef CONFIG_ADC_DMA
  have = 0;

#else
  /* If DMA is not supported, then we will have only a single ADC channel */

  have = 2;
  sample->as_y = 0;
#endif

  for (i = 0, offset = 0;
       i < MAX_ADC_CHANNELS && offset < nread && have != 3;
       i++, offset += sizeof(struct adc_msg_s))
    {
      ptr = &adcmsg[i];

      /* Is this one of the channels that we need? */

      if ((have & 1) == 0 && ptr->am_channel == 0)
        {
          int32_t tmp = ptr->am_data;
          sample->as_x = (int16_t)tmp;
          have |= 1;

          iinfo("X sample: %ld -> %d\n", (long)tmp, (int)sample->as_x);
        }

#ifdef CONFIG_ADC_DMA
      if ((have & 2) == 0 && ptr->am_channel == 1)
        {
          int32_t tmp = ptr->am_data;
          sample->as_y = (int16_t)tmp;
          have |= 2;

          iinfo("Y sample: %ld -> %d\n", (long)tmp, (int)sample->as_y);
        }
#endif
    }

  if (have != 3)
    {
      ierr("ERROR: Could not find joystick channels\n");
      return -EIO;
    }

#else
  /* ADC support is disabled */

  sample->as_x = 0;
  sample->as_y = 0;
#endif

  /* Sample the discrete button inputs */

  sample->as_buttons = ajoy_buttons(lower);
  iinfo("Returning: %02x\n", sample->as_buttons);
  return OK;
}

/****************************************************************************
 * Name: ajoy_buttons
 *
 * Description:
 *   Return the current state of button data (only)
 *
 ****************************************************************************/

static ajoy_buttonset_t ajoy_buttons(FAR const struct ajoy_lowerhalf_s *lower)
{
  ajoy_buttonset_t ret = 0;
  int i;

  /* Read each joystick GPIO value */

  for (i = 0; i < AJOY_NGPIOS; i++)
    {
      /* Button outputs are pulled high. So a sensed low level means that the
       * button is pressed.
       */

      if (!stm32_gpioread(g_joygpio[i]))
        {
          ret |= (1 << i);
        }
    }

  iinfo("Returning: %02x\n", ret);
  return ret;
}

/****************************************************************************
 * Name: ajoy_enable
 *
 * Description:
 *   Enable interrupts on the selected set of joystick buttons.  And empty
 *   set will disable all interrupts.
 *
 ****************************************************************************/

static void ajoy_enable(FAR const struct ajoy_lowerhalf_s *lower,
                         ajoy_buttonset_t press, ajoy_buttonset_t release,
                         ajoy_handler_t handler, FAR void *arg)
{
  irqstate_t flags;
  ajoy_buttonset_t either = press | release;
  ajoy_buttonset_t bit;
  bool rising;
  bool falling;
  int i;

  /* Start with all interrupts disabled */

  flags = enter_critical_section();
  ajoy_disable();

  iinfo("press: %02x release: %02x handler: %p arg: %p\n",
        press, release, handler, arg);

  /* If no events are indicated or if no handler is provided, then this
   * must really be a request to disable interrupts.
   */

  if (either && handler)
    {
      /* Save the new the handler and argument */

      g_ajoyhandler = handler;
      g_ajoyarg     = arg;

      /* Check each GPIO. */

      for (i = 0; i < AJOY_NGPIOS; i++)
        {
          /* Enable interrupts on each pin that has either a press or
           * release event associated with it.
           */

          bit = (1 << i);
          if ((either & bit) != 0)
            {
              /* Active low so a press corresponds to a falling edge and
               * a release corresponds to a rising edge.
               */

              falling = ((press & bit) != 0);
              rising  = ((release & bit) != 0);

              iinfo("GPIO %d: rising: %d falling: %d\n",
                    i, rising, falling);

              stm32_gpiosetevent(g_joygpio[i], rising, falling,
                                 true, ajoy_interrupt, NULL);
            }
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ajoy_disable
 *
 * Description:
 *   Disable all joystick interrupts
 *
 ****************************************************************************/

static void ajoy_disable(void)
{
  irqstate_t flags;
  int i;

  /* Disable each joystick interrupt */

  flags = enter_critical_section();
  for (i = 0; i < AJOY_NGPIOS; i++)
    {
      stm32_gpiosetevent(g_joygpio[i], false, false, false, NULL, NULL);
    }

  leave_critical_section(flags);

  /* Nullify the handler and argument */

  g_ajoyhandler = NULL;
  g_ajoyarg     = NULL;
}

/****************************************************************************
 * Name: ajoy_interrupt
 *
 * Description:
 *   Discrete joystick interrupt handler
 *
 ****************************************************************************/

static int ajoy_interrupt(int irq, FAR void *context, FAR void *arg)
{
  DEBUGASSERT(g_ajoyhandler);

  if (g_ajoyhandler)
    {
      g_ajoyhandler(&g_ajoylower, g_ajoyarg);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ajoy_initialize
 *
 * Description:
 *   Initialize and register the button joystick driver
 *
 ****************************************************************************/

int board_ajoy_initialize(void)
{
  int ret;
  int i;

#ifndef NO_JOYSTICK_ADC
  iinfo("Initialize ADC driver: /dev/adc0\n");

  /* Open the ADC driver for reading.
   * NOTE: The ADC driver was initialized earlier in the bring-up sequence.
   */

  ret = file_open(&g_adcfile, "/dev/adc0", O_RDONLY);
  if (ret < 0)
    {
      ierr("ERROR: Failed to open /dev/adc0: %d\n", ret);
      return ret;
    }
#endif

  /* Configure the GPIO pins as interrupting inputs.  NOTE: This is
   * unnecessary for interrupting pins since it will also be done by
   * stm32_gpiosetevent().
   */

  for (i = 0; i < AJOY_NGPIOS; i++)
    {
      /* Configure the PIO as an input */

      stm32_configgpio(g_joygpio[i]);
    }

  /* Register the joystick device as /dev/ajoy0 */

  iinfo("Initialize joystick driver: /dev/ajoy0\n");

  ret = ajoy_register("/dev/ajoy0", &g_ajoylower);
  if (ret < 0)
    {
      ierr("ERROR: ajoy_register failed: %d\n", ret);
#ifndef NO_JOYSTICK_ADC
      file_close(&g_adcfile);
#endif
    }

  return ret;
}

#endif /* CONFIG_AJOYSTICK */
