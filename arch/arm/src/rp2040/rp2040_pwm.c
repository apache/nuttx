/****************************************************************************
 * arch/arm/src/rp2040/rp2040_pwm.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>
#include "rp2040_pwm.h"
#include "rp2040_gpio.h"


/****************************************************************************
 * Local Function Prototypes
 ****************************************************************************/

static int  pwm_setup    ( struct pwm_lowerhalf_s  * dev ); 

static int  pwm_shutdown ( struct pwm_lowerhalf_s  * dev );

static int  pwm_start    ( struct pwm_lowerhalf_s  * dev,
                           const struct pwm_info_s * info );

static int  pwm_stop     ( struct pwm_lowerhalf_s  * dev );

static int  pwm_ioctl    ( struct pwm_lowerhalf_s  * dev, 
                           int                       cmd,
                           unsigned long             arg );


static void setup_slice  ( struct rp2040_pwm_lowerhalf_s  * priv );

static void setup_duty   ( struct rp2040_pwm_lowerhalf_s  * priv );

/****************************************************************************
 * Local Inline Function
 ****************************************************************************/


static inline void pwm_set_enabled( uint slice_num )
{
  (*(volatile uint32_t *) RP2040_PWM_CSR(slice_num)) |= RP2040_PWM_CSR_EN;
}

static inline void pwm_clear_enabled( uint slice_num )
{
  putreg32( 0, RP2040_PWM_CSR(slice_num) );
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PWM operations */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = pwm_setup,
  .shutdown    = pwm_shutdown,
  .start       = pwm_start,
  .stop        = pwm_stop,
  .ioctl       = pwm_ioctl
};

/****************************************************************************
 * Name: rp2040_pwm_initialize
 *
 * Description:
 *   Initialize the selected PWM port. And return a unique instance of struct
 *   struct rp2040_pwm_lowerhalf_s.  This function may be called to obtain 
 *   multiple instances of the interface, each of which may be set up with a
 *   different frequency and address.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple PWM interfaces)
 *   GPIO pin number for pin A
 *   GPIO pin number for pin B (CONFIG_PWM_NCHANNELS == 2)
 *
 * Returned Value:
 *   Valid PWM device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
struct rp2040_pwm_lowerhalf_s *rp2040_pwm_initialize( int port, 
                                                      int pin_a, 
                                                      int pin_b )
#else
struct rp2040_pwm_lowerhalf_s *rp2040_pwm_initialize(int port, int pin)
#endif
{
  struct rp2040_pwm_lowerhalf_s *data;

  data = calloc( 1, sizeof (struct rp2040_pwm_lowerhalf_s) );

  if (data != NULL)
    {
      data->ops  = &g_pwmops;
      data->num  = port;
#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
      if (pin_a == 2*port  ||  pin_a == 2*port + 16)
        {
          data->pin[0] = pin_a;
        }
        else
        {
          data->pin[0] = -1;
        }

      if (pin_b == 2*port + 1  ||  pin_b == 2*port + 17)
        {
          data->pin[1] = pin_b;
        }
        else
        {
          data->pin[1] = -1;
        }
#else
      if (pin == 2*port  ||  pin == 2*port + 16)
        {
          data->pin = pin;
        }
        else
        {
          data->pin = -1;
        }

#endif
    }

  return data;
}

/****************************************************************************
 * Name: rp2040_pwm_uninitialize
 *
 * Description:
 *   De-initialize the selected pwm port, and power down the device.
 *
 * Input Parameter:
 *   Device structure as returned by the rp2040_pwmdev_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int rp2040_pwm_uninitialize(struct pwm_lowerhalf_s *dev)
{
  free( dev );
  return ( OK );
}

/****************************************************************************
 * Local Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int pwm_setup( struct pwm_lowerhalf_s  * dev )
{
  struct rp2040_pwm_lowerhalf_s *priv = (struct rp2040_pwm_lowerhalf_s *)dev;

#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  pwminfo( "PWM%d pin_a %d pin_b %d\n", priv->num, priv->pin[0], priv->pin[1] );

  if (priv->pin[0] >= 0)
    {
      pwminfo( "PWM%d setting pin_a %d\n", priv->num );

      rp2040_gpio_set_function( priv->pin[0], RP2040_GPIO_FUNC_PWM );
    }

  if (priv->pin[1] >= 0)
    {
      pwminfo( "PWM%d setting pin_b %d\n", priv->num );

      rp2040_gpio_set_function( priv->pin[1], RP2040_GPIO_FUNC_PWM );
    }
#else
  pwminfo( "PWM%d pin %d\n", priv->num, priv->pin );

  if (priv->pin >= 0)
    {
      rp2040_gpio_set_function( priv->pin, RP2040_GPIO_FUNC_PWM );
    }
#endif

  return 0;
} 

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int pwm_shutdown ( struct pwm_lowerhalf_s  * dev )
{
  struct rp2040_pwm_lowerhalf_s *priv = (struct rp2040_pwm_lowerhalf_s *)dev;

  pwminfo( "PWM%d\n", priv->num );

  /* Stop timer */

  pwm_stop( dev );

  /* Clear timer and channel configuration */

  priv->frequency = 0;
  priv->divisor   = 0x00000010;  /* hex 1.0 */
  priv->top       = 0xFFFF;

#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  for (int i = 0; i < CONFIG_PWM_NCHANNELS; ++i)
    {
      priv->duty[i] = 0;
    }
#else
  priv->duty = 0;
#endif

  return 0;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int pwm_start( struct pwm_lowerhalf_s  * dev,
               const struct pwm_info_s * info )
{
  struct rp2040_pwm_lowerhalf_s *priv = (struct rp2040_pwm_lowerhalf_s *)dev;

  pwminfo( "PWM%d\n", priv->num );

  /* Update timer with given PWM timer frequency */

  if (priv->frequency != info->frequency)
    {
      priv->frequency = info->frequency;

      /* We want to compute the top and divisor to give the finest control */

      setup_slice( priv );
    }

  /* Update timer with given PWM channel duty */

#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  for (int i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      if (priv->duty[i] != info->channels[i].duty)
        {
          priv->duty[i] = info->channels[i].duty;
        }
    }
#else
  if (priv->duty != info[0].duty)
    {
      priv->duty = info[0].duty;
    }
#endif

  setup_duty( priv );
  pwm_set_enabled( priv->num );

  return 0;
}
                      
/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int pwm_stop( struct pwm_lowerhalf_s  * dev )
{
  struct rp2040_pwm_lowerhalf_s *priv = (struct rp2040_pwm_lowerhalf_s *)dev;
  irqstate_t flags;

  pwminfo( "PWM%d\n", priv->num );

  flags = enter_critical_section();

  pwm_clear_enabled( priv->num );

  leave_critical_section(flags);
  return 0;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int pwm_ioctl( struct pwm_lowerhalf_s  * dev, 
               int                       cmd,
               unsigned long             arg )
{
#ifdef CONFIG_DEBUG_PWM_INFO
  struct rp2040_pwm_lowerhalf_s *priv = (struct rp2040_pwm_lowerhalf_s *)dev;

  pwminfo( "PWM%d\n", priv->num );
#endif

  switch (cmd)
    {
    case PWMIOC_RP2040_SETINVERTPULSE:

      modreg32( (arg & 0x03) << 2,
                RP2040_PWM_CSR_B_INV | RP2040_PWM_CSR_A_INV, 
                RP2040_PWM_CSR_OFFSET(priv->num) );
                
      return 0;

    case PWMIOC_RP2040_GETINVERTPULSE:
      return (  getreg32( RP2040_PWM_CSR_OFFSET(priv->num) ) 
              & (RP2040_PWM_CSR_B_INV | RP2040_PWM_CSR_A_INV)) >> 2;

    case PWMIOC_RP2040_SETPHASECORRECT:
      modreg32( (arg != 0) ? RP2040_PWM_CSR_PH_CORRECT : 0x00,
                RP2040_PWM_CSR_PH_CORRECT, 
                RP2040_PWM_CSR_OFFSET(priv->num) );

      return 0;

    case PWMIOC_RP2040_GETPHASECORRECT:
      return (  getreg32( RP2040_PWM_CSR_OFFSET(priv->num) ) 
              & RP2040_PWM_CSR_PH_CORRECT) ? 1 : 0;
  }

  return -ENOTTY;
}

/****************************************************************************
 * Name: setup_slice
 *
 * Description:
 *   compute and set the clock divisor and top value based on frequency.
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *
 ****************************************************************************/

void setup_slice( struct rp2040_pwm_lowerhalf_s  * priv )
{
  uint32_t max_freq = BOARD_SYS_FREQ / 0x10000; /* initially, with full range count */

  pwminfo( "PWM%d freq %d max %d\n", priv->num, priv->frequency, max_freq );

  if (priv->frequency <= max_freq)
    {
      /* We can keep full range count and slow clock down with divisor */

      priv->top     = 0xFFFF;
    }
    else
    {
      /* we need to speed things up by reducing top */
      priv->top     = 0xFFFF / (priv->frequency / max_freq);

      /* compute new maximum frequency */
      max_freq      = BOARD_SYS_FREQ / (priv->top + 1);
    }

  priv->divisor = 16 * max_freq / priv->frequency;

  pwminfo( "PWM%d top 0x%08X div 0x%08X\n", priv->num, priv->top, priv->divisor );

  putreg32( priv->top,     RP2040_PWM_TOP(priv->num) );
  putreg32( priv->divisor, RP2040_PWM_DIV(priv->num) );
}

/****************************************************************************
 * Name: setup_duty
 *
 * Description:
 *   compute and set the compare values.
 *
 * Input Parameters:
 *   priv    - A reference to the lower half PWM driver state structure
 *
 ****************************************************************************/

void setup_duty( struct rp2040_pwm_lowerhalf_s  * priv )
{  
#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  uint32_t compare =    (0xFFFF * (uint32_t)priv->duty[0] / priv->top)
                     + ((0xFFFF * (uint32_t)priv->duty[1] / priv->top) << 16);
#else
  uint32_t compare = 0xFFFF * (uint32_t)priv->duty / priv->top;
#endif

  pwminfo( "PWM%d duties 0x%08X\n", priv->num, compare );

  putreg32( compare, RP2040_PWM_CC(priv->num) );
}
