/****************************************************************************
 * arch/sim/src/sim/sim_gpiochip.c
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

#include <debug.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/ioexpander.h>

#include "sim_gpiochip.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIOCHIP_LINE_BASE        60
#define SIM_GPIOCHIP_WDOG_DELAY   USEC2TICK(500)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_gpiochip_callback_s
{
  ioe_callback_t cbfunc;
  void *cbarg;
};

struct sim_gpiochip_dev_s
{
  const struct ioexpander_ops_s *ops;   /* gpiochip vtable */
  struct sim_gpiochip_callback_s cb[CONFIG_IOEXPANDER_NPINS];
  struct host_gpiochip_dev *dev;
  struct wdog_s wdog;                   /* poll work */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sim_gpiochip_direction(struct ioexpander_dev_s *dev,
                                  uint8_t pin, int direction);
static int sim_gpiochip_option(struct ioexpander_dev_s *dev, uint8_t pin,
                               int option, void *val);
static int sim_gpiochip_writepin(struct ioexpander_dev_s *dev, uint8_t pin,
                                 bool value);
static int sim_gpiochip_readpin(struct ioexpander_dev_s *dev, uint8_t pin,
                                bool *value);
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static void *sim_gpiochip_attach(struct ioexpander_dev_s *dev,
                                 uint16_t pinset,
                                 ioe_callback_t callback,
                                 void *arg);
static int sim_gpiochip_detach(struct ioexpander_dev_s *dev,
                               void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ioexpander_ops_s g_sim_gpiochip_ops =
{
  .ioe_direction = sim_gpiochip_direction,
  .ioe_option    = sim_gpiochip_option,
  .ioe_writepin  = sim_gpiochip_writepin,
  .ioe_readpin   = sim_gpiochip_readpin,
  .ioe_readbuf   = sim_gpiochip_readpin,
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  .ioe_attach    = sim_gpiochip_attach,
  .ioe_detach    = sim_gpiochip_detach,
#endif
};

struct ioexpander_dev_s *g_gpiochip = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_gpiochip_direction
 *
 * Description:
 *   Provide gpiochip pin direction config.
 *
 * Input Parameters:
 *   dev       - A pointer to instance of sim gpiochip device.
 *   pin       - The pin number.
 *   direction - The direction of the pin.
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

static int sim_gpiochip_direction(struct ioexpander_dev_s *dev,
                                  uint8_t pin, int direction)
{
  struct sim_gpiochip_dev_s *priv = (struct sim_gpiochip_dev_s *)dev;

  if (direction == IOEXPANDER_DIRECTION_OUT ||
      direction == IOEXPANDER_DIRECTION_OUT_OPENDRAIN)
    {
      return host_gpiochip_direction(priv->dev, pin, false);
    }
  else
    {
      return host_gpiochip_direction(priv->dev, pin, true);
    }
}

/****************************************************************************
 * Name: sim_gpiochip_option
 *
 * Description:
 *   Provide gpiochip pin option.
 *
 * Input Parameters:
 *   dev    - A pointer to instance of sim gpiochip device.
 *   pin    - The pin number.
 *   option.- The option type.
 *   val    - A pointer to val of the pin.
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

static int sim_gpiochip_option(struct ioexpander_dev_s *dev, uint8_t pin,
                               int option, void *val)
{
  struct sim_gpiochip_dev_s *priv = (struct sim_gpiochip_dev_s *)dev;
  uint16_t cfgset = 0;
  int ret = 0;

  if (option == IOEXPANDER_OPTION_INTCFG)
    {
      switch ((uintptr_t)val)
        {
          case IOEXPANDER_VAL_FALLING:
            cfgset = GPIOCHIP_LINE_FLAG_FALLING;
            break;

          case IOEXPANDER_VAL_RISING:
            cfgset = GPIOCHIP_LINE_FLAG_RISING;
            break;

          case IOEXPANDER_VAL_BOTH:
            cfgset = GPIOCHIP_LINE_FLAG_BOTH;
            break;

          case IOEXPANDER_VAL_DISABLE:
            cfgset = GPIOCHIP_LINE_FLAG_DISABLE;
            break;

          default:
            return -ENOTSUP;
        }

      ret = host_gpiochip_irq_request(priv->dev, pin, cfgset);
      if (ret < 0)
        {
          gpioerr("ERROR: Failed to request event: %s\n", strerror(errno));
        }
    }
  else
    {
      gpioinfo("gpiochip io option not support\n");
      return -ENOTSUP;
    }

  return ret;
}

/****************************************************************************
 * Name: sim_gpiochip_writepin
 *
 * Description:
 *   Write gpiochip pin value.
 *
 * Input Parameters:
 *   dev   - A pointer to instance of sim gpiochip device.
 *   pin   - The pin number.
 *   value - The value write to the pin.
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

static int sim_gpiochip_writepin(struct ioexpander_dev_s *dev, uint8_t pin,
                                 bool value)
{
  struct sim_gpiochip_dev_s *priv = (struct sim_gpiochip_dev_s *)dev;

  return host_gpiochip_writepin(priv->dev, pin, value);
}

/****************************************************************************
 * Name: sim_gpiochip_readpin
 *
 * Description:
 *   Read gpiochip pin value.
 *
 * Input Parameters:
 *   dev   - A pointer to instance of sim gpiochip device.
 *   pin   - The pin number.
 *   value - The value write to the pin.
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

static int sim_gpiochip_readpin(struct ioexpander_dev_s *dev, uint8_t pin,
                                bool *value)
{
  struct sim_gpiochip_dev_s *priv = (struct sim_gpiochip_dev_s *)dev;

  return host_gpiochip_readpin(priv->dev, pin, value);
}

#ifdef CONFIG_IOEXPANDER_INT_ENABLE

/****************************************************************************
 * Name: sim_gpiochip_attach
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

static void *sim_gpiochip_attach(struct ioexpander_dev_s *dev,
                                 ioe_pinset_t pinset,
                                 ioe_callback_t callback,
                                 void *arg)
{
  struct sim_gpiochip_dev_s *priv = (struct sim_gpiochip_dev_s *)dev;
  void *handle = NULL;
  int i;

  for (i = 0; i < CONFIG_IOEXPANDER_NPINS; i++)
    {
      if (pinset & (1 << i))
        {
          priv->cb[i].cbarg = arg;
          handle = &priv->cb[i];
          priv->cb[i].cbfunc = callback;
        }
    }

  return handle;
}

/****************************************************************************
 * Name: sim_gpiochip_detach
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

static int sim_gpiochip_detach(struct ioexpander_dev_s *dev, void *handle)
{
  struct sim_gpiochip_dev_s *priv = (struct sim_gpiochip_dev_s *)dev;
  struct sim_gpiochip_callback_s *cb = handle;

  if (priv == NULL || cb == NULL)
    {
      gpioerr("ERROR: Invalid handle\n");
      return -EINVAL;
    }

  cb->cbfunc = NULL;
  cb->cbarg = NULL;
  return 0;
}
#endif

/****************************************************************************
 * Name: sim_gpiochip_register_gpio
 *
 * Description:
 *   register gpio for gpiochip device
 *
 * Input Parameters:
 *   priv - A pointer to instance of sim gpiochip device.
 *
 * Returned Value:
 *   0 for OK.
 *
 ****************************************************************************/

static int sim_gpiochip_register_gpio(struct sim_gpiochip_dev_s *priv)
{
  struct ioexpander_dev_s *ioe = (struct ioexpander_dev_s *)priv;
  bool input;
  int line;
  int ret;

  for (line = 0; line < CONFIG_IOEXPANDER_NPINS; line++)
    {
      ret = host_gpiochip_get_line(priv->dev, line, &input);
      if (ret != 0)
        {
          continue;
        }

      if (input)
        {
          ret = gpio_lower_half(ioe, line, GPIO_INPUT_PIN,
                                GPIOCHIP_LINE_BASE + line);
        }
      else
        {
          ret = gpio_lower_half(ioe, line, GPIO_OUTPUT_PIN,
                                GPIOCHIP_LINE_BASE + line);
        }

      if (ret < 0)
        {
          return ret;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: sim_gpiochip_irq_process
 *
 * Description:
 *   work to poll irq event for gpiochip device
 *
 * Input Parameters:
 *   priv - A pointer to instance of sim gpiochip device.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sim_gpiochip_irq_process(struct sim_gpiochip_dev_s *priv)
{
  int line;

  for (line = 0; line < CONFIG_IOEXPANDER_NPINS; line++)
    {
      if (host_gpiochip_irq_active(priv->dev, line))
        {
          if (priv->cb[line].cbfunc)
            {
              priv->cb[line].cbfunc((struct ioexpander_dev_s *)priv,
                                    line, priv->cb[line].cbarg);
            }
        }
    }
}

/****************************************************************************
 * Name: sim_gpiochip_interrupt
 *
 * Description:
 *   wdog timer function for gpiochip device
 *
 * Input Parameters:
 *   arg - A pointer to instance of sim gpiochip device.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sim_gpiochip_interrupt(wdparm_t arg)
{
  struct sim_gpiochip_dev_s *priv = (struct sim_gpiochip_dev_s *)arg;

  if (priv)
    {
      sim_gpiochip_irq_process(priv);

      wd_start(&priv->wdog, SIM_GPIOCHIP_WDOG_DELAY,
               sim_gpiochip_interrupt, (wdparm_t)priv);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_gpiochip_initialize
 *
 * Description:
 *   Initialize one gpiochip device
 *
 * Input Parameters:
 *   path - the name of gpiochip device in sim, e.g. "/dev/gpiochipN".
 *
 * Returned Value:
 *   The pointer to the instance of sim gpiochip device.
 *
 ****************************************************************************/

int sim_gpiochip_initialize(const char *path)
{
  struct sim_gpiochip_dev_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct sim_gpiochip_dev_s));
  if (priv == NULL)
    {
      gpioerr("Failed to allocate memory for gpiochip device");
      return -ENOMEM;
    }

  priv->ops = &g_sim_gpiochip_ops;

  priv->dev = host_gpiochip_alloc(path);
  if (priv->dev == NULL)
    {
      gpioerr("Failed to init gpiochip: %d", ret);
      kmm_free(priv);
      return -ENODEV;
    }

  ret = sim_gpiochip_register_gpio(priv);
  if (ret < 0)
    {
      gpioerr("Failed to register gpio: %d", ret);
      host_gpiochip_free(priv->dev);
      kmm_free(priv);
      return ret;
    }

  wd_start(&priv->wdog, SIM_GPIOCHIP_WDOG_DELAY,
           sim_gpiochip_interrupt, (wdparm_t)priv);

  g_gpiochip = (struct ioexpander_dev_s *)priv;

  return 0;
}

/****************************************************************************
 * Name: sim_gpiochip_get_ioe
 *
 * Description:
 *   Get the ioexpander pointer of gpiochip device
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The pointer to the instance of sim gpiochip device.
 *
 ****************************************************************************/

struct ioexpander_dev_s *sim_gpiochip_get_ioe(void)
{
  return g_gpiochip;
}
