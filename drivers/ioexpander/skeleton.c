/****************************************************************************
 * drivers/ioexpander/skeleton.c
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
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/skeleton.h>

#include "skeleton.h"

#if defined(CONFIG_IOEXPANDER_skeleton)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* This type represents on registered pin interrupt callback */

struct skel_callback_s
{
  ioe_pinset_t pinset;          /* Set of pin interrupts that will generate the callback */
  ioe_callback_t cbfunc;        /* The saved callback function pointer */
};
#endif

/* This structure represents the state of the I/O Expander driver */

struct skel_dev_s
{
  struct ioexpander_dev_s dev;  /* Nested structure to allow casting as public gpio expander */
#ifdef CONFIG_skeleton_MULTIPLE
  FAR struct skel_dev_s *flink; /* Supports a singly linked list of drivers */
#endif
  sem_t exclsem;                /* Mutual exclusion */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  struct work_s work;           /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct skel_callback_s cb[CONFIG_SKELETON_INT_NCALLBACKS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int skel_lock(FAR struct skel_dev_s *priv);

static int skel_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int dir);
static int skel_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                       int opt, void *val);
static int skel_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         bool value);
static int skel_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value);
static int skel_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int skel_multiwritepin(FAR struct ioexpander_dev_s *dev,
                              FAR uint8_t *pins, FAR bool *values,
                              int count);
static int skel_multireadpin(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count);
static int skel_multireadbuf(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int skel_attach(FAR struct ioexpander_dev_s *dev,
                       ioe_pinset_t pinset, ioe_callback_t callback);
static int skel_detach(FAR struct ioexpander_dev_s *dev,
                       FAR void *handle);

static void skel_irqworker(void *arg);
static void skel_interrupt(FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_skeleton_MULTIPLE
/* If only a single device is supported, then the driver state structure may
 * as well be pre-allocated.
 */

static struct skel_dev_s g_skel;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_skel_ops =
{
  skel_direction,
  skel_option,
  skel_writepin,
  skel_readpin,
  skel_readbuf
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , skel_multiwritepin
  , skel_multireadpin
  , skel_multireadbuf
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , skel_attach
  , skel_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: skel_lock
 *
 * Description:
 *   Get exclusive access to the I/O Expander
 *
 ****************************************************************************/

static int skel_lock(FAR struct skel_dev_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->exclsem);
}

#define skel_unlock(p) nxsem_post(&(p)->exclsem)

/****************************************************************************
 * Name: skel_direction
 *
 * Description:
 *   Set the direction of an ioexpander pin. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   dir - One of the IOEXPANDER_DIRECTION_ macros
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int skel_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int direction)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;
  int ret;

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  gpioinfo("pin=%u direction=%s\n",
           pin, (direction == IOEXPANDER_DIRECTION_IN) ? "IN" : "OUT");

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS);

  /* Get exclusive access to the I/O Expander */

  ret = skel_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the pin direction in the I/O Expander */
#warning Missing logic

  skel_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: skel_option
 *
 * Description:
 *   Set pin options. Required.
 *   Since all IO expanders have various pin options, this API allows setting
 *     pin options in a flexible way.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   opt - One of the IOEXPANDER_OPTION_ macros
 *   val - The option's value
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int skel_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                       int opt, FAR void *value)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;
  int ret = -ENOSYS;

  gpioinfo("addr=%02x pin=%u option=%u\n",  priv->addr, pin, opt);

  DEBUGASSERT(priv != NULL);

  /* Check for pin polarity inversion. */

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      /* Get exclusive access to the I/O Expander */

      ret = skel_lock(priv);
      if (ret < 0)
        {
          return ret;
        }

      /* Set the pin option */
#warning Missing logic

      skel_unlock(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: skel_writepin
 *
 * Description:
 *   Set the pin level. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   val - The pin level. Usually TRUE will set the pin high,
 *         except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int skel_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         bool value)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;
  int ret;

  gpioinfo("pin=%u value=%u\n", pin, value);

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS);

  /* Get exclusive access to the I/O Expander */

  ret = skel_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Write the pin value */
#warning Missing logic

  skel_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: skel_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the pin level is stored. Usually TRUE
 *            if the pin is high, except if OPTION_INVERT has been set on
 *            this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int skel_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;
  int ret;

  gpioinfo("pin=%u\n", priv->addr);

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS &&
              value != NULL);

  /* Get exclusive access to the I/O Expander */

  ret = skel_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the pin value */
#warning Missing logic

  /* Return the pin value via the value pointer */
#warning Missing logic

  skel_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: skel_readbuf
 *
 * Description:
 *   Read the buffered pin level.
 *   This can be different from the actual pin state. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the level is stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int skel_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;
  int ret;

  /* Get exclusive access to the I/O Expander */

  ret = skel_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the buffered pin level */
#warning Missing logic

  skel_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: skel_getmultibits
 *
 * Description:
 *  Read multiple bits from I/O Expander registers.
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int skel_getmultibits(FAR struct skel_dev_s *priv, FAR uint8_t *pins,
                             FAR bool *values, int count)
{
  ioe_pinset_t pinset;
  int pin;
  int ret = OK;
  int i;

  /* Read the pinset from the IO-Expander hardware */
#warning Missing logic

  /* Read the requested bits */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      if (pin >= CONFIG_IOEXPANDER_NPINS)
        {
          return -ENXIO;
        }

      values[i] = (((pinset >> pin) & 1) != 0);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: skel_multiwritepin
 *
 * Description:
 *   Set the pin level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pins - The list of pin indexes to alter in this call
 *   val - The list of pin levels.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int skel_multiwritepin(FAR struct ioexpander_dev_s *dev,
                              FAR uint8_t *pins, FAR bool *values, int count)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;
  ioe_pinset_t pinset;
  int pin;
  int ret;
  int i;

  /* Get exclusive access to the I/O Expander */

  ret = skel_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the pinset from the IO-Expander hardware */
#warning Missing logic

  /* Apply the user defined changes */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      if (pin >= CONFIG_IOEXPANDER_NPINS)
        {
          skel_unlock(priv);
          return -ENXIO;
        }

      if (values[i])
        {
          pinset |= ((ioe_pinset_t)1 << pin);
        }
      else
        {
          pinset &= ~((ioe_pinset_t)1 << pin);
        }
    }

  /* Now write back the new pins states */
#warning Missing logic

  skel_unlock(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: skel_multireadpin
 *
 * Description:
 *   Read the actual level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The list of pin indexes to read
 *   valptr - Pointer to a buffer where the pin levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int skel_multireadpin(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;
  int ret;

  gpioinfo("count=%u\n", count);

  DEBUGASSERT(priv != NULL && pins != NULL && values != NULL && count > 0);

  /* Get exclusive access to the I/O Expander */

  ret = skel_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = skel_getmultibits(priv, pins, values, count);
  skel_unlock(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: skel_multireadbuf
 *
 * Description:
 *   Read the buffered level of multiple pins. This routine may be faster
 *   than individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the buffered levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int skel_multireadbuf(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;
  int ret;

  gpioinfo("count=%u\n", count);

  DEBUGASSERT(priv != NULL && pins != NULL && values != NULL && count > 0);

  /* Get exclusive access to the I/O Expander */

  ret = skel_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = skel_getmultibits(priv, pins, values, count);
  skel_unlock(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: skel_attach
 *
 * Description:
 *   Attach a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   pinset   - The set of pin events that will generate the callback
 *   callback - The pointer to callback function.  NULL will detach the
 *              callback.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int skel_attach(FAR struct ioexpander_dev_s *dev, ioe_pinset_t pinset,
                       ioe_callback_t callback)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;
  int ret;
  int i;

  /* Get exclusive access to the I/O Expander */

  ret = skel_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Find and available in entry in the callback table */

  ret = -ENOSPC;
  for (i = 0; i < CONFIG_SKELETON_INT_NCALLBACKS; i++)
    {
      /* Is this entry available (i.e., no callback attached) */

      if (priv->cb[i].cbfunc == NULL)
        {
          /* Yes.. use this entry */

          priv->cb[i].pinset = pinset;
          priv->cb[i].cbfunc = callback;
          ret = OK;
        }
    }

  /* Add this callback to the table */

  skel_unlock(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: skel_detach_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by skel_attach_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int skel_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;
  FAR struct skel_callback_s *cb =
    (FAR struct skel_callback_s *)handle;

  DEBUGASSERT(priv != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&priv->cb[0] &&
              (uintptr_t)cb <=
              (uintptr_t)&priv->cb[CONFIG_SKELETON_INT_NCALLBACKS - 1]);
  UNUSED(priv);

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}
#endif

/****************************************************************************
 * Name: skel_irqworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static void skel_irqworker(void *arg)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)arg;
  ioe_pinset_t pinset;
  int ret;
  int i;

  /* Read the pinset from the IO-Expander hardware */
#warning Missing logic

  /* Perform pin interrupt callbacks */

  for (i = 0; i < CONFIG_SKELETON_INT_NCALLBACKS; i++)
    {
      /* Is this entry valid (i.e., callback attached)?  If so, did andy of
       * the requested pin interrupts occur?
       */

      if (priv->cb[i].cbfunc != NULL)
        {
          /* Did any of the requested pin interrupts occur? */

          ioe_pinset_t match = pinset & priv->cb[i].pinset;
          if (match != 0)
            {
              /* Yes.. perform the callback */

              priv->cb[i].cbfunc(&priv->dev, match);
            }
        }
    }

  /* Re-enable interrupts */
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: skel_interrupt
 *
 * Description:
 *   Handle GPIO interrupt events (this function executes in the
 *   context of the interrupt).
 *
 *   NOTE: A more typical prototype for an interrupt handler would be:
 *
 *     int skel_interrupt(int irq, FAR void *context, FAR void *arg)
 *
 *   However, it is assume that the lower half, board specific interface
 *   can provide intercept the actual interrupt, and call this function with
 *   the arg that can be mapped to the provide driver structure instance.
 *
 *   Presumably the lower level interface provides an attach() method that
 *   provides both the address of skel_interrupt() as well as the arg value.
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static void skel_interrupt(FAR void *arg)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Defer interrupt processing to the worker thread.  This is not only
   * much kinder in the use of system resources but is probably necessary
   * to access the I/O expander device.
   *
   * Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in skel_irqworker() when the work is
   * completed.
   */

  if (work_available(&priv->work))
    {
      /* Disable interrupts */
#warning Missing logic

      /* Schedule interrupt related work on the high priority worker
       * thread.
       */

      work_queue(HPWORK, &priv->work, skel_irqworker,
                 (FAR void *)priv, 0);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: skel_initialize
 *
 * Description:
 *   Initialize a I/O Expander device.
 *
 * NOTE: There are no arguments to the initialization function this
 * skeleton example.  Typical implementations take two arguments:
 *
 * 1) A reference to an I2C or SPI interface used to interactive with the
 *    device, and
 * 2) A read-only configuration structure that provides things like:  I2C
 *    or SPI characteristics and callbacks to attach, enable, and disable
 *    interrupts.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *skel_initialize(void)
{
  FAR struct skel_dev_s *priv;

#ifdef CONFIG_skeleton_MULTIPLE
  /* Allocate the device state structure */

  priv = (FAR struct skel_dev_s *)kmm_zalloc(sizeof(struct skel_dev_s));
  if (!priv)
    {
      gpioerr("ERROR: Failed to allocate driver instance\n");
      return NULL;
    }
#else
  /* Use the one-and-only I/O Expander driver instance */

  priv = &g_skel;
#endif

  /* Initialize the device state structure
   * NOTE: Normally you would also save the I2C/SPI device interface and
   * any configuration information here as well.
   */

  priv->dev.ops = &g_skel_ops;

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  /* Attach the I/O expander interrupt handler and enable interrupts */
#warning Missing logic

#endif

  nxsem_init(&priv->exclsem, 0, 1);
  return &priv->dev;
}

#endif /* CONFIG_IOEXPANDER_skeleton */
