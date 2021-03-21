/****************************************************************************
 * include/nuttx/ioexpander/ioexpander.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_IOEXPANDER_H
#define __INCLUDE_NUTTX_IOEXPANDER_IOEXPANDER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_IOEXPANDER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_IOEXPANDER_NPINS
#  define CONFIG_IOEXPANDER_NPINS 16
#endif

/* Pin definitions **********************************************************/

#define IOEXPANDER_DIRECTION_IN            0  /* float */
#define IOEXPANDER_DIRECTION_IN_PULLUP     1
#define IOEXPANDER_DIRECTION_IN_PULLDOWN   2
#define IOEXPANDER_DIRECTION_OUT           3  /* push-pull */
#define IOEXPANDER_DIRECTION_OUT_OPENDRAIN 4

#define IOEXPANDER_PINMASK         (((ioe_pinset_t)1 << CONFIG_IOEXPANDER_NPINS) - 1)
#define PINSET_ALL                 (~((ioe_pinset_t)0))

/* Pin options */

#define IOEXPANDER_OPTION_INVERT   1  /* Set the "active" level for a pin */
#  define IOEXPANDER_VAL_NORMAL    0  /* Normal, no inversion */
#  define IOEXPANDER_VAL_INVERT    1  /* Inverted */

#define IOEXPANDER_OPTION_INTCFG   2  /* Configure interrupt for a pin */
#  define IOEXPANDER_VAL_DISABLE   0  /* 0000 Disable pin  interrupts */
#  define IOEXPANDER_VAL_LEVEL     1  /* xx01 Interrupt on level (vs. edge) */
#    define IOEXPANDER_VAL_HIGH    5  /* 0101 Interrupt on high level */
#    define IOEXPANDER_VAL_LOW     9  /* 1001 Interrupt on low level */
#  define IOEXPANDER_VAL_EDGE      2  /* xx10 Interrupt on edge (vs. level) */
#    define IOEXPANDER_VAL_RISING  6  /* 0110 Interrupt on rising edge */
#    define IOEXPANDER_VAL_FALLING 10 /* 1010 Interrupt on falling edge */
#    define IOEXPANDER_VAL_BOTH    14 /* 1110 Interrupt on both edges */

/* Access macros ************************************************************/

/****************************************************************************
 * Name: IOEXP_SETDIRECTION
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

#define IOEXP_SETDIRECTION(dev,pin,dir) ((dev)->ops->ioe_direction(dev,pin,dir))

/****************************************************************************
 * Name: IOEXP_SETOPTION
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

#define IOEXP_SETOPTION(dev,pin,opt,val) ((dev)->ops->ioe_option(dev,pin,opt,val))

/****************************************************************************
 * Name: IOEXP_WRITEPIN
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

#define IOEXP_WRITEPIN(dev,pin,val)  ((dev)->ops->ioe_writepin(dev,pin,val))

/****************************************************************************
 * Name: IOEXP_READPIN
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

#define IOEXP_READPIN(dev,pin,valptr)  ((dev)->ops->ioe_readpin(dev,pin,valptr))

/****************************************************************************
 * Name: IOEXP_READBUF
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

#define IOEXP_READBUF(dev,pin,valptr) ((dev)->ops->ioe_readbuf(dev,pin,valptr))

#ifdef CONFIG_IOEXPANDER_MULTIPIN

/****************************************************************************
 * Name: IOEXP_MULTIWRITEPIN
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

#define IOEXP_MULTIWRITEPIN(dev,pins,vals,count) \
                           ((dev)->ops->ioe_multiwritepin(dev,pins,vals,count))

/****************************************************************************
 * Name: IOEXP_MULTIREADPIN
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

#define IOEXP_MULTIREADPIN(dev,pins,vals,count) \
                          ((dev)->ops->ioe_multireadpin(dev,pins,vals,count))

/****************************************************************************
 * Name: IOEXP_MULTIREADBUF
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

#define IOEXP_MULTIREADBUF(dev,pins,vals,count) \
                          ((dev)->ops->ioe_multireadbuf(dev,pins,vals,count))

#endif /* CONFIG_IOEXPANDER_MULTIPIN */

/****************************************************************************
 * Name: IOEP_ATTACH
 *
 * Description:
 *   Attach and enable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   pinset   - The set of pin events that will generate the callback
 *   callback - The pointer to callback function.  NULL will detach the
 *              callback.
 *   arg      - User-provided callback argument
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  This handle may be
 *   used later to detach and disable the pin interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#define IOEP_ATTACH(dev,pinset,callback,arg) \
                   ((dev)->ops->ioe_attach(dev,pinset,callback,arg))
#endif

/****************************************************************************
 * Name: IOEP_DETACH
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by IOEP_ATTACH
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#define IOEP_DETACH(dev,handle) ((dev)->ops->ioe_detach(dev,handle))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This type represents a bitmap of pins
 *
 * For IOE NPINS greater than 64, ioe_pinset_t represent one interrupt pin
 * number instead of a bitmap of pins.
 */

#if CONFIG_IOEXPANDER_NPINS <= 8
typedef uint8_t ioe_pinset_t;
#elif CONFIG_IOEXPANDER_NPINS <= 16
typedef uint16_t ioe_pinset_t;
#elif CONFIG_IOEXPANDER_NPINS <= 32
typedef uint32_t ioe_pinset_t;
#elif CONFIG_IOEXPANDER_NPINS <= 64
typedef uint64_t ioe_pinset_t;
#else
typedef uint8_t ioe_pinset_t;
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* This type represents a pin interrupt callback function */

struct ioexpander_dev_s;
typedef CODE int (*ioe_callback_t)(FAR struct ioexpander_dev_s *dev,
                                   ioe_pinset_t pinset, FAR void *arg);
#endif /* CONFIG_IOEXPANDER_INT_ENABLE */

/* I/O expander interface methods */

struct ioexpander_dev_s;
struct ioexpander_ops_s
{
  CODE int (*ioe_direction)(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            int direction);
  CODE int (*ioe_option)(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         int opt, FAR void *val);
  CODE int (*ioe_writepin)(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           bool value);
  CODE int (*ioe_readpin)(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          FAR bool *value);
  CODE int (*ioe_readbuf)(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  CODE int (*ioe_multiwritepin)(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count);
  CODE int (*ioe_multireadpin)(FAR struct ioexpander_dev_s *dev,
                               FAR uint8_t *pins, FAR bool *values,
                               int count);
  CODE int (*ioe_multireadbuf)(FAR struct ioexpander_dev_s *dev,
                               FAR uint8_t *pins, FAR bool *values,
                               int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  CODE FAR void *(*ioe_attach)(FAR struct ioexpander_dev_s *dev,
                               ioe_pinset_t pinset,
                               ioe_callback_t callback, FAR void *arg);
  CODE int (*ioe_detach)(FAR struct ioexpander_dev_s *dev,
                         FAR void *handle);
#endif
};

struct ioexpander_dev_s
{
  /* "Lower half" operations provided by the I/O expander lower half */

  FAR const struct ioexpander_ops_s *ops;

  /* Internal storage used by the I/O expander may (internal to the I/O
   * expander implementation).
   */
};

#endif /* CONFIG_IOEXPANDER */
#endif /* __INCLUDE_NUTTX_IOEXPANDER_IOEXPANDER_H */
