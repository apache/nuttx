/****************************************************************************
 * include/nuttx/pinctrl/pinctrl.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_PINCTRL_PINCTRL_H
#define __INCLUDE_NUTTX_PINCTRL_PINCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/param.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Command:     PINCTRLC_SETFUNCTION
 * Description: Set the mux function of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SETSTRENGTH
 * Description: Set the driver strength of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SETDRIVER
 * Description: Set the driver type of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SETSLEWRATE
 * Description: Set slewrate of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SELECTGPIO
 * Description: Select gpio function of pinctrl pin
 * Argument:    The uint32_t pinctrl number
 *
 * Command:     PINCTRLC_GETPAD
 * Description: Describe the current configuration of one pad
 * Argument:    A pointer to an instance of struct pinctrl_getpad_s
 *
 */

#define PINCTRLC_SETFUNCTION _PINCTRLIOC(1)
#define PINCTRLC_SETSTRENGTH _PINCTRLIOC(2)
#define PINCTRLC_SETDRIVER   _PINCTRLIOC(3)
#define PINCTRLC_SETSLEWRATE _PINCTRLIOC(4)
#define PINCTRLC_SELECTGPIO  _PINCTRLIOC(5)
#define PINCTRLC_GETPAD      _PINCTRLIOC(6)

/* Validity bits for struct pinctrl_padinfo_s.  A field is meaningful only
 * when its bit is set in the have member: a pad may have no function
 * select, no bias, no drive strength.
 */

#define PINCTRL_HAVE_FUNCTION (1 << 0)
#define PINCTRL_HAVE_STRENGTH (1 << 1)
#define PINCTRL_HAVE_PULL     (1 << 2)
#define PINCTRL_HAVE_SLEWRATE (1 << 3)
#define PINCTRL_HAVE_INPUT    (1 << 4)
#define PINCTRL_HAVE_SCHMITT  (1 << 5)

#define PINCTRL_NAME_MAX      24  /* Longest name plus a terminator */
#define PINCTRL_EXTRA_MAX     48

/* One pad in a controller's name table: the pad's name, then the name of
 * each documented function select in select order.  Pass NULL in a slot
 * whose select the manual does not document.  The compound literal sizes
 * the array to exactly what is listed; at file scope it has static
 * storage.
 */

#define PINCTRL_PADNAME(padname, ...)                                     \
  {                                                                       \
    (padname), (FAR const char *const[]){__VA_ARGS__},                    \
    nitems(((FAR const char *const[]){__VA_ARGS__}))                      \
  }

/* Access macros ************************************************************/

/****************************************************************************
 * Name: PINCTRL_SETFUNCTION
 *
 * Description:
 *   Set the mux function of the pinctrl pin.
 *
 * Input Parameters:
 *   dev      - Device-specific state data.
 *   pin      - the pinctrl controller number.
 *   function - the pinctrl pin function number.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETFUNCTION(dev, pin, function) ((dev)->ops->set_function(dev, pin, function))

/****************************************************************************
 * Name: PINCTRL_SETSTRENGTH
 *
 * Description:
 *   Set the driver strength of the pinctrl pin
 *
 * Input Parameters:
 *   dev      - Device-specific state data.
 *   pin      - the pinctrl controller number.
 *   strength - the pinctrl pin driver strength number.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETSTRENGTH(dev, pin, strength) ((dev)->ops->set_strength(dev, pin, strength))

/****************************************************************************
 * Name: PINCTRL_SETDRIVER
 *
 * Description:
 *   Set the driver type of the pinctrl pin
 *
 * Input Parameters:
 *   dev    - Device-specific state data.
 *   pin    - the pinctrl controller number.
 *   driver - the pinctrl_drivertype_e type.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETDRIVER(dev, pin, driver) ((dev)->ops->set_driver(dev, pin, driver))

/****************************************************************************
 * Name: PINCTRL_SETSLEWRATE
 *
 * Description:
 *   Set the slewrate of the pinctrl pin
 *
 * Input Parameters:
 *   dev      - Device-specific state data.
 *   pin      - the pinctrl controller number.
 *   slewrate - the slewsrate state.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETSLEWRATE(dev, pin, slewrate) ((dev)->ops->set_slewrate(dev, pin, slewrate))

/****************************************************************************
 * Name: PINCTRL_SELECTGPIO
 *
 * Description:
 *   Select gpio function the pinctrl pin
 *
 * Input Parameters:
 *   dev   - Device-specific state data.
 *   pin   - the pinctrl controller number.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SELECTGPIO(dev, pin) ((dev)->ops->select_gpio(dev, pin))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Identifies the type of the pinctrl driver type */

enum pinctrl_drivertype_e
{
  BIAS_DISABLE = 0,
  BIAS_PULLUP,
  BIAS_PULLDOWN,
  BIAS_STRONG_PULLDOWN,
  BIAS_NDRIVERTYPES
};

/****************************************************************************
 * This describes pinctrl ioctl structure in pinctrl command operation.
 * the follow command use this structure:
 * PINCTRLC_SETFUNCTION, PINCTRLC_SETSTRENGTH,
 * PINCTRLC_SETDRIVER, PINCTRLC_SETSLEWRATE.
 ****************************************************************************/

struct pinctrl_param_s
{
  uint32_t pin;
  union
  {
    uint32_t                  function;
    uint32_t                  strength;
    enum pinctrl_drivertype_e type;
    uint32_t                  slewrate;
  } para;
};

/* What a controller can say about one pad.  Self contained: the strings
 * are embedded, so the same structure serves the get_pad method and the
 * PINCTRLC_GETPAD ioctl across the user/kernel boundary.  An empty name
 * means unnamed.
 */

struct pinctrl_padinfo_s
{
  uint32_t have;                       /* PINCTRL_HAVE_* validity bits */
  char     name[PINCTRL_NAME_MAX];     /* Pad name */
  uint32_t function;                   /* Current function select */
  char     funcname[PINCTRL_NAME_MAX]; /* What that function selects */
  uint32_t strength;                   /* Drive strength, hardware units */
  bool     pullup;                     /* Pull up enabled */
  bool     pulldown;                   /* Pull down enabled */
  uint32_t slewrate;                   /* Slew rate, hardware units */
  bool     input;                      /* Input buffer enabled */
  bool     schmitt;                    /* Schmitt trigger enabled */
  char     extra[PINCTRL_EXTRA_MAX];   /* Controller specific key:value
                                        * fields, appended to the pad's
                                        * /proc/pinctrl line */
};

/* PINCTRLC_GETPAD argument */

struct pinctrl_getpad_s
{
  uint32_t pin;                        /* In */
  struct pinctrl_padinfo_s info;       /* Out */
};

/* One pad in a controller's name table; declare entries with
 * PINCTRL_PADNAME().
 */

struct pinctrl_padname_s
{
  FAR const char *name;                /* Pad name */
  FAR const char *const *funcs;        /* Function select names, in order */
  uint32_t nfuncs;                     /* Entries in funcs */
};

/* pinctrl interface methods */

struct pinctrl_dev_s;
struct pinctrl_ops_s
{
  int (*set_function)(FAR struct pinctrl_dev_s *dev, uint32_t pin,
                      uint32_t function);
  int (*set_strength)(FAR struct pinctrl_dev_s *dev,  uint32_t pin,
                      uint32_t strength);
  int (*set_driver)(FAR struct pinctrl_dev_s *dev, uint32_t pin,
                    enum pinctrl_drivertype_e type);
  int (*set_slewrate)(FAR struct pinctrl_dev_s *dev, uint32_t pin,
                      uint32_t slewrate);
  int (*select_gpio)(FAR struct pinctrl_dev_s *dev, uint32_t pin);

  /* Describe one pad.  The only member here that reads rather than
   * writes.  Optional; a controller without it is listed in /proc/pinctrl
   * with a note and PINCTRLC_GETPAD returns -ENOTSUP.
   *
   * Zero the structure, fill only what the pad really has, and set the
   * matching PINCTRL_HAVE_* bits.  Fields the structure has no member for
   * go in extra, in the same key:value form the /proc/pinctrl renderer
   * uses.  Returns OK, or -EINVAL for a pin this controller does not
   * have.
   */

  int (*get_pad)(FAR struct pinctrl_dev_s *dev, uint32_t pin,
                 FAR struct pinctrl_padinfo_s *info);
};

struct pinctrl_dev_s
{
  /* "Lower half" operations provided by the pinctrl lower half */

  FAR const struct pinctrl_ops_s *ops;

  /* Pads this controller has; get_pad answers for pins 0..npins-1 */

  uint32_t npins;
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinctrl_padname
 *
 * Description:
 *   The name of pin in a PINCTRL_PADNAME() table of npads entries, or
 *   NULL if the pin is out of range or unnamed.
 *
 ****************************************************************************/

static inline FAR const char *
pinctrl_padname(FAR const struct pinctrl_padname_s *table, size_t npads,
                uint32_t pin)
{
  return pin < npads ? table[pin].name : NULL;
}

/****************************************************************************
 * Name: pinctrl_funcname
 *
 * Description:
 *   The name of function select func on pin, or NULL if the table does
 *   not document it.
 *
 ****************************************************************************/

static inline FAR const char *
pinctrl_funcname(FAR const struct pinctrl_padname_s *table, size_t npads,
                 uint32_t pin, uint32_t func)
{
  return pin < npads && func < table[pin].nfuncs ?
         table[pin].funcs[func] : NULL;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: pinctrl_register
 *
 * Description:
 *   Register PINCTRL device driver.
 *
 ****************************************************************************/

int pinctrl_register(FAR struct pinctrl_dev_s *dev, int minor);

/****************************************************************************
 * Name: pinctrl_unregister
 *
 * Description:
 *   Unregister PINCTRL device driver.
 *
 ****************************************************************************/

void pinctrl_unregister(FAR struct pinctrl_dev_s *dev, int minor);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_PINCTRL_PINCTRL_H */
