/****************************************************************************
 * include/nuttx/power/regulator.h
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

#ifndef __INCLUDE_NUTTX_POWER_REGULATOR_H
#define __INCLUDE_NUTTX_POWER_REGULATOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/power/pm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum regulator_mode_e
{
  REGULATOR_MODE_INVALID = 0,
  REGULATOR_MODE_FAST,
  REGULATOR_MODE_NORMAL,
  REGULATOR_MODE_IDLE,
  REGULATOR_MODE_STANDBY,
};

struct regulator_dev_s;

struct regulator_s
{
  struct work_s disable_work;
  int min_uv;
  int max_uv;
  struct list_node list;
  FAR struct regulator_dev_s *rdev;
};

struct regulator_state_s
{
  int uv;
  enum regulator_mode_e mode;
};

struct regulator_ops_s
{
  CODE int (*list_voltage)(FAR struct regulator_dev_s *rdev,
                           unsigned int selector);
  CODE int (*set_voltage)(FAR struct regulator_dev_s *rdev, int min_uv,
                          int max_uv, FAR unsigned int *selector);
  CODE int (*set_voltage_sel)(FAR struct regulator_dev_s *rdev,
                              unsigned int selector);
  CODE int (*get_voltage)(FAR struct regulator_dev_s *rdev);
  CODE int (*get_voltage_sel)(FAR struct regulator_dev_s *rdev);
  CODE int (*enable)(FAR struct regulator_dev_s *rdev);
  CODE int (*is_enabled)(FAR struct regulator_dev_s *rdev);
  CODE int (*disable)(FAR struct regulator_dev_s *rdev);
  CODE int (*enable_pulldown)(FAR struct regulator_dev_s *rdev);
  CODE int (*disable_pulldown)(FAR struct regulator_dev_s *rdev);
  CODE int (*set_mode)(FAR struct regulator_dev_s *rdev,
                       enum regulator_mode_e mode);
  CODE enum regulator_mode_e (*get_mode)(FAR struct regulator_dev_s *rdev);
  CODE int (*set_suspend_mode)(FAR struct regulator_dev_s *rdev,
                               enum regulator_mode_e mode);
  CODE int (*set_suspend_voltage)(FAR struct regulator_dev_s *, int uv);
  CODE int (*resume)(FAR struct regulator_dev_s *rdev);
};

/* This structure describes the regulators capabilities */

struct regulator_desc_s
{
  FAR const char *name;             /* Regulator output name */
  unsigned int   id;                /* Numerical id for a given regulator of
                                     * a device
                                     */
  unsigned int   n_voltages;        /* Number of discrete voltages */
  unsigned int   vsel_reg;          /* Device register for voltage selection */
  unsigned int   vsel_mask;         /* Register mask, for voltage selection */
  unsigned int   enable_reg;        /* Device register for enable/disable */
  unsigned int   enable_mask;       /* Register mask for enable/disable */
  unsigned int   enable_time;       /* Time for initial enable of regulator */
  unsigned int   ramp_delay;        /* Rate of change for setting new voltage */
  unsigned int   uv_step;           /* Voltage per step if linear mapping_uv */
  unsigned int   min_uv;            /* Minimum acceptable voltage */
  unsigned int   max_uv;            /* Maximum acceptable voltage */
  unsigned int   pulldown;          /* Enable pulldown when disabled */
  unsigned int   pulldown_reg;      /* Device register, for pulldown enable */
  unsigned int   pulldown_mask;     /* Register mask, for pulldown enable */
  unsigned int   apply_uv;          /* If true, the voltage specifed (between)
                                     * min_uv and max_uv will be applied during
                                     * initialisation.
                                     */
  unsigned int   boot_on;           /* true if this regulator is to be enabled
                                     * at power up/reset
                                     */
  unsigned int   always_on;
  FAR const char *supply_name;
#ifdef CONFIG_PM
  unsigned int auto_lp;
  unsigned int domain;
  struct regulator_state_s states[PM_COUNT];
#endif
};

struct regulator_dev_s
{
  FAR const struct regulator_desc_s *desc;
  FAR const struct regulator_ops_s *ops;
  uint32_t use_count;
  uint32_t open_count;
  mutex_t regulator_lock;
  struct list_node list;
  struct list_node consumer_list;
  FAR struct regulator_s *supply;
#ifdef CONFIG_PM
  struct pm_callback_s pm_cb;
#endif
  FAR void *priv;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: regulator_register
 *
 * Description:
 *   Register a lower half regulator driver with the common, upper-half
 *   regulator driver.
 *
 * Input parameters:
 *   desc - The regulator descriptor struct.
 *   ops  - The regulator operations pointer
 *
 * Returned value:
 *    The pointer to struct regulator_dev_s on success or NULL on failure.
 *
 ****************************************************************************/

FAR struct regulator_dev_s *
regulator_register(FAR const struct regulator_desc_s *desc,
                   FAR const struct regulator_ops_s *ops,
                   FAR void *priv);

/****************************************************************************
 * Name: regulator_unregister
 *
 * Description:
 *   Unregister a lower half regulator driver with the common, upper-half
 *   regulator driver.
 *
 * Input parameters:
 *   rdev - The regulator dev pointer.
 *
 * Returned value:
 *    N/A
 *
 ****************************************************************************/

void regulator_unregister(FAR struct regulator_dev_s *rdev);

/****************************************************************************
 * Name: regulator_gpio_init
 *
 * Description:
 *
 * Input Parameters:
 *
 *   iodev  - The ioexpander dev pointer.
 *   desc   - The regulator desc pointer, must contain follow section
 *            name          - The regulator name.
 *            enable_reg    - The regulator gpio pin number.
 *            enable_mask   -
 *                            true : enable is high, disable is low
 *                            false: enable is low,  disable is high
 *
 * Returned Value:
 *
 ****************************************************************************/

int regulator_gpio_init(FAR struct ioexpander_dev_s *iodev,
                        FAR const struct regulator_desc_s *desc);

#if defined(CONFIG_REGULATOR_RPMSG)

/****************************************************************************
 * Name: regulator_rpmsg_get
 *
 * Description:
 *
 * Input Parameters:
 *
 *   name - the name for register the rpmsg regulator dev
 *
 * Returned Value:
 *
 *   Regulator dev pointer
 *
 ****************************************************************************/

FAR struct regulator_dev_s *regulator_rpmsg_get(FAR const char *name);

/****************************************************************************
 * Name: regulator_rpmsg_server_init
 *
 * Description:
 *
 *   Establish rpmsg channel for the operations of the remote regulator
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int regulator_rpmsg_server_init(void);

#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_POWER_REGULATOR_H */
