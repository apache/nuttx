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
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct regulator_dev;

struct regulator
{
  struct work_s disable_work;
  int min_uv;
  int max_uv;
  struct list_node list;
  struct regulator_dev *rdev;
  uint64_t id;
};

struct regulator_ops
{
  int (*list_voltage)(struct regulator_dev *rdev, unsigned selector);
  int (*set_voltage)(struct regulator_dev *rdev, int min_uv,
                     int max_uv, unsigned *selector);
  int (*set_voltage_sel)(struct regulator_dev *rdev, unsigned selector);
  int (*get_voltage)(struct regulator_dev *rdev);
  int (*get_voltage_sel)(struct regulator_dev *rdev);
  int (*enable)(struct regulator_dev *rdev);
  int (*is_enabled)(struct regulator_dev *rdev);
  int (*disable)(struct regulator_dev *rdev);
};

/* This structure defines the regulator state structure */

struct regulator_desc
{
  const char *name;
  int id;
  unsigned int n_voltages;
  unsigned int vsel_reg;
  unsigned int vsel_mask;
  unsigned int enable_reg;
  unsigned int enable_mask;
  unsigned int enable_time;
  unsigned int ramp_delay;
  unsigned int uv_step;
  unsigned int min_uv;
  unsigned int max_uv;
  unsigned int apply_uv;
  bool boot_on;
};

struct regulator_dev
{
  const struct regulator_desc *desc;
  const struct regulator_ops *ops;
  uint32_t use_count;
  uint32_t open_count;
  sem_t regulator_sem;
  struct list_node list;
  struct list_node consumer_list;

  void *priv;
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
 *    The pointer to struct regulator_dev on success or NULL on failure.
 *
 ****************************************************************************/

struct regulator_dev *regulator_register(const struct regulator_desc *desc,
                                         const struct regulator_ops *ops,
                                         void *priv);

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

void regulator_unregister(struct regulator_dev *rdev);


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

int regulator_gpio_init(struct ioexpander_dev_s *iodev,
                        const struct regulator_desc *desc);

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

struct regulator_dev *regulator_rpmsg_get(const char *name);

/****************************************************************************
 * Name: regulator_rpmsg_init
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int regulator_rpmsg_init(void);

#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_POWER_REGULATOR_H */
