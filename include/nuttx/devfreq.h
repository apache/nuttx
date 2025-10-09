/****************************************************************************
 * include/nuttx/devfreq.h
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

#ifndef __INCLUDE_NUTTX_DEVFREQ_H
#define __INCLUDE_NUTTX_DEVFREQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/devfreq/devfreq_qos.h>
#include <nuttx/list.h>
#include <nuttx/notifier.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVFREQ_PRECHANGE     0
#define DEVFREQ_POSTCHANGE    1

/* Special Values of .frequency field */

#define DEVFREQ_ENTRY_INVALID ~0u
#define DEVFREQ_ENTRY_END     ~1u

#define DEVFREQ_RELATION_L    0   /* lowest frequency at or above target */
#define DEVFREQ_RELATION_H    1   /* highest frequency below or at target */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct devfreq_s
{
  char name[NAME_MAX];
  struct list_node node;

  FAR const struct devfreq_governor_s *governor;
  FAR const struct devfreq_driver_s *driver;

  FAR const uint32_t *freq_table;

  struct qos_constraints_s constraints;

  struct blocking_notifier_head notifier_list;

  uint32_t min;                     /* in kHz */
  uint32_t max;                     /* in kHz */
  uint32_t cur;                     /* in kHz */

  bool suspended;

  mutex_t lock;

  FAR void *priv;
};

struct devfreq_governor_s
{
  char name[NAME_MAX];
  CODE int (*init)(FAR struct devfreq_s *devfreq);
  CODE int (*exit)(FAR struct devfreq_s *devfreq);
  CODE int (*start)(FAR struct devfreq_s *devfreq);
  CODE void (*stop)(FAR struct devfreq_s *devfreq);
  CODE uint32_t (*limit)(FAR struct devfreq_s *devfreq);
};

struct devfreq_driver_s
{
  CODE FAR const uint32_t *
           (*get_table)(FAR struct devfreq_s *devfreq);
  CODE int (*target_index)(FAR struct devfreq_s *devfreq,
                           size_t index);
  CODE uint32_t (*get_frequency)(FAR struct devfreq_s *devfreq);
  CODE int (*suspend)(FAR struct devfreq_s *devfreq);
  CODE int (*resume)(FAR struct devfreq_s *devfreq);
};

struct devfreq_notifier_s
{
  uint32_t old;
  uint32_t new;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: devfreq_register
 *
 * Description:
 *   Register devfreq device
 *
 * Input Parameters:
 *   name - device name
 *   governor - governor
 *   driver - driver
 *   priv - private data
 *
 * Returned Value:
 *   devfreq device on success; NULL on failure
 *
 ****************************************************************************/

FAR struct devfreq_s *devfreq_register(
                          FAR const char *name,
                          FAR const struct devfreq_governor_s *governor,
                          FAR const struct devfreq_driver_s *driver,
                          FAR void *priv);

/****************************************************************************
 * Name: devfreq_unregister
 *
 * Description:
 *   unregister devfreq
 *
 * Input Parameters:
 *   devfreq - devfreq_s handle
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int devfreq_unregister(FAR struct devfreq_s *devfreq);

/****************************************************************************
 * Name: devfreq_suspend
 *
 * Description:
 *   suspend devfreq governors
 *
 * Input Parameters:
 *   devfreq - devfreq_s handle
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int devfreq_suspend(FAR struct devfreq_s *devfreq);

/****************************************************************************
 * Name: devfreq_resume
 *
 * Description:
 *   resume devfreq governors
 *
 * Input Parameters:
 *   devfreq - devfreq_s handle
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int devfreq_resume(FAR struct devfreq_s *devfreq);

/****************************************************************************
 * Name: devfreq_set_governor
 *
 * Description:
 *   set devfreq governor
 *
 * Input Parameters:
 *   devfreq - devfreq_s handle
 *   governor - governor handle
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int devfreq_register_notifier(FAR struct devfreq_s *devfreq,
                              FAR struct notifier_block *nb);

/****************************************************************************
 * Name: devfreq_unregister_notifier
 *
 * Description:
 *   unregister devfreq notifier
 *
 * Input Parameters:
 *   devfreq - devfreq_s handle
 *   nb - notifier block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int devfreq_unregister_notifier(FAR struct devfreq_s *devfreq,
                                FAR struct notifier_block *nb);

/****************************************************************************
 * Name: devfreq_get_frequency
 *
 * Description:
 *   get the current device frequency (in kHz)
 *
 * Input Parameters:
 *   devfreq - devfreq_s handle
 *
 * Returned Value:
 *   a non-negative value
 *
 ****************************************************************************/

uint32_t devfreq_get_frequency(FAR struct devfreq_s *devfreq);

/****************************************************************************
 * Name: devfreq_qos_add_request
 *
 * Description:
 *   Insert new frequency QoS request
 *
 * Input Parameters:
 *   policy - devfreq_policy handle
 *   min    - min freq
 *   max    - max freq
 *
 * Returned Value:
 *   qos handle for update and remove, or NULL if fail
 *
 ****************************************************************************/

FAR struct qos_request_s *devfreq_qos_add_request(
                            FAR struct devfreq_s *devfreq,
                            uint32_t min, uint32_t max);

/****************************************************************************
 * Name: devfreq_qos_update_request
 *
 * Description:
 *   Update frequency QoS request from its list.
 *
 * Input Parameters:
 *   qos - Request to remove.
 *   min - min freq
 *   max - max freq
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int devfreq_qos_update_request(FAR struct devfreq_s *devfreq,
                               FAR struct qos_request_s *qos,
                               uint32_t min, uint32_t max);

/****************************************************************************
 * Name: devfreq_qos_remove_request
 *
 * Description:
 *   Remove frequency QoS request from its list.
 *
 * Input Parameters:
 *   qos - Request to remove.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int devfreq_qos_remove_request(FAR struct devfreq_s *devfreq,
                               FAR struct qos_request_s *req);

/****************************************************************************
 * Name: devfreq_find_by_name
 *
 * Description:
 *   find a devfreq entry from global list by name
 *
 * Input Parameters:
 *   name - devfreq name
 *
 * Returned Value:
 *   devfreq handle
 *
 ****************************************************************************/

FAR struct devfreq_s *devfreq_find_by_name(FAR const char *name);

/****************************************************************************
 * Name: devfreq_find_by_index
 *
 * Description:
 *   find a devfreq entry from global list by index
 *
 * Input Parameters:
 *   index - devfreq index
 *
 * Returned Value:
 *   devfreq handle
 *
 ****************************************************************************/

FAR struct devfreq_s *devfreq_find_by_index(size_t index);

/****************************************************************************
 * Name: devfreq_procfs_initialize
 *
 * Description:
 *   initialize procfs for devfreq, called by devfreq_initialize()
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void devfreq_procfs_initialize(void);

FAR struct devfreq_governor_s *devfreq_performance(void);
FAR struct devfreq_governor_s *devfreq_powersave(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_DEVFREQ_H */
