/****************************************************************************
 * include/nuttx/cpufreq.h
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

#ifndef __INCLUDE_NUTTX_CPUFREQ_H
#define __INCLUDE_NUTTX_CPUFREQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/compiler.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mutex.h>
#include <nuttx/queue.h>

#ifdef CONFIG_CPUFREQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Terminates a frequency table */

#define CPUFREQ_TABLE_END        (~0u)

/* Passed as a QoS bound to leave that side unconstrained */

#define CPUFREQ_NO_LIMIT         (0)

/* ioctl commands for the optional /dev/cpufreq character device.
 *
 * CPUFREQIOC_GET_FREQUENCY  Arg: FAR unsigned int *.  Receives the
 *                           current frequency.
 * CPUFREQIOC_SET_REQUEST    Arg: FAR const struct cpufreq_request_s *.
 *                           Installs or updates this file descriptor's
 *                           QoS request.  Released on close.
 * CPUFREQIOC_CLEAR_REQUEST  Arg: none.  Removes this descriptor's
 *                           request.
 * CPUFREQIOC_GET_TABLE      Arg: FAR struct cpufreq_table_query_s *.
 *                           Copies out the frequency table.
 */

#define CPUFREQIOC_GET_FREQUENCY _CPUFREQIOC(0x0001)
#define CPUFREQIOC_SET_REQUEST   _CPUFREQIOC(0x0002)
#define CPUFREQIOC_CLEAR_REQUEST _CPUFREQIOC(0x0003)
#define CPUFREQIOC_GET_TABLE     _CPUFREQIOC(0x0004)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cpufreq_policy;

/* One entry of a driver's frequency table.  The table is ascending and
 * ends with an entry whose frequency is CPUFREQ_TABLE_END.  The unit is
 * the lower half's choice (kHz by Linux convention), as long as every
 * consumer of the same policy agrees.
 */

struct cpufreq_frequency_table
{
  unsigned int frequency;
};

/* The lower half: what a platform must provide.  get_table and
 * target_index are mandatory; the rest may be NULL.
 */

struct cpufreq_driver
{
  CODE FAR const struct cpufreq_frequency_table *
                 (*get_table)(FAR struct cpufreq_policy *policy);
  CODE int (*target_index)(FAR struct cpufreq_policy *policy,
                           unsigned int index);
  CODE int (*get_frequency)(FAR struct cpufreq_policy *policy);
  CODE int (*suspend)(FAR struct cpufreq_policy *policy);
  CODE int (*resume)(FAR struct cpufreq_policy *policy);
};

/* The policy: one per system.  The driver pointer must stay the first
 * member; existing consumers reach the driver by casting the policy.
 */

struct cpufreq_policy
{
  FAR struct cpufreq_driver *driver;

  /* Internal to the upper half */

  FAR const struct cpufreq_frequency_table *table;
  unsigned int nentries;
  unsigned int current;      /* Index of the entry last applied  */
  bool suspended;
  mutex_t lock;
  dq_queue_t requests;
};

/* One QoS request: a [min, max] window the resolved frequency must
 * respect.  See cpufreq.c for how competing windows resolve.
 */

struct cpufreq_qos
{
  dq_entry_t node;
  unsigned int min;
  unsigned int max;
};

/* Argument of CPUFREQIOC_SET_REQUEST */

struct cpufreq_request_s
{
  unsigned int min;          /* Lowest acceptable, or CPUFREQ_NO_LIMIT  */
  unsigned int max;          /* Highest acceptable, or CPUFREQ_NO_LIMIT */
};

/* Argument of CPUFREQIOC_GET_TABLE */

struct cpufreq_table_query_s
{
  FAR unsigned int *frequencies;  /* Where to put them            */
  unsigned int maxlen;            /* Room, in entries             */
  unsigned int nentries;          /* How many the table really has */
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
 * Name: cpufreq_init
 *
 * Description:
 *   Bring up the framework over a lower half.  Called once, by the
 *   platform, after the hardware it drives is ready.  Registers
 *   /dev/cpufreq when CONFIG_CPUFREQ_CHARDEV is enabled.
 *
 * Input Parameters:
 *   driver - The lower half.  Must outlive the framework.
 *
 * Returned Value:
 *   Zero on success; a negated errno otherwise.  -EBUSY if called twice.
 *
 ****************************************************************************/

int cpufreq_init(FAR struct cpufreq_driver *driver);

/****************************************************************************
 * Name: cpufreq_policy_get
 *
 * Description:
 *   The system's policy, or NULL before cpufreq_init.
 *
 ****************************************************************************/

FAR struct cpufreq_policy *cpufreq_policy_get(void);

/****************************************************************************
 * Name: cpufreq_qos_add_request
 *
 * Description:
 *   Constrain the frequency to [min, max] and re-resolve.  Either bound
 *   may be CPUFREQ_NO_LIMIT.
 *
 * Returned Value:
 *   The request, to update or remove later; NULL on failure.
 *
 ****************************************************************************/

FAR struct cpufreq_qos *cpufreq_qos_add_request(
                             FAR struct cpufreq_policy *policy,
                             unsigned int min, unsigned int max);

/****************************************************************************
 * Name: cpufreq_qos_update_request
 *
 * Description:
 *   Change an installed request's window and re-resolve.
 *
 ****************************************************************************/

int cpufreq_qos_update_request(FAR struct cpufreq_qos *qos,
                               unsigned int min, unsigned int max);

/****************************************************************************
 * Name: cpufreq_qos_remove_request
 *
 * Description:
 *   Withdraw a request and re-resolve without it.  The request is freed.
 *
 ****************************************************************************/

int cpufreq_qos_remove_request(FAR struct cpufreq_qos *qos);

/****************************************************************************
 * Name: cpufreq_suspend / cpufreq_resume
 *
 * Description:
 *   Hand the hardware to the lower half's suspend path and back.  While
 *   suspended the resolver leaves the hardware alone; requests are still
 *   accepted and take effect on resume.
 *
 ****************************************************************************/

int cpufreq_suspend(void);
int cpufreq_resume(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_CPUFREQ */
#endif /* __INCLUDE_NUTTX_CPUFREQ_H */
