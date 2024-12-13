/****************************************************************************
 * include/nuttx/timers/capture.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_CAPTURE_H
#define __INCLUDE_NUTTX_TIMERS_CAPTURE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* Command:     CAPIOC_DUTYCYCLE
 * Description: Get the pwm duty from the capture.
 * Arguments:   int8_t pointer to the location to return the duty.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define CAPIOC_DUTYCYCLE _CAPIOC(1)

/* Command:     CAPIOC_FREQUENCE
 * Description: Get the pulse frequence from the capture.
 * Arguments:   int32_t pointer to the location to return the frequence.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define CAPIOC_FREQUENCE _CAPIOC(2)

/* Command:     CAPIOC_EDGES
 * Description: Get the pwm edges from the capture.
 * Arguments:   int32_t pointer to the location to return the edges.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define CAPIOC_EDGES     _CAPIOC(3)

/* Command:     CAPIOC_ALL
 * Description: Get the pwm duty, pulse frequence, pwm edges, from
 *              the capture.
 * Arguments:   A reference to struct cap_all_s.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define CAPIOC_ALL       _CAPIOC(4)

/* Command:     CAPIOC_PULSES
 * Description: Read pulse count value.
 * Arguments:   Int pointer to the location to return the count value.
 * Return:      OK on success; ERROR on failure.
 */

#define CAPIOC_PULSES    _CAPIOC(5)

/* Command:     CAPIOC_CLR_CNT
 * Description: Clear pulse count value. Stop counting and then starting
 *              back holds previous count value.
 * Arguments:   None.
 * Return:      OK on success; ERROR on failure.
 */

#define CAPIOC_CLR_CNT   _CAPIOC(6)

/* Command:     CAPIOC_FILTER
 * Description: Configures glitch filter.
 * Arguments:   uint32_t for glitch filter value in ns
 *              or 0 for disable.
 * Return:      OK on success; ERROR on failure.
 */

#define CAPIOC_FILTER    _CAPIOC(7)

/* Command:     CAPIOC_HANDLER
 * Description: Set user function on event callback.
 * Arguments:   xcpt_t type for callback function or NULL
 * Return:      OK on success; ERROR on failure.
 */

#define CAPIOC_HANDLER   _CAPIOC(8)

/* Command:     CAPIOC_ADD_WP
 * Description: Add wacthpoint to unit.
 * Arguments:   Int value to watch.
 * Return:      OK on success; ERROR on failure.
 */

#define CAPIOC_ADD_WP    _CAPIOC(9)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cap_all_s
{
  uint32_t freq;
  uint32_t edges;
  uint8_t  duty;
};

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct cap_lowerhalf_s;
struct cap_ops_s
{
  /* Required methods *******************************************************/

  /* Start the capture, resetting the configure of timers */

  CODE int (*start)(FAR struct cap_lowerhalf_s *lower);

  /* Stop the capture */

  CODE int (*stop)(FAR struct cap_lowerhalf_s *lower);

  /* Get the result pwm capture duty value */

  CODE int (*getduty)(FAR struct cap_lowerhalf_s *lower,
                      FAR uint8_t *duty);

  /* Get the result pwm capture frequence value */

  CODE int (*getfreq)(FAR struct cap_lowerhalf_s *lower,
                      FAR uint32_t *freq);

  /* Get the result pwm capture edges value */

  CODE int (*getedges)(FAR struct cap_lowerhalf_s *lower,
                       FAR uint32_t *edges);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct cap_lowerhalf_s *dev,
                    int cmd, unsigned long arg);
};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure.  "lower half" drivers will have an
 * internal structure definition that will be cast-compatible with this
 * structure definitions.
 */

struct cap_lowerhalf_s
{
  FAR const struct cap_ops_s *ops;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name: cap_register
 *
 * Description:
 *   Register the pulse capture lower half device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/cap0"
 *   lower - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.  The following
 *   possible error values may be returned (most are returned by
 *   register_driver()):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int cap_register(FAR const char *devpath,
                 FAR struct cap_lowerhalf_s *lower);

int cap_register_multiple(FAR const char *devpath,
                          FAR struct cap_lowerhalf_s **lower, int n);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TIMERS_CAPTURE_H */
