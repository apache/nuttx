/****************************************************************************
 * include/nuttx/sensors/cxd5610_gnss.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_CXD5610_GNSS_H
#define __INCLUDE_NUTTX_SENSORS_CXD5610_GNSS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct cxd5610_gnss_lowerhalf_s;

/* Structure for cxd5610 lower-half operations */

struct cxd5610_gnss_lowerops_s
{
  /* Configure the cxd5610 gnss sensor device */

  CODE int (*send)(FAR struct cxd5610_gnss_lowerhalf_s *lower,
                   FAR uint8_t *buffer, int buflen);
  CODE int (*recv)(FAR struct cxd5610_gnss_lowerhalf_s *lower,
                   FAR uint8_t *buffer, int buflen);
  CODE int (*enableint)(FAR struct cxd5610_gnss_lowerhalf_s *lower,
                        CODE void (*handler)(void));
  CODE int (*disableint)(FAR struct cxd5610_gnss_lowerhalf_s *lower);
};

/* Structure for cxd5610 lower-half driver */

struct cxd5610_gnss_lowerhalf_s
{
  FAR const struct cxd5610_gnss_lowerops_s *ops;
};

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
 * Name: cxd5610_gnss_register
 *
 * Description:
 *   Register the CXD5610 GNSS character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register.
 *   lower   - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd5610_gnss_register(FAR const char *devpath,
                          FAR struct cxd5610_gnss_lowerhalf_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_CXD5610_GNSS_H */
