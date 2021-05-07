/****************************************************************************
 * include/sys/sockio.h
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

#ifndef __INCLUDE_SYS_SOCKIO_H
#define __INCLUDE_SYS_SOCKIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Get NuttX configuration and NuttX-specific network IOCTL definitions */

#include <netinet/in.h>

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/net/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMSFNAMSIZ 8

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* RFC3678: IPv4 Options
 *
 * o  ioctl() SIOCGIPMSFILTER: to retrieve the list of source addresses
 *    that comprise the source filter along with the current filter mode.
 *
 * o  ioctl() SIOCSIPMSFILTER: to set or modify the source filter content
 *    (e.g., unicast source address list) or mode (exclude or include).
 *
 * Ioctl option                  Argument type
 * ----------------------------- ----------------------
 * SIOCGIPMSFILTER               struct ip_msfilter
 * SIOCSIPMSFILTER               struct ip_msfilter
 *
 * The structure ip_msfilter, defined in <netinet/in.h> is used with
 * these IOCTL commands to pass filter information.  The field
 * imsf_fmode is a 32-bit integer that identifies the filter mode.
 * The value of this field must be either MCAST_INCLUDE or
 * MCAST_EXCLUDE, which are likewise defined in <netinet/in.h>.
 */

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_SOCKIO_H */
