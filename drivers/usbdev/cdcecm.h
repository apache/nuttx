/****************************************************************************
 * drivers/usbdev/cdcecm.h
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

#ifndef __DRIVERS_USBDEV_CDCECM_H
#define __DRIVERS_USBDEV_CDCECM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>

#include <nuttx/usb/cdcecm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CDCECM_VERSIONNO         (0x0100)
#define CDCECM_MXDESCLEN         (80)
#define CDCECM_MAXSTRLEN         (CDCECM_MXDESCLEN - 2)
#define CDCECM_NCONFIGS          (1)
#define CDCECM_NINTERFACES       (2)
#define CDCECM_NUM_EPS           (3)

#define CDCECM_MANUFACTURERSTRID (1)
#define CDCECM_PRODUCTSTRID      (2)
#define CDCECM_SERIALSTRID       (3)
#define CDCECM_CONFIGSTRID       (4)
#define CDCECM_MACSTRID          (5)
#define CDCECM_NSTRIDS           (5)

#define CDCECM_STR_LANGUAGE      (0x0409) /* en-us */

#define CDCECM_CONFIGID_NONE     (0)
#define CDCECM_CONFIGID          (1)

#define CDCECM_SELFPOWERED       (0)
#define CDCECM_REMOTEWAKEUP      (0)

#ifndef MIN
#  define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#endif /* __DRIVERS_USBDEV_CDCECM_H */
