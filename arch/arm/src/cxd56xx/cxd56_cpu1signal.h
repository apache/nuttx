/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpu1signal.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_CPU1SIGNAL_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_CPU1SIGNAL_H

/* CPU1 Notifyable functions */

#define CXD56_CPU1_DATA_TYPE_GNSS         0
#define CXD56_CPU1_DATA_TYPE_GEOFENCE     1
#define CXD56_CPU1_DATA_TYPE_PVTLOG       2
#define CXD56_CPU1_DATA_TYPE_AGPS         3
#define CXD56_CPU1_DATA_TYPE_RTK          4
#define CXD56_CPU1_DATA_TYPE_SPECTRUM     5
#define CXD56_CPU1_DATA_TYPE_INFO         6
#define CXD56_CPU1_DATA_TYPE_BACKUP       7
#define CXD56_CPU1_DATA_TYPE_CEP          8
#define CXD56_CPU1_DATA_TYPE_CEPFILE      9
#define CXD56_CPU1_DATA_TYPE_BKUPFILE     10
#define CXD56_CPU1_DATA_TYPE_GPSEPHEMERIS 11
#define CXD56_CPU1_DATA_TYPE_GLNEPHEMERIS 12
#define CXD56_CPU1_DATA_TYPE_CPUFIFOAPI   13
#define CXD56_CPU1_DATA_TYPE_SBAS         14
#define CXD56_CPU1_DATA_TYPE_DCREPORT     15
#define CXD56_CPU1_DATA_TYPE_SARRLM       16
#define CXD56_CPU1_DATA_TYPE_MAX          17

/* CPU1 devices */

#define CXD56_CPU1_DEV_GNSS      (CXD56_CPU1_DATA_TYPE_GNSS)
#define CXD56_CPU1_DEV_GEOFENCE  (CXD56_CPU1_DATA_TYPE_GEOFENCE)

#define CXD56_CPU1_DEV_MASK       0xff
#define CXD56_CPU1_GET_DEV(DATA)  ((DATA) & CXD56_CPU1_DEV_MASK)
#define CXD56_CPU1_GET_DATA(DATA) ((DATA) >> 8)

#if CXD56_CPU1_DATA_TYPE_MAX > (CXD56_CPU1_DEV_MASK + 1)
#error "CXD56_CPU1_DEV must be smaller than 0xf"
#endif

typedef void (*cxd56_cpu1sighandler_t)(uint32_t data, void *userdata);

extern int cxd56_cpu1siginit(uint8_t cpu1dev, void *data);
extern int cxd56_cpu1siguninit(uint8_t cpu1dev);
extern void cxd56_cpu1sigregisterhandler(uint8_t                cpu1dev,
                                         cxd56_cpu1sighandler_t handler);
extern void cxd56_cpu1sigunregisterhandler(uint8_t cpu1dev);
extern int cxd56_cpu1sigsend(uint8_t sigtype, uint32_t data);

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_CPU1SIGNAL_H */
