/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpu1signal.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

typedef void (*cxd56_cpu1sighandler_t)(uint32_t data, FAR void *userdata);

extern int cxd56_cpu1siginit(uint8_t cpu1dev, FAR void *data);
extern int cxd56_cpu1siguninit(uint8_t cpu1dev);
extern void cxd56_cpu1sigregisterhandler(uint8_t                cpu1dev,
                                         cxd56_cpu1sighandler_t handler);
extern void cxd56_cpu1sigunregisterhandler(uint8_t cpu1dev);
extern int cxd56_cpu1sigsend(uint8_t sigtype, uint32_t data);

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_CPU1SIGNAL_H */
