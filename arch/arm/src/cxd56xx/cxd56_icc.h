/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_icc.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_ICC_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_ICC_H

#define CXD56_PROTO_MSG       0 /* Generic message */
#define CXD56_PROTO_MBX       1
#define CXD56_PROTO_SEM       2
#define CXD56_PROTO_FLG       3
#define CXD56_PROTO_MPF       4
#define CXD56_PROTO_DBG       5
#define CXD56_PROTO_AUDIO     6
#define CXD56_PROTO_CALLBACK  7
#define CXD56_PROTO_HOTSLEEP  8
#define CXD56_PROTO_IMAGE     9
#define CXD56_PROTO_PM       10 /* Power manager */
#define CXD56_PROTO_HOSTIF   11
#define CXD56_PROTO_SYSCTL   12
#define CXD56_PROTO_GNSS     13
#define CXD56_PROTO_SIG      15 /* Inter-CPU Comm signal */

typedef int (*cxd56_icchandler_t)(int cpuid, int protoid, uint32_t pdata,
                                  uint32_t data, void *userdata);
typedef int (*cxd56_iccsighandler_t)(int8_t signo, uint16_t sigdata,
                                     uint32_t data, void *userdata);

struct cxd56_iccmsg_s
{
  int8_t cpuid;
  int8_t msgid;
  uint16_t protodata;
  uint32_t data;
};
typedef struct cxd56_iccmsg_s iccmsg_t;

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int cxd56_iccinit(int protoid);
int cxd56_iccinitmsg(int cpuid);
void cxd56_iccuninit(int protoid);
void cxd56_iccuninitmsg(int cpuid);
int cxd56_iccregisterhandler(int protoid, cxd56_icchandler_t handler,
                             void *data);
int cxd56_iccregistersighandler(int cpuid, cxd56_iccsighandler_t handler,
                                void *data);
int cxd56_iccsend(int protoid, iccmsg_t *msg, int32_t ms);
int cxd56_iccrecv(int protoid, iccmsg_t *msg, int32_t ms);
int cxd56_iccsendmsg(iccmsg_t *msg, int32_t ms);
int cxd56_iccrecvmsg(iccmsg_t *msg, int32_t ms);
int cxd56_iccsignal(int8_t cpuid, int8_t signo, int16_t sigdata,
                    uint32_t data);
int cxd56_iccnotify(int cpuid, int signo, void *sigdata);

void cxd56_iccinitialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_ICC_H */
