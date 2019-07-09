/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_icc.h
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
#define CXD56_PROTO_SYSCTL   12
#define CXD56_PROTO_GNSS     13
#define CXD56_PROTO_SIG      15 /* Inter-CPU Comm signal */

typedef int (*cxd56_icchandler_t)(int cpuid, int protoid, uint32_t pdata,
                                  uint32_t data, FAR void *userdata);
typedef int (*cxd56_iccsighandler_t)(int8_t signo, uint16_t sigdata,
                                     uint32_t data, FAR void *userdata);

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
                             FAR void *data);
int cxd56_iccregistersighandler(int cpuid, cxd56_iccsighandler_t handler,
                                FAR void *data);
int cxd56_iccsend(int protoid, FAR iccmsg_t *msg, int32_t ms);
int cxd56_iccrecv(int protoid, FAR iccmsg_t *msg, int32_t ms);
int cxd56_iccsendmsg(FAR iccmsg_t *msg, int32_t ms);
int cxd56_iccrecvmsg(FAR iccmsg_t *msg, int32_t ms);
int cxd56_iccsignal(int8_t cpuid, int8_t signo, int16_t sigdata,
                    uint32_t data);
int cxd56_iccnotify(int cpuid, int signo, FAR void *sigdata);

void cxd56_iccinitialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_ICC_H */
