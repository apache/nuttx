/****************************************************************************
 * include/nuttx/rptun/rptun.h
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#ifndef __INCLUDE_NUTTX_RPTUN_RPTUN_H
#define __INCLUDE_NUTTX_RPTUN_RPTUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RPTUN

#include <nuttx/fs/ioctl.h>
#include <openamp/open_amp.h>

#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPTUNIOC_START              _RPTUNIOC(1)
#define RPTUNIOC_STOP               _RPTUNIOC(2)

#define RPTUN_NOTIFY_ALL            (UINT32_MAX - 0)

/* Access macros ************************************************************/

/****************************************************************************
 * Name: RPTUN_GET_CPUNAME
 *
 * Description:
 *   Get remote cpu name
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   Cpu name on success, NULL on failure.
 *
 ****************************************************************************/

#define RPTUN_GET_CPUNAME(d) ((d)->ops->get_cpuname(d))

/****************************************************************************
 * Name: RPTUN_GET_FIRMWARE
 *
 * Description:
 *   Get remote firmware name
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   Firmware name on success, NULL on failure.
 *
 ****************************************************************************/

#define RPTUN_GET_FIRMWARE(d) ((d)->ops->get_firmware(d))

/****************************************************************************
 * Name: RPTUN_GET_ADDRENV
 *
 * Description:
 *   Get adress env list
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   Addrenv pointer on success, NULL on failure.
 *
 ****************************************************************************/

#define RPTUN_GET_ADDRENV(d) ((d)->ops->get_addrenv(d))

/****************************************************************************
 * Name: RPTUN_GET_RESOURCE
 *
 * Description:
 *   Get rptun resouce
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   Resource pointer on success, NULL on failure
 *
 ****************************************************************************/

#define RPTUN_GET_RESOURCE(d) ((d)->ops->get_resource(d))

/****************************************************************************
 * Name: RPTUN_IS_AUTOSTART
 *
 * Description:
 *   AUTO start or not
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   True autostart, false not autostart
 *
 ****************************************************************************/

#define RPTUN_IS_AUTOSTART(d) ((d)->ops->is_autostart(d))

/****************************************************************************
 * Name: RPTUN_IS_MASTER
 *
 * Description:
 *   IS master or not
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   True master, false remote
 *
 ****************************************************************************/

#define RPTUN_IS_MASTER(d) ((d)->ops->is_master(d))

/****************************************************************************
 * Name: RPTUN_START
 *
 * Description:
 *   START remote cpu
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPTUN_START(d) ((d)->ops->start(d))

/****************************************************************************
 * Name: RPTUN_STOP
 *
 * Description:
 *   STOP remote cpu
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPTUN_STOP(d) ((d)->ops->stop(d))

/****************************************************************************
 * Name: RPTUN_NOTIFY
 *
 * Description:
 *   Notify remote core there is a message to get.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   vqid - Message to notify
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPTUN_NOTIFY(d,v) ((d)->ops->notify(d,v))

/****************************************************************************
 * Name: RPTUN_REGISTER_CALLBACK
 *
 * Description:
 *   Attach to receive a callback when something is received on RPTUN
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   callback - The function to be called when something has been received
 *   arg      - A caller provided value to return with the callback
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPTUN_REGISTER_CALLBACK(d,c,a) ((d)->ops->register_callback(d,c,a))

/****************************************************************************
 * Name: RPTUN_UNREGISTER_CALLBACK
 *
 * Description:
 *   Detach RPTUN callback
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPTUN_UNREGISTER_CALLBACK(d) ((d)->ops->register_callback(d,NULL,NULL))

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE int (*rptun_callback_t)(FAR void *arg, uint32_t vqid);

struct rptun_addrenv_s
{
  uintptr_t pa;
  uintptr_t da;
  size_t    size;
};

struct __attribute__((aligned(B2C(8)))) rptun_rsc_s
{
  struct resource_table    rsc_tbl_hdr;
  unsigned int             offset[2];
  struct fw_rsc_trace      log_trace;
  struct fw_rsc_vdev       rpmsg_vdev;
  struct fw_rsc_vdev_vring rpmsg_vring0;
  struct fw_rsc_vdev_vring rpmsg_vring1;
  unsigned int             buf_size;
};

struct rptun_dev_s;
struct rptun_ops_s
{
  CODE FAR const char *(*get_cpuname)(FAR struct rptun_dev_s *dev);
  CODE FAR const char *(*get_firmware)(FAR struct rptun_dev_s *dev);

  CODE FAR const struct rptun_addrenv_s *(*get_addrenv)(FAR struct rptun_dev_s *dev);
  CODE FAR struct rptun_rsc_s *(*get_resource)(FAR struct rptun_dev_s *dev);

  CODE bool (*is_autostart)(FAR struct rptun_dev_s *dev);
  CODE bool (*is_master)(FAR struct rptun_dev_s *dev);

  CODE int (*start)(FAR struct rptun_dev_s *dev);
  CODE int (*stop)(FAR struct rptun_dev_s *dev);
  CODE int (*notify)(FAR struct rptun_dev_s *dev, uint32_t vqid);
  CODE int (*register_callback)(FAR struct rptun_dev_s *dev,
                                rptun_callback_t callback, FAR void *arg);
};

struct rptun_dev_s
{
  FAR const struct rptun_ops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int rptun_initialize(FAR struct rptun_dev_s *dev);
int rptun_boot(FAR const char *cpuname);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RPTUN */
#endif /* __INCLUDE_NUTTX_RPTUN_RPTUN_H */

