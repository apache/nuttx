/****************************************************************************
 * include/nuttx/sdio_slave.h
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

#ifndef __INCLUDE_NUTTX_SDIO_SLAVE_H
#define __INCLUDE_NUTTX_SDIO_SLAVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: SDIO_SLAVE_SETCALLBACK
 *
 * Description:
 *   Set callback to hardware, when the buffer have received or send
 *   completed, hardware should call the callback to tell user transfer
 *   finshed.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO Slave device interface
 *   callback - User defined that recived transfer finshed
 *   arg - The context for setcallback
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

#define SDIO_SLAVE_SETCALLBACK(dev, callback, arg) ((dev)->setcallback(dev, callback, arg))

/****************************************************************************
 * Name: SDIO_SLAVE_RECVBUFFER
 *
 * Description:
 *   Receive buffer from hardware without blocking.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO Slave device interface
 *   buf - Address of the transfer buffer.
 *   len - Length of the transfer buffer.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

#define SDIO_SLAVE_RECVBUFFER(dev, buf, len) ((dev)->recvbuffer(dev, buf, len))

/****************************************************************************
 * Name: SDIO_SLAVE_SENDBUFFER
 *
 * Description:
 *   Send buffer to hardware without blocking.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO Slave device interface.
 *   buf - Address of the transfer buffer.
 *   len - Length of the transfer buffer.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

#define SDIO_SLAVE_SENDBUFFER(dev, buf, len) ((dev)->sendbuffer(dev, buf, len))

/****************************************************************************
 * Name: SDIO_SLAVE_CANCELBUFFER
 *
 * Description:
 *   Cancel the buffer from hardware.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO Slave device interface
 *   buf - Address of the transfer buffer.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

#define SDIO_SLAVE_CANCELBUFFER(dev, buf) ((dev)->cancelbuffer(dev, buf))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_slave_finish
 *
 * Description:
 *   Defines the transfer finish callback. User should not block in it,
 *   hardware should call it when the buffer transfer completed. If
 *   you want to use this buffer to transfer again, you need call
 *   recvbuffer/sendbuffer again.
 *
 * Input Parameters:
 *   recv - true on recive finshed; false on send finshed.
 *   buf - Address of the transfer buffer.
 *   result - negated errno on failure; the size received.
 *   arg - the context of setcallback.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

typedef CODE void sdio_slave_finish(bool recv, FAR void *buf,
                                    ssize_t result, FAR void *arg);

struct sdio_slave_s
{
  CODE int (*setcallback)(FAR struct sdio_slave_s *dev,
                          sdio_slave_finish callback, FAR void *arg);
  CODE int (*recvbuffer)(FAR struct sdio_slave_s *dev,
                         FAR void *buf, ssize_t len);
  CODE int (*sendbuffer)(FAR struct sdio_slave_s *dev,
                         FAR const void *buf, ssize_t len);
  CODE int (*cancelbuffer)(FAR struct sdio_slave_s *dev,
                           FAR const void *buf);
};

/****************************************************************************
 * Public Functions Definitions
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

#endif /* __INCLUDE_NUTTX_SDIO_SLAVE_H */
