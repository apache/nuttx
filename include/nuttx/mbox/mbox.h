/****************************************************************************
 * include/nuttx/mbox/mbox.h
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

#ifndef __INCLUDE_NUTTX_MBOX_MBOX_H
#define __INCLUDE_NUTTX_MBOX_MBOX_H

#ifdef CONFIG_MBOX

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Access macros ************************************************************/

/****************************************************************************
 * Name: MBOX_SEND
 *
 * Description:
 *   Send a 32bits message to remote core from specific channel
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   ch    - Mbox specific channel
 *   msg   - Message to send
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define MBOX_SEND(d,c,m) ((d)->ops->send(d,c,m))

/****************************************************************************
 * Name: MBOX_REGISTER_CALLBACK
 *
 * Description:
 *   Attach to receive a callback when something is received on MBOX
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   ch       - Mbox specific channel
 *   callback - The function to be called when something has been received
 *   arg      - A caller provided value to return with the callback
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define MBOX_REGISTER_CALLBACK(d,c,cb,a) \
  ((d)->ops->registercallback(d,c,cb,a))

/****************************************************************************
 * Name: MBOX_UNREGISTER_CALLBACK
 *
 * Description:
 *   Detach MBOX callback
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   ch       - Mbox specific channel
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define MBOX_UNREGISTER_CALLBACK(d,c) \
  ((d)->ops->registercallback(d,c,NULL,NULL))

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mbox_dev_s;
typedef CODE int (*mbox_receive_t)(FAR void *arg, uintptr_t msg);

struct mbox_ops_s
{
  CODE int (*send)(FAR struct mbox_dev_s *dev, uint32_t ch, uintptr_t msg);
  CODE int (*registercallback)(FAR struct mbox_dev_s *dev, uint32_t ch,
                               mbox_receive_t callback, FAR void *arg);
};

struct mbox_dev_s
{
  FAR const struct mbox_ops_s *ops;
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

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MBOX */
#endif /* __INCLUDE_NUTTX_MBOX_MBOX_H */
