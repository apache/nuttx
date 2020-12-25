/****************************************************************************
 * include/nuttx/mbox/mbox.h
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
