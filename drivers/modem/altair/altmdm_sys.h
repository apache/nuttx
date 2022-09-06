/****************************************************************************
 * drivers/modem/altair/altmdm_sys.h
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

#ifndef __DRIVERS_MODEM_ALTAIR_ALTMDM_SYS_H
#define __DRIVERS_MODEM_ALTAIR_ALTMDM_SYS_H

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <time.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTMDM_SYS_FLAG_WMODEOR  0
#define ALTMDM_SYS_FLAG_WMODEAND 1
#define ALTMDM_SYS_FLAG_TMOFEVR  0

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct altmdm_sys_lock_s
{
  mutex_t lock;
};

struct altmdm_sys_csem_s
{
  sem_t sem;
};

struct altmdm_sys_flag_s
{
  sem_t sem;
  uint32_t flag;
};

struct altmdm_sys_flagstate_s
{
  uint32_t flag_pattern;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_sys_initlock
 *
 * Description:
 *   Initialize lock resource.
 *
 ****************************************************************************/

int altmdm_sys_initlock(FAR struct altmdm_sys_lock_s *handle);

/****************************************************************************
 * Name: altmdm_sys_deletelock
 *
 * Description:
 *   Delete lock resource
 *
 ****************************************************************************/

int altmdm_sys_deletelock(FAR struct altmdm_sys_lock_s *handle);

/****************************************************************************
 * Name: altmdm_sys_lock
 *
 * Description:
 *   Acquire lock.
 *
 ****************************************************************************/

int altmdm_sys_lock(FAR struct altmdm_sys_lock_s *handle);

/****************************************************************************
 * Name: altmdm_sys_unlock
 *
 * Description:
 *   Release lock.
 *
 ****************************************************************************/

int altmdm_sys_unlock(FAR struct altmdm_sys_lock_s *handle);

/****************************************************************************
 * Name: altmdm_sys_initcsem
 *
 * Description:
 *   Initialize counting semaphore.
 *
 ****************************************************************************/

int altmdm_sys_initcsem(FAR struct altmdm_sys_csem_s *handle);

/****************************************************************************
 * Name: altmdm_sys_deletecsem
 *
 * Description:
 *   Delete counting semaphore.
 *
 ****************************************************************************/

int altmdm_sys_deletecsem(FAR struct altmdm_sys_csem_s *handle);

/****************************************************************************
 * Name: altmdm_sys_waitcsem
 *
 * Description:
 *   Wait counting semaphore.
 *
 ****************************************************************************/

int altmdm_sys_waitcsem(FAR struct altmdm_sys_csem_s *handle);

/****************************************************************************
 * Name: altmdm_sys_postcsem
 *
 * Description:
 *   Post counting semaphore.
 *
 ****************************************************************************/

int altmdm_sys_postcsem(FAR struct altmdm_sys_csem_s *handle);

/****************************************************************************
 * Name: altmdm_sys_getcsemvalue
 *
 * Description:
 *   Get value of counting semaphore.
 *
 ****************************************************************************/

int altmdm_sys_getcsemvalue(FAR struct altmdm_sys_csem_s *handle,
                            FAR int *value);

/****************************************************************************
 * Name: altmdm_sys_initflag
 *
 * Description:
 *   Initialize event flag resource.
 *
 ****************************************************************************/

int altmdm_sys_initflag(FAR struct altmdm_sys_flag_s *handle);

/****************************************************************************
 * Name: altmdm_sys_deleteflag
 *
 * Description:
 *   Delete event flag resource.
 *
 ****************************************************************************/

int altmdm_sys_deleteflag(FAR struct altmdm_sys_flag_s *handle);

/****************************************************************************
 * Name: altmdm_sys_waitflag
 *
 * Description:
 *   Wait event flag.
 *
 ****************************************************************************/

int altmdm_sys_waitflag(FAR struct altmdm_sys_flag_s *handle,
                        uint32_t wait_pattern, uint32_t wait_mode,
                        FAR uint32_t * pattern, uint32_t timeout_ms);

/****************************************************************************
 * Name: altmdm_sys_setflag
 *
 * Description:
 *   Set event flag.
 *
 ****************************************************************************/

int altmdm_sys_setflag(FAR struct altmdm_sys_flag_s *handle,
                       uint32_t pattern);

/****************************************************************************
 * Name: altmdm_sys_clearflag
 *
 * Description:
 *   Clear event flag.
 *
 ****************************************************************************/

int altmdm_sys_clearflag(FAR struct altmdm_sys_flag_s *handle,
                         uint32_t pattern);

/****************************************************************************
 * Name: altmdm_sys_referflag
 *
 * Description:
 *   Refer event flag.
 *
 ****************************************************************************/

int altmdm_sys_referflag(FAR struct altmdm_sys_flag_s *handle,
                         FAR struct altmdm_sys_flagstate_s *status);

/****************************************************************************
 * Name: altmdm_sys_starttimer
 *
 * Description:
 *   Start timer.
 *
 ****************************************************************************/

timer_t altmdm_sys_starttimer(int first_ms, int interval_ms,
                              FAR void *handler, int int_param,
                              FAR void *ptr_param);

/****************************************************************************
 * Name: altmdm_sys_restarttimer
 *
 * Description:
 *   Restart timer.
 *
 ****************************************************************************/

int altmdm_sys_restarttimer(timer_t timerid, int first_ms, int interval_ms);

/****************************************************************************
 * Name: altmdm_sys_stoptimer
 *
 * Description:
 *   Stop timer.
 *
 ****************************************************************************/

void altmdm_sys_stoptimer(timer_t timerid);

#endif
#endif /* __DRIVERS_MODEM_ALTAIR_ALTMDM_SYS_H */
