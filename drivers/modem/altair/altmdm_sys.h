/****************************************************************************
 * drivers/modem/altmdm/altmdm_sys.h
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

#ifndef __DRIVERS_MODEM_ALTMDM_ALTMDM_SYS_H
#define __DRIVERS_MODEM_ALTMDM_ALTMDM_SYS_H

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <time.h>
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
  sem_t sem;
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
#endif /* __DRIVERS_MODEM_ALTMDM_ALTMDM_SYS_H */
