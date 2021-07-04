/****************************************************************************
 * drivers/1wire/1wire_internal.h
 *
 *   Copyright (C) 2018 Haltian Ltd. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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

#ifndef __DRIVERS_1WIRE_1WIRE_INTERNAL_H
#define __DRIVERS_1WIRE_1WIRE_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <nuttx/semaphore.h>
#include <nuttx/power/pm.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct onewire_dev_s;

struct onewire_sem_s
{
  sem_t sem;
  pid_t holder;                        /* The current holder of the semaphore */
  int16_t count;                       /* Number of counts held */
};

struct onewire_master_s
{
  FAR struct onewire_dev_s *dev;       /* 1-Wire lower half */
  struct onewire_sem_s devsem;         /* Re-entrant semaphore */
  int nslaves;                         /* Number of 1-wire slaves */
  int maxslaves;                       /* Maximum number of 1-wire slaves */
  bool insearch;                       /* If we are in middle of 1-wire search */
#ifdef CONFIG_PM
  struct pm_callback_s pm_cb;          /* PM callbacks */
#endif
};

struct onewire_slave_s
{
  FAR struct onewire_master_s *master; /* Slave's bus master */
  uint64_t romcode;                    /* Unique ROM id in bus */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Additional CRC helpers from 1wire_crc.c */

bool onewire_valid_rom(uint64_t rom);

/* Rest are from 1wire.c */

int  onewire_sem_wait(FAR struct onewire_master_s *master);
void onewire_sem_post(FAR struct onewire_master_s *master);

int onewire_addslave(FAR struct onewire_master_s *master,
                     FAR struct onewire_slave_s *slave);
int onewire_removeslave(FAR struct onewire_master_s *master,
                        FAR struct onewire_slave_s *slave);

/****************************************************************************
 * Name: onewire_reset_resume
 *
 * Description:
 *
 ****************************************************************************/

int onewire_reset_resume(FAR struct onewire_master_s *master);

/****************************************************************************
 * Name: onewire_reset_select
 *
 * Description:
 *
 ****************************************************************************/

int onewire_reset_select(FAR struct onewire_master_s *master,
                         uint64_t romcode);

/****************************************************************************
 * Name: onewire_triplet
 *
 * Description:
 *   Used by 1-wire search algorithm. Reads two bits and writes
 *   one based on comparison of read bits.
 *
 * Input Parameters:
 *   search_bit - Bit to write if both id_bit and cmp_id_bit match
 *
 * Output Parameters:
 *   taken_bit  - Bit indicating the direction where the search is
 *                progressing.
 *
 * Return Value:
 *   Number of valid bits or negative on error.
 *
 ****************************************************************************/

int onewire_triplet(FAR struct onewire_master_s *master,
                    uint8_t search_bit,
                    FAR uint8_t *taken_bit);

/****************************************************************************
 * Name: onewire_search
 *
 * Description:
 *   Search all devices from a 1-wire network. This is the 1-wire search
 *   algorithm from Maxim Application Note 187. Note! This is an atomic
 *   operation. The callback 'cb_search' can't execute any function that will
 *   lock this bus, because of the locked state as long the search is active.
 *
 * Input Parameters:
 *   master    - Pointer to the allocated 1-wire interface
 *   family    - Limit search to devices of matching family
 *   alarmonly - Limit search to devices on alarm state
 *   cb_search - Callback to call on each device found
 *   arg       - Argument passed to cb_search
 *
 * Return Value:
 *   Number of slaves present and matching family.
 *
 ****************************************************************************/

int onewire_search(FAR struct onewire_master_s *master,
                   int family,
                   bool alarmonly,
                   CODE void (*cb_search)(int family,
                                          uint64_t romcode,
                                          FAR void *arg),
                   FAR void *arg);

#endif /* __DRIVERS_1WIRE_1WIRE_INTERNAL_H */
