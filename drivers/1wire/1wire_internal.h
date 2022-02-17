/****************************************************************************
 * drivers/1wire/1wire_internal.h
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
