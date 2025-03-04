/****************************************************************************
 * include/nuttx/coresight/coresight_etm.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_ETM_H
#define __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_ETM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/coresight/coresight.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETM_MAX_ADDR_CMP         16
#define ETM_MAX_CNTR             4
#define ETM_MAX_CTXID_CMP        3

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct etm_config_s
{
  uint32_t ctrl;                            /* ETMCR */
  uint32_t trigger_event;                   /* ETMTRIGGER */
  uint32_t startstop_ctrl;                  /* ETMTSSCR */
  uint32_t enable_event;                    /* ETMTEEVR */
  uint32_t enable_ctrl1;                    /* ETMTECR1 */
  uint32_t enable_ctrl2;                    /* ETMTECR2 */
  uint32_t fifofull_level;                  /* ETMFFLR */
  uint8_t  addr_idx;                        /* Index for the address comparator selection. */
  uint32_t addr_val[ETM_MAX_ADDR_CMP];      /* Value for address comparator register. */
  uint32_t addr_acctype[ETM_MAX_ADDR_CMP];  /* Access type for address comparator register. */
  uint32_t addr_type[ETM_MAX_ADDR_CMP];     /* Current status of the comparator register. */
  uint8_t  cntr_idx;                        /* Index for the counter register selection */
  uint32_t cntr_rld_val[ETM_MAX_CNTR];      /* Reload value of a counter register. */
  uint32_t cntr_event[ETM_MAX_CNTR];        /* Control for counter enable register. */
  uint32_t cntr_rld_event[ETM_MAX_CNTR];    /* Value for counter reload event register. */
  uint32_t cntr_val[ETM_MAX_CNTR];          /* Counter value register. */
  uint32_t seq_12_event;                    /* Event causing the transition from 1 to 2 */
  uint32_t seq_21_event;                    /* Event causing the transition from 2 to 1 */
  uint32_t seq_23_event;                    /* Event causing the transition from 2 to 3 */
  uint32_t seq_31_event;                    /* Event causing the transition from 3 to 1 */
  uint32_t seq_32_event;                    /* Event causing the transition from 3 to 2 */
  uint32_t seq_13_event;                    /* Event causing the transition from 1 to 3 */
  uint32_t seq_curr_state;                  /* Current value of the sequencer register. */
  uint8_t  ctxid_idx;                       /* Index for the context ID registers. */
  uint32_t ctxid_pid[ETM_MAX_CTXID_CMP];    /* Value for the context ID to trigger on */
  uint32_t ctxid_mask;                      /* Mask applicable to all the context IDs. */
  uint32_t sync_freq;                       /* Synchronisation frequency. */
  uint32_t timestamp_event;                 /* ETMTSEVR */
};

struct coresight_etm_dev_s
{
  struct coresight_dev_s csdev;
  struct etm_config_s cfg;
  int cpu;                       /* The cpu this component is affined to */
  int port_size;                 /* Out port size */
  int traceid;                   /* Trace id */
  uint8_t arch;                  /* ETM/PTM version number */
  uint8_t nr_addr_cmp;           /* Number of pairs of address comparators */
  uint8_t nr_cntr;               /* Number of counters */
  uint8_t nr_ext_inp;            /* Number of external input */
  uint8_t nr_ext_out;            /* Number of external output */
  uint8_t nr_ctxid_cmp;          /* Number of contextID comparators */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: etm_register
 *
 * Description:
 *   Register an ETM/PTM devices.
 *
 * Input Parameters:
 *   desc  - A description of this coresight device.
 *
 * Returned Value:
 *   Pointer to an ETM device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_etm_dev_s *
etm_register(FAR const struct coresight_desc_s *desc);

/****************************************************************************
 * Name: etm_unregister
 *
 * Description:
 *   Unregister an EMT/PTM device.
 *
 ****************************************************************************/

void etm_unregister(FAR struct coresight_etm_dev_s *etmdev);

/****************************************************************************
 * Name: etm_config
 *
 * Description:
 *   Configure the etm device.
 *
 * Input Parameters:
 *   etmdev  - Pointer to the ETM device to config.
 *   config  - Configuration need to be set to ETM device.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int etm_config(FAR struct coresight_etm_dev_s *etmdev,
               FAR const struct etm_config_s *config);

#endif  //__INCLUDE_NUTTX_CORESIGHT_CORESIGHT_ETM_H
