/****************************************************************************
 * include/nuttx/coresight/coresight_etm4.h
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

#ifndef __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_ETM4_H
#define __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_ETM4_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/coresight/coresight.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ETM4 resources */

#define ETM4_MAX_CNTR               4
#define ETM4_MAX_SEQ_STATES         4
#define ETM4_MAX_SINGLE_ADDR_CMP    16
#define ETM4_MAX_CTXID_CMP          8
#define ETM4_MAX_VMID_CMP           8
#define ETM4_MAX_RES_SEL            32
#define ETM4_MAX_SS_CMP             8
#define ETM4_CYC_THRESHOLD_DEFAULT  0x100

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct etm4_config_s
{
  uint32_t mode;                               /* Controls various modes supported by this ETM */
  uint32_t pe_sel;                             /* Controls which PE to trace */
  uint32_t cfg;                                /* Controls the tracing options */
  uint32_t eventctrl0;                         /* Controls the tracing of arbitrary events */
  uint32_t eventctrl1;                         /* Controls the behavior of the events that @event_ctrl0 selects */
  uint32_t stall_ctrl;                         /* Enables trace unit functionality that prevents trace unit buffer overflows */
  uint32_t ts_ctrl;                            /* Controls the insertion of global timestamps in the trace streams */
  uint32_t syncfreq;                           /* Controls how often trace synchronization requests occur. the TRCCCCTLR register */
  uint32_t ccctlr;                             /* Sets the threshold value for cycle counting */
  uint32_t bb_ctrl;                            /* Controls which regions in the memory map are enabled to use branch broadcasting */
  uint32_t vinst_ctrl;                         /* Controls instruction trace filtering */
  uint32_t viiectlr;                           /* Set or read, the address range comparators */
  uint32_t vissctlr;                           /* Set, or read, the single address comparators that control the ViewInst start-stop logic */
  uint32_t vipcssctlr;                         /* Set, or read, which PE comparator inputs can control the ViewInst start-stop logic */
  uint32_t seq_ctrl[ETM4_MAX_SEQ_STATES];      /* Control for the sequencer state transition control register */
  uint32_t seq_rst;                            /* Moves the sequencer to state 0 when a programmed event occurs */
  uint32_t seq_state;                          /* Set, or read the sequencer state */
  uint32_t cntrldvr[ETM4_MAX_CNTR];            /* Sets or returns the reload count value for a counter */
  uint32_t cntr_ctrl[ETM4_MAX_CNTR];           /* Controls the operation of a counter */
  uint32_t cntr_val[ETM4_MAX_CNTR];            /* Sets or returns the value for a counter */
  uint32_t res_ctrl[ETM4_MAX_RES_SEL];         /* Controls the selection of the resources in the trace unit */
  uint32_t ss_ctrl[ETM4_MAX_SS_CMP];           /* Controls the corresponding single-shot comparator resource */
  uint32_t ss_status[ETM4_MAX_SS_CMP];         /* The status of the corresponding single-shot comparator */
  uint32_t ss_pe_cmp[ETM4_MAX_SS_CMP];         /* Selects the PE comparator inputs for Single-shot control */
  uint64_t addr_val[ETM4_MAX_SINGLE_ADDR_CMP]; /* Value for address comparator */
  uint64_t addr_acc[ETM4_MAX_SINGLE_ADDR_CMP]; /* Address comparator access type */
  uint64_t ctxid_pid[ETM4_MAX_CTXID_CMP];      /* Value of the context ID comparator */
  uint32_t ctxid_mask0;                        /* Context ID comparator mask for comparator 0-3 */
  uint32_t ctxid_mask1;                        /* Context ID comparator mask for comparator 4-7 */
  uint64_t vmid_val[ETM4_MAX_VMID_CMP];        /* Value of the VM ID comparator */
  uint32_t vmid_mask0;                         /* VM ID comparator mask for comparator 0-3 */
  uint32_t vmid_mask1;                         /* VM ID comparator mask for comparator 4-7 */
  uint32_t ext_inp;                            /* External input selection */
  uint8_t  s_ex_level;                         /* Secure ELs where tracing is supported */
};

struct coresight_etm4_dev_s
{
  struct coresight_dev_s csdev;
  struct etm4_config_s cfg;
  int cpu;                     /* The cpu this component is affined to */
  uint8_t arch;                /* ETM architecture version */
  uint8_t nr_pe;               /* The number of processing entity available for tracing */
  uint8_t nr_pe_cmp;           /* he number of processing entity comparator inputs that are available for tracing */
  uint8_t nr_addr_cmp;         /* Number of pairs of address comparators available as found in ETMIDR4 0-3 */
  uint8_t nr_cntr;             /* Number of counters as found in ETMIDR5 bit 28-30 */
  uint8_t nr_ext_inp;          /* Number of external input */
  uint8_t numcidc;             /* Number of contextID comparators */
  uint8_t numvmidc;            /* Number of VMID comparators */
  uint8_t nrseqstate;          /* The number of sequencer states that are implemented */
  uint8_t nr_event;            /* Indicates how many events the trace unit support */
  uint8_t nr_resource;         /* The number of resource selection pairs available for tracing */
  uint8_t nr_ss_cmp;           /* Number of single-shot comparator controls that are available */
  uint8_t trcid;               /* value of the current ID for this component */
  uint8_t trcid_size;          /* Indicates the trace ID width */
  uint8_t ts_size;             /* Global timestamp size field */
  uint8_t ctxid_size;          /* Size of the context ID field to consider */
  uint8_t vmid_size;           /* Size of the VM ID comparator to consider */
  uint8_t ccsize;              /* Indicates the size of the cycle counter in bits */
  uint8_t s_ex_level;          /* In secure state, indicates whether instruction tracing is supported for the corresponding Exception level */
  uint8_t ns_ex_level;         /* In non-secure state, indicates whether instruction tracing is supported for the corresponding Exception level */
  uint8_t q_support;           /* Q element support characteristics */
  uint16_t ccitmin;            /* minimum value that can be programmed in */
  uint64_t trfcr;              /* If the CPU supports FEAT_TRF, set TRFCR_ELx to enable tracing at all levels; otherwise, set it to 0 */
  bool os_unlock;              /* True if access to management registers is allowed */
  bool instrp0;                /* Tracing of load and store instructions as P0 elements is supported */
  bool trcbb;                  /* Indicates if the trace unit supports branch broadcast tracing */
  bool trccond;                /* If the trace unit supports conditional instruction tracing */
  bool retstack;               /* Indicates if the implementation supports a return stack */
  bool trccci;                 /* Indicates if the trace unit supports cycle counting for instruction */
  bool trc_error;              /* Whether a trace unit can trace a system error exception */
  bool syncpr;                 /* Indicates if an implementation has a fixed synchronization period */
  bool stallctl;               /* If functionality that prevents trace unit buffer overflows is available */
  bool sysstall;               /* Does the system support stall control of the PE? */
  bool nooverflow;             /* Indicate if overflow prevention is supported */
  bool atbtrig;                /* If the implementation can support ATB triggers */
  bool lpoverride;             /* If the implementation can support low-power state over */
  bool skip_power_up;          /* Indicates if an implementation can skip powering up the trace unit */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: etm4_register
 *
 * Description:
 *   Register an ETMv4 device with the system.
 *
 * Input Parameters:
 *   desc - Pointer to the description of the coresight device.
 *
 * Returned Value:
 *   Pointer to the ETMv4 device structure on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_etm4_dev_s *
etm4_register(FAR const struct coresight_desc_s *desc);

/****************************************************************************
 * Name: etm4_unregister
 *
 * Description:
 *   Unregister an ETMv4 device from the system.
 *
 * Input Parameters:
 *   etmdev - Pointer to the ETMv4 device structure.
 *
 ****************************************************************************/

void etm4_unregister(FAR struct coresight_etm4_dev_s *etmdev);

/****************************************************************************
 * Name: etm4_config
 *
 * Description:
 *   Configure the ETMv4 device based on the provided configuration.
 *
 * Input Parameters:
 *   etmdev - Pointer to the ETMv4 device structure.
 *   config - Pointer to the ETMv4 configuration structure.
 *
 ****************************************************************************/

int etm4_config(FAR struct coresight_etm4_dev_s *etmdev,
                FAR const struct etm4_config_s *config);

#endif /* __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_ETM4_H */
