/****************************************************************************
 * include/nuttx/coresight/coresight.h
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

#ifndef __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_H
#define __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <nuttx/clk/clk.h>
#include <nuttx/list.h>
#include <nuttx/power/pm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum coresight_dev_type_e
{
  CORESIGHT_DEV_TYPE_SOURCE,
  CORESIGHT_DEV_TYPE_LINK,
  CORESIGHT_DEV_TYPE_SINK,
  CORESIGHT_DEV_TYPE_MAX
};

enum coresight_dev_subtype_source_e
{
  CORESIGHT_DEV_SUBTYPE_SOURCE_PROC,       /* ETM */
  CORESIGHT_DEV_SUBTYPE_SOURCE_SOFTWARE,   /* STM */
};

enum coresight_dev_subtype_link_e
{
  CORESIGHT_DEV_SUBTYPE_LINK_MERG,         /* Funnel */
  CORESIGHT_DEV_SUBTYPE_LINK_SPLIT,        /* Replocator */
  CORESIGHT_DEV_SUBTYPE_LINK_FIFO,         /* TMC ETF */
};

enum coresight_dev_subtype_sink_e
{
  CORESIGHT_DEV_SUBTYPE_SINK_PORT,         /* TPIU */
  CORESIGHT_DEV_SUBTYPE_SINK_BUFFER,       /* ETB */
  CORESIGHT_DEV_SUBTYPE_SINK_TMC_BUFFER,   /* TMC ETB */
  CORESIGHT_DEV_SUBTYPE_SINK_TMC_SYSMEM,   /* TMC ETR */
  CORESIGHT_DEV_SUBTYPE_SINK_TMC_ETF,      /* TMC ETF */
};

/* This structure is used to unify different subtype of devices. */

union coresight_dev_subtype_u
{
  enum coresight_dev_subtype_source_e source_subtype;
  enum coresight_dev_subtype_link_e link_subtype;
  enum coresight_dev_subtype_sink_e sink_subtype;
};

struct coresight_dev_s;

struct coresight_sink_ops_s
{
  int (*enable)(FAR struct coresight_dev_s *csdev);
  void (*disable)(FAR struct coresight_dev_s *csdev);
};

struct coresight_link_ops_s
{
  int (*enable)(FAR struct coresight_dev_s *csdev, int iport, int oport);
  void (*disable)(FAR struct coresight_dev_s *csdev, int iport, int oport);
};

struct coresight_source_ops_s
{
  int (*enable)(FAR struct coresight_dev_s *csdev);
  void (*disable)(FAR struct coresight_dev_s *csdev);
};

/* This structure is used to unify different operations of devices. */

struct coresight_ops_s
{
  union
    {
      FAR const struct coresight_sink_ops_s *sink_ops;
      FAR const struct coresight_link_ops_s *link_ops;
      FAR const struct coresight_source_ops_s *source_ops;
    };
};

struct coresight_portdesc_s
{
  /* Coresight device's name this port connects to. */

  FAR const char *remote;

  /* Port connects to. */

  int port;
};

struct coresight_desc_s
{
  FAR const char *name;
#ifdef CONFIG_CLK
  FAR const char *clkname;
#endif
  uintptr_t addr;
  enum coresight_dev_type_e type;
  union coresight_dev_subtype_u subtype;

  /* Used in ETM device. */

  uint8_t cpu;

  /* Used in funnel devices. */

  int inport_num;

  /* Used in STM device: start address of extend stimulus port, this memory
   * should be reserved for stm use.
   * Size of this prealloced memory equals to (256 x number of ports).
   */

  uintptr_t stimulus_port_addr;

  /* Used in TMC-ETR device. */

  uint32_t buffer_size;
  uint32_t burst_size;
  uint32_t caps;

  /* Description of outports of current device. */

  int outport_num;
  struct coresight_portdesc_s outports[CONFIG_CORESIGHT_MAX_OUTPORT_NUM];
};

/* Use to build the trace path. */

struct coresight_connect_s
{
  int srcport;
  int destport;

  /* Used to find the dest device when build the trace path. */

  FAR const char *destname;
  FAR struct coresight_dev_s *srcdev;
  FAR struct coresight_dev_s *destdev;
};

struct coresight_dev_s
{
  FAR const char *name;
#ifdef CONFIG_CLK
  FAR struct clk_s *clk;
#endif
#ifdef CONFIG_PM
  struct pm_callback_s pmcb;
#endif

  /* Coresight device's enable count. */

  uint8_t refcnt;

  /* Memory-mapped base address of current coresight device. */

  uintptr_t addr;
  enum coresight_dev_type_e type;
  union coresight_dev_subtype_u subtype;
  FAR const struct coresight_ops_s *ops;

  /* Used to connect all the coresight device register to coresight bus. */

  struct list_node node;

  /* Used in source coresight device as trace path's list head. */

  struct list_node path;

  /* Out port number current coresight device have. */

  int outport_num;

  /* Pointer to an array of connections, array size is equal
   * to the outport number.
   */

  FAR struct coresight_connect_s *outconns;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: coresight_register
 *
 * Description:
 *   Register a coresight device to the coresight bus.
 *
 * Input Parameters:
 *   csdev  - Pointer to the coresight device that needs to be registered.
 *   desc   - Pointer to the attribute description of this coresight device.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_register(FAR struct coresight_dev_s *csdev,
                       FAR const struct coresight_desc_s *desc);

/****************************************************************************
 * Name: coresight_unregister
 *
 * Description:
 *   Unregister a coresight device from coresight bus.
 *
 * Input Parameters:
 *   csdev  - Pointer to the coresight device that needs to be unregistered.
 *
 ****************************************************************************/

void coresight_unregister(FAR struct coresight_dev_s *csdev);

/****************************************************************************
 * Name: coresight_enable
 *
 * Description:
 *   Enable trace start from srcdev to destdev.
 *
 * Input Parameters:
 *   srcdev  - Source device that generates trace data.
 *   destdev - Sink device that finally accepts the trace data.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_enable(FAR struct coresight_dev_s *srcdev,
                     FAR struct coresight_dev_s *destdev);

/****************************************************************************
 * Name: coresight_disable
 *
 * Description:
 *   Disable the trace start from srcdev to destdev.
 *
 * Input Parameters:
 *   srcdev  - Source device that generates trace data.
 *
 ****************************************************************************/

void coresight_disable(FAR struct coresight_dev_s *srcdev);

#endif  /* __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_H */
