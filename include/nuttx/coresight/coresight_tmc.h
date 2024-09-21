/****************************************************************************
 * include/nuttx/coresight/coresight_tmc.h
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

#ifndef __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_TMC_H
#define __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_TMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/mutex.h>
#include <nuttx/coresight/coresight.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum tmc_config_type_e
{
  TMC_CONFIG_TYPE_ETB,
  TMC_CONFIG_TYPE_ETR,
  TMC_CONFIG_TYPE_ETF,
};

enum tmc_mem_intf_width_e
{
  TMC_MEM_INTF_WIDTH_32BITS = 1,
  TMC_MEM_INTF_WIDTH_64BITS = 2,
  TMC_MEM_INTF_WIDTH_128BITS = 4,
  TMC_MEM_INTF_WIDTH_256BITS = 8,
};

enum tmc_etr_mode_e
{
  TMC_ETR_MODE_FLAT,                    /* Uses contiguous flat buffer. */
  TMC_ETR_MODE_ETR_SG,                  /* Uses in-built TMC ETR SG mechanism. */
  TMC_ETR_MODE_CATU,                    /* Use SG mechanism in CATU. */
};

struct coresight_tmc_dev_s
{
  struct coresight_dev_s csdev;
  enum tmc_config_type_e config_type;   /* Device type: ETB/ETR/ETF. */
  enum tmc_mem_intf_width_e mmwidth;    /* Width of the memory interface databus, in bytes. */
  uint32_t trigger_cntr;                /* Amount of words to store after a trigger. */
  uint32_t size;                        /* RAM buffer size. */
  uint32_t burst_size;                  /* Max burst size used in ETR devices. */
  FAR uint32_t *buf;                    /* Pointer to the RAM buf. */
  uint32_t len;                         /* Valid data len in RAM buffer. */
  mutex_t lock;                         /* Mutex for driver's open/close. */
  uint32_t caps;                        /* Capalilities current etr device has. */
  enum tmc_etr_mode_e mode;             /* ETR buffer mode. */
  uint32_t offset;                      /* Data offset in ETR buffer. */
  uint8_t opencnt;                      /* TMC device's open count. */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tmc_register
 *
 * Description:
 *   Register a TMC devices.
 *
 * Input Parameters:
 *   desc  - A description of this coresight device.
 *
 * Returned Value:
 *   Pointer to a TMC device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_tmc_dev_s *
tmc_register(FAR const struct coresight_desc_s *desc);

/****************************************************************************
 * Name: tmc_unregister
 *
 * Description:
 *   Unregister a TMC devices.
 *
 * Input Parameters:
 *   tmcdev  - Pointer to the TMC device.
 *
 ****************************************************************************/

void tmc_unregister(FAR struct coresight_tmc_dev_s *tmcdev);

#endif  //__INCLUDE_NUTTX_CORESIGHT_CORESIGHT_TMC_H
