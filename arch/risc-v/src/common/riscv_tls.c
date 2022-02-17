/****************************************************************************
 * arch/risc-v/src/common/riscv_tls.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/tls.h>

#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_tls_size
 *
 * Description:
 *   Get TLS (sizeof(struct tls_info_s) + tdata + tbss) section size.
 *
 * Returned Value:
 *   Size of (sizeof(struct tls_info_s) + tdata + tbss).
 *
 ****************************************************************************/

int up_tls_size(void)
{
  return sizeof(struct tls_info_s) +
         sizeof(uint32_t) * (_END_TBSS - _START_TDATA);
}

/****************************************************************************
 * Name: up_tls_initialize
 *
 * Description:
 *   Initialize thread local region.
 *
 * Input Parameters:
 *   info - The TLS structure to initialize.
 *
 ****************************************************************************/

void up_tls_initialize(struct tls_info_s *info)
{
  uint8_t *tls_data = (uint8_t *)(info + 1);

  uint32_t tdata_len = sizeof(uint32_t) * (_END_TDATA - _START_TDATA);
  uint32_t tbss_len = sizeof(uint32_t) * (_END_TBSS - _START_TBSS);

  memcpy(tls_data, _START_TDATA, tdata_len);
  memset(tls_data + tdata_len, 0, tbss_len);
}
