/****************************************************************************
 * drivers/modem/alt1250/altcom_cmd_log.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_LOG_H
#define __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_LOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/modem/alt1250.h>
#include <nuttx/wireless/lte/lte.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTCOM_LOGSPATH "logs/"
#define ALTCOM_PATH_LEN_MAX (LTE_LOG_NAME_LEN + 5)
#define ALTCOM_LIST_LEN_MAX (LTE_LOG_LIST_SIZE * ALTCOM_PATH_LEN_MAX)
#define ALTCOM_LOG_ACCESS_PATH_LEN_MAX (256)
#define ALTCOM_LOG_READ_LEN_MAX (2000)
#define ALTCOM_LOG_OPEN_FLAGS (0x1)
#define ALTCOM_LOG_SEEK_SET (0)
#define ALTCOM_LOG_SEEK_CUR (1)
#define ALTCOM_LOG_SEEK_END (2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct apicmd_cmddat_clogs_s
{
  uint8_t pathlen;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_clogsres_s
{
  int32_t altcom_result;
  uint8_t pathlen;
  char path[ALTCOM_PATH_LEN_MAX];
} end_packed_struct;

begin_packed_struct struct apicmddbg_getloglist_s
{
  uint8_t listsize;
  uint8_t pathlen;
} end_packed_struct;

begin_packed_struct struct apicmddbg_getloglistres_s
{
  int32_t altcom_result;
  uint8_t listsize;
  uint8_t pathlen;
  char list[ALTCOM_LIST_LEN_MAX];
} end_packed_struct;

begin_packed_struct struct apicmd_logopen_s
{
  char path[ALTCOM_LOG_ACCESS_PATH_LEN_MAX];
  int32_t flags;
  int32_t mode;
} end_packed_struct;

begin_packed_struct struct apicmd_logcommonres_s
{
  int32_t altcom_result;
  int32_t unused;
} end_packed_struct;

begin_packed_struct struct apicmd_logclose_s
{
  int32_t fd;
} end_packed_struct;

begin_packed_struct struct apicmd_logread_s
{
  int32_t fd;
  int32_t readlen;
} end_packed_struct;

begin_packed_struct struct apicmd_logreadres_s
{
  int32_t altcom_result;
  int32_t unused;
  char readdata[ALTCOM_LOG_READ_LEN_MAX];
} end_packed_struct;

begin_packed_struct struct apicmd_logremove_s
{
  char path[ALTCOM_LOG_ACCESS_PATH_LEN_MAX];
} end_packed_struct;

begin_packed_struct struct apicmd_loglseek_s
{
  int32_t fd;
  int32_t offset;
  int32_t whence;
} end_packed_struct;

#endif  /* __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_LOG_H */
