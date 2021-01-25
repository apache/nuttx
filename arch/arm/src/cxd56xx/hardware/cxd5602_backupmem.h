/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd5602_backupmem.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD5602_BACKUPMEM_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD5602_BACKUPMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD56_BKUP_SRAM_BASE (0x04400000)
#define BKUP ((backup_info_t*)CXD56_BKUP_SRAM_BASE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct
{
  uint32_t rcosc_clock;                 /* 0x04400000 ~ 0x04400003 */
  uint32_t chip_revision;               /* 0x04400004 ~ 0x04400007 */
  uint32_t sbl_version;                 /* 0x04400008 ~ 0x0440000b */
  uint32_t sysfw_version;               /* 0x0440000c ~ 0x0440000f */
  uint32_t gnssfw_version;              /* 0x04400010 ~ 0x04400013 */
  uint32_t reserved_version[2];         /* 0x04400014 ~ 0x0440001b */
  uint32_t fw_free_space;               /* 0x0440001c ~ 0x0440001f */
  uint32_t bootcause;                   /* 0x04400020 ~ 0x04400023 */
  uint32_t bootmask;                    /* 0x04400024 ~ 0x04400027 */
  uint32_t bootreserve;                 /* 0x04400028 ~ 0x0440002b */
  uint32_t systemconfig;                /* 0x0440002c ~ 0x0440002f */
  uint8_t  rtc_saved_data[32];          /* 0x04400030 ~ 0x0440004f */
  uint32_t irq_wake_map[4];             /* 0x04400050 ~ 0x0440005f */
  uint32_t irq_inv_map[4];              /* 0x04400060 ~ 0x0440006f */
  uint8_t  reserved0[0x100 - 0x70];     /* 0x04400070 ~ 0x044000ff */
  uint8_t  power_monitor_data[0x420];   /* 0x04400100 ~ 0x0440051f */
  uint8_t  reserved1[2 * 1024 - 0x520]; /* 0x04400520 ~ 0x044007ff (2KB-0x520) */
  uint8_t  gnss_backup_data[24 * 1024]; /* 0x04400800 ~ 0x044067ff (24KB) */
  uint8_t  gnss_pvtlog_data[4 * 1024];  /* 0x04406800 ~ 0x044077ff (4KB) */
  uint8_t  reserved_romcode[2 * 1024];  /* 0x04407800 ~ 0x04407fff (2KB) */
  uint8_t  log[32 * 1024];              /* 0x04408000 ~ 0x0440ffff (32KB) */
} backup_info_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD5602_BACKUPMEM_H */
