/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd5602_backupmem.h
 *
 *   Copyright (C) 2008-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
