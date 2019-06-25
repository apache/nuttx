/****************************************************************************
 * include/arch/chip/battery_ioctl.h
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

#ifndef __INCLUDE_ARCH_CHIP_BATTERY_IOCTL_H
#define __INCLUDE_ARCH_CHIP_BATTERY_IOCTL_H

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ioctl commands */

#define BATIOC_GET_CHGVOLTAGE    _BATIOC(0x0010)
#define BATIOC_GET_CHGCURRENT    _BATIOC(0x0012)
#define BATIOC_GET_RECHARGEVOL   _BATIOC(0x0013)
#define BATIOC_SET_RECHARGEVOL   _BATIOC(0x0014)
#define BATIOC_GET_COMPCURRENT   _BATIOC(0x0015)
#define BATIOC_SET_COMPCURRENT   _BATIOC(0x0016)
#define BATIOC_GET_TEMPTABLE     _BATIOC(0x0017)
#define BATIOC_SET_TEMPTABLE     _BATIOC(0x0018)
#define BATIOC_GET_CURRENT       _BATIOC(0x0019)
#define BATIOC_GET_VOLTAGE       _BATIOC(0x001a)

#define BATIOC_MONITOR_ENABLE    _BATIOC(0x0030)
#define BATIOC_MONITOR_STATUS    _BATIOC(0x0031)
#define BATIOC_MONITOR_SET       _BATIOC(0x0032)
#define BATIOC_MONITOR_GET       _BATIOC(0x0033)

#define BATIOC_DEBUG             _BATIOC(0x00db)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct battery_temp_table_s
{
  int T60; /* 60 degree C */
  int T45; /* 45 degree C */
  int T10; /* 10 degree C */
  int T00; /*  0 degree C */
};

struct bat_monitor_enable_s
{
  int on;
  int interval;
  int threshold_volt;
  int threshold_current;
};

struct bat_monitor_status_s
{
  int run;
  int index;
  int latest;
  int totalwatt;
  int totaltime;
};

struct bat_monitor_set_s
{
  int clearbuf;
  int clearsum;
};

struct bat_monitor_rec_s
{
  uint16_t index;
  uint16_t timestamp;
  uint16_t voltage;
  int16_t  current;
};

struct bat_monitor_log_s
{
  FAR struct bat_monitor_rec_s *rec;
  int index;
  int size;
};

#endif /* __INCLUDE_ARCH_CHIP_BATTERY_IOCTL_H */
