/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gnss.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fixedmath.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>

#include <arch/chip/gnss.h>
#include <arch/chip/pm.h>
#include <arch/board/board.h>

#include "cxd56_gnss_api.h"
#include "cxd56_cpu1signal.h"
#include "cxd56_gnss.h"
#include "cxd56_pinconfig.h"

#if defined(CONFIG_CXD56_GNSS)

/****************************************************************************
 * External Defined Functions
 ****************************************************************************/

extern int fw_pm_loadimage(int cpuid, const char *filename);
extern int fw_pm_startcpu(int cpuid, int wait);
extern int fw_pm_sleepcpu(int cpuid, int mode);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CXD56_GNSS_NPOLLWAITERS
#  define CONFIG_CXD56_GNSS_NPOLLWAITERS      4
#endif

#ifndef CONFIG_CXD56_GNSS_NSIGNALRECEIVERS
#  define CONFIG_CXD56_GNSS_NSIGNALRECEIVERS  4
#endif

#ifndef CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE
#  define CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE 1024
#endif

#ifndef CONFIG_CXD56_GNSS_BACKUP_FILENAME
#  define CONFIG_CXD56_GNSS_BACKUP_FILENAME   "/mnt/spif/gnss_backup.bin"
#endif

#ifndef CONFIG_CXD56_GNSS_CEP_FILENAME
#  define CONFIG_CXD56_GNSS_CEP_FILENAME      "/mnt/spif/gnss_cep.bin"
#endif

#define CXD56_GNSS_GPS_CPUID                  1
#ifdef CONFIG_CXD56_GNSS_FW_RTK
#  define CXD56_GNSS_FWNAME                   "gnssfwrtk"
#else
#  define CXD56_GNSS_FWNAME                   "gnssfw"
#endif
#ifndef PM_SLEEP_MODE_COLD
#  define PM_SLEEP_MODE_COLD                  2
#endif
#ifndef PM_SLEEP_MODE_HOT_ENABLE
#  define PM_SLEEP_MODE_HOT_ENABLE            7
#endif
#ifndef PM_SLEEP_MODE_HOT_DISABLE
#  define PM_SLEEP_MODE_HOT_DISABLE           8
#endif

/* Notify data of PUBLISH_TYPE_GNSS */

#define CXD56_GNSS_NOTIFY_TYPE_POSITION       0
#define CXD56_GNSS_NOTIFY_TYPE_BOOTCOMP       1
#define CXD56_GNSS_NOTIFY_TYPE_REQBKUPDAT     2
#define CXD56_GNSS_NOTIFY_TYPE_REQCEPOPEN     3
#define CXD56_GNSS_NOTIFY_TYPE_REQCEPCLOSE    4
#define CXD56_GNSS_NOTIFY_TYPE_REQCEPDAT      5
#define CXD56_GNSS_NOTIFY_TYPE_REQCEPBUFFREE  6

/* GNSS core CPU FIFO interface API */

#define CXD56_GNSS_GD_GNSS_START              0
#define CXD56_GNSS_GD_GNSS_STOP               1
#define CXD56_GNSS_GD_GNSS_CEPINITASSISTDATA  2

/* CPU FIFO API bitfield converter */

#define CXD56_GNSS_CPUFIFOAPI_SET_DATA(API, DATA) (((DATA) << 8) | (API))

/* Common info shared with GNSS core */

#define GNSS_SHARED_INFO_MAX_ARGC             6

/* GDSP File read/write arguments */

#define GNSS_ARGS_FILE_OFFSET                 0
#define GNSS_ARGS_FILE_BUF                    1
#define GNSS_ARGS_FILE_LENGTH                 2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cxd56_gnss_sig_s
{
  uint8_t                         enable;
  pid_t                           pid;
  struct cxd56_gnss_signal_info_s info;
};

struct cxd56_gnss_shared_info_s
{
  int      retval;
  uint32_t argc;
  uint32_t argv[GNSS_SHARED_INFO_MAX_ARGC];
};

struct cxd56_devsig_table_s
{
  uint8_t                sigtype;
  cxd56_cpu1sighandler_t handler;
};

struct cxd56_gnss_dev_s
{
  sem_t                           devsem;
  sem_t                           syncsem;
  uint8_t                         num_open;
  uint8_t                         notify_data;
  struct file                     cepfp;
  void *                          cepbuf;
  struct pollfd                  *fds[CONFIG_CXD56_GNSS_NPOLLWAITERS];
#if CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0
  struct cxd56_gnss_sig_s         sigs[CONFIG_CXD56_GNSS_NSIGNALRECEIVERS];
#endif
  struct cxd56_gnss_shared_info_s shared_info;
  sem_t                           ioctllock;
  sem_t                           apiwait;
  int                             apiret;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ioctl command functions */

static int cxd56_gnss_start(struct file *filep, unsigned long arg);
static int cxd56_gnss_stop(struct file *filep, unsigned long arg);
static int cxd56_gnss_select_satellite_system(struct file *filep,
                                              unsigned long arg);
static int cxd56_gnss_get_satellite_system(struct file *filep,
                                           unsigned long arg);
static int
cxd56_gnss_set_receiver_position_ellipsoidal(struct file *filep,
                                             unsigned long arg);
static int cxd56_gnss_set_receiver_position_orthogonal(
                                   struct file *filep,
                                   unsigned long arg);
static int cxd56_gnss_set_ope_mode(struct file *filep,
                                   unsigned long arg);
static int cxd56_gnss_get_ope_mode(struct file *filep,
                                   unsigned long arg);
static int cxd56_gnss_set_tcxo_offset(struct file *filep,
                                      unsigned long arg);
static int cxd56_gnss_get_tcxo_offset(struct file *filep,
                                      unsigned long arg);
static int cxd56_gnss_set_time(struct file *filep,
                               unsigned long arg);
static int cxd56_gnss_get_almanac(struct file *filep,
                                  unsigned long arg);
static int cxd56_gnss_set_almanac(struct file *filep,
                                  unsigned long arg);
static int cxd56_gnss_get_ephemeris(struct file *filep,
                                    unsigned long arg);
static int cxd56_gnss_set_ephemeris(struct file *filep,
                                    unsigned long arg);
static int cxd56_gnss_save_backup_data(struct file *filep,
                                       unsigned long arg);
static int cxd56_gnss_erase_backup_data(struct file *filep,
                                        unsigned long arg);
static int cxd56_gnss_open_cep_data(struct file *filep,
                                    unsigned long arg);
static int cxd56_gnss_close_cep_data(struct file *filep,
                                     unsigned long arg);
static int cxd56_gnss_check_cep_data(struct file *filep,
                                     unsigned long arg);
static int cxd56_gnss_get_cep_age(struct file *filep,
                                  unsigned long arg);
static int cxd56_gnss_reset_cep_flag(struct file *filep,
                                     unsigned long arg);
static int cxd56_gnss_set_acquist_data(struct file *filep,
                                       unsigned long arg);
static int cxd56_gnss_set_frametime(struct file *filep,
                                    unsigned long arg);
static int cxd56_gnss_set_tau_gps(struct file *filep,
                                  unsigned long arg);
static int cxd56_gnss_set_time_gps(struct file *filep,
                                   unsigned long arg);
static int cxd56_gnss_clear_receiver_info(struct file *filep,
                                          unsigned long arg);
static int cxd56_gnss_set_tow_assist(struct file *filep,
                                     unsigned long arg);
static int cxd56_gnss_set_utc_model(struct file *filep,
                                    unsigned long arg);
static int cxd56_gnss_control_spectrum(struct file *filep,
                                       unsigned long arg);
static int cxd56_gnss_start_test(struct file *filep,
                                 unsigned long arg);
static int cxd56_gnss_stop_test(struct file *filep,
                                unsigned long arg);
static int cxd56_gnss_get_test_result(struct file *filep,
                                      unsigned long arg);
static int cxd56_gnss_set_signal(struct file *filep,
                                 unsigned long arg);
static int cxd56_gnss_start_pvtlog(struct file *filep,
                                   unsigned long arg);
static int cxd56_gnss_stop_pvtlog(struct file *filep,
                                  unsigned long arg);
static int cxd56_gnss_delete_pvtlog(struct file *filep,
                                    unsigned long arg);
static int cxd56_gnss_get_pvtlog_status(struct file *filep,
                                        unsigned long arg);
static int cxd56_gnss_start_rtk_output(struct file *filep,
                                       unsigned long arg);
static int cxd56_gnss_stop_rtk_output(struct file *filep,
                                      unsigned long arg);
static int cxd56_gnss_set_rtk_interval(struct file *filep,
                                       unsigned long arg);
static int cxd56_gnss_get_rtk_interval(struct file *filep,
                                       unsigned long arg);
static int cxd56_gnss_select_rtk_satellite(struct file *filep,
                                           unsigned long arg);
static int cxd56_gnss_get_rtk_satellite(struct file *filep,
                                        unsigned long arg);
static int cxd56_gnss_set_rtk_ephemeris_enable(struct file *filep,
                                               unsigned long arg);
static int cxd56_gnss_get_rtk_ephemeris_enable(struct file *filep,
                                               unsigned long arg);
static int cxd56_gnss_start_navmsg_output(struct file *filep,
                                          unsigned long arg);
static int cxd56_gnss_get_var_ephemeris(struct file *filep,
                                        unsigned long arg);
static int cxd56_gnss_set_var_ephemeris(struct file *filep,
                                        unsigned long arg);
static int cxd56_gnss_set_usecase(struct file *filep,
                                  unsigned long arg);
static int cxd56_gnss_get_usecase(struct file *filep,
                                  unsigned long arg);
static int cxd56_gnss_set_1pps_output(struct file *filep,
                                      unsigned long arg);
static int cxd56_gnss_get_1pps_output(struct file *filep,
                                      unsigned long arg);

/* file operation functions */

static int cxd56_gnss_open(struct file *filep);
static int cxd56_gnss_close(struct file *filep);
static ssize_t cxd56_gnss_read(struct file *filep, char *buffer,
                               size_t len);
static ssize_t cxd56_gnss_write(struct file *filep,
                                const char *buffer, size_t buflen);
static int cxd56_gnss_ioctl(struct file *filep, int cmd,
                            unsigned long arg);
static int cxd56_gnss_poll(struct file *filep, struct pollfd *fds,
                           bool setup);
static int8_t cxd56_gnss_select_notifytype(off_t fpos, uint32_t *offset);

static int cxd56_gnss_cpufifo_api(struct file *filep,
                                  unsigned int api,
                                  unsigned int data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_gnssfops =
{
  cxd56_gnss_open,  /* open */
  cxd56_gnss_close, /* close */
  cxd56_gnss_read,  /* read */
  cxd56_gnss_write, /* write */
  NULL,             /* seek */
  cxd56_gnss_ioctl, /* ioctl */
  cxd56_gnss_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL            /* unlink */
#endif
};

/* GNSS ioctl command list */

static int (*g_cmdlist[CXD56_GNSS_IOCTL_MAX])(struct file *filep,
                                              unsigned long arg) =
{
  NULL,                    /* CXD56_GNSS_IOCTL_INVAL = 0 */
  cxd56_gnss_start,
  cxd56_gnss_stop,
  cxd56_gnss_select_satellite_system,
  cxd56_gnss_get_satellite_system,
  cxd56_gnss_set_receiver_position_ellipsoidal,
  cxd56_gnss_set_receiver_position_orthogonal,
  cxd56_gnss_set_ope_mode,
  cxd56_gnss_get_ope_mode,
  cxd56_gnss_set_tcxo_offset,
  cxd56_gnss_get_tcxo_offset,
  cxd56_gnss_set_time,
  cxd56_gnss_get_almanac,
  cxd56_gnss_set_almanac,
  cxd56_gnss_get_ephemeris,
  cxd56_gnss_set_ephemeris,
  cxd56_gnss_save_backup_data,
  cxd56_gnss_erase_backup_data,
  cxd56_gnss_open_cep_data,
  cxd56_gnss_close_cep_data,
  cxd56_gnss_check_cep_data,
  cxd56_gnss_get_cep_age,
  cxd56_gnss_reset_cep_flag,
  cxd56_gnss_start_rtk_output,
  cxd56_gnss_stop_rtk_output,
  cxd56_gnss_set_rtk_interval,
  cxd56_gnss_get_rtk_interval,
  cxd56_gnss_select_rtk_satellite,
  cxd56_gnss_get_rtk_satellite,
  cxd56_gnss_set_rtk_ephemeris_enable,
  cxd56_gnss_get_rtk_ephemeris_enable,
  cxd56_gnss_set_acquist_data,
  cxd56_gnss_set_frametime,
  cxd56_gnss_set_tau_gps,
  cxd56_gnss_set_time_gps,
  cxd56_gnss_clear_receiver_info,
  cxd56_gnss_set_tow_assist,
  cxd56_gnss_set_utc_model,
  cxd56_gnss_control_spectrum,
  cxd56_gnss_start_test,
  cxd56_gnss_stop_test,
  cxd56_gnss_get_test_result,
  cxd56_gnss_set_signal,
  cxd56_gnss_start_pvtlog,
  cxd56_gnss_stop_pvtlog,
  cxd56_gnss_delete_pvtlog,
  cxd56_gnss_get_pvtlog_status,
  cxd56_gnss_start_navmsg_output,
  cxd56_gnss_set_var_ephemeris,
  cxd56_gnss_get_var_ephemeris,
  cxd56_gnss_set_usecase,
  cxd56_gnss_get_usecase,
  cxd56_gnss_set_1pps_output,
  cxd56_gnss_get_1pps_output,

  /* max CXD56_GNSS_IOCTL_MAX */
};

static struct pm_cpu_freqlock_s g_lv_lock =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('G', 'T', 0),
                      PM_CPUFREQLOCK_FLAG_LV);

/* Lock to prohibit clock change in gnss open */

static struct pm_cpu_freqlock_s g_hold_lock =
  PM_CPUFREQLOCK_INIT(0, PM_CPUFREQLOCK_FLAG_HOLD);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IOCTL private functions */

/****************************************************************************
 * Name: cxd56_gnss_start
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_START command.
 *   Start a positioning
 *   beginning to search the satellites and measure the receiver position
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_start(struct file *filep, unsigned long arg)
{
  int     ret;
  int     retry = 50;
  uint8_t start_mode = (uint8_t)arg;

  ret = board_lna_power_control(true);
  if (ret < 0)
    {
      return ret;
    }

  while (!g_rtc_enabled && 0 < retry--)
    {
      /* GNSS requires stable RTC */

      nxsig_usleep(100 * 1000);
    }

  ret = cxd56_gnss_cpufifo_api(filep, CXD56_GNSS_GD_GNSS_START,
                               start_mode);
  if (ret < 0)
    {
      board_lna_power_control(false);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_stop
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_STOP command.
 *   Stop a positioning.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_stop(struct file *filep, unsigned long arg)
{
  int ret;

  ret = cxd56_gnss_cpufifo_api(filep, CXD56_GNSS_GD_GNSS_STOP, 0);
  board_lna_power_control(false);

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_get_satellite_system
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM command.
 *   Select GNSSs to positioning
 *   These are able to specified by CXD56_GNSS_B_SAT_XXX defines.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_select_satellite_system(struct file *filep,
                                              unsigned long arg)
{
  uint32_t system = (uint32_t)arg;

  return fw_gd_selectsatellitesystem(system);
}

/****************************************************************************
 * Name: cxd56_gnss_get_satellite_system
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_GET_SATELLITE_SYSTEM command.
 *   Get current using GNSSs to positioning
 *   A argument 'satellite' indicates current GNSSs by bit fields defined by
 *   CXD56_GNSS_B_SAT_XXX.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_satellite_system(struct file *filep,
                                           unsigned long arg)
{
  int ret;
  uint32_t system = 0;

  if (!arg)
    {
      return -EINVAL;
    }

  ret = fw_gd_getsatellitesystem(&system);
  *(uint32_t *)arg = system;

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_set_receiver_position_ellipsoidal
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ELLIPSOIDAL command.
 *   Set the rough receiver position
 *     arg = { double lat, double lon, double height }
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int
cxd56_gnss_set_receiver_position_ellipsoidal(struct file *filep,
                                             unsigned long arg)
{
  struct cxd56_gnss_ellipsoidal_position_s *pos;

  if (!arg)
    {
      return -EINVAL;
    }

  pos = (struct cxd56_gnss_ellipsoidal_position_s *)arg;

  return fw_gd_setreceiverpositionellipsoidal(&pos->latitude,
                                           &pos->longitude,
                                           &pos->altitude);
}

/****************************************************************************
 * Name: cxd56_gnss_set_receiver_position_orthogonal
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ORTHOGONAL command.
 *   Set the rough receiver position as orgothonal
 *     arg = { int32_t x, int32_t y, int32_t z }
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_receiver_position_orthogonal(
                                           struct file *filep,
                                           unsigned long arg)
{
  struct cxd56_gnss_orthogonal_position_s *pos;

  if (!arg)
    {
      return -EINVAL;
    }

  pos = (struct cxd56_gnss_orthogonal_position_s *)arg;
  return fw_gd_setreceiverpositionorthogonal(pos->x, pos->y, pos->z);
}

/****************************************************************************
 * Name: cxd56_gnss_set_ope_mode
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SET_OPE_MODE command.
 *   Set GNSS operation mode.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_ope_mode(struct file *filep, unsigned long arg)
{
  struct cxd56_gnss_ope_mode_param_s *ope_mode;
  if (!arg)
    {
      return -EINVAL;
    }

  ope_mode = (struct cxd56_gnss_ope_mode_param_s *)arg;

  return fw_gd_setoperationmode(ope_mode->mode, ope_mode->cycle);
}

/****************************************************************************
 * Name: cxd56_gnss_get_ope_mode
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_GET_OPE_MODE command.
 *   Set the TCXO offset
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_ope_mode(struct file *filep, unsigned long arg)
{
  struct cxd56_gnss_ope_mode_param_s *ope_mode;
  if (!arg)
    {
      return -EINVAL;
    }

  ope_mode = (struct cxd56_gnss_ope_mode_param_s *)arg;

  return fw_gd_getoperationmode(&ope_mode->mode, &ope_mode->cycle);
}

/****************************************************************************
 * Name: cxd56_gnss_set_tcxo_offset
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SET_TCXO_OFFSET command.
 *   Set the TCXO offset
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_tcxo_offset(struct file *filep,
                                      unsigned long arg)
{
  int32_t offset = (int32_t)arg;

  return fw_gd_settcxooffset(offset);
}

/****************************************************************************
 * Name: cxd56_gnss_get_tcxo_offset
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_GET_TCXO_OFFSET command.
 *   Get the TCXO offset
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_tcxo_offset(struct file *filep,
                                      unsigned long arg)
{
  int     ret;
  int32_t offset = 0;

  if (!arg)
    {
      return -EINVAL;
    }

  ret              = fw_gd_gettcxooffset(&offset);
  *(uint32_t *)arg = offset;

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_set_time
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SET_TIME command.
 *   Set the estimated current time of the receiver.
 *   1st argument date & time are in UTC.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_time(struct file *filep, unsigned long arg)
{
  struct cxd56_gnss_datetime_s *date_time;
  int ret;

  if (!arg)
    {
      return -EINVAL;
    }

  date_time = (struct cxd56_gnss_datetime_s *)arg;

  up_pm_acquire_freqlock(&g_lv_lock);
  ret = fw_gd_settime(&date_time->date, &date_time->time);
  up_pm_release_freqlock(&g_lv_lock);

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_get_almanac
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_GET_ALMANAC command.
 *   Get the almanac data
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_almanac(struct file *filep, unsigned long arg)
{
  struct cxd56_gnss_orbital_param_s *param;
  uint32_t                           almanac_size;

  if (!arg)
    {
      return -EINVAL;
    }

  param = (struct cxd56_gnss_orbital_param_s *)arg;

  return fw_gd_getalmanac(param->type, param->data, &almanac_size);
}

/****************************************************************************
 * Name: cxd56_gnss_set_almanac
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SET_ALMANAC command.
 *   Set the almanac data
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_almanac(struct file *filep, unsigned long arg)
{
  struct cxd56_gnss_orbital_param_s *param;

  if (!arg)
    {
      return -EINVAL;
    }

  param = (struct cxd56_gnss_orbital_param_s *)arg;

  return fw_gd_setalmanac(param->type, param->data);
}

/****************************************************************************
 * Name: cxd56_gnss_get_ephemeris
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_GET_EPHEMERIS command.
 *   Get the Ephemeris data
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_ephemeris(struct file *filep,
                                    unsigned long arg)
{
  struct cxd56_gnss_orbital_param_s *param;
  uint32_t                           ephemeris_size;

  if (!arg)
    {
      return -EINVAL;
    }

  param = (struct cxd56_gnss_orbital_param_s *)arg;

  return fw_gd_getephemeris(param->type, param->data, &ephemeris_size);
}

/****************************************************************************
 * Name: cxd56_gnss_set_ephemeris
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SET_EPHEMERIS command.
 *   Set the Ephemeris data
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_ephemeris(struct file *filep,
                                    unsigned long arg)
{
  struct cxd56_gnss_orbital_param_s *param;

  if (!arg)
    {
      return -EINVAL;
    }

  param = (struct cxd56_gnss_orbital_param_s *)arg;

  return fw_gd_setephemeris(param->type, param->data);
}

/****************************************************************************
 * Name: cxd56_gnss_save_backup_data
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SAVE_BACKUP_DATA command.
 *   Save the backup data to a Flash memory.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_save_backup_data(struct file *filep,
                                       unsigned long arg)
{
  char       *buf;
  struct file file;
  int         n = 0;
  int32_t     offset = 0;

  buf = (char *)kmm_malloc(CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE);
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  n = file_open(&file, CONFIG_CXD56_GNSS_BACKUP_FILENAME,
                O_WRONLY | O_CREAT | O_TRUNC);
  if (n < 0)
    {
      kmm_free(buf);
      return n;
    }

  do
    {
      n = fw_gd_readbuffer(CXD56_CPU1_DATA_TYPE_BACKUP, offset, buf,
                           CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE);
      if (n <= 0)
        {
          break;
        }

      n = file_write(&file, buf, n);
      offset += n;
    }
  while (n == CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE);

  kmm_free(buf);
  file_close(&file);

  return n < 0 ? n : 0;
}

/****************************************************************************
 * Name: cxd56_gnss_erase_backup_data
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_ERASE_BACKUP_DATA command.
 *   Erase the backup data on a Flash memory.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_erase_backup_data(struct file *filep,
                                        unsigned long arg)
{
  return nx_unlink(CONFIG_CXD56_GNSS_BACKUP_FILENAME);
}

/****************************************************************************
 * Name: cxd56_gnss_open_cep_data
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_OPEN_CEP_DATA command.
 *   Open CEP data file
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_open_cep_data(struct file *filep,
                                    unsigned long arg)
{
  return cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_CEPFILE, TRUE);
}

/****************************************************************************
 * Name: cxd56_gnss_close_cep_data
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_CLOSE_CEP_DATA command.
 *   Close CEP data file
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_close_cep_data(struct file *filep,
                                     unsigned long arg)
{
  return cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_CEPFILE, FALSE);
}

/****************************************************************************
 * Name: cxd56_gnss_check_cep_data
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_CHECK_CEP_DATA command.
 *   Check CEP data valid
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_check_cep_data(struct file *filep,
                                     unsigned long arg)
{
  return fw_gd_cepcheckassistdata();
}

/****************************************************************************
 * Name: cxd56_gnss_get_cep_age
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_GET_CEP_AGE command.
 *   Get CEP valid term
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_cep_age(struct file *filep, unsigned long arg)
{
  struct cxd56_gnss_cep_age_s *age;

  if (!arg)
    {
      return -EINVAL;
    }

  age = (struct cxd56_gnss_cep_age_s *)arg;

  return fw_gd_cepgetagedata(&age->age, &age->cepi);
}

/****************************************************************************
 * Name: cxd56_gnss_reset_cep_flag
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_RESET_CEP_FLAG command.
 *   Reset CEP data init flag & valid flag
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_reset_cep_flag(struct file *filep,
                                     unsigned long arg)
{
  return cxd56_gnss_cpufifo_api(filep,
                                CXD56_GNSS_GD_GNSS_CEPINITASSISTDATA, 0);
}

/****************************************************************************
 * Name: cxd56_gnss_set_acquist_data
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_AGPS_SET_ACQUIST command.
 *   AGPS set acquist data
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_acquist_data(struct file *filep,
                                       unsigned long arg)
{
  struct cxd56_gnss_agps_acquist_s *acquist;

  if (!arg)
    {
      return -EINVAL;
    }

  acquist = (struct cxd56_gnss_agps_acquist_s *)arg;

  return fw_gd_setacquist(acquist->data, acquist->size);
}

/****************************************************************************
 * Name: cxd56_gnss_set_frametime
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_AGPS_SET_FRAMETIME command.
 *   AGPS set frame time
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_frametime(struct file *filep,
                                    unsigned long arg)
{
  struct cxd56_gnss_agps_frametime_s *frametime;

  if (!arg)
    {
      return -EINVAL;
    }

  frametime = (struct cxd56_gnss_agps_frametime_s *)arg;

  return fw_gd_setframetime(frametime->sec, frametime->frac);
}

/****************************************************************************
 * Name: cxd56_gnss_set_tau_gps
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_AGPS_SET_TAU_GPS command.
 *   AGPS set TAU GPS
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_tau_gps(struct file *filep, unsigned long arg)
{
  struct cxd56_gnss_agps_tau_gps_s *taugpstime;

  if (!arg)
    {
      return -EINVAL;
    }

  taugpstime = (struct cxd56_gnss_agps_tau_gps_s *)arg;

  return fw_gd_settaugps(&taugpstime->taugps);
}

/****************************************************************************
 * Name: cxd56_gnss_set_time_gps
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_AGPS_SET_TIME_GPS command.
 *   Set high precision receiver time
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_time_gps(struct file *filep, unsigned long arg)
{
  struct cxd56_gnss_agps_time_gps_s *time_gps;

  if (!arg)
    {
      return -EINVAL;
    }

  time_gps = (struct cxd56_gnss_agps_time_gps_s *)arg;

  return fw_gd_settimegps(&time_gps->date, &time_gps->time);
}

/****************************************************************************
 * Name: cxd56_gnss_clear_receiver_info
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_AGPS_CLEAR_RECEIVER_INFO command.
 *   Clear info(s) for hot start such as ephemeris.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_clear_receiver_info(struct file *filep,
                                          unsigned long arg)
{
  uint32_t clear_type = arg;

  return fw_gd_clearreceiverinfo(clear_type);
}

/****************************************************************************
 * Name: cxd56_gnss_set_tow_assist
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_AGPS_SET_TOW_ASSIST command.
 *   AGPS set acquist data
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_tow_assist(struct file *filep,
                                     unsigned long arg)
{
  struct cxd56_gnss_agps_tow_assist_s *assist;

  if (!arg)
    {
      return -EINVAL;
    }

  assist = (struct cxd56_gnss_agps_tow_assist_s *)arg;

  return fw_gd_settowassist(assist->data, assist->size);
}

/****************************************************************************
 * Name: cxd56_gnss_set_utc_model
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_AGPS_SET_UTC_MODEL command.
 *   AGPS set UTC model
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_utc_model(struct file *filep,
                                    unsigned long arg)
{
  struct cxd56_gnss_agps_utc_model_s *model;

  if (!arg)
    {
      return -EINVAL;
    }

  model = (struct cxd56_gnss_agps_utc_model_s *)arg;

  return fw_gd_setutcmodel(model->data, model->size);
}

/****************************************************************************
 * Name: cxd56_gnss_control_spectrum
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SPECTRUM_CONTROL command.
 *   Enable or not to output spectrum data of GNSS signal
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_control_spectrum(struct file *filep,
                                       unsigned long arg)
{
  struct cxd56_gnss_spectrum_control_s *control;

  if (!arg)
    {
      return -EINVAL;
    }

  control = (struct cxd56_gnss_spectrum_control_s *)arg;

  return fw_gd_spectrumcontrol(control->time, control->enable,
                               control->point1, control->step1,
                               control->point2, control->step2);
}

/****************************************************************************
 * Name: cxd56_gnss_start_test
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_FACTORY_START_TEST command.
 *   Start GPS factory test
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_start_test(struct file *filep, unsigned long arg)
{
  int ret;
  int retry = 50;
  struct cxd56_gnss_test_info_s *info;

  /* check argument */

  if (!arg)
    {
      ret = -EINVAL;
    }
  else
    {
      /* Power on the LNA device */

      ret = board_lna_power_control(true);
      if (ret < 0)
        {
          return ret;
        }

      while (!g_rtc_enabled && 0 < retry--)
        {
          /* GNSS requires stable RTC */

          nxsig_usleep(100 * 1000);
        }

      /* set parameter */

      info = (struct cxd56_gnss_test_info_s *)arg;
      fw_gd_startgpstest(info->satellite, info->reserve1,
                      info->reserve2, info->reserve3);

      /* start test */

      ret = fw_gd_start(CXD56_GNSS_STMOD_COLD);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_stop_test
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_FACTORY_STOP_TEST command.
 *   Stop GPS factory test
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_stop_test(struct file *filep, unsigned long arg)
{
  int ret;

  /* term test */

  ret = fw_gd_stopgpstest();
  if (ret == OK)
    {
      /* stop test */

      ret = fw_gd_stop();
    }

  /* Power off the LNA device */

  board_lna_power_control(false);

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_get_test_result
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_FACTORY_GET_TEST_RESULT command.
 *   Get GPS factory test result
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_test_result(struct file *filep,
                                      unsigned long arg)
{
  struct cxd56_gnss_test_result_s *result;

  if (!arg)
    {
      return -EINVAL;
    }

  result = (struct cxd56_gnss_test_result_s *)arg;

  return fw_gd_getgpstestresult(&result->cn, &result->doppler);
}

/****************************************************************************
 * Name: cxd56_gnss_set_signal
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SIGNAL_SET command.
 *   Set signal information for synchronous reading data
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_signal(struct file *filep, unsigned long arg)
{
  int ret = 0;

#if CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0
  struct inode                       *inode;
  struct cxd56_gnss_dev_s            *priv;
  struct cxd56_gnss_signal_setting_s *setting;
  struct cxd56_gnss_sig_s            *sig;
  struct cxd56_gnss_sig_s            *checksig;
  pid_t                               pid;
  int                                 i;

  if (!arg)
    {
      return -EINVAL;
    }

  setting = (struct cxd56_gnss_signal_setting_s *)arg;
  if (setting->gnsssig >= CXD56_CPU1_DATA_TYPE_MAX)
    {
      return -EPROTOTYPE;
    }

  inode = filep->f_inode;
  priv  = (struct cxd56_gnss_dev_s *)inode->i_private;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  sig = NULL;
  pid = getpid();
  for (i = 0; i < CONFIG_CXD56_GNSS_NSIGNALRECEIVERS; i++)
    {
      checksig = &priv->sigs[i];
      if (setting->enable)
        {
          if (sig == NULL && !checksig->enable)
            {
              sig = checksig;
            }
          else if (checksig->info.gnsssig == setting->gnsssig &&
                   checksig->pid == pid)
            {
              sig = checksig;
              break;
            }
        }
      else if (checksig->info.gnsssig == setting->gnsssig &&
               checksig->pid == pid)
        {
          checksig->enable = 0;
          goto _success;
        }
    }

  if (sig == NULL)
    {
      ret = -ENOENT;
      goto _err;
    }

  fw_gd_setnotifymask(setting->gnsssig, FALSE);

  sig->enable       = 1;
  sig->pid          = pid;
  sig->info.fd      = setting->fd;
  sig->info.gnsssig = setting->gnsssig;
  sig->info.signo   = setting->signo;
  sig->info.data    = setting->data;

  _success:
  _err:
  nxsem_post(&priv->devsem);
#endif /* CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0 */

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_start_pvtlog
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_PVTLOG_START command.
 *   Start saving PVT logs.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_start_pvtlog(struct file *filep, unsigned long arg)
{
  struct cxd56_pvtlog_setting_s *setting;

  if (!arg)
    {
      return -EINVAL;
    }

  setting = (struct cxd56_pvtlog_setting_s *)arg;

  return fw_gd_registerpvtlog(setting->cycle, setting->threshold);
}

/****************************************************************************
 * Name: cxd56_gnss_stop_pvtlog
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_PVTLOG_STOP command.
 *   Stop saving PVT logs.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_stop_pvtlog(struct file *filep, unsigned long arg)
{
  return fw_gd_releasepvtlog();
}

/****************************************************************************
 * Name: cxd56_gnss_delete_pvtlog
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_PVTLOG_STOP command.
 *   Delete stored PVT logs.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_delete_pvtlog(struct file *filep,
                                    unsigned long arg)
{
  return fw_gd_pvtlogdeletelog();
}

/****************************************************************************
 * Name: cxd56_gnss_get_pvtlog_status
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_PVTLOG_GET_STATUS command.
 *   Get stored log status of PVTLOG.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_pvtlog_status(struct file *filep,
                                        unsigned long arg)
{
  struct cxd56_pvtlog_status_s *status;

  if (!arg)
    {
      return -EINVAL;
    }

  status = (struct cxd56_pvtlog_status_s *)arg;

  return fw_gd_pvtloggetlogstatus(&status->status);
}

/****************************************************************************
 * Name: cxd56_gnss_start_rtk_output
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_RTK_START command.
 *   Start RTK data output
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_start_rtk_output(struct file *filep,
                                       unsigned long arg)
{
  struct cxd56_rtk_setting_s *setting;

  if (!arg)
    {
      return -EINVAL;
    }

  setting = (struct cxd56_rtk_setting_s *)arg;
  setting->sbasout = 0;

  return fw_gd_rtkstart(setting);
}

/****************************************************************************
 * Name: cxd56_gnss_stop_rtk_output
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_RTK_STOP command.
 *   Stop RTK data output
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_stop_rtk_output(struct file *filep,
                                      unsigned long arg)
{
  return fw_gd_rtkstop();
}

/****************************************************************************
 * Name: cxd56_gnss_set_rtk_interval
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_RTK_SET_INTERVAL command.
 *   Set RTK data output interval
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_rtk_interval(struct file *filep,
                                       unsigned long arg)
{
  int interval = (int)arg;

  return fw_gd_rtksetoutputinterval(interval);
}

/****************************************************************************
 * Name: cxd56_gnss_get_rtk_interval
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_RTK_GET_INTERVAL command.
 *   Get RTK data output interval setting
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_rtk_interval(struct file *filep,
                                       unsigned long arg)
{
  int ret;
  int interval = 0;

  if (!arg)
    {
      return -EINVAL;
    }

  ret              = fw_gd_rtkgetoutputinterval(&interval);
  *(uint32_t *)arg = interval;

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_select_rtk_satellite
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_RTK_SELECT_SATELLITE_SYSTEM command.
 *   Select RTK satellite type
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_select_rtk_satellite(struct file *filep,
                                           unsigned long arg)
{
  uint32_t gnss  = (uint32_t)arg;

  return fw_gd_rtksetgnss(gnss);
}

/****************************************************************************
 * Name: cxd56_gnss_get_rtk_ephemeris_enable
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_RTK_GET_SATELLITE_SYSTEM command.
 *   Get RTK satellite type setting
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_rtk_satellite(struct file *filep,
                                        unsigned long arg)
{
  int       ret;
  uint32_t  gnss = 0;

  if (!arg)
    {
      return -EINVAL;
    }

  ret              = fw_gd_rtkgetgnss(&gnss);
  *(uint32_t *)arg = gnss;

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_get_rtk_ephemeris_enable
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_RTK_SET_EPHEMERIS_ENABLER command.
 *   Set RTK ephemeris notify enable setting
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_rtk_ephemeris_enable(struct file *filep,
                                               unsigned long arg)
{
  int enable = (int)arg;

  return fw_gd_rtksetephnotify(enable);
}

/****************************************************************************
 * Name: cxd56_gnss_get_rtk_ephemeris_enable
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_RTK_GET_EPHEMERIS_ENABLER command.
 *   Get RTK ephemeris notify enable setting.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_rtk_ephemeris_enable(struct file *filep,
                                               unsigned long arg)
{
  int ret;
  int enable = 0;

  if (!arg)
    {
      return -EINVAL;
    }

  ret              = fw_gd_rtkgetephnotify(&enable);
  *(uint32_t *)arg = enable;

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_start_navmsg_output
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_NAVMSG_START command.
 *   Start NAVMSG data output
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_start_navmsg_output(struct file *filep,
                                          unsigned long arg)
{
  struct cxd56_rtk_setting_s *setting;

  if (!arg)
    {
      return -EINVAL;
    }

  setting = (struct cxd56_rtk_setting_s *)arg;

  return fw_gd_rtkstart(setting);
}

/****************************************************************************
 * Name: cxd56_gnss_set_var_ephemeris
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_SET_VAR_EPHEMERIS command.
 *   Set the Ephemeris data
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_var_ephemeris(struct file *filep,
                                        unsigned long arg)
{
  struct cxd56_gnss_set_var_ephemeris_s *param;

  if (!arg)
    {
      return -EINVAL;
    }

  param = (struct cxd56_gnss_set_var_ephemeris_s *)arg;

  return fw_gd_setvarephemeris(param->data, param->size);
}

/****************************************************************************
 * Name: cxd56_gnss_get_var_ephemeris
 *
 * Description:
 *   Process CXD56_GNSS_IOCTL_GET_VAR_EPHEMERIS command.
 *   Get the Ephemeris data
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_var_ephemeris(struct file *filep,
                                        unsigned long arg)
{
  struct cxd56_gnss_get_var_ephemeris_s *param;

  if (!arg)
    {
      return -EINVAL;
    }

  param = (struct cxd56_gnss_get_var_ephemeris_s *)arg;

  return fw_gd_getvarephemeris(param->type, param->data, param->size);
}

/****************************************************************************
 * Name: cxd56_gnss_set_usecase
 *
 * Description:
 *   Set usecase mode
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_usecase(struct file *filep,
                                  unsigned long arg)
{
  return fw_gd_setusecase(arg);
}

/****************************************************************************
 * Name: cxd56_gnss_get_usecase
 *
 * Description:
 *   Get usecase mode
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_usecase(struct file *filep,
                                  unsigned long arg)
{
  int ret;
  uint32_t usecase = 0;

  if (!arg)
    {
      return -EINVAL;
    }

  ret = fw_gd_getusecase(&usecase);
  *(uint32_t *)arg = usecase;

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_set_1pps_output
 *
 * Description:
 *   Set enable or disable of 1PPS output
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_set_1pps_output(struct file *filep,
                                      unsigned long arg)
{
  if (arg)
    {
      /* Enable 1PPS output pin */

#ifdef CONFIG_CXD56_GNSS_1PPS_PIN_GNSS_1PPS_OUT
      CXD56_PIN_CONFIGS(PINCONFS_GNSS_1PPS_OUT);
#else
      CXD56_PIN_CONFIGS(PINCONFS_HIF_IRQ_OUT_GNSS_1PPS_OUT);
#endif
    }
  else
    {
      /* Disable 1PPS output pin */

#ifdef CONFIG_CXD56_GNSS_1PPS_PIN_GNSS_1PPS_OUT
      CXD56_PIN_CONFIGS(PINCONFS_GNSS_1PPS_OUT_GPIO);
#else
      CXD56_PIN_CONFIGS(PINCONFS_HIF_IRQ_OUT_GPIO);
#endif
    }

  return fw_gd_set1ppsoutput(arg);
}

/****************************************************************************
 * Name: cxd56_gnss_get_1pps_output
 *
 * Description:
 *   Get the current 1PPS output setting
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   arg   - Data for command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_get_1pps_output(struct file *filep,
                                      unsigned long arg)
{
  int ret;
  uint32_t enable = 0;

  if (!arg)
    {
      return -EINVAL;
    }

  ret = fw_gd_get1ppsoutput(&enable);
  *(uint32_t *)arg = enable;

  return ret;
}

/* Synchronized with processes and CPUs
 *  CXD56_GNSS signal handler and utils
 */

/****************************************************************************
 * Name: cxd56_gnss_wait_notify
 *
 * Description:
 *   Wait notify from GNSS CPU with timeout.
 *
 * Input Parameters:
 *   sem     - Semaphore for waiting
 *   waitset - Wait time in seconds
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_wait_notify(sem_t *sem, time_t waitsec)
{
  return nxsem_tickwait(sem, SEC2TICK(waitsec));
}

/****************************************************************************
 * Name: cxd56_gnss_read_cep_file
 *
 * Description:
 *   Read a CEP data packet from file and notify to GNSS CPU.
 *
 * Input Parameters:
 *   fp     - File pointer
 *   offset - File offset of last read
 *   len    - packet size to read
 *   retval - Status to read file
 *
 * Returned Value:
 *   Buffer address allocated in this function for reading data.
 *
 ****************************************************************************/

static char *
cxd56_gnss_read_cep_file(struct file *fp, int32_t offset,
                         size_t len, int *retval)
{
  char *buf;
  int   ret;

  if (fp == NULL)
    {
      ret = -ENOENT;
      goto _err0;
    }

  buf = (char *)kmm_malloc(len);
  if (buf == NULL)
    {
      ret = -ENOMEM;
      goto _err0;
    }

  ret = file_seek(fp, offset, SEEK_SET);
  if (ret < 0)
    {
      goto _err1;
    }

  ret = file_read(fp, buf, len);
  if (ret <= 0)
    {
      goto _err1;
    }

  *retval = ret;
  cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_CEP, (uint32_t)buf);

  return buf;

  /* Send signal to CPU1 in error for just notify completion of read
   * sequence.
   */

  _err1:
  kmm_free(buf);
  _err0:
  *retval = ret;
  cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_CEP, 0);

  return NULL;
}

/****************************************************************************
 * Name: cxd56_gnss_read_backup_file
 *
 * Description:
 *   Read a backup data packet from file and notify to GNSS CPU.
 *
 * Input Parameters:
 *   retval - Status to read file
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cxd56_gnss_read_backup_file(int *retval)
{
  char       *buf;
  struct file file;
  int32_t     offset = 0;
  size_t      n;
  int         ret = 0;

  buf = (char *)kmm_malloc(CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE);
  if (buf == NULL)
    {
      ret = -ENOMEM;
      goto _err;
    }

  ret = file_open(&file, CONFIG_CXD56_GNSS_BACKUP_FILENAME, O_RDONLY);
  if (ret < 0)
    {
      kmm_free(buf);
      goto _err;
    }

  do
    {
      n = file_read(&file, buf, CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE);
      if (n <= 0)
        {
          ret = n;
          break;
        }

      ret = fw_gd_writebuffer(CXD56_CPU1_DATA_TYPE_BACKUP, offset, buf, n);
      if (ret < 0)
        {
          break;
        }

      offset += n;
    }
  while (n > 0);

  file_close(&file);
  kmm_free(buf);

  /* Notify the termination of backup sequence by write zero length data */

  _err:
  *retval = ret;
  cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_BKUPFILE, 0);
}

/****************************************************************************
 * Name: cxd56_gnss_common_signalhandler
 *
 * Description:
 *   Common signal handler from GNSS CPU.
 *
 * Input Parameters:
 *   data     - Received data from GNSS CPU
 *   userdata - User data, this is the device information specified by the
 *              second argument of the function cxd56_cpu1siginit.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0
static void cxd56_gnss_common_signalhandler(uint32_t data,
                                            void *userdata)
{
  struct cxd56_gnss_dev_s *priv =
                          (struct cxd56_gnss_dev_s *)userdata;
  uint8_t                  sigtype = CXD56_CPU1_GET_DEV(data);
  int                      issetmask = 0;
  int                      i;
  int                      ret;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return;
    }

  for (i = 0; i < CONFIG_CXD56_GNSS_NSIGNALRECEIVERS; i++)
    {
      struct cxd56_gnss_sig_s *sig = &priv->sigs[i];
      if (sig->enable && sig->info.gnsssig == sigtype)
        {
          union sigval value;

          value.sival_ptr = &sig->info;
          nxsig_queue(sig->pid, sig->info.signo, value);
          issetmask = 1;
        }
    }

  if (issetmask)
    {
      fw_gd_setnotifymask(sigtype, FALSE);
    }

  nxsem_post(&priv->devsem);
}
#endif /* CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0 */

/****************************************************************************
 * Name: cxd56_gnss_default_sighandler
 *
 * Description:
 *   Handler for GNSS type notification from GNSS CPU for signal and poll.
 *
 * Input Parameters:
 *   data     - Received data from GNSS CPU
 *   userdata - User data, this is the device information specified by the
 *              second argument of the function cxd56_cpu1siginit.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cxd56_gnss_default_sighandler(uint32_t data, void *userdata)
{
  struct cxd56_gnss_dev_s *priv =
                          (struct cxd56_gnss_dev_s *)userdata;
  int                      ret;
  int                      dtype = CXD56_CPU1_GET_DATA(data);

  switch (dtype)
    {
    case CXD56_GNSS_NOTIFY_TYPE_REQCEPDAT:
      {
        priv->cepbuf = cxd56_gnss_read_cep_file(
          &priv->cepfp, priv->shared_info.argv[GNSS_ARGS_FILE_OFFSET],
          priv->shared_info.argv[GNSS_ARGS_FILE_LENGTH],
          &priv->shared_info.retval);
        return;
      }

    case CXD56_GNSS_NOTIFY_TYPE_REQCEPBUFFREE:
      if (priv->cepbuf)
        {
          kmm_free(priv->cepbuf);
        }

      return;

    case CXD56_GNSS_NOTIFY_TYPE_BOOTCOMP:
      if (priv->num_open == 0)
        {
          /* Post to wait-semaphore in cxd56_gnss_open to notify completion
           * of GNSS core initialization in first device open.
           */

          priv->notify_data = dtype;
          nxsem_post(&priv->syncsem);
        }

      return;

    case CXD56_GNSS_NOTIFY_TYPE_REQBKUPDAT:
      cxd56_gnss_read_backup_file(&priv->shared_info.retval);
      return;

    case CXD56_GNSS_NOTIFY_TYPE_REQCEPOPEN:
      if (priv->cepfp.f_inode != NULL)
        {
          file_close(&priv->cepfp);
        }

      file_open(&priv->cepfp, CONFIG_CXD56_GNSS_CEP_FILENAME, O_RDONLY);
      return;

    case CXD56_GNSS_NOTIFY_TYPE_REQCEPCLOSE:
      if (priv->cepfp.f_inode != NULL)
        {
          file_close(&priv->cepfp);
        }

      return;

    default:
      break;
    }

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return;
    }

  poll_notify(priv->fds, CONFIG_CXD56_GNSS_NPOLLWAITERS, POLLIN);

  nxsem_post(&priv->devsem);

#if CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0
  cxd56_gnss_common_signalhandler(data, userdata);
#endif
}

/****************************************************************************
 * Name: cxd56_gnss_cpufifoapi_signalhandler
 *
 * Description:
 *   Handler for API type notification from GNSS CPU.
 *
 * Input Parameters:
 *   data     - Received data from GNSS CPU
 *   userdata - User data, this is the device information specified by the
 *              second argument of the function cxd56_cpu1siginit.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cxd56_gnss_cpufifoapi_signalhandler(uint32_t data,
                                                void *userdata)
{
  struct cxd56_gnss_dev_s *priv =
                              (struct cxd56_gnss_dev_s *)userdata;

  priv->apiret = CXD56_CPU1_GET_DATA((int)data);
  nxsem_post(&priv->apiwait);
}

/****************************************************************************
 * Name: cxd56_gnss_cpufifo_api
 *
 * Description:
 *   Send API type event to GNSS CPU.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   api   - Signal type
 *   data  - Any data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_cpufifo_api(struct file *filep, unsigned int api,
                                  unsigned int data)
{
  struct inode            *inode;
  struct cxd56_gnss_dev_s *priv;
  unsigned int             type;
  int                      ret = OK;

  inode = filep->f_inode;
  priv  = (struct cxd56_gnss_dev_s *)inode->i_private;

  type = CXD56_GNSS_CPUFIFOAPI_SET_DATA(api, data);
  cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_CPUFIFOAPI, type);

  ret = nxsem_wait(&priv->apiwait);
  if (ret < 0)
    {
      /* If nxsem_wait returns -EINTR, there is a possibility that the signal
       * for GNSS set with CXD56_GNSS_IOCTL_SIGNAL_SET is unmasked
       * by SIG_UNMASK in the signal mask.
       */

      _warn("Cannot wait GNSS semaphore %d\n", ret);
      goto _err;
    }

  ret = priv->apiret;

  _err:
  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_select_notifytype
 *
 * Description:
 *   Decide notify type about data from GNSS device
 *
 * Input Parameters:
 *   fpos   - file offset indicated about data type
 *   offset - Actual offset value
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int8_t cxd56_gnss_select_notifytype(off_t fpos, uint32_t *offset)
{
  int8_t type;

  if ((fpos >= CXD56_GNSS_READ_OFFSET_LAST_GNSS) &&
      (fpos < CXD56_GNSS_READ_OFFSET_AGPS))
    {
      type = CXD56_CPU1_DATA_TYPE_GNSS;
      *offset = fpos;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_AGPS)
    {
      type = CXD56_CPU1_DATA_TYPE_AGPS;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_RTK)
    {
      type = CXD56_CPU1_DATA_TYPE_RTK;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_GPSEPHEMERIS)
    {
      type = CXD56_CPU1_DATA_TYPE_GPSEPHEMERIS;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_GLNEPHEMERIS)
    {
      type = CXD56_CPU1_DATA_TYPE_GLNEPHEMERIS;
      *offset = 0;
    }
  else if ((fpos == CXD56_GNSS_READ_OFFSET_SPECTRUM) ||
           (fpos == CXD56_GNSS_READ_OFFSET_INFO))
    {
      type = CXD56_CPU1_DATA_TYPE_SPECTRUM;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_PVTLOG)
    {
      type = CXD56_CPU1_DATA_TYPE_PVTLOG;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_SBAS)
    {
      type = CXD56_CPU1_DATA_TYPE_SBAS;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_DCREPORT)
    {
      type = CXD56_CPU1_DATA_TYPE_DCREPORT;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_SARRLM)
    {
      type = CXD56_CPU1_DATA_TYPE_SARRLM;
      *offset = 0;
    }
  else
    {
      type = -1;
    }

  return type;
}

/****************************************************************************
 * Name: cxd56_gnss_initialize
 *
 * Description:
 *   initialize gnss device
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_initialize(struct cxd56_gnss_dev_s *dev)
{
  int32_t ret = 0;

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_open
 *
 * Description:
 *   Standard character driver open method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_open(struct file *filep)
{
  struct inode *           inode;
  struct cxd56_gnss_dev_s *priv;
  int                      ret = OK;
  int                      retry = 50;

  inode = filep->f_inode;
  priv  = (struct cxd56_gnss_dev_s *)inode->i_private;

  while (!g_rtc_enabled && 0 < retry--)
    {
      /* GNSS requires stable RTC */

      usleep(100 * 1000);
    }

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->num_open == 0)
    {
      ret = nxsem_init(&priv->syncsem, 0, 0);
      if (ret < 0)
        {
          goto _err0;
        }

      nxsem_set_protocol(&priv->syncsem, SEM_PRIO_NONE);

      /* Prohibit the clock change during loading image */

      up_pm_acquire_freqlock(&g_hold_lock);

      ret = fw_pm_loadimage(CXD56_GNSS_GPS_CPUID, CXD56_GNSS_FWNAME);

      /* Allow the clock change after loading image */

      up_pm_release_freqlock(&g_hold_lock);

      if (ret < 0)
        {
          goto _err1;
        }

      ret = fw_pm_startcpu(CXD56_GNSS_GPS_CPUID, 1);
      if (ret < 0)
        {
          goto _err2;
        }

#ifndef CONFIG_CXD56_GNSS_HOT_SLEEP
      fw_pm_sleepcpu(CXD56_GNSS_GPS_CPUID, PM_SLEEP_MODE_HOT_DISABLE);
#endif

      /* Wait the request from GNSS core to restore backup data,
       * or for completion of initialization of GNSS core here.
       * It is post the semaphore syncsem from cxd56_gnss_default_sighandler.
       */

      ret = cxd56_gnss_wait_notify(&priv->syncsem, 5);
      if (ret < 0)
        {
          goto _err2;
        }

      ret = fw_gd_writebuffer(CXD56_CPU1_DATA_TYPE_INFO, 0,
                              &priv->shared_info, sizeof(priv->shared_info));
      if (ret < 0)
        {
          goto _err2;
        }

      nxsem_destroy(&priv->syncsem);
    }

  priv->num_open++;
  goto _success;

  _err2:
#ifndef CONFIG_CXD56_GNSS_HOT_SLEEP
  fw_pm_sleepcpu(CXD56_GNSS_GPS_CPUID, PM_SLEEP_MODE_HOT_ENABLE);
#endif
  fw_pm_sleepcpu(CXD56_GNSS_GPS_CPUID, PM_SLEEP_MODE_COLD);
  _err1:
  nxsem_destroy(&priv->syncsem);
  _err0:
  _success:
  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_close
 *
 * Description:
 *   Standard character driver close method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_close(struct file *filep)
{
  struct inode *          inode;
  struct cxd56_gnss_dev_s *priv;
  int                     ret = OK;

  inode = filep->f_inode;
  priv  = (struct cxd56_gnss_dev_s *)inode->i_private;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  priv->num_open--;
  if (priv->num_open == 0)
    {
#ifndef CONFIG_CXD56_GNSS_HOT_SLEEP
      fw_pm_sleepcpu(CXD56_GNSS_GPS_CPUID, PM_SLEEP_MODE_HOT_ENABLE);
#endif

      ret = fw_pm_sleepcpu(CXD56_GNSS_GPS_CPUID, PM_SLEEP_MODE_COLD);
      if (ret < 0)
        {
          goto errout;
        }
    }

  errout:
  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_read
 *
 * Description:
 *   Standard character driver read method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   buffer - Buffer to read from GNSS device
 *   buflen - The read length of the buffer
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static ssize_t cxd56_gnss_read(struct file *filep, char *buffer,
                               size_t len)
{
  int32_t   ret = 0;
  uint32_t  offset = 0;
  int8_t    type;

  if (!buffer)
    {
      ret = -EINVAL;
      goto _err;
    }

  if (len == 0)
    {
      goto _success;
    }

  /* setect data type */

  type = cxd56_gnss_select_notifytype(filep->f_pos, &offset);
  if (type < 0)
    {
      ret = -ESPIPE;
      goto _err;
    }

  if (type == CXD56_CPU1_DATA_TYPE_GNSS)
    {
      /* Trim len if read would go beyond end of device */

      if ((offset + len) > sizeof(struct cxd56_gnss_positiondata_s))
        {
          len = sizeof(struct cxd56_gnss_positiondata_s) - offset;
        }
    }
  else if (type == CXD56_CPU1_DATA_TYPE_AGPS)
    {
      if ((offset + len) > sizeof(struct cxd56_supl_mesurementdata_s))
        {
          len = sizeof(struct cxd56_supl_mesurementdata_s) - offset;
        }
    }

  /* fw_gd_readbuffer returns copied data size or negative error code */

  ret = fw_gd_readbuffer(type, offset, buffer, len);

  _err:
  _success:
  filep->f_pos = 0;
  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_write
 *
 * Description:
 *   Standard character driver write method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   buffer - Buffer to write
 *   buflen - The write length of the buffer
 *
 * Returned Value:
 *   Always returns -ENOENT error.
 *
 ****************************************************************************/

static ssize_t cxd56_gnss_write(struct file *filep,
                                const char *buffer, size_t buflen)
{
  return -ENOENT;
}

/****************************************************************************
 * Name: cxd56_gnss_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   fds   - Array of file descriptor
 *   setup - 1 if start poll, 0 if stop poll
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_ioctl(struct file *filep, int cmd,
                            unsigned long arg)
{
  struct inode *          inode;
  struct cxd56_gnss_dev_s *priv;
  int ret;

  inode = filep->f_inode;
  priv  = (struct cxd56_gnss_dev_s *)inode->i_private;

  if (cmd <= CXD56_GNSS_IOCTL_INVAL || cmd >= CXD56_GNSS_IOCTL_MAX)
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&priv->ioctllock);
  if (ret < 0)
    {
      return ret;
    }

  ret = g_cmdlist[cmd](filep, arg);

  nxsem_post(&priv->ioctllock);

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   fds   - array of file descriptor
 *   setup - 1 if start poll, 0 if stop poll
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_poll(struct file *filep, struct pollfd *fds,
                           bool setup)
{
  struct inode            *inode;
  struct cxd56_gnss_dev_s *priv;
  int                      ret = OK;
  int                      i;

  inode = filep->f_inode;
  priv  = (struct cxd56_gnss_dev_s *)inode->i_private;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      for (i = 0; i < CONFIG_CXD56_GNSS_NPOLLWAITERS; i++)
        {
          /* Find an unused slot */

          if (priv->fds[i] == NULL)
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              fw_gd_setnotifymask(CXD56_CPU1_DEV_GNSS, FALSE);
              break;
            }
        }

      /* No space in priv fds array for poll handling */

      if (i >= CONFIG_CXD56_GNSS_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret       = -EBUSY;
          goto errout;
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_register
 *
 * Description:
 *   Register the GNSS character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gps"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_register(const char *devpath)
{
  struct cxd56_gnss_dev_s *priv;
  int                      i;
  int                      ret;

  static struct cxd56_devsig_table_s devsig_table[] =
  {
    {
      CXD56_CPU1_DATA_TYPE_GNSS,
      cxd56_gnss_default_sighandler
    },
    {
      CXD56_CPU1_DATA_TYPE_AGPS,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_RTK,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_GPSEPHEMERIS,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_GLNEPHEMERIS,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_SPECTRUM,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_PVTLOG,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_CPUFIFOAPI,
      cxd56_gnss_cpufifoapi_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_SBAS,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_DCREPORT,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_SARRLM,
      cxd56_gnss_common_signalhandler
    }
  };

  priv = (struct cxd56_gnss_dev_s *)kmm_malloc(
    sizeof(struct cxd56_gnss_dev_s));
  if (!priv)
    {
      gnsserr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  memset(priv, 0, sizeof(struct cxd56_gnss_dev_s));

  ret = nxsem_init(&priv->devsem, 0, 1);
  if (ret < 0)
    {
      gnsserr("Failed to initialize gnss devsem!\n");
      goto _err0;
    }

  ret = nxsem_init(&priv->apiwait, 0, 0);
  if (ret < 0)
    {
      gnsserr("Failed to initialize gnss apiwait!\n");
      goto _err0;
    }

  nxsem_set_protocol(&priv->apiwait, SEM_PRIO_NONE);

  ret = nxsem_init(&priv->ioctllock, 0, 1);
  if (ret < 0)
    {
      gnsserr("Failed to initialize gnss ioctllock!\n");
      goto _err0;
    }

  ret = cxd56_gnss_initialize(priv);
  if (ret < 0)
    {
      gnsserr("Failed to initialize gnss device!\n");
      goto _err0;
    }

  ret = register_driver(devpath, &g_gnssfops, 0666, priv);
  if (ret < 0)
    {
      gnsserr("Failed to register driver: %d\n", ret);
      goto _err0;
    }

  for (i = 0; i < sizeof(devsig_table) / sizeof(devsig_table[0]); i++)
    {
      ret = cxd56_cpu1siginit(devsig_table[i].sigtype, priv);
      if (ret < 0)
        {
          gnsserr("Failed to initialize ICC for GPS CPU: %d,%d\n", ret,
                devsig_table[i].sigtype);
          goto _err2;
        }

      cxd56_cpu1sigregisterhandler(devsig_table[i].sigtype,
                                   devsig_table[i].handler);
    }

  gnssinfo("GNSS driver loaded successfully!\n");

  return ret;

  _err2:
  unregister_driver(devpath);

  _err0:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: cxd56_gnssinitialize
 *
 * Description:
 *   Initialize GNSS device
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gps"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gnssinitialize(const char *devpath)
{
  int ret;

  gnssinfo("Initializing GNSS..\n");

  ret = cxd56_gnss_register(devpath);
  if (ret < 0)
    {
      gnsserr("Error registering GNSS\n");
    }

  return ret;
}

#endif
