/****************************************************************************
 * drivers/input/mxt.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#ifndef __DRIVERS_INPUT_MXT_H
#define __DRIVERS_INPUT_MXT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Support T6 only if debug is enabled */

#undef MXT_SUPPORT_T6
#if !defined(CONFIG_DEBUG_FEATURES)
#  undef CONFIG_DEBUG_INFO
#  undef CONFIG_DEBUG_INPUT
#endif

#if defined(CONFIG_DEBUG_INFO) && defined(CONFIG_DEBUG_INPUT)
#  define MXT_SUPPORT_T6 1
#endif

/* MXT Register Definitions *************************************************/

#define MXT_INFO                       0x00 /* 7-bit info block */
#  define MXT_FAMILY_ID                0x00 /* MXT family ID */
#  define MXT_VARIANT_ID               0x01 /* MXT variant ID */
#  define MXT_VERSION                  0x02 /* MXT version number */
#  define MXT_BUILD                    0x03 /* MXT build number */
#  define MXT_MATRIX_X_SIZE            0x04 /* Matrix X size */
#  define MXT_MATRIX_Y_SIZE            0x05 /* Matrix Y size */
#  define MXT_OBJECT_NUM               0x06 /* Number of objects */
#define MXT_OBJECT_START               0x07

/* Object types */

#define MXT_GEN_MESSAGE_T5             5
#define MXT_GEN_COMMAND_T6             6
#define MXT_GEN_POWER_T7               7
#define MXT_GEN_ACQUIRE_T8             8
#define MXT_TOUCH_MULTI_T9             9
#define MXT_TOUCH_KEYARRAY_T15         15
#define MXT_SPT_COMMSCONFIG_T18        18
#define MXT_SPT_GPIOPWM_T19            19
#define MXT_PROCI_GRIPFACE_T20         20
#define MXT_PROCG_NOISE_T22            22
#define MXT_TOUCH_PROXIMITY_T23        23
#define MXT_PROCI_ONETOUCH_T24         24
#define MXT_SPT_SELFTEST_T25           25
#define MXT_PROCI_TWOTOUCH_T27         27
#define MXT_SPT_CTECONFIG_T28          28
#define MXT_DEBUG_DIAGNOSTIC_T37       37
#define MXT_SPT_USERDATA_T38           38
#define MXT_PROCI_GRIP_T40             40
#define MXT_PROCI_PALM_T41             41
#define MXT_PROCI_TOUCHSUPPRESSION_T42 42
#define MXT_SPT_DIGITIZER_T43          43
#define MXT_SPT_MESSAGECOUNT_T44       44
#define MXT_SPT_CTECONFIG_T46          46
#define MXT_PROCI_STYLUS_T47           47
#define MXT_PROCG_NOISESUPPRESSION_T48 48
#define MXT_TOUCH_PROXKEY_T52          52
#define MXT_GEN_DATASOURCE_T53         53

/* MXT_GEN_COMMAND_T6 field */

#define MXT_COMMAND_RESET              0
#define MXT_COMMAND_BACKUPNV           1
#define MXT_COMMAND_CALIBRATE          2
#define MXT_COMMAND_REPORTALL          3
#define MXT_COMMAND_DIAGNOSTIC         5

/* MXT_GEN_POWER_T7 field */

#define MXT_POWER_IDLEACQINT           0
#define MXT_POWER_ACTVACQINT           1
#define MXT_POWER_ACTV2IDLETO          2

/* MXT_GEN_ACQUIRE_T8 field */

#define MXT_ACQUIRE_CHRGTIME           0
#define MXT_ACQUIRE_TCHDRIFT           2
#define MXT_ACQUIRE_DRIFTST            3
#define MXT_ACQUIRE_TCHAUTOCAL         4
#define MXT_ACQUIRE_SYNC               5
#define MXT_ACQUIRE_ATCHCALST          6
#define MXT_ACQUIRE_ATCHCALSTHR        7

/* MXT_TOUCH_MULTI_T9 field */

#define MXT_TOUCH_CTRL                 0
#define MXT_TOUCH_XORIGIN              1
#define MXT_TOUCH_YORIGIN              2
#define MXT_TOUCH_XSIZE                3
#define MXT_TOUCH_YSIZE                4
#define MXT_TOUCH_BLEN                 6
#define MXT_TOUCH_TCHTHR               7
#define MXT_TOUCH_TCHDI                8
#define MXT_TOUCH_ORIENT               9
#define MXT_TOUCH_MOVHYSTI             11
#define MXT_TOUCH_MOVHYSTN             12
#define MXT_TOUCH_NUMTOUCH             14
#define MXT_TOUCH_MRGHYST              15
#define MXT_TOUCH_MRGTHR               16
#define MXT_TOUCH_AMPHYST              17
#define MXT_TOUCH_XRANGE_LSB           18
#define MXT_TOUCH_XRANGE_MSB           19
#define MXT_TOUCH_YRANGE_LSB           20
#define MXT_TOUCH_YRANGE_MSB           21
#define MXT_TOUCH_XLOCLIP              22
#define MXT_TOUCH_XHICLIP              23
#define MXT_TOUCH_YLOCLIP              24
#define MXT_TOUCH_YHICLIP              25
#define MXT_TOUCH_XEDGECTRL            26
#define MXT_TOUCH_XEDGEDIST            27
#define MXT_TOUCH_YEDGECTRL            28
#define MXT_TOUCH_YEDGEDIST            29
#define MXT_TOUCH_JUMPLIMIT            30

/* MXT_PROCI_GRIPFACE_T20 field */

#define MXT_GRIPFACE_CTRL              0
#define MXT_GRIPFACE_XLOGRIP           1
#define MXT_GRIPFACE_XHIGRIP           2
#define MXT_GRIPFACE_YLOGRIP           3
#define MXT_GRIPFACE_YHIGRIP           4
#define MXT_GRIPFACE_MAXTCHS           5
#define MXT_GRIPFACE_SZTHR1            7
#define MXT_GRIPFACE_SZTHR2            8
#define MXT_GRIPFACE_SHPTHR1           9
#define MXT_GRIPFACE_SHPTHR2           10
#define MXT_GRIPFACE_SUPEXTTO          11

/* MXT_PROCI_NOISE field */

#define MXT_NOISE_CTRL                 0
#define MXT_NOISE_OUTFLEN              1
#define MXT_NOISE_GCAFUL_LSB           3
#define MXT_NOISE_GCAFUL_MSB           4
#define MXT_NOISE_GCAFLL_LSB           5
#define MXT_NOISE_GCAFLL_MSB           6
#define MXT_NOISE_ACTVGCAFVALID        7
#define MXT_NOISE_NOISETHR             8
#define MXT_NOISE_FREQHOPSCALE         10
#define MXT_NOISE_FREQ0                11
#define MXT_NOISE_FREQ1                12
#define MXT_NOISE_FREQ2                13
#define MXT_NOISE_FREQ3                14
#define MXT_NOISE_FREQ4                15
#define MXT_NOISE_IDLEGCAFVALID        16

/* MXT_SPT_COMMSCONFIG_T18 */

#define MXT_COMMS_CTRL                 0
#define MXT_COMMS_CMD                  1

/* MXT_SPT_CTECONFIG_T28 field */

#define MXT_CTE_CTRL                   0
#define MXT_CTE_CMD                    1
#define MXT_CTE_MODE                   2
#define MXT_CTE_IDLEGCAFDEPTH          3
#define MXT_CTE_ACTVGCAFDEPTH          4
#define MXT_CTE_VOLTAGE                5

#define MXT_VOLTAGE_DEFAULT            2700000
#define MXT_VOLTAGE_STEP               10000

/* Definitions for MXT_GEN_COMMAND_T6 */

#define MXT_BOOT_VALUE                 0xa5
#define MXT_BACKUP_VALUE               0x55
#define MXT_BACKUP_TIME                50000   /* microseconds */
#define MXT_RESET_TIME                 500000  /* microseconds */

/* MXT_SPT_GPIOPWM_T19 field */

#define MXT_MAX_BUTTONS                4       /* Up to four buttons */
#define MXT_GPIO0_MASK                 0x04
#define MXT_GPIO1_MASK                 0x08
#define MXT_GPIO2_MASK                 0x10
#define MXT_GPIO3_MASK                 0x20

/* Command to unlock bootloader */

#define MXT_UNLOCK_CMD_MSB             0xaa
#define MXT_UNLOCK_CMD_LSB             0xdc

/* Bootloader mode status */

#define MXT_WAITING_BOOTLOAD_CMD       0xc0  /* Valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA         0x80  /* Valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK            0x02
#define MXT_FRAME_CRC_FAIL             0x03
#define MXT_FRAME_CRC_PASS             0x04
#define MXT_APP_CRC_FAIL               0x40  /* Valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK           0x3f

/* Touch status */

#define MXT_UNGRIP                     (1 << 0)
#define MXT_SUPPRESS                   (1 << 1)
#define MXT_AMP                        (1 << 2)
#define MXT_VECTOR                     (1 << 3)
#define MXT_MOVE                       (1 << 4)
#define MXT_RELEASE                    (1 << 5)
#define MXT_PRESS                      (1 << 6)
#define MXT_DETECT                     (1 << 7)

/* Touchscreen absolute values */

#define MXT_MAX_AREA                   0xff
#define MXT_PIXELS_PER_MM              20

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This structure describes one maXTouch object */

struct mxt_object_s
{
  uint8_t type;                        /* Object type */
  uint8_t addr[2];                     /* Start address */
  uint8_t size;                        /* Size of each instance - 1 */
  uint8_t ninstances;                  /* Number of instances - 1 */
  uint8_t nids;                        /* Number of report IDs */
};
#define MXT_OBJECT_SIZE 6

/* This structure describes one maXTouch message */

struct mxt_msg_s
{
  uint8_t id;                          /* Report ID */
  uint8_t body[7];                     /* Message body */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_INPUT_MXT_H */
