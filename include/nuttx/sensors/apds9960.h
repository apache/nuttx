/********************************************************************************************
 * drivers/sensors/apds9960.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_APDS9960_H
#define __INCLUDE_NUTTX_SENSORS_APDS9960_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_SENSORS_APDS9960)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default APDS-9960 Identification value */

#define APDS9960_ID_VAL        0xab

/* Device I2C Address */

#define APDS9960_I2C_ADDR      0x39

/* Registers mapping */

#define APDS9960_ENABLE        0x80 /* Power ON, Prox./ALS/Gest (Int.) Enable */
#define APDS9960_ATIME         0x82 /* ALS ADC Integration Time */
#define APDS9960_WTIME         0x83 /* Wait Time */
#define APDS9960_AILTL         0x84 /* ALS Low Threshold, lower byte */
#define APDS9960_AILTH         0x85 /* ALS Low Threshold, higher byte */
#define APDS9960_AIHTL         0x86 /* ALS High Threshold, lower byte */
#define APDS9960_AIHTH         0x87 /* ALS High Threshold, higher byte */
#define APDS9960_PILT          0x89 /* Proximity Low Threshold */
#define APDS9960_PIHT          0x8b /* Proximity High Threshold */
#define APDS9960_PERS          0x8c /* Proximity Persistence */
#define APDS9960_CONFIG1       0x8d /* Wait Long Enable */
#define APDS9960_PPULSE        0x8e /* Proximity Pulse */
#define APDS9960_CONTROL       0x8f /* Gain and LED control */
#define APDS9960_CONFIG2       0x90 /* Saturation interrupt and LED Boost config. */
#define APDS9960_ID            0x92 /* APDS-9960 Identification register */
#define APDS9960_STATUS        0x93 /* Proximity Saturation/Inter/Valid status */
#define APDS9960_CDATAL        0x94 /* Clear Data, Low byte */
#define APDS9960_CDATAH        0x95 /* Clear Data, High byte */
#define APDS9960_RDATAL        0x96 /* Red Data, Low byte */
#define APDS9960_RDATAH        0x97 /* Red Data, High byte */
#define APDS9960_GDATAL        0x98 /* Green Data, Low byte */
#define APDS9960_GDATAH        0x99 /* Green Data, High byte */
#define APDS9960_BDATAL        0x9a /* Blue Data, Low byte */
#define APDS9960_BDATAH        0x9b /* Blue Data, High byte */
#define APDS9960_PDATA         0x9c /* Proximity data */
#define APDS9960_POFFSET_UR    0x9d /* Prox. offset UP/RIGHT */
#define APDS9960_POFFSET_DL    0x9e /* Prox. offset DOWN/LEFT */
#define APDS9960_CONFIG3       0x9f /* Prox. gain compensation U/D/L/R Mask config */
#define APDS9960_GPENTH        0xa0 /* Gesture Prox. Entry Threshold */
#define APDS9960_GEXTH         0xa1 /* Gesture Exit Threshold */
#define APDS9960_GCONFIG1      0xa2 /* Gesture Config1 register */
#define APDS9960_GCONFIG2      0xa3 /* Gesture Config2 register */
#define APDS9960_GOFFSET_U     0xa4 /* Gesture Offset Up */
#define APDS9960_GOFFSET_D     0xa5 /* Gesture Offset Down */
#define APDS9960_GPULSE        0xa6 /* Gesture Pulse config */
#define APDS9960_GOFFSET_L     0xa7 /* Gesture Offset Left */
#define APDS9960_GOFFSET_R     0xa9 /* Gesture Offset Up */
#define APDS9960_GCONFIG3      0xaa /* Gesture Config3 register */
#define APDS9960_GCONFIG4      0xab /* Gesture Config4 register */
#define APDS9960_GFLVL         0xae /* Gesture FIFO level */
#define APDS9960_GSTATUS       0xaf /* Gesture Status */
#define APDS9960_PICLEAR       0xe5 /* Proximity Interrupt Clear */
#define APDS9960_CICLEAR       0xe5 /* Clear Channel Interrupt Clear */
#define APDS9960_AICLEAR       0xe7 /* All Non-Gesture Interrupt Clear */
#define APDS9960_GFIFO_U       0xfc /* Gesture FIFO Data UP */
#define APDS9960_GFIFO_D       0xfd /* Gesture FIFO Data DOWN */
#define APDS9960_GFIFO_L       0xfe /* Gesture FIFO Data LEFT */
#define APDS9960_GFIFO_R       0xff /* Gesture FIFO Data RIGHT */

/* ENABLE Register */

#define PON                    (1 << 0) /* Bit 0: Power ON */
#define AEN                    (1 << 1) /* Bit 1: ALS Enable ??? */
#define PEN                    (1 << 2) /* Bit 2: Prox. Enable */
#define WEN                    (1 << 3) /* Bit 3: Wait Enable */
#define AIEN                   (1 << 4) /* Bit 4: ALS Int. Enable */
#define PIEN                   (1 << 5) /* Bit 5: Prox. Int. Enable */
#define GEN                    (1 << 6) /* Bit 6: Gesture Enable */

/* PERS Register */

#define APERS_SHIFT            0        /* Bits 0-3: ALS Int. Persistence */
#define APERS_MASK             (0xf << PERS_APERS_SHIFT)
#define PPERS_SHIFT            4        /* Bits 4-7: Prox. Int. Persistence */
#define PPERS_MASK             (0xf << PERS_APERS_SHIFT)

/* CONFIG1 Register */

#define LOWPOW                 (1 << 0) /* Bit 0: Low Power (Gesture) */
#define WLONG                  (1 << 1) /* Bit 1: Wait Long Enable (ALS) */

/* PPULSE Register */

#define PPULSE_SHIFT           0        /* Bits 0-5: Prox. Pulse Count (1 up to 64 pulses) */
#define PPULSE_MASK            (0x3f << PPULSE_SHIFT)
#  define PPULSE_NUM(n)        ((n-1) << PPULSE_SHIFT)
#define PPLEN_SHIFT            6        /* Bits 6-7: Prox. Pulse Lenght */
#define PPLEN_MASK             (3 << PPLEN_SHIFT)
#  define PPLEN_4US            (0 << PPLEN_SHIFT)
#  define PPLEN_8US            (1 << PPLEN_SHIFT)
#  define PPLEN_16US           (2 << PPLEN_SHIFT)
#  define PPLEN_32US           (3 << PPLEN_SHIFT)

/* CONTROL Register */

#define AGAIN_SHIFT            0        /* Bits 0-1: ALS Gain Control */
#define AGAIN_MASK             (3 << AGAIN_SHIFT)
#  define AGAIN_1X             (0 << AGAIN_SHIFT)
#  define AGAIN_2X             (1 << AGAIN_SHIFT)
#  define AGAIN_4X             (2 << AGAIN_SHIFT)
#  define AGAIN_8X             (3 << AGAIN_SHIFT)
#define PGAIN_SHIFT            2        /* Bits 2-3: Proximity Gain Control */
#define PGAIN_MASK             (3 << PGAIN_SHIFT)
#  define PGAIN_1X             (0 << PGAIN_SHIFT)
#  define PGAIN_2X             (1 << PGAIN_SHIFT)
#  define PGAIN_4X             (2 << PGAIN_SHIFT)
#  define PGAIN_8X             (3 << PGAIN_SHIFT)
#define LDRIVE_SHIFT           6        /* Bits 6-7: LED Drive Strength */
#define LDRIVE_MASK            (3 << LDRIVE_SHIFT)
#  define LDRIVE_100MA         (0 << LDRIVE_SHIFT)
#  define LDRIVE_50MA          (1 << LDRIVE_SHIFT)
#  define LDRIVE_25MA          (2 << LDRIVE_SHIFT)
#  define LDRIVE_12p5MA        (3 << LDRIVE_SHIFT)

/* CONFIG2 Register */

#define LEDBOOST_SHIFT         4        /* Bits 4-5: Proximity/Gesture LED Boost */
#define LEDBOOST_MASK          (3 << LEDBOOST_SHIFT)
#  define LEDBOOST_100         (0 << LEDBOOST_SHIFT) /* Boost LED 100% */
#  define LEDBOOST_150         (1 << LEDBOOST_SHIFT) /* Boost LED 150% */
#  define LEDBOOST_200         (2 << LEDBOOST_SHIFT) /* Boost LED 200% */
#  define LEDBOOST_300         (3 << LEDBOOST_SHIFT) /* Boost LED 300% */
#define CPSIEN                 (1 << 6) /* Bit 6: Clear diode Saturation Interrupt Enable */
#define PSIEN                  (1 << 7) /* Bit 7: Proximity Saturation Interrupt Enable */

/* STATUS Regiser */

#define AVALID                 (1 << 0) /* Bit 0: ALS Valid */
#define PVALID                 (1 << 1) /* Bit 1: Proximity Valid */
#define AINT                   (1 << 4) /* Bit 4: ALS Interrupt */
#define PINT                   (1 << 5) /* Bit 5: Proximity Interrupt */
#define PGSAT                  (1 << 6) /* Bit 6: Proximity Saturation */
#define CPSAT                  (1 << 7) /* Bit 7: ALS Interrupt */

/* CONFIG3 Register */

#define PMSK_R                 (1 << 0) /* Bit 0: Proximity Mask RIGHT Enable */
#define PMSK_L                 (1 << 1) /* Bit 1: Proximity Mask LEFT Enable */
#define PMSK_D                 (1 << 2) /* Bit 2: Proximity Mask DOWN Enable */
#define PMSK_U                 (1 << 3) /* Bit 3: Proximity Mask UP Enable */
#define PCMP                   (1 << 5) /* Bit 5: Proximity Gain Compensation Enable */

/* GCONFIG1 Register */

#define GEXPERS_SHIFT          0        /* Bits 0-1: Gesture Exit Persistence */
#define GEXPERS_MASK           (3 << GEXPERS_SHIFT)
#  define GEXPERS_1ST          (0 << GEXPERS_SHIFT) /* 1st 'gesture end' exits */
#  define GEXPERS_2ND          (1 << GEXPERS_SHIFT) /* 2nd 'gesture end' exits */
#  define GEXPERS_4TH          (2 << GEXPERS_SHIFT) /* 4th 'gesture end' exits */
#  define GEXPERS_7TH          (3 << GEXPERS_SHIFT) /* 7th 'gesture end' exits */
#define GEXMSK_SHIFT           2        /* Bits 2-5: Gesture Exit Mask */
#define GEXMSK_MASK            (0xf << GEXMSK_SHIFT)
#define GFIFOTH_SHIFT          6        /* Bits 6-7: Gesture FIFO Threshold */
#define GFIFOTH_MASK           (3 << GFIFOTH_SHIFT)
#  define GFIFOTH_1DS          (0 << GFIFOTH_SHIFT) /* Interrupt after 1 dataset */
#  define GFIFOTH_4DS          (1 << GFIFOTH_SHIFT) /* Interrupt after 4 datasets */
#  define GFIFOTH_8DS          (2 << GFIFOTH_SHIFT) /* Interrupt after 8 datasets */
#  define GFIFOTH_16DS         (3 << GFIFOTH_SHIFT) /* Interrupt after 8 datasets */

/* GCONFIG2 Register */

#define GWTIME_SHIFT           0        /* Bits 0-2: Gesture Wait Time */
#define GWTIME_MASK            (7 << GWTIME_SHIFT)
#  define GWTIME_0MS           (0 << GWTIME_SHIFT)
#  define GWTIME_2p8MS         (1 << GWTIME_SHIFT)
#  define GWTIME_5p6MS         (2 << GWTIME_SHIFT)
#  define GWTIME_8p4MS         (3 << GWTIME_SHIFT)
#  define GWTIME_14MS          (4 << GWTIME_SHIFT)
#  define GWTIME_22p4MS        (5 << GWTIME_SHIFT)
#  define GWTIME_30p8MS        (6 << GWTIME_SHIFT)
#  define GWTIME_39p2MS        (7 << GWTIME_SHIFT)
#define GLDRIVE_SHIFT          3        /* Bits 3-4: Gesture LED Drive Strength */
#define GLDRIVE_MASK           (3 << GLDRIVE_SHIFT)
#  define GLDRIVE_100MA        (0 << GLDRIVE_SHIFT)
#  define GLDRIVE_50MA         (1 << GLDRIVE_SHIFT)
#  define GLDRIVE_25MA         (2 << GLDRIVE_SHIFT)
#  define GLDRIVE_12p5MA       (3 << GLDRIVE_SHIFT)
#define GGAIN_SHIFT            5        /* Bits 5-6: Gesture Gain Control */
#define GGAIN_MASK             (3 << GGAIN_SHIFT)
#  define GGAIN_1X             (0 << GGAIN_SHIFT)
#  define GGAIN_2X             (1 << GGAIN_SHIFT)
#  define GGAIN_4X             (2 << GGAIN_SHIFT)
#  define GGAIN_8X             (3 << GGAIN_SHIFT)

/* GPULSE Register */

#define GPULSE_SHIFT           0        /* Bits 0-5: Pulse Count */
#define GPULSE_MASK            (0x3f << GPULSE_SHIFT)
#  define GPULSE_NUM(n)        ((n-1) << GPULSE_SHIFT)
#define GPLEN_SHIFT            6        /* Bit 6-7: Gesture Pulse Length */
#define GPLEN_MASK             (3 << GPLEN_SHIFT)
#  define GPLEN_4US            (0 << GPLEN_SHIFT)
#  define GPLEN_8US            (1 << GPLEN_SHIFT)
#  define GPLEN_16US           (2 << GPLEN_SHIFT)
#  define GPLEN_32US           (3 << GPLEN_SHIFT)

/* GCONFIG3 Register */

#define GDIMS_SHIFT            0        /* Bits 0-1: Gesture Dimension Select */
#define GDIMS_MASK             (3 << GDIMS_SHIFT)

/* GCONFIG4 Register */

#define GMODE                  (1 << 0) /* Bit 0: Gesture Mode */
#define GIEN                   (1 << 1) /* Bit 1: Gesture Interrupt Enable */

/* GSTATUS Register */

#define GVALID                 (1 << 0) /* Bit 0: Gesture Valid */
#define GFOV                   (1 << 1) /* Bit 1: Gesture FIFO Overflow */

/* Default values for device initialization */

#define DEFAULT_ATIME          219      /* 103ms */
#define DEFAULT_WTIME          246      /* 27ms  */
#define DEFAULT_PPULSE         (PPLEN_16US | PPULSE_NUM(8)) /* 16us and 8 pulses */
#define DEFAULT_POFFSET_UR     0        /* 0 offset */
#define DEFAULT_POFFSET_DL     0        /* 0 offset */
#define DEFAULT_CONFIG1        0x60     /* No 12x wait (WTIME) factor */
#define DEFAULT_PILT           0        /* Low proximity threshold               */
#define DEFAULT_PIHT           50       /* High proximity threshold              */
#define DEFAULT_PERS           0x11     /* 2 consecutive prox or ALS for int.    */
#define DEFAULT_CONTROL        (AGAIN_4X | PGAIN_4X | LDRIVE_100MA)
#define DEFAULT_CONFIG2        0x01     /* No saturation interrupts or LED boost */
#define DEFAULT_CONFIG3        0        /* Enable all photodiodes, no SAI        */
#define DEFAULT_GPENTH         40       /* Threshold for entering gesture mode   */
#define DEFAULT_GEXTH          30       /* Threshold for exiting gesture mode    */
#define DEFAULT_GCONFIG1       GFIFOTH_8DS /* 8 gesture events for int., 1 for exit */
#define DEFAULT_GCONFIG2       (GGAIN_4X | GLDRIVE_100MA /*| GWTIME_2p8MS*/ )
#define DEFAULT_GOFFSET_U      0        /* No offset scaling for gesture mode */
#define DEFAULT_GOFFSET_D      0        /* No offset scaling for gesture mode */
#define DEFAULT_GOFFSET_L      0        /* No offset scaling for gesture mode */
#define DEFAULT_GOFFSET_R      0        /* No offset scaling for gesture mode */
#define DEFAULT_GPULSE         (GPLEN_32US | GPULSE_NUM(10)) /* 32us, 10 pulses */
#define DEFAULT_GCONFIG3       0        /* All photodiodes active during gesture */
#define DEFAULT_GCONFIG4       0        /* Disable gesture interrupts */
#define DEFAULT_AILTL          0xFF     /* Force interrupt for calibration */
#define DEFAULT_AILTH          0xFF     /* Force interrupt for calibration */
#define DEFAULT_AIHTL          0
#define DEFAULT_AIHTH          0
#define FIFO_PAUSE_TIME        30000    /* Wait period (ms) between FIFO reads */

/* Gesture parameters */

#define GESTURE_THRESHOLD_OUT  10
#define GESTURE_SENSITIVITY_1  50
#define GESTURE_SENSITIVITY_2  20

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Direction definitions */

enum
{
  DIR_NONE,
  DIR_LEFT,
  DIR_RIGHT,
  DIR_UP,
  DIR_DOWN,
  DIR_NEAR,
  DIR_FAR,
  DIR_ALL
};

/* State definitions */

enum
{
  NA_STATE,
  NEAR_STATE,
  FAR_STATE,
  ALL_STATE
};

/* Interrupt configuration data structure */

struct apds9960_config_s
{
  CODE int (*irq_attach)(FAR struct apds9960_config_s * state, xcpt_t isr,
                         FAR void *arg);
  FAR struct i2c_master_s *i2c_dev;
  uint8_t i2c_addr;
};

/* Container for gesture data */

struct gesture_data_s
{
  uint8_t u_data[32];
  uint8_t d_data[32];
  uint8_t l_data[32];
  uint8_t r_data[32];
  uint8_t index;
  uint8_t total_gestures;
  uint8_t in_threshold;
  uint8_t out_threshold;
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: apds9960_register
 *
 * Description:
 *   Register the APDS9960 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gest0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *              APDS9960
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct i2c_master_s;
int apds9960_register(FAR const char *devpath,
                      FAR struct apds9960_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_APDS9960 */
#endif /* __INCLUDE_NUTTX_SENSORS_APDS9960_H */
