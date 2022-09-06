/****************************************************************************
 * drivers/audio/cs43l22.h
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

/* Reference:
 *   "CS43L22 Ultra Low Power CODEC for Portable Audio Applications, Pre-
 *    Production", September 2012, Rev 3.3, Wolfson Microelectronics
 */

#ifndef __DRIVERS_AUDIO_CS43L22_H
#define __DRIVERS_AUDIO_CS43L22_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <pthread.h>
#include <mqueue.h>

#include <nuttx/wqueue.h>
#include <nuttx/fs/ioctl.h>

#define getreg32(a)      (*(volatile uint32_t *)(a))
#define putreg32(v,a)    (*(volatile uint32_t *)(a) = (v))
#define getreg16(a)      (*(volatile uint16_t *)(a))
#define putreg16(v,a)    (*(volatile uint16_t *)(a) = (v))

#ifdef CONFIG_AUDIO

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* So far, I have not been able to get FLL lock interrupts. Worse, I have
 * been able to get the FLL to claim that it is locked at all even when
 * polling.  What am I doing wrong?
 *
 * Hmmm.. seems unnecessary anyway
 */

#undef CS43L22_USE_FFLOCK_INT
#undef CS43L22_USE_FFLOCK_POLL

/* Registers Addresses ******************************************************/

#define CS43L22_ID_REV             0x01 /* Chip I.D. and Revision */
#define CS43L22_POWER_CTRL1        0x02 /* Power Control 1 */
#define CS43L22_POWER_CTRL2        0x04 /* Power Control 2 */
#define CS43L22_CLOCK_CTRL         0x05 /* Clocking Control */
#define CS43L22_INTERFACE_CTRL1    0x06 /* Interface Control 1 */
#define CS43L22_INTERFACE_CTRL2    0x07 /* Interface Control 2 */
#define CS43L22_PASS_SEL_A         0x08 /* Passthrough x Select: PassA */
#define CS43L22_PASS_SEL_B         0x09 /* Passthrough x Select: PassB */
#define CS43L22_ANLG_ZC_SR_SEL     0x0A /* Analog ZC and SR Settings */
#define CS43L22_PASS_GANG_CTRL     0x0C /* Passthrough Gang Control */
#define CS43L22_PLAYBACK_CTRL1     0x0D /* Playback Control 1 */
#define CS43L22_MISCLLNS_CTRL      0x0E /* Miscellaneous Controls */
#define CS43L22_PLAYBACK_CTRL2     0x0F /* Playback Control 2 */
#define CS43L22_PASS_VOL_A         0x14 /* Passthrough x Volume: PASSAVOL */
#define CS43L22_PASS_VOL_B         0x15 /* Passthrough x Volume: PASSBVOL */
#define CS43L22_PCM_VOL_A          0x1A /* PCMx Volume: PCMA */
#define CS43L22_PCM_VOL_B          0x1B /* PCMx Volume: PCMB */
#define CS43L22_BP_FREQ_ON_TIME    0x1C /* Beep Frequency & On Time */
#define CS43L22_BP_VOL_OFF_TIME    0x1D /* Beep Volume & Off Time */
#define CS43L22_BP_TONE_CFG        0x1E /* Beep & Tone Configuration */
#define CS43L22_TONE_CTRL          0x1F /* Tone Control */
#define CS43L22_MS_VOL_CTRL_A      0x20 /* Master Volume Control: MSTA */
#define CS43L22_MS_VOL_CTRL_B      0x21 /* Master Volume Control: MSTB */
#define CS43L22_HP_VOL_CTRL_A      0x22 /* Headphone Volume Control: HPA */
#define CS43L22_HP_VOL_CTRL_B      0x23 /* Headphone Volume Control: HPB */
#define CS43L22_SPK_VOL_CTRL_A     0x24 /* Speaker Volume Control: SPKA */
#define CS43L22_SPK_VOL_CTRL_B     0x25 /* Speaker Volume Control: SPKB */
#define CS43L22_PCM_CH_SWAP        0x26 /* PCM Channel Swap */
#define CS43L22_LIM_CTRL1          0x27 /* Limiter Control 1, Min/Max Thresholds */
#define CS43L22_LIM_CTRL2          0x28 /* Limiter Control 2, Release Rate */
#define CS43L22_LIM_ATTACK_RATE    0x29 /* Limiter Attack Rate */
#define CS43L22_STATUS             0x2E /* Status */
#define CS43L22_BAT_COMP           0x2F /* Battery Compensation */
#define CS43L22_VP_BAT_LEVEL       0x30 /* VP Battery Level */
#define CS43L22_SPK_STATUS         0x31 /* Speaker Status */
#define CS43L22_TEMP_MON_CTRL      0x32 /* Temperature Monitor Control */
#define CS43L22_THERMAL_FOLDBACK   0x33 /* Thermal Foldback */
#define CS43L22_CHRG_PUMP_FREQ     0x34 /* Charge Pump Frequency */

#define CS43L22_HPBMUTE            (1 << 7)
#define CS43L22_HPAMUTE            (1 << 6)
#define CS43L22_SPKBMUTE           (1 << 5)
#define CS43L22_SPKAMUTE           (1 << 4)

/* Register Default Values **************************************************/

/* Registers have some undocumented bits set on power up.
 * These probably should be retained on writes (?).
 */

#define CS43L22_ID_REV_DEFAULT             0xe3 /* Chip I.D. and Revision */
#define CS43L22_POWER_CTRL1_DEFAULT        0x01 /* Power Control 1 */
#define CS43L22_POWER_CTRL2_DEFAULT        0x05 /* Power Control 2 */
#define CS43L22_CLOCK_CTRL_DEFAULT         0xa0 /* Clocking Control */
#define CS43L22_INTERFACE_CTRL1_DEFAULT    0x00 /* Interface Control 1 */
#define CS43L22_INTERFACE_CTRL2_DEFAULT    0x00 /* Interface Control 2 */
#define CS43L22_PASS_SEL_A_DEFAULT         0x81 /* Passthrough x Select: PassA */
#define CS43L22_PASS_SEL_B_DEFAULT         0x81 /* Passthrough x Select: PassB */
#define CS43L22_ANLG_ZC_SR_SEL_DEFAULT     0xa5 /* Analog ZC and SR Settings */
#define CS43L22_PASS_GANG_CTRL_DEFAULT     0x00 /* Passthrough Gang Control */
#define CS43L22_PLAYBACK_CTRL1_DEFAULT     0x60 /* Playback Control 1 */
#define CS43L22_MISCLLNS_CTRL_DEFAULT      0x02 /* Miscellaneous Controls */
#define CS43L22_PLAYBACK_CTRL2_DEFAULT     0x00 /* Playback Control 2 */
#define CS43L22_PASS_VOL_A_DEFAULT         0x00 /* Passthrough x Volume: PASSAVOL */
#define CS43L22_PASS_VOL_B_DEFAULT         0x00 /* Passthrough x Volume: PASSBVOL */
#define CS43L22_PCM_VOL_A_DEFAULT          0x00 /* PCMx Volume: PCMA */
#define CS43L22_PCM_VOL_B_DEFAULT          0x00 /* PCMx Volume: PCMB */
#define CS43L22_BP_FREQ_ON_TIME_DEFAULT    0x00 /* Beep Frequency & On Time */
#define CS43L22_BP_VOL_OFF_TIME_DEFAULT    0x00 /* Beep Volume & Off Time */
#define CS43L22_BP_TONE_CFG_DEFAULT        0x00 /* Beep & Tone Configuration */
#define CS43L22_TONE_CTRL_DEFAULT          0x88 /* Tone Control */
#define CS43L22_MS_VOL_CTRL_A_DEFAULT      0x00 /* Master Volume Control: MSTA */
#define CS43L22_MS_VOL_CTRL_B_DEFAULT      0x00 /* Master Volume Control: MSTB */
#define CS43L22_HP_VOL_CTRL_A_DEFAULT      0x00 /* Headphone Volume Control: HPA */
#define CS43L22_HP_VOL_CTRL_B_DEFAULT      0x00 /* Headphone Volume Control: HPB */
#define CS43L22_SPK_VOL_CTRL_A_DEFAULT     0x00 /* Speaker Volume Control: SPKA */
#define CS43L22_SPK_VOL_CTRL_B_DEFAULT     0x00 /* Speaker Volume Control: SPKB */
#define CS43L22_PCM_CH_SWAP_DEFAULT        0x00 /* PCM Channel Swap */
#define CS43L22_LIM_CTRL1_DEFAULT          0x00 /* Limiter Control 1, Min/Max Thresholds */
#define CS43L22_LIM_CTRL2_DEFAULT          0x7f /* Limiter Control 2, Release Rate */
#define CS43L22_LIM_ATTACK_RATE_DEFAULT    0xc0 /* Limiter Attack Rate */
#define CS43L22_STATUS_DEFAULT             0x00 /* Status */
#define CS43L22_BAT_COMP_DEFAULT           0x00 /* Battery Compensation */
#define CS43L22_VP_BAT_LEVEL_DEFAULT       0x00 /* VP Battery Level */
#define CS43L22_SPK_STATUS_DEFAULT         0x00 /* Speaker Status */
#define CS43L22_TEMP_MON_CTRL_DEFAULT      0x3b /* Temperature Monitor Control */
#define CS43L22_THERMAL_FOLDBACK_DEFAULT   0x00 /* Thermal Foldback */
#define CS43L22_CHRG_PUMP_FREQ_DEFAULT     0x5f /* Charge Pump Frequency */

/* Register Bit Definitions *************************************************/

/* 0x01 Chip I.D. and Revision (Read Only) */
#define CS43L22_DEV_ID_REV            (0xe3)
#define CS43L22_ID_SHIFT              (3)
#define CS43L22_ID_MASK               (0x1f << CS43L22_ID_SHIFT)
#define CS43L22_REV_SHIFT             (0)
#define CS43L22_REV_MASK              (0x07 << CS43L22_REV_SHIFT)

/* 0x02 Power Control 1 */
#define CS43L22_POWER_DOWN            (0x01) /* Powered Down */
#define CS43L22_POWER_UP              (0x9e) /* Powered Up */

/* 0x04 Power Control 2 */
#define CS43L22_PDN_HPB_SHIFT         (6)                          /* Bits 6-7: Headphone channel B Control */
#define CS43L22_PDN_HPB_ON_HW_PIN_LO  (0 << CS43L22_PDN_HPB_SHIFT) /* PDN_HPx[1:0] 00 Headphone channel is ON when the SPK/HP_SW pin, 6, is LO
                                                                    *                 Headphone channel is OFF when the SPK/HP_SW pin, 6, is HI
                                                                    */
#define CS43L22_PDN_HPB_ON_HW_PIN_HI  (1 << CS43L22_PDN_HPB_SHIFT) /* PDN_HPx[1:0] 01 Headphone channel is ON when the SPK/HP_SW pin, 6, is HI
                                                                    *                 Headphone channel is OFF when the SPK/HP_SW pin, 6, is LO
                                                                    */
#define CS43L22_PDN_HPB_ON            (2 << CS43L22_PDN_HPB_SHIFT) /* PDN_HPx[1:0] 10 Headphone channel is always ON */
#define CS43L22_PDN_HPB_OFF           (3 << CS43L22_PDN_HPB_SHIFT) /* PDN_HPx[1:0] 11 Headphone channel is always OFF */

#define CS43L22_PDN_HPA_SHIFT         (4)                          /* Bits 4-5: Headphone channel A Control */
#define CS43L22_PDN_HPA_ON_HW_PIN_LO  (0 << CS43L22_PDN_HPA_SHIFT) /* PDN_HPx[1:0] 00 Headphone channel is ON when the SPK/HP_SW pin, 6, is LO
                                                                    *                 Headphone channel is OFF when the SPK/HP_SW pin, 6, is HI */
#define CS43L22_PDN_HPA_ON_HW_PIN_HI  (1 << CS43L22_PDN_HPA_SHIFT) /* PDN_HPx[1:0] 01 Headphone channel is ON when the SPK/HP_SW pin, 6, is HI
                                                                    *                 Headphone channel is OFF when the SPK/HP_SW pin, 6, is LO
                                                                    */
#define CS43L22_PDN_HPA_ON            (2 << CS43L22_PDN_HPA_SHIFT) /* PDN_HPx[1:0] 10 Headphone channel is always ON */
#define CS43L22_PDN_HPA_OFF           (3 << CS43L22_PDN_HPA_SHIFT) /* PDN_HPx[1:0] 11 Headphone channel is always OFF */

#define CS43L22_PDN_SPKB_SHIFT        (2)                           /* Bits 2-3: Speaker channel B Control */
#define CS43L22_PDN_SPKB_ON_HW_PIN_LO (0 << CS43L22_PDN_SPKB_SHIFT) /* PDN_HPx[1:0] 00 Speaker channel is ON when the SPK/HP_SW pin, 6, is LO
                                                                     *                 Speaker channel is OFF when the SPK/HP_SW pin, 6, is HI
                                                                     */

#define CS43L22_PDN_SPKB_ON_HW_PIN_HI (1 << CS43L22_PDN_SPKB_SHIFT) /* PDN_HPx[1:0] 01 Speaker channel is ON when the SPK/HP_SW pin, 6, is HI
                                                                     *                 Speaker channel is OFF when the SPK/HP_SW pin, 6, is LO
                                                                     */
#define CS43L22_PDN_SPKB_ON           (2 << CS43L22_PDN_SPKB_SHIFT) /* PDN_HPx[1:0] 10 Speaker channel is always ON */
#define CS43L22_PDN_SPKB_OFF          (3 << CS43L22_PDN_SPKB_SHIFT) /* PDN_HPx[1:0] 11 Speaker channel is always OFF */

#define CS43L22_PDN_SPKA_SHIFT        (0)                           /* Bits 0-1: Speaker channel A Control */
#define CS43L22_PDN_SPKA_ON_HW_PIN_LO (0 << CS43L22_PDN_SPKA_SHIFT) /* PDN_HPx[1:0] 00 Speaker channel is ON when the SPK/HP_SW pin, 6, is LO
                                                                     *                 Speaker channel is OFF when the SPK/HP_SW pin, 6, is HI
                                                                     */
#define CS43L22_PDN_SPKA_ON_HW_PIN_HI (1 << CS43L22_PDN_SPKA_SHIFT) /* PDN_HPx[1:0] 01 Speaker channel is ON when the SPK/HP_SW pin, 6, is HI
                                                                     *                 Speaker channel is OFF when the SPK/HP_SW pin, 6, is LO
                                                                     */
#define CS43L22_PDN_SPKA_ON           (2 << CS43L22_PDN_SPKA_SHIFT) /* PDN_HPx[1:0] 10 Speaker channel is always ON */
#define CS43L22_PDN_SPKA_OFF          (3 << CS43L22_PDN_SPKA_SHIFT) /* PDN_HPx[1:0] 11 Speaker channel is always OFF */

/* 0x05 Clocking Control */
#define CS43L22_AUTO_DETECT_ENABLE    (1 << 7)                      /* Auto-detection of speed mode enable */

#define CS43L22_SPEED_SHIFT           (5)                           /* Bits 5-6: Speed mode */
#define CS43L22_SPEED_DOUBLE          (0 << CS43L22_SPEED_SHIFT)    /* Slave: Double-Speed Mode (DSM - 50 kHz -100 kHz Fs) Master: MCLK=512 SCLK=64*/
#define CS43L22_SPEED_SINGLE          (1 << CS43L22_SPEED_SHIFT)    /* Slave: Single-Speed Mode (SSM - 4 kHz -50 kHz Fs)   Master: MCLK=256 SCLK=64*/
#define CS43L22_SPEED_HALF            (2 << CS43L22_SPEED_SHIFT)    /* Slave: Half-Speed Mode (HSM - 12.5kHz -25 kHz Fs)   Master: MCLK=128 SCLK=64*/
#define CS43L22_SPEED_QUARTER         (3 << CS43L22_SPEED_SHIFT)    /* Slave: Quarter-Speed Mode (QSM - 4 kHz -12.5 kHz Fs)Master: MCLK=128 SCLK=64*/

#define CS43L22_32k_GROUP_ENABLE      (1 << 4)                      /* Bit 4: Specifies whether or not the input/output sample rate is 8 kHz, 16 kHz or 32 kHz */

#define CS43L22_VIDEOCLK_ENABLE       (1 << 3)                      /* Bit 3: Specifies whether or not the external MCLK frequency is 27 MHz */

#define CS43L22_MCLK_LRCK_RATIO_SHIFT (1)                           /* Bits 1-2: Internal MCLK/LRCK Ratio */

#define CS43L22_RATIO_128_64          (0 << CS43L22_MCLK_LRCK_RATIO_SHIFT)    /* RATIO[1:0] Internal MCLK Cycles per LRCK=128, SCLK/LRCK=64 Ratio in Master Mode */
#define CS43L22_RATIO_125_62          (1 << CS43L22_MCLK_LRCK_RATIO_SHIFT)    /* RATIO[1:0] Internal MCLK Cycles per LRCK=125, SCLK/LRCK=62 Ratio in Master Mode */
#define CS43L22_RATIO_132_66          (2 << CS43L22_MCLK_LRCK_RATIO_SHIFT)    /* RATIO[1:0] Internal MCLK Cycles per LRCK=132, SCLK/LRCK=66 Ratio in Master Mode */
#define CS43L22_RATIO_136_68          (3 << CS43L22_MCLK_LRCK_RATIO_SHIFT)    /* RATIO[1:0] Internal MCLK Cycles per LRCK=136, SCLK/LRCK=68 Ratio in Master Mode */

#define CS43L22_CLKDIV2_ENABLE        (1 << 0)                      /* Bit 0: Divided by 2 */

/* 0x06 Interface Control 1 */
#define CS43L22_MODE_MASTER           (1 << 7)                      /* Configures the serial port I/O clocking */

#define CS43L22_SCLK_POLARITY_INVERT  (1 << 6)                      /* Configures the polarity of the SCLK signal */

#define CS43L22_DSP_MODE_ENABLE       (1 << 4)                      /* Configures a data-packed interface format for the DAC */

#define CS43L22_DAC_IF_FORMAT_SHIFT   (2)                                   /* Bits 2-3: Configures the digital interface format for data on SDIN */
#define CS43L22_DAC_IF_LEFT_JUSTIFIED (0 << CS43L22_DAC_IF_FORMAT_SHIFT)    /* DACDIF[1:0] Left Justified, up to 24-bit data */
#define CS43L22_DAC_IF_I2S            (1 << CS43L22_DAC_IF_FORMAT_SHIFT)    /* DACDIF[1:0] I2S, up to 24-bit data */
#define CS43L22_DAC_IF_RIGHT_JUSTIFIED (2 << CS43L22_DAC_IF_FORMAT_SHIFT)   /* DACDIF[1:0] Right Justified */
#define CS43L22_DAC_IF_RESERVED       (3 << CS43L22_DAC_IF_FORMAT_SHIFT)    /* DACDIF[1:0] Reserved */

#define CS43L22_AUDIO_WORD_LENGHT_SHIFT (0)                                  /* Bits 0-1: Configures the audio sample word length used for the data into SDIN */
#define CS43L22_AWL_DSP_32_RJ_24      (0 << CS43L22_AUDIO_WORD_LENGHT_SHIFT) /* AWL[1:0] DSP Mode: 32-bit data, Right Justified: 24-bit data */
#define CS43L22_AWL_DSP_24_RJ_20      (1 << CS43L22_AUDIO_WORD_LENGHT_SHIFT) /* AWL[1:0] DSP Mode: 24-bit data, Right Justified: 20-bit data */
#define CS43L22_AWL_DSP_20_RJ_18      (2 << CS43L22_AUDIO_WORD_LENGHT_SHIFT) /* AWL[1:0] DSP Mode: 20-bit data, Right Justified: 18-bit data */
#define CS43L22_AWL_DSP_16_RJ_16      (3 << CS43L22_AUDIO_WORD_LENGHT_SHIFT) /* AWL[1:0] DSP Mode: 16 bit data, Right Justified: 16-bit data */

/* 0x0E Miscellaneous Controls */
#define CS43L22_FREEZE                (1 << 3)                      /* Configures a hold on all register settings */
#define CS43L22_DEEMPHASIS_ENABLE     (1 << 2)                      /* Configures a 15μs/50μs digital de-emphasis filter response on the headphone/line and speaker outputs */

/* 0x1F Tone Control */
#define CS43L22_TREB_GAIN_SHIFT       (4)                           /* Sets the gain of the treble shelving filter */
#define CS43L22_TREB_GAIN(a)          ((a) << CS43L22_TREB_GAIN_SHIFT)
                                                                    /* TREB[3:0] Gain Setting: */
                                                                    /* 0000 +12.0 dB */
                                                                    /* ··· ··· */
                                                                    /* 0111 +1.5 dB */
                                                                    /* 1000 0 dB */
                                                                    /* 1001 -1.5 dB */
                                                                    /* 1111 -10.5 dB */
                                                                    /* Step Size: 1.5 dB */

#define CS43L22_BASS_GAIN_SHIFT       (0)                           /* Sets the gain of the bass shelving filter */
#define CS43L22_BASS_GAIN(a)          ((a) << CS43L22_BASS_GAIN_SHIFT)
                                                                    /* BASS[3:0] Gain Setting: */
                                                                    /* 0000 +12.0 dB */
                                                                    /* ··· ··· */
                                                                    /* 0111 +1.5 dB */
                                                                    /* 1000 0 dB */
                                                                    /* 1001 -1.5 dB */
                                                                    /* 1111 -10.5 dB */
                                                                    /* Step Size: 1.5 dB */

/* FLL Configuration ********************************************************/

/* Default FLL configuration */

#define CS43L22_DEFAULT_SAMPRATE      11025     /* Initial sample rate */
#define CS43L22_DEFAULT_NCHANNELS     1         /* Initial number of channels */
#define CS43L22_DEFAULT_BPSAMP        16        /* Initial bits per sample */

#define CS43L22_NFLLRATIO             5         /* Number of FLL_RATIO values */

#define CS43L22_MINOUTDIV             4         /* Minimum FLL_OUTDIV divider */
#define CS43L22_MAXOUTDIV             64        /* Maximum FLL_OUTDIV divider */

#define CS43L22_BCLK_MAXDIV           20        /* Maximum BCLK divider */

#define CS43L22_FRAMELEN8             14        /* Bits per frame for 8-bit data */
#define CS43L22_FRAMELEN16            32        /* Bits per frame for 16-bit data */

/* Commonly defined and redefined macros */

#ifndef MIN
#  define MIN(a,b)                    (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b)                    (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cs43l22_dev_s
{
  /* We are an audio lower half driver (We are also the upper "half" of
   * the CS43L22 driver with respect to the board lower half driver).
   *
   * Terminology:
   * Our "lower" half audio instances will be called dev for the publicly
   * visible version and "priv" for the version that only this driver
   * knows.  From the point of view of this driver, it is the board lower
   * "half" that is referred to as "lower".
   */

  struct audio_lowerhalf_s dev;             /* CS43L22 audio lower half (this device) */

  /* Our specific driver data goes here */

  FAR const struct cs43l22_lower_s *lower;  /* Pointer to the board lower functions */
  FAR struct i2c_master_s *i2c;             /* I2C driver to use */
  FAR struct i2s_dev_s   *i2s;              /* I2S driver to use */
  struct dq_queue_s       pendq;            /* Queue of pending buffers to be sent */
  struct dq_queue_s       doneq;            /* Queue of sent buffers to be returned */
  struct file             mq;               /* Message queue for receiving messages */
  char                    mqname[16];       /* Our message queue name */
  pthread_t               threadid;         /* ID of our thread */
  uint32_t                bitrate;          /* Actual programmed bit rate */
  mutex_t                 pendlock;         /* Protect pendq */
#ifdef CS43L22_USE_FFLOCK_INT
  struct work_s           work;             /* Interrupt work */
#endif
  uint16_t                samprate;         /* Configured samprate (samples/sec) */
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  uint16_t                balance;          /* Current balance level (b16) */
#endif /* CONFIG_AUDIO_EXCLUDE_BALANCE */
  uint8_t                 volume;           /* Current volume level {0..63} */
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */
  uint8_t                 nchannels;        /* Number of channels (1 or 2) */
  uint8_t                 bpsamp;           /* Bits per sample (8 or 16) */
  volatile uint8_t        inflight;         /* Number of audio buffers in-flight */
#ifdef CS43L22_USE_FFLOCK_INT
  volatile bool           locked;           /* FLL is locked */
#endif
  bool                    running;          /* True: Worker thread is running */
  bool                    paused;           /* True: Playing is paused */
  bool                    mute;             /* True: Output is muted */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  bool                    terminating;      /* True: Stop requested */
#endif
  bool                    reserved;         /* True: Device is reserved */
  volatile int            result;           /* The result of the last transfer */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_CS43L22_CLKDEBUG
extern const uint8_t g_sysclk_scaleb1[CS43L22_BCLK_MAXDIV + 1];
extern const uint8_t g_fllratio[CS43L22_NFLLRATIO];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cs43l22_readreg
 *
 * Description:
 *    Read the specified 8-bit register from the CS43L22 device.
 *
 ****************************************************************************/

#if defined(CONFIG_CS43L22_REGDUMP) || defined(CONFIG_CS43L22_CLKDEBUG)
struct cs43l22_dev_s;
uint8_t cs43l22_readreg(FAR struct cs43l22_dev_s *priv, uint8_t regaddr);
#endif

#endif /* CONFIG_AUDIO */
#endif /* __DRIVERS_AUDIO_CS43L22_H */
