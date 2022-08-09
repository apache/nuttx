/****************************************************************************
 * drivers/sensors/gh3020/gh3020.c
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
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <math.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched_note.h>
#include <nuttx/sensors/gh3020.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/wqueue.h>
#include <sys/types.h>

#include "gh3020_bridge.h"
#include "gh3020_def.h"
#include "gh3x2x_drv.h"
#include "gh3x2x_inner.h"
#include "gh3x2x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configurations */

#define GH3020_BATCH_NUMBER      20         /* Maximum slots for each PPG */
#define GH3020_TIAGAIN_FACTEST   0          /* TIA gain = 0(10K) in factest */

/* SPI parameters */

#define GH3020_SPI_FREQ_MAX      8000000    /* Maximum SPI CLK = 8MHz */
#define GH3020_SPI_FREQ_MIN      1000000    /* Minimum SPI CLK = 1MHz */
#define GH3020_SPI_NBITS         8          /* Bit per word on SPI */

/* Misc parameters */

#define GH3020_SAMPLERATE_MAX    1000.0f    /* Maximum sample rate = 1kHz */
#define GH3020_SAMPLERATE_MIN    25.0f      /* Minimum sample rate = 25Hz */
#define GH3020_RDMODE_INTERRPT   0          /* Read data in interrupts */
#define GH3020_RDMODE_POLLING    1          /* Read data with polling */
#define GH3020_LED_DRV_SCALE_N   GH3020_LED_DRV_N_FOR_EFUSE_200MA

/* Default settings */

#define GH3020_INTVL_DFT         40000      /* Default interval = 40 ms */
#define GH3020_SR_DFT            25         /* Default sample rate = 25 Hz */
#define GH3020_CURRENT_DFT       10000      /* Default LED current = 10mA */

/* Constants */

#define GH3020_CFGLIST_NUM       2          /* Init cfg list number */
#define GH3020_ONE_SECOND        1000000.0f /* 1 second = 1000000 us */

/* Macros */

#define GH3020_CEILING(x, y)     ((x) + ((y)- 1)) / (y)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* GH3020 FIFO data information struct */

struct gh3020_fifodata_s
{
  uint32_t adccode;                      /* Sampling rawdata of ADC */
  uint8_t  slot_idx;                     /* Slot index */
  uint8_t  adc_idx;                      /* ADC index */
  uint8_t  flag_ledadj_ecgrecover;       /* Flag of PPG or recover of ECG */
  uint8_t  flag_ledadj_agc_up;           /* Adj flag of PPG 0: down 1:up */
};

/* Sensor struct */

struct gh3020_sensor_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s lower;       /* Lower half sensor driver */
  FAR struct gh3020_dev_s *dev;          /* Pointer to the device struct */
  unsigned long interval;                /* Sample interval (us) */
  unsigned long batch;                   /* Batch latency (us) */
  uint32_t current;                      /* LED driver current (uA) */
  uint8_t chidx;                         /* PPG channel index */
  bool activated;                        /* If it's activated now. */
  bool activating;                       /* If it will be activated later */
  bool inactivating;                     /* If it will be inactivated later */
  bool reset_cfg;                    /* If it will reset LED current */
};

/* Device struct */

struct gh3020_dev_s
{
  /* PPG sensors structures. */

  struct gh3020_sensor_s sensor[GH3020_SENSOR_NUM];
  uint64_t timestamp;                    /* Current timestamp (us) */
  uint64_t timestamp_last;               /* Last timestamp (us) */
  struct work_s work_intrpt;             /* Interrupt worker */
  struct work_s work_poll;               /* Polling worker */

  /* Device configuration struct pointer. */

  FAR const struct gh3020_config_s *config;

  /* Buffer for pushing PPG data. */

  struct sensor_ppgq ppgdata[GH3020_SENSOR_NUM][GH3020_BATCH_NUMBER];
  unsigned long batch;                   /* Common batch(us) for interrupts */
  unsigned long interval;                /* Common interval(us) for polling */
  uint32_t intvl_prev;                   /* Previous common interval(us) */
  uint32_t channelmode;                  /* PPG channels status mode */
  float tia_calibr;                      /* TIA calibration coefficent */
  float led_calibr;                      /* LED calibration coefficent */
  uint16_t efuse;                        /* EFUSE for TIA and LED driver */
  uint16_t fifowtm;                      /* FIFO water marker, unit is data */
  uint8_t ppgdatacnt[GH3020_SENSOR_NUM]; /* Data number of each PPG channel */
  uint8_t activated;                     /* How many sensors are activated */
  bool factest_mode;                     /* If it's in factory test mode */
  bool updating;                         /* If any sensor need updating */
  bool load_efuse;                       /* If EFUSE has been loaded */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* GH3020 SPI interface handle functions */

static int gh3020_configspi(FAR struct gh3020_dev_s *priv);
static void gh3020_spi_exchange(FAR struct gh3020_dev_s *priv,
                                FAR uint8_t *buf, uint16_t nwords);

/* Functions which won't operate GH3020 directly, such as data processions */

static uint16_t gh3020_calcu_fifowtm(FAR struct gh3020_dev_s *priv);
static void gh3020_extract_frame(FAR struct gh3020_dev_s *priv, uint8_t idx,
                                 FAR struct sensor_ppgq *pppg,
                                 FAR const struct gh3020_frameinfo_s
                                 *pframeinfo);

/* GH3020 common operation functions */

static void gh3020_push_data(FAR struct gh3020_dev_s *priv);
static void gh3020_reset_cfg(FAR struct gh3020_dev_s *priv, uint8_t chidx);
static void gh3020_restart_new_fifowtm(FAR struct gh3020_dev_s *priv,
                                       uint16_t fifowtm);
static void gh3020_switch_poll2intrpt(FAR struct gh3020_dev_s *priv);
static void gh3020_switch_intrpt2poll(FAR struct gh3020_dev_s *priv);
static void gh3020_update_sensor(FAR struct gh3020_dev_s *priv);

/* Operations for factest */

#ifdef CONFIG_FACTEST_SENSORS_GH3020
static void gh3x2x_factest_start(uint32_t channelmode, uint32_t current);
#endif

/* Sensor ops functions */

static int gh3020_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           bool enable);
static int gh3020_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us);
static int gh3020_batch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR unsigned long *latency_us);
#ifdef CONFIG_FACTEST_SENSORS_GH3020
static int gh3020_selftest(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           unsigned long arg);
#endif
static int gh3020_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, int cmd,
                          unsigned long arg);

/* Sensor interrupt/polling functions */

static int gh3020_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg);
static void gh3020_worker_intrpt(FAR void *arg);
static void gh3020_worker_poll(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Local storage for device struct pointer. */

FAR static struct gh3020_dev_s *g_priv;

/* The function code of each PPG channel. */

static const uint32_t gh3020_channel_function_list[GH3020_SENSOR_NUM] =
  {
    GH3X2X_FUNCTION_HR,               /* HR(slot1) for ch0 (green dynamic) */
    GH3X2X_FUNCTION_SPO2,             /* SPO2(slot2) for ch1 (red dynamic) */
    GH3X2X_FUNCTION_HRV,              /* HRV(slot3) for ch2 (IR dynamic) */
    GH3X2X_FUNCTION_HSM               /* HSM(slot0) for ch5 (IR fixed) */
  };

/* The ADC number of each PPG channel. */

static const uint8_t gh3020_adc_num_list[GH3020_SENSOR_NUM] =
  {
    GH3020_QUAD_ADC_NUM,              /* HR(slot1) for ch0 (green dynamic) */
    GH3020_SINGLE_ADC_NUM,            /* SPO2(slot3) for ch1 (red dynamic) */
    GH3020_SINGLE_ADC_NUM,            /* HRV(slot4) for ch2 (IR dynamic) */
    GH3020_SINGLE_ADC_NUM             /* HSM(slot0) for ch5 (IR fixed) */
  };

/* The instance number(node name, such ppgq0/1/2/5) of each PPG channel. */

static const uint8_t gh3020_node_name_list[GH3020_SENSOR_NUM] =
  {
    GH3020_PPG0_NODE_NAME,           /* HR(slot1) for ch0 (green dynamic) */
    GH3020_PPG1_NODE_NAME,           /* SPO2(slot3) for ch1 (red dynamic) */
    GH3020_PPG2_NODE_NAME,           /* HRV(slot4) for ch2 (IR dynamic) */
    GH3020_PPG5_NODE_NAME            /* HSM(slot0) for ch5 (IR fixed) */
  };

/* Green dynamic use 4 ADCs. R/IR dynamic use PD3-ADC0, IR fixed use PD0 */

static const uint8_t
  gh3020_adc_en_list[GH3020_SENSOR_NUM][GH3020_TOTAL_ADC_NUM] =
  {
    {1, 1, 1, 1},
    {1, 0, 0, 0},
    {1, 0, 0, 0},
    {1, 0, 0, 0}
  };

/* GH3020 ADC gain register values vs actual ADC gains (KOhm) */

static const uint16_t gh3020_gain_list[] =
  {
    10, 25, 50, 75, 100, 250, 500, 750, 1000, 1250, 1500, 1750, 2000
  };

/* Sensor operations */

static const struct sensor_ops_s g_gh3020_ops =
{
  .activate     = gh3020_activate,
  .set_interval = gh3020_set_interval,
  .batch        = gh3020_batch,
#ifdef CONFIG_FACTEST_SENSORS_GH3020
  .selftest     = gh3020_selftest,
#endif
  .control      = gh3020_control,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Initial register values for normal functions */

struct gh3020_reg_s gh3020_reglist_normal[] =
{
  {0x0004, 0x001f}, /* Maxmium sample rate = 32KHz / (31 +１) = 1000Hz */
  {0x0006, 0x0004}, /* GH3020_REG_DATA_CTRL0. Not mentioned in datasheet. */
  {0x000a, 0x00c8}, /* Default FIFO watermark = 200 */
  {0x000e, 0x000d}, /* GH3020_REG_WKUP_TMR. Not mentioned in datasheet. */
  {0x0100, 0x0100}, /* Slot0 uses slot_cfg0, slot1 uses slot_cfg1 */
  {0x0102, 0x0302}, /* Slot2 uses slot_cfg2, slot3 uses slot_cfg3 */
  {0x0104, 0x0804}, /* Slot4 uses slot_cfg4 */
  {0x010a, 0x2744}, /* Slot_cfg0(fixed IR): use FIFO, only ADC0 */
  {0x010c, 0x0001}, /* PD0 connected to TIA0 */
  {0x010e, 0x0000}, /* PDx not connected to TIAx */
  {0x0110, 0x0c13}, /* DC&BG cancel, LED drv0&1 fixed, 1st BG, 256x ADC */
  {0x0112, 0x4442}, /* TIA0 50KOhm, others default 100KOhm */
  {0x0114, 0x0002}, /* ADC0 use 200pF TIA_CF, others use default 50pF */
  {0x011e, 0x0105}, /* LED drv1 to LED1(to IR LED) pin, 4.4mA */
  {0x0120, 0x0106}, /* LED drv1 to LED1(to IR LED) pin, 5.3mA */
  {0x0126, 0x277c}, /* Slot_cfg1(tunning G): sync signal, use FIFO, 4ADCs */
  {0x012c, 0x0f14}, /* DC&BG cancel, ADC0 modifies drv0&1, 512x ADC */
  {0x0130, 0x00aa}, /* All ADCs use 200pF TIA_CF */
  {0x013a, 0x0011}, /* LED drv0 to LED0(to green LED) pin, 14.9mA */
  {0x013c, 0x0011}, /* LED drv1 to LED0(to green LED) pin, 14.99mA */
  {0x013e, 0x0a77}, /* Either LED driver current approximately 9~100mA */
  {0x0142, 0x2744}, /* Slot_cfg2(tunning R): use FIFO, only ADC0 */
  {0x0144, 0x0008}, /* PD3 connected to TIA0 */
  {0x0146, 0x0000}, /* PDx not connected to TIAx */
  {0x0148, 0x0f16}, /* DC&BG cancel, ADC0 modifies drv0&1, 2048x ADC */
  {0x014a, 0x4441}, /* TIA0 25KOhm, others default 100KOhm */
  {0x014c, 0x0002}, /* ADC0 use 200pF TIA_CF, others use default 50pF */
  {0x0156, 0x0245}, /* LED drv0 to LED2(to red LED) pin, 59.28mA */
  {0x0158, 0x0245}, /* LED drv1 to LED2(to red LED) pin, 60.72mA */
  {0x015a, 0x2e77}, /* Either LED driver current approximately 40~100 mA */
  {0x015e, 0x2744}, /* Slot_cfg3(tunning IR): use FIFO, only ADC0 */
  {0x0160, 0x0008}, /* PD3 connected to TIA0 */
  {0x0162, 0x0000}, /* PDx not connected to TIAx */
  {0x0164, 0x0f16}, /* DC&BG cancel, ADC0 modifies drv0&1, 2048xADC */
  {0x0166, 0x4441}, /* TIA0 25KOHm, others default 100KOhm */
  {0x0168, 0x0002}, /* ADC0 use 200pF TIA_CF, others use default 50pF */
  {0x0172, 0x012e}, /* LED drv0 to LED1(to IR LED) pin, 39.87mA */
  {0x0174, 0x012e}, /* LED drv1 to LED1(to IR LED) pin, 40.52mA */
  {0x0176, 0x2277}, /* Either LED driver current approximately 30~100 mA */
  {0x017a, 0xc704}, /* Slot_cfg4(Goodix ADT): doesn't use FIFO, ADC0 */
  {0x017c, 0x0001}, /* TIA0 connected to PD0, TIA disconnected */
  {0x017e, 0x0000}, /* TIA2&3 disconnected */
  {0x0180, 0x0c10}, /* BG&DC cancel, fixed LED current, 1st BG, 64x ADC */
  {0x0182, 0x4442}, /* TIA0 50KOhm, TIA1~3 100KOhm */
  {0x0184, 0x0002}, /* ADC0 uses 200pF TIA_CF, the rest use 50pF */
  {0x018a, 0x0003}, /* ADC0 BG cancel source 256uA max. */
  {0x018e, 0x0105}, /* LED drv0 to LED1(to IR LED) pin, 4.4mA */
  {0x0190, 0x0106}, /* LED drv1 to LED1(to IR LED) pin, 5.3mA */
  {0x01ec, 0x00d3}, /* Slot0's time 211 us (IR fixed) */
  {0x01ee, 0x0122}, /* Slot1's time 290 us (green dynamic) */
  {0x01f0, 0x02fa}, /* Slot2's time 762 us (red dynamic) */
  {0x01f2, 0x02fa}, /* Slot3's time 762 us (IR dynamic) */
  {0x01f4, 0x0098}, /* Slot4's time 152 us (hard wear-on detection) */
  {0x0200, 0x0120}, /* GH3020_REG_AFE_REG0. Not mentioned in datasheet. */
  {0x0280, 0x0a00}, /* Starting LED tunning need continuous 10 samples */
  {0x0282, 0xf530}, /* Tunning start upper threshold */
  {0x0284, 0x9388}, /* Tunning start lower threshold */
  {0x0286, 0xc45c}, /* Tunning done upper threshold */
  {0x0288, 0xa328}, /* Tunning done lower threshold */
  {0x0408, 0x0001}, /* Wear-on detection enabled */
  {0x0410, 0x49f0}, /* Approaching detection upper threshold LSB 16bits */
  {0x0412, 0x0082}, /* Approaching detection upper threshold MSB 8bits */
  {0x0414, 0x49f0}, /* Approaching detection lower threshold LSB 16bits */
  {0x0416, 0x0082}, /* Approaching detection lower threshold MSB 8bits */
  {0x0422, 0x0011}, /* 1 data exceeding threshold will start wear-on detect */
  {0x0424, 0x0901}, /* Wear-on2 detector confirm time 2s; wear-on1 0.4s */
  {0x0426, 0x0102}, /* Wear-on logic: OR; wear-on_as0: prox1 */
  {0x0428, 0x0102}, /* Wear-off logic: OR; wear-off_as0: prox1 */
  {0x0502, 0x4f60}, /* INT: Rst,wear-off/on,tunning fail/done,FIFO ovf/full */
  {0x0504, 0x000a}, /* Interrupt pulse width 33us * 10 */
  {0x0588, 0x0008}, /* OSC32K_TEMP. Not mentioned in datasheet. */
  {0x0692, 0x0233}, /* LED_DRV_AD_REG. Not mentioned in datasheet. */
  {0x0694, 0x0400}, /* ECG_IA_AD_REG. Not mentioned in datasheet. */
  {0x0698, 0x2310}, /* ECG_IA_AD_REG2. Not mentioned in datasheet. */
  {0x0700, 0x0000}, /* EFUSE for LED current calibration disabled */
  {0x1000, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x1002, 0x3080}, /* Virtual register. Maintained by Goodix. */
  {0x1004, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x1006, 0xf5ad}, /* Virtual register. Maintained by Goodix. */
  {0x1008, 0x2a34}, /* Virtual register. Maintained by Goodix. */
  {0x100a, 0x0001}, /* Virtual register. Maintained by Goodix. */
  {0x10e0, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x10e2, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x10e4, 0xff02}, /* Virtual register. Maintained by Goodix. */
  {0x10e6, 0xffff}, /* Virtual register. Maintained by Goodix. */
  {0x10e8, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x10ea, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x10ec, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x10ee, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x1120, 0x00c8}, /* Virtual register. Maintained by Goodix. */
  {0x1122, 0x0005}, /* Virtual register. Maintained by Goodix. */
  {0x1124, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x1160, 0x000e}, /* Virtual register. Maintained by Goodix. */
  {0x1162, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x1164, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x1166, 0x4568}, /* Virtual register. Maintained by Goodix. */
  {0x1168, 0x9940}, /* Virtual register. Maintained by Goodix. */
  {0x116a, 0x00d9}, /* Virtual register. Maintained by Goodix. */
  {0x116c, 0x8800}, /* Virtual register. Maintained by Goodix. */
  {0x116e, 0x0093}, /* Virtual register. Maintained by Goodix. */
  {0x1170, 0x5c00}, /* Virtual register. Maintained by Goodix. */
  {0x1172, 0x00c4}, /* Virtual register. Maintained by Goodix. */
  {0x1174, 0x2800}, /* Virtual register. Maintained by Goodix. */
  {0x1176, 0x00a3}, /* Virtual register. Maintained by Goodix. */
  {0x2000, 0x0001}, /* Virtual register. Maintained by Goodix. */
  {0x2002, 0x0082}, /* Virtual register. Maintained by Goodix. */
  {0x2022, 0x0004}, /* Virtual register. Maintained by Goodix. */
  {0x2024, 0x2921}, /* Virtual register. Maintained by Goodix. */
  {0x2026, 0x3931}, /* Virtual register. Maintained by Goodix. */
  {0x2044, 0x0001}, /* Virtual register. Maintained by Goodix. */
  {0x2046, 0x0062}, /* Virtual register. Maintained by Goodix. */
  {0x2066, 0x0001}, /* Virtual register. Maintained by Goodix. */
  {0x2068, 0x0002}, /* Virtual register. Maintained by Goodix. */
  {0x20cc, 0x0001}, /* Virtual register. Maintained by Goodix. */
  {0x20ce, 0x0043}, /* Virtual register. Maintained by Goodix. */
  {0x2880, 0x0005}, /* Virtual register. Maintained by Goodix. */
  {0x2882, 0x0019}, /* Virtual register. Maintained by Goodix. */
  {0x2884, 0x0019}, /* Virtual register. Maintained by Goodix. */
  {0x2886, 0x0019}, /* Virtual register. Maintained by Goodix. */
  {0x288c, 0x0019}, /* Virtual register. Maintained by Goodix. */
  {0x3000, 0x0001}, /* Virtual register. Maintained by Goodix. */
  {0x3002, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x3004, 0x49f0}, /* Virtual register. Maintained by Goodix. */
  {0x3006, 0x0082}, /* Virtual register. Maintained by Goodix. */
  {0x35c0, 0x0004}, /* Virtual register. Maintained by Goodix. */
  {0x35c4, 0x0100}, /* Virtual register. Maintained by Goodix. */
  {0x35c6, 0x0302}, /* Virtual register. Maintained by Goodix. */
  {0xffff, 0x0001}
};

/* Initial register values for factest mode */

const struct gh3020_reg_s gh3020_reglist_factest[] =
{
  {0x0004, 0x001f}, /* Maxmium sample rate = 32KHz / (31 +１) = 1000Hz */
  {0x000a, 0x00c8}, /* Default FIFO watermark = 200 */
  {0x000e, 0x000d}, /* GH3020_REG_WKUP_TMR. Not mentioned in datasheet. */
  {0x0100, 0x0100}, /* Slot0 uses slot_cfg0, slot1 uses slot_cfg1 */
  {0x0102, 0x0302}, /* Slot2 uses slot_cfg2, slot3 uses slot_cfg3 */
  {0x010a, 0x277c}, /* Slot_cfg0(dark): sync signal, use FIFO, 4ADCs */
  {0x0110, 0x0004}, /* No BG or DC cancel, 512x ADC */
  {0x0112, 0x0000}, /* TIA0~3 10KOHm */
  {0x0114, 0x00aa}, /* All ADCs use 200pF TIA_CF */
  {0x0126, 0x277c}, /* Slot_cfg1(fixed G): sync signal, use FIFO, 4ADCs */
  {0x012c, 0x0c14}, /* DC&BG cancel, LED drv0&1 fixed, 1st BG, 512x ADC */
  {0x012e, 0x0000}, /* TIA0~3 10KOHm */
  {0x0130, 0x00aa}, /* All ADCs use 200pF TIA_CF */
  {0x013a, 0x0019}, /* LED drv0 to LED0(to green LED) pin */
  {0x013c, 0x0019}, /* LED drv0 to LED0(to green LED) pin */
  {0x0142, 0x277c}, /* Slot_cfg2(fix R): sync signal, use FIFO, 4ADCs */
  {0x0148, 0x0c14}, /* DC&BG cancel, LED drv0&1 fixed, 1st BG, 512x ADC */
  {0x014a, 0x0000}, /* TIA0~3 10KOHm */
  {0x014c, 0x00aa}, /* All ADCs use 200pF TIA_CF */
  {0x0156, 0x0219}, /* LED drv0 to LED2(to red LED) pin */
  {0x0158, 0x0219}, /* LED drv0 to LED2(to red LED) pin */
  {0x015e, 0x277c}, /* Slot_cfg3(fix IR): sync signal, use FIFO, 4ADCs */
  {0x0164, 0x0c14}, /* DC&BG cancel, LED drv0&1 fixed, 1st BG, 512x ADC */
  {0x0166, 0x0000}, /* TIA0~3 10KOHm */
  {0x0168, 0x00aa}, /* All ADCs use 200pF TIA_CF */
  {0x0172, 0x0119}, /* LED drv0 to LED1(to IR LED) pin */
  {0x0174, 0x0119}, /* LED drv0 to LED1(to IR LED) pin */
  {0x01ec, 0x005f}, /* Slot0's time 95 us (dark) */
  {0x01ee, 0x0122}, /* Slot1's time 290 us (green fixed) */
  {0x01f0, 0x0122}, /* Slot1's time 290 us (R fixed) */
  {0x01f2, 0x0122}, /* Slot1's time 290 us (IR fixed) */
  {0x0200, 0x0120}, /* GH3020_REG_AFE_REG0. Not mentioned in datasheet. */
  {0x0502, 0x4f60}, /* INT: Rst,wear-off/on,tunning fail/done,FIFO ovf/full */
  {0x0504, 0x000a}, /* Interrupt pulse width 33us * 10 */
  {0x0588, 0x0008}, /* OSC32K_TEMP. Not mentioned in datasheet. */
  {0x0692, 0x0233}, /* LED_DRV_AD_REG. Not mentioned in datasheet. */
  {0x0694, 0x0400}, /* ECG_IA_AD_REG. Not mentioned in datasheet. */
  {0x0698, 0x2310}, /* ECG_IA_AD_REG2. Not mentioned in datasheet. */
  {0x0700, 0x0000}, /* EFUSE for LED current calibration disabled */
  {0x1000, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x1002, 0x3080}, /* Virtual register. Maintained by Goodix. */
  {0x1004, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x1006, 0xd082}, /* Virtual register. Maintained by Goodix. */
  {0x1008, 0x297e}, /* Virtual register. Maintained by Goodix. */
  {0x100a, 0x0001}, /* Virtual register. Maintained by Goodix. */
  {0x10e0, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x10e2, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x10e4, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x10e6, 0xff02}, /* Virtual register. Maintained by Goodix. */
  {0x10e8, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x10ea, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x10ec, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x10ee, 0x0202}, /* Virtual register. Maintained by Goodix. */
  {0x1120, 0x00c8}, /* Virtual register. Maintained by Goodix. */
  {0x1122, 0x0005}, /* Virtual register. Maintained by Goodix. */
  {0x1124, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x1140, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x1142, 0x0019}, /* Virtual register. Maintained by Goodix. */
  {0x1144, 0x0001}, /* Virtual register. Maintained by Goodix. */
  {0x1146, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x21ba, 0x0010}, /* Virtual register. Maintained by Goodix. */
  {0x21bc, 0x0d05}, /* Virtual register. Maintained by Goodix. */
  {0x21be, 0x1d15}, /* Virtual register. Maintained by Goodix. */
  {0x21c0, 0x2921}, /* Virtual register. Maintained by Goodix. */
  {0x21c2, 0x3931}, /* Virtual register. Maintained by Goodix. */
  {0x21c4, 0x4b43}, /* Virtual register. Maintained by Goodix. */
  {0x21c6, 0x5b53}, /* Virtual register. Maintained by Goodix. */
  {0x21c8, 0x6a62}, /* Virtual register. Maintained by Goodix. */
  {0x21ca, 0x7a72}, /* Virtual register. Maintained by Goodix. */
  {0x289a, 0x0019}, /* Virtual register. Maintained by Goodix. */
  {0x3000, 0x0400}, /* Virtual register. Maintained by Goodix. */
  {0x3002, 0x0000}, /* Virtual register. Maintained by Goodix. */
  {0x3004, 0x49f0}, /* Virtual register. Maintained by Goodix. */
  {0x3006, 0x0082}, /* Virtual register. Maintained by Goodix. */
  {0xffff, 0x0001}
};

/* Initial register configurations list */

const struct gh3020_initcfg_s gh3020_initcfg_list[GH3020_CFGLIST_NUM] =
{
  {
    gh3020_reglist_normal,
    sizeof(gh3020_reglist_normal) / sizeof(FAR struct gh3020_reg_s)
  },

  {
    gh3020_reglist_factest,
    sizeof(gh3020_reglist_factest) / sizeof(FAR struct gh3020_reg_s)
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gh3020_configspi
 *
 * Description:
 *   Set SPI mode, frequency and bits per word, according to gh3020_dev_s->
 *   gh3020_config_s.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_configspi(FAR struct gh3020_dev_s *priv)
{
  int freq = priv->config->freq;

  /* Set SPI frequency in an acceptable range. */

  if (freq > GH3020_SPI_FREQ_MAX)
    {
      freq = GH3020_SPI_FREQ_MAX;
    }
  else if (freq < GH3020_SPI_FREQ_MIN)
    {
      freq = GH3020_SPI_FREQ_MIN;
    }

  if (SPI_SETFREQUENCY(priv->config->spi, freq) != freq)
    {
      return -EIO;
    }

  /* GH3020 SPI supports default mode0. If one must used mode3, GH3020 can
   * set SPI in mode3 under mode0 first.
   */

  SPI_SETMODE(priv->config->spi, SPIDEV_MODE0);

  /* Set number of bits per word. */

  SPI_SETBITS(priv->config->spi, GH3020_SPI_NBITS);

  return OK;
}

/****************************************************************************
 * Name: gh3020_spi_exchange
 *
 * Description:
 *   Control CS pin and exchange content via SPI.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   buf    - A pointer to the buffer which stores data to exchange.
 *   nwords - The numbers of word (8bits per word) to be read.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_spi_exchange(FAR struct gh3020_dev_s *priv,
                                FAR uint8_t *buf, uint16_t nwords)
{
  /* Pull down CS pin manually */

  IOEXP_WRITEPIN(priv->config->ioedev, priv->config->gpiocs, 0);

  /* Selects the device. */

  SPI_SELECT(priv->config->spi, priv->config->cs, true);

  /* First send read command, then sends arbitrary content (nwords bytes) to
   * read a block of nwords bytes.
   */

  SPI_EXCHANGE(priv->config->spi, buf, buf, nwords);

  /* Pull up CS pin manually */

  IOEXP_WRITEPIN(priv->config->ioedev, priv->config->gpiocs, 1);

  /* Deselect the device and release the SPI bus. */

  SPI_SELECT(priv->config->spi, priv->config->cs, false);
}

/****************************************************************************
 * Name: gh3020_calcu_fifowtm
 *
 * Description:
 *   Calculate FIFO watermark according to the common minium batch latency
 *   and each channel's sample interval.
 *
 * Input Parameters:
 *   priv - The device struct pointer.
 *
 * Returned Value:
 *   If there's no activated channel, return 0.
 *   If the common minium batch latency is 0, polling mode is used, return 0.
 *   Otherwise return calculated FIFO watermark (uints in data, each sample
 *   has 4 data from 4 PD).
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static uint16_t gh3020_calcu_fifowtm(FAR struct gh3020_dev_s *priv)
{
  uint32_t batch_min = 0xffffffff;
  uint16_t fifowtm = 0;
  uint8_t idx;

  /* Search minium batch latency among all activated PPG channel. */

  for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
    {
      if (priv->sensor[idx].activated == true &&
          priv->sensor[idx].batch < batch_min)
        {
          batch_min = priv->sensor[idx].batch;
        }
    }

  /* If any PPG is activated, update the batch in device struct */

  if (batch_min != 0xffffffff)
    {
      priv->batch = batch_min;

      /* If minium batch > 0, reading will use FIFO watermark interrupt. */

      if (batch_min > 0)
        {
          for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
            {
              if (priv->sensor[idx].activated == true)
                {
                  /* Device's watermark is sum of all components. The unit of
                   * watermark is data (one data per ADC).
                   */

                  fifowtm = fifowtm +
                            batch_min / priv->sensor[idx].interval *
                            gh3020_adc_num_list[idx];
                }
            }
        }
    }

  return fifowtm;
}

/****************************************************************************
 * Name: gh3020_extract_frame
 *
 * Description:
 *   Extract PPG values, gains and LED current from frame readback, and store
 *   them in specialized PPGQ struct.
 *
 * Input Parameters:
 *   priv       - The device struct pointer.
 *   idx        - PPG channel index.
 *   pppg       - Pointer to PPGQ struct to store this frame's useful infos.
 *   pframeinfo - Pointer to the info of frame readback.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_extract_frame(FAR struct gh3020_dev_s *priv, uint8_t idx,
                                 FAR struct sensor_ppgq *pppg,
                                 FAR const struct gh3020_frameinfo_s
                                 *pframeinfo)
{
  uint32_t gain;
  uint8_t led_drv0;
  uint8_t led_drv1;
  uint8_t i;
  uint8_t fifo_data_idx;

  led_drv0 = (pframeinfo->punFrameAgcInfo[0] & 0x0000ff00u) >> 8;
  led_drv1 = (pframeinfo->punFrameAgcInfo[0] & 0x00ff0000u) >> 16;
  if (priv->load_efuse)
    {
      pppg->current = (uint32_t)
        (GH3020_LED_DRV_VOLTAGE_UV / (priv->led_calibr * 255.0f / led_drv0 +
                                   GH3020_LED_DRV0_Y0) +
         GH3020_LED_DRV_VOLTAGE_UV / (priv->led_calibr * 255.0f / led_drv1 +
                                   GH3020_LED_DRV1_Y1));
    }
  else
    {
      pppg->current = (uint32_t)
        (GH3020_LED_DRV_VOLTAGE_UV / (255.0f / led_drv0 + GH3020_LED_DRV0_Y0) +
         GH3020_LED_DRV_VOLTAGE_UV / (255.0f / led_drv1 + GH3020_LED_DRV1_Y1));
    }

  fifo_data_idx = 0;
  for (i = 0; i < GH3020_TOTAL_ADC_NUM; i++)
    {
      if (gh3020_adc_en_list[idx][i])
        {
          pppg->ppg[i] =  pframeinfo->punFrameRawdata[i];
          gain = (uint32_t)pframeinfo->punFrameAgcInfo[i] & 0x0000000f;
          pppg->gain[i] = gh3020_gain_list[gain];
          fifo_data_idx++;
        }
    }
}

/****************************************************************************
 * Name: gh3020_push_data
 *
 * Description:
 *   Push activated PPG channel's data.
 *
 * Input Parameters:
 *   priv - The device struct pointer.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_push_data(FAR struct gh3020_dev_s *priv)
{
  uint64_t ts_interval;
  int32_t ts_jitter;
  bool ts_abnormal;
  uint8_t idx;
  uint8_t j;

  /* Calculate timestamp for each sample and push data for each channel. */

  for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
    {
      if (priv->ppgdatacnt[idx] > 0 && priv->sensor[idx].activated == true)
        {
          for (j = 0; j < priv->ppgdatacnt[idx]; j++)
            {
              priv->ppgdata[idx][j].timestamp = priv->timestamp -
                (priv->ppgdatacnt[idx] - 1 - j) * priv->sensor[idx].interval;
            }

          priv->sensor[idx].lower.push_event(priv->sensor[idx].lower.priv,
                                             priv->ppgdata[idx],
                                             sizeof(struct sensor_ppgq)
                                             * priv->ppgdatacnt[idx]);

          ts_interval = priv->timestamp - priv->timestamp_last;
          if (priv->batch > 0)
            {
              ts_jitter = abs((int32_t)ts_interval - (int32_t)priv->batch);
              ts_abnormal = (ts_jitter > (priv->batch >> 3)) ? true : false;
            }
          else
            {
              ts_jitter = abs((int32_t)ts_interval -
                              (int32_t)priv->interval);
              ts_abnormal = (ts_jitter > (priv->interval >> 3)) ?
                            true : false;
            }

          if (ts_abnormal)
            {
              syslog(LOG_INFO, "[gh3020] push ppg%u ts intvl %llu\n",
                     gh3020_node_name_list[idx], ts_interval);
            }
        }
    }

  GH3020_DEBUG_LOG("[gh3020] ts %llu push: 0-%u, 1-%u, 2-%u, 5-%u\n",
                   priv->timestamp, priv->ppgdatacnt[0], priv->ppgdatacnt[1],
                   priv->ppgdatacnt[2], priv->ppgdatacnt[3]);
}

/****************************************************************************
 * Name: gh3020_reset_cfg
 *
 * Description:
 *   For PPG0/1/2/, reset their LED current and TIA gain config.
 *
 * Input Parameters:
 *   priv - The device struct pointer.
 *   chidx - PPG channel index, 0, 1, or 2.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   The full scale of LED driver is prior defined.
 *
 ****************************************************************************/

static void gh3020_reset_cfg(FAR struct gh3020_dev_s *priv, uint8_t chidx)
{
  float current_ma;
  float resist;
  uint16_t drv_code;
  uint8_t i;
  struct gh3020_reg_s cfg_init[9] =
    {
      {0X012e, 0x4444}, /* TIAx 100KOhm */
      {0x013a, 0x0011}, /* LED drv0 to LED0(to green LED) pin, 14.9mA */
      {0x013c, 0x0011}, /* LED drv1 to LED0(to green LED) pin, 14.99mA */
      {0x014a, 0x4441}, /* TIA0 25KOhm, others default 100KOhm */
      {0x0156, 0x0245}, /* LED drv0 to LED2(to red LED) pin, 59.28mA */
      {0x0158, 0x0245}, /* LED drv1 to LED2(to red LED) pin, 60.72mA */
      {0x0166, 0x4441}, /* TIA0 25KOhm, others default 100KOhm */
      {0x0172, 0x012e}, /* LED drv0 to LED1(to IR LED) pin, 39.87mA */
      {0x0174, 0x012e}  /* LED drv1 to LED1(to IR LED) pin, 40.52mA */
    };

  if (chidx > GH3020_PPG2_IDX)
    {
      return;
    }

  /* Reset TIA gain cfg */

  gh3020_spi_writereg(cfg_init[chidx * 3].regaddr,
                      cfg_init[chidx * 3].regval);

  /* There are 2 LED drivers */

  for (i = 1; i < 3; i++)
    {
      drv_code = cfg_init[chidx * 3 + i].regval & 0xff;

      /* Calibrate LED driver code if EFUSE exists */

      if (priv->efuse)
        {
          resist = ((i == 1) ? GH3020_LED_DRV0_Y0 : GH3020_LED_DRV1_Y1);
          current_ma = GH3020_LED_DRV_VOLTAGE_MV /
                       (255.0f * GH3020_LED_DRV_SCALE_N /
                        (float)drv_code + resist);
          drv_code = (uint16_t)(0.5f + (priv->led_calibr * 255.0f *
                                (float)GH3020_LED_DRV_SCALE_N) /
                                (GH3020_LED_DRV_VOLTAGE_MV / current_ma -
                                 resist));
        }

      gh3020_spi_writereg(cfg_init[chidx * 3 + i].regaddr,
                          (cfg_init[chidx * 3 + i].regval & 0xff00) |
                           drv_code);
    }
}

/****************************************************************************
 * Name: gh3020_restart_new_fifowtm
 *
 * Description:
 *   Congifure the GH3020 with new FIFO watermark and start it. Reading mode
 *   will alse change if needed.
 *
 * Input Parameters:
 *   priv    - The device struct pointer.
 *   fifowtm - New FIFO watermark.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   GH3020 has been stopped or has not been started.
 *
 ****************************************************************************/

static void gh3020_restart_new_fifowtm(FAR struct gh3020_dev_s *priv,
                                       uint16_t fifowtm)
{
  uint8_t idx;

  GH3020_DEBUG_LOG("[gh3020] restart, interval %lu, new wtm %u(data), old "
                   "wtm %u(data)\n", priv->interval, fifowtm, priv->fifowtm);

  /* Reset with initialize and some other operations may cause the device's
   * interrupt pin to toggle abnormally. If interrupt reading is used,
   * disable it first.
   */

  if (priv->fifowtm > 0)
    {
      IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                      IOEXPANDER_OPTION_INTCFG,
                      (FAR void *)IOEXPANDER_VAL_DISABLE);
    }

  /* Either old or new FIFO watermark is 0 while the other one is not 0,
   * reading mode should switch between interrupt mode and polling mode.
   */

  if ((fifowtm > 0 && priv->fifowtm == 0) ||
      (fifowtm == 0 && priv->fifowtm > 0))
    {
      /* GH3020 changes into corresponding reading mode. */

      if (fifowtm > 0)
        {
          gh3020_rdmode_switch(GH3020_RDMODE_INTERRPT);
        }
      else
        {
          gh3020_rdmode_switch(GH3020_RDMODE_POLLING);
        }

      /* After switch reading mode, GH3020 must be initialized again. Then
       * set each activated or to-be-activated channel's sample rate again.
       */

      gh3020_init();
      for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
        {
          if (priv->sensor[idx].activated)
            {
              gh3020_samplerate_set(gh3020_channel_function_list[idx],
                (uint16_t)
                (GH3020_ONE_SECOND / priv->sensor[idx].interval));
            }
        }

      /* Set FIFO watermark if not 0, then restart sampling. */

      if (fifowtm > 0)
        {
          gh3020_set_fifowtm(fifowtm);
        }

      /* Some events may be processed between init and start */

      gh3020_fifo_process();
      gh3020_start_sampling(priv->channelmode);

      /* Switch MCU's reading mode between polling and interrupt if needed,
       * or just enable the interrupt again
       */

      if (fifowtm > 0)
        {
          /* If it's polling before, switch to interrupt */

          if (priv->fifowtm == 0)
            {
              gh3020_switch_poll2intrpt(priv);
            }

          /* Otherwise only turn on the interrupt again */

          else
            {
              IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                              IOEXPANDER_OPTION_INTCFG,
                              (FAR void *)IOEXPANDER_VAL_RISING);
            }
        }
      else if (fifowtm == 0 && priv->fifowtm > 0)
        {
          gh3020_switch_intrpt2poll(priv);
        }

      GH3020_DEBUG_LOG("[gh3020] reinit+restart\n");
    }
  else
    {
      /* Seek if any PPG channel need reset LED current */

      for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
        {
          if (priv->sensor[idx].reset_cfg == true)
            {
              GH3020_DEBUG_LOG("[gh3020] need reset LED\n");
              break;
            }
        }

      /* If YES, the device must exit low power mode first, then write reg */

      if (idx < GH3020_SENSOR_NUM)
        {
          /* Exit low power mode. Note that we dont't enter low power, since
           * it will enter low power mode when calling gh3020_start_sampling
           */

          GH3X2X_ExitLowPowerMode();
          for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
            {
              if (priv->sensor[idx].reset_cfg)
                {
                  gh3020_reset_cfg(priv, idx);
                  priv->sensor[idx].reset_cfg = false;
                }
            }
        }

      /* GH3020 remains interrupt/polling, only watermark shall be changed. */

      if (fifowtm > 0)
        {
          gh3020_set_fifowtm(fifowtm);
          IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                          IOEXPANDER_OPTION_INTCFG,
                          (FAR void *)IOEXPANDER_VAL_RISING);
        }

      gh3020_start_sampling(priv->channelmode);
      GH3020_DEBUG_LOG("[gh3020] restart\n");
    }

  SCHED_NOTE_PRINTF("GH3020 starts, fifowtm=%u", fifowtm);
  priv->fifowtm = fifowtm;
}

/****************************************************************************
 * Name: gh3020_switch_poll2intrpt
 *
 * Description:
 *   Reading mode swtiched from polling to interrupt.
 *
 * Input Parameters:
 *   priv - The device struct pointer.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_switch_poll2intrpt(FAR struct gh3020_dev_s *priv)
{
  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_RISING);
  work_cancel(HPWORK, &priv->work_poll);
}

/****************************************************************************
 * Name: gh3020_switch_intrpt2poll
 *
 * Description:
 *   Reading mode swtiched from interrupt to polling.
 *
 * Input Parameters:
 *   priv - The device struct pointer.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_switch_intrpt2poll(FAR struct gh3020_dev_s *priv)
{
  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);
  work_queue(HPWORK, &priv->work_poll, gh3020_worker_poll, priv,
             priv->interval / USEC_PER_TICK);
}

/****************************************************************************
 * Name: gh3020_update_sensor
 *
 * Description:
 *   Update sensors' activating status after FIFO reading.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_update_sensor(FAR struct gh3020_dev_s *priv)
{
  FAR struct gh3020_sensor_s *sensor;
  unsigned long interval_min;
  uint16_t fifowtm;
  uint16_t rate;
  uint8_t idx;

  if (priv->updating == true)
    {
      gh3020_stop_sampling(priv->channelmode);
      priv->updating = false;
      priv->activated = 0;
      priv->channelmode = 0;

      /* Check each sensor, add activating ones, remove inactivating ones. */

      for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
        {
          sensor = &priv->sensor[idx];
          if (sensor->activating == true || (sensor->activated == true &&
              sensor->inactivating == false))
            {
              rate = (uint16_t)(GH3020_ONE_SECOND / sensor->interval);
              priv->channelmode = priv->channelmode |
                                  gh3020_channel_function_list[idx];
              gh3020_samplerate_set(
                    gh3020_channel_function_list[idx], rate);
              if (sensor->activating == true)
                {
                  SCHED_NOTE_PRINTF("activate ppgq%u, rate=%u",
                                    gh3020_node_name_list[idx], rate);
                  syslog(LOG_INFO, "[gh3020] update ppg%u r=%u\n",
                         gh3020_node_name_list[idx], rate);
                }

              sensor->activating = false;
              sensor->activated = true;
              priv->activated++;
            }
          else if (sensor->inactivating == true)
            {
              sensor->inactivating = false;
              sensor->activated = false;
              SCHED_NOTE_PRINTF("inactivate ppgq%u",
                                gh3020_node_name_list[idx]);
              syslog(LOG_INFO, "[gh3020] update ppg%u stop\n",
                     gh3020_node_name_list[idx]);
            }
        }

      /* If GH3020 shall restart because of some activated channels */

      if (priv->activated > 0)
        {
          fifowtm = gh3020_calcu_fifowtm(priv);

          /* Seek for minium sample interval. */

          interval_min = 0xffffffff;
          for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
            {
              if (priv->sensor[idx].activated == true &&
                  priv->sensor[idx].interval < interval_min)
                {
                  interval_min = priv->sensor[idx].interval;
                }
            }

          priv->interval = interval_min;
          gh3020_restart_new_fifowtm(priv, fifowtm);
        }

      /* Otherwise GH3020 will not start as no channel is activated */

      else
        {
          if (priv->fifowtm > 0)
            {
              IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                              IOEXPANDER_OPTION_INTCFG,
                              (FAR void *)IOEXPANDER_VAL_DISABLE);
            }
          else
            {
              work_cancel(HPWORK, &priv->work_poll);
            }

          SCHED_NOTE_PRINTF("GH3020 stops");
          syslog(LOG_INFO, "[gh3020] stops\n");
        }
    }
}

/****************************************************************************
 * Name: gh3020_factest_start
 *
 * Description:
 *   GH3020 start sample in factest mode with specialized LED color and
 *   driver current.
 *
 * Input Parameters:
 *   channelmode - The PPG channel want to test.
 *   current     - LED driver current.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_FACTEST_SENSORS_GH3020
static void gh3x2x_factest_start(uint32_t channelmode, uint32_t current)
{
  struct gh3020_factestmode_param_s sample_para;

  memset(&sample_para, 0, sizeof(sample_para));
  memset(&sample_para.tia_gain[4], GH3020_FACTEST_TIA_GAIN_100_K, 12);
  sample_para.channelmode = GH3X2X_FUNCTION_TEST1;
  sample_para.tia_gain_change_en = true;
  sample_para.sample_rate_change_en = true;
  sample_para.int_time_change_en = true;
  sample_para.led_current_change_en = true;
  sample_para.sample_rate = GH3020_SR_DFT;
  sample_para.int_time = GH3020_FACTEST_INT_TIME_79_US;

  switch (channelmode)
    {
      case GH3X2X_FUNCTION_HR:    /* Green */
        {
          sample_para.led_drv0_current[7] = (uint8_t)(current / 2000);
          sample_para.led_drv1_current[7] = (uint8_t)(current / 2000);
        }
        break;

      case GH3X2X_FUNCTION_SPO2:  /* Red */
        {
          sample_para.led_drv0_current[11] = (uint8_t)(current / 2000);
          sample_para.led_drv1_current[11] = (uint8_t)(current / 2000);
        }
        break;

      case GH3X2X_FUNCTION_HRV:   /* IR */
        {
          sample_para.led_drv0_current[15] = (uint8_t)(current / 2000);
          sample_para.led_drv1_current[15] = (uint8_t)(current / 2000);
        }
        break;

      case GH3X2X_FUNCTION_RESP:  /* Dark */
          break;

      default:
        {
          snerr("Invalid PPG channel mode.\n");
          return;
        }
  }

  gh3020_start_sampling_factest(GH3X2X_FUNCTION_TEST1, &sample_para, 1);
}
#endif  /* CONFIG_FACTEST_SENSORS_GH3020 */

/****************************************************************************
 * Name: gh3020_activate
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   lower  - The instance of lower half sensor driver.
 *   filep  - The pointer of file, represents each user using the sensor.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           bool enable)
{
  FAR struct gh3020_sensor_s *sensor = (FAR struct gh3020_sensor_s *)lower;
  FAR struct gh3020_dev_s *priv;
  uint16_t rate;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL);

  priv = sensor->dev;
  syslog(LOG_INFO, "[gh3020] ppg%u activate %d->%d\n",
         gh3020_node_name_list[sensor->chidx], sensor->activated, enable);

#ifdef CONFIG_FACTEST_SENSORS_GH3020
  if (priv->factest_mode == true)
    {
      /* In factest mode, activate operation will be performed
       * immediately, where we can change its activated status now.
       */

      sensor->activated = enable;
      if (enable == true)
        {
          gh3020_rdmode_switch(GH3020_RDMODE_POLLING);
          gh3020_init();
          gh3020_fifo_process();
          gh3x2x_factest_start(
            gh3020_channel_function_list[sensor->chidx],
            sensor->current);
          priv->activated = 1;
          priv->intvl_prev = priv->interval;
          priv->interval = GH3020_INTVL_DFT;
          work_queue(HPWORK, &priv->work_poll, gh3020_worker_poll, priv,
                      priv->interval / USEC_PER_TICK);
        }
      else
        {
          gh3020_stop_sampling_factest();
          priv->activated = 0;
          priv->interval = priv->intvl_prev;
          work_cancel(HPWORK, &priv->work_poll);
        }
    }
  else
#endif  /* CONFIG_FACTEST_SENSORS_GH3020 */
    {
      /* If any PPG channel has been activated, mark it as activating (
       * will be activated) or inactivating (will be inactivated). The
       * activated status will be changed in updating when a poll or
       * interrupt comes. If it's changed several times, the last operation
       * will be valid.
       */

      if (priv->activated > 0)
        {
          priv->updating = true;
          if (enable == true)
            {
              sensor->activating = true;
              sensor->inactivating = false;
              if (sensor->activated == false &&
                  sensor->chidx != GH3020_PPG5_IDX)
                {
                  sensor->reset_cfg = true;
                }
            }
          else
            {
              sensor->inactivating = true;
              sensor->activating = false;
              sensor->reset_cfg = false;
            }
        }

      /* Otherwise no channel is activated now. */

      else
        {
          /* If one want to activate a channel, since there's no other
           * activated channel, the activate operation of this channel is
           * performed here and the activated status changes. If one want to
           * deactive the channel, do nothing since no channel is activated.
           */

          sensor->activated = enable;

          if (enable == true)
            {
              priv->activated++;
              priv->interval = sensor->interval;
              priv->batch = sensor->batch;
              rate = (uint16_t)(GH3020_ONE_SECOND / sensor->interval);
              if (sensor->batch > 0)
                {
                  gh3020_rdmode_switch(GH3020_RDMODE_INTERRPT);
                }
              else
                {
                  gh3020_rdmode_switch(GH3020_RDMODE_POLLING);
                }

              /* GH3020 must be initialized after switch reading mode. */

              gh3020_init();

              /* Enable the channel with desired sample rate. */

              priv->channelmode =
                gh3020_channel_function_list[sensor->chidx];
              gh3020_samplerate_set(
                gh3020_channel_function_list[sensor->chidx], rate);

              /* Set FIFO watermark if needed. */

              priv->fifowtm = sensor->batch / sensor->interval *
                              gh3020_adc_num_list[sensor->chidx];
              if (sensor->batch > 0)
                {
                  gh3020_set_fifowtm(priv->fifowtm);
                }

              /* Some events may be processed between init and start */

              gh3020_fifo_process();
              gh3020_start_sampling(priv->channelmode);
              if (sensor->batch > 0)
                {
                  IOEXP_SETOPTION(priv->config->ioedev,
                                  priv->config->intpin,
                                  IOEXPANDER_OPTION_INTCFG,
                                  (FAR void *)IOEXPANDER_VAL_RISING);
                }
              else
                {
                  work_queue(HPWORK, &priv->work_poll,
                              gh3020_worker_poll, priv,
                              priv->interval / USEC_PER_TICK);
                }

              SCHED_NOTE_PRINTF("activate ppgq%u, rate=%u, GH3020 starts, "
                                "fifowtm=%u",
                                gh3020_node_name_list[sensor->chidx], rate,
                                priv->fifowtm);
              syslog(LOG_INFO, "[gh3020] en ppg%u r=%u wtm=%u\n",
                     gh3020_node_name_list[sensor->chidx], rate,
                     priv->fifowtm);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: gh3020_batch
 *
 * Description:
 *   Set PPG sensor's maximum report latency in microseconds.
 *
 * Input Parameters:
 *   filep      - The pointer of file, represents each user using the sensor.
 *   lower      - The instance of lower half sensor driver.
 *   latency_us - the time between batch data, in us. It may by overwrite
 *                by lower half driver.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_batch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR unsigned long *latency_us)
{
  FAR struct gh3020_sensor_s *sensor = (FAR struct gh3020_sensor_s *)lower;
  FAR struct gh3020_dev_s *priv;
  unsigned long max_latency;
  uint16_t fifowtm = 0;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL && latency_us != NULL);

  priv = sensor->dev;
  if (priv->factest_mode == true)
    {
      return OK;
    }

  /* Latency is modified to reasonable value. */

  if (*latency_us > 0)
    {
      max_latency = sensor->lower.nbuffer * sensor->interval;
      if (*latency_us > max_latency)
        {
          *latency_us = max_latency;
        }
      else if (*latency_us < sensor->interval)
        {
          *latency_us = sensor->interval;
        }

      fifowtm = GH3020_CEILING(*latency_us, sensor->interval);
      *latency_us = fifowtm * sensor->interval;
    }

  GH3020_DEBUG_LOG("[gh3020] set ppg%u batch actual=%lu\n",
                   gh3020_node_name_list[sensor->chidx], *latency_us);

  /* Do something only when the batch changed. */

  if (*latency_us != sensor->batch)
    {
      sensor->batch = *latency_us;

      /* If GH3020 is running, flag it and new batch will take effect when
       * next FIFO procession comes. Otherwise the new batch will take effect
       * when activating this PPG channel.
       */

      if (priv->activated > 0)
        {
          priv->updating = true;
        }
    }
  else
    {
      return OK;
    }

  return OK;
}

/****************************************************************************
 * Name: gh3020_set_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us > max_interval it will be truncated to max_interval and
 *   if *period_us < min_interval it will be replaced by min_interval. The
 *   new interval will take effect when activating or reading FIFO.
 *
 * Input Parameters:
 *   lower     - The instance of lower half sensor driver.
 *   filep     - The pointer of file, represents each user using the sensor.
 *   period_us - The time between report data, in us. It may by overwrite
 *               by lower half driver.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us)
{
  FAR struct gh3020_sensor_s *sensor = (FAR struct gh3020_sensor_s *)lower;
  FAR struct gh3020_dev_s *priv;
  float freq;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL && period_us != NULL);

  priv = sensor->dev;
  if (priv->factest_mode == true)
    {
      return OK;
    }

  /* Calculate best sample rate */

  freq = GH3020_ONE_SECOND / *period_us;
  if (freq > GH3020_SAMPLERATE_MAX)
    {
      freq = GH3020_SAMPLERATE_MAX;
    }
  else if (freq < GH3020_SAMPLERATE_MIN)
    {
      freq = GH3020_SAMPLERATE_MIN;
    }

  *period_us = GH3020_ONE_SECOND / freq;
  GH3020_DEBUG_LOG("[gh3020] set ppg%d interval actual=%lu\n",
                   gh3020_node_name_list[sensor->chidx], *period_us);

  /* Do something only when the interval changed. */

  if (*period_us != sensor->interval)
    {
      sensor->interval = *period_us;

      /* If this PPG channel is running, new interval will take effect when
       * next FIFO procession comes. Otherwise do nothing, the new interval
       * will take effect when activating this sensor (PPG channel).
       */

      if (sensor->activated == true)
        {
          priv->updating = true;
        }
    }
  else
    {
      return OK;
    }

  return OK;
}

/****************************************************************************
 * Name: gh3020_selftest
 *
 * Description:
 *   Selftest of PPG sensor, i.e. the selftest of GH3020.
 *
 * Input Parameters:
 *   lower - The instance of lower half sensor driver.
 *   filep - The pointer of file, represents each user using the sensor.
 *   arg   - The parameters associated with cmd.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_FACTEST_SENSORS_GH3020
static int gh3020_selftest(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           unsigned long arg)
{
  FAR struct gh3020_dev_s *priv = (FAR struct gh3020_dev_s *)lower;

  DEBUGASSERT(priv != NULL);

  switch (arg)
    {
      case SNIOC_SIMPLE_CHECK:              /* Check ID command. */
        {
          if (gh3020_init() != OK)
            {
              return -ENODEV;
            }
          else
            {
              return OK;
            }
        }

      /* In the case above, function has returned thus no break is needed. */

      default:                              /* Other cmd tag */
        {
          snerr("The cmd was not supported: %d\n", -ENOTTY);
          return -ENOTTY;
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: gh3020_control
 *
 * Description:
 *   With this method, the user can set some special config for the sensor,
 *   such as changing the custom mode, setting the custom resolution, reset,
 *   etc, which are all parsed and implemented by lower half driver.
 *
 * Input Parameters:
 *   lower - The instance of lower half sensor driver.
 *   filep - The pointer of file, represents each user using the sensor.
 *   cmd   - The special cmd for sensor.
 *   arg   - The parameters associated with cmd.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, int cmd,
                          unsigned long arg)
{
  FAR struct gh3020_sensor_s *sensor = (FAR struct gh3020_sensor_s *)lower;
#ifdef CONFIG_FACTEST_SENSORS_GH3020
  FAR struct sensor_ioctl_s *ioctl = (FAR struct sensor_ioctl_s *)arg;
  FAR struct gh3020_dev_s *priv;
#endif

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL);

#ifdef CONFIG_FACTEST_SENSORS_GH3020
  syslog(LOG_INFO, "[gh3020] ioctl cmd %x arg %lx\n", cmd, arg);
  priv = sensor->dev;
#endif

  switch (cmd)
    {
#ifdef CONFIG_FACTEST_SENSORS_GH3020
      case SNIOC_CTRL_LED_CURRENT:      /* Set LED current(uA) */
        {
          if (priv->factest_mode == true)
            {
              FAR uint32_t *current = (FAR uint32_t *)ioctl->data;
              uint32_t leddrv_val;

              leddrv_val = (uint32_t)roundf((float)*current * 0xffu / 1000 /
                                            GH3020_LEDDRV_CURRENT_MA_MAX);
              sensor->current = leddrv_val * GH3020_LEDDRV_CURRENT_MA_MAX *
                                1000 / 0xffu;
              *current = sensor->current;
              if (sensor->activated == true)
                {
                  gh3020_stop_sampling_factest();
                  gh3x2x_factest_start(
                    gh3020_channel_function_list[sensor->chidx],
                    sensor->current);
                }
            }
          else
            {
              return -EINVAL;
            }
        }
        break;

      case SNIOC_CTRL_OPEN_FACTEST:    /* Enter factory test mode */
        {
          if (priv->factest_mode == false)
            {
              priv->factest_mode = true;
              syslog(LOG_INFO, "[gh3020] enter factest\n");
            }
        }
        break;

      case SNIOC_CTRL_EXIT_FACTEST:     /* Exit factory test mode */
        {
          if (priv->factest_mode == true)
            {
              priv->factest_mode = false;
              syslog(LOG_INFO, "[gh3020] exit factest\n");
              if (priv->activated > 0)
                {
                  syslog(LOG_INFO, "[gh3020] stop current factest sample\n");
                  gh3020_stop_sampling_factest();
                  priv->activated = 0;
                  sensor->activated = false;
                  priv->interval = priv->intvl_prev;
                  work_cancel(HPWORK, &priv->work_poll);
                }
            }
        }
        break;
#endif  /* CONFIG_FACTEST_SENSORS_GH3020 */

      default:
        {
          return -ENOTTY;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: gh3020_interrupt_handler
 *
 * Description:
 *   Handle the sensor interrupt.
 *
 * Input Parameters:
 *   dev     - ioexpander device.
 *   pinset  - Interrupt pin.
 *   arg     - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int gh3020_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the GH3020 INTx
   * pin, when an event, such as FIFO is almost full, has occured.
   */

  FAR struct gh3020_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp */

  priv->timestamp_last = priv->timestamp;
  priv->timestamp = sensor_get_timestamp();

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long, and neither should we
   * lock the I2C/SPI bus within an interrupt.
   */

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);
  work_queue(HPWORK, &priv->work_intrpt, gh3020_worker_intrpt, priv, 0);

  return OK;
}

/****************************************************************************
 * Name: gh3020_worker_intrpt
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long, and neither should we
 *   lock the I2C/SPI bus within an interrupt.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_worker_intrpt(FAR void *arg)
{
  FAR struct gh3020_dev_s *priv = arg;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  GH3020_DEBUG_LOG("[gh3020] interrupt worker, ts=%llu\n", priv->timestamp);

  /* Enable entering next interrupt. */

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_RISING);

  /* Start new counter for each channel. */

  memset(priv->ppgdatacnt, 0, GH3020_SENSOR_NUM);

  /* Goodix's documents said that this function must be called before calling
   * gh3020_fifo_process() in an interrupt procession.
   */

  gh3x2x_int_handler_call_back();

  /* This function from Goodix's library has an obvious delay and must be
   * called after arranging next worker.
   */

  gh3020_fifo_process();
  gh3020_update_sensor(priv);
  gh3020_push_data(priv);
}

/****************************************************************************
 * Name: gh3020_worker_poll
 *
 * Description:
 *   Task the worker with polling the latest sensor data.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void gh3020_worker_poll(FAR void *arg)
{
  FAR struct gh3020_dev_s *priv = arg;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Get timestamp and arrange next worker immediately once enter polling. */

  priv->timestamp_last = priv->timestamp;
  priv->timestamp = sensor_get_timestamp();
  if (priv->activated > 0)
    {
      work_queue(HPWORK, &priv->work_poll, gh3020_worker_poll, priv,
                 priv->interval / USEC_PER_TICK);
    }

  GH3020_DEBUG_LOG("[gh3020] poll worker, ts=%llu\n", priv->timestamp);

  /* Start new counter for each channel. */

  memset(priv->ppgdatacnt, 0, GH3020_SENSOR_NUM);

  /* This function from Goodix's library has an obvious delay and must be
   * called after arranging next worker.
   */

  gh3020_fifo_process();
  gh3020_update_sensor(priv);
  gh3020_push_data(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: GH3X2X_WriteReg
 *
 * Description:
 *   Equal to gh3020_spi_writereg. Used by vendor's library, thus the func
 *   name has to remain origin.
 *
 * Input Parameters:
 *   regaddr - Register address.
 *   regval  - Value to be written.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

void GH3X2X_WriteReg(uint16_t regaddr, uint16_t regval)
{
    gh3020_spi_writereg(regaddr, regval);
}

/****************************************************************************
 * Name: GH3X2X_ReadReg
 *
 * Description:
 *   Equal to gh3020_spi_readreg. Used by vendor's library, thus the function
 *   name has to remain origin.
 *
 * Input Parameters:
 *   regaddr - Register address.
 *
 * Returned Value:
 *   The register value read.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

uint16_t GH3X2X_ReadReg(uint16_t regaddr)
{
  return gh3020_spi_readreg(regaddr);
}

/****************************************************************************
 * Name: gh3020_spi_sendcmd
 *
 * Description:
 *   Send command via SPI.
 *
 * Input Parameters:
 *   cmd - The command to be send.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

void gh3020_spi_sendcmd(uint8_t cmd)
{
  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(g_priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  if (gh3020_configspi(g_priv) != OK)
    {
      snerr("SPI configuration failed.\n");
      SPI_LOCK(g_priv->config->spi, false);
      return;
    }

  gh3020_spi_exchange(g_priv, &cmd, 1);

  /* Release the SPI bus. */

  SPI_LOCK(g_priv->config->spi, false);
}

/****************************************************************************
 * Name: gh3020_spi_writereg
 *
 * Description:
 *   Write specialized value to a register of specialized address.
 *
 * Input Parameters:
 *   regaddr - Register address.
 *   regval  - Value to be written.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

void gh3020_spi_writereg(uint16_t regaddr, uint16_t regval)
{
  uint8_t buf[GH3020_SPI_WR_BUF_LEN];

  /* Write a register need: write_cmd + address + content_length + content */

  buf[0] = GH3020_SPI_CMD_WRITE;
  buf[1] = (uint8_t)((regaddr >> 8) & 0xff);
  buf[2] = (uint8_t)(regaddr & 0xff);
  buf[3] = GH3020_SPI_WR_LEN_H;
  buf[4] = GH3020_SPI_WR_LEN_L;
  buf[5] = (uint8_t)((regval >> 8) & 0xff);
  buf[6] = (uint8_t)(regval & 0xff);

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(g_priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  if (gh3020_configspi(g_priv) != OK)
    {
      snerr("SPI configuration failed.\n");
      SPI_LOCK(g_priv->config->spi, false);
      return;
    }

  gh3020_spi_exchange(g_priv, buf, GH3020_SPI_WR_BUF_LEN);

  /* Release the SPI bus. */

  SPI_LOCK(g_priv->config->spi, false);
}

/****************************************************************************
 * Name: gh3020_spi_readreg
 *
 * Description:
 *   Read a register value of specialized register address via SPI.
 *
 * Input Parameters:
 *   regaddr - Register address.
 *
 * Returned Value:
 *   The register value read.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

uint16_t gh3020_spi_readreg(uint16_t regaddr)
{
  uint8_t buf[GH3020_SPI_RD_BUF_LEN];

  buf[0] = GH3020_SPI_CMD_WRITE;
  buf[1] = (uint8_t)((regaddr >> 8) & 0xff);
  buf[2] = (uint8_t)(regaddr & 0xff);

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(g_priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  if (gh3020_configspi(g_priv) != OK)
    {
      snerr("SPI configuration failed.\n");
      SPI_LOCK(g_priv->config->spi, false);
      return 0;
    }

  /* Write the register address. */

  gh3020_spi_exchange(g_priv, buf, GH3020_SPI_RD_BUF_LEN);

  /* Read the register. */

  buf[0] = GH3020_SPI_CMD_READ;
  gh3020_spi_exchange(g_priv, buf, GH3020_SPI_RD_BUF_LEN);

  /* Release the SPI bus. */

  SPI_LOCK(g_priv->config->spi, false);

  return (uint16_t)((((uint16_t)buf[1] << 8) & 0xff00) |
                    ((uint16_t)buf[2] & 0xff));
}

/****************************************************************************
 * Name: gh3020_spi_readfifo
 *
 * Description:
 *   Read FIFO via SPI.
 *
 * Input Parameters:
 *   pbuf - Pointer to buffer for storing values readback.
 *   len  - How many bytes to be read.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

void gh3020_spi_readfifo(FAR uint8_t *pbuf, uint16_t len)
{
  uint8_t localbuf[GH3020_SPI_LOCALBUF_SIZE];

  localbuf[0] = GH3020_SPI_CMD_WRITE;
  localbuf[1] = (uint8_t)((GH3020_REG_FIFO >> 8) & 0xff);
  localbuf[2] = (uint8_t)(GH3020_REG_FIFO & 0xff);

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(g_priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  if (gh3020_configspi(g_priv) != OK)
    {
      snerr("SPI configuration failed.\n");
      SPI_LOCK(g_priv->config->spi, false);
      return;
    }

  /* Write reading address. */

  gh3020_spi_exchange(g_priv, localbuf, GH3020_SPI_RD_BUF_LEN);

  /* Prepare for reading. */

  localbuf[0] = GH3020_SPI_CMD_READ;

  /* If there are not so many bytes to read, we use local buffer for SPI
   * exchanging and copy the readout to destination buffer. Otherwise we
   * have to exchange twice, bringing a inherent delay between 2 exchanges.
   */

  if (len + 1 <= GH3020_SPI_LOCALBUF_SIZE)
    {
      gh3020_spi_exchange(g_priv, localbuf, len + 1);
      memcpy(pbuf, localbuf + 1, len);
    }
  else
    {
      /* Pull down CS pin manually */

      IOEXP_WRITEPIN(g_priv->config->ioedev, g_priv->config->gpiocs, 0);

      /* Selects the device. */

      SPI_SELECT(g_priv->config->spi, g_priv->config->cs, true);

      /* First send read command, then send arbitrary content to read a block
       * of nwords bytes.
       */

      SPI_EXCHANGE(g_priv->config->spi, localbuf, localbuf, 1);
      SPI_EXCHANGE(g_priv->config->spi, pbuf, pbuf, len);

      /* Pull up CS pin manually */

      IOEXP_WRITEPIN(g_priv->config->ioedev, g_priv->config->gpiocs, 1);

      /* Deselect the device and release the SPI bus. */

      SPI_SELECT(g_priv->config->spi, g_priv->config->cs, false);
    }

  /* Release the SPI bus. */

  SPI_LOCK(g_priv->config->spi, false);
}

/****************************************************************************
 * Name: gh3020_spi_writebits
 *
 * Description:
 *   Write specialized bits field to a register.
 *
 * Input Parameters:
 *   regaddr - Register address.
 *   lsb     - LSB position of the bits field in the register.
 *   msb     - MSB position of the bits field in the register.
 *   val     - Value of the bits field to be written.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

void gh3020_spi_writebits(uint16_t regaddr, uint8_t lsb, uint8_t msb,
                          uint16_t val)
{
    uint16_t mask = ((0x0001 << (msb - lsb + 1)) - 1) << lsb;
    uint16_t regval;

    regval = gh3020_spi_readreg(regaddr);
    GH3020_VAL_CLEAR_BIT(regval, mask);
    GH3020_VAL_SET_BIT(regval, (val << lsb) & mask);
    gh3020_spi_writereg(regaddr, regval);
}

/****************************************************************************
 * Name: gh3020_spi_readbits
 *
 * Description:
 *   Read specialized bits field from a register.
 *
 * Input Parameters:
 *   regaddr - Register address.
 *   lsb     - LSB position of the bits field in the register.
 *   msb     - MSB position of the bits field in the register.
 *
 * Returned Value:
 *   Value of the bits field to be read.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

uint16_t gh3020_spi_readbits(uint16_t regaddr, uint8_t lsb, uint8_t msb)
{
  uint16_t mask = ((0x0001 << (msb - lsb + 1)) - 1) << lsb;
  uint16_t regval;

  regval = gh3020_spi_readreg(regaddr);
  GH3020_VAL_GET_BIT(regval, mask);

  return (regval >> lsb);
}

/****************************************************************************
 * Name: gh3020_rstctrl
 *
 * Description:
 *   Control the level of reset pin.
 *
 * Input Parameters:
 *   pinlevel - Desired level of reset pin.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

void gh3020_rstctrl(uint8_t pinlevel)
{
  if (pinlevel == 0)
    {
      IOEXP_WRITEPIN(g_priv->config->ioerpmsg, g_priv->config->rstpin, 0);
    }
  else
    {
      IOEXP_WRITEPIN(g_priv->config->ioerpmsg, g_priv->config->rstpin, 1);
    }
}

/****************************************************************************
 * Name: gh3020_get_efuse
 *
 * Description:
 *   Get EFUSE for calibration.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

uint16_t gh3020_get_efuse(void)
{
  if (g_priv->load_efuse == false)
    {
      int8_t tia_efuse;
      int8_t led_efuse;

      /* Enable EFUSE and load value into the register. */

      gh3020_spi_writereg(GH3020_REG_EFUSE_CTRL_CMD,
                          GH3020_MSK_EFUSE_CTRL_CMD_EN);
      gh3020_spi_writereg(GH3020_REG_EFUSE_CTRL_CMD,
                          GH3020_MSK_EFUSE_CTRL_CMD_EN |
                          GH3020_MSK_EFUSE_CTRL_CMD_START);

      /* Wait until it's ready, then read it. */

      while (!(gh3020_spi_readreg(GH3020_REG_EFUSE_CTRL_STATUS) &
               GH3020_MSK_EFUSE_CTRL_STATUS_DONE));
      g_priv->efuse = gh3020_spi_readreg(GH3020_REG_EFUSE_CTRL_RDATA_1);

      /* Disable EFUSE clock to save energy. */

      gh3020_spi_writereg(GH3020_REG_EFUSE_CTRL_CMD, 0);

      /* Mark EFUSE as loaded. Calculate TIA calibration coefficent. */

      tia_efuse = (int8_t)(g_priv->efuse >> GH3020_OFFSET_EFUSE1_TIA);
      led_efuse = (int8_t)(g_priv->efuse & GH3020_MSK_EFUSE_CTRL_RDATA1_LED);
      g_priv->tia_calibr = 1.0f / (1.0f + (float)tia_efuse / 512.0f);
      g_priv->led_calibr = 1.0f + (float)led_efuse / 256.0f;
      g_priv->load_efuse = true;
    }

  return g_priv->efuse;
}

/****************************************************************************
 * Name: gh3020_get_ppg_data
 *
 * Description:
 *   Get PPG data from the info of frame readback.
 *
 * Input Parameters:
 *   pframeinfo - Pointer to the info of frame readback.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

void gh3020_get_ppg_data(FAR const struct gh3020_frameinfo_s *pframeinfo)
{
  uint8_t idx;

  switch (pframeinfo->unFunctionID)
    {
      case GH3X2X_FUNCTION_HR:        /* ppg0: green dynamic */
        {
          idx = GH3020_PPG0_IDX;
        }
        break;

      case GH3X2X_FUNCTION_HSM:       /* ppg5: IR fixed */
        {
          idx = GH3020_PPG5_IDX;
        }
        break;

      case GH3X2X_FUNCTION_SPO2:      /* ppg1: red dynamic */
        {
          idx = GH3020_PPG1_IDX;
        }
        break;

      case GH3X2X_FUNCTION_HRV:       /* ppg2: IR dynamic */
        {
          idx = GH3020_PPG2_IDX;
        }
        break;

      default:                        /* No such data */
        return;
    }

  GH3020_DEBUG_LOG("[gh3020] get ppg%u data hook\n",
                   gh3020_node_name_list[idx]);
  if (g_priv->sensor[idx].activated == true &&
      g_priv->ppgdatacnt[idx] < GH3020_BATCH_NUMBER)
    {
      gh3020_extract_frame(g_priv, idx,
                           &g_priv->ppgdata[idx][g_priv->ppgdatacnt[idx]],
                           pframeinfo);
      g_priv->ppgdatacnt[idx]++;
    }
}

/****************************************************************************
 * Name: gh3020_get_rawdata
 *
 * Description:
 *   Get PPG data from the FIFO in factest mode.
 *
 * Input Parameters:
 *   pbuf - Pointer to the FIFO readback buffer.
 *   len  - Length of the FIFO readback buffer.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_FACTEST_SENSORS_GH3020
void gh3020_get_rawdata(FAR uint8_t *pbuf, uint16_t len)
{
  struct sensor_ppgq ppg[4];
  struct gh3020_fifodata_s fifoinfo;
  uint32_t temp;
  int32_t rawdata;
  uint8_t chidx;
  uint8_t i;

  if (len / GH3020_BYTES_PER_DATA > 0 && g_priv->factest_mode == true)
    {
      for (i = 0; i < len; i = i + GH3020_BYTES_PER_DATA)
        {
          /* Swap big endian to little endian */

          temp = ((uint32_t)pbuf[i + 0] << 24) +
                 ((uint32_t)pbuf[i + 1] << 16) +
                 ((uint32_t)pbuf[i + 2] << 8) + (uint32_t)pbuf[i + 3];

          /* Pick rawdata and flag */

          fifoinfo.adccode = temp & 0x00ffffff;
          fifoinfo.slot_idx = (temp >> 29) & 0x00000007;
          fifoinfo.adc_idx = (temp >> 27) & 0x00000003;
          fifoinfo.flag_ledadj_ecgrecover = (temp >> 26) & 0x00000001;
          fifoinfo.flag_ledadj_agc_up = (temp >> 25) & 0x00000001;

          rawdata = (int32_t)(fifoinfo.adccode) - GH3020_ADC_BASELINE;
          if (rawdata < 0)
            {
              rawdata = 0;
            }

          switch (fifoinfo.slot_idx)
            {
              case 0:       /* slot0 - dark */
                {
                  chidx = GH3020_PPG3_FACTEST_IDX;
                }
                break;

              case 1:       /* slot1 - green */
                {
                  chidx = GH3020_PPG0_FACTEST_IDX;
                }
                break;

              case 2:       /* slot2 - red */
                {
                  chidx = GH3020_PPG1_FACTEST_IDX;
                }
                break;

              case 3:       /* slot3 - ir */
                {
                  chidx = GH3020_PPG2_FACTEST_IDX;
                }
                break;

              default:      /* No such data */
                {
                  chidx = GH3020_SENSOR_NUM;
                }
                break;
            }

          if (g_priv->sensor[chidx].activated == true)
            {
              syslog(LOG_INFO, "[gh3020] ch%uraw%u=%08lx\n", chidx,
                     fifoinfo.adc_idx, temp);
            }

          if (chidx < GH3020_SENSOR_NUM)
            {
              ppg[chidx].ppg[fifoinfo.adc_idx] = (uint32_t)rawdata;
              ppg[chidx].current = g_priv->sensor[chidx].current;
              ppg[chidx].gain[fifoinfo.adc_idx] =
                gh3020_gain_list[GH3020_TIAGAIN_FACTEST];
            }
        }

      for (chidx = 0; chidx < GH3020_FACTEST_SENSOR_NUM; chidx++)
        {
          if (g_priv->sensor[chidx].activated == true &&
              g_priv->ppgdatacnt[chidx] < GH3020_BATCH_NUMBER)
            {
              g_priv->ppgdata[chidx][g_priv->ppgdatacnt[chidx]] = ppg[chidx];
              g_priv->ppgdatacnt[chidx]++;
            }
        }
    }
}
#endif  /* CONFIG_FACTEST_SENSORS_GH3020 */

/****************************************************************************
 * Name: gh3020_register
 *
 * Description:
 *   Register the GH3020 character device.
 *
 * Input Parameters:
 *   devno  - The full path to the driver to register. E.g., "/dev/gh3020"
 *   config - An instance of the SPI interface to use to communicate with
 *            gh3020
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int gh3020_register(int devno, FAR const struct gh3020_config_s *config)
{
  FAR struct gh3020_dev_s *priv;
  FAR void *ioehandle;
  uint8_t idx;
  int ret;

  /* Sanity check */

  DEBUGASSERT(config != NULL);

  /* Initialize the GH3020 device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (!priv)
    {
      snerr("Failed to allocate instance.\n");
      return -ENOMEM;
    }

  g_priv = priv;
  priv->config = config;
  for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
    {
      priv->sensor[idx].lower.ops = &g_gh3020_ops;
      priv->sensor[idx].lower.type = SENSOR_TYPE_PPGQ;
      priv->sensor[idx].lower.nbuffer = GH3020_BATCH_NUMBER;
      priv->sensor[idx].dev = priv;
      priv->sensor[idx].interval = GH3020_INTVL_DFT;
      priv->sensor[idx].batch = GH3020_INTVL_DFT;
      priv->sensor[idx].current = GH3020_CURRENT_DFT;
      priv->sensor[idx].chidx = (uint8_t)idx;
    }

  /* Initialize the device. After that, it will enter low-power mode(10uA).
   * Note that gh3020_init will return 0 for OK or a positive error code.
   */

  gh3020_rdmode_switch(GH3020_RDMODE_INTERRPT);
  ret = gh3020_init();
  if (ret != OK)
    {
      ret = -ret;
      snerr("Device ID doesn't match: %d\n", ret);
      goto err_exit;
    }

  /* Configure interrupt pin */

  ret = IOEXP_SETDIRECTION(priv->config->ioedev, priv->config->intpin,
                           IOEXPANDER_DIRECTION_IN);
  if (ret < 0)
    {
      snerr("Failed to set direction: %d\n", ret);
      goto err_exit;
    }

  ioehandle = IOEP_ATTACH(priv->config->ioedev, priv->config->intpin,
                          gh3020_interrupt_handler, priv);
  if (ioehandle == NULL)
    {
      ret = -EIO;
      snerr("Failed to attach: %d\n", ret);
      goto err_exit;
    }

  ret = IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                        IOEXPANDER_OPTION_INTCFG,
                        (FAR void *)IOEXPANDER_VAL_DISABLE);
  if (ret < 0)
    {
      snerr("Failed to set option: %d\n", ret);
      goto err_iodetach;
    }

  /* Register the character driver */

  for (idx = 0; idx < GH3020_SENSOR_NUM; idx++)
    {
      ret = sensor_register(&(priv->sensor[idx].lower),
                            devno * GH3020_SENSOR_NUM +
                            gh3020_node_name_list[idx]);
      if (ret < 0)
        {
          snerr("Failed to register GH3020 ppgq%d driver: %d\n",
                devno * GH3020_SENSOR_NUM + gh3020_node_name_list[idx], ret);

          /* Unregister all registered ppgq sensors */

          while (idx)
            {
              /* PPG5 is the last node. If its registion failed, we need not
               * to deal this special case (i.e. PPG5's topic_idx != idx)
               */

              idx--;
              sensor_unregister(&priv->sensor[idx].lower,
                                devno * GH3020_SENSOR_NUM +
                                gh3020_node_name_list[idx]);
            }

          goto err_iodetach;
        }
    }

  syslog(LOG_INFO, "[gh3020] driver v2.1.16, build 2022-07-12\n");

  return ret;

err_iodetach:
  IOEP_DETACH(priv->config->ioedev, gh3020_interrupt_handler);

err_exit:
  kmm_free(priv);
  return ret;
}
