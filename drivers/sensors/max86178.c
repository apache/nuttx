/****************************************************************************
 * drivers/sensors/max86178.c
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

#include <sys/types.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/max86178.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/wqueue.h>

#include "max86178_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configurations */

#define MAX86178_FIFO_SLOTS_PPG  56         /* Maximum slots for PPG (2-ch) */
#define MAX86178_FIFO_SLOTS_ECG  28         /* Maximum slots for ECG */
#define MAX86178_INIT_TABLE_SIZE 11         /* Size of initializing table */

/* PPG optimization */

#define MAX86178_PPG_OP_HIGH0    983040     /* Upper. ADC range0 (4uA) */
#define MAX86178_PPG_OP_MID0     950272     /* Middle. ADC range0 (4uA) */
#define MAX86178_PPG_OP_SPAN0    24576      /* Span. ADC range0 (4uA) */
#define MAX86178_PPG_OP_LOW0     917504     /* Lower. ADC range0 (4uA) */
#define MAX86178_PPG_OP_HIGH1    983040     /* Upper. ADC range1 (8uA) */
#define MAX86178_PPG_OP_MID1     786432     /* Middle. ADC range1 (8uA) */
#define MAX86178_PPG_OP_SPAN1    65536      /* Span. ADC range1 (8uA) */
#define MAX86178_PPG_OP_LOW1     524288     /* Lower. ADC range1 (8uA) */
#define MAX86178_PPG_OP_HIGH2    524288     /* Upper. ADC range1 (16uA) */
#define MAX86178_PPG_OP_MID2     393216     /* Middle. ADC range2 (16uA) */
#define MAX86178_PPG_OP_SPAN2    32768      /* Span. ADC range1 (16uA) */
#define MAX86178_PPG_OP_LOW2     262144     /* Lower. ADC range2 (16uA) */
#define MAX86178_PPG_OP_HIGH3    262144     /* Upper. ADC range2 (32uA) */
#define MAX86178_PPG_OP_MID3     196608     /* Middle. ADC range1 (32uA) */
#define MAX86178_PPG_OP_SPAN3    16384      /* Span. ADC range1 (32uA) */
#define MAX86178_PPG_OP_LOW3     131072     /* Lower. ADC range1 (32uA) */
#define MAX86178_PPG_OP_TRIG     10         /* PPG optimization trigger */
#define MAX86178_PPG_OP_WINDOW   5          /* PPG optimization slide size */
#define MAX86178_PPG_OP_TIMES    10         /* Max optimization try times */

/* Digital interface parameters */

#ifdef CONFIG_SENSORS_MAX86178_I2C
# define MAX86178_I2C_MAX_FREQ   400000     /* Maximum SCL frequency 400kHz */
# define MAX86178_I2C_MIN_FREQ   100000     /* Minimum SCL frequency 100kHz */
# define MAX86178_I2C_ADDR_LEN   7          /* Only support 7bit address */

#else /* CONFIG_SENSORS_MAX86178_SPI */
# define MAX86178_SPI_MAX_FREQ   25000000   /* Maximum SCLK 25MHz */
# define MAX86178_SPI_MIN_FREQ   100000     /* Minimum SCLK 100kHz */
# define MAX86178_SPI_NBITS      8          /* Only support 8 bits per word */

#endif

/* MAX86178 AFE contains ECG, PPG and BioZ AFE. CUrrently ECG and PPG is
 * used. ECG has only one channel, while PPG has 4 different measurements and
 * each one should be a sensor node.
 */

#define MAX86178_PPG0_SENSOR_IDX 0          /* PPG0 (green) sensor index. */
#define MAX86178_PPG1_SENSOR_IDX 1          /* PPG1 (red) sensor index. */
#define MAX86178_PPG2_SENSOR_IDX 2          /* PPG2 (IR sensor index. */
#define MAX86178_PPG3_SENSOR_IDX 3          /* PPG3 (dark) sensor index. */
#define MAX86178_PPG_SENSOR_NUM  4          /* Total PPG sensors number. */

/* Control commands */

#define MAX86178_CTRL_CHECKID    0          /* Check device ID. */
#define MAX86178_ECG_CTRL_GAIN   0x90       /* Set ECG gain. */
#define MAX86178_PPG_CTRL_LEDPA  0x90       /* Set PPG LED current. */
#define MAX86178_PPG_CTRL_GAIN1  0x91       /* Set PPG ADC1's gain. */
#define MAX86178_PPG_CTRL_GAIN2  0x92       /* Set PPG ADC2's gain. */
#define MAX86178_PPG_CTRL_OPTM   0x93       /* Set PPG optimization ON/OFF */

/* Default settings */

/* The settings below are involved with each other and some other register
 * values which are not listed here. It means that ECG, PPG and BioZ sample
 * rates can't be set easily. Later we will add a function to calculate them
 * for any case, or seek a best one in an list of some combinations of these
 * settings, or set them manually in menuconfig. We're not sure which method
 * is most suitable, thus currently their values are limited.
 */

#define MAX86178_ECG_INTVL_DFT   4000       /* Default ECG interval = 4 ms */
#define MAX86178_PPG_INTVL_DFT   40000      /* Default PPG interval = 40 ms */
#define MAX86178_PPG_FRDIV_DFT   1280       /* FRDIV = 12800 i.e. FPS=25Hz */
#define MAX86178_PPG_GLEDPA_DFT  80         /* Green LED 80*0.125=10mA */
#define MAX86178_PPG_RLEDPA_DFT  200        /* Red LED 200*0.25=50mA */
#define MAX86178_PPG_IRLEDPA_DFT 160        /* IR LED 160*0.25=40mA */
#define MAX86178_PPG_GLEDC_DFT   10000      /* Green LED current=10000uA */
#define MAX86178_PPG_RLEDC_DFT   50000      /* Red LED current=50000uA */
#define MAX86178_PPG_IRLEDC_DFT  40000      /* IR LED current=40000uA */
#define MAX86178_PPG_ADCRGE_DFT  2          /* ADC range2 16uA i.e. gain=2 */
#define MAX86178_PPG_GAIN_DFT    2          /* Gain=2, i.e. ADC range2 16uA */
#define MAX86178_MDIV_DFT        125        /* Default MDIV for PLL = 4 MHz */
#define MAX86178_REF_CLK_DFT     32000.0f   /* Default REF_CLK = 32kHz */

/* Default for ECG_PLL = 4 MHz */

#define MAX86178_ECG_NDIV_DFT    125        /* ECG_ADC_CLK = 32kHz */
#define MAX86178_ECG_FDIV_DFT    1          /* ECG_PLL_CLK = PLL_CLK */
#define MAX86178_ECG_PLL_DFT     4000000.0f /* ECG_PLL_CLK = 4MHz */

/* The ECG range settings should vary with practices according to the
 * hardware design, and is determined through experiments.
 */

#define MAX86178_ECG_INARGE_DFT  MAX86178_ECGCFG2_INARGE_0
#define MAX86178_ECG_INAGAIN_DFT MAX86178_ECGCFG2_INAGAIN_1
#define MAX86178_ECG_PGAGAIN_DFT MAX86178_ECGCFG2_PGAGAIN_4
#define MAX86178_ECG_GAIN_DFT    0.095367431640625f

/* Constant parameters */

#define MAX86178_ECG_SR_MAX      2048       /* ECG sample rate <= 2048Hz. */
#define MAX86178_ECG_SR_MIN      250        /* ECG sample rate >= 250Hz. */
#define MAX86178_ECG_ADC_CLK_MAX 32768      /* ECG_ADC_CLK = 32768 Hz max. */
#define MAX86178_ECG_ADC_CLK_MIN 19000      /* ECG_ADC_CLK = 19.0kHz min. */
#define MAX86178_ECG_DECRATE_NUM 6          /* ECG_DEC_RATE has 6 choices. */
#define MAX86178_ECG_NEGATIVE    0xfffc0000 /* Prefix for negative ECG */
#define MAX86178_PPG_LEDPASTEP   32000u     /* PPG LED PA range step=32mA */
#define MAX86178_PPG_LEDPAMAX    127500u    /* PPG LED PA <= 127500uA */
#define MAX86178_PPG_LEDLSBSTEP  125u       /* PPG LED PA LSB step = 125uA */

#define MAX86178_ONE_SECOND      1000000.0f /* 1 second = 1000000 us */

#define MAX86178_CEILING(x, y)   ((x) + ((y)- 1)) / (y)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* MAX86178 register address and values struct. */

struct max86178_reg_table_s
{
  uint8_t regaddr;                       /* Register address */
  uint8_t regval;                        /* Register value */
};

/* PPG optimization struct */

struct max86178_ppg_optim_s
{
  uint32_t total;
  uint8_t up_trigger;
  uint8_t low_trigger;
  uint8_t adc_range;
  uint8_t cnt;
  uint8_t times;
  bool trigger;
};

/* PPG optimization aimed parameters struct */

struct max86178_ppg_optim_param_s
{
  uint32_t upper;
  uint32_t lower;
  uint32_t middle;
  uint32_t span;
};

/* ECG sensor struct */

struct max86178_ecg_sensor_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s lower;       /* Lower half sensor driver */
  FAR struct max86178_dev_s *dev;        /* Point to the device struct */
  unsigned long interval_desired;        /* Desired sample interval(us) */
  unsigned long batch_desired;           /* Desired batch latency (us) */
  float factor;                          /* ECG = ADC counts * factor */
  uint16_t ndiv;                         /* Sample rate=PLL/(ndiv*decrate) */
  uint8_t decrate;                       /* Sample rate=PLL/(ndiv*decrate) */
  bool activating;                       /* If it will be activated soon */
  bool inactivating;                     /* If it will be inactivated soon */
  bool activated;                        /* If it's activated now. */
};

/* Sensor struct */

struct max86178_ppg_sensor_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s lower;       /* Lower half sensor driver */
  FAR struct max86178_dev_s *dev;        /* Point to the device struct */
  struct max86178_ppg_optim_s optm1;     /* For PPG ADC1 optimization */
  struct max86178_ppg_optim_s optm2;     /* For PPG ADC2 optimization */
  unsigned long interval_desired;        /* Desired sensor interval */
  unsigned long batch_desired;           /* Desired sensor batch latency */
  uint32_t current;                      /* LED driver current (uA) */
  uint16_t frdiv;                        /* PPG fps = ppg_ref_clk / frdiv */
  uint16_t samples_per_meas;             /* N samples in a measurement */
  uint8_t gain1;                         /* PPG ADC1 gain */
  uint8_t gain2;                         /* PPG ADC2 gain */
  uint8_t chidx;                         /* PPG channel index */
  bool activating;                       /* If it will be activated soon. */
  bool inactivating;                     /* If it will be inactivated soon. */
  bool activated;                        /* If it's activated now. */
  bool auto_optimize;                    /* If auto-optimization is used. */
};

/* Device struct */

struct max86178_dev_s
{
  /* PPG sensors structures. */

  struct max86178_ppg_sensor_s ppg_sensor[MAX86178_PPG_SENSOR_NUM];

  /* ECG sensor structure */

  struct max86178_ecg_sensor_s ecg_sensor;
  uint64_t timestamp;                    /* Current timestamp (us) */
  uint64_t timestamp_prev;               /* Last timestampe (us) */
  struct work_s work_intrpt;             /* Interrupt worker */
  struct work_s work_poll;               /* Polling worker */

  /* Device configuration struct pointer. */

  FAR const struct max86178_config_s *config;

  /* Buffer to store PPGs data for pushing. */

  struct sensor_ppgd ppgdata[MAX86178_PPG_SENSOR_NUM]
                                  [MAX86178_FIFO_SLOTS_PPG];

  /* Buffer to store ECG data for pushing. */

  struct sensor_ecg ecgdata[MAX86178_FIFO_SLOTS_ECG];

  /* Buffer for reading FIFO. MAX86178 has a 256-sample (3Byte/sample) FIFO */

  uint8_t fifobuf[MAX86178_FIFO_SIZE_BYTES];
  unsigned long interval;                /* Device's sample interval */
  unsigned long batch;                   /* Device's batch latency */
  uint32_t fifowtm;                      /* Total FIFO water marker */
  uint8_t ppg_activated;                 /* How many PPGs are activated */
  bool update_interval;                  /* Any sensor is updating interval */
  bool update_batch;                     /* Any sensor is updating batch */
  bool update_activate;                  /* Any sensor is updating activate */
  bool activated;                        /* If any sensor is activated */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* Interface (I2C/SPI) functions */

#ifdef CONFIG_SENSORS_MAX86178_SPI
static int max86178_configspi(FAR struct max86178_dev_s *priv);
#endif
static int max86178_readregs(FAR struct max86178_dev_s *priv,
                             uint8_t startaddr, FAR uint8_t *recvbuf,
                             size_t nbytes);
static int max86178_writeregs(FAR struct max86178_dev_s *priv,
                              uint8_t startaddr, FAR uint8_t *sendbuf,
                              size_t nbytes);
static int max86178_readsingle(FAR struct max86178_dev_s *priv,
                               uint8_t regaddr,  uint8_t *regval);
static int max86178_writesingle(FAR struct max86178_dev_s *priv,
                                uint8_t regaddr, uint8_t regval);

/* MAX86178 common handle functions */

static void max86178_batch_calcu(FAR struct max86178_dev_s *priv,
                                 FAR unsigned long *latency_us,
                                 uint32_t batch_number, unsigned long interval);
static int max86178_checkid(FAR struct max86178_dev_s *priv);
static void max86178_enable(FAR struct max86178_dev_s *priv, bool enable);
static void max86178_fifo_flush(FAR struct max86178_dev_s *priv);
static int max86178_fifo_read(FAR struct max86178_dev_s *priv);
static void max86178_fifo_set(FAR struct max86178_dev_s *priv);
static void max86178_init(FAR struct max86178_dev_s *priv);
static int max86178_selftest(FAR struct max86178_dev_s *priv,
                             unsigned long arg);
static void max86178_shutdown(FAR struct max86178_dev_s *priv);
static void max86178_softreset(FAR struct max86178_dev_s *priv);
static void max86178_update_sensors(FAR struct max86178_dev_s *priv);

/* MAX86178 ECG handle functions */

static void max86178_ecg_calcu_clk(FAR struct max86178_ecg_sensor_s *sensor,
                                   FAR unsigned long *interval);
static float max86178_ecg_calcudata(FAR struct max86178_ecg_sensor_s *sensor,
                                    uint32_t sample);
static int max86178_ecg_enable(FAR struct max86178_dev_s *priv, bool enable);
static void max86178_ecg_set_sr(FAR struct max86178_dev_s *priv,
                                uint8_t decrate, uint16_t ndiv);

/* MAX86178 PPG handle functions */

static uint32_t max86178_ppg_calcudata(uint32_t sample);
static uint16_t max86178_ppg_calcu_frdiv(unsigned long interval);
static void max86178_ppg_dealsample(FAR struct max86178_dev_s *priv,
                                    uint8_t chidx, uint32_t sample,
                                    FAR uint32_t *count,
                                    FAR uint8_t *toggle);
static void max86178_ppg_enable(FAR struct max86178_dev_s *priv,
                                uint8_t chidx, bool enable);
static void max86178_ppg_optimize(FAR struct max86178_ppg_sensor_s *sensor);
static void max86178_ppg_optim_record(FAR struct max86178_ppg_optim_s *optim,
                                      uint32_t ppg);
static void max86178_ppg_set_current(FAR struct max86178_ppg_sensor_s
                                     *sensor, FAR uint32_t *current);
static void max86178_ppg_set_fps(FAR struct max86178_dev_s *priv,
                                 uint16_t frdiv);

/* ECG sensor ops functions */

static int max86178_ecg_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable);
static int max86178_ecg_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR unsigned long *period_us);
static int max86178_ecg_batch(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR unsigned long *latency_us);
static int max86178_ecg_selftest(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 unsigned long arg);
static int max86178_ecg_control(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                int cmd, unsigned long arg);

/* PPG sensor ops functions */

static int max86178_ppg_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable);
static int max86178_ppg_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR unsigned long *period_us);
static int max86178_ppg_batch(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR unsigned long *latency_us);
static int max86178_ppg_selftest(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 unsigned long arg);
static int max86178_ppg_control(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                int cmd, unsigned long arg);

/* Sensor interrupt functions */

static int  max86178_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                       ioe_pinset_t pinset, FAR void *arg);
static void max86178_worker_intrpt(FAR void *arg);
static void max86178_worker_poll(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Register address and values for initializing. */

static const struct max86178_reg_table_s
  max86178_init_table[MAX86178_INIT_TABLE_SIZE] =
  {
    /* MEAS1 selects LED driver A to drive LED3(green) for PPG ch0. */

    {
      MAX86178_REG_MEAS1SEL, MAX86178_MEASXSEL_DRVA_LED3
    },

    /* MEAS2 selects LED driver A to drive LED2(red) for PPG ch1. */

    {
      MAX86178_REG_MEAS2SEL, MAX86178_MEASXSEL_DRVA_LED2
    },

    /* MEAS3 selects LED driver B to drive LED1(IR) for PPG ch2. */

    {
      MAX86178_REG_MEAS3SEL, MAX86178_MEASXSEL_DRVB_LED1
    },

    /* MEAS4 works in direct ambient mode for PPG ch3. */

    {
      MAX86178_REG_MEAS4SEL, MAX86178_MEASXSEL_AMB_EN
    },

    /* For MEAS1(PPG ch0), PPG1 selects PD1&3， PPG2 selects PD2&4. */

    {
      MAX86178_REG_MEAS1CFG5, MAX86178_MEASXCFG5_PD1SEL_PPG1 |
                              MAX86178_MEASXCFG5_PD3SEL_PPG1 |
                              MAX86178_MEASXCFG5_PD2SEL_PPG2 |
                              MAX86178_MEASXCFG5_PD4SEL_PPG2
    },

    /* For MEAS2(PPG ch1), PPG1 selects PD1&3， PPG2 selects PD2&4. */

    {
      MAX86178_REG_MEAS2CFG5, MAX86178_MEASXCFG5_PD1SEL_PPG1 |
                              MAX86178_MEASXCFG5_PD3SEL_PPG1 |
                              MAX86178_MEASXCFG5_PD2SEL_PPG2 |
                              MAX86178_MEASXCFG5_PD4SEL_PPG2
    },

    /* For MEAS3(PPG ch2), PPG1 selects PD1&3， PPG2 selects PD2&4. */

    {
      MAX86178_REG_MEAS3CFG5, MAX86178_MEASXCFG5_PD1SEL_PPG1 |
                              MAX86178_MEASXCFG5_PD3SEL_PPG1 |
                              MAX86178_MEASXCFG5_PD2SEL_PPG2 |
                              MAX86178_MEASXCFG5_PD4SEL_PPG2
    },

    /* For MEAS4(PPG ch3), PPG1 selects PD1&3， PPG2 selects PD2&4. */

    {
      MAX86178_REG_MEAS4CFG5, MAX86178_MEASXCFG5_PD1SEL_PPG1 |
                              MAX86178_MEASXCFG5_PD3SEL_PPG1 |
                              MAX86178_MEASXCFG5_PD2SEL_PPG2 |
                              MAX86178_MEASXCFG5_PD4SEL_PPG2
    },

    /* ECG_PLL_CLK = PLL_CLK. */

    {
      MAX86178_REG_PLLCFG4, MAX86178_ECG_FDIV_DFT
    },

    /* REF CLK comes from internal 32.0kHz OSC. */

    {
      MAX86178_REG_PLLCFG6, MAX86178_PLLCFG6_REFCLK_32K
    },

    /* FIFO rolls over and FIFO interrupt cleared by status or FIFO reading */

    {
      MAX86178_REG_FIFOCFG2, MAX86178_FIFOCFG2_STATCLR |
                             MAX86178_FIFOCFG2_ROLL
    }
  };

/* PPG optimization parameters for 4 ADC ranges */

static const struct max86178_ppg_optim_param_s
  max86178_ppg_optim_param_table[4] =
  {
    /* ADC range0 (4uA) */

    {
      MAX86178_PPG_OP_HIGH0, MAX86178_PPG_OP_LOW0, MAX86178_PPG_OP_MID0,
      MAX86178_PPG_OP_SPAN0
    },

    /* ADC range1 (8uA) */

    {
      MAX86178_PPG_OP_HIGH1, MAX86178_PPG_OP_LOW1, MAX86178_PPG_OP_MID1,
      MAX86178_PPG_OP_SPAN1
    },

    /* ADC range2 (16uA) */

    {
      MAX86178_PPG_OP_HIGH2, MAX86178_PPG_OP_LOW2, MAX86178_PPG_OP_MID2,
      MAX86178_PPG_OP_SPAN2
    },

    /* ADC range3 (32uA) */

    {
      MAX86178_PPG_OP_HIGH3, MAX86178_PPG_OP_LOW3, MAX86178_PPG_OP_MID3,
      MAX86178_PPG_OP_SPAN3
    }
  };

static const struct sensor_ops_s g_max86178_ecg_ops =
{
  .activate     = max86178_ecg_activate,
  .set_interval = max86178_ecg_set_interval,
  .batch        = max86178_ecg_batch,
  .selftest     = max86178_ecg_selftest,
  .control      = max86178_ecg_control,
};

static const struct sensor_ops_s g_max86178_ppg_ops =
{
  .activate     = max86178_ppg_activate,
  .set_interval = max86178_ppg_set_interval,
  .batch        = max86178_ppg_batch,
  .selftest     = max86178_ppg_selftest,
  .control      = max86178_ppg_control,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max86178_configspi
 *
 * Description:
 *   Set SPI mode, frequency and bits per word, according to max86178_dev_s->
 *   max86178_config_s.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_MAX86178_SPI
static int max86178_configspi(FAR struct max86178_dev_s *priv)
{
  int freq = priv->config->freq;

  /* Set SPI frequency in an acceptable range. */

  if (freq > MAX86178_SPI_MAX_FREQ)
    {
      freq = MAX86178_SPI_MAX_FREQ;
    }
  else if (freq < MAX86178_SPI_MIN_FREQ)
    {
      freq = MAX86178_SPI_MIN_FREQ;
    }

  if (SPI_SETFREQUENCY(priv->config->spi, freq) != freq)
    {
      return -EIO;
    }

  /* MAX86178 SPI supports only mode0. */

  SPI_SETMODE(priv->config->spi, SPIDEV_MODE0);

  /* Set number of bits per word. */

  SPI_SETBITS(priv->config->spi, MAX86178_SPI_NBITS);

  return OK;
}
#endif

/****************************************************************************
 * Name: max86178_readregs
 *
 * Description:
 *   Read MAX86178 registers from a start address. Except FIFODATA, each
 *   register has 8 bits and the register address will automatically increase
 *   until 0xff. Reading FIFODATA is an exception, where the register address
 *   will stay at FIFODATA(0x0c) and the samples in FIFO will be read one by
 *   one. In FIFO reading, note that each sample has 3 bytes.
 *
 * Input Parameters:
 *   priv      - Device struct.
 *   startaddr - Address of the 1st register to be read.
 *   recvbuf   - A pointer to the buffer which stores the readouts.
 *   nwords    - The numbers of word (8bits per word) to be read.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int max86178_readregs(FAR struct max86178_dev_s *priv,
                             uint8_t startaddr, FAR uint8_t *recvbuf,
                             size_t nwords)
{
  int ret;

#ifdef CONFIG_SENSORS_MAX86178_I2C
  struct i2c_config_s config;
  int freq = priv->config->freq;

  /* Set up the I2C configuration */

  if (freq > MAX86178_I2C_MAX_FREQ)
    {
      freq = MAX86178_I2C_MAX_FREQ;
    }
  else if (freq < MAX86178_I2C_MIN_FREQ)
    {
      freq = MAX86178_I2C_MIN_FREQ;
    }

  config.frequency = freq;
  config.address = priv->config->i2c_addr;
  config.addrlen = MAX86178_I2C_ADDR_LEN;

  /* After slave address with W, it send 1 byte of start address. Then it
   * repeats a start, sends slave address with R, and finally reads nbytes.
   */

  ret = i2c_writeread(priv->config->i2c, &config, &startaddr, 1,
                      recvbuf, nwords);
  if (ret < 0)
    {
      snerr("I2C writeread failed: %d\n", ret);
    }

  return ret;

#else /* CONFIG_SENSORS_MAX86178_SPI */

  /* For reading, the 1st byte is start address and the following bit is 1. */

  uint8_t txbuf[2] = {
    startaddr, 0x80,
    };

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  ret = max86178_configspi(priv);
  if (ret < 0)
    {
      snerr("SPI configuration failed: %d\n", ret);
      SPI_LOCK(priv->config->spi, false);
      return ret;
    }

  /* Selects the device. Set CS (as a GPIO) low. */

  SPI_SELECT(priv->config->spi, priv->config->cs, true);
  IOEXP_WRITEPIN(priv->config->ioedev, priv->config->gpiocs, 0);

  /* First send start address and 0x80(means reading). Then sends arbitrary
   * content (nwords bytes) to read a block of nwords bytes.
   */

  SPI_EXCHANGE(priv->config->spi, txbuf, txbuf, 2);
  SPI_EXCHANGE(priv->config->spi, recvbuf, recvbuf, nwords);

  /* Deselect the device, set CS high and release the SPI bus. */

  IOEXP_WRITEPIN(priv->config->ioedev, priv->config->gpiocs, 1);
  SPI_SELECT(priv->config->spi, priv->config->cs, false);
  SPI_LOCK(priv->config->spi, false);

  return OK;
#endif
}

/****************************************************************************
 * Name: max86178_writeregs
 *
 * Description:
 *   Write MAX86178 registers from a start address. The register address will
 *   automatically increase until 0xFF. DO NOT use on FIFODATA register. The
 *   datasheet didn't describe what will happen when writing several bytes
 *   to FIFODATA register. It's not sure if the address will increase or not.
 *
 * Input Parameters:
 *   priv      - Device struct.
 *   startaddr - Address of the 1st register to be written.
 *   sendbuf   - A pointer to the buffer which stores data to be written.
 *   nwords    - The numbers of word (8bits per word) to be written.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int max86178_writeregs(FAR struct max86178_dev_s *priv,
                              uint8_t startaddr, FAR uint8_t *sendbuf,
                              size_t nwords)
{
  int ret;

#ifdef CONFIG_SENSORS_MAX86178_I2C
  struct i2c_config_s config;
  int freq = priv->config->freq;

  /* Set up the I2C configuration */

  if (freq > MAX86178_I2C_MAX_FREQ)
    {
      freq = MAX86178_I2C_MAX_FREQ;
    }
  else if (freq < MAX86178_I2C_MIN_FREQ)
    {
      freq = MAX86178_I2C_MIN_FREQ;
    }

  config.frequency = freq;
  config.address = priv->config->i2c_addr;
  config.addrlen = MAX86178_I2C_ADDR_LEN;

  /* After slave address with W, it sends 1 byte of start address without a
   * STOP. When rbuflen, the i2c_writeread()'s parameter, is a negative, the
   * second i2c message didn't have a START condition and is a continuation
   * of the previous transfer without a STOP. So it will send |rbuflen| bytes
   * then send a STOP condition. See nuttx/drivers/i2c/i2c_wrtieread.c and
   * nuttx/include/nuttx/i2c/i2c_master.h for detail.
   */

  ret = i2c_writeread(priv->config->i2c, &config, &startaddr, 1,
                      sendbuf, -nwords);
  if (ret < 0)
    {
      snerr("I2C write failed: %d\n", ret);
    }

  return ret;

#else /* CONFIG_SENSORS_MAX86178_SPI */

  /* For writing, the 1st byte is start address and the following bit is 0. */

  uint8_t txbuf[2] = {
    startaddr, 0x00,
    };

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  ret = max86178_configspi(priv);
  if (ret < 0)
    {
      snerr("SPI configuration failed: %d\n", ret);
      SPI_LOCK(priv->config->spi, false);
      return ret;
    }

  /* Selects the device. Set CS (as a GPIO) low. */

  SPI_SELECT(priv->config->spi, priv->config->cs, true);
  IOEXP_WRITEPIN(priv->config->ioedev, priv->config->gpiocs, 0);

  /* First send start address and 0x00(means reading). Then write a block. */

  SPI_EXCHANGE(priv->config->spi, txbuf, txbuf, 2);
  SPI_EXCHANGE(priv->config->spi, sendbuf, sendbuf, nwords);

  /* Deselect the device, set CS high and release the SPI bus. */

  IOEXP_WRITEPIN(priv->config->ioedev, priv->config->gpiocs, 1);
  SPI_SELECT(priv->config->spi, priv->config->cs, false);
  SPI_LOCK(priv->config->spi, false);

  return OK;
#endif
}

/****************************************************************************
 * Name: max86178_readsingle
 *
 * Description:
 *   Read the value of a single 8-bit register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Address of the register to be read from.
 *   regval  - A pointer to the variable to stores the readout.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int max86178_readsingle(FAR struct max86178_dev_s *priv,
                               uint8_t regaddr, uint8_t *regval)
{
  return max86178_readregs(priv, regaddr, regval, 1);
}

/****************************************************************************
 * Name: max86178_writesingle
 *
 * Description:
 *   Write the value to a single 8-bit register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Address of the destination register.
 *   regval  - The value to be written.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int max86178_writesingle(FAR struct max86178_dev_s *priv,
                                uint8_t regaddr, uint8_t regval)
{
  return max86178_writeregs(priv, regaddr, &regval, 1);
}

/****************************************************************************
 * Name: max86178_batch_calcu
 *
 * Description:
 *   Calculate reasonable batch latency and fifo watermark according to the
 *   sample interval and maximum batch number.
 *
 * Input Parameters:
 *   priv         - Device struct
 *   latency_us   - The time between batch data, in us.
 *   batch_number - Maximum number or batch data one time.
 *   interval     - Sample interval in us.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_batch_calcu(FAR struct max86178_dev_s *priv,
                                 FAR unsigned long *latency_us,
                                 uint32_t batch_number, unsigned long interval)
{
  unsigned long max_latency;
  uint32_t fifowtm;

  if (*latency_us > 0)
    {
      max_latency = batch_number * interval;
      if (*latency_us > max_latency)
        {
          *latency_us = max_latency;
        }
      else if (*latency_us < interval)
        {
          *latency_us = interval;
        }

      fifowtm = MAX86178_CEILING(*latency_us, interval);
      *latency_us = fifowtm * interval;
    }
}

/****************************************************************************
 * Name: max86178_checkid
 *
 * Description:
 *   Read and verify the MAX86178 chip ID
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int max86178_checkid(FAR struct max86178_dev_s *priv)
{
  uint8_t devid;
  int ret;

  ret = max86178_readsingle(priv, MAX86178_REG_PARTID, &devid);
  if (ret < 0)
    {
      return ret;
    }

  if (devid != MAX86178_PARTID)
    {
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_enable
 *
 * Description:
 *   MAX86178 enter normal mode or shutdown mode.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - true(enter normal mode) and false(enter shutdown mode).
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_enable(FAR struct max86178_dev_s *priv, bool enable)
{
  uint8_t regval;

  if (enable)
    {
      /* If one wants to enabled the device when the device is not enabled,
       * enable it. If the device has been enabled, do nothing.
       */

      if (priv->activated == false)
        {
          /* Read and write MAX86178_SYSCFG1_NORMAL to exit shutdown mode.
           * PLL will not be enable here, it will be when ECG is enabled.
           */

          max86178_readsingle(priv, MAX86178_REG_SYSCFG1, &regval);
          regval = regval & (~MAX86178_SYSCFG1_SHDN_MASK);
          regval = regval | MAX86178_SYSCFG1_NORMAL;
          max86178_writesingle(priv, MAX86178_REG_SYSCFG1, regval);

          /* Initialize the device when it's enabled, since the device may be
           * powered down when inactivated.
           */

          max86178_init(priv);
          priv->activated = true;
        }
    }
  else
    {
      /* If one wants to disabled the device when the device is enabled,
       * disable it. Otherwise if the device has been disabled, do nothing.
       */

      if (priv->activated == true)
        {
          /* Enter shutdown mode. */

          max86178_shutdown(priv);
          priv->activated = false;
        }
    }
}

/****************************************************************************
 * Name: max86178_fifo_flush
 *
 * Description:
 *   Flush FIFO. The contents will be discarded.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_fifo_flush(FAR struct max86178_dev_s *priv)
{
  uint8_t regval;

  max86178_readsingle(priv, MAX86178_REG_FIFOCFG2, &regval);
  regval |= MAX86178_FIFOCFG2_FLUSH;
  max86178_writesingle(priv, MAX86178_REG_FIFOCFG2, regval);
}

/****************************************************************************
 * Name: max86178_fifo_read
 *
 * Description:
 *   Read all available data in FIFO.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   1. ECG and PPG are synchronous sampled. Asynchronous sampling need
 *      timing data to determine the timestamp, and will be implented later.
 *   2. One of two PPG channels works at the same time. Perhaps we will add
 *      procession for 2 channels working the same time. Currently, only 1
 *      channel working at the same time is enough.
 *
 ****************************************************************************/

static int max86178_fifo_read(FAR struct max86178_dev_s *priv)
{
  uint32_t fifobytes;
  uint32_t temp_sample;
  uint32_t data_per_sample;
  uint32_t counter_ecg = 0;
  uint32_t counter_ppg[MAX86178_PPG_SENSOR_NUM] =
    {
      0, 0, 0, 0
    };

  uint16_t num = 0;
  uint16_t i;
  uint16_t j;
  uint8_t temp_num;
  uint8_t chidx = MAX86178_PPG_SENSOR_NUM - 1;
  uint8_t toggle_ppg[MAX86178_PPG_SENSOR_NUM] =
    {
      1, 1, 1, 1
    };

  /* Get number of samples in FIFO. */

  max86178_readsingle(priv, MAX86178_REG_FIFOCNT1, &temp_num);
  if (temp_num & MAX86178_FIFOCNT1_CNT_MSB_MASK)
    {
      num = 256;
    }

  max86178_readsingle(priv, MAX86178_REG_FIFOCNTLSB, &temp_num);
  data_per_sample = priv->ecg_sensor.activated ? 1 : 0;
  data_per_sample = data_per_sample + priv->ppg_activated * 2;
  num = (num + temp_num);
  if (data_per_sample > 1)
    {
      num = num / data_per_sample * data_per_sample;
    }

  /* If there's no data to read, return. */

  if (num == 0)
    {
      return -ENODATA;
    }

  /* Calculate how many bytes are in FIFO. */

  fifobytes = num * MAX86178_FIFO_BYTES_PER_SAMPLE;

  /* Read the FIFO in number of FIFO watermark */

  max86178_readregs(priv, MAX86178_REG_FIFODATA, priv->fifobuf, fifobytes);

  /* Deal each sample in FIFO. The last sample is the newest sample. */

  for (i = 0 ; i < num; i++)
    {
      temp_sample =
            (priv->fifobuf[i * MAX86178_FIFO_BYTES_PER_SAMPLE] << 16) |
            (priv->fifobuf[i * MAX86178_FIFO_BYTES_PER_SAMPLE + 1] << 8) |
            (priv->fifobuf[i * MAX86178_FIFO_BYTES_PER_SAMPLE + 2]);

      switch (temp_sample & MAX86178_FIFOTAG_MASK_PRE)
        {
          case MAX86178_FIFOTAG_PRE_MEAS1:       /* PPG MEAS1 data tag. */
            {
              chidx = MAX86178_PPG0_SENSOR_IDX;
              max86178_ppg_dealsample(priv, chidx, temp_sample,
                                      &counter_ppg[chidx],
                                      &toggle_ppg[chidx]);
            }
            break;

          case MAX86178_FIFOTAG_PRE_MEAS2:       /* PPG MEAS2 data tag. */
            {
              chidx = MAX86178_PPG1_SENSOR_IDX;
              max86178_ppg_dealsample(priv, chidx, temp_sample,
                                      &counter_ppg[chidx],
                                      &toggle_ppg[chidx]);
            }
            break;

          case MAX86178_FIFOTAG_PRE_MEAS3:       /* PPG MEAS3 data tag. */
            {
              chidx = MAX86178_PPG2_SENSOR_IDX;
              max86178_ppg_dealsample(priv, chidx, temp_sample,
                                      &counter_ppg[chidx],
                                      &toggle_ppg[chidx]);
            }
            break;

          case MAX86178_FIFOTAG_PRE_MEAS4:       /* PPG MEAS4 data tag. */
            {
              chidx = MAX86178_PPG3_SENSOR_IDX;
              max86178_ppg_dealsample(priv, chidx, temp_sample,
                                      &counter_ppg[chidx],
                                      &toggle_ppg[chidx]);
            }
            break;

          case MAX86178_FIFOTAG_PRE_ALC_OVF:     /* PPG ambient overflow. */

          /* ALC_OVF causes a max output, as the same as EXP_OVF case. */

          case MAX86178_FIFOTAG_PRE_EXP_OVF:     /* PPG exposure overflow. */
            {
              /* If the overflow data appears after a PPG channel's data end
               * when next channel's data comes, search for the next enabled
               * PPG channel. If it's the first data during this FIFO
               * reading, search from PPG ch0(chidx's initial value if the
               * last PPG channel index).
               * Otherwise the overflow data appears when a PPG channel's
               * data was dealing, the overflow data belongs to it.
               */

              if (toggle_ppg[chidx] == 1)
                {
                  do
                    {
                      chidx++;
                      if (chidx == MAX86178_PPG_SENSOR_NUM)
                        {
                          chidx = 0;
                        }
                    }
                  while (priv->ppg_sensor[chidx].activated == false);
                }

              max86178_ppg_dealsample(priv, chidx, MAX86178_ABS_PPG_MAX,
                                      &counter_ppg[chidx],
                                      &toggle_ppg[chidx]);
            }
            break;

          case MAX86178_FIFOTAG_PRE_ECG:         /* ECG data tag. */
            {
              priv->ecgdata[counter_ecg].ecg =
                max86178_ecg_calcudata(&priv->ecg_sensor, temp_sample);
              counter_ecg++;
            }
            break;

          default:                               /* Other data tags. */
            {
              /* Corresponding procession for the other tags will be added in
               * future.
               */
            }
            break;
        }
    }

  if (counter_ecg > 0)
    {
      /* Calculate each timestamp from the last one. */

      for (i = 0; i < counter_ecg; i++)
        {
          priv->ecgdata[i].timestamp = priv->timestamp -
            priv->interval * (counter_ecg - i + 1);
        }

      priv->ecg_sensor.lower.push_event(priv->ecg_sensor.lower.priv,
                                        priv->ecgdata,
                                        sizeof(FAR struct sensor_ecg) *
                                        counter_ecg);
    }

  for (i = 0; i < MAX86178_PPG_SENSOR_NUM; i++)
  {
    /* Calculate each timestamp from the last one. */

    if (counter_ppg[i] > 0 && priv->ppg_sensor[i].inactivating == false)
      {
        for (j = 0; j < counter_ppg[i]; j++)
          {
            priv->ppgdata[i][j].timestamp = priv->timestamp -
              priv->interval * (counter_ppg[i] - j - 1);
          }

        priv->ppg_sensor[i].lower.push_event(priv->ppg_sensor[i].lower.priv,
          priv->ppgdata[i],
          sizeof(FAR struct sensor_ppgd) * counter_ppg[i]);
      }

    /* Optimize PPG0~2 (dark channel need no optimization) if needed. */

    if (priv->ppg_sensor[i].auto_optimize == true &&
        i != MAX86178_PPG3_SENSOR_IDX)
      {
        max86178_ppg_optimize(&priv->ppg_sensor[i]);
      }
  }

  return OK;
}

/****************************************************************************
 * Name: max86178_fifo_set
 *
 * Description:
 *   Set MAX86178's FIFO watermark and its interrupt, or disable them.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_fifo_set(FAR struct max86178_dev_s *priv)
{
  uint8_t regval;

  /* Set water mark. STAT1_FIFOAFULL is set when FIFO reach 256 - FIFOAFULL,
   * where 256 is the FIFO size (uint in samples, 3Bytes per sample).
   */

  if (priv->fifowtm > 0)
    {
      regval = MAX86178_FIFO_SIZE_SAMPLES - priv->fifowtm;
      max86178_writesingle(priv, MAX86178_REG_FIFOAFULL, regval);

      /* Set interrupt pin of MAX86178. */

      max86178_readsingle(priv, MAX86178_REG_PINFUNC, &regval);
      regval = regval & (~MAX86178_PINFUNC_INT1_FCFG_MASK);
      regval = regval | MAX86178_PINFUNC_INT1_RDCLR;
      max86178_writesingle(priv, MAX86178_REG_PINFUNC, regval);

      max86178_readsingle(priv, MAX86178_REG_PINCFG, &regval);
      regval = regval & (~MAX86178_PINCFG_INT1_OCFG_MASK);
      regval = regval | MAX86178_PINCFG_INT1_PP_HIGH;
      max86178_writesingle(priv, MAX86178_REG_PINCFG, regval);

      /* Enable FIFO_A_FULL interrupt. */

      max86178_readsingle(priv, MAX86178_REG_INT1EN1, &regval);
      regval = regval | MAX86178_INTXEN1_AFULL_EN;
      max86178_writesingle(priv, MAX86178_REG_INT1EN1, regval);
    }
  else
    {
      /* Disable interrupt pin of MAX86178. */

      max86178_readsingle(priv, MAX86178_REG_PINFUNC, &regval);
      regval = regval & (~MAX86178_PINFUNC_INT1_FCFG_MASK);
      regval = regval | MAX86178_PINFUNC_INT1_DISABLE;
      max86178_writesingle(priv, MAX86178_REG_PINFUNC, regval);

      /* Disable FIFO_A_FULL interrupt. */

      max86178_readsingle(priv, MAX86178_REG_INT1EN1, &regval);
      regval = regval & (~MAX86178_INTXEN1_AFULL_EN);
      max86178_writesingle(priv, MAX86178_REG_INT1EN1, regval);
    }
}

/****************************************************************************
 * Name: max86178_init
 *
 * Description:
 *   Initialize the MAX86178 into default status.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_init(FAR struct max86178_dev_s *priv)
{
  int i;

  for (i = 0; i < MAX86178_INIT_TABLE_SIZE; i++)
    {
      max86178_writesingle(priv, max86178_init_table[i].regaddr,
                           max86178_init_table[i].regval);
    }
}

/****************************************************************************
 * Name: max86178_selftest
 *
 * Description:
 *   MAX86178 has no specailized self-testing function. Currently only
 *   checking device ID is supported.
 *
 * Input Parameters:
 *   priv - The device struct.
 *   arg  - The parameters associated with cmd.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *   -ENOTTY - The cmd don't support.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int max86178_selftest(FAR struct max86178_dev_s *priv,
                             unsigned long arg)
{
  switch (arg)
    {
      case MAX86178_CTRL_CHECKID:           /* Check ID command. */
        {
          return max86178_checkid(priv);
        }

      /* In the case above, function has returned thus no break is need. */

      default:                              /* Other cmd tag */
        {
          snerr("The cmd was not supported: %d\n", -ENOTTY);
          return -ENOTTY;
        }
    }
}

/****************************************************************************
 * Name: max86178_shutdown
 *
 * Description:
 *   MAX86178 enter shutdown mode. SPI is available during shutdown mode.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - true(enter normal mode) and false(enter shutdown mode).
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_shutdown(FAR struct max86178_dev_s *priv)
{
  uint8_t regval;

  /* Read and write PLLCFG_PLL_EN to disable PLL if it's on. */

  max86178_readsingle(priv, MAX86178_REG_PLLCFG1, &regval);
  if (regval & MAX86178_PLLCFG1_PLL_EN_MASK)
    {
      regval = regval & (~MAX86178_PLLCFG1_PLL_EN_MASK);
      regval = regval | MAX86178_PLLCFG1_PLL_DIS;
      max86178_writesingle(priv, MAX86178_REG_PLLCFG1, regval);
    }

  /* Put interrupt pin of MAX86178 in open-drain to avoid current leakage. */

  max86178_readsingle(priv, MAX86178_REG_PINCFG, &regval);
  regval = regval & (~MAX86178_PINCFG_INT1_OCFG_MASK);
  regval = regval | MAX86178_PINCFG_INT1_OD;
  max86178_writesingle(priv, MAX86178_REG_PINCFG, regval);

  /* Read and write MAX86178_SYSCFG1_NORMAL to enter shutdown mode. */

  max86178_readsingle(priv, MAX86178_REG_SYSCFG1, &regval);
  regval = regval & (~MAX86178_SYSCFG1_SHDN_MASK);
  regval = regval | MAX86178_SYSCFG1_SHDN;
  max86178_writesingle(priv, MAX86178_REG_SYSCFG1, regval);
}

/****************************************************************************
 * Name: max86178_softreset
 *
 * Description:
 *   All registers valuse will be set to power-on-reset state.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_softreset(FAR struct max86178_dev_s *priv)
{
  uint8_t regval;

  /* Disable PLL */

  max86178_readsingle(priv, MAX86178_REG_PLLCFG1, &regval);
  regval = regval & (~MAX86178_PLLCFG1_PLL_EN);
  max86178_writesingle(priv, MAX86178_REG_PLLCFG1, regval);

  /* Whatever the other bits in MAX86178_REG_SYSCFG1 are, they will be 0
   * (default) after softreset. Thus there's no need to read them out.
   */

  regval = MAX86178_SYSCFG1_RESET;
  max86178_writesingle(priv, MAX86178_REG_SYSCFG1, regval);
}

/****************************************************************************
 * Name: max86178_update_sensors
 *
 * Description:
 *   Update current activated sensors' interval and batch, or activate new
 *   sensors after FIFO reading.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   If activated, ECG's sample interval must be equal to PPG's. If ECG is
 *   not actiaved, PPG's sample interval is generally greater than ECG's.
 *
 ****************************************************************************/

static void max86178_update_sensors(FAR struct max86178_dev_s *priv)
{
  unsigned long intrvl_min;
  unsigned long batch_min;
  uint32_t fifowtm = 0;
  uint16_t ppg_frdiv;
  uint8_t ppg_activating;
  uint8_t ecg_activating = 0;
  uint8_t ppg_chidx;

  /* Step 1. Modify device's interval if needed. */

  if (priv->update_interval || priv->update_activate)
    {
      /* Find the activating(will be activated soon below) or activated with
       * no inactivating(will not be inactivated below) PPG sensor with the
       * minimum interval.
       */

      intrvl_min = 0xffffffff;
      for (ppg_chidx = 0; ppg_chidx < MAX86178_PPG_SENSOR_NUM; ppg_chidx++)
        {
          if (((priv->ppg_sensor[ppg_chidx].activated == true &&
              priv->ppg_sensor[ppg_chidx].inactivating == false) ||
              priv->ppg_sensor[ppg_chidx].activating) &&
              priv->ppg_sensor[ppg_chidx].interval_desired < intrvl_min)
            {
              intrvl_min = priv->ppg_sensor[ppg_chidx].interval_desired;
            }
        }

      /* ECG and PPG shall sample synchronously. If ECG has had or will have
       * a less interval, device interval should use it, or it will use the
       * minium of PPGs'.
       */

      if ((priv->ecg_sensor.activated == true &&
          priv->ecg_sensor.inactivating == false) ||
          priv->ecg_sensor.activating == true)
        {
          if (priv->ecg_sensor.interval_desired < intrvl_min)
            {
              intrvl_min = priv->ecg_sensor.interval_desired;
            }
        }

      /* If minimum interval has changed, some sensors will be activated. Set
       * new PPG FPS and ECG SR then. Otherwise do nothing since no sensor
       * will be activated or the interval will not change.
       */

      if (intrvl_min != 0xffffffff && intrvl_min != priv->interval)
        {
          ppg_frdiv = max86178_ppg_calcu_frdiv(intrvl_min);
          max86178_ppg_set_fps(priv, ppg_frdiv);
          max86178_ecg_calcu_clk(&priv->ecg_sensor, &intrvl_min);
          max86178_ecg_set_sr(priv, priv->ecg_sensor.decrate,
                              priv->ecg_sensor.ndiv);
          priv->interval = intrvl_min;

          /* FIFO watermark shall be modified for new sample interval if
           * interrupt reading is used. There's nothing to do for polling
           * reading, the new interval will take effect when queuing next
           * polling worker.
           */

          if (priv->fifowtm > 0)
            {
              priv->update_batch = true;
            }
        }

      /* Flag is cleared after updating PPG interval is completed. */

      priv->update_interval = false;
    }

  /* Step 2. Modify the FIFO watermark if any sensor's batch has changed or
   * any sensor is to be activate or inactivate.
   */

  if (priv->update_batch || priv->update_activate)
    {
      /* Find the minimum batch latency among all activating(will be
       * activated soon below) and activated with no inactivating(will not be
       * inactivated below) PPG sensors. And count the number of PPG sensors
       * that will be or remain activated.
       */

      batch_min = 0xffffffff;
      ppg_activating = 0;
      for (ppg_chidx = 0; ppg_chidx < MAX86178_PPG_SENSOR_NUM; ppg_chidx++)
        {
          if ((priv->ppg_sensor[ppg_chidx].activated == true &&
              priv->ppg_sensor[ppg_chidx].inactivating == false) ||
              priv->ppg_sensor[ppg_chidx].activating)
            {
              batch_min = priv->ppg_sensor[ppg_chidx].batch_desired;
              ppg_activating++;
            }
        }

      /* ECG and PPG shall use the same batch latency. */

      if ((priv->ecg_sensor.activated == true &&
          priv->ecg_sensor.inactivating == false) ||
          priv->ecg_sensor.activating == true)
        {
          ecg_activating = 1;
          if (priv->ecg_sensor.batch_desired < batch_min)
            {
              batch_min = priv->ecg_sensor.batch_desired;
            }
        }

      /* If some sensor will be or remain activated, calculate new FIFO
       * watermark. Otherwise do nothing since no sensor will be activated or
       * FIFO is not needed.
       */

      if (batch_min != 0xffffffff && batch_min != 0)
        {
          fifowtm = batch_min / priv->interval *
                    (2 * ppg_activating + ecg_activating);
        }

      /* Set new FIFO watermark if it's different from current one. Switch
       * between polling and interrupt if needed.
       */

      if (batch_min != 0xffffffff && fifowtm != priv->fifowtm)
        {
          if (priv->activated == true)
            {
              if (fifowtm > 0 && priv->fifowtm == 0)
                {
                  work_cancel(HPWORK, &priv->work_poll);
                  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                                  IOEXPANDER_OPTION_INTCFG,
                                  (FAR void *)IOEXPANDER_VAL_RISING);
                }
              else if (fifowtm == 0 && priv->fifowtm > 0)
                {
                  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                                  IOEXPANDER_OPTION_INTCFG,
                                  (FAR void *)IOEXPANDER_VAL_DISABLE);
                  work_queue(HPWORK, &priv->work_poll, max86178_worker_poll,
                             priv, priv->interval / USEC_PER_TICK);
                }
            }

          priv->fifowtm = fifowtm;
          max86178_fifo_flush(priv);
          max86178_fifo_set(priv);
        }

      /* Complete the batch modification. */

      priv->update_batch = false;
    }

  /* Step 3. If need to activate or inactivate some sensors. */

  if (priv->update_activate)
    {
      if (priv->ecg_sensor.activating == true)
        {
          max86178_ecg_enable(priv, true);
        }
      else if (priv->ecg_sensor.inactivating == true)
        {
          max86178_ecg_enable(priv, false);
        }

      for (ppg_chidx = 0; ppg_chidx < MAX86178_PPG_SENSOR_NUM; ppg_chidx++)
        {
          if (priv->ppg_sensor[ppg_chidx].activating)
            {
              max86178_ppg_enable(priv, ppg_chidx, true);
            }
          else if (priv->ppg_sensor[ppg_chidx].inactivating)
            {
              max86178_ppg_enable(priv, ppg_chidx, false);
            }
        }

      priv->update_activate = false;
    }

  /* Step 4. If no sensor is activated, shut down and disable reading. */

  if (priv->ecg_sensor.activated == false && priv->ppg_activated == 0)
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

      max86178_enable(priv, false);
    }
}

/****************************************************************************
 * Name: max86178_ecg_calcu_batch
 *
 * Description:
 *   Calculate ECG clock parameters according to the sample interval.
 *
 * Input Parameters:
 *   sensor   - ECG sensor struct.
 *   interval - Sample interval in us.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_ecg_calcu_clk(FAR struct max86178_ecg_sensor_s *sensor,
                                   FAR unsigned long *interval)
{
  float freq;
  uint16_t decrate;
  uint16_t ndiv;
  uint8_t decrate_options[MAX86178_ECG_DECRATE_NUM] = {
    MAX86178_ECGCFG1_DEC_RATE_16, MAX86178_ECGCFG1_DEC_RATE_32,
    MAX86178_ECGCFG1_DEC_RATE_64, MAX86178_ECGCFG1_DEC_RATE_128,
    MAX86178_ECGCFG1_DEC_RATE_256, MAX86178_ECGCFG1_DEC_RATE_512,
    };

  uint16_t decratelist[MAX86178_ECG_DECRATE_NUM] = {
    MAX86178_ECG_DEC_RATE_16, MAX86178_ECG_DEC_RATE_32,
    MAX86178_ECG_DEC_RATE_64, MAX86178_ECG_DEC_RATE_128,
    MAX86178_ECG_DEC_RATE_256, MAX86178_ECG_DEC_RATE_512,
    };

  uint8_t i;

  /* Find the period that matches best. */

  freq = MAX86178_ONE_SECOND / *interval;
  if (freq < MAX86178_ECG_SR_MIN)
    {
      freq = MAX86178_ECG_SR_MIN;
    }
  else if (freq > MAX86178_ECG_SR_MAX)
    {
      freq = MAX86178_ECG_SR_MAX;
    }

  /* From little one to great one, the first DEC_RATE which makes ECG_ADC_CLK
   * greater than MAX86178_ECG_ADC_CLK_MIN, is the only DEC_RATE which can
   * make ECG_ADC_CLK to be in range of [MAX86178_ECG_ADC_CLK_MIN,
   * MAX86178_ECG_ADC_CLK_MAX].
   */

  decrate = decratelist[0];
  for (i = 0; i < MAX86178_ECG_DECRATE_NUM; i++)
    {
      if (freq * decratelist[i] > MAX86178_ECG_ADC_CLK_MIN)
        {
          decrate = decratelist[i];
          break;
        }
    }

  ndiv = MAX86178_ECG_PLL_DFT / (freq * decrate);
  freq = MAX86178_ECG_PLL_DFT / (ndiv * decrate);
  *interval = MAX86178_ONE_SECOND / freq;
  sensor->ndiv = ndiv;
  sensor->decrate = decrate_options[i];
}

/****************************************************************************
 * Name: max86178_ecg_calcudata
 *
 * Description:
 *   Calculate ECG value from origin sample data.
 *
 * Input Parameters:
 *   sensor - ECG sensor struct.
 *   sample - Origin ECG sample data from FIFO
 *
 * Returned Value:
 *   Calculated ECG value.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static float max86178_ecg_calcudata(FAR struct max86178_ecg_sensor_s *sensor,
                                    uint32_t sample)
{
  float temp_float;
  int temp_int;

  /* ECG value is a 2's complement code 18bits number. The MSB 6bits are tag.
   * The 17th bit is the sign bit.
   */

  sample = sample & (~MAX86178_FIFOTAG_MASK_POST);
  if (sample & MAX86178_ECG_SIGN_MASK)
    {
      sample = sample | MAX86178_ECG_NEGATIVE;
    }

  temp_int = sample;
  temp_float = (float)temp_int;

  return (temp_float * sensor->factor);
}

/****************************************************************************
 * Name: max86178_ecg_enable
 *
 * Description:
 *   Enable or disable sensor device.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int max86178_ecg_enable(FAR struct max86178_dev_s *priv, bool enable)
{
  uint8_t regval;
  int i;

  if (enable)
    {
      /* To enable ECG, ensure the MAX86178 is enabled. */

      max86178_enable(priv, enable);

      /* Read and write PLL settings (PLL CLK = 4 MHz) */

      max86178_readsingle(priv, MAX86178_REG_PLLCFG6, &regval);
      regval = regval & (~MAX86178_PLLCFG6_CLKFRQSEL_MASK);
      regval = regval | MAX86178_PLLCFG6_REFCLK_32K;
      max86178_writesingle(priv, MAX86178_REG_PLLCFG6, regval);
      regval = MAX86178_MDIV_DFT;
      max86178_writesingle(priv, MAX86178_REG_MDIVLSB, regval);

      /* Read and write MAX86178_PLLCFG_PLL_EN to enable PLL */

      max86178_readsingle(priv, MAX86178_REG_PLLCFG1, &regval);
      regval = regval & (~MAX86178_PLLCFG1_MDIV_MSB_MASK);
      regval = regval & (~MAX86178_PLLCFG1_PLL_EN_MASK);
      regval = regval | MAX86178_PLLCFG1_PLL_EN;
      max86178_writesingle(priv, MAX86178_REG_PLLCFG1, regval);

      /* Wait until the PLL is locked or time is out. */

      for (i = 0; i < MAX86178_PLL_PHASE_LOCK_TIME; i++)
        {
          up_mdelay(1);

          max86178_readsingle(priv, MAX86178_REG_STAT3, &regval);
          if (regval & MAX86178_STAT3_PHASE_LOCK)
            {
              break;
            }
        }

      if (i == MAX86178_PLL_PHASE_LOCK_TIME)
        {
          return -ETIME;
        }

      /* Set ECG gain to default */

      max86178_readsingle(priv, MAX86178_REG_ECGCFG2, &regval);
      regval = regval & MAX86178_ECGCFG2_IPOL_MASK;
      regval = regval | MAX86178_ECG_PGAGAIN_DFT | MAX86178_ECG_INARGE_DFT |
                        MAX86178_ECG_INAGAIN_DFT;
      max86178_writesingle(priv, MAX86178_REG_ECGCFG2, regval);

      /* Connect ECG block inputs to MAX86178 pins */

      max86178_readsingle(priv, MAX86178_REG_ECGCFG3, &regval);
      regval = regval & (~MAX86178_ECGCFG3_MUXSEL_MASK);
      regval = regval | MAX86178_ECGCFG3_MUXSEL_EL123;
      max86178_writesingle(priv, MAX86178_REG_ECGCFG3, regval);
      max86178_writesingle(priv, MAX86178_REG_ECGCALCFG3, 0);

      /* Enable ECG */

      max86178_readsingle(priv, MAX86178_REG_ECGCFG1, &regval);
      regval = regval & (~MAX86178_ECGCFG1_ECG_EN_MASK);
      regval = regval | MAX86178_ECGCFG1_ECG_EN;
      max86178_writesingle(priv, MAX86178_REG_ECGCFG1, regval);
    }
  else
    {
      /* Disconnect ECG block inputs from MAX86178 pins */

      max86178_readsingle(priv, MAX86178_REG_ECGCFG3, &regval);
      regval = regval & (~MAX86178_ECGCFG3_MUXSEL_MASK);
      regval = regval | MAX86178_ECGCFG3_MUXSEL_NC;
      max86178_writesingle(priv, MAX86178_REG_ECGCFG3, regval);

      /* Disable ECG. */

      max86178_readsingle(priv, MAX86178_REG_ECGCFG1, &regval);
      regval = regval & (~MAX86178_ECGCFG1_ECG_EN_MASK);
      regval = regval | MAX86178_ECGCFG1_ECG_DIS;
      max86178_writesingle(priv, MAX86178_REG_ECGCFG1, regval);
    }

  priv->ecg_sensor.activated = enable;
  priv->ecg_sensor.activating = false;
  priv->ecg_sensor.inactivating = false;

  return OK;
}

/****************************************************************************
 * Name: max86178_ecg_set_sr
 *
 * Description:
 *   Set the ECG SR(sample rate).
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   decrate - Decimal rate for ECG ADC.
 *   ndiv    - NDIV divide ratio for ECG clock
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_ecg_set_sr(FAR struct max86178_dev_s *priv,
                                uint8_t decrate, uint16_t ndiv)
{
  uint8_t regval;

  max86178_readsingle(priv, MAX86178_REG_ECGCFG1, &regval);
  regval = regval & (~MAX86178_ECGCFG1_DEC_RATE_MASK);
  regval = regval | decrate;
  max86178_writesingle(priv, MAX86178_REG_ECGCFG1, regval);

  max86178_readsingle(priv, MAX86178_REG_PLLCFG4, &regval);
  regval = regval & (~MAX86178_PLLCFG4_ECGNDIVH_MASK);
  regval = regval | (uint8_t)((ndiv >> 8) << MAX86178_PLLCFG4_ECGNDIVH_OFST);
  max86178_writesingle(priv, MAX86178_REG_PLLCFG4, regval);
  regval = (uint8_t)ndiv;
  max86178_writesingle(priv, MAX86178_REG_ECGNDIVLSB, regval);
}

/****************************************************************************
 * Name: max86178_ppg_caludata
 *
 * Description:
 *   Calculate PPG value from origin sample data.
 *
 * Input Parameters:
 *   sample - Origin PPG sample data from FIFO.
 *
 * Returned Value:
 *   Calculated PPG value.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static uint32_t max86178_ppg_calcudata(uint32_t sample)
{
  return sample & (~MAX86178_FIFOTAG_MASK_PRE);
}

/****************************************************************************
 * Name: max86178_ppg_calu_frdiv
 *
 * Description:
 *   Calculate PPG frame clock FRDIV.
 *
 * Input Parameters:
 *   interval - PPG sample interval(us).
 *
 * Returned Value:
 *   Calculated PPG frame clock FRDIV.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static uint16_t max86178_ppg_calcu_frdiv(unsigned long interval)
{
  float freq;
  uint16_t frdiv;

  freq = MAX86178_ONE_SECOND / interval;
  frdiv = (uint16_t)(MAX86178_REF_CLK_DFT / freq);
  if (frdiv < MAX86178_FRCLKDIV_MIN)
    {
      frdiv = MAX86178_FRCLKDIV_MIN;
    }
  else if (frdiv > MAX86178_FRCLKDIV_MAX)
    {
      frdiv = MAX86178_FRCLKDIV_MAX;
    }

  return frdiv;
}

/****************************************************************************
 * Name: max86178_ppg_dealsample
 *
 * Description:
 *   Deal the sample data from FIFO - store it in the buffer for push_event,
 *   modify the count and toggle flag.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   chidx  - The channel index of the PPG sensor to be operated.
 *   sample - The sample data from FIFO.
 *   count  - Count of the samples in result buffer.
 *   toggle - A flag indicate which PPG ADC is operating.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_ppg_dealsample(FAR struct max86178_dev_s *priv,
                                    uint8_t chidx, uint32_t sample,
                                    FAR uint32_t *count, FAR uint8_t *toggle)
{
  uint32_t ppg;

  ppg = max86178_ppg_calcudata(sample);
  priv->ppgdata[chidx][*count].current = priv->ppg_sensor[chidx].current;
  if (*toggle == 1)
    {
      priv->ppgdata[chidx][*count].ppg[0] = ppg;
      priv->ppgdata[chidx][*count].gain[0] = priv->ppg_sensor[chidx].gain1;
      *toggle = 2;

      /* Record for optimization */

      max86178_ppg_optim_record(&priv->ppg_sensor[chidx].optm1, ppg);
    }
  else
    {
      priv->ppgdata[chidx][*count].ppg[1] = ppg;
      priv->ppgdata[chidx][*count].gain[1] = priv->ppg_sensor[chidx].gain2;
      *toggle = 1;
      *count = *count + 1;

      /* Record for optimization */

      max86178_ppg_optim_record(&priv->ppg_sensor[chidx].optm2, ppg);
    }
}

/****************************************************************************
 * Name: max86178_ppg_optimze
 *
 * Description:
 *   Modify PPG's parameters to make PPG's ouput in optimum range. The LED
 *   current and ADC input range (which will affect ADC gain) will be
 *   modified.
 *
 * Input Parameters:
 *   sensor - PPG sensor struct.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_ppg_optimize(FAR struct max86178_ppg_sensor_s *sensor)
{
  uint32_t avg = 0;

  /* If PPG1, which should have higher SNR than PPG2, shall be optimized. */

  if (sensor->optm1.trigger)
    {
      if (sensor->optm1.up_trigger >= MAX86178_PPG_OP_TRIG)
        {
          if (sensor->current > MAX86178_PPG_LEDLSBSTEP)
            {
              avg = sensor->optm1.total / sensor->optm1.up_trigger;
            }

          sensor->optm1.up_trigger = 0;
        }
      else
        {
          if (sensor->current < MAX86178_PPG_LEDPAMAX)
            {
              avg = sensor->optm1.total / sensor->optm1.low_trigger;
            }

          sensor->optm1.low_trigger = 0;
        }

      if (avg != 0)
        {
          sensor->current = (uint32_t)((uint64_t)sensor->current * (uint64_t)
            max86178_ppg_optim_param_table[sensor->optm1.adc_range].middle /
            (uint64_t)avg);
          max86178_ppg_set_current(sensor, &sensor->current);
          sensor->optm1.total = 0;
          sensor->optm1.cnt = 0;
          sensor->optm1.times = 1;
        }

      sensor->optm1.trigger = false;
    }

  /* If driver is optimizing PPG1 */

  if (sensor->optm1.times > 0 && sensor->optm1.cnt > MAX86178_PPG_OP_WINDOW)
    {
      avg = sensor->optm1.total / sensor->optm1.cnt;
      if (avg <=
          max86178_ppg_optim_param_table[sensor->optm1.adc_range].middle +
          max86178_ppg_optim_param_table[sensor->optm1.adc_range].span &&
          avg >=
          max86178_ppg_optim_param_table[sensor->optm1.adc_range].middle -
          max86178_ppg_optim_param_table[sensor->optm1.adc_range].span)
        {
          sensor->optm1.times = 0;
        }
      else
        {
          sensor->current = (uint32_t)((uint64_t)sensor->current * (uint64_t)
            max86178_ppg_optim_param_table[sensor->optm1.adc_range].middle /
            (uint64_t)avg);
          max86178_ppg_set_current(sensor, &sensor->current);
          sensor->optm1.times++;
          if (sensor->optm1.times > MAX86178_PPG_OP_TIMES)
            {
              sensor->optm1.times = 0;
            }
        }

      if (sensor->optm1.times == 0)
        {
          sensor->optm2.low_trigger = 0;
          sensor->optm2.up_trigger = 0;
          sensor->optm2.total = 0;
          sensor->optm2.cnt = 0;
          sensor->optm2.trigger = false;
        }

      sensor->optm1.total = 0;
      sensor->optm1.cnt = 0;
    }
}

/****************************************************************************
 * Name: max86178_ppg_optim_record
 *
 * Description:
 *   Record data for PPG optimization.
 *
 * Input Parameters:
 *   optim - Pointer to PPG optimization struct.
 *   ppg   - PPG value to be recorded.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_ppg_optim_record(FAR struct max86178_ppg_optim_s *optim,
                                      uint32_t ppg)
{
  /* If optimization didn't start, record and check if PPG exceed the range.
   * If optimization has started, only record the PPG value.
   */

  if (optim->times == 0)
    {
      /* If PPG value exceeds the range, record it. Otherwise clear the
       * counters. Thus only either continuously too great or continuously
       * too little PPG values will trigger an optimization.
       */

      if (ppg > max86178_ppg_optim_param_table[optim->adc_range].upper)
        {
          /* If PPG was less than lower bound before but not reach the
           * threshold to trigger optimization, clear its counter and start
           * upper counter. Otherwise increase the upper counter and record
           * the PPG value.
           */

          if (optim->low_trigger > 0 && optim->trigger == false)
            {
              optim->low_trigger = 0;
              optim->up_trigger = 1;
              optim->total = ppg;
            }
          else if (optim->low_trigger == 0)
            {
              optim->up_trigger++;
              optim->total = optim->total + ppg;
            }
        }
      else if (ppg < max86178_ppg_optim_param_table[optim->adc_range].lower)
        {
          /* If PPG was greater than upper bound before but not reach the
           * threshold to trigger optimization, clear its counter and start
           * lower counter. Otherwise increase the lower counter and record
           * the PPG value.
           */

          if (optim->up_trigger > 0 && optim->trigger == false)
            {
              optim->up_trigger = 0;
              optim->low_trigger = 1;
              optim->total = ppg;
            }
          else if (optim->up_trigger == 0)
            {
              optim->low_trigger++;
              optim->total = optim->total + ppg;
            }
        }
      else
        {
          /* If optimization will not be triggered, normal PPG value will
           * clear the triggers.
           */

          if (optim->trigger == false)
            {
              optim->up_trigger = 0;
              optim->low_trigger = 0;
              optim->total = 0;
            }
        }

      /* If either counter reach the threshold, change the flag. */

      if (optim->up_trigger >= MAX86178_PPG_OP_TRIG ||
          optim->low_trigger >= MAX86178_PPG_OP_TRIG)
        {
          optim->trigger = true;
        }
    }
  else
    {
      optim->total = optim->total + ppg;
      optim->cnt++;
    }
}

/****************************************************************************
 * Name: max86178_ppg_enable
 *
 * Description:
 *   Enable or disable the PPG measurement according to the channel index.
 *
 * Input Parameters:
 *   priv     - Device struct.
 *   chidx - The channel index of the PPG sensor to be operated.
 *   enable - True: enable the PPG measurement.
 *            False: disable the PPG measurement.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_ppg_enable(FAR struct max86178_dev_s *priv,
                                uint8_t chidx, bool enable)
{
  uint8_t regval;

  /* To enable ECG, ensure the MAX86178 is enabled. */

  max86178_enable(priv, enable);
  max86178_readsingle(priv, MAX86178_REG_PPGCFG1, &regval);
  if (enable)
    {
      /* If activating, set LED driver current */

      max86178_ppg_set_current(&priv->ppg_sensor[chidx],
                               &priv->ppg_sensor[chidx].current);
      regval = regval | (1 << chidx);
      priv->ppg_activated++;
    }
  else
    {
      regval = regval & (~(1 << chidx));
      priv->ppg_activated--;
    }

  max86178_writesingle(priv, MAX86178_REG_PPGCFG1, regval);
  priv->ppg_sensor[chidx].activated = enable;
  priv->ppg_sensor[chidx].activating = false;
  priv->ppg_sensor[chidx].inactivating = false;
}

/****************************************************************************
 * Name: max86178_ppg_set_current
 *
 * Description:
 *   Set the PPG LED driver current.
 *
 * Input Parameters:
 *   sensor  - PPG sensor struct.
 *   current - Pointer to the desired current value (uA).
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_ppg_set_current(FAR struct max86178_ppg_sensor_s
                                     *sensor, FAR uint32_t *current)
{
  FAR struct max86178_dev_s *priv;

  /* Address of register that contains the LED driver current range, for
   * PPG ch0~2. PPG ch3 is not configurable.
   */

  uint8_t ledrge_regaddr_table[MAX86178_PPG_SENSOR_NUM - 1] =
    {
      MAX86178_REG_MEAS1CFG4, MAX86178_REG_MEAS2CFG4, MAX86178_REG_MEAS3CFG4
    };

  /* Address of register that contains the LED driver current counts, for
   * PPG ch0~2. PPG ch3 is not configurable.
   */

  uint8_t ledpa_regaddr_table[MAX86178_PPG_SENSOR_NUM - 1] =
    {
      MAX86178_REG_MEAS1LEDA, MAX86178_REG_MEAS2LEDA, MAX86178_REG_MEAS3LEDB
    };

  uint32_t range;
  uint32_t cnt;
  uint32_t lsb;
  uint8_t regval;

  DEBUGASSERT(sensor->dev != NULL);

  priv = sensor->dev;

  /* If current exceeds 128mA (RGE3), it's limited to 128mA. */

  if (*current > MAX86178_PPG_LEDPAMAX)
    {
      *current = MAX86178_PPG_LEDPAMAX;
    }

  /*   current(uA)     | result |               range                |  LSB
   * 0 <= i < 32000    |   0    | MAX86178_MEASXCFG4_LEDRGE_32MA(0)  |  125
   * 32000<= i <64000  |   1    | MAX86178_MEASXCFG4_LEDRGE_64MA(1)  | 125*2
   * 64000<= i <96000  |   2    | MAX86178_MEASXCFG4_LEDRGE_96MA(2)  | 125*3
   * 96000<= i <128000 |   3    | MAX86178_MEASXCFG4_LEDRGE_128MA(3) | 125*4
   */

  range = *current / MAX86178_PPG_LEDPASTEP;
  lsb = MAX86178_PPG_LEDLSBSTEP * (range + 1);

  /* Calculate how many LSB current has (255 max.). Select the closest one. */

  cnt = (*current + (lsb >> 1)) / lsb;
  if (cnt > 255)
    {
      cnt = 255;
    }

  *current = lsb * cnt;
  sensor->current = *current;

  /* Write configuration into registers. */

  max86178_readsingle(priv, ledrge_regaddr_table[sensor->chidx], &regval);
  if ((regval & MAX86178_MEASXCFG4_LEDRGE_MASK) != range)
    {
      regval = regval & (~MAX86178_MEASXCFG4_LEDRGE_MASK);
      regval = regval | (uint8_t)(range & MAX86178_MEASXCFG4_LEDRGE_MASK);
      max86178_writesingle(priv, ledrge_regaddr_table[sensor->chidx],
                           regval);
    }

  max86178_readsingle(priv, ledpa_regaddr_table[sensor->chidx], &regval);
  if (regval != (uint8_t)cnt)
    {
      regval = (uint8_t)cnt;
      max86178_writesingle(priv, ledpa_regaddr_table[sensor->chidx], regval);
    }
}

/****************************************************************************
 * Name: max86178_ppg_set_fps
 *
 * Description:
 *   Set the PPG FPS(frame per second).
 *
 * Input Parameters:
 *   priv     - Device struct.
 *   interval - Frame interval(us). FPS = 1000000(us)/interval
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void max86178_ppg_set_fps(FAR struct max86178_dev_s *priv,
                                 uint16_t frdiv)
{
  uint8_t frdivregs[2];

  frdivregs[0] = (uint8_t)(frdiv >> 8);
  frdivregs[1] = (uint8_t)frdiv;
  max86178_writeregs(priv, MAX86178_REG_FRCLKDIVH, frdivregs, 2);
}

/****************************************************************************
 * Name: max86178_ecg_activate
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

static int max86178_ecg_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable)
{
  FAR struct max86178_ecg_sensor_s *sensor =
    (FAR struct max86178_ecg_sensor_s *)lower;
  FAR struct max86178_dev_s *priv;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL);

  /* Get the pointer of the device from sensor. */

  priv = sensor->dev;

  /* Operate only if the activated status will change. */

  if (sensor->activated != enable)
    {
      if (enable)
        {
          /* If there's no activated sensor thus the device is shut-down, set
           * the ECG clock and FIFO settings, then activate ECG sensor. If
           * there has been any activated sensor, record the new activating,
           * and it will be activated with its settings when the next FIFO
           * reading comes.
           */

          if (priv->ppg_activated == 0)
            {
              /* Enable the device first. */

              max86178_enable(priv, enable);

              /* Set ECG clock to satisfy its interval. */

              max86178_ecg_calcu_clk(sensor, &sensor->interval_desired);
              max86178_ecg_set_sr(priv, sensor->decrate, sensor->ndiv);
              priv->interval = sensor->interval_desired;

              /* Set interrupt and FIFO watermark. */

              priv->batch = sensor->batch_desired;
              priv->fifowtm = priv->batch / priv->interval;
              max86178_fifo_set(priv);

              /* Start ECG measurement. Status and flags will be updated. If
               * use interrupt reading, the interrupt should be enabled first
               * to avoid missing first edge. If use polling, the first
               * polling must comes after ECG enabled or the polling will
               * stop after one time.
               */

              if (priv->fifowtm > 0)
                {
                  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                                  IOEXPANDER_OPTION_INTCFG,
                                  (FAR void *)IOEXPANDER_VAL_RISING);
                  max86178_ecg_enable(priv, true);
                }
              else
                {
                  max86178_ecg_enable(priv, true);
                  work_queue(HPWORK, &priv->work_poll, max86178_worker_poll,
                             priv, priv->interval / USEC_PER_TICK);
                }
            }
          else
            {
              sensor->activating = true;
              priv->update_activate = true;
            }
        }
      else
        {
          /* If it's going to be inactivated, it will take effect when next
           * FIFO reading completes.
           */

          sensor->inactivating = true;
          priv->update_activate = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_ecg_batch
 *
 * Description:
 *   Set ECG sensor's maximum report latency in microseconds.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   filep      - The pointer of file, represents each user using the sensor.
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

static int max86178_ecg_batch(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR unsigned long *latency_us)
{
  FAR struct max86178_ecg_sensor_s *sensor =
    (FAR struct max86178_ecg_sensor_s *)lower;
  FAR struct max86178_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL && latency_us != NULL);

  /* Get the pointer of the device from sensor. */

  priv = sensor->dev;

  /* Calculate the batch latency and record it if different from current
   * latency. New latency setting will not be congifured into MAX86178 now.
   * If sensor has been activated, the new latency will take effect when next
   * FIFO reading comes. If has not been activated, it will take effect when
   * being activated.
   */

  max86178_batch_calcu(priv, latency_us, sensor->lower.nbuffer,
                       sensor->interval_desired);
  if (*latency_us != sensor->batch_desired)
    {
      sensor->batch_desired = *latency_us;
      priv->update_batch = true;
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_ecg_set_interval
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

static int max86178_ecg_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR unsigned long *period_us)
{
  FAR struct max86178_ecg_sensor_s *sensor =
    (FAR struct max86178_ecg_sensor_s *)lower;
  FAR struct max86178_dev_s *priv;
  unsigned long interval;

  /* Sanity check */

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL && period_us != NULL);

  /* Get the pointer of the device from sensor. */

  priv = sensor->dev;

  /* Find the best matched period and correspoing clock parameters. Record
   * them if the new period is different from current interval. New interval
   * setting will not be congifured into MAX86178 immediately. If any sensor
   * has been activated, the new latency will take effect when next FIFO
   * reading comes. If has not been activated, it will take effect when being
   * activated.
   */

  interval = *period_us;
  max86178_ecg_calcu_clk(sensor, &interval);
  *period_us = interval;
  if (sensor->interval_desired != interval)
    {
      sensor->interval_desired = interval;
      priv->update_interval = true;
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_ecg_selftest
 *
 * Description:
 *   Selftest of ECG sensor, i.e. the selftest of MAX86178.
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

static int max86178_ecg_selftest(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 unsigned long arg)
{
  FAR struct max86178_ecg_sensor_s *sensor =
    (FAR struct max86178_ecg_sensor_s *)lower;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL);

  return max86178_selftest(sensor->dev, arg);
}

/****************************************************************************
 * Name: max86178_ecg_control
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

static int max86178_ecg_control(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                int cmd, unsigned long arg)
{
  FAR struct max86178_ecg_sensor_s *sensor =
    (FAR struct max86178_ecg_sensor_s *)lower;
  FAR struct max86178_dev_s *priv;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL);

  priv = sensor->dev;

  switch (cmd)
    {
      case MAX86178_ECG_CTRL_GAIN:     /* Set ECG gain. */
        {
          uint8_t regval;
          uint8_t gaincfg = (uint8_t)arg;

          if (gaincfg > 127)
            {
              return -EINVAL;
            }

          max86178_readsingle(priv, MAX86178_REG_ECGCFG2, &regval);
          regval = regval & (~MAX86178_ECGCFG2_PGAGAIN_MASK);
          regval = regval | gaincfg;
          max86178_writesingle(priv, MAX86178_REG_ECGCFG2, regval);
        }
        break;

      default:                         /* Invalid command. */
        {
          return -ENOTTY;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_ppg_activate
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

static int max86178_ppg_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable)
{
  FAR struct max86178_ppg_sensor_s *sensor =
    (FAR struct max86178_ppg_sensor_s *)lower;
  FAR struct max86178_dev_s *priv;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL);

  /* Get the pointer of the device from sensor. */

  priv = sensor->dev;

  /* Operate only if the activated status will change. */

  if (sensor->activated != enable)
    {
      if (enable)
        {
          /* If there's no activated sensor thus the device is shut-down, set
           * the PPG frame clock and FIFO settings, then activate this PPG
           * sensor. If there has been any activated sensor, record the new
           * activating, and it will be activated with its settings when the
           * next FIFO reading comes.
           */

          if (priv->ppg_activated == 0 &&
              priv->ecg_sensor.activated == false)
            {
              /* Enable the device first. */

              max86178_enable(priv, enable);

              /* Set PPG frame clock to satisfy its interval. */

              max86178_ppg_set_fps(priv, sensor->frdiv);
              priv->interval = sensor->interval_desired;

              /* Set interrupt and FIFO watermark. */

              priv->batch = sensor->batch_desired;
              priv->fifowtm = priv->batch / priv->interval * 2;
              max86178_fifo_set(priv);

              /* Start PPG measurement. Status and flags will be updated. If
               * use interrupt reading, the interrupt should be enabled first
               * to avoid missing first edge. If use polling, the first
               * polling must comes after ECG enabled or the polling will
               * stop after one time.
               */

              if (priv->fifowtm > 0)
                {
                  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                                  IOEXPANDER_OPTION_INTCFG,
                                  (FAR void *)IOEXPANDER_VAL_RISING);
                  max86178_ppg_enable(priv, sensor->chidx, true);
                }
              else
                {
                  max86178_ppg_enable(priv, sensor->chidx, true);
                  work_queue(HPWORK, &priv->work_poll, max86178_worker_poll,
                             priv, priv->interval / USEC_PER_TICK);
                }
            }
          else
            {
              sensor->activating = true;
              priv->update_activate = true;
            }
        }
      else
        {
          sensor->inactivating = true;
          priv->update_activate = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_ppg_batch
 *
 * Description:
 *   Set PPG sensor's maximum report latency in microseconds.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   filep      - The pointer of file, represents each user using the sensor.
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

static int max86178_ppg_batch(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR unsigned long *latency_us)
{
  FAR struct max86178_ppg_sensor_s *sensor =
    (FAR struct max86178_ppg_sensor_s *)lower;
  FAR struct max86178_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL && latency_us != NULL);

  /* Get the pointer of the device from sensor. */

  priv = sensor->dev;

  /* Calculate the batch latency and record it if different from current
   * latency. New latency setting will not be congifured into MAX86178 now.
   * If sensor has been activated, the new latency will take effect when next
   * FIFO reading comes. If has not been activated, it will take effect when
   * being activated.
   */

  max86178_batch_calcu(priv, latency_us, sensor->lower.nbuffer,
                       sensor->interval_desired);
  if (*latency_us != sensor->batch_desired)
    {
      sensor->batch_desired = *latency_us;
      priv->update_batch = true;
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_ppg_set_interval
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
 *   Currently timing data is disabled, then the FR_CLK comes from REF_CLK or
 *   ECG_ADC_CLK, which are both 32000Hz.
 *
 ****************************************************************************/

static int max86178_ppg_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR unsigned long *period_us)
{
  FAR struct max86178_ppg_sensor_s *sensor =
    (FAR struct max86178_ppg_sensor_s *)lower;
  FAR struct max86178_dev_s *priv;
  float freq;
  uint16_t frdiv;

  /* Sanity check */

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL && period_us != NULL);

  /* Get the pointer of the device from sensor. */

  priv = sensor->dev;

  /* Find the best matched period and record it if different from current
   * interval. New interval setting will not be congifured into MAX86178 now.
   * If any sensor has been activated, the new latency will take effect when
   * next FIFO reading comes. If has not been activated, it will take effect
   * when being activated.
   */

  frdiv = max86178_ppg_calcu_frdiv(*period_us);
  freq = MAX86178_REF_CLK_DFT / frdiv;
  *period_us = MAX86178_ONE_SECOND / freq;
  sensor->frdiv = frdiv;
  if (sensor->interval_desired != *period_us)
    {
      sensor->interval_desired = *period_us;
      priv->update_interval = true;
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_ppg_selftest
 *
 * Description:
 *   Selftest of PPG sensor, i.e. the selftest of MAX86178.
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

static int max86178_ppg_selftest(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 unsigned long arg)
{
  FAR struct max86178_ppg_sensor_s *sensor =
    (FAR struct max86178_ppg_sensor_s *)lower;

  DEBUGASSERT(sensor != NULL && sensor->dev != NULL);

  return max86178_selftest(sensor->dev, arg);
}

/****************************************************************************
 * Name: max86178_ppg_control
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

static int max86178_ppg_control(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                int cmd, unsigned long arg)
{
  FAR struct max86178_ppg_sensor_s *sensor =
    (FAR struct max86178_ppg_sensor_s *)lower;

  DEBUGASSERT(sensor != NULL);

  switch (cmd)
    {
      case MAX86178_PPG_CTRL_LEDPA:  /* Set PPG LED current */
        {
          FAR uint32_t *current = (FAR uint32_t *)arg;

          if (current == NULL)
            {
              snerr("Empty argument.\n");
              return -EINVAL;
            }

          if (sensor->chidx < MAX86178_PPG3_SENSOR_IDX)
            {
              if (*current != sensor->current)
                {
                  max86178_ppg_set_current(sensor, current);
                }
            }
          else
            {
              /* For PPG3 (dark) channel, current configurations will be
               * ignored. It seems to be always 0uA from APP's sight.
               */

              *current = 0;
            }
        }
        break;

      case MAX86178_PPG_CTRL_OPTM:  /* Set PPG auto-optimization mode */
        {
          FAR bool *enable = (FAR bool *)arg;

          if (enable == NULL)
            {
              snerr("Empty argument.\n");
              return -EINVAL;
            }

          sensor->auto_optimize = *enable;
        }
        break;

      default:                       /* Invalid command. */
        {
          return -ENOTTY;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: max86178_interrupt_handler
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

static int max86178_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                      ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the MAX86178 INTx
   * pin, when an event, such as FIFO is almost full, has occured.
   */

  FAR struct max86178_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp */

  priv->timestamp = sensor_get_timestamp();

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long, and neither should we
   * lock the I2C/SPI bus within an interrupt.
   */

  work_queue(HPWORK, &priv->work_intrpt, max86178_worker_intrpt, priv, 0);
  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);

  return OK;
}

/****************************************************************************
 * Name: max86178_worker_intrpt
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

static void max86178_worker_intrpt(FAR void *arg)
{
  FAR struct max86178_dev_s *priv = arg;
  uint8_t status[5];

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_RISING);

  /* Read registers from STAT1 to STAT5, where all interrupts come from. */

  max86178_readregs(priv, MAX86178_REG_STAT1, status, 5);

  /* If it's a FIFO almost full interrupt indicating FIFO reached the set
   * watermark, read and push the data to topics.
   */

  if ((status[0] & MAX86178_STAT1_FIFOAFULL) != 0)
    {
      max86178_fifo_read(priv);
      max86178_update_sensors(priv);
    }

  /* Procession for the other types of interrupt will be added later. */
}

/****************************************************************************
 * Name: max86178_worker_poll
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

static void max86178_worker_poll(FAR void *arg)
{
  FAR struct max86178_dev_s *priv = arg;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  priv->timestamp = sensor_get_timestamp();
  if (priv->activated && priv->fifowtm == 0)
    {
      work_queue(HPWORK, &priv->work_poll, max86178_worker_poll, priv,
                 priv->interval / USEC_PER_TICK);
    }

  max86178_fifo_read(priv);
  max86178_update_sensors(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max86178_register
 *
 * Description:
 *   Register the MAX86178 character device.
 *
 * Input Parameters:
 *   devno  - The full path to the driver to register. E.g., "/dev/max86178"
 *   config - An instance of the SPI interface to use to communicate with
 *             MAX86178
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int max86178_register(int devno, FAR const struct max86178_config_s *config)
{
  FAR struct max86178_dev_s *priv;
  FAR void *ioehandle;
  int idx;
  int ret;

  /* Sanity check */

  DEBUGASSERT(config != NULL);

  /* Initialize the MAX86178 device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (!priv)
    {
      snerr("Failed to allocate instance.\n");
      return -ENOMEM;
    }

  priv->config = config;

  /* Configure ECG sensor. */

  priv->ecg_sensor.lower.ops = &g_max86178_ecg_ops;
  priv->ecg_sensor.lower.type = SENSOR_TYPE_ECG;
  priv->ecg_sensor.lower.nbuffer = MAX86178_FIFO_SLOTS_ECG;
  priv->ecg_sensor.dev = priv;
  priv->ecg_sensor.interval_desired = MAX86178_ECG_INTVL_DFT;
  priv->ecg_sensor.ndiv = MAX86178_ECG_NDIV_DFT;
  priv->ecg_sensor.factor = MAX86178_ECG_GAIN_DFT;

  /* Configure PPG sensors. */

  for (idx = 0; idx < MAX86178_PPG_SENSOR_NUM; idx++)
    {
      priv->ppg_sensor[idx].lower.ops = &g_max86178_ppg_ops;
      priv->ppg_sensor[idx].lower.type = SENSOR_TYPE_PPGD;
      priv->ppg_sensor[idx].lower.nbuffer = MAX86178_FIFO_SLOTS_PPG;
      priv->ppg_sensor[idx].dev = priv;
      priv->ppg_sensor[idx].optm1.adc_range = MAX86178_PPG_ADCRGE_DFT;
      priv->ppg_sensor[idx].optm2.adc_range = MAX86178_PPG_ADCRGE_DFT;
      priv->ppg_sensor[idx].interval_desired = MAX86178_PPG_INTVL_DFT;
      priv->ppg_sensor[idx].frdiv = MAX86178_PPG_FRDIV_DFT;
      priv->ppg_sensor[idx].gain1 = MAX86178_PPG_GAIN_DFT;
      priv->ppg_sensor[idx].gain2 = MAX86178_PPG_GAIN_DFT;
      priv->ppg_sensor[idx].chidx = (uint8_t)idx;
      priv->ppg_sensor[idx].auto_optimize = true;
    }

  priv->ppg_sensor[0].current = MAX86178_PPG_GLEDC_DFT;
  priv->ppg_sensor[1].current = MAX86178_PPG_RLEDC_DFT;
  priv->ppg_sensor[2].current = MAX86178_PPG_IRLEDC_DFT;

  /* Check the part ID */

  ret = max86178_checkid(priv);
  if (ret < 0)
    {
      snerr("Device ID doesn't match: %d\n", ret);
      goto err_exit;
    }

  /* Device soft-reset and enter shutdown mode. */

  max86178_softreset(priv);
  max86178_shutdown(priv);

  /* Configure interrupt pin */

  ret = IOEXP_SETDIRECTION(priv->config->ioedev, priv->config->intpin,
                           IOEXPANDER_DIRECTION_IN);
  if (ret < 0)
    {
      snerr("Failed to set IO direction: %d\n", ret);
      goto err_exit;
    }

  ioehandle = IOEP_ATTACH(priv->config->ioedev, priv->config->intpin,
                          max86178_interrupt_handler, priv);
  if (ioehandle == NULL)
    {
      ret = -EIO;
      snerr("Failed to attach IO: %d\n", ret);
      goto err_exit;
    }

  ret = IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                        IOEXPANDER_OPTION_INTCFG,
                        (FAR void *)IOEXPANDER_VAL_DISABLE);
  if (ret < 0)
    {
      snerr("Failed to set IO option: %d\n", ret);
      goto err_iodetach;
    }

  /* Register the character driver */

  ret = sensor_register((&(priv->ecg_sensor.lower)), devno);
  if (ret < 0)
    {
      snerr("Failed to register ECG driver: %d\n", ret);
      goto err_iodetach;
    }

  for (idx = 0; idx < MAX86178_PPG_SENSOR_NUM; idx++)
    {
      ret = sensor_register((&(priv->ppg_sensor[idx].lower)),
                            devno * MAX86178_PPG_SENSOR_NUM + idx);
      if (ret < 0)
        {
          snerr("Failed to register PPG%d driver: %d\n", idx, ret);

          /* Unregister ecg sensor and all registered ppg sensors */

          sensor_unregister((&(priv->ecg_sensor.lower)), devno);
          idx--;
          for (; idx >= 0; idx--)
            {
              sensor_unregister((&(priv->ppg_sensor[idx].lower)),
                                devno * MAX86178_PPG_SENSOR_NUM + idx);
            }

          goto err_iodetach;
        }
    }

  return ret;

err_iodetach:
  IOEP_DETACH(priv->config->ioedev, max86178_interrupt_handler);

err_exit:
  kmm_free(priv);
  return ret;
}
