/****************************************************************************
 * drivers/sensors/ad5940.c
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
#include <nuttx/sensors/ad5940.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/wqueue.h>

#include "ad5940_def.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Misc. paramaters */

#define AD5940_INITTABLESIZE     14         /* Length of initial table. */
#define AD5940_WAIT_STEP         100        /* Wait time step is 100us. */
#define AD5940_TIMEOUT_HFOSC     10000      /* Wait <=10ms for HFOSC ready. */
#define AD5940_TIMEOUT_LFOSC     10000      /* Wait <=10ms for LFOSC ready. */
#define AD5940_TIMEOUT_DFTRDY    5000       /* Wait <=5ms for DFT ready. */
#define AD5940_TIMEOUT_INISEQ    5000       /* Wait <=5ms for init seq end. */
#define AD5940_FIFOSLOTS_MAX     20         /* FIFO can store 1000 results. */
#define AD5940_RCAL_RESISTOR     1000       /* RCAL resistor on board */

/* Configurations */

#define AD5940_FIFOWTM_DEFAULT   0          /* Default FIFO watermark is 0. */
#define AD5940_INTVL_DEFAULT     50000      /* Default inteval = 50 ms. */
#define AD5940_BIAODR_DEFAULT    20.0f      /* Default BIA out rate = 20Hz. */
#define AD5940_BIAODR_MAX        50.0f      /* Max. BIA output rate = 50Hz. */
#define AD5940_BIA_SWEEPPT_MAX   100        /* Points for BIA freq sweep. */
#define AD5940_BIA_BUFFSIZE      512        /* Buffer size for BIA SEQ. */
#define AD5940_BITWIDTH_WGFCW    30         /* WGFCW bitwidth = 30. */

/* BIA function control */

#define AD5940_BIACTRL_START     0          /* BIA function starts. */
#define AD5940_BIACTRL_RESTART   1          /* BIA restarts with new config */
#define AD5940_BIACTRL_STOPNOW   2          /* BIA function stops. */
#define AD5940_BIACTRL_GETFREQ   3          /* Get current frequency. */
#define AD5940_BIACTRL_SHUTDOWN  4          /* Power down all parts in AFE. */

/* Control commands */

#define AD5940_CTRL_CHECKID      0x00       /* Check ID cmd via selftest(). */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Registers addressed and contents list for AD5940 initialization. */

struct ad5940_init_regtable_s
{
  uint16_t regaddr;        /* Register address. */
  uint32_t regdata;        /* Register content. */
};

/* Structure used to store register information(address and data) in SEQ. */

struct ad5940_seqgenreginfo_s
{
  uint32_t regaddr  :8;   /* 8bit address is enough for sequencer. */
  uint32_t regval   :24;  /* Reg data is limited to 24bit by sequencer. */
};

/* Sequencer generator data base */

struct ad5940_seqgen_s
{
  FAR struct ad5940_seqgenreginfo_s
           *preginfo;     /* Pointer to buffer where stores register info */
  uint32_t buffersize;    /* Total buffer size */
  uint32_t *pseqbuff;     /* The buffer for sequence generator */
  uint32_t seqlen;        /* Generated sequence length till now */
  uint32_t regcount;      /* The count of REG info available in *preginfo */
  int      lasterror;     /* The last error message. */
  bool     enginestart;   /* Flag to mark start of the generator */
};

/* AD5940 configuration for BIA (body impedance analysis) */

struct ad5940_biacfg_s
{
  /* Sweep freqency configuration */

  struct ad5940_softsweepcfg_s sweepcfg;

  /* Initailize seq */

  struct ad5940_seqinfo_s initseqinfo;

  /* Measurement seq */

  struct ad5940_seqinfo_s measureseqinfo;

  /* CAL Rtia values */

  float    rtiacaltable[AD5940_BIA_SWEEPPT_MAX][2];

  float    rtiacurrvalue[2];    /* Calibrated Rtia value of current freq */
  float    sweepcurrfreq;       /* Sweep current frequency */
  float    sweepnextfreq;       /* Sweep next frequency */
  float    freqofdata;          /* The frequency of latest data sampled */
  uint32_t seqstartaddr;        /* Initialaztion sequence start in AD5940 */
  uint32_t maxseqlen;           /* Initialaztion sequence maximum size */
  uint32_t seqstartaddrcal;     /* Measurement sequence start in AD5940 */
  uint32_t maxseqlencal;        /* Measurement sequence maximum size */
  float    sysclkfreq;          /* The real frequency of system clock */
  float    wuptclkfreq;         /* The clock frequency of Wakeup Timer */
  float    adcclkfreq;          /* The real frequency of ADC clock */
  uint32_t fifothreshd;         /* FIFO threshold. Should be N*4 */
  float    biaodr;              /* BIA ODR */
  int      numofdata;           /* Default -1 means never stop. */
  float    sinfreq;             /* Frequency of excitation signal */
  float    rcalval;             /* Rcal value in Ohm */
  uint32_t pwrmod;              /* Control Chip power mode(LP/HP) */
  float    dacvoltpp;           /* DAC output voltage in mV, 800mVp-p max. */
  float    maxodr;              /* Max ODR for sampling in this config */
  uint32_t excitbufgain;        /* EXCITBUFGAIN_2 or EXCITBUFGAIN_0P25 */
  uint32_t hsdacgain;           /* HSDACGAIN_1 or HSDACGAIN_0P2 */
  uint32_t hsdacupdaterate;     /* DAC update rate = SYSCLK/DIV. 7~255. */
  uint32_t adcpgagain;          /* PGA Gain selection. -1.5V< Signal <1.5V */
  uint32_t hstiartiasel;        /* Use internal RTIA. */
  uint32_t ctiasel;             /* Select CTIA in pF unit from 0 to 31pF */
  uint32_t dftnum;              /* DFT number */
  uint32_t dftsrc;              /* DFT Source */
  uint32_t measseqcyclecount;   /* How long the measurement seq will take */
  uint8_t  adcsinc3osr;         /* SINC3 OSR selection */
  uint8_t  adcsinc2osr;         /* SINC2 OSR selection */
  bool     bparachanged;        /* Indicate to generate sequence again. */
  bool     redortiacal;         /* Set true if need calibration. */
  bool     hanwinen;            /* Enable Hanning window */
  bool     biainited;           /* Generated sequence commands */
};

/* Device struct */

struct ad5940_dev_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s        lower;         /* Lower half driver */
  struct work_s                    work_intrpt;   /* Interrupt handler */
  struct work_s                    work_poll;     /* Polling handler */
  uint64_t                         timestamp;     /* Units is us */
  struct ad5940_biacfg_s           biacfg;        /* BIA configure struct */
  FAR const struct ad5940_config_s *config;       /* The board config */
  FAR struct ad5940_seqgen_s       *pseqgendb;    /* SEQ generator database */

  /* Buffer to store data readout from sensor hardware FIFO */

  uint32_t                         fifobuf[AD5940_FIFOSLOTS_MAX
                                           * AD5940_RESULTS_PER_MEAS];
  unsigned long                    interval;      /* Sensor interval(us) */
  float                            odr;           /* Sensor ODR(Hz) */
  unsigned long                    batch_latency; /* Batch latency(us) */
  uint16_t                         fifowtm;       /* FIFO water marker */
  bool                             activated;     /* Device state */
  bool                             poll2batch;    /* Change polling->batch */
  bool                             batch2poll;    /* Change batch->polling */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* AD5940 register operations via interface (SPI) or sequence generator */

static int  ad5940_readreg(FAR struct ad5940_dev_s *priv, uint16_t regaddr,
                           FAR uint32_t *recvdata);
static int  ad5940_writereg(FAR struct ad5940_dev_s *priv, uint16_t regaddr,
                            uint32_t senddata);
static int  ad5940_readfifo(FAR struct ad5940_dev_s *priv,
                            FAR uint32_t *pbuff, uint32_t cnt);
static int  ad5940_configspi(FAR struct ad5940_dev_s *priv);
static void ad5940_spiexchange(FAR struct ad5940_dev_s *priv,
                               FAR uint8_t *txbuf, FAR uint8_t *rxbuf,
                               uint32_t nwords);
static int  ad5940_spireadreg(FAR struct ad5940_dev_s *priv,
                              uint16_t regaddr, FAR uint32_t *recvdata);
static int  ad5940_spiwritereg(FAR struct ad5940_dev_s *priv,
                               uint16_t regaddr, uint32_t senddata);
static int  ad5940_seqreadreg(FAR struct ad5940_dev_s *priv,
                              uint16_t regaddr, FAR uint32_t *regval);
static int  ad5940_seqwritereg(FAR struct ad5940_seqgen_s *pseqdb,
                               uint16_t regaddr, uint32_t regdata);
static int  ad5940_seqgeninsert(FAR struct ad5940_seqgen_s *pseqdb,
                                uint32_t cmdword);
static int  ad5940_seqgensearchreg(FAR struct ad5940_seqgen_s *pseqdb,
                                   uint32_t regaddr, FAR uint32_t *pindex);
static int  ad5940_seqreginfoinsert(FAR struct ad5940_seqgen_s *pseqdb,
                                    uint16_t regaddr, uint32_t regdata);

/* AD5940 common handle functions */

static int      ad5940_checkid(FAR struct ad5940_dev_s *priv);
static void     ad5940_hardreset(FAR struct ad5940_dev_s *priv);
static void     ad5940_init(FAR struct ad5940_dev_s *priv);
static void     ad5940_entersleep(FAR struct ad5940_dev_s *priv);
static int      ad5940_shutdown(FAR struct ad5940_dev_s *priv);
static void     ad5940_sleeplock(FAR struct ad5940_dev_s *priv, bool lock);
static int      ad5940_waitflag(FAR struct ad5940_dev_s *priv,
                                const uint32_t regaddr, const uint32_t mask,
                                const uint32_t maxtime);
static int      ad5940_wakeup(FAR struct ad5940_dev_s *priv, int trycnt);
static int      ad5940_adcbasecfg(FAR struct ad5940_dev_s *priv,
                                  FAR const struct ad5940_adcbasecfg_s
                                  *pcfg);
static int      ad5940_adcfiltercfg(FAR struct ad5940_dev_s *priv,
                                    FAR const struct ad5940_adcfiltercfg_s
                                    *pcfg);
static int      ad5940_adcmuxcfg(FAR struct ad5940_dev_s *priv,
                                 uint32_t adcmuxp, uint32_t adcmuxn);
static int      ad5940_afectrl(FAR struct ad5940_dev_s *priv,
                               uint32_t ctrlset, bool state);
static int      ad5940_clkcfg(FAR struct ad5940_dev_s *priv);
static void     ad5940_dftcfg(FAR struct ad5940_dev_s *priv,
                              FAR const struct ad5940_dftcfg_s *pcfg);
static int      ad5940_dspcfg(FAR struct ad5940_dev_s *priv,
                              FAR const struct ad5940_dspcfg_s *pcfg);
static void     ad5940_fifocfg(FAR struct ad5940_dev_s *priv,
                               FAR const struct ad5940_fifocfg_s *pfifocfg);
static void     ad5940_fifoctrl(FAR struct ad5940_dev_s *priv,
                                uint32_t fifosrc, bool fifoen);
static uint32_t ad5940_fifogetcnt(FAR struct ad5940_dev_s *priv);
static int      ad5940_hfosc16mhzctrl(FAR struct ad5940_dev_s *priv);
static void     ad5940_hsdaccfg(FAR struct ad5940_dev_s *priv,
                                FAR const struct ad5940_hsdaccfg_s *pcfg);
static int      ad5940_hsloopcfg(FAR struct ad5940_dev_s *priv,
                                 FAR const struct ad5940_hsloopcfg_s *pcfg);
static int      ad5940_hsrtiacal(FAR struct ad5940_dev_s *priv,
                                 FAR const struct ad5940_hsrtiacal_s *pcfg,
                                 FAR void *presult);
static int      ad5940_hstiacfg(FAR struct ad5940_dev_s *priv,
                                FAR const struct ad5940_hstiacfg_s *pcfg);
static void     ad5940_intrptcfg(FAR struct ad5940_dev_s *priv,
                                 uint32_t intctrlsel, uint32_t intsrc,
                                 bool state);
static void     ad5940_intrptclr(FAR struct ad5940_dev_s *priv,
                                 uint32_t intsrc);
static void     ad5940_lpampcfg(FAR struct ad5940_dev_s *priv,
                                FAR const struct ad5940_lpampcfg_s *pcfg);
static void     ad5940_lpdac0write(FAR struct ad5940_dev_s *priv,
                                   uint16_t data12bit, uint8_t data6bit);
static void     ad5940_lpdaccfg(FAR struct ad5940_dev_s *priv,
                                FAR const struct ad5940_lpdaccfg_s *pcfg);
static void     ad5940_lploopcfg(FAR struct ad5940_dev_s *priv,
                                 FAR const struct ad5940_lploopcfg_s *pcfg);
static void     ad5940_refcfg(FAR struct ad5940_dev_s *priv,
                              FAR const struct ad5940_aferefcfg_s *pcfg);
static void     ad5940_seqcfg(FAR struct ad5940_dev_s *priv,
                              FAR const struct ad5940_seqcfg_s *pcfg);
static void     ad5940_seqgeninit(FAR struct ad5940_dev_s *priv,
                                  FAR struct ad5940_seqgen_s *pseqdb,
                                  uint32_t *pbuffer, uint32_t buffersize);
static int      ad5940_seqinfocfg(FAR struct ad5940_dev_s *priv,
                                  FAR struct ad5940_seqinfo_s *pseq);
static void     ad5940_setdexrtia(FAR struct ad5940_dev_s *priv,
                                  uint32_t dexpin, uint32_t dertia,
                                  uint32_t derload);
static void     ad5940_set_intpin(FAR struct ad5940_dev_s *priv,
                                  bool enable);
static void     ad5940_statisticcfg(FAR struct ad5940_dev_s *priv,
                                    FAR const struct ad5940_statcfg_s
                                    *pstatcfg);
static void     ad5940_swmatrixcfg(FAR struct ad5940_dev_s *priv,
                                   FAR const struct ad5940_swmatrixcfg_s
                                   *pcfg);
static void     ad5940_wgcfg(FAR struct ad5940_dev_s *priv,
                             FAR const struct ad5940_wgcfg_s *pcfg);
static void     ad5940_wgfreqctrl(FAR struct ad5940_dev_s *priv,
                                  float sinfreqhz, float wgclock);
static void     ad5940_wuptcfg(FAR struct ad5940_dev_s *priv,
                                FAR struct ad5940_wuptcfg_s *pcfg);
static void     ad5940_wuptctrl(FAR struct ad5940_dev_s *priv, bool enable);

/* AD5940 handle functions for specified usage */

static void ad5940_biacfgstructinit(FAR struct ad5940_dev_s *priv,
                                    FAR struct ad5940_biacfg_s *pbiacfg);
static int  ad5940_biaplatformcfg(FAR struct ad5940_dev_s *priv);
static int  ad5940_biafuncinit(FAR struct ad5940_dev_s *priv,
                               FAR struct ad5940_biacfg_s *pbiacfg,
                               const uint32_t bufsize);
static int  ad5940_biactrl(FAR struct ad5940_dev_s *priv,
                           FAR struct ad5940_biacfg_s *pbiacfg,
                           int32_t ctrlcmd, FAR void *ppara);
static int  ad5940_biadataprocess(FAR struct ad5940_dev_s *priv,
                                  FAR uint32_t *pdata, uint32_t cnt);
static int  ad5940_biaregmodify(FAR struct ad5940_dev_s *priv,
                                FAR struct ad5940_biacfg_s *pbiacfg);
static int  ad5940_biartiacal(FAR struct ad5940_dev_s *priv,
                              FAR struct ad5940_biacfg_s *pbiacfg);
static int  ad5940_biaseqcfggen(FAR struct ad5940_dev_s *priv,
                                FAR struct ad5940_biacfg_s *pbiacfg);
static int  ad5940_biaseqmeasuregen(FAR struct ad5940_dev_s *priv,
                                    FAR struct ad5940_biacfg_s *pbiacfg);

/* AD5940 data operations */

static bool     ad5940_checktable(uint8_t value, const uint8_t *table,
                                  uint32_t len, uint32_t *index);
static int      ad5940_clkscalculate(FAR struct ad5940_clkscalinfo_s
                                     *pfilterinfo, FAR uint32_t *pclocks);
static FAR struct ad5940_fimpcar_s
                ad5940_complexdivint(FAR const struct ad5940_iimpcar_s *a,
                                     FAR const struct ad5940_iimpcar_s *b);
static float    ad5940_complexmag(FAR const struct ad5940_fimpcar_s *a);
static float    ad5940_complexphase(FAR const struct ad5940_fimpcar_s *a);
static bool     ad5940_notch50hzavailable(FAR const struct
                                          ad5940_adcfiltercfg_s *pfilterinfo,
                                          uint8_t *dl);
static bool     ad5940_notch60hzavailable(FAR const struct
                                          ad5940_adcfiltercfg_s *pfilterinfo,
                                          uint8_t *dl);
static uint32_t ad5940_seqcycletime(FAR struct ad5940_seqgen_s *pseqdb);
static int      ad5940_seqgenfetchseq(FAR struct ad5940_seqgen_s *pseqdb,
                                      FAR const uint32_t **ppseqcmd,
                                      FAR uint32_t *pseqlen);
static void     ad5940_seqgenstart(FAR struct ad5940_seqgen_s *pseqdb);
static void     ad5940_sweepnext(FAR struct ad5940_softsweepcfg_s *pcfg,
                                 float *pnextfreq);
static uint32_t ad5940_wgfreqwordcal(float sinfreqhz, float wgclock);

/* Sensor ops functions */

static int  ad5940_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            bool enable);
static int  ad5940_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *period_us);
static int  ad5940_batch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR unsigned long *latency_us);
static int  ad5940_selftest(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            unsigned long arg);

/* Sensor interrupt functions */

static int  ad5940_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg);
static void ad5940_worker_intrpt(FAR void *arg);
static void ad5940_worker_poll(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This table is provided by Analog Device Inc. for AD5940 initialization.
 * The excat values must be written to its responding registers in the order
 * strictly when initializing. Some registers are reserved for initializing
 * only and must be not operated when using it normally. Never change this
 * table until Analog Device Inc. provides a new version.
 *    addr,  value
 */

FAR const struct ad5940_init_regtable_s
  ad5940_init_regtable[AD5940_INITTABLESIZE] =
  {
    {0x0908, 0x02c9},
    {0x0c08, 0x206c},
    {0x21f0, 0x0010},
    {0x0410, 0x02c9},
    {0x0a28, 0x0009},
    {0x238c, 0x0104},
    {0x0a04, 0x4859},
    {0x0a04, 0xf27b},
    {0x0a00, 0x8009},
    {0x22f0, 0x0000},
    {0x2230, 0xde87a5af},
    {0x2250, 0x103f},
    {0x22b0, 0x203c},
    {0x2230, 0xde87a5a0},
  };

/* This table if provided by Analog Device Inc. for AD5940 RTIA calibration.
 * Never change this table until Analog Device Inc. provides a new version.
 */

const uint32_t ad5940_hprtiatable[9] =
{
  200, 1000, 5000, 10000, 20000, 40000, 80000, 160000, 0,
};

/* This table if provided by Analog Device Inc. for AD5940 RTIA calibration.
 * Never change this table until Analog Device Inc. provides a new version.
 *  0    10    30    50    100 Ohm Rload
 */

const uint8_t ad5940_dertiatable[3][5] =
{
  {0x00, 0x01, 0x02, 0x03, 0x06}, /* RTIA 50Ohm */
  {0x03, 0x04, 0x05, 0x06, 0x07}, /* RTIA 100Ohm */
  {0x07, 0x07, 0x09, 0x09, 0x0a}, /* RTIA 200Ohm */
};

/* Following values are provided by Analog Device Inc. for BIA (default) */

FAR const struct ad5940_biacfg_s ad5940_biacfgdefault =
{
  .seqstartaddr = 0,
  .maxseqlen = 512,
  .seqstartaddrcal = 0,
  .maxseqlencal = 0,
  .sysclkfreq = 16000000.0,
  .wuptclkfreq = 32000.0,
  .adcclkfreq = 16000000.0,
  .biaodr = 20.0,
  .numofdata = -1,
  .rcalval = AD5940_RCAL_RESISTOR,
  .pwrmod = AD5940_AFEPWR_LP,
  .hstiartiasel = AD5940_HSTIARTIA_1K,
  .ctiasel = 16,
  .excitbufgain = AD5940_EXCITBUFGAIN_2,
  .hsdacgain = AD5940_HSDACGAIN_1,
  .hsdacupdaterate = 7,
  .dacvoltpp = 800.0,
  .sinfreq = 50000.0,
  .adcpgagain = AD5940_ADCPGA_1,
  .adcsinc3osr = AD5940_ADCSINC3OSR_2,
  .adcsinc2osr = AD5940_ADCSINC2OSR_22,
  .dftnum = AD5940_DFTNUM_8192,
  .dftsrc = AD5940_DFTSRC_SINC3,
  .sweepcfg.sweepen = false,
  .sweepcfg.sweepstart = 10000,
  .sweepcfg.sweepstop = 150000.0,
  .sweepcfg.sweeppoints = 100,
  .sweepcfg.sweeplog = true,
  .sweepcfg.sweepindex = 0,
  .fifothreshd = 4,
  .bparachanged = false,
  .biainited = false,
  .hanwinen = true,
  .redortiacal = false,
  .measseqcyclecount = 0,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const struct sensor_ops_s g_ad5940_ops =
{
  .activate     = ad5940_activate,
  .set_interval = ad5940_set_interval,
  .batch        = ad5940_batch,
  .selftest     = ad5940_selftest,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ad5940_readreg
 *
 * Description:
 *   Read an AD5940 register/sequencer generator from the specified address
 *   (except FIFO).
 *
 * Input Parameters:
 *   priv     - Device struct.
 *   regaddr  - Address of the register to be read.
 *   recvdata - A pointer to the variable which will store the readout.
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_readreg(FAR struct ad5940_dev_s *priv, uint16_t regaddr,
                          FAR uint32_t *recvdata)
{
  if (priv->pseqgendb != NULL && priv->pseqgendb->enginestart == true)
    {
      return ad5940_seqreadreg(priv, regaddr, recvdata);
    }
  else
    {
      return ad5940_spireadreg(priv, regaddr, recvdata);
    }
}

/****************************************************************************
 * Name: ad5940_writereg
 *
 * Description:
 *   Write an AD5940 register/sequence generator to the specified address
 *   (except FIFO).
 *
 * Input Parameters:
 *   priv     - Device struct.
 *   regaddr  - Address of the register to be written.
 *   senddata - The data to be written.
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_writereg(FAR struct ad5940_dev_s *priv, uint16_t regaddr,
                           uint32_t senddata)
{
  if (priv->pseqgendb != NULL && priv->pseqgendb->enginestart == true)
    {
      return ad5940_seqwritereg(priv->pseqgendb, regaddr, senddata);
    }
  else
    {
      return ad5940_spiwritereg(priv, regaddr, senddata);
    }
}

/****************************************************************************
 * Name: ad5940_readfifo
 *
 * Description:
 *   Read the data fifo of AD5940.
 *   (1) If there are less than 3 results in the data FIFO, the data can be
 *       read back from the DATAFIFORD register, like reading a 32b register.
 *       First:
 *       CS low |    Byte 0     | Byte 1~2 | CS high (>80ns)
 *              | Cmd: set addr | Reg addr |
 *       Then:
 *       CS low |    Byte 0     | Byte 1 | 4bytes | 4bytes | ... | CS high
 *              | Cmd: read reg | Dummy  | Data 0 | Data 1 | ... |
 *   (2) However, if there are more than three results in the FIFO, a more
 *       efficient SPI transaction protocol is implemented as:
 *       First:
 *       CS low |     Byte 0     | Byte 1~6 | 0x00 * 4 | 0x00 * 4 | ...
 *              | Cmd: read fifo |   Dummy  |  Data 0  |  Data 1  |
 *       When it comes to read the last 2 results:
 *       | 0xXX(None-zero) * 4 | 0xXX(None-zero) * 4 | CS high
 *       | Cmd: write reg |    Write data(16b/32b)   |
 *   Below is why reading the last 2 results is special:
 *   The fifo data is returned on the advanced peripheral bus (APB). The APB
 *   reads Data N when the SPI bus is transferring Data N-2. Assuming Data N
 *   is the last data in the FIFO, the read offset for Data N-1 on MOSI is
 *   set to a nonzero value. Then, the APB reads a different register than
 *   the DATAFIFORD register. If zeros are transmitted on MOSI when reading
 *   Data N-1, the APB will continue to read the Data N+1 in DATAFIFORD
 *   register, then the data FIFO underflows, which causes an underflow
 *   error.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pbuf - A pointer to the buffer which will store the readouts.
 *   cnt  - The number of data to be read.
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_readfifo(FAR struct ad5940_dev_s *priv,
                           FAR uint32_t *pbuff, uint32_t cnt)
{
  uint8_t rxbuf[AD5940_BYTES_PER_RESULT];
  int i;
  int ret;

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  ret = ad5940_configspi(priv);
  if (ret < 0)
    {
      SPI_LOCK(priv->config->spi, false);
      return ret;
    }

  if (cnt < AD5940_RDFIFO_CRITIC)
    {
      /* If we read only 1 or 2 results, read from DATAFIFORD register. */

      uint8_t txbuf_setaddr[AD5940_SPICNT_SETADDR] = {
        AD5940_SPICMD_SETADDR, (uint8_t)(AD5940_REG_AFE_DATAFIFORD >> 8),
        (uint8_t)AD5940_REG_AFE_DATAFIFORD
        };

      uint8_t txbuf_read[AD5940_SPICNT_READREG32] = {
        AD5940_SPICMD_READREG, 0, 0, 0, 0, 0
        };

      /* Send address to operate first, then remain CS high for 1 us. */

      ad5940_spiexchange(priv, txbuf_setaddr, rxbuf, AD5940_SPICNT_SETADDR);
      up_udelay(1);

      /* Then read results from REG_DATAFIFORD one by one. */

      for (i = 0; i < cnt; i++)
        {
          /* For each result, repeat transferring "CS low, Cmd-read reg,
           * dummy byte, 4bytes data, CS high" on SPI.
           */

          ad5940_spiexchange(priv, txbuf_read, rxbuf,
                             AD5940_SPICNT_READREG32);
          up_udelay(1);
          pbuff[i] = ((uint32_t)rxbuf[0] << 24) |
                     ((uint32_t)rxbuf[1] << 16) |
                     ((uint32_t)rxbuf[2] << 8) | (uint32_t)rxbuf[3];
        }
    }
  else
    {
      /* If we read 3 results or more, use READFIFO command. */

      uint8_t txbuf_preread[AD5940_SPICNT_PREFIFO] = {
        AD5940_SPICMD_READFIFO, 0, 0, 0, 0, 0, 0
        };

      uint8_t txbuf_zero[AD5940_BYTES_PER_RESULT] = {
        0, 0, 0, 0
        };

      uint8_t txbuf_nonzero[AD5940_BYTES_PER_RESULT] = {
        AD5940_SPIOFFSET_NONZERO, AD5940_SPIOFFSET_NONZERO,
        AD5940_SPIOFFSET_NONZERO, AD5940_SPIOFFSET_NONZERO
        };

      /* Active CS to select device. Release till FIFO reading finishes. */

      SPI_SELECT(priv->config->spi, priv->config->cs, true);
      IOEXP_WRITEPIN(priv->config->ioedev, priv->config->gpiocs, 0);

      /* First send the command "read fifo" and following 6 dummy bytes. */

      SPI_EXCHANGE(priv->config->spi, txbuf_preread, txbuf_preread,
                   AD5940_SPICNT_PREFIFO);

      /* Then read results from REG_DATAFIFORD one by one until the last 2. */

      for (i = 0; i < cnt - AD5940_RDFIFO_TAIL_SIZE; i++)
        {
          SPI_EXCHANGE(priv->config->spi, txbuf_zero, rxbuf,
                       AD5940_BYTES_PER_RESULT);
          pbuff[i] = ((uint32_t)rxbuf[0] << 24) |
                     ((uint32_t)rxbuf[1] << 16) |
                     ((uint32_t)rxbuf[2] << 8) | (uint32_t)rxbuf[3];
        }

      /* Last 2 bytes need to be read with non-zero offset on SPI MOSI */

      for (; i < cnt; i++)
        {
          SPI_EXCHANGE(priv->config->spi, txbuf_nonzero, rxbuf,
                       AD5940_BYTES_PER_RESULT);
          pbuff[i] = ((uint32_t)rxbuf[0] << 24) |
                     ((uint32_t)rxbuf[1] << 16) |
                     ((uint32_t)rxbuf[2] << 8) | (uint32_t)rxbuf[3];
        }

      /* Set CS high and deselect the device. */

      IOEXP_WRITEPIN(priv->config->ioedev, priv->config->gpiocs, 1);
      SPI_SELECT(priv->config->spi, priv->config->cs, false);
    }

  /* Release the SPI bus at last. */

  SPI_LOCK(priv->config->spi, false);
  return OK;
}

/****************************************************************************
 * Name: ad5940_configspi
 *
 * Description:
 *   Set SPI mode, frequency and bits per word, according to ad5940_dev_s->
 *   ad5940_config_s.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_configspi(FAR struct ad5940_dev_s *priv)
{
  int freq = priv->config->freq;

  /* Set SPI frequency in an acceptable range. */

  if (freq > AD5940_SPI_MAX_FREQ)
    {
      freq = AD5940_SPI_MAX_FREQ;
    }
  else if (freq < AD5940_SPI_MIN_FREQ)
    {
      freq = AD5940_SPI_MIN_FREQ;
    }

  if (SPI_SETFREQUENCY(priv->config->spi, freq) != freq)
    {
      snerr("Failed to configure SPI.\n");
      return -EIO;
    }

  /* AD5940 SPI supports only mode0. */

  SPI_SETMODE(priv->config->spi, SPIDEV_MODE0);

  /* Set number of bits per word. */

  SPI_SETBITS(priv->config->spi, AD5940_SPI_NBITS);

  return OK;
}

/****************************************************************************
 * Name: ad5940_spiexchange
 *
 * Description:
 *   SPI exchange consists of SPI selecting, CS(GPIO) controlling and
 *   exchange. Use this function to reduce code length.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   txbuf  - A pointer to the array which stored the content to send.
 *   rxbuf  - A pointer to the array which will store the readout.
 *   nwords - Number of words to exchange.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_spiexchange(FAR struct ad5940_dev_s *priv,
                               FAR uint8_t *txbuf, FAR uint8_t *rxbuf,
                               uint32_t nwords)
{
  /* Set CS to low and select the device. */

  SPI_SELECT(priv->config->spi, priv->config->cs, true);
  IOEXP_WRITEPIN(priv->config->ioedev, priv->config->gpiocs, 0);

  /* Exchange the content. */

  SPI_EXCHANGE(priv->config->spi, txbuf, rxbuf, nwords);

  /* Deselect the device for at lead 80ns. */

  IOEXP_WRITEPIN(priv->config->ioedev, priv->config->gpiocs, 1);
  SPI_SELECT(priv->config->spi, priv->config->cs, false);
}

/****************************************************************************
 * Name: ad5940_spireadreg
 *
 * Description:
 *   Read an AD5940 register from the specified address(except FIFO). Some
 *   registers have 16 bits while the others have 32 bits. For reading，set
 *   register address first:
 *   CS low |    Byte 0     | Byte 1~2 | CS high (>80ns)
 *          | Cmd: set addr | Reg addr |
 *   Then read the register:
 *   CS low |    Byte 0     |   Byte 1   | Byte 2~3(16b)/2~5(32b) | CS high
 *          | Cmd: read reg | Dummy read |   Read data(16b/32b)   |
 *
 * Input Parameters:
 *   priv     - Device struct.
 *   regaddr  - Address of the register to be read.
 *   recvdata - A pointer to the variable which will store the readout.
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_spireadreg(FAR struct ad5940_dev_s *priv, uint16_t regaddr,
                             FAR uint32_t *recvdata)
{
  int ret;

  uint8_t txbuf_setaddr[AD5940_SPICNT_SETADDR] = {
    AD5940_SPICMD_SETADDR, (uint8_t)(regaddr >> 8), (uint8_t)regaddr,
    };

  uint8_t txbuf_read[AD5940_SPICNT_READREG32] = {
    AD5940_SPICMD_READREG, 0, 0, 0, 0, 0,
    };

  uint8_t rxbuf[AD5940_SPICNT_READREG32];

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  ret = ad5940_configspi(priv);
  if (ret < 0)
    {
      SPI_LOCK(priv->config->spi, false);
      return ret;
    }

  /* Send the address to read first, then wait for 1 us. */

  ad5940_spiexchange(priv, txbuf_setaddr, rxbuf, AD5940_SPICNT_SETADDR);
  up_udelay(1);

  /* Read from the address set. */

  if (regaddr < AD5940_REGADDR_STARTOF32 || regaddr > AD5940_REGADDR_ENDOF32)
    {
      /* For 0 <= register addrress < 0x1000, or register address > 0x3014,
       * the register has 16bits, one reading needs to trans 4 bytes. The
       * readout is in byte 2 (MSB) and byte 3 (LSB).
       */

      ad5940_spiexchange(priv, txbuf_read, rxbuf, AD5940_SPICNT_READREG16);
      *recvdata = ((uint32_t)rxbuf[2] << 8) | rxbuf[3];
    }
  else
    {
      /* For 0x1000 <= register addrress <= 0x3014, the register has 32bits,
       * one reading needs to trans 6 bytes. The readout is stored in byte 2
       * (MSB) to byte 5 (LSB).
       */

      ad5940_spiexchange(priv, txbuf_read, rxbuf, AD5940_SPICNT_READREG32);
      *recvdata = ((uint32_t)rxbuf[2] << 24) |
                  ((uint32_t)rxbuf[3] << 16) |
                  ((uint32_t)rxbuf[4] << 8) | rxbuf[5];
    }

  /* Release the SPI bus. */

  SPI_LOCK(priv->config->spi, false);
  return OK;
}

/****************************************************************************
 * Name: ad5940_spiwritereg
 *
 * Description:
 *   Write an AD5940 register to the specified address(except FIFO). Some
 *   registers have 16 bits while the others have 32 bits. For writing，set
 *   register address first:
 *   CS low |    Byte 0     | Byte 1~2 | CS high (>80ns)
 *          | Cmd: set addr | Reg addr |
 *   Then write the register:
 *   CS low |     Byte 0     |   Byte 1~2(16b)/1~4(32b) | CS high
 *          | Cmd: write reg |    Write data(16b/32b)   |
 *
 * Input Parameters:
 *   priv     - Device struct.
 *   regaddr  - Address of the register to be written.
 *   senddata - The data to be written.
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_spiwritereg(FAR struct ad5940_dev_s *priv,
                              uint16_t regaddr, uint32_t senddata)
{
  int ret;

  uint8_t txbuf_setaddr[AD5940_SPICNT_SETADDR] = {
    AD5940_SPICMD_SETADDR, (uint8_t)(regaddr >> 8), (uint8_t)regaddr
    };

  uint8_t txbuf_write[AD5940_SPICNT_WRITEREG32] = {
    AD5940_SPICMD_WRITEREG, 0, 0, 0, 0
    };

  /* Lock the SPI bus thus only one device can access it at the same time. */

  SPI_LOCK(priv->config->spi, true);

  /* Configure SPI frequency, mode and number of bits per word. */

  ret = ad5940_configspi(priv);
  if (ret < 0)
    {
      SPI_LOCK(priv->config->spi, false);
      return ret;
    }

  /* Set address to operate first, then wait for 1 us. */

  ad5940_spiexchange(priv, txbuf_setaddr, txbuf_setaddr,
                     AD5940_SPICNT_SETADDR);
  up_udelay(1);

  /* Write the register. */

  if (regaddr < AD5940_REGADDR_STARTOF32 || regaddr > AD5940_REGADDR_ENDOF32)
    {
      /* For 0 <= register addrress < 0x1000, or register address > 0x3014,
       * the register has 16bits, one writing needs to trans 3 bytes. The
       * data to send should be stored in byte 1 (MSB) and byte 2 (LSB).
       */

      txbuf_write[1] = senddata >> 8;
      txbuf_write[2] = senddata;
      ad5940_spiexchange(priv, txbuf_write, txbuf_write,
                         AD5940_SPICNT_WRITEREG16);
    }
  else
    {
      /* For 0x1000 <= register addrress <= 0x3014, the register has 32bits,
       * one writing needs to trans 5 bytes. The data to send should be
       * stored in byte 1 (MSB) to byte 4 (LSB).
       */

      txbuf_write[1] = senddata >> 24;
      txbuf_write[2] = senddata >> 16;
      txbuf_write[3] = senddata >> 8;
      txbuf_write[4] = senddata;
      ad5940_spiexchange(priv, txbuf_write, txbuf_write,
                         AD5940_SPICNT_WRITEREG32);
    }

  /* Release the SPI bus. */

  SPI_LOCK(priv->config->spi, false);
  return OK;
}

/****************************************************************************
 * Name: ad5940_seqreadreg
 *
 * Description:
 *   Get current register value. If we have record in data-base, read it.
 *   Otherwise, return the register default value.
 *
 * Input Parameters:
 *   priv     - Device struct.
 *   regaddr  - Address of the register to be read.
 *   recvdata - A pointer to the variable which will store the readout.
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_seqreadreg(FAR struct ad5940_dev_s *priv,
                             uint16_t regaddr, FAR uint32_t *regval)
{
  uint32_t regindex;
  int ret;

  if (ad5940_seqgensearchreg(priv->pseqgendb, regaddr, &regindex) != OK)
    {
      /* There is no record in data-base, read the default value from AD5940
       * via SPI.  This requires AD5940 in active state.
       */

      ret = ad5940_spireadreg(priv, regaddr, regval);
      if (ret < 0)
        {
          return ret;
        }

      ret = ad5940_seqreginfoinsert(priv->pseqgendb, regaddr, *regval);
      if (ret < 0)
        {
          return ret;
        }
    }
  else
    {
      /* If there's a record in database, return the recorded value. */

      *regval = priv->pseqgendb->preginfo[regindex].regval;
    }

  return OK;
}

/****************************************************************************
 * Name: ad5940_seqwritereg
 *
 * Description:
 *   Generate a sequencer command to write register. If the register address
 *   is out of range, it won't generate a command. This function will also
 *   update the register-info in data-base to record current register value.
 *
 * Input Parameters:
 *   pseqdb  - Pointer to sequence generator data base struct.
 *   regaddr - Register address.
 *   regdata - Register value.
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_seqwritereg(FAR struct ad5940_seqgen_s *pseqdb,
                              uint16_t regaddr, uint32_t regdata)
{
  uint32_t regindex;
  int ret;

  if (regaddr > AD5940_REGADDR_ENDOFSEQ)
    {
      pseqdb->lasterror = -EINVAL;
      snerr("AD5940 SEQ writing was out of range.\n");
      return -EINVAL;
    }

  if (ad5940_seqgensearchreg(pseqdb, regaddr, &regindex) == OK)
    {
      /* Store register value */

      pseqdb->preginfo[regindex].regval = regdata;
    }
  else
    {
      ret = ad5940_seqreginfoinsert(pseqdb, regaddr, regdata);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Generate Sequence command */

  return ad5940_seqgeninsert(pseqdb, AD5940_SEQ_WR(regaddr, regdata));
}

/****************************************************************************
 * Name: ad5940_seqgeninsert
 *
 * Description:
 *   Manually input a command to sequencer generator.
 *
 * Input Parameters:
 *   pseqdb  - Pointer to sequence generator data base struct.
 *   cmdword - The 32-bit width sequencer command word.
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_seqgeninsert(FAR struct ad5940_seqgen_s *pseqdb,
                               uint32_t cmdword)
{
  uint32_t temp = pseqdb->regcount + pseqdb->seqlen;

  /* Generate Sequence command */

  if (temp < pseqdb->buffersize)
    {
      pseqdb->pseqbuff[pseqdb->seqlen] = cmdword;
      pseqdb->seqlen++;
    }
  else
    {
      /* There is no buffer */

      snerr("AD5940 SEQ buffer was not enough for command.\n");
      pseqdb->lasterror = -ENOBUFS;
      return -ENOBUFS;
    }

  return OK;
}

/****************************************************************************
 * Name: ad5940_seqgensearchreg
 *
 * Description:
 *   Search data-base to get current register value.
 *
 * Input Parameters:
 *   pseqdb  - Pointer to sequence generator data base struct.
 *   regaddr - The register address.
 *   pindex  - Pointer to a variable that used to store index of found
 *             register-info.
 *
 * Returned Value:
 *   OK if register found in data-base, otherwise return a negated errno.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_seqgensearchreg(FAR struct ad5940_seqgen_s *pseqdb,
                                  uint32_t regaddr, FAR uint32_t *pindex)
{
  uint32_t i;

  regaddr = (regaddr >> 2) & 0xff;
  for (i = 0; i < pseqdb->regcount; i++)
    {
      if (regaddr == pseqdb->preginfo[i].regaddr)
        {
          *pindex = i;
          return OK;
        }
    }

  return -ENODATA;
}

/****************************************************************************
 * Name: ad5940_seqreginfoinsert
 *
 * Description:
 *   Record the current register info to data-base. Update LastError if there
 *   is an error.
 *
 * Input Parameters:
 *   pseqdb  - Pointer to sequence generator data base struct.
 *   regaddr - Register address.
 *   regdata - Register value.
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_seqreginfoinsert(FAR struct ad5940_seqgen_s *pseqdb,
                                    uint16_t regaddr, uint32_t regdata)
{
  uint32_t temp = pseqdb->regcount + pseqdb->seqlen;

  if (temp < pseqdb->buffersize)
    {
      pseqdb->preginfo--;
      pseqdb->preginfo[0].regaddr = (regaddr >> 2) & 0xff;
      pseqdb->preginfo[0].regval = regdata & 0x00ffffff;
      pseqdb->regcount++;
      return OK;
    }
  else
    {
      /* There is no more buffer  */

      snerr("AD5940 SEQ buffer was not enough for reg info.\n");
      pseqdb->lasterror = -ENOBUFS;
      return -ENOBUFS;
    }
}

/****************************************************************************
 * Name: ad5940_checkid
 *
 * Description:
 *   Read and verify the AD5940 manufacturer ID and chip ID.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_checkid(FAR struct ad5940_dev_s *priv)
{
  uint32_t manufid;
  uint32_t chipid;
  int ret;

  ret = ad5940_readreg(priv, AD5940_REG_AFECON_ADIID, &manufid);
  if (ret < 0)
    {
      snerr("AD5940 failed to read manufactory ID.\n");
      return ret;
    }

  ret = ad5940_readreg(priv, AD5940_REG_AFECON_CHIPID, &chipid);
  if (ret < 0)
    {
      snerr("AD5940 failed to read chip ID.\n");
      return ret;
    }

  if (manufid != AD5940_ADIID || chipid != AD5940_CHIPID)
    {
      snerr("AD5940 ID mismatched.\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: ad5940_hardreset
 *
 * Description:
 *   Hardware reset the device.
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

static void ad5940_hardreset(FAR struct ad5940_dev_s *priv)
{
  IOEXP_WRITEPIN(priv->config->ioerpmsg, priv->config->gpiorst, 0);
  up_udelay(AD5940_RST_LOW_TIME);
  IOEXP_WRITEPIN(priv->config->ioerpmsg, priv->config->gpiorst, 1);
  up_udelay(AD5940_RST_HIGH_TIME);
}

/****************************************************************************
 * Name: ad5940_init
 *
 * Description:
 *   Initialize AD5940. This function must be called whenever a reset
 *   (software reset or hardware reset or power up) happened. This function
 *   is used to put AD5940 to correct state.
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

static void ad5940_init(FAR struct ad5940_dev_s *priv)
{
  uint8_t i;

  for (i = 0; i < AD5940_INITTABLESIZE; i++)
    {
      ad5940_writereg(priv, ad5940_init_regtable[i].regaddr,
                      ad5940_init_regtable[i].regdata);
    }
}

/****************************************************************************
 * Name: ad5940_entersleep
 *
 * Description:
 *   Put AFE to hibernate.
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

static void ad5940_entersleep(FAR struct ad5940_dev_s *priv)
{
  ad5940_writereg(priv, AD5940_REG_AFE_SEQTRGSLP, 0);
  ad5940_writereg(priv, AD5940_REG_AFE_SEQTRGSLP,
                  AD5940_BITM_AFE_SEQTRGSLP_TRGSLP);
}

/****************************************************************************
 * Name: ad5940_shutdowm
 *
 * Description:
 *   Turn off LP-Loop and put AFE to hibernate mode. By function
 *   ad5940_entersleep, we can put most blocks to hibernate mode except LP
 *   block. This function will shut down LP block and then enter sleep mode.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *    OK(0) for success, or a negated errno for any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_shutdown(FAR struct ad5940_dev_s *priv)
{
  FAR struct ad5940_aferefcfg_s aferefcfg;
  FAR struct ad5940_lploopcfg_s *plploopcfg;
  uint32_t regval;

  plploopcfg = kmm_zalloc(sizeof(struct ad5940_lploopcfg_s));
  if (plploopcfg == NULL)
    {
      return -ENOMEM;
    }

  memset(&aferefcfg, 0, sizeof(struct ad5940_aferefcfg_s));

  /* Turn off LP-loop manually, which is not affected by sleep/hibernate. */

  ad5940_refcfg(priv, &aferefcfg);
  ad5940_lploopcfg(priv, plploopcfg);

  /* Change power mode to sleep mode. */

  ad5940_writereg(priv, AD5940_REG_ALLON_PWRKEY, AD5940_PWRKEY_UNLOCK1);
  ad5940_writereg(priv, AD5940_REG_ALLON_PWRKEY, AD5940_PWRKEY_UNLOCK2);
  ad5940_readreg(priv, AD5940_REG_ALLON_PWRMOD, &regval);
  regval = regval & (~AD5940_BITM_ALLON_PWRMOD_PWRMOD);
  regval = regval | AD5940_ENUM_ALLON_PWRMOD_SLPMOD;
  ad5940_writereg(priv, AD5940_REG_ALLON_PWRMOD, regval);

  kmm_free(plploopcfg);
  return OK;
}

/****************************************************************************
 * Name: ad5940_sleeplock
 *
 * Description:
 *   Enter an incorrect key to lock sleep mode key, or enter correct key to
 *   unlock it. Once key is unlocked, it will always be effect until someone
 *   manually locks it.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   lock - True to lock the key, AD5940 is prohibited to enter sleep mode.
 *          False to unlock the key, so we can enter sleep(hibernate) mode.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_sleeplock(FAR struct ad5940_dev_s *priv, bool lock)
{
  if (lock == true)
    {
      ad5940_writereg(priv, AD5940_REG_AFE_SEQSLPLOCK, AD5940_SLPKEY_LOCK);
    }
  else
    {
      ad5940_writereg(priv, AD5940_REG_AFE_SEQSLPLOCK, AD5940_SLPKEY_UNLOCK);
    }
}

/****************************************************************************
 * Name: ad5940_waitflag
 *
 * Description:
 *   Wait until the specified bit in the register is set or time is out.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - The specified register address.
 *   mask    - The specified bit in the register.
 *   maxtime - The longest time to wait. Waiting for longer than this time
 *             will cause a time-out error.
 *
 * Returned Value:
 *   OK if have waited for the specified bit set.
 *   -ETIME if have waited for longer than the maxtime.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_waitflag(FAR struct ad5940_dev_s *priv,
                           const uint32_t regaddr, const uint32_t mask,
                           const uint32_t maxtime)
{
  uint32_t timecnt = 0;
  uint32_t tempreg;

  ad5940_readreg(priv, regaddr, &tempreg);
  while ((tempreg & mask) == 0)
    {
      up_udelay(AD5940_WAIT_STEP);
      timecnt = timecnt + AD5940_WAIT_STEP;
      if (timecnt > maxtime)
        {
          return -ETIME;
        }

      ad5940_readreg(priv, regaddr, &tempreg);
    }

  return OK;
}

/****************************************************************************
 * Name: ad5940_wakeup
 *
 * Description:
 *   Try to wakeup AD5940 by read register. Any SPI operation can wakeup
 *   AD5940. ad5940_init() must be called before calling this function.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   cnt  - Specify how many times we will read register. Zero or negative
 *          number will bring a failure.
 *
 * Returned Value:
 *   OK(0) for success, or a negated errno for any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_wakeup(FAR struct ad5940_dev_s *priv, int trycnt)
{
  uint32_t tempreg;

  if (trycnt > 0)
    {
      while (trycnt > 0)
        {
          ad5940_readreg(priv, AD5940_REG_AFECON_ADIID, &tempreg);
          trycnt--;
          if (tempreg == AD5940_ADIID)
            {
              return OK;
            }
        }

      snerr("Failed to wakeup AD5940.\n");
      return -EAGAIN;
    }
  else
    {
      snerr("Wake up times must be positive.\n");
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: ad5940_adcbasecfg
 *
 * Description:
 *   Configure  ADC peripheral according to the specified parameters.
 *
 * Input Parameters:
 *   priv      - Device struct.
 *   pagpiocfg - Pointer to configuration structure
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_adcbasecfg(FAR struct ad5940_dev_s *priv,
                             FAR const struct ad5940_adcbasecfg_s *pcfg)
{
  uint32_t tempreg ;

  tempreg = pcfg->adcmuxp;
  tempreg = tempreg |
            ((uint32_t)pcfg->adcmuxn << AD5940_BITP_AFE_ADCCON_MUXSELN);
  tempreg = tempreg |
            ((uint32_t)pcfg->adcpga << AD5940_BITP_AFE_ADCCON_GNPGA);
  return ad5940_writereg(priv, AD5940_REG_AFE_ADCCON, tempreg);
}

/****************************************************************************
 * Name: ad5940_adcfiltercfg
 *
 * Description:
 *   Configure ADC filter according to the specified parameters.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configuration structure.
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_adcfiltercfg(FAR struct ad5940_dev_s *priv,
                               FAR const struct ad5940_adcfiltercfg_s *pcfg)
{
  uint32_t tempreg;
  int ret;

  ad5940_readreg(priv, AD5940_REG_AFE_ADCFILTERCON, &tempreg);

  /* Keep this bit setting. */

  tempreg &= AD5940_BITM_AFE_ADCFILTERCON_AVRGEN;
  tempreg |= pcfg->adcrate;
  if (pcfg->bpnotch == true)
    {
      tempreg |= AD5940_BITM_AFE_ADCFILTERCON_LPFBYPEN;
    }

  if (pcfg->bpsinc3 == true)
    {
      tempreg |= AD5940_BITM_AFE_ADCFILTERCON_SINC3BYP;
    }

  /* Average filter is enabled when DFT source is DFTSRC_AVG in function
   * ad5940_dftcfg.  Once average function is enabled, it's automatically set
   * as DFT source, register DFTCON.DFTINSEL is ignored.
   */

  tempreg = tempreg | ((uint32_t)pcfg->adcsinc2osr <<
                       AD5940_BITP_AFE_ADCFILTERCON_SINC2OSR);
  tempreg = tempreg | ((uint32_t)pcfg->adcsinc3osr <<
                       AD5940_BITP_AFE_ADCFILTERCON_SINC3OSR);
  tempreg = tempreg | ((uint32_t)pcfg->adcavgnum <<
                       AD5940_BITP_AFE_ADCFILTERCON_AVRGNUM);
  ret = ad5940_writereg(priv, AD5940_REG_AFE_ADCFILTERCON, tempreg);
  if (ret < 0)
    {
      return ret;
    }

  /* SINC2+Notch has a block enable/disable bit in AFECON register */

  if (pcfg->sinc2notchenable)
    {
      ret = ad5940_afectrl(priv, AD5940_AFECTRL_SINC2NOTCH, true);
    }

  return ret;
}

/****************************************************************************
 * Name: ad5940_adcmuxcfg
 *
 * Description:
 *   Configure ADC input MUX
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   adcmuxp - ADC channel P configuration.
 *   adcmuxn - ADC channel N configuration.
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_adcmuxcfg(FAR struct ad5940_dev_s *priv,
                            uint32_t adcmuxp, uint32_t adcmuxn)
{
  uint32_t tempreg;
  int ret;

  ret = ad5940_readreg(priv, AD5940_REG_AFE_ADCCON, &tempreg);
  if (ret < 0)
    {
      return ret;
    }

  tempreg = tempreg & (~(AD5940_BITM_AFE_ADCCON_MUXSELN |
                         AD5940_BITM_AFE_ADCCON_MUXSELP));
  tempreg = tempreg | (adcmuxp << AD5940_BITP_AFE_ADCCON_MUXSELP);
  tempreg = tempreg | (adcmuxn << AD5940_BITP_AFE_ADCCON_MUXSELN);
  return ad5940_writereg(priv, AD5940_REG_AFE_ADCCON, tempreg);
}

/****************************************************************************
 * Name: ad5940_afectrl
 *
 * Description:
 *   Control most AFE digital and analog block within one register access.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   ctrlset - A set of blocks that will be controlled select it from
 *             AFECTRL_Const. Below is two examples to use it.
 *   state   - Enable or disable selected control set signal.
 *             false: Disable or power down selected block(s).
 *             true: Enable all selected block(s).
 *
 * Returned Value:
 *    OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_afectrl(FAR struct ad5940_dev_s *priv, uint32_t ctrlset,
                           bool state)
{
  uint32_t tempreg;

  ad5940_readreg(priv, AD5940_REG_AFE_AFECON, &tempreg);
  if (state == true)
    {
      /* Clear bits to enable HPREF and ALDOLimit */

      if (ctrlset & AD5940_AFECTRL_HPREFPWR)
        {
          tempreg = tempreg & (~AD5940_BITM_AFE_AFECON_HPREFDIS);
          ctrlset = ctrlset & (~AD5940_AFECTRL_HPREFPWR);
        }

      if (ctrlset & AD5940_AFECTRL_ALDOLIMIT)
        {
          tempreg = tempreg & (~AD5940_BITM_AFE_AFECON_ALDOILIMITEN);
          ctrlset = ctrlset & (~AD5940_AFECTRL_ALDOLIMIT);
        }

      tempreg = tempreg | ctrlset;
    }
  else
    {
      /* Set bits to disable HPREF and ALDOLimit */

      if (ctrlset & AD5940_AFECTRL_HPREFPWR)
        {
          tempreg = tempreg | AD5940_BITM_AFE_AFECON_HPREFDIS;
          ctrlset = ctrlset & (~AD5940_AFECTRL_HPREFPWR);
        }

      if (ctrlset & AD5940_AFECTRL_ALDOLIMIT)
        {
          tempreg = tempreg | AD5940_BITM_AFE_AFECON_ALDOILIMITEN;
          ctrlset = ctrlset & (~AD5940_AFECTRL_ALDOLIMIT);
        }

      tempreg = tempreg & (~ctrlset);
    }

  return ad5940_writereg(priv, AD5940_REG_AFE_AFECON, tempreg);
}

/****************************************************************************
 * Name: ad5940_clkcfg
 *
 * Description:
 *   Configure AD5940 clock block.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   OK for success, a negated errno for failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_clkcfg(FAR struct ad5940_dev_s *priv)
{
  uint32_t tempreg;
  uint32_t regosccon;
  int ret = 0;

  ad5940_readreg(priv, AD5940_REG_ALLON_OSCCON, &regosccon);

  /* Enable HF OSC if it's not enabled yet. */

  if ((regosccon & AD5940_BITM_ALLON_OSCCON_HFOSCEN) == 0)
    {
      regosccon = regosccon | AD5940_BITM_ALLON_OSCCON_HFOSCEN;
      ad5940_writereg(priv, AD5940_REG_ALLON_OSCKEY, AD5940_KEY_OSCCON);
      ad5940_writereg(priv, AD5940_REG_ALLON_OSCCON, regosccon);
    }

  /* Wait until HFOSC is OK. */

  ret = ad5940_waitflag(priv, AD5940_REG_ALLON_OSCCON,
                        AD5940_BITM_ALLON_OSCCON_HFOSCOK,
                        AD5940_TIMEOUT_HFOSC);
  if (ret < 0)
    {
      snerr("Time was out when enabling AD5940 HFOSC.\n");
      return ret;
    }

  ret = ad5940_hfosc16mhzctrl(priv);
  if (ret < 0)
    {
      snerr("Failed to set HFOSC to 16MHz.\n");
      return ret;
    }

  /* Enable LF OSC if it's not enabled yet. */

  if ((regosccon & AD5940_BITM_ALLON_OSCCON_LFOSCEN) == 0)
    {
      regosccon = regosccon | AD5940_BITM_ALLON_OSCCON_LFOSCEN;
      ad5940_writereg(priv, AD5940_REG_ALLON_OSCKEY, AD5940_KEY_OSCCON);
      ad5940_writereg(priv, AD5940_REG_ALLON_OSCCON, regosccon);
    }

  /* Wait until LFOSC is OK. */

  ret = ad5940_waitflag(priv, AD5940_REG_ALLON_OSCCON,
                        AD5940_BITM_ALLON_OSCCON_LFOSCOK,
                        AD5940_TIMEOUT_LFOSC);
  if (ret < 0)
    {
      snerr("Time was out when enabling AD5940 LFOSC.\n");
      return ret;
    }

  /* Switch clocks step1: set clock divider. */

  tempreg = (AD5940_SYSCLKDIV_1 << AD5940_BITP_AFECON_CLKCON0_SYSCLKDIV) |
            (AD5940_ADCCLKDIV_1 << AD5940_BITP_AFECON_CLKCON0_ADCCLKDIV);
  ad5940_writereg(priv, AD5940_REG_AFECON_CLKCON0, tempreg);
  up_udelay(100);

  /* Switch clocks step2: set clock source. */

  tempreg = AD5940_SYSCLKSRC_HFOSC |
            (AD5940_ADCCLKSRC_HFOSC << AD5940_BITP_AFECON_CLKSEL_ADCCLKSEL);
  ad5940_writereg(priv, AD5940_REG_AFECON_CLKSEL, tempreg);

  /* Disable HF XTAL. */

  regosccon = regosccon & (~AD5940_BITM_ALLON_OSCCON_HFXTALEN);
  ad5940_writereg(priv, AD5940_REG_ALLON_OSCKEY, AD5940_KEY_OSCCON);
  ad5940_writereg(priv, AD5940_REG_ALLON_OSCCON, regosccon);

  return OK;
}

/****************************************************************************
 * Name: ad5940_dftcfg
 *
 * Description:
 *   Configure AD5940 DFT number and source and hanning window.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configure structure.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_dftcfg(FAR struct ad5940_dev_s *priv,
                          FAR const struct ad5940_dftcfg_s *pcfg)
{
  uint32_t reg_dftcon = 0;
  uint32_t reg_adcfilter;

  /* Deal with DFTSRC_AVG. Once average function is enabled, it's
   * automatically set as DFT source.
   */

  ad5940_readreg(priv, AD5940_REG_AFE_ADCFILTERCON, &reg_adcfilter);
  if (pcfg->dftsrc == AD5940_DFTSRC_AVG)
    {
      reg_adcfilter = reg_adcfilter | AD5940_BITM_AFE_ADCFILTERCON_AVRGEN;
      ad5940_writereg(priv, AD5940_REG_AFE_ADCFILTERCON, reg_adcfilter);
    }
  else
    {
      /* Disable Average function and set correct DFT source. */

      reg_adcfilter = reg_adcfilter & (~AD5940_BITM_AFE_ADCFILTERCON_AVRGEN);
      ad5940_writereg(priv, AD5940_REG_AFE_ADCFILTERCON, reg_adcfilter);

      /* Set new DFT source. */

      reg_dftcon = reg_dftcon | (pcfg->dftsrc <<
                                 AD5940_BITP_AFE_DFTCON_DFTINSEL);
    }

  /* Set DFT number. */

  reg_dftcon = reg_dftcon | (pcfg->dftnum << AD5940_BITP_AFE_DFTCON_DFTNUM);
  if (pcfg->hanwinen == true)
    {
      reg_dftcon |= AD5940_BITM_AFE_DFTCON_HANNINGEN;
    }

  ad5940_writereg(priv, AD5940_REG_AFE_DFTCON, reg_dftcon);
}

/****************************************************************************
 * Name: ad5940_dspcfg
 *
 * Description:
 *   Control AD5940 DSP block.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configure structure.
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_dspcfg(FAR struct ad5940_dev_s *priv,
                         FAR const struct ad5940_dspcfg_s *pcfg)
{
  int ret;

  ret = ad5940_adcbasecfg(priv, &pcfg->adcbasecfg);
  if (ret < 0)
    {
      return ret;
    }

  ret = ad5940_adcfiltercfg(priv, &pcfg->adcfiltercfg);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure AD5940 ADC digital comparator function. */

  ad5940_writereg(priv, AD5940_REG_AFE_ADCMIN, pcfg->adcdigcompcfg.adcmin);
  ad5940_writereg(priv, AD5940_REG_AFE_ADCMINSM,
                  pcfg->adcdigcompcfg.adcminhys);
  ad5940_writereg(priv, AD5940_REG_AFE_ADCMAX, pcfg->adcdigcompcfg.adcmax);
  ad5940_writereg(priv, AD5940_REG_AFE_ADCMAXSMEN,
                  pcfg->adcdigcompcfg.adcmaxhys);

  /* Configure AD5940 DFT number and source and hanning window. */

  ad5940_dftcfg(priv, &pcfg->dftcfg);

  /* Configure AD5940 statistic functions. */

  ad5940_statisticcfg(priv, &pcfg->statcfg);

  return OK;
}

/****************************************************************************
 * Name: ad5940_fifocfg
 *
 * Description:
 *   Configure AD5940 FIFO.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   pclkcfg - A pointer to configuration strcuture
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_fifocfg(FAR struct ad5940_dev_s *priv,
                           FAR const struct ad5940_fifocfg_s *pfifocfg)
{
  uint32_t tempreg;

  /* Disable FIFO firstly */

  ad5940_writereg(priv, AD5940_REG_AFE_FIFOCON, 0);

  /* CMDDATACON register. Configure this firstly */

  ad5940_readreg(priv, AD5940_REG_AFE_CMDDATACON, &tempreg);
  tempreg = tempreg & (AD5940_BITM_AFE_CMDDATACON_CMD_MEM_SEL |
                       AD5940_BITM_AFE_CMDDATACON_CMDMEMMDE);
  tempreg = tempreg |
            (pfifocfg->fifomode << AD5940_BITP_AFE_CMDDATACON_DATAMEMMDE);
  tempreg = tempreg |
            (pfifocfg->fifosize << AD5940_BITP_AFE_CMDDATACON_DATA_MEM_SEL);
  ad5940_writereg(priv, AD5940_REG_AFE_CMDDATACON, tempreg);

  /* FIFO Threshold */

  ad5940_writereg(priv, AD5940_REG_AFE_DATAFIFOTHRES,
                  pfifocfg->fifothreshd <<
                  AD5940_BITP_AFE_DATAFIFOTHRES_HIGHTHRES);

  /* FIFOCON register. Final step is to enable FIFO */

  ad5940_fifoctrl(priv, pfifocfg->fifosrc, pfifocfg->fifoen);
}

/****************************************************************************
 * Name: ad5940_fifoctrl
 *
 * Description:
 *   Configure AD5940 FIFO source and enable or disable FIFO.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   fifosrc - available choices are in FIFOSRC_Const
 *             FIFOSRC_SINC3: SINC3 data
 *             FIFOSRC_DFT: DFT real and imaginary part
 *             FIFOSRC_SINC2NOTCH: SINC2+NOTCH block. Notch can be bypassed,
 *                                 so SINC2 data can be feed to FIFO
 *             FIFOSRC_VAR: Statistic variance output
 *             FIFOSRC_MEAN: Statistic mean output
 *   fifoen  - Enable or disable the FIFO.
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_fifoctrl(FAR struct ad5940_dev_s *priv, uint32_t fifosrc,
                            bool fifoen)
{
  uint32_t tempreg = 0;

  if (fifoen == true)
    {
      tempreg = tempreg | AD5940_BITM_AFE_FIFOCON_DATAFIFOEN;
    }

  tempreg = tempreg | (fifosrc << AD5940_BITP_AFE_FIFOCON_DATAFIFOSRCSEL);
  ad5940_writereg(priv, AD5940_REG_AFE_FIFOCON, tempreg);
}

/****************************************************************************
 * Name: ad5940_fifogetcnt
 *
 * Description:
 *   Get data count in FIFO
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   Data count in FIFO.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static uint32_t ad5940_fifogetcnt(FAR struct ad5940_dev_s *priv)
{
  uint32_t regval;

  ad5940_readreg(priv, AD5940_REG_AFE_FIFOCNTSTA, &regval);
  regval = regval >> AD5940_BITP_AFE_FIFOCNTSTA_DATAFIFOCNT;

  return regval;
}

/****************************************************************************
 * Name: ad5940_hfosc16mhzctrl
 *
 * Description:
 *   Configure internal HFOSC to output 16MHz.
 *
 * Input Parameters:
 *   priv      - Device struct.
 *
 * Returned Value:
 *   OK for success, a negated errno for failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_hfosc16mhzctrl(FAR struct ad5940_dev_s *priv)
{
  uint32_t readclken1;
  uint32_t readosccon;
  uint32_t bit8;
  uint32_t bit9;
  int ret = 0;

  ad5940_readreg(priv, AD5940_REG_AFECON_CLKEN1, &readclken1);

  /* Fix bug in silicon, bit8 and bit9 is swapped when read back. */

  bit8 = (readclken1 >> 9) & 0x01;
  bit9 = (readclken1 >> 8) & 0x01;
  readclken1 = readclken1 & 0xff;
  readclken1 |= (bit8 << 8) | (bit9 << 9);

  /* Disable ACLK during clock changing */

  ad5940_writereg(priv, AD5940_REG_AFECON_CLKEN1,
                  readclken1 | AD5940_BITM_AFECON_CLKEN1_ACLKDIS);

  /* Enable 16MHz output(bit definition-0: 32MHz, 1: 16MHz) */

  ad5940_readreg(priv, AD5940_REG_AFE_HPOSCCON, &readosccon);
  ad5940_writereg(priv, AD5940_REG_AFE_HPOSCCON,
                  readosccon | AD5940_BITM_AFE_HPOSCCON_CLK32MHZEN);
  ret = ad5940_waitflag(priv, AD5940_REG_ALLON_OSCCON,
                        AD5940_BITM_ALLON_OSCCON_HFOSCOK,
                        AD5940_TIMEOUT_HFOSC);
  if (ret < 0)
    {
      snerr("Time was out when enabling AD5940 16MHz.\n");
      return ret;
    }

  /* Enable ACLK again */

  ad5940_writereg(priv, AD5940_REG_AFECON_CLKEN1,
                  readclken1 & (~AD5940_BITM_AFECON_CLKEN1_ACLKDIS));
  return ret;
}

/****************************************************************************
 * Name: ad5940_hsdaccfg
 *
 * Description:
 *   Configure AD5940 HSDAC
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configure structure.
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_hsdaccfg(FAR struct ad5940_dev_s *priv,
                            FAR const struct ad5940_hsdaccfg_s *pcfg)
{
  uint32_t tempreg = 0;

  /* Enable attenuator */

  if (pcfg->excitbufgain == AD5940_EXCITBUFGAIN_0P25)
    {
      tempreg |= AD5940_BITM_AFE_HSDACCON_INAMPGNMDE;
    }

  if (pcfg->hsdacgain == AD5940_HSDACGAIN_0P2)
    {
      tempreg |= AD5940_BITM_AFE_HSDACCON_ATTENEN;
    }

  tempreg = tempreg | ((pcfg->hsdacupdaterate & 0xff) <<
                       AD5940_BITP_AFE_HSDACCON_RATE);
  ad5940_writereg(priv, AD5940_REG_AFE_HSDACCON, tempreg);
}

/****************************************************************************
 * Name: ad5940_hsloopcfg
 *
 * Description:
 *   Configure high speed loop (high bandwidth loop or called excitation
 *   loop). This configuration includes HSDAC, HSTIA and switch matrix.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configure structure.
 *
 * Returned Value:
 *   OK for success, a negated errno for failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_hsloopcfg(FAR struct ad5940_dev_s *priv,
                            FAR const struct ad5940_hsloopcfg_s *pcfg)
{
  int ret;

  ad5940_hsdaccfg(priv, &pcfg->hsdaccfg);
  ret = ad5940_hstiacfg(priv, &pcfg->hstiacfg);
  if (ret < 0)
    {
      return ret;
    }

  ad5940_swmatrixcfg(priv, &pcfg->swmatcfg);
  ad5940_wgcfg(priv, &pcfg->wgcfg);

  return ret;
}

/****************************************************************************
 * Name: ad5940_hsrtiacal
 *
 * Description:
 *   Measure HSTIA internal RTIA impedance for calibration.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   pcfg    - Pointer to calibration structure.
 *   presult - Pointer to a variable that used to store result.
 *             If bPolarResult in structure is set, then use type
 *             ad5940_fimpol_s, otherwise use ad5940_fimcar_s
 *
 * Returned Value:
 *   OK for success, a negated errno for failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_hsrtiacal(FAR struct ad5940_dev_s *priv,
                            FAR const struct ad5940_hsrtiacal_s *pcfg,
                            FAR void *presult)
{
  FAR struct ad5940_hsloopcfg_s *phsloop;
  FAR struct ad5940_dspcfg_s *pdspcfg;
  FAR struct ad5940_aferefcfg_s aferefcfg;
  FAR struct ad5940_iimpcar_s dftrcal;
  FAR struct ad5940_iimpcar_s dftrtia;
  FAR struct ad5940_fimpcar_s fimptemp;
  uint32_t intccfg;
  uint32_t excitbuffgain;
  uint32_t hsdacgain;
  uint32_t rtiaval;
  uint32_t wgampword;
  float excitvolt;
  bool adcclk32mhzmode;
  int ret;

  phsloop = kmm_zalloc(sizeof(struct ad5940_hsloopcfg_s));
  if (phsloop == NULL)
    {
      return -ENOMEM;
    }

  pdspcfg = kmm_zalloc(sizeof(struct ad5940_dspcfg_s));
  if (pdspcfg == NULL)
    {
      kmm_free(phsloop);
      return -ENOMEM;
    }

  memset(&aferefcfg, 0, sizeof(struct ad5940_aferefcfg_s));

  if (presult == NULL || pcfg == NULL || pcfg->frcal == 0 ||
      pcfg->hstiacfg.hstiartiasel > AD5940_HSTIARTIA_160K ||
      pcfg->hstiacfg.hstiartiasel == AD5940_HSTIARTIA_OPEN)
    {
      snerr("Invalid arguments were found.\n");
      ret = -EINVAL;
      goto end;
    }

  adcclk32mhzmode = (pcfg->adcclkfreq > (32000000 * 0.8));

  /* Calculate the excitation voltage we should use based on RCAL/Rtia. */

  rtiaval = ad5940_hprtiatable[pcfg->hstiacfg.hstiartiasel];

  /* DAC output voltage calculation. RCAL value should be similar to RTIA so
   * the accuracy is best. HSTIA output voltage should be limited to 0.2V to
   * AVDD-0.2V, with 1.1V bias. We use 80% of this range for safe. Because
   * the bias voltage is fixed to 1.1V, so for AC signal maximum amplitude is
   * 1.1V-0.2V = 0.9Vp. That's 1.8Vpp.
   * Formula is:    ExcitVolt(in mVpp) = (1800mVpp*80% / RTIA) * RCAL
   * ADC input range is +-1.5V which is enough for calibration.
   */

  excitvolt = 1800 * 0.8 * pcfg->frcal / rtiaval;

  /* Voltage is so small that we can enable the attenuator of DAC(1/5) and
   * excitation buffer(1/4). 800mVpp is the DAC output voltage
   */

  if (excitvolt <= 800 * 0.05)
    {
      excitbuffgain = AD5940_EXCITBUFGAIN_0P25;
      hsdacgain = AD5940_HSDACGAIN_0P2;

      /* Excitation buffer voltage full range is 800mVpp*0.05 = 40mVpp.
       * Assign value with rounding (0.5 LSB error).
       */

      wgampword = ((uint32_t)(excitvolt / 40 * 2047 * 2) + 1) >> 1;
    }
  else if (excitvolt <= 800 * 0.25)
    {
      /* Enable Excitation buffer attenuator */

      excitbuffgain = AD5940_EXCITBUFGAIN_0P25;
      hsdacgain = AD5940_HSDACGAIN_1;

      /* Excitation buffer voltage full range is 800mVpp*0.25 = 200mVpp. */

      wgampword = ((uint32_t)(excitvolt / 200 * 2047 * 2) + 1) >> 1;
    }
  else if (excitvolt <= 800 * 0.4)
    {
      /* Enable DAC attenuator */

      excitbuffgain = AD5940_EXCITBUFGAIN_2;
      hsdacgain = AD5940_HSDACGAIN_0P2;

      /* Excitation buffer voltage full range is 800mVpp*0.4 = 320mV. */

      wgampword = ((uint32_t)(excitvolt / 320 * 2047 * 2) + 1) >> 1;
    }
  else
    {
      /* No attenuator is needed. This is the best condition which means RTIA
       * is close to RCAL.
       */

      excitbuffgain = AD5940_EXCITBUFGAIN_2;
      hsdacgain = AD5940_HSDACGAIN_1;

      /* Excitation buffer voltage full range is 800mVpp*2=1600mVpp. */

      wgampword = ((uint32_t)(excitvolt / 1600 * 2047 * 2) + 1) >> 1;
    }

  if (wgampword > 0x7ff)
    {
      wgampword = 0x7ff;
    }

  /* Interupt controller configuration. */

  ad5940_readreg(priv, AD5940_REG_INTC_INTCSEL1, &intccfg);

  /* Enable DFT ready Interrupt in INTC1. */

  ad5940_intrptcfg(priv, AD5940_AFEINTC_1, AD5940_AFEINTSRC_DFTRDY, true);

  /* Init all to disable state. */

  ad5940_afectrl(priv, AD5940_AFECTRL_ALL, false);

  /* Configure reference system. */

  aferefcfg.hpbandgapen = true;
  aferefcfg.hp1v1buffen = true;
  aferefcfg.hp1v8buffen = true;
  aferefcfg.disc1v1cap = false;
  aferefcfg.disc1v8cap = false;
  aferefcfg.hp1v8thembuff = false;
  aferefcfg.hp1v8ilimit = false;
  aferefcfg.lp1v1buffen = false;
  aferefcfg.lp1v8buffen = false;
  aferefcfg.lpbandgapen = false;
  aferefcfg.lprefbufen = false;
  aferefcfg.lprefboosten = false;
  ad5940_refcfg(priv, &aferefcfg);

  /* Configure HP Loop. */

  phsloop->hsdaccfg.excitbufgain = excitbuffgain;
  phsloop->hsdaccfg.hsdacgain = hsdacgain;
  phsloop->hsdaccfg.hsdacupdaterate = 7;
  memcpy(&phsloop->hstiacfg, &pcfg->hstiacfg, sizeof(pcfg->hstiacfg));
  phsloop->swmatcfg.dswitch = AD5940_SWD_RCAL0;
  phsloop->swmatcfg.pswitch = AD5940_SWP_RCAL0;
  phsloop->swmatcfg.nswitch = AD5940_SWN_RCAL1;
  phsloop->swmatcfg.tswitch = AD5940_SWT_RCAL1 | AD5940_SWT_TRTIA;
  phsloop->wgcfg.wgtype = AD5940_WGTYPE_SIN;
  phsloop->wgcfg.gaincalen = true;
  phsloop->wgcfg.offsetcalen = true;
  phsloop->wgcfg.sincfg.sinfreqword =
                   ad5940_wgfreqwordcal(pcfg->ffreq, pcfg->sysclkfreq);
  phsloop->wgcfg.sincfg.sinamplitudeword = wgampword;
  phsloop->wgcfg.sincfg.sinoffsetword = 0;
  phsloop->wgcfg.sincfg.sinphaseword = 0;
  ret = ad5940_hsloopcfg(priv, phsloop);
  if (ret < 0)
    {
      snerr("Failed to configure HS loop.\n");
      goto end;
    }

  /* Configure DSP. */

  pdspcfg->adcbasecfg.adcmuxn = AD5940_ADCMUXN_N_NODE;
  pdspcfg->adcbasecfg.adcmuxp = AD5940_ADCMUXP_P_NODE;
  pdspcfg->adcbasecfg.adcpga = AD5940_ADCPGA_1P5;
  pdspcfg->adcfiltercfg.adcavgnum = AD5940_ADCAVGNUM_16;
  pdspcfg->adcfiltercfg.adcrate =
              adcclk32mhzmode? AD5940_ADCRATE_1P6MHZ: AD5940_ADCRATE_800KHZ;
  pdspcfg->adcfiltercfg.adcsinc2osr = pcfg->adcsinc2osr;
  pdspcfg->adcfiltercfg.adcsinc3osr = pcfg->adcsinc3osr;
  pdspcfg->adcfiltercfg.bpnotch = true;
  pdspcfg->adcfiltercfg.bpsinc3 = false;
  pdspcfg->adcfiltercfg.sinc2notchenable = true;
  memcpy(&pdspcfg->dftcfg, &pcfg->dftcfg, sizeof(pcfg->dftcfg));
  ret = ad5940_dspcfg(priv, pdspcfg);
  if (ret < 0)
    {
      snerr("Failed to configure DSP.\n");
      goto end;
    }

  /* Enable all of them. They are automatically turned off during hibernate
   * mode to save power.
   */

  ad5940_afectrl(priv, AD5940_AFECTRL_HSTIAPWR | AD5940_AFECTRL_INAMPPWR |
                       AD5940_AFECTRL_EXTBUFPWR | AD5940_AFECTRL_DACREFPWR |
                       AD5940_AFECTRL_HSDACPWR | AD5940_AFECTRL_SINC2NOTCH,
                       true);

  /* Enable Waveform generator, ADC power. */

  ad5940_afectrl(priv, AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR, true);

  /* Wait for sometime. */

  up_udelay(250);

  /* Start ADC convert and DFT. */

  ad5940_afectrl(priv, AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT, true);

  /* Wait until DFT is ready. */

  ret = ad5940_waitflag(priv, AD5940_REG_INTC_INTCFLAG1,
                        AD5940_AFEINTSRC_DFTRDY, AD5940_TIMEOUT_DFTRDY);
  if (ret < 0)
    {
      snerr("Time was out when waiting for DFT ready.\n");
      goto end;
    }

  /* Stop ADC convert and DFT. */

  ad5940_afectrl(priv, AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT |
                       AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR,
                       false);
  ad5940_intrptclr(priv, AD5940_AFEINTSRC_DFTRDY);

  /* Read DFT results. */

  ad5940_readreg(priv, AD5940_REG_AFE_DFTREAL,
                 (FAR uint32_t *)(&dftrcal.real));
  ad5940_readreg(priv, AD5940_AFERESULT_DFTIMAGE,
                 (FAR uint32_t *)(&dftrcal.image));

  /* Configure ADC MUX. */

  ret = ad5940_adcmuxcfg(priv, AD5940_ADCMUXP_HSTIA_P,
                         AD5940_ADCMUXN_HSTIA_N);
  if (ret < 0)
    {
      snerr("Failed to configure ADC MUX.\n");
      goto end;
    }

  /* Enable Waveform generator, ADC power */

  ad5940_afectrl(priv, AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR, true);

  /* Wait for sometime. */

  up_udelay(250);

  /* Start ADC convert and DFT */

  ad5940_afectrl(priv, AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT, true);

  /* Wait until DFT is ready. */

  ret = ad5940_waitflag(priv, AD5940_REG_INTC_INTCFLAG1,
                        AD5940_AFEINTSRC_DFTRDY, AD5940_TIMEOUT_DFTRDY);
  if (ret < 0)
    {
      snerr("Time was out when waiting for DFT ready.\n");
      goto end;
    }

  /* Stop ADC convert and DFT. */

  ad5940_afectrl(priv, AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT |
                       AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR, false);
  ad5940_intrptclr(priv, AD5940_AFEINTSRC_DFTRDY);

  /* Read DFT results. */

  ad5940_readreg(priv, AD5940_REG_AFE_DFTREAL,
                 (FAR uint32_t *)(&dftrtia.real));
  ad5940_readreg(priv, AD5940_AFERESULT_DFTIMAGE,
                 (FAR uint32_t *)(&dftrtia.image));

  /* DFT result has 18bits. If bit17 is 1, the result is a negative. */

  if (dftrcal.real & (1L << 17))
    {
      dftrcal.real |= 0xfffc0000;
    }

  if (dftrcal.image & (1L << 17))
    {
      dftrcal.image |= 0xfffc0000;
    }

  if (dftrtia.real & (1L << 17))
    {
      dftrtia.real |= 0xfffc0000;
    }

  if (dftrtia.image & (1l << 17))
    {
      dftrtia.image |= 0xfffc0000;
    }

  /* ADC MUX is set to HSTIA_P and HSTIA_N. While the current flow through
   * RCAL and then into RTIA, the current direction should be from HSTIA_N to
   * HSTIA_P if we measure the voltage across RCAL by MUXSELP_P_NODE and
   * MUXSELN_N_NODE. So we should add negative signs to RTIA DFT results.
   * The impedance engine inside of AD594x give us real part and imaginary
   * part of DFT. Due to technology used, the imaginary part in register is
   * the opposite number. So we add a negative sign on the imaginary parts.
   * Thus, the real part of RTIA DFT result and the imaginary part of RCAL
   * DFT result should be added a negitive sign.
   */

  dftrtia.real = -dftrtia.real;
  dftrcal.image = -dftrcal.image;

  /* Calculate the impedance of RTIA. */

  fimptemp = ad5940_complexdivint(&dftrtia, &dftrcal);
  fimptemp.real = fimptemp.real * pcfg->frcal;
  fimptemp.image = fimptemp.image * pcfg->frcal;
  if (pcfg->bpolarresult == false)
    {
      *(FAR struct ad5940_fimpcar_s *)presult = fimptemp;
    }
  else
    {
      ((FAR struct ad5940_fimppol_s *)presult)->magnitude =
                                              ad5940_complexmag(&fimptemp);
      ((FAR struct ad5940_fimppol_s *)presult)->phase =
                                              ad5940_complexphase(&fimptemp);
    }

  /* Restore INTC1 DFT configure. */

  if (!(intccfg & AD5940_AFEINTSRC_DFTRDY))
    {
      ad5940_intrptcfg(priv, AD5940_AFEINTC_1, AD5940_AFEINTSRC_DFTRDY,
                       false);
    }

end:
  kmm_free(phsloop);
  kmm_free(pdspcfg);
  return ret;
}

/****************************************************************************
 * Name: ad5940_hsrtiacfg
 *
 * Description:
 *   Configure AD5940 high speed TIA amplifier
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configure structure.
 *
 * Returned Value:
 *   OK for success, a negated errno for failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_hstiacfg(FAR struct ad5940_dev_s *priv,
                           FAR const struct ad5940_hstiacfg_s *pcfg)
{
  uint32_t tempreg = 0;

  /* Available hstiadertia is 1k, 5k,...,160k, short, OPEN.
   * Available hstiaderload is OPEN, 0R,..., 100R.
   */

  if (pcfg == NULL || pcfg->hstiadertia < AD5940_HSTIADERTIA_1K ||
      pcfg->hstiadertia > AD5940_HSTIADERTIA_OPEN ||
      pcfg->hstiaderload > AD5940_HSTIADERLOAD_OPEN)
    {
      return -EINVAL;
    }

  tempreg = tempreg | pcfg->hstiabias;
  ad5940_writereg(priv, AD5940_REG_AFE_HSTIACON, tempreg);

  /* Calculate CTIA value */

  tempreg = pcfg->hstiactia << AD5940_BITP_AFE_HSRTIACON_CTIACON;
  tempreg = tempreg | pcfg->hstiartiasel;
  if (pcfg->diodeclose == true)
    {
      /* Close switch 6 */

      tempreg |= AD5940_BITM_AFE_HSRTIACON_TIASW6CON;
    }

  ad5940_writereg(priv, AD5940_REG_AFE_HSRTIACON, tempreg);

  /* Set DExRESCON */

  ad5940_setdexrtia(priv, AD5940_HSTIADE0PIN, pcfg->hstiadertia,
                    pcfg->hstiaderload);

  return OK;
}

/****************************************************************************
 * Name: ad5940_intrptcfg
 *
 * Description:
 *   Enable or disable selected interrupt source(s)
 *
 * Input Parameters:
 *   priv       - Device struct.
 *   intctrlsel - {AD5940_AFEINTC_0,AD5940_AFEINTC_1}. Configure interrupt
 *                controller 0 or 1
 *   intsrc     - Select a value from AD5940_AFEINTSRC_xxx definations.
 *   state      - Enable(true) or disable(false) these interrupt source(s)
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_intrptcfg(FAR struct ad5940_dev_s *priv,
                             uint32_t intctrlsel, uint32_t intsrc,
                             bool state)
{
  uint32_t tempreg;
  uint16_t regaddr = AD5940_REG_INTC_INTCSEL0;

  if (intctrlsel == AD5940_AFEINTC_1)
    {
      regaddr = AD5940_REG_INTC_INTCSEL1;
    }

  ad5940_readreg(priv, regaddr, &tempreg);
  if (state == true)
    {
      tempreg |= intsrc;
    }
  else
    {
      tempreg &= ~(intsrc);
    }

  ad5940_writereg(priv, regaddr, tempreg);
}

/****************************************************************************
 * Name: ad5940_intrptclr
 *
 * Description:
 *   Clear selected interrupt(s) flag. INTC0Flag and INTC1Flag are both
 *   cleared.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   intsrc - Select a value from AD5940_AFEINTSRC_xxx definations.
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_intrptclr(FAR struct ad5940_dev_s *priv, uint32_t intsrc)
{
  ad5940_writereg(priv, AD5940_REG_INTC_INTCCLR, intsrc);
}

/****************************************************************************
 * Name: ad5940_lpampcfg
 *
 * Description:
 *   Configure AD5940 LP TIA and PA.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configuration structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_lpampcfg(FAR struct ad5940_dev_s *priv,
                            FAR const struct ad5940_lpampcfg_s *pcfg)
{
  uint32_t tempreg;

  tempreg = 0;
  if (pcfg->lppapwren == false)
    {
      tempreg |= AD5940_BITM_AFE_LPTIACONX_PAPDEN;
    }

  if (pcfg->lptiapwren == false)
    {
      tempreg |= AD5940_BITM_AFE_LPTIACONX_TIAPDEN;
    }

  if (pcfg->lpamppwrmod == AD5940_LPAMPPWR_HALF)
    {
      tempreg |= AD5940_BITM_AFE_LPTIACONX_HALFPWR;
    }
  else
    {
      tempreg = tempreg | (pcfg->lpamppwrmod <<
                           AD5940_BITP_AFE_LPTIACONX_IBOOST);
    }

  tempreg = tempreg | (pcfg->lptiartia << AD5940_BITP_AFE_LPTIACONX_TIAGAIN);
  tempreg = tempreg | (pcfg->lptiarload << AD5940_BITP_AFE_LPTIACONX_TIARL);
  tempreg = tempreg | (pcfg->lptiarf << AD5940_BITP_AFE_LPTIACONX_TIARF);

  /* For AD5940, only LPAMP0 is available */

  ad5940_writereg(priv, AD5940_REG_AFE_LPTIACON0, tempreg);
  ad5940_writereg(priv, AD5940_REG_AFE_LPTIASW0, pcfg->lptiasw);
}

/****************************************************************************
 * Name: ad5940_lpdac0write
 *
 * Description:
 *   Write LPDAC0 data to AD5940.
 *
 * Input Parameters:
 *   priv      - Device struct.
 *   data12bit - 12bit DAC data
 *   data6bit  - 6bit DAC data
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_lpdac0write(FAR struct ad5940_dev_s *priv,
                                uint16_t data12bit, uint8_t data6bit)
{
  data6bit &= 0x3f;
  data12bit &= 0x0fff;
  ad5940_writereg(priv, AD5940_REG_AFE_LPDACDAT0,
                  ((uint32_t)data6bit << 12) | (uint32_t)data12bit);
}

/****************************************************************************
 * Name: ad5940_lpdaccfg
 *
 * Description:
 *   Configure AD5940 LPDAC.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configuration structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_lpdaccfg(FAR struct ad5940_dev_s *priv,
                            FAR const struct ad5940_lpdaccfg_s *pcfg)
{
  uint32_t tempreg;

  tempreg = 0;
  tempreg = pcfg->lpdacsrc << AD5940_BITP_AFE_LPDACCONX_WAVETYPE;
  tempreg = tempreg | (pcfg->vzeromux << AD5940_BITP_AFE_LPDACCONX_VZEROMUX);
  tempreg = tempreg | (pcfg->vbiasmux << AD5940_BITP_AFE_LPDACCONX_VBIASMUX);
  tempreg = tempreg | (pcfg->lpdacref << AD5940_BITP_AFE_LPDACCONX_REFSEL);
  if (pcfg->datarst == false)
    {
      tempreg |= AD5940_BITM_AFE_LPDACCONX_RSTEN;
    }

  if (pcfg->poweren == false)
    {
      tempreg |= AD5940_BITM_AFE_LPDACCONX_PWDEN;
    }

  /* For AD5940, only LPDAC0 is available */

  ad5940_writereg(priv, AD5940_REG_AFE_LPDACCON0, tempreg);
  ad5940_lpdac0write(priv, pcfg->dacdata12bit, pcfg->dacdata6bit);
  ad5940_writereg(priv, AD5940_REG_AFE_LPDACSW0,
                  pcfg->lpdacsw | AD5940_BITM_AFE_LPDACSWX_LPMODEDIS);
}

/****************************************************************************
 * Name: ad5940_lploopcfg
 *
 * Description:
 *   Configure low power loop include LPDAC LPAmp(PA and TIA).
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configuration structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_lploopcfg(FAR struct ad5940_dev_s *priv,
                             FAR const struct ad5940_lploopcfg_s *pcfg)
{
  ad5940_lpdaccfg(priv, &pcfg->lpdaccfg);
  ad5940_lpampcfg(priv, &pcfg->lpampcfg);
}

/****************************************************************************
 * Name: ad5940_refcfg
 *
 * Description:
 *   Configure AD5940 reference buffer include 1.8V/1.1V high/low power
 *   buffers.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to buffer configure structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_refcfg(FAR struct ad5940_dev_s *priv,
                          FAR const struct ad5940_aferefcfg_s *pcfg)
{
  uint32_t tempreg;

  /* HP Reference(bandgap) */

  ad5940_readreg(priv, AD5940_REG_AFE_AFECON, &tempreg);
  tempreg &= ~AD5940_BITM_AFE_AFECON_HPREFDIS;
  if (pcfg->hpbandgapen == false)
    {
      tempreg |= AD5940_BITM_AFE_AFECON_HPREFDIS;
    }

  ad5940_writereg(priv, AD5940_REG_AFE_AFECON, tempreg);

  /* Reference buffer configure */

  ad5940_readreg(priv, AD5940_REG_AFE_BUFCON, &tempreg);
  if (pcfg->hp1v8buffen == true)
    {
      tempreg |= AD5940_BITM_AFE_BUFCON_V1P8HPADCEN;
    }

  if (pcfg->hp1v1buffen == true)
    {
      tempreg |= AD5940_BITM_AFE_BUFCON_V1P1HPADCEN;
    }

  if (pcfg->lp1v8buffen == true)
    {
      tempreg |= AD5940_BITM_AFE_BUFCON_V1P8LPADCEN;
    }

  if (pcfg->lp1v1buffen == true)
    {
      tempreg |= AD5940_BITM_AFE_BUFCON_V1P1LPADCEN;
    }

  if (pcfg->hp1v8thembuff == true)
    {
      tempreg |= AD5940_BITM_AFE_BUFCON_V1P8THERMSTEN;
    }

  if (pcfg->hp1v8ilimit == true)
    {
      tempreg |= AD5940_BITM_AFE_BUFCON_V1P8HPADCILIMITEN;
    }

  if (pcfg->disc1v8cap == true)
    {
      tempreg |= AD5940_BITM_AFE_BUFCON_V1P8HPADCCHGDIS;
    }

  if (pcfg->disc1v1cap == true)
    {
      tempreg |= AD5940_BITM_AFE_BUFCON_V1P1LPADCCHGDIS;
    }

  ad5940_writereg(priv, AD5940_REG_AFE_BUFCON, tempreg);

  /* LPREFBUFCON */

  tempreg = 0;
  if (pcfg->lprefbufen == false)
    {
      tempreg |= AD5940_BITM_AFE_LPREFBUFCON_LPBUF2P5DIS;
    }

  if (pcfg->lpbandgapen == false)
    {
      tempreg |= AD5940_BITM_AFE_LPREFBUFCON_LPREFDIS;
    }

  if (pcfg->lprefboosten == true)
    {
      tempreg |= AD5940_BITM_AFE_LPREFBUFCON_BOOSTCURRENT;
    }

  ad5940_writereg(priv, AD5940_REG_AFE_LPREFBUFCON, tempreg);
}

/****************************************************************************
 * Name: ad5940_seqcfg
 *
 * Description:
 *   Configure AD5940 sequencer.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configuration structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_seqcfg(FAR struct ad5940_dev_s *priv,
                          FAR const struct ad5940_seqcfg_s *pcfg)
{
  uint32_t tempreg;
  uint32_t fifocon;

  /* Record FIFOCON, then disable FIFO before changing memory configuration */

  ad5940_readreg(priv, AD5940_REG_AFE_FIFOCON, &fifocon);
  ad5940_writereg(priv, AD5940_REG_AFE_FIFOCON, 0);

  /* Configure CMDDATACON register */

  ad5940_readreg(priv, AD5940_REG_AFE_CMDDATACON, &tempreg);

  /* Clear settings for sequencer memory */

  tempreg = tempreg & ~(AD5940_BITM_AFE_CMDDATACON_CMDMEMMDE |
                        AD5940_BITM_AFE_CMDDATACON_CMD_MEM_SEL);

  /* Sequencer is always in memory mode */

  tempreg = tempreg | AD5940_ENUM_AFE_CMDDATACON_CMEM;
  tempreg = tempreg | (pcfg->seqmemsize <<
                       AD5940_BITP_AFE_CMDDATACON_CMD_MEM_SEL);
  ad5940_writereg(priv, AD5940_REG_AFE_CMDDATACON, tempreg);

  /* If count to be cleared, disable sequencer firstly. When sequencer is
   * disabled, any write to SEQCNT will clear CNT and CRC register
   */

  if (pcfg->seqcntcrcclr == true)
    {
      ad5940_writereg(priv, AD5940_REG_AFE_SEQCON, 0);
      ad5940_writereg(priv, AD5940_REG_AFE_SEQCNT, 0);
    }

  tempreg = 0;
  if (pcfg->seqenable == true)
    {
      tempreg |= AD5940_BITM_AFE_SEQCON_SEQEN;
    }

  tempreg = tempreg | (pcfg->seqwrtimer << AD5940_BITP_AFE_SEQCON_SEQWRTMR);
  ad5940_writereg(priv, AD5940_REG_AFE_SEQCON, tempreg);

  /* Restore FIFO configuration */

  ad5940_writereg(priv, AD5940_REG_AFE_FIFOCON, fifocon);
}

/****************************************************************************
 * Name: ad5940_seqcmdwrite
 *
 * Description:
 *   Write sequencer commands to AD5940 SRAM.
 *
 * Input Parameters:
 *   priv      - Device struct.
 *   startaddr - Start address
 *   pcmd      - Pointer to the buffer which stores commands
 *   cmdcnt    - Count of command to write.
 *
 * Returned Value:
 *   OK for success, a negated errno for failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_seqcmdwrite(FAR struct ad5940_dev_s *priv,
                              uint32_t startaddr, FAR const uint32_t *pcmd,
                              uint32_t cmdcnt)
{
  while (cmdcnt--)
    {
      ad5940_writereg(priv, AD5940_REG_AFE_CMDFIFOWADDR, startaddr++);
      ad5940_writereg(priv, AD5940_REG_AFE_CMDFIFOWRITE, *pcmd++);
    }

  return OK;
}

/****************************************************************************
 * Name: ad5940_seqgeninit
 *
 * Description:
 *   Initialize sequencer generator with specified buffer. The buffer is used
 *   to store sequencer generated and record register value changes. The
 *   command is stored from start address of buffer while register value is
 *   stored from end of buffer.
 *   Buffer[0] : First sequencer command;
 *   Buffer[1] : Second Sequencer command;
 *   ...
 *   Buffer[Last-1]: The second register value record.
 *   Buffer[Last]: The first register value record.
 *
 * Input Parameters:
 *   priv       - Device structure.
 *   pbuffer    - Pointer to the buffer.
 *   buffersize - The buffer length.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_seqgeninit(FAR struct ad5940_dev_s *priv,
                              FAR struct ad5940_seqgen_s *pseqdb,
                              uint32_t *pbuffer, uint32_t buffersize)
{
  if (buffersize < 2)
    {
      snerr("Buffer was too small for SEQ.\n");
      return;
    }

  pseqdb->buffersize = buffersize;
  pseqdb->pseqbuff = pbuffer;

  /* Point to the last element in buffer. Size of uint32 = size of struct
   * ad5940_seqgenreginfo_s
   */

  pseqdb->preginfo = (FAR struct ad5940_seqgenreginfo_s *)pbuffer +
                                                         buffersize - 1;
  pseqdb->seqlen = 0;
  pseqdb->regcount = 0;
  pseqdb->lasterror = OK;
  pseqdb->enginestart = false;
  priv->pseqgendb = pseqdb;
}

/****************************************************************************
 * Name: ad5940_seqinfocfg
 *
 * Description:
 *   Initialize sequence info. There are four set of registers that record
 *   sequence information. The info contains command start address in SRAM
 *   and sequence length. Hardware can automatically manage these four
 *   sequences. If the application requires more than 4 sequences, user
 *   should manually record the sequence info(address and length) in MCU.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   gpio - Select from {AGPIO_Pinx}. The combination of GPIO pins. The
 *          selected pins will be set to high. Others will be pulled low.
 *
 * Returned Value:
 *   OK for success, a negated errno for failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_seqinfocfg(FAR struct ad5940_dev_s *priv,
                             FAR struct ad5940_seqinfo_s *pseq)
{
  uint16_t regaddr;
  int ret;

  switch (pseq->seqid)
    {
      case AD5940_SEQID_0:
        {
          regaddr = AD5940_REG_AFE_SEQ0INFO;
        }
        break;

      case AD5940_SEQID_1:
        {
          regaddr = AD5940_REG_AFE_SEQ1INFO;
        }
        break;

      case AD5940_SEQID_2:
        {
          regaddr = AD5940_REG_AFE_SEQ2INFO;
        }
        break;

      case AD5940_SEQID_3:
        {
          regaddr = AD5940_REG_AFE_SEQ3INFO;
        }
        break;

      default:
        {
          return -EINVAL;
        }
    }

  ret = ad5940_writereg(priv, regaddr, (pseq->seqlen << 16) |
                                       pseq->seqramaddr);
  if (ret < 0)
    {
      return ret;
    }

  if (pseq->writesram == true)
    {
      ret = ad5940_seqcmdwrite(priv, pseq->seqramaddr, pseq->pseqcmd,
                               pseq->seqlen);
    }

  return ret;
}

/****************************************************************************
 * Name: ad5940_setdexrtia
 *
 * Description:
 *   Configure AD5940 DEx HSTIA resistor.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   dexpin  - Select from AD5940_HSTIADE0PIN, AD5940_HSTIADE1PIN
 *   dertia  - DEx node RTIA selection from HSTIADERTIA_Const
 *   derload - DEx node Rload selection from HSTIADERLOAD_Const
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_setdexrtia(FAR struct ad5940_dev_s *priv, uint32_t dexpin,
                              uint32_t dertia, uint32_t derload)
{
  uint32_t tempreg;

  if (dertia >= AD5940_HSTIADERTIA_OPEN)
    {
      /* RTIA_DEx disconnected, set bit[7:3] to 0x1f */

      tempreg = 0x1f << 3;
    }
  else if (dertia >= AD5940_HSTIADERTIA_1K)
    {
      tempreg = (dertia + AD5940_HSTIADERTIA_OFST) << 3;
    }
  else
    {
      /* DERTIA 50/100/200Ohm */

      if (derload < AD5940_HSTIADERLOAD_OPEN)
        {
          tempreg = (uint32_t)(ad5940_dertiatable[dertia][derload]) << 3;
        }
      else
        {
          /* Set it to HSTIADERTIA_OPEN. */

          tempreg = 0x1f << 3;
        }
    }

  tempreg |= derload;
  if (dexpin)
    {
      ad5940_writereg(priv, AD5940_REG_AFE_DE1RESCON, tempreg);
    }
  else
    {
      ad5940_writereg(priv, AD5940_REG_AFE_DE0RESCON, tempreg);
    }
}

/****************************************************************************
 * Name: ad5940_set_intpin
 *
 * Description:
 *   Configure AD5940 GPO for interrupt.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - Enable interrupt pin or not.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_set_intpin(FAR struct ad5940_dev_s *priv, bool enable)
{
  if (enable)
    {
      ad5940_writereg(priv, AD5940_REG_AGPIO_GP0OEN, AD5940_AGPIO_PIN0);
    }
  else
    {
      ad5940_writereg(priv, AD5940_REG_AGPIO_GP0OEN, 0);
    }

  ad5940_writereg(priv, AD5940_REG_AGPIO_GP0CON, AD5940_GP0_INT0);
  ad5940_writereg(priv, AD5940_REG_AGPIO_GP0IEN, 0);
  ad5940_writereg(priv, AD5940_REG_AGPIO_GP0PE, 0);
  ad5940_writereg(priv, AD5940_REG_AGPIO_GP0OUT, 0);
}

/****************************************************************************
 * Name: ad5940_statisticcfg
 *
 * Description:
 *   Configure AD5940 statistic functions.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configure structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_statisticcfg(FAR struct ad5940_dev_s *priv,
                                FAR const struct ad5940_statcfg_s *pstatcfg)
{
  uint32_t tempreg;

  tempreg = 0;
  if (pstatcfg->statenable == true)
    {
      tempreg |= AD5940_BITM_AFE_STATSCON_STATSEN;
    }

  tempreg = tempreg | (pstatcfg->statsample <<
                       AD5940_BITP_AFE_STATSCON_SMPLENUM);
  tempreg = tempreg | (pstatcfg->statdev << AD5940_BITP_AFE_STATSCON_STDDEV);
  ad5940_writereg(priv, AD5940_REG_AFE_STATSCON, tempreg);
}

/****************************************************************************
 * Name: ad5940_swmatrixcfg
 *
 * Description:
 *   Configure AD5940 switch matrix.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configure structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_swmatrixcfg(FAR struct ad5940_dev_s *priv,
                               FAR const struct ad5940_swmatrixcfg_s *pcfg)
{
  ad5940_writereg(priv, AD5940_REG_AFE_DSWFULLCON, pcfg->dswitch);
  ad5940_writereg(priv, AD5940_REG_AFE_PSWFULLCON, pcfg->pswitch);
  ad5940_writereg(priv, AD5940_REG_AFE_NSWFULLCON, pcfg->nswitch);
  ad5940_writereg(priv, AD5940_REG_AFE_TSWFULLCON, pcfg->tswitch);
  ad5940_writereg(priv, AD5940_REG_AFE_SWCON,
                  AD5940_BITM_AFE_SWCON_SWSOURCESEL);
}

/****************************************************************************
 * Name: ad5940_wgcfg
 *
 * Description:
 *   Configure AD5940 waveform generator
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configure structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_wgcfg(FAR struct ad5940_dev_s *priv,
                         FAR const struct ad5940_wgcfg_s *pcfg)
{
  uint32_t tempreg;

  if (pcfg->wgtype == AD5940_WGTYPE_SIN)
    {
      /* Configure sine wave generator */

      ad5940_writereg(priv, AD5940_REG_AFE_WGFCW, pcfg->sincfg.sinfreqword);
      ad5940_writereg(priv, AD5940_REG_AFE_WGAMPLITUDE,
                      pcfg->sincfg.sinamplitudeword);
      ad5940_writereg(priv, AD5940_REG_AFE_WGOFFSET,
                      pcfg->sincfg.sinoffsetword);
      ad5940_writereg(priv, AD5940_REG_AFE_WGPHASE,
                      pcfg->sincfg.sinphaseword);
    }
  else if (pcfg->wgtype == AD5940_WGTYPE_TRAPZ)
    {
      /* Configure trapezoid generator */

      ad5940_writereg(priv, AD5940_REG_AFE_WGDCLEVEL1,
                      pcfg->trapzcfg.wgtrapzdclevel1);
      ad5940_writereg(priv, AD5940_REG_AFE_WGDCLEVEL2,
                      pcfg->trapzcfg.wgtrapzdclevel2);
      ad5940_writereg(priv, AD5940_REG_AFE_WGDELAY1,
                      pcfg->trapzcfg.wgtrapzdelay1);
      ad5940_writereg(priv, AD5940_REG_AFE_WGDELAY2,
                      pcfg->trapzcfg.wgtrapzdelay2);
      ad5940_writereg(priv, AD5940_REG_AFE_WGSLOPE1,
                      pcfg->trapzcfg.wgtrapzslope1);
      ad5940_writereg(priv, AD5940_REG_AFE_WGSLOPE2,
                      pcfg->trapzcfg.wgtrapzslope2);
    }
  else
    {
      /* Write DAC data. It's only have effect when wgtype set to
       * AD5940_WGTYPE_MMR
       */

      ad5940_writereg(priv, AD5940_REG_AFE_HSDACDAT, pcfg->wgcode);
    }

  tempreg = 0;
  if (pcfg->gaincalen == true)
    {
      tempreg |= AD5940_BITM_AFE_WGCON_DACGAINCAL;
    }

  if (pcfg->offsetcalen == true)
    {
      tempreg |= AD5940_BITM_AFE_WGCON_DACOFFSETCAL;
    }

  tempreg = tempreg | (pcfg->wgtype << AD5940_BITP_AFE_WGCON_TYPESEL);
  ad5940_writereg(priv, AD5940_REG_AFE_WGCON, tempreg);
}

/****************************************************************************
 * Name: ad5940_wgfreqctrl
 *
 * Description:
 *   Update WG SIN wave frequency in Hz.
 *
 * Input Parameters:
 *   priv      - Device struct.
 *   sinfreqhz - The desired frequency in Hz.
 *   WGClock   - The clock for WG. It's same as system clock and the default
 *               value is internal 16MHz HSOSC.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_wgfreqctrl(FAR struct ad5940_dev_s *priv, float sinfreqhz,
                              float wgclock)
{
  uint32_t freq_word;

  freq_word = ad5940_wgfreqwordcal(sinfreqhz, wgclock);
  ad5940_writereg(priv, AD5940_REG_AFE_WGFCW, freq_word);
}

/****************************************************************************
 * Name: ad5940_wuptcfg
 *
 * Description:
 *   Configure AD5940 wakeup timer.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to configure structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_wuptcfg(FAR struct ad5940_dev_s *priv,
                           FAR struct ad5940_wuptcfg_s *pcfg)
{
  uint32_t tempreg;

  /* Sleep and Wakeup time */

  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ0WUPL,
                  pcfg->seqxwakeuptime[0] & 0xffff);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ0WUPH,
                  (pcfg->seqxwakeuptime[0] & 0xf0000) >> 16);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ0SLEEPL,
                  pcfg->seqxsleeptime[0] & 0xffff);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ0SLEEPH,
                  (pcfg->seqxsleeptime[0] & 0xf0000) >> 16);

  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ1WUPL,
                  pcfg->seqxwakeuptime[1] & 0xffff);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ1WUPH,
                  (pcfg->seqxwakeuptime[1] & 0xf0000) >> 16);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ1SLEEPL,
                  pcfg->seqxsleeptime[1] & 0xffff);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ1SLEEPH,
                  (pcfg->seqxsleeptime[1] & 0xf0000) >> 16);

  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ2WUPL,
                  pcfg->seqxwakeuptime[2] & 0xffff);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ2WUPH,
                  (pcfg->seqxwakeuptime[2] & 0xf0000) >> 16);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ2SLEEPL,
                  pcfg->seqxsleeptime[2] & 0xffff);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ2SLEEPH,
                  (pcfg->seqxsleeptime[2] & 0xf0000) >> 16);

  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ3WUPL,
                  pcfg->seqxwakeuptime[3] & 0xffff);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ3WUPH,
                  (pcfg->seqxwakeuptime[3] & 0xf0000) >> 16);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ3SLEEPL,
                  pcfg->seqxsleeptime[3] & 0xffff);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQ3SLEEPH,
                  (pcfg->seqxsleeptime[3] & 0xf0000) >> 16);

  /* We always allow Wupt to wakeup AFE automatically. */

  ad5940_writereg(priv, AD5940_REG_ALLON_TMRCON,
                  AD5940_BITM_ALLON_TMRCON_TMRINTEN);

  /* Wupt order */

  tempreg = 0;
  tempreg = tempreg | ((pcfg->wuptorder[0] & 0x03)
                       << AD5940_BITP_WUPTMR_SEQORDER_SEQA);
  tempreg = tempreg | ((pcfg->wuptorder[1] & 0x03)
                       << AD5940_BITP_WUPTMR_SEQORDER_SEQB);
  tempreg = tempreg | ((pcfg->wuptorder[2] & 0x03)
                       << AD5940_BITP_WUPTMR_SEQORDER_SEQC);
  tempreg = tempreg | ((pcfg->wuptorder[3] & 0x03)
                       << AD5940_BITP_WUPTMR_SEQORDER_SEQD);
  tempreg = tempreg | ((pcfg->wuptorder[4] & 0x03)
                       << AD5940_BITP_WUPTMR_SEQORDER_SEQE);
  tempreg = tempreg | ((pcfg->wuptorder[5] & 0x03)
                       << AD5940_BITP_WUPTMR_SEQORDER_SEQF);
  tempreg = tempreg | ((pcfg->wuptorder[6] & 0x03)
                       << AD5940_BITP_WUPTMR_SEQORDER_SEQG);
  tempreg = tempreg | ((pcfg->wuptorder[7] & 0x03)
                       << AD5940_BITP_WUPTMR_SEQORDER_SEQH);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_SEQORDER, tempreg);

  tempreg = 0;
  if (pcfg->wupten == true)
    {
      tempreg |= AD5940_BITM_WUPTMR_CON_EN;
    }

  /* We always allow Wupt to trigger sequencer */

  tempreg = tempreg | (pcfg->wuptendseq << AD5940_BITP_WUPTMR_CON_ENDSEQ);
  ad5940_writereg(priv, AD5940_REG_WUPTMR_CON, tempreg);
}

/****************************************************************************
 * Name: ad5940_wuptctrl
 *
 * Description:
 *   Enable or disable AD5940 wakeup timer.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - TRUE: enable wakeup timer; FALSE: disable wakeup timer.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_wuptctrl(FAR struct ad5940_dev_s *priv, bool enable)
{
  uint32_t tempreg;

  ad5940_readreg(priv, AD5940_REG_WUPTMR_CON, &tempreg);
  tempreg = tempreg & (~AD5940_BITM_WUPTMR_CON_EN);
  if (enable == true)
    {
      tempreg |= AD5940_BITM_WUPTMR_CON_EN;
    }

  ad5940_writereg(priv, AD5940_REG_WUPTMR_CON, tempreg);
}

/****************************************************************************
 * Name: ad5940_biacfgstructinit
 *
 * Description:
 *   Initialize BIA configurations in device struct.
 *
 * Input Parameters:
 *   priv - Device struct
 *   pcfg - Pointer to BIA configuration structure.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_biacfgstructinit(FAR struct ad5940_dev_s *priv,
                                    FAR struct ad5940_biacfg_s *pbiacfg)
{
  /* Set some members that are different from default values. */

  pbiacfg->biaodr = AD5940_ONE_SECOND / (float)priv->interval;
  pbiacfg->fifothreshd = priv->fifowtm * AD5940_RESULTS_PER_MEAS;

  /* Recover the members that may change during last activating. */

  pbiacfg->biainited = false;
  pbiacfg->bparachanged = false;
  pbiacfg->redortiacal = false;
  pbiacfg->freqofdata = 0;
  pbiacfg->maxodr = 0;
  pbiacfg->measseqcyclecount = 0;
  pbiacfg->rtiacurrvalue[0] = 0;
  pbiacfg->rtiacurrvalue[1] = 0;
  pbiacfg->sweepcurrfreq = 0;
  pbiacfg->sweepnextfreq = 0;
  memset(&pbiacfg->initseqinfo, 0, sizeof(struct ad5940_seqinfo_s));
  memset(&pbiacfg->measureseqinfo, 0, sizeof(struct ad5940_seqinfo_s));
  memcpy(&pbiacfg->sweepcfg, &ad5940_biacfgdefault.sweepcfg,
         sizeof(struct ad5940_softsweepcfg_s));
}

/****************************************************************************
 * Name: ad5940_biaplatformcfg
 *
 * Description:
 *   Initialize AD5940 basic blocks for BIA(Bioelectrical Impedance Analysis)
 *   function.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_biaplatformcfg(FAR struct ad5940_dev_s *priv)
{
  FAR struct ad5940_fifocfg_s fifocfg;
  int ret;

  ad5940_hardreset(priv);
  ad5940_init(priv);

  /* Step1. Configure clock. Internal 16MHz and 32768Hz */

  ret = ad5940_clkcfg(priv);
  if (ret < 0)
    {
      snerr("Failed to configure AD5940 CLK: %d\n", ret);
      return ret;
    }

  /* Step2. Configure FIFO and Sequencer. FIFO 4KB and seq 2KB. */

  fifocfg.fifomode    = AD5940_FIFOMODE_FIFO;
  fifocfg.fifosize    = AD5940_FIFOSIZE_4KB;
  fifocfg.fifosrc     = AD5940_FIFOSRC_DFT;
  fifocfg.fifothreshd = priv->fifowtm * AD5940_RESULTS_PER_MEAS;
  fifocfg.fifoen      = true;
  ad5940_fifocfg(priv, &fifocfg);

  /* Step3. Interrupt controller. Interrupt Controller 0 will control GP0 to
   * generate a rising edge interrupt to MCU.
   */

  ad5940_intrptclr(priv, AD5940_AFEINTSRC_ALLINT);
  ad5940_intrptcfg(priv, AD5940_AFEINTC_1, AD5940_AFEINTSRC_ALLINT, true);
  ad5940_intrptcfg(priv, AD5940_AFEINTC_0, AD5940_AFEINTSRC_DATAFIFOTHRESH,
                   true);
  ad5940_spiwritereg(priv, AD5940_REG_INTC_INTCPOL,
                     AD5940_ENUM_INTC_INTCPOL_RISING);

  /* Step4: Configure GPIO. GP0 for INT0, no pull, output 0, no input. */

  if (priv->fifowtm > 0)
    {
      ad5940_set_intpin(priv, true);
    }

  /* Allow AFE to enter sleep mode. */

  return ad5940_writereg(priv, AD5940_REG_AFE_SEQSLPLOCK,
                         AD5940_SLPKEY_UNLOCK);
}

/****************************************************************************
 * Name: ad5940_biafuncinit
 *
 * Description:
 *   BIA functions initialization.
 *
 * Input Parameters:
 *    priv    - Device struct.
 *    pcfg    - Pointer to BIA configuration structure.
 *    bufsize - Length of the array, which is to store sequencer commands
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_biafuncinit(FAR struct ad5940_dev_s *priv,
                              FAR struct ad5940_biacfg_s *pbiacfg,
                              const uint32_t bufsize)
{
  FAR struct ad5940_seqcfg_s seq_cfg;
  FAR struct ad5940_fifocfg_s fifo_cfg;
  FAR struct ad5940_seqgen_s *pseqdb;
  uint32_t *pbuf;
  uint32_t tempreg;
  int ret;

  /* Initialize memory for BIA */

  pseqdb = kmm_zalloc(sizeof(struct ad5940_seqgen_s));
  if (pseqdb == NULL)
    {
      snerr("No space to store SEQ database.\n");
      return -ENOMEM;
    }

  pbuf = kmm_zalloc(bufsize * sizeof(uint32_t));
  if (pbuf == NULL)
    {
      snerr("No space to store SEQ buffer.\n");
      kmm_free(pseqdb);
      return -ENOMEM;
    }

  /* Wakeup AFE by read register, read 10 times at most */

  ret = ad5940_wakeup(priv, 10);
  if (ret < 0)
    {
      goto end;
    }

  /* Configure sequencer and stop it */

  seq_cfg.seqmemsize   = AD5940_SEQMEMSIZE_2KB;
  seq_cfg.seqbreaken   = false;
  seq_cfg.seqignoreen  = false;
  seq_cfg.seqcntcrcclr = true;
  seq_cfg.seqenable    = false;
  seq_cfg.seqwrtimer   = 0;
  ad5940_seqcfg(priv, &seq_cfg);

  /* Do calibration on the first initializaion */

  if (pbiacfg->redortiacal == true || pbiacfg->biainited == false)
    {
      ret = ad5940_biartiacal(priv, pbiacfg);
      if (ret < 0)
        {
          snerr("Failed to calibrate BIA RTIA: %d\n", ret);
          goto end;
        }

      pbiacfg->redortiacal = false;
    }

  /* Reconfigure FIFO. Disable FIFO firstly */

  ad5940_fifoctrl(priv, AD5940_FIFOSRC_DFT, false);
  fifo_cfg.fifoen = true;
  fifo_cfg.fifomode = AD5940_FIFOMODE_FIFO;
  fifo_cfg.fifosize = AD5940_FIFOSIZE_4KB;
  fifo_cfg.fifosrc = AD5940_FIFOSRC_DFT;
  fifo_cfg.fifothreshd = priv->biacfg.fifothreshd;
  ad5940_fifocfg(priv, &fifo_cfg);
  ad5940_intrptclr(priv, AD5940_AFEINTSRC_ALLINT);

  /* Initialize sequencer generator */

  if (pbiacfg->biainited == false || pbiacfg->bparachanged == true)
    {
      if (bufsize < 2)
        {
          ret = -EINVAL;
          goto end;
        }

      ad5940_seqgeninit(priv, pseqdb, pbuf, bufsize);

      /* Generate initialize sequence */

      ret = ad5940_biaseqcfggen(priv, pbiacfg);
      if (ret < 0)
        {
          snerr("BIA init seq generating failed.\n");
          goto end;
        }

      /* Generate measurement sequence */

      ret = ad5940_biaseqmeasuregen(priv, pbiacfg);
      if (ret < 0)
        {
          snerr("BIA measure seq generating failed.\n");
          goto end;
        }

      /* Clear this flag as we already implemented the new configuration */

      pbiacfg->bparachanged = false;
    }

  /* Initialization sequencer  */

  pbiacfg->initseqinfo.writesram = false;
  ad5940_seqinfocfg(priv, &pbiacfg->initseqinfo);

  /* Enable initial sequencer and trigger it. */

  seq_cfg.seqenable = true;
  ad5940_seqcfg(priv, &seq_cfg);
  ad5940_writereg(priv, AD5940_REG_AFECON_TRIGSEQ,
                  1L << pbiacfg->initseqinfo.seqid);

  /* Wait for initial sequencer end. */

  ret = ad5940_waitflag(priv, AD5940_REG_INTC_INTCFLAG1,
                        AD5940_AFEINTSRC_ENDSEQ, AD5940_TIMEOUT_INISEQ);
  if (ret < 0)
    {
      snerr("Init seq was timeout.\n");
      goto end;
    }

  /* Measurement sequence  */

  pbiacfg->measureseqinfo.writesram = false;
  ad5940_seqinfocfg(priv, &pbiacfg->measureseqinfo);

  /* Enable sequencer, and wait for trigger. */

  seq_cfg.seqenable = true;
  ad5940_seqcfg(priv, &seq_cfg);

  /* Set AFE power mode and system bandwidth. */

  tempreg = pbiacfg->pwrmod | AD5940_ENUM_AFE_PMBW_BW250;
  ad5940_writereg(priv, AD5940_REG_AFE_PMBW, tempreg);

  /* Configure AD5940 swtich MUX for measurement. */

  ad5940_writereg(priv, AD5940_REG_AFE_SWMUX, 1 << 3);

  /* BIA application has been initialized. */

  pbiacfg->biainited = true;

end:
  kmm_free(pseqdb);
  kmm_free(pbuf);
  priv->pseqgendb = NULL;
  return ret;
}

/****************************************************************************
 * Name: ad5940_biactrl
 *
 * Description:
 *   BIA functions initialization.
 *
 * Input Parameters:
 *    priv    - Device struct.
 *    pbiacfg - Pointer to BIA configuration structure.
 *    ctrlcmd - Control command word
 *    ppara   - Pointer to parameters
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_biactrl(FAR struct ad5940_dev_s *priv,
                          FAR struct ad5940_biacfg_s *pbiacfg,
                          int32_t ctrlcmd, FAR void *ppara)
{
  int ret;

  if (pbiacfg == NULL || ctrlcmd > AD5940_BIACTRL_SHUTDOWN)
    {
      return -EINVAL;
    }

  switch (ctrlcmd)
    {
      case AD5940_BIACTRL_START:
        {
          FAR struct ad5940_wuptcfg_s *wuptcfg;

          wuptcfg = kmm_zalloc(sizeof(struct ad5940_wuptcfg_s));
          if (wuptcfg == NULL)
            {
              return -ENOMEM;
            }

          /* Wakeup AFE by read register, read 10 times at most */

          ret = ad5940_wakeup(priv, 10);
          if (ret < 0)
            {
              kmm_free(wuptcfg);
              return ret;
            }

          if (pbiacfg->biainited == false)
            {
              kmm_free(wuptcfg);
              return -EPERM;
            }

          /* Start it */

          wuptcfg->wupten = true;
          wuptcfg->wuptendseq = AD5940_WUPTENDSEQ_A;
          wuptcfg->wuptorder[0] = AD5940_SEQID_0;
          wuptcfg->seqxsleeptime[AD5940_SEQID_0] =
            (uint32_t)(pbiacfg->wuptclkfreq / pbiacfg->biaodr) - 2 - 1;

          /* The minimum value is 1. Do not set it to zero. Set it to 1 will
           * spend 2 32kHz clock.
           */

          wuptcfg->seqxwakeuptime[AD5940_SEQID_0] = 1;
          ad5940_wuptcfg(priv, wuptcfg);

          kmm_free(wuptcfg);
        }
        break;

      case AD5940_BIACTRL_STOPNOW:
        {
          /* Wakeup AFE by read register, read 10 times at most */

          ret = ad5940_wakeup(priv, 10);
          if (ret < 0)
            {
              return ret;
            }

          /* Start WUPT right now */

          ad5940_wuptctrl(priv, true);

          /* There is chance this operation will fail because sequencer could
           * put AFE back to hibernate mode just after waking up. Use
           * STOPSYNC is better.
           */

          ad5940_wuptctrl(priv, false);
        }
        break;

      case AD5940_BIACTRL_GETFREQ:
        {
          if (ppara)
          {
            if (pbiacfg->sweepcfg.sweepen == true)
              {
                *(float *)ppara = pbiacfg->freqofdata;
              }
            else
              {
                *(float *)ppara = pbiacfg->sinfreq;
              }
          }
        }
        break;

      case AD5940_BIACTRL_SHUTDOWN:
        {
          /* Stop the measurement if it's running. */

          ad5940_biactrl(priv, pbiacfg, AD5940_BIACTRL_STOPNOW, NULL);

          /* Reset the device to avoid wake-up by sequencer. */

          ad5940_hardreset(priv);

          /* Turn off LPloop related blocks which are not controlled
           * automatically by sleep operation
           */

          ad5940_shutdown(priv);
        }
        break;

      default:
        {
          /* Can't reach here */
        }
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: ad5940_ad5940_biadataprocess
 *
 * Description:
 *   Process the readouts into sensor results and push them to topic.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   pdata - Pointer to array which stores results.
 *   cnt   - Data count in pdata.
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_biadataprocess(FAR struct ad5940_dev_s *priv,
                                 FAR uint32_t *pdata, uint32_t cnt)
{
  struct sensor_impd pout[AD5940_FIFOSLOTS_MAX];
  FAR struct ad5940_iimpcar_s *psrc = (FAR struct ad5940_iimpcar_s *)pdata;
  FAR struct ad5940_iimpcar_s *pdftvolt;
  FAR struct ad5940_iimpcar_s *pdftcurr;
  float voltmag;
  float voltphase;
  float currmag;
  float currphase;
  uint32_t imprescnt = cnt / AD5940_RESULTS_PER_MEAS;
  uint32_t i;

  /* Convert DFT result to int32_t type */

  for (i = 0; i < cnt; i++)
    {
      /* Bit17 is the sign bit. */

      pdata[i] &= 0x3ffff;
      if (pdata[i] & (1 << 17))
        {
          pdata[i] |= 0xfffc0000;
        }
    }

  for (i = 0; i < imprescnt; i++)
    {
      pdftcurr = psrc++;
      pdftvolt = psrc++;

      voltmag = sqrt((float)pdftvolt->real * pdftvolt->real +
                     (float)pdftvolt->image * pdftvolt->image);
      voltphase = atan2(-pdftvolt->image, pdftvolt->real);
      currmag = sqrt((float)pdftcurr->real * pdftcurr->real +
                     (float)pdftcurr->image * pdftcurr->image);
      currphase = atan2(-pdftcurr->image, pdftcurr->real);
      voltmag = voltmag / currmag * priv->biacfg.rtiacurrvalue[0];
      voltphase = voltphase - currphase + priv->biacfg.rtiacurrvalue[1];
      pout[i].real = voltmag;
      pout[i].imag = voltphase;
      if (priv->fifowtm > 0)
        {
          pout[i].timestamp = priv->timestamp - priv->interval *
                              (priv->fifowtm - i - 1);
        }
      else
        {
          pout[i].timestamp = priv->timestamp;
        }
    }

  priv->lower.push_event(priv->lower.priv, pout,
                         imprescnt * sizeof(struct sensor_impd));

  /* Calculate next frequency point */

  if (priv->biacfg.sweepcfg.sweepen)
    {
      priv->biacfg.freqofdata = priv->biacfg.sweepcurrfreq;
      priv->biacfg.sweepcurrfreq = priv->biacfg.sweepnextfreq;
      priv->biacfg.rtiacurrvalue[0] =
          priv->biacfg.rtiacaltable[priv->biacfg.sweepcfg.sweepindex][0];
      priv->biacfg.rtiacurrvalue[1] =
          priv->biacfg.rtiacaltable[priv->biacfg.sweepcfg.sweepindex][1];
      ad5940_sweepnext(&priv->biacfg.sweepcfg,
                       &priv->biacfg.sweepnextfreq);
    }

  return OK;
}

/****************************************************************************
 * Name: ad5940_biaregmodify
 *
 * Description:
 *    Modify registers when AFE wakeup。
 *
 * Input Parameters:
 *    priv    - Device struct.
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_biaregmodify(FAR struct ad5940_dev_s *priv,
                               FAR struct ad5940_biacfg_s *pbiacfg)
{
  /* Need to set new frequency and set power mode */

  if (pbiacfg->sweepcfg.sweepen)
    {
      ad5940_wgfreqctrl(priv, pbiacfg->sweepnextfreq, pbiacfg->sysclkfreq);
    }

  /* If AFE will switch between batch mode and polling mode. */

  if (priv->poll2batch)
    {
      work_cancel(HPWORK, &priv->work_poll);
      IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                      IOEXPANDER_OPTION_INTCFG,
                      (FAR void *)IOEXPANDER_VAL_RISING);
      ad5940_set_intpin(priv, true);
      priv->poll2batch = false;
    }
  else if (priv->batch2poll)
    {
      work_queue(HPWORK, &priv->work_poll, ad5940_worker_poll, priv,
                     priv->interval / USEC_PER_TICK);
      IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                      IOEXPANDER_OPTION_INTCFG,
                      (FAR void *)IOEXPANDER_VAL_DISABLE);
      ad5940_set_intpin(priv, false);
      priv->batch2poll = false;
    }

  /* A new FIFO watermark need to be set, causing by ad5940_batch(). If AFE
   * is changing from polling mode to batch mode, this case is also suitable.
   */

  if (priv->fifowtm > 0 &&
      pbiacfg->fifothreshd != priv->fifowtm * AD5940_RESULTS_PER_MEAS)
    {
      pbiacfg->fifothreshd = priv->fifowtm * AD5940_RESULTS_PER_MEAS;
      ad5940_writereg(priv, AD5940_REG_AFE_DATAFIFOTHRES,
                      pbiacfg->fifothreshd <<
                      AD5940_BITP_AFE_DATAFIFOTHRES_HIGHTHRES);
    }

  /* A new interval need to be set, causing by ad5940_interval() */

  if (pbiacfg->biaodr != priv->odr)
    {
      /* Stop WUPT */

      ad5940_wuptctrl(priv, false);
      pbiacfg->biaodr = priv->odr;

      /* When BIA start again, new ODR is set by new WUPT config */

      ad5940_biactrl(priv, pbiacfg, AD5940_BIACTRL_START, NULL);
    }

  return OK;
}

/****************************************************************************
 * Name: ad5940_biartiacal
 *
 * Description:
 *   Calibrate RTIA for BIA.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to BIA configuration structure. Some items in it will be
 *          changed in this function, so there's no "const" prefix.
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_biartiacal(FAR struct ad5940_dev_s *priv,
                             FAR struct ad5940_biacfg_s *pbiacfg)
{
  FAR struct ad5940_hsrtiacal_s *phsrtiacal;
  uint32_t i;
  int ret;

  phsrtiacal = kmm_zalloc(sizeof(struct ad5940_hsrtiacal_s));
  if (phsrtiacal == NULL)
    {
      return -ENOMEM;
    }

  phsrtiacal->adcclkfreq = pbiacfg->adcclkfreq;
  phsrtiacal->adcsinc2osr = pbiacfg->adcsinc2osr;
  phsrtiacal->adcsinc3osr = pbiacfg->adcsinc3osr;
  phsrtiacal->bpolarresult = true;
  phsrtiacal->dftcfg.dftnum = pbiacfg->dftnum;
  phsrtiacal->dftcfg.dftsrc = pbiacfg->dftsrc;
  phsrtiacal->dftcfg.hanwinen = pbiacfg->hanwinen;
  phsrtiacal->frcal = pbiacfg->rcalval;
  phsrtiacal->hstiacfg.hstiabias = AD5940_HSTIABIAS_1P1;
  phsrtiacal->hstiacfg.hstiactia = pbiacfg->ctiasel;
  phsrtiacal->hstiacfg.hstiaderload = AD5940_HSTIADERLOAD_OPEN;
  phsrtiacal->hstiacfg.hstiadertia = AD5940_HSTIADERTIA_TODE;
  phsrtiacal->hstiacfg.hstiartiasel = pbiacfg->hstiartiasel;
  phsrtiacal->sysclkfreq = pbiacfg->sysclkfreq;
  phsrtiacal->ffreq = pbiacfg->sweepcfg.sweepstart;

  if (pbiacfg->sweepcfg.sweepen == true)
    {
      pbiacfg->sweepcfg.sweepindex = 0;
      for (i = 0; i < pbiacfg->sweepcfg.sweeppoints; i++)
        {
          ret = ad5940_hsrtiacal(priv, phsrtiacal, pbiacfg->rtiacaltable[i]);
          if (ret < 0)
            {
              snerr("Failed to calibrate HS RTIA.\n");
              kmm_free(phsrtiacal);
              return ret;
            }

          ad5940_sweepnext(&pbiacfg->sweepcfg, &phsrtiacal->ffreq);
        }

      pbiacfg->sweepcfg.sweepindex = 0;
      pbiacfg->rtiacurrvalue[0] =
               pbiacfg->rtiacaltable[pbiacfg->sweepcfg.sweepindex][0];
      pbiacfg->rtiacurrvalue[1] =
               pbiacfg->rtiacaltable[pbiacfg->sweepcfg.sweepindex][1];
    }
  else
    {
      phsrtiacal->ffreq = pbiacfg->sinfreq;
      ret = ad5940_hsrtiacal(priv, phsrtiacal, pbiacfg->rtiacurrvalue);
      if (ret < 0)
        {
          snerr("Failed to calibrate HS RTIA.\n");
          kmm_free(phsrtiacal);
          return ret;
        }
    }

  kmm_free(phsrtiacal);
  return OK;
}

/****************************************************************************
 * Name: ad5940_biaseqcfggen
 *
 * Description:
 *   Generate init sequence for BIA.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   pcfg - Pointer to the BIA configure struct.
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_biaseqcfggen(FAR struct ad5940_dev_s *priv,
                               FAR struct ad5940_biacfg_s *pbiacfg)
{
  FAR struct ad5940_aferefcfg_s aferefcfg;
  FAR struct ad5940_hsloopcfg_s *phsloop;
  FAR struct ad5940_lploopcfg_s *plploop;
  FAR struct ad5940_dspcfg_s *pdspcfg;
  uint32_t const *pseqcmd;
  uint32_t seqlen;
  float sinfreq;
  int ret;

  phsloop = kmm_zalloc(sizeof(struct ad5940_hsloopcfg_s));
  if (phsloop == NULL)
    {
      return -ENOMEM;
    }

  plploop = kmm_zalloc(sizeof(struct ad5940_lploopcfg_s));
  if (plploop == NULL)
    {
      kmm_free(phsloop);
      return -ENOMEM;
    }

  pdspcfg = kmm_zalloc(sizeof(struct ad5940_dspcfg_s));
  if (pdspcfg == NULL)
    {
      kmm_free(phsloop);
      kmm_free(plploop);
      return -ENOMEM;
    }

  /* Start sequence generator here, ad5940_writereg/readreg will operate the
   * sequencer database.
   */

  ad5940_seqgenstart(priv->pseqgendb);

  /* Init all to disable state */

  aferefcfg.hpbandgapen = true;
  aferefcfg.hp1v1buffen = true;
  aferefcfg.hp1v8buffen = true;
  aferefcfg.disc1v1cap = false;
  aferefcfg.disc1v8cap = false;
  aferefcfg.hp1v8thembuff = false;
  aferefcfg.hp1v8ilimit = false;
  aferefcfg.lp1v1buffen = false;
  aferefcfg.lp1v8buffen = false;

  /* LP reference control - turn off them to save power */

  aferefcfg.lpbandgapen = true;
  aferefcfg.lprefbufen = true;
  aferefcfg.lprefboosten = false;
  ad5940_refcfg(priv, &aferefcfg);

  phsloop->hsdaccfg.excitbufgain = pbiacfg->excitbufgain;
  phsloop->hsdaccfg.hsdacgain = pbiacfg->hsdacgain;
  phsloop->hsdaccfg.hsdacupdaterate = pbiacfg->hsdacupdaterate;
  phsloop->hstiacfg.diodeclose = false;
  phsloop->hstiacfg.hstiabias = AD5940_HSTIABIAS_1P1;
  phsloop->hstiacfg.hstiactia = pbiacfg->ctiasel;
  phsloop->hstiacfg.hstiaderload = AD5940_HSTIADERLOAD_OPEN;
  phsloop->hstiacfg.hstiadertia = AD5940_HSTIADERTIA_OPEN;
  phsloop->hstiacfg.hstiartiasel = pbiacfg->hstiartiasel;
  phsloop->swmatcfg.dswitch = AD5940_SWD_OPEN;
  phsloop->swmatcfg.pswitch = AD5940_SWP_PL | AD5940_SWP_PL2;
  phsloop->swmatcfg.nswitch = AD5940_SWN_NL | AD5940_SWN_NL2;
  phsloop->swmatcfg.tswitch = AD5940_SWT_TRTIA;
  phsloop->wgcfg.wgtype = AD5940_WGTYPE_SIN;
  phsloop->wgcfg.gaincalen = false;
  phsloop->wgcfg.offsetcalen = false;
  if (pbiacfg->sweepcfg.sweepen == true)
    {
      pbiacfg->sweepcfg.sweepindex = 0;
      pbiacfg->freqofdata = pbiacfg->sweepcfg.sweepstart;
      pbiacfg->sweepcurrfreq = pbiacfg->sweepcfg.sweepstart;
      ad5940_sweepnext(&pbiacfg->sweepcfg, &pbiacfg->sweepnextfreq);
      sinfreq = pbiacfg->sweepcurrfreq;
    }
  else
    {
      sinfreq = pbiacfg->sinfreq;
      pbiacfg->freqofdata = sinfreq;
    }

  phsloop->wgcfg.sincfg.sinfreqword =
           ad5940_wgfreqwordcal(sinfreq, pbiacfg->sysclkfreq);
  phsloop->wgcfg.sincfg.sinamplitudeword =
           (uint32_t)(pbiacfg->dacvoltpp / 800.0f * 2047 + 0.5f);
  phsloop->wgcfg.sincfg.sinoffsetword = 0;
  phsloop->wgcfg.sincfg.sinphaseword = 0;
  ret = ad5940_hsloopcfg(priv, phsloop);
  if (ret < 0)
    {
      snerr("Failed to write hsloop config in SEQ.\n");
      goto end;
    }

  plploop->lpdaccfg.lpdacsel = AD5940_LPDAC0;
  plploop->lpdaccfg.lpdacsrc = AD5940_LPDACSRC_MMR;
  plploop->lpdaccfg.lpdacsw = AD5940_LPDACSW_VBIAS2LPPA |
                              AD5940_LPDACSW_VBIAS2PIN |
                              AD5940_LPDACSW_VZERO2LPTIA |
                              AD5940_LPDACSW_VZERO2PIN;
  plploop->lpdaccfg.vzeromux = AD5940_LPDACVZERO_6BIT;
  plploop->lpdaccfg.vbiasmux = AD5940_LPDACVBIAS_12BIT;
  plploop->lpdaccfg.lpdacref = AD5940_LPDACREF_2P5;
  plploop->lpdaccfg.datarst = false;
  plploop->lpdaccfg.poweren = true;
  plploop->lpdaccfg.dacdata12bit = (uint32_t)((1100 - 200) / 2200.0 * 4095);
  plploop->lpdaccfg.dacdata6bit = 31;
  plploop->lpampcfg.lpampsel = AD5940_LPAMP0;
  plploop->lpampcfg.lpamppwrmod = AD5940_LPAMPPWR_NORM;
  plploop->lpampcfg.lppapwren = true;
  plploop->lpampcfg.lptiapwren = true;
  plploop->lpampcfg.lptiarf = AD5940_LPTIARF_20K;
  plploop->lpampcfg.lptiarload = AD5940_LPTIARLOAD_SHORT;
  plploop->lpampcfg.lptiartia = AD5940_LPTIARTIA_OPEN;
  plploop->lpampcfg.lptiasw = AD5940_LPTIASW(5) | AD5940_LPTIASW(6) |
                              AD5940_LPTIASW(7) | AD5940_LPTIASW(8) |
                              AD5940_LPTIASW(9) | AD5940_LPTIASW(12) |
                              AD5940_LPTIASW(13);
  ad5940_lploopcfg(priv, plploop);

  pdspcfg->adcbasecfg.adcmuxn = AD5940_ADCMUXN_HSTIA_N;
  pdspcfg->adcbasecfg.adcmuxp = AD5940_ADCMUXP_HSTIA_P;
  pdspcfg->adcbasecfg.adcpga = pbiacfg->adcpgagain;
  memset(&pdspcfg->adcdigcompcfg, 0, sizeof(struct ad5940_adcdigcomp_s));
  pdspcfg->adcfiltercfg.adcavgnum = AD5940_ADCAVGNUM_16;
  pdspcfg->adcfiltercfg.adcrate = AD5940_ADCRATE_800KHZ;
  pdspcfg->adcfiltercfg.adcsinc2osr = pbiacfg->adcsinc2osr;
  pdspcfg->adcfiltercfg.adcsinc3osr = pbiacfg->adcsinc3osr;
  pdspcfg->adcfiltercfg.bpsinc3 = false;
  pdspcfg->adcfiltercfg.bpnotch = true;
  pdspcfg->adcfiltercfg.sinc2notchenable = true;
  pdspcfg->dftcfg.dftnum = pbiacfg->dftnum;
  pdspcfg->dftcfg.dftsrc = pbiacfg->dftsrc;
  pdspcfg->dftcfg.hanwinen = pbiacfg->hanwinen;

  /* Don't care about Statistic */

  memset(&pdspcfg->statcfg, 0, sizeof(struct ad5940_statcfg_s));
  ret = ad5940_dspcfg(priv, pdspcfg);
  if (ret < 0)
    {
      snerr("Failed to write dsp config in SEQ.\n");
      goto end;
    }

  /* Enable all of them. They are automatically turned off during hibernate
   * mode to save power
   */

  ad5940_afectrl(priv, AD5940_AFECTRL_HPREFPWR | AD5940_AFECTRL_HSTIAPWR |
                       AD5940_AFECTRL_INAMPPWR | AD5940_AFECTRL_EXTBUFPWR |
                       AD5940_AFECTRL_WG | AD5940_AFECTRL_DACREFPWR |
                       AD5940_AFECTRL_HSDACPWR | AD5940_AFECTRL_SINC2NOTCH,
                       true);
  ad5940_writereg(priv, AD5940_REG_AFE_SYNCEXTDEVICE, 0);

  /* Sequence end. Add one extra command to disable sequencer for
   * initialization sequence because we only want it to run one time.
   */

  ad5940_seqgeninsert(priv->pseqgendb, AD5940_SEQ_STOP());

  /* Stop here */

  ret = ad5940_seqgenfetchseq(priv->pseqgendb, &pseqcmd, &seqlen);
  if (ret < 0)
    {
      snerr("SEQ GEN fetch seq error.\n");
      goto end;
    }

  /* Stop sequencer generator, ad5940_writereg/readreg will operate SPI. */

  priv->pseqgendb->enginestart = false;

  pbiacfg->initseqinfo.seqid = AD5940_SEQID_1;
  pbiacfg->initseqinfo.seqramaddr = pbiacfg->seqstartaddr;
  pbiacfg->initseqinfo.pseqcmd = pseqcmd;
  pbiacfg->initseqinfo.seqlen = seqlen;

  /* Write command to AD5940 SRAM */

  ret = ad5940_seqcmdwrite(priv, pbiacfg->initseqinfo.seqramaddr, pseqcmd,
                           seqlen);

end:
  kmm_free(plploop);
  kmm_free(phsloop);
  kmm_free(pdspcfg);
  return ret;
}

/****************************************************************************
 * Name: ad5940_biaseqmeasuregen
 *
 * Description:
 *   Generate init sequence for BIA.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   pcfg - Pointer to the BIA configure struct.
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_biaseqmeasuregen(FAR struct ad5940_dev_s *priv,
                                   FAR struct ad5940_biacfg_s *pbiacfg)
{
  FAR struct ad5940_swmatrixcfg_s swcfg;
  FAR struct ad5940_clkscalinfo_s clkscal;
  uint32_t const *pseqcmd;
  uint32_t seqlen;
  uint32_t waitclks;
  int ret;

  clkscal.datatype = AD5940_DATATYPE_DFT;
  clkscal.dftsrc = pbiacfg->dftsrc;

  /* 2 ^ (dftnumber + 2) */

  clkscal.datacount = 1L << (pbiacfg->dftnum + 2);
  clkscal.adcsinc2osr = pbiacfg->adcsinc2osr;
  clkscal.adcsinc3osr = pbiacfg->adcsinc3osr;
  clkscal.adcavgnum = 0;
  clkscal.ratiosys2adcclk = pbiacfg->sysclkfreq / pbiacfg->adcclkfreq;
  ad5940_clkscalculate(&clkscal, &waitclks);

  /* Start sequence generator here, ad5940_writereg/readreg will operate the
   * sequencer database.
   */

  ad5940_seqgenstart(priv->pseqgendb);

  ad5940_writereg(priv, AD5940_REG_AFE_SYNCEXTDEVICE, AD5940_AGPIO_PIN6);

  ad5940_seqgeninsert(priv->pseqgendb, AD5940_SEQ_WAIT(16 * 250));
  swcfg.dswitch = AD5940_SWD_CE0;
  swcfg.pswitch = AD5940_SWP_CE0;
  swcfg.nswitch = AD5940_SWN_AIN1;
  swcfg.tswitch = AD5940_SWT_AIN1 | AD5940_SWT_TRTIA;
  ad5940_swmatrixcfg(priv, &swcfg);

  ad5940_adcmuxcfg(priv, AD5940_ADCMUXP_HSTIA_P, AD5940_ADCMUXN_HSTIA_N);

  /* Enable Waveform generator, ADC power */

  ad5940_afectrl(priv, AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR, true);
  ad5940_seqgeninsert(priv->pseqgendb, AD5940_SEQ_WAIT(16 * 50));

  /* Start ADC convert and DFT */

  ad5940_afectrl(priv, AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT, true);

  /* wait for first data ready */

  ad5940_seqgeninsert(priv->pseqgendb, AD5940_SEQ_WAIT(waitclks));

  /* Stop ADC convert and DFT */

  ad5940_afectrl(priv, AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT |
                       AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR, false);
  ad5940_adcmuxcfg(priv, AD5940_ADCMUXP_AIN3, AD5940_ADCMUXN_AIN2);

  /* Enable Waveform generator, ADC power */

  ad5940_afectrl(priv, AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR, true);

  /* Delay for signal settling DFT_WAIT */

  ad5940_seqgeninsert(priv->pseqgendb, AD5940_SEQ_WAIT(16 * 50));

  /* Start ADC convert and DFT */

  ad5940_afectrl(priv, AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT, true);

  /* Wait for first data ready */

  ad5940_seqgeninsert(priv->pseqgendb, AD5940_SEQ_WAIT(waitclks));

  /* Stop ADC convert and DFT */

  ad5940_afectrl(priv, AD5940_AFECTRL_ADCCNV | AD5940_AFECTRL_DFT |
                       AD5940_AFECTRL_WG | AD5940_AFECTRL_ADCPWR, false);

  swcfg.dswitch = AD5940_SWD_OPEN;
  swcfg.pswitch = AD5940_SWP_PL | AD5940_SWP_PL2;
  swcfg.nswitch = AD5940_SWN_NL | AD5940_SWN_NL2;
  swcfg.tswitch = AD5940_SWT_TRTIA;

  /* Float switches */

  ad5940_swmatrixcfg(priv, &swcfg);
  ad5940_writereg(priv, AD5940_REG_AFE_SYNCEXTDEVICE, 0);

  /* Goto hibernate */

  ad5940_entersleep(priv);

  /* Sequence end. */

  ret = ad5940_seqgenfetchseq(priv->pseqgendb, &pseqcmd, &seqlen);
  if (ret < 0)
    {
      return ret;
    }

  /* Stop sequencer generator, ad5940_writereg/readreg will operate SPI. */

  priv->pseqgendb->enginestart = false;

  pbiacfg->measseqcyclecount = ad5940_seqcycletime(priv->pseqgendb);
  pbiacfg->maxodr = 1 / (((pbiacfg->measseqcyclecount + 10) / 16.0)* 1E-6);
  if (pbiacfg->biaodr > pbiacfg->maxodr)
    {
      /* We have requested a sampling rate that cannot be achieved with the
       * time it takes to acquire a sample.
       */

      pbiacfg->biaodr = pbiacfg->maxodr;
    }

  pbiacfg->measureseqinfo.seqid = AD5940_SEQID_0;
  pbiacfg->measureseqinfo.seqramaddr = pbiacfg->initseqinfo.seqramaddr +
                                    pbiacfg->initseqinfo.seqlen ;
  pbiacfg->measureseqinfo.pseqcmd = pseqcmd;
  pbiacfg->measureseqinfo.seqlen = seqlen;

  /* Write command to SRAM */

  return ad5940_seqcmdwrite(priv, pbiacfg->measureseqinfo.seqramaddr,
                            pseqcmd, seqlen);
}

/****************************************************************************
 * Name: ad5940_ad5940_checktable
 *
 * Description:
 *   Check if an uint8_t value exist in table.
 *
 * Input Parameters:
 *   value - The value to be found.
 *   table - Pointer to the check list.
 *   len   - Length of the table
 *   index - Return the index if found.
 *
 * Returned Value:
 *   True for found, false for not found.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static bool ad5940_checktable(uint8_t value, const uint8_t *table,
                              uint32_t len, uint32_t *index)
{
  uint32_t i;

  for (i = 0; i < len; i++)
    {
      if (value == table[i])
        {
          *index = i;
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: ad5940_clkscalculate
 *
 * Description:
 *   Calculate how many clocks are needed in sequencer wait command to
 *   generate required number of data from filter output. When measurement is
 *   done, it's recommend to disable blocks like ADCPWR, ADCCNV, SINC2, DFT
 *   etc. If blocks remain powered up, they may need less clocks to generate
 *   required number of output. Use function ad5940_afectrl to control these
 *   blocks.
 *
 * Input Parameters:
 *   pfilterinfo - Pointer to configuration structure.
 *   pclocks     - Pointer used to store results.
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_clkscalculate(FAR struct ad5940_clkscalinfo_s *pfilterinfo,
                                FAR uint32_t *pclocks)
{
  uint32_t temp = 0;
  const uint32_t sinc2osr_table[13] =
    {
      22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333, 0
    };

  const uint32_t sinc3osr_table[4] =
    {
      5, 4, 2, 0,
    };

  if (pfilterinfo == NULL || pclocks == NULL)
    {
      return -EINVAL;
    }

  *pclocks = 0;
  switch (pfilterinfo->datatype)
  {
    case AD5940_DATATYPE_ADCRAW:
      {
        temp = (uint32_t)(20 * pfilterinfo->datacount *
                          pfilterinfo->ratiosys2adcclk);
      }
      break;

    case AD5940_DATATYPE_SINC3:
      {
        temp = (uint32_t)(((pfilterinfo->datacount + 2) *
                           sinc3osr_table[pfilterinfo->adcsinc3osr] + 1) *
                          20 * pfilterinfo->ratiosys2adcclk + 0.5f);
      }
      break;

    case AD5940_DATATYPE_SINC2:
      {
        temp = (pfilterinfo->datacount + 1) *
               sinc2osr_table[pfilterinfo->adcsinc2osr] + 1;
        pfilterinfo->datatype = AD5940_DATATYPE_SINC3;
        pfilterinfo->datacount = temp;
        ad5940_clkscalculate(pfilterinfo, &temp);
        pfilterinfo->datatype = AD5940_DATATYPE_SINC2;

        /* Need extra 15 clocks for FIFO etc. Just to be safe. */

        temp += 15;
      }
      break;

    case AD5940_DATATYPE_NOTCH:
      {
        FAR struct ad5940_adcfiltercfg_s filter;

        /* dl is the extra data input needed for filter to output 1st data */

        uint8_t dl = 0;
        uint8_t dl_50;
        uint8_t dl_60;

        filter.adcrate = pfilterinfo->adcrate;
        filter.adcsinc3osr = pfilterinfo->adcsinc3osr;
        filter.adcsinc2osr = pfilterinfo->adcsinc2osr;

        if (ad5940_notch50hzavailable(&filter, &dl_50))
          {
            dl += dl_50 - 1;
          }

        if (ad5940_notch60hzavailable(&filter, &dl_60))
          {
            dl += dl_60 - 1;
          }

        pfilterinfo->datatype = AD5940_DATATYPE_SINC2;
        pfilterinfo->datacount += dl;
        ad5940_clkscalculate(pfilterinfo, &temp);

        /* Restore the filter info. */

        pfilterinfo->datatype = AD5940_DATATYPE_NOTCH;
        pfilterinfo->datacount -= dl;
      }
      break;

    case AD5940_DATATYPE_DFT:
      {
        switch (pfilterinfo->dftsrc)
        {
          case AD5940_DFTSRC_ADCRAW:
            {
              pfilterinfo->datatype = AD5940_DATATYPE_ADCRAW;
              ad5940_clkscalculate(pfilterinfo, &temp);
            }
            break;

          case AD5940_DFTSRC_SINC3:
            {
              pfilterinfo->datatype = AD5940_DATATYPE_SINC3;
              ad5940_clkscalculate(pfilterinfo, &temp);
            }
            break;

          case AD5940_DFTSRC_SINC2NOTCH:
            {
              if (pfilterinfo->bpnotch)
                {
                  pfilterinfo->datatype = AD5940_DATATYPE_SINC2;
                }
              else
                {
                  pfilterinfo->datatype = AD5940_DATATYPE_NOTCH;
                }

              ad5940_clkscalculate(pfilterinfo, &temp);
            }
            break;

          case AD5940_DFTSRC_AVG:
            {
              pfilterinfo->datatype = AD5940_DATATYPE_SINC3;
              pfilterinfo->datacount = pfilterinfo->datacount *
                                       (1L << (pfilterinfo->adcavgnum + 1));
              ad5940_clkscalculate(pfilterinfo, &temp);
            }
            break;

          default:
            break;
        }

        pfilterinfo->datatype = AD5940_DATATYPE_DFT;

         /* Add margin */

        temp += 25;
      }
      break;

    default:
      {
      }
      break;
  }

  *pclocks = temp;
  return OK;
}

/****************************************************************************
 * Name: ad5940_complexdivint
 *
 * Description:
 *   Do complex integer number division, return a float result.
 *
 * Input Parameters:
 *   a - The dividend (integer)
 *   b - The divisor (integer)
 *
 * Returned Value:
 *   Result (float).
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static FAR struct ad5940_fimpcar_s ad5940_complexdivint(
                                        FAR const struct ad5940_iimpcar_s *a,
                                        FAR const struct ad5940_iimpcar_s *b)
{
  FAR struct ad5940_fimpcar_s res;
  float temp;

  temp = (float)b->real * b->real + (float)b->image * b->image;
  res.real = (float)a->real * b->real + (float)a->image * b->image;
  res.real = res.real / temp;
  res.image = (float)a->image * b->real - (float)a->real * b->image;
  res.image = res.image / temp;
  return res;
}

/****************************************************************************
 * Name: ad5940_complexmag
 *
 * Description:
 *   Calculate the complex float number magnitude.
 *
 * Input Parameters:
 *   a - The complex float number
 *
 * Returned Value:
 *   Magnitude (float).
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static float ad5940_complexmag(FAR const struct ad5940_fimpcar_s *a)
{
  return sqrt(a->real * a->real + a->image * a->image);
}

/****************************************************************************
 * Name: ad5940_complexmag
 *
 * Description:
 *   Calculate the complex float number phase.
 *
 * Input Parameters:
 *   a - The complex float number
 *
 * Returned Value:
 *   Phase (float).
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static float ad5940_complexphase(FAR const struct ad5940_fimpcar_s *a)
{
  return atan2(a->image, a->real);
}

/****************************************************************************
 * Name: ad5940_notch50hzavailable
 *
 * Description:
 *   Return if the SINC3/SINC2 combination is available for notch 50Hz
 *   filter. If it's not availabe, hardware automatically bypass Notch even
 *   if it's enabled.
 *
 * Input Parameters:
 *   pfilterinfo - The filter configuration, only need sinc2/sinc3 osr and
 *                 adc data rate information.
 *   dl          - Result
 *
 * Returned Value:
 *   True if notch 50Hz filter is available, otherwise return False.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static bool ad5940_notch50hzavailable(FAR const struct ad5940_adcfiltercfg_s
                                      *pfilterinfo, uint8_t *dl)
{
  uint32_t index;

  if ((pfilterinfo->adcrate == AD5940_ADCRATE_800KHZ &&
       pfilterinfo->adcsinc3osr == AD5940_ADCSINC3OSR_2) ||
      (pfilterinfo->adcrate == AD5940_ADCRATE_1P6MHZ &&
       pfilterinfo->adcsinc3osr != AD5940_ADCSINC3OSR_2))
    {
      /* This combination suits for filter: SINC3 OSR2, for 800kSPS;  SINC3
       * OSR4 and OSR5 for 1.6MSPS
       */

      const uint8_t available_sinc2_osr[5] =
        {
          AD5940_ADCSINC2OSR_533, AD5940_ADCSINC2OSR_667,
          AD5940_ADCSINC2OSR_800, AD5940_ADCSINC2OSR_889,
          AD5940_ADCSINC2OSR_1333
        };

      const uint8_t dl_50hz[5] =
        {
          15, 12, 10, 9, 6,
        };

      if (ad5940_checktable(pfilterinfo->adcsinc2osr, available_sinc2_osr,
                            sizeof(available_sinc2_osr), &index))
        {
          *dl = dl_50hz[index];
          return true;
        }
    }
  else if (pfilterinfo->adcrate == AD5940_ADCRATE_1P6MHZ &&
           pfilterinfo->adcsinc3osr == AD5940_ADCSINC3OSR_2)
    {
      /* This combination suits for filter: SINC3 OSR2 for 1.6MSPS */

      const uint8_t available_sinc2_osr[4] =
        {
          AD5940_ADCSINC2OSR_889, AD5940_ADCSINC2OSR_1067,
          AD5940_ADCSINC2OSR_1333,
        };

      const uint8_t dl_50hz[3] =
        {
          18, 15, 12,
        };

      if (ad5940_checktable(pfilterinfo->adcsinc2osr, available_sinc2_osr,
                            sizeof(available_sinc2_osr), &index))
        {
          *dl = dl_50hz[index];
          return true;
        }
    }
  else if (pfilterinfo->adcrate == AD5940_ADCRATE_800KHZ &&
           pfilterinfo->adcsinc3osr != AD5940_ADCSINC3OSR_2)
    {
      /* This combination suits for filter: SINC3 OSR4 and OSR5 for 800kSPS */

      const uint8_t available_sinc2_osr[6] =
        {
          AD5940_ADCSINC2OSR_178, AD5940_ADCSINC2OSR_267,
          AD5940_ADCSINC2OSR_533, AD5940_ADCSINC2OSR_640,
          AD5940_ADCSINC2OSR_800, AD5940_ADCSINC2OSR_1067,
        };

      const uint8_t dl_50hz[6] =
        {
          18, 12, 6, 5, 4, 3,
        };

      if (ad5940_checktable(pfilterinfo->adcsinc2osr, available_sinc2_osr,
                            sizeof(available_sinc2_osr), &index))
        {
          *dl = dl_50hz[index];
          return true;
        }
    }

  *dl = 0;
  return false;
}

/****************************************************************************
 * Name: ad5940_notch60hzavailable
 *
 * Description:
 *   Return if the SINC3/SINC2 combination is available for notch 60Hz
 *   filter. If it's not availabe, hardware automatically bypass Notch even
 *   if it's enabled.
 *
 * Input Parameters:
 *   pfilterinfo - The filter configuration, only need sinc2/sinc3 osr and
 *                 adc data rate information.
 *   dl          - Result
 *
 * Returned Value:
 *   True if notch 60Hz filter is available, otherwise return false.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static bool ad5940_notch60hzavailable(FAR const struct ad5940_adcfiltercfg_s
                                      *pfilterinfo, uint8_t *dl)
{
  uint32_t index;

  if ((pfilterinfo->adcrate == AD5940_ADCRATE_800KHZ &&
       pfilterinfo->adcsinc3osr == AD5940_ADCSINC3OSR_2) ||
      (pfilterinfo->adcrate == AD5940_ADCRATE_1P6MHZ &&
       pfilterinfo->adcsinc3osr != AD5940_ADCSINC3OSR_2))
    {
      /* This combination suits for filter: SINC3 OSR2, for 800kSPS; SINC3
       * OSR4 and OSR5 for 1.6MSPS
       */

      const uint8_t available_sinc2_osr[2] =
        {
          AD5940_ADCSINC2OSR_667, AD5940_ADCSINC2OSR_1333,
        };

      const uint8_t dl_60hz[2] =
        {
          10, 5,
        };

      if (ad5940_checktable(pfilterinfo->adcsinc2osr, available_sinc2_osr,
                            sizeof(available_sinc2_osr), &index))
        {
          *dl = dl_60hz[index];
          return true;
        }
    }
  else if (pfilterinfo->adcrate == AD5940_ADCRATE_1P6MHZ &&
           pfilterinfo->adcsinc3osr == AD5940_ADCSINC3OSR_2)
    {
      /* This combination suits for filter: SINC3 OSR2 for 1.6MSPS */

      const uint8_t available_sinc2_osr[2] =
        {
          AD5940_ADCSINC2OSR_889, AD5940_ADCSINC2OSR_1333,
        };

      const uint8_t dl_60hz[2] =
        {
          15, 10,
        };

      if (ad5940_checktable(pfilterinfo->adcsinc2osr, available_sinc2_osr,
                            sizeof(available_sinc2_osr), &index))
        {
          *dl = dl_60hz[index];
          return true;
        }
    }
  else if (pfilterinfo->adcrate == AD5940_ADCRATE_800KHZ &&
           pfilterinfo->adcsinc3osr != AD5940_ADCSINC3OSR_2)
    {
      /* This combination suits for filter: SINC3 OSR4 and OSR5 for 800kSPS */

      const uint8_t available_sinc2_osr[6] =
        {
          AD5940_ADCSINC2OSR_178, AD5940_ADCSINC2OSR_267,
          AD5940_ADCSINC2OSR_533, AD5940_ADCSINC2OSR_667,
          AD5940_ADCSINC2OSR_889, AD5940_ADCSINC2OSR_1333,
        };

      const uint8_t dl_60hz[6] =
        {
          15, 10, 5, 4, 3, 2,
        };

      if (ad5940_checktable(pfilterinfo->adcsinc2osr, available_sinc2_osr,
                            sizeof(available_sinc2_osr), &index))
        {
          *dl = dl_60hz[index];
          return true;
        }
    }

  *dl = 0;
  return false;
}

/****************************************************************************
 * Name: ad5940_seqcycletime
 *
 * Description:
 *   Calculate the number of cycles in the sequence.
 *
 * Input Parameters:
 *   pseqdb - Pointer to sequency generator data base.
 *
 * Returned Value:
 *   Return Number of ACLK Cycles that a generated sequence will take.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static uint32_t ad5940_seqcycletime(FAR struct ad5940_seqgen_s *pseqdb)
{
  uint32_t cycles = 0;
  uint32_t cmd;
  uint32_t i;

  for (i = 0; i < pseqdb->regcount; i++)
    {
      cmd = (pseqdb->pseqbuff[i]  >> 30) & 0x3;
      if (cmd & 0x2)
        {
          /* A write command */

          cycles = cycles + 1;
        }
      else
        {
          if (cmd & 0x1)
            {
              /* Timeout Command */

              cycles = cycles + 1;
            }
          else
            {
              /* Wait command */

              cycles = cycles + (pseqdb->pseqbuff[i] & 0x3fffffff);
            }
        }
    }

  return cycles;
}

/****************************************************************************
 * Name: ad5940_seqgenfetchseq
 *
 * Description:
 *   Get sequencer command generated.
 *
 * Input Parameters:
 *   pseqdb   - Pointer to sequency generator data base.
 *   ppseqcmd - Pointer to a variable(pointer) used to store the pointer to
 *              generated sequencer command.
 *   pseqlen  - Pointer to a variable that used to store how many commands
 *              available in buffer.
 *
 * Returned Value:
 *   Return lasterror in sequence generator data base.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_seqgenfetchseq(FAR struct ad5940_seqgen_s *pseqdb,
                                 FAR const uint32_t **ppseqcmd,
                                 FAR uint32_t *pseqlen)
{
  if (ppseqcmd)
    {
      *ppseqcmd = pseqdb->pseqbuff;
    }

  if (pseqlen)
    {
      *pseqlen = pseqdb->seqlen;
    }

  return pseqdb->lasterror;
}

/****************************************************************************
 * Name: ad5940_seqgenstart
 *
 * Description:
 *   Start the sequencer generator. Once started, the register write function
 *   will be recorded to sequencer generator.
 *
 * Input Parameters:
 *   pseqdb - Pointer to sequency generator data base.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_seqgenstart(FAR struct ad5940_seqgen_s *pseqdb)
{
  pseqdb->seqlen = 0;
  pseqdb->lasterror = 0;
  pseqdb->enginestart = true;
}

/****************************************************************************
 * Name: ad5940_sweepnext
 *
 * Description:
 *   For sweep function, calculate next frequency point according to pcfg.
 *
 * Input Parameters:
 *   sinfreqhz - Target frequency in Hz unit.
 *   wgclock   - Waveform generator clock frequency in Hz unit. The clock is
 *               sourced from system clock, default value is 16MHz HFOSC.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ad5940_sweepnext(FAR struct ad5940_softsweepcfg_s *pcfg,
                             float *pnextfreq)
{
  float frequency;

  /* Log step */

  if (pcfg->sweeplog)
    {
      if (pcfg->sweepstart < pcfg->sweepstop)
        {
          pcfg->sweepindex++;
          if (pcfg->sweepindex == pcfg->sweeppoints)
            {
              pcfg->sweepindex = 0;
            }

          frequency = pcfg->sweepstart *
                      pow(10, pcfg->sweepindex *
                              log10(pcfg->sweepstop / pcfg->sweepstart) /
                              (pcfg->sweeppoints - 1));
        }
      else
        {
          pcfg->sweepindex--;
          if (pcfg->sweepindex >= pcfg->sweeppoints)
            {
              pcfg->sweepindex = pcfg->sweeppoints - 1;
            }

          frequency = pcfg->sweepstop *
                      pow(10, pcfg->sweepindex *
                              log10(pcfg->sweepstart / pcfg->sweepstop) /
                              (pcfg->sweeppoints - 1));
        }
    }

  /* Linear step */

  else
    {
      if (pcfg->sweepstart < pcfg->sweepstop)
        {
          pcfg->sweepindex++;
          if (pcfg->sweepindex == pcfg->sweeppoints)
            {
              pcfg->sweepindex = 0;
            }

          frequency = pcfg->sweepstart +
                      pcfg->sweepindex *
                      (double)(pcfg->sweepstop - pcfg->sweepstart) /
                      (pcfg->sweeppoints - 1);
        }
      else
        {
          pcfg->sweepindex--;
          if (pcfg->sweepindex >= pcfg->sweeppoints)
            {
              pcfg->sweepindex = pcfg->sweeppoints - 1;
            }

          frequency = pcfg->sweepstop +
                      pcfg->sweepindex *
                      (double)(pcfg->sweepstart - pcfg->sweepstop) /
                      (pcfg->sweeppoints - 1);
        }
    }

  *pnextfreq = frequency;
}

/****************************************************************************
 * Name: ad5940_wgfreqwordcal
 *
 * Description:
 *   Calculate sine wave generator frequency word. The maxim frequency is
 *   250kHz-1LSB
 *
 * Input Parameters:
 *   sinfreqhz - Target frequency in Hz unit.
 *   wgclock   - Waveform generator clock frequency in Hz unit. The clock is
 *               sourced from system clock, default value is 16MHz HFOSC.
 *
 * Returned Value:
 *   Sine wave generator frequency word.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static uint32_t ad5940_wgfreqwordcal(float sinfreqhz, float wgclock)
{
  uint32_t temp;

  if (wgclock == 0)
    {
      return 0;
    }

  temp = (uint32_t)(sinfreqhz * (1LL << AD5940_BITWIDTH_WGFCW) / wgclock
                    + 0.5f);
  if (temp > 0xffffff)
    {
      temp = 0xffffff;
    }

  return temp;
}

/****************************************************************************
 * Name: ad5940_activate
 *
 * Description:
 *   Enable or disable sensor device. When enable sensor, initialilize the
 *   AFE.
 *
 * Input Parameters:
 *   lower  - The instance of lower half sensor driver.
 *   filep  - The pointer of file, represents each user using the sensor.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           bool enable)
{
  FAR struct ad5940_dev_s *priv = (FAR struct ad5940_dev_s *)lower;
  FAR struct ad5940_biacfg_s *pbiacfg;

  DEBUGASSERT(lower != NULL);

  /* Get pointer of BIA configure struct in priv, set the struct as default */

  pbiacfg = &priv->biacfg;

  if (enable == priv->activated)
    {
      /* If AD5940 has been in the state required, do nothing and return OK */

      return OK;
    }

  if (enable == true)
    {
      /* Intialize device for BIA */

      ad5940_biaplatformcfg(priv);

      /* Configure BIA parameters and attach the BIA configure struct pointer
       * to the device struct.
       */

      ad5940_biacfgstructinit(priv, pbiacfg);

      /* BIA initialization contains wake-up, calibration if it hasn't been
       * calibrated before, and preparation for measurement.
       */

      ad5940_biafuncinit(priv, pbiacfg, AD5940_BIA_BUFFSIZE);

      /* If FIFO is used (FIFO watermark > 0), set the interrupt pin on MCU
       * for receiving AD5940's interrupt. AD5940's GPIO have been configured
       * as active-high. Otherwise start a worker task to poll the data.
       */

      if (priv->fifowtm > 0)
        {
          IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                          IOEXPANDER_OPTION_INTCFG,
                          (FAR void *)IOEXPANDER_VAL_RISING);
        }
      else
        {
          work_queue(HPWORK, &priv->work_poll, ad5940_worker_poll, priv,
                     priv->interval / USEC_PER_TICK);
        }

      /* Control BIA measurement to start. The last parameter has no meaning
       * in this suitation.
       */

      ad5940_biactrl(priv, pbiacfg, AD5940_BIACTRL_START, NULL);
      priv->activated = true;
    }
  else
    {
      /* Stop BIA measurements and the interrupt (FIFO is used) or the worker
       * task (FIFO is not used).
       */

      ad5940_biactrl(priv, pbiacfg, AD5940_BIACTRL_SHUTDOWN, NULL);
      priv->activated = false;
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
    }

  return OK;
}

/****************************************************************************
 * Name: ad5940_set_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us > max_delay it will be truncated to max_delay and if
 *   *period_us < min_delay it will be replaced by min_delay.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   filep      - The pointer of file, represents each user using the sensor.
 *   period_us  - The time between report data, in us. It may by overwrite
 *                by lower half driver.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ad5940_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us)
{
  FAR struct ad5940_dev_s * priv = (FAR struct ad5940_dev_s *)lower;
  float freq;

  /* Sanity check */

  DEBUGASSERT(priv != NULL && period_us != NULL);

  /* Desired ODR can't exist the max ODR */

  freq = AD5940_ONE_SECOND / *period_us;
  if (freq > AD5940_BIAODR_MAX)
    {
      freq = AD5940_BIAODR_MAX;
    }

  /* Calculate the read BIA ODR, where:
   * "(uint32_t)(priv->biacfg.wuptclkfreq / freq)" means the count for the
   * wake-up/sleep timer, i.e. the measurement period in unit WUPTCLKs;
   * "AD5940_ONE_SECOND / priv->biacfg.wuptclkfreq" means the period of
   * WUPTCLK in unit us.
   */

  *period_us = (priv->biacfg.wuptclkfreq / freq) *
               AD5940_ONE_SECOND / priv->biacfg.wuptclkfreq;
  priv->interval = *period_us;
  priv->odr = AD5940_ONE_SECOND / (float)priv->interval;

  /* If ad5940_interval() is called before activating device, new ODR will be
   * written to AD5940 once it's activated.
   * If ad5940_interval() is called during device has been activated, new ODR
   * will be written to AD5940 in ad5940_biaregmodify() when next interrupt
   * or polling comes.
   */

  return OK;
}

/****************************************************************************
 * Name: ad5940_batch
 *
 * Description:
 *   Set sensor's maximum report latency in microseconds.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   filep      - The pointer of file, represents each user using the sensor.
 *   latency_us - the time between batch data, in us. It may by overwrite
 *                by lower half driver.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int  ad5940_batch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR unsigned long *latency_us)
{
  FAR struct ad5940_dev_s *priv = (FAR struct ad5940_dev_s *)lower;
  unsigned long max_latency;

  /* Sanity check */

  DEBUGASSERT(priv != NULL && latency_us != NULL);

  if (*latency_us == 0)
    {
      /* If AFE has been activated with batch mode, change the flag and thus
       * the AFE will switch into polling mode when next interrupt comes.
       */

      if (priv->activated && priv->fifowtm > 0)
        {
          priv->batch2poll = true;
        }

      priv->fifowtm = 0;
    }
  else
    {
      max_latency = lower->nbuffer * priv->interval;
      if (*latency_us > max_latency)
        {
          *latency_us = max_latency;
        }
      else if (*latency_us < priv->interval)
        {
          *latency_us = priv->interval;
        }

      /* If AFE has been activated with polling, change the flag and thus the
       * AFE will switch into batch mode when next polling comes.
       */

      if (priv->activated && priv->fifowtm == 0)
        {
          priv->poll2batch = true;
        }

      priv->fifowtm = AD5940_CEILING(*latency_us, priv->interval);
      *latency_us = priv->fifowtm * priv->interval;
      priv->batch_latency = *latency_us;
    }

  /* If ad5940_batch is called before activating device, FIFO watermark will
   * be written to AD5940 once it's activated.
   * If ad5940_batch is called during device has been activated, new FIFO
   * watermark will be written to AD5940 in ad5940_biaregmodify() when next
   * interrupt or polling comes.
   */

  return OK;
}

/****************************************************************************
 * Name: ad5940_selftest
 *
 * Description:
 *   AD5940 has no specailized self-testing function. Currently only
 *   checking device ID is supported.
 *
 * Input Parameters:
 *   lower - The instance of lower half sensor driver.
 *   filep - The pointer of file, represents each user using the sensor.
 *   arg   - The parameters associated with cmd.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on any failure.
 *   -ENOTTY - The cmd don't support.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int  ad5940_selftest(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            unsigned long arg)
{
  FAR struct ad5940_dev_s *priv = (FAR struct ad5940_dev_s *)lower;
  int ret;

  DEBUGASSERT(lower != NULL);

  switch (arg)
    {
      case AD5940_CTRL_CHECKID:             /* Check ID command. */
        {
          ret = ad5940_wakeup(priv, 10);
          if (ret == OK)
            {
              ret = ad5940_checkid(priv);
            }

          /* After wake-up, it should shut down to save power. */

          ad5940_shutdown(priv);
          return ret;
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
 * Name: ad5940_interrupt_handler
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
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ad5940_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the AD5940 INTx
   * pin, when an event, such as FIFO is almost full, has occured.
   */

  FAR struct ad5940_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp */

  priv->timestamp = sensor_get_timestamp();

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long, and neither should we
   * lock the SPI bus within an interrupt.
   */

  work_queue(HPWORK, &priv->work_intrpt, ad5940_worker_intrpt, priv, 0);
  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);

  return OK;
}

/****************************************************************************
 * Name: ad5940_worker_intrpt
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

static void ad5940_worker_intrpt(FAR void *arg)
{
  FAR struct ad5940_dev_s *priv = arg;
  uint32_t fifocnt;
  uint32_t intcflg;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_RISING);

  /* Wakeup AFE by read register, read 10 times at most */

  if (ad5940_wakeup(priv, 10) != OK)
    {
      return;
    }

  /* Don't enter hibernate */

  ad5940_sleeplock(priv, true);

  /* Interrupt: data FIFO reached watermark. */

  ad5940_readreg(priv, AD5940_REG_INTC_INTCFLAG0, &intcflg);
  if ((intcflg & AD5940_AFEINTSRC_DATAFIFOTHRESH) != 0)
    {
      /* A measurement contains 4(AD5940_RESULTS_PER_MEAS) data */

      fifocnt = ad5940_fifogetcnt(priv) / AD5940_RESULTS_PER_MEAS *
                AD5940_RESULTS_PER_MEAS;

      if (fifocnt == 0)
        {
          snerr("No data in FIFO.\n");
          goto exit;
        }

      if (fifocnt > (AD5940_FIFOSLOTS_MAX * AD5940_RESULTS_PER_MEAS))
        {
          fifocnt = AD5940_FIFOSLOTS_MAX * AD5940_RESULTS_PER_MEAS;
        }

      /* Read data out and clear the interrupt flag. */

      ad5940_readfifo(priv, priv->fifobuf, fifocnt);
      ad5940_intrptclr(priv, AD5940_AFEINTSRC_DATAFIFOTHRESH);

      /* If it's needed to do AFE re-configure, do it here when AFE is in
       * active state.
       */

      ad5940_biaregmodify(priv, &priv->biacfg);

      /* Process data and push the results. */

      ad5940_biadataprocess(priv, priv->fifobuf, fifocnt);
    }

  /* Procession for the other types of interrupt will be added later. */

exit:

  /* If the device will go on measuring, unlock sleep key to allow AFE to
   * enter hibernate mode. Otherwise, never operate the SPI, or the AFE will
   * be waked up again.
   */

  if (priv->activated == true)
    {
      ad5940_sleeplock(priv, false);
    }
}

/****************************************************************************
 * Name: ad5940_worker
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

static void ad5940_worker_poll(FAR void *arg)
{
  FAR struct ad5940_dev_s *priv = arg;
  uint32_t fifocnt;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  priv->timestamp = sensor_get_timestamp();
  work_queue(HPWORK, &priv->work_poll, ad5940_worker_poll,
             priv, priv->interval / USEC_PER_TICK);

  /* Wakeup AFE by read register, read 10 times at most */

  if (ad5940_wakeup(priv, 10) != OK)
    {
      return;
    }

  /* Don't enter hibernate */

  ad5940_sleeplock(priv, true);

  /* A measurement contains 4(AD5940_RESULTS_PER_MEAS) data */

  fifocnt = ad5940_fifogetcnt(priv) / AD5940_RESULTS_PER_MEAS *
            AD5940_RESULTS_PER_MEAS;

  if (fifocnt >= AD5940_RESULTS_PER_MEAS)
    {
      if (fifocnt > (AD5940_FIFOSLOTS_MAX * AD5940_RESULTS_PER_MEAS))
        {
          fifocnt = AD5940_FIFOSLOTS_MAX * AD5940_RESULTS_PER_MEAS;
        }

      /* Read data out. */

      ad5940_readfifo(priv, priv->fifobuf, fifocnt);

      /* If it's needed to do AFE re-configure, do it here when AFE is in
       * active state.
       */

      ad5940_biaregmodify(priv, &priv->biacfg);

      /* Process data and push the results. */

      ad5940_biadataprocess(priv, priv->fifobuf, fifocnt);
    }

  /* If the device will go on measuring, unlock sleep key to allow AFE to
   * enter hibernate mode. Otherwise, never operate the SPI, or the AFE will
   * be waked up again.
   */

  if (priv->activated == true)
    {
      ad5940_sleeplock(priv, false);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ad5940_register
 *
 * Description:
 *   Register the AD5940 character device.
 *
 * Input Parameters:
 *   devno  - The full path to the driver to register. E.g., "/dev/ad5940"
 *   config - An instance of the SPI interface to use to communicate with
 *            AD5940.
 *
 * Returned Value:
 *   OK(0) for success; a negated errno on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int ad5940_register(int devno, FAR const struct ad5940_config_s *config)
{
  FAR struct ad5940_dev_s *priv;
  FAR void *ioehandle;
  int ret;

  /* Sanity check */

  DEBUGASSERT(config != NULL);

  /* Initialize the AD5940 device structure and BIA configure structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (!priv)
    {
      snerr("Failed to allocate instance: %d.\n", -ENOMEM);
      return -ENOMEM;
    }

  /* Set default priv initial values */

  memcpy(&priv->biacfg, &ad5940_biacfgdefault,
         sizeof(const struct ad5940_biacfg_s));
  priv->config = config;
  priv->lower.ops = &g_ad5940_ops;
  priv->lower.type = SENSOR_TYPE_IMPEDANCE;
  priv->lower.nbuffer = AD5940_FIFOSLOTS_MAX;
  priv->interval = AD5940_INTVL_DEFAULT;
  priv->odr = AD5940_BIAODR_DEFAULT;
  priv->fifowtm = AD5940_FIFOWTM_DEFAULT;

  /* Check the part ID */

  ret = ad5940_checkid(priv);
  if (ret < 0)
    {
      snerr("AD5940 ID doesn't match: %d\n", ret);
      goto err_exit;
    }

  ad5940_init(priv);

  /* Enter shutdown mode */

  ad5940_shutdown(priv);

  /* Set GPIO on MCU for AD5940 interrupt */

  ret = IOEXP_SETDIRECTION(priv->config->ioedev, priv->config->intpin,
                           IOEXPANDER_DIRECTION_IN);
  if (ret < 0)
    {
      snerr("Failed to set GPIO direction: %d\n", ret);
      goto err_exit;
    }

  ioehandle = IOEP_ATTACH(priv->config->ioedev, priv->config->intpin,
                          ad5940_interrupt_handler, priv);
  if (ioehandle == NULL)
    {
      snerr("Failed to GPIO attach: %d\n", ret);
      ret = -EIO;
      goto err_exit;
    }

  ret = IOEXP_SETOPTION(priv->config->ioedev, priv->config->intpin,
                        IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);
  if (ret < 0)
    {
      snerr("Failed to set GPIO option: %d\n", ret);
      goto err_iodetach;
    }

  /* Register the character driver */

  ret = sensor_register((&(priv->lower)), devno);
  if (ret < 0)
    {
      snerr("Failed to register imdepance sensor driver: %d\n", ret);
      goto err_iodetach;
    }

  return ret;

err_iodetach:
  IOEP_DETACH(priv->config->ioedev, ad5940_interrupt_handler);

err_exit:
  kmm_free(priv);
  return ret;
}
