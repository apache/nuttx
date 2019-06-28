/****************************************************************************
 * drivers/sensors/vl53l1x.c
 * Character driver for the st vl53l1x distance and brigh sensor.
 *
 *   Copyright (C) 2019 Acutronics Robotics. All rights reserved.
 *   Author: Acutronics Robotics (Juan Flores Muñoz) <juan@erlerobotics.com>
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

/****************************************************************************
 * Included files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <fixedmath.h>
#include <stdlib.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/random.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/vl53l1x.h>
#include <nuttx/signal.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_VL53L1X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VLM53L1X_FREQ 100000
#define VL53L1X_ADDR  0x29

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct vl53l1x_dev_s
{
  FAR struct i2c_master_s *i2c; /* i2c interface */
  uint8_t addr;                 /* vl53l0x i2c address */
  int freq;                     /* vl53l0x frequency */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_vl51l1x_default_configuration[] =
{
  0x00,
  0x00,
  0x00,
  0x01,
  0x02,
  0x00,
  0x02,
  0x08,
  0x00,
  0x08,
  0x10,
  0x01,
  0x01,
  0x00,
  0x00,
  0x00,
  0x00,
  0xff,
  0x00,
  0x0f,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x20,
  0x0b,
  0x00,
  0x00,
  0x02,
  0x0a,
  0x21,
  0x00,
  0x00,
  0x05,
  0x00,
  0x00,
  0x00,
  0x00,
  0xc8,
  0x00,
  0x00,
  0x38,
  0xff,
  0x01,
  0x00,
  0x08,
  0x00,
  0x00,
  0x01,
  0xdb,
  0x0f,
  0x01,
  0xf1,
  0x0d,
  0x01,
  0x68,
  0x00,
  0x80,
  0x08,
  0xb8,
  0x00,
  0x00,
  0x00,
  0x00,
  0x0f,
  0x89,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x01,
  0x0f,
  0x0d,
  0x0e,
  0x0e,
  0x00,
  0x00,
  0x02,
  0xc7,
  0xff,
  0x9b,
  0x00,
  0x00,
  0x00,
  0x01,
  0x00,
  0x00
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t vl53l1x_getreg8(FAR struct vl53l1x_dev_s *priv,
                               uint16_t regaddr);
static uint16_t vl53l1x_getreg16(FAR struct vl53l1x_dev_s *priv,
                                 uint16_t regaddr);
static uint32_t vl53l1x_getreg32(FAR struct vl53l1x_dev_s *priv,
                                 uint16_t regaddr);
static void vl53l1x_putreg8(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                            uint8_t regval);
static void vl53l1x_putreg16(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                             uint16_t regval);
static void vl53l1x_putreg32(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                             uint32_t regval);
static void vl53l1x_seti2caddress(uint8_t new_address);
static void vl53l1x_sensorinit(FAR struct vl53l1x_dev_s *priv);
static void vl53l1x_clearinterrupt(FAR struct vl53l1x_dev_s *priv);
static void vl53l1x_setinterruptpolarity(FAR struct vl53l1x_dev_s *priv,
                                         uint8_t intpol);
static void vl53l1x_getinterruptpolarity(FAR uint8_t *pintpol);
static void vl53l1x_startranging(FAR struct vl53l1x_dev_s *priv);
static void vl53l1x_stopranging(FAR struct vl53l1x_dev_s *priv);
static void vl53l1x_checkfordataready(FAR struct vl53l1x_dev_s *priv,
                                      FAR uint8_t *isdataready);
static void vl53l1x_settimingbudgetinms(FAR struct vl53l1x_dev_s *priv,
                                        uint16_t timingbudgetinms);
static void vl53l1x_gettimingbudgetinms(FAR struct vl53l1x_dev_s *priv,
                                        FAR uint16_t *ptimingbudgetinms);
static void vl53l1x_setdistancemode(FAR struct vl53l1x_dev_s *priv,
                                    uint16_t distancemode);
static void vl53l1x_getdistancemode(FAR struct vl53l1x_dev_s *priv,
                                    FAR uint16_t *pdistancemode);
static void vl53l1x_setintermeasurementinms(FAR struct vl53l1x_dev_s *priv,
                                            uint16_t intermeasurementinms);
static void vl53l1x_getintermeasurementinms(FAR struct vl53l1x_dev_s *priv,
                                            FAR uint16_t *pim);
static void vl53l1x_bootstate(FAR struct vl53l1x_dev_s *priv,
                              FAR uint8_t *state);
static void vl53l1x_getsensorid(FAR struct vl53l1x_dev_s *priv,
                                FAR uint16_t *id);
static void vl53l1x_getdistance(FAR struct vl53l1x_dev_s *priv,
                                FAR uint16_t *distance);
static void vl53l1x_getsignalperspad(FAR struct vl53l1x_dev_s *priv,
                                     FAR uint16_t *signalpersp);
static void vl53l1x_getambientperspad(FAR struct vl53l1x_dev_s *priv,
                                      FAR uint16_t *amb);
static void vl53l1x_getsignalrate(FAR struct vl53l1x_dev_s *priv,
                                  FAR uint16_t *signalrate);
static void vl53l1x_getspadnb(FAR struct vl53l1x_dev_s *priv,
                              FAR uint16_t *spnb);
static void vl53l1x_getambientrate(FAR struct vl53l1x_dev_s *priv,
                                   FAR uint16_t *ambrate);
static void vl53l1x_getrangestatus(FAR struct vl53l1x_dev_s *priv,
                                   FAR uint8_t *rangestatus);
static void vl53l1x_setffset(FAR struct vl53l1x_dev_s *priv,
                             int16_t offsetvalue);
static void vl53l1x_getoffset(FAR struct vl53l1x_dev_s *priv,
                              FAR int16_t *offset);
static void vl53l1x_setxtalk(FAR struct vl53l1x_dev_s *priv,
                             uint16_t xtalkvalue);
static void vl53l1x_getxtalk(FAR struct vl53l1x_dev_s *priv,
                             FAR uint16_t *xtalk);
static void vl53l1x_setdistancethreshold(FAR struct vl53l1x_dev_s *priv,
                                         uint16_t threshlow,
                                         uint16_t threshhigh, uint8_t window,
                                         uint8_t intonnotarget);
static void vl53l1x_getdistancethresholdwindow(FAR struct vl53l1x_dev_s *priv,
                                               FAR uint16_t *window);
static void vl53l1x_getdistancethresholdlow(FAR struct vl53l1x_dev_s *priv,
                                            FAR uint16_t *low);
static void vl53l1x_getdistancethresholdhigh(FAR struct vl53l1x_dev_s *priv,
                                             FAR uint16_t *high);
static void vl53l1x_setroi(FAR struct vl53l1x_dev_s *priv, uint16_t x,
                           uint16_t y);
static void vl53l1x_getroi_xy(FAR struct vl53l1x_dev_s *priv,
                              FAR uint16_t *roi_x, FAR uint16_t *roi_y);
static void vl53l1x_setsignalthreshold(FAR struct vl53l1x_dev_s *priv,
                                       uint16_t signal);
static void vl53l1x_getsignalthreshold(FAR struct vl53l1x_dev_s *priv,
                                       FAR uint16_t *signal);
static void vl53l1x_setsigmathreshold(FAR struct vl53l1x_dev_s *priv,
                                      uint16_t sigma);
static void vl53l1x_getsigmathreshold(FAR struct vl53l1x_dev_s *priv,
                                      FAR uint16_t *signal);
static void vl53l1x_starttemperatureupdate(FAR struct vl53l1x_dev_s *priv);
static void vl53l1x_calibrateoffset(FAR struct vl53l1x_dev_s *priv,
                                    uint16_t targetdistinmm,
                                    FAR int16_t *offset);
static void vl53l1x_calibratextalk(FAR struct vl53l1x_dev_s *priv,
                                   uint16_t targetdistinmm,
                                   FAR uint16_t *xtalk);

/* Character driver methods */

static int vl53l1x_open(FAR struct file *filep);
static int vl53l1x_close(FAR struct file *filep);
static ssize_t vl53l1x_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t vl53l1x_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static ssize_t vl53l1x_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_vl53l1xfops =
{
  vl53l1x_open,                 /* open */
  vl53l1x_close,                /* close */
  vl53l1x_read,                 /* read */
  vl53l1x_write,                /* write */
  NULL,                         /* seek */
  vl53l1x_ioctl,                /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  NULL,                         /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL,                         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vl53l1x_sensorinit
 *
 * Description:
 *   This function loads the 135 bytes default values to initialize the
 *   sensor.
 *
 ****************************************************************************/

static void vl53l1x_sensorinit(FAR struct vl53l1x_dev_s *priv)
{
  uint8_t addr = 0x00;
  uint8_t tmp = 0;

  for (addr = 0x2d; addr <= 0x87; addr++)
    {
      vl53l1x_putreg8(priv, addr, g_vl51l1x_default_configuration[addr - 0x2d]);
    }

  vl53l1x_startranging(priv);
  while (tmp == 0)
    {
      vl53l1x_checkfordataready(priv, &tmp);
    }

  tmp = 0;
  vl53l1x_clearinterrupt(priv);
  vl53l1x_stopranging(priv);
  vl53l1x_putreg8(priv, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);

  vl53l1x_putreg8(priv, 0x0b, 0);
}

/****************************************************************************
 * Name: vl53l1x_clearinterrupt
 *
 * Description:
 *   This function clears the interrupt, to be called after a ranging data
 *   reading to arm the interrupt for the next data ready event.
 *
 ****************************************************************************/

static void vl53l1x_clearinterrupt(FAR struct vl53l1x_dev_s *priv)
{
  vl53l1x_putreg8(priv, SYSTEM__INTERRUPT_CLEAR, 0x01);
}

/****************************************************************************
 * Name: vl53l1x_setinterruptpolarity
 *
 * Description:
 *   This function programs the interrupt polarity.
 *
 ****************************************************************************/

static void vl53l1x_setinterruptpolarity(FAR struct vl53l1x_dev_s *priv,
                                         uint8_t newpolarity)
{
  uint8_t temp;

  temp = vl53l1x_getreg8(priv, GPIO_HV_MUX__CTRL);
  temp = temp & 0xef;
  vl53l1x_putreg8(priv, GPIO_HV_MUX__CTRL, temp | (!(newpolarity & 1)) << 4);
}

/****************************************************************************
 * Name: vl53l1x_startranging
 *
 * Description:
 *   This function starts the ranging distance operation. the ranging
 *   operation is continuous. the clear interrupt has to be done after each
 *   get data to allow the interrupt to raise when the next data is ready.
 *
 ****************************************************************************/

static void vl53l1x_startranging(FAR struct vl53l1x_dev_s *priv)
{
  vl53l1x_putreg8(priv, SYSTEM__MODE_START, 0x40);      /* Enable vl53l1x */
}

/****************************************************************************
 * Name: vl53l1x_stopranging
 *
 * Description:
 *   This function stops the ranging.
 *
 ****************************************************************************/

static void vl53l1x_stopranging(FAR struct vl53l1x_dev_s *priv)
{
  vl53l1x_putreg8(priv, SYSTEM__MODE_START, 0x00);      /* Disable vl53l1x */
}

/****************************************************************************
 * Name: vl53l1x_checkfordataready
 *
 * Description:
 *   This function checks if the new ranging data is available by polling the
 *   dedicated register.
 *
 ****************************************************************************/

static void vl53l1x_checkfordataready(FAR struct vl53l1x_dev_s *priv,
                                      FAR uint8_t *isdataready)
{
  uint8_t temp;
  uint8_t intpol;

  /* Read in the register to check if a new value is available */

  temp = vl53l1x_getreg8(priv, GPIO__TIO_HV_STATUS);

  if ((temp & 1) == intpol)
    {
      *isdataready = 1;
    }
  else
    {
      *isdataready = 0;
    }
}

/****************************************************************************
 * Name: vl53l1x_settimingbudgetinms
 *
 * Description:
 *   This function programs the timing budget in ms.
 *
 ****************************************************************************/

static void vl53l1x_settimingbudgetinms(FAR struct vl53l1x_dev_s *priv,
                                        uint16_t timingbudgetinms)
{
  uint16_t dm;
  vl53l1x_getdistancemode(priv, &dm);

  if (dm == 1)                  /* Short distancemode */
    {
      switch (timingbudgetinms)
        {
        case 15:                /* Only available in short distance mode */
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01d);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027);
          break;

        case 20:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006e);
          break;

        case 33:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00d6);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006e);
          break;

        case 50:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1ae);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01e8);
          break;

        case 100:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02e1);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388);
          break;

        case 200:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03e1);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496);
          break;

        case 500:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05c1);
          break;

        default:
          break;
        }
    }
  else
    {
      switch (timingbudgetinms)
        {
        case 20:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001e);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022);
          break;

        case 33:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006e);
          break;

        case 50:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00ad);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00c6);
          break;

        case 100:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01cc);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01ea);
          break;

        case 200:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02d9);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02f8);
          break;

        case 500:
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048f);
          vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04a4);
          break;

        default:
          break;
        }
    }
}

/****************************************************************************
 * Name: vl53l1x_gettimingbudgetinms
 *
 * Description:
 *   This function returns the timing budget in ms.
 *
 ****************************************************************************/

static void vl53l1x_gettimingbudgetinms(FAR struct vl53l1x_dev_s *priv,
                                        FAR uint16_t *ptimingbudget)
{
  uint16_t temp;

  temp = vl53l1x_getreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI);
  switch (temp)
    {
    case 0x001d:
      *ptimingbudget = 15;
      break;

    case 0x0051:
    case 0x001e:
      *ptimingbudget = 20;
      break;

    case 0x00d6:
    case 0x0060:
      *ptimingbudget = 33;
      break;

    case 0x1ae:
    case 0x00ad:
      *ptimingbudget = 50;
      break;

    case 0x02e1:
    case 0x01cc:
      *ptimingbudget = 100;
      break;

    case 0x03e1:
    case 0x02d9:
      *ptimingbudget = 200;
      break;

    case 0x0591:
    case 0x048f:
      *ptimingbudget = 500;
      break;

    default:
      *ptimingbudget = 0;
      break;
    }
}

/****************************************************************************
 * Name: vl53l1x_setdistancemode
 *
 * Description:
 *   This function programs the distance mode (1=short, 2=long(default)).
 *   short mode max distance is limited to 1.3 m but better ambient immunity.
 *   long mode can range up to 4 m in the dark with 200 ms timing budget.
 *
 ****************************************************************************/

static void vl53l1x_setdistancemode(FAR struct vl53l1x_dev_s *priv,
                                    uint16_t dm)
{
  uint16_t tb;

  vl53l1x_gettimingbudgetinms(priv, &tb);
  switch (dm)
    {
    case 1:
      vl53l1x_putreg8(priv, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
      vl53l1x_putreg8(priv, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      vl53l1x_putreg8(priv, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      vl53l1x_putreg8(priv, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
      vl53l1x_putreg16(priv, SD_CONFIG__WOI_SD0, 0x0705);
      vl53l1x_putreg16(priv, SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
      break;

    case 2:
      vl53l1x_putreg8(priv, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0a);
      vl53l1x_putreg8(priv, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0f);
      vl53l1x_putreg8(priv, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0d);
      vl53l1x_putreg8(priv, RANGE_CONFIG__VALID_PHASE_HIGH, 0xb8);
      vl53l1x_putreg16(priv, SD_CONFIG__WOI_SD0, 0x0f0d);
      vl53l1x_putreg16(priv, SD_CONFIG__INITIAL_PHASE_SD0, 0x0e0e);
      break;

    default:
      break;
    }

  vl53l1x_settimingbudgetinms(priv, tb);
}

/****************************************************************************
 * Name: vl53l1x_getdistancemode
 *
 * Description:
 *   This function returns the current distance mode (1=short, 2=long).
 *
 ****************************************************************************/

static void vl53l1x_getdistancemode(FAR struct vl53l1x_dev_s *priv,
                                    FAR uint16_t *dm)
{
  uint8_t tempdm;

  tempdm = vl53l1x_getreg8(priv, PHASECAL_CONFIG__TIMEOUT_MACROP);
  if (tempdm == 0x14)
    {
      *dm = 1;
    }

  if (tempdm == 0x0a)
    {
      *dm = 2;
    }
}

/****************************************************************************
 * Name: vl53l1x_setintermeasurementinms
 *
 * Description:
 *   This function programs the intermeasurement period in ms.
 *
 ****************************************************************************/

static void vl53l1x_setintermeasurementinms(FAR struct vl53l1x_dev_s *priv,
                                            uint16_t intermeasms)
{
  uint16_t clockpll;

  clockpll = vl53l1x_getreg16(priv, VL53L1_RESULT__OSC_CALIBRATE_VAL);
  clockpll = clockpll & 0x3ff;
  vl53l1x_putreg32(priv, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
                   (uint32_t)(clockpll * intermeasms * 1.075));
}

/****************************************************************************
 * Name: vl53l1x_getintermeasurementinms
 *
 * Description:
 *   This function returns the inter-measurement period in ms.
 *
 ****************************************************************************/

static void vl53l1x_getintermeasurementinms(FAR struct vl53l1x_dev_s *priv,
                                            FAR uint16_t *pim)
{
  uint16_t clockpll;
  uint32_t tmp;

  tmp = vl53l1x_getreg32(priv, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD);

  clockpll  = vl53l1x_getreg16(priv, VL53L1_RESULT__OSC_CALIBRATE_VAL);
  clockpll &= 0x3ff;

  *pim = (uint16_t)(tmp / (clockpll * 1.065));
}

/****************************************************************************
 * Name: vl53l1x_bootstate
 *
 * Description:
 *  This function returns the boot state of the device
 *  (1:booted, 0:not booted)
 *
 ****************************************************************************/

static void vl53l1x_bootstate(FAR struct vl53l1x_dev_s *priv,
                              FAR uint8_t *state)
{
  uint8_t tmp = 0;

  tmp = vl53l1x_getreg8(priv, VL53L1_FIRMWARE__SYSTEM_STATUS);
  *state = tmp;
}

/****************************************************************************
 * Name: vl53l1x_getsensorid
 *
 * Description:
 *  This function returns the sensor id, sensor id must be 0xeeac.
 *
 ****************************************************************************/

static void vl53l1x_getsensorid(FAR struct vl53l1x_dev_s *priv,
                                FAR uint16_t *sensorid)
{
  uint16_t tmp = 0;

  tmp = vl53l1x_getreg16(priv, VL53L1_IDENTIFICATION__MODEL_ID);
  *sensorid = tmp;
}

/****************************************************************************
 * Name: vl53l1x_getdistance
 *
 * Description:
 *  This function returns the distance measured by the sensor in mm.
 *
 ****************************************************************************/

static void vl53l1x_getdistance(FAR struct vl53l1x_dev_s *priv,
                                FAR uint16_t *distance)
{
  uint16_t tmp;

  tmp =
    vl53l1x_getreg16(priv,
                     VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0);
  *distance = tmp;
}

/****************************************************************************
 * Name: vl53l1x_getsignalperspad
 *
 * Description:
 *  This function returns the returned signal per spad in kcps/spad.
 *  with kcps stands for kilo count per second.
 *
 ****************************************************************************/

static void vl53l1x_getsignalperspad(FAR struct vl53l1x_dev_s *priv,
                                     FAR uint16_t *signalrate)
{
  uint16_t spnb = 1;
  uint16_t signal;

  signal =
    vl53l1x_getreg16(priv,
       VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0);

  spnb = vl53l1x_getreg16(priv, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0);
  *signalrate = (uint16_t)(2000.0 * signal / spnb);
}

/****************************************************************************
 * Name: vl53l1x_getambientperspad
 *
 * Description:
 *  This function returns the ambient per spad in kcps/spad.
 *
 ****************************************************************************/

static void vl53l1x_getambientperspad(FAR struct vl53l1x_dev_s *priv,
                                      FAR uint16_t *ambpersp)
{
  uint16_t ambientrate;
  uint16_t spnb = 1;

  ambientrate = vl53l1x_getreg16(priv, RESULT__AMBIENT_COUNT_RATE_MCPS_SD);
  spnb = vl53l1x_getreg16(priv, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0);
  *ambpersp = (uint16_t)(2000.0 * ambientrate / spnb);
}

/****************************************************************************
 * Name: vl53l1x_getsignalrate
 *
 * Description:
 *  This function returns the returned signal in kcps.
 *
 ****************************************************************************/

static void vl53l1x_getsignalrate(FAR struct vl53l1x_dev_s *priv,
                                  FAR uint16_t *signal)
{
  uint16_t tmp;

  tmp = vl53l1x_getreg16(priv,
          VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0);
  *signal = tmp * 8;
}

/****************************************************************************
 * Name: vl53l1x_getspadnb
 *
 * Description:
 *  This function returns the current number of enabled spads.
 *
 ****************************************************************************/

static void vl53l1x_getspadnb(FAR struct vl53l1x_dev_s *priv,
                              FAR uint16_t *spnb)
{
  uint16_t tmp;

  tmp = vl53l1x_getreg16(priv, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0);
  *spnb = tmp >> 8;
}

/****************************************************************************
 * Name: vl53l1x_getambientrate
 *
 * Description:
 *  This function returns the ambient rate in kcps.
 *
 ****************************************************************************/

static void vl53l1x_getambientrate(FAR struct vl53l1x_dev_s *priv,
                                   FAR uint16_t *ambrate)
{
  uint16_t tmp;

  tmp = vl53l1x_getreg16(priv, RESULT__AMBIENT_COUNT_RATE_MCPS_SD);
  *ambrate = tmp * 8;
}

/****************************************************************************
 * Name: vl53l1x_getrangestatus
 *
 * Description:
 *  This function returns the ranging status error.
 *
 ****************************************************************************/

static void vl53l1x_getrangestatus(FAR struct vl53l1x_dev_s *priv,
                                   FAR uint8_t *rangestatus)
{
  uint8_t rgst;

  rgst = vl53l1x_getreg8(priv, VL53L1_RESULT__RANGE_STATUS);
  rgst = rgst & 0x1f;
  switch (rgst)
    {
    case 9:
      rgst = 0;
      break;

    case 6:
      rgst = 1;
      break;

    case 4:
      rgst = 2;
      break;

    case 8:
      rgst = 3;
      break;

    case 5:
      rgst = 4;
      break;

    case 3:
      rgst = 5;
      break;

    case 19:
      rgst = 6;
      break;

    case 7:
      rgst = 7;
      break;

    case 12:
      rgst = 9;
      break;

    case 18:
      rgst = 10;
      break;

    case 22:
      rgst = 11;
      break;

    case 23:
      rgst = 12;
      break;

    case 13:
      rgst = 13;
      break;

    default:
      rgst = 255;
      break;
    }

  *rangestatus = rgst;
}

/****************************************************************************
 * Name: vl53l1x_setoffset
 *
 * Description:
 *  This function programs the offset correction in mm.
 *
 ****************************************************************************/

static void vl53l1x_setoffset(FAR struct vl53l1x_dev_s *priv,
                              int16_t offsetvalue)
{
  int16_t temp;

  temp = (offsetvalue * 4);
  vl53l1x_putreg16(priv, ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)temp);
  vl53l1x_putreg16(priv, MM_CONFIG__INNER_OFFSET_MM, 0x0);
  vl53l1x_putreg16(priv, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
}

/****************************************************************************
 * Name: vl53l1x_getoffset
 *
 * Description:
 *  This function returns the programmed offset correction value in mm.
 *
 ****************************************************************************/

static void vl53l1x_getoffset(FAR struct vl53l1x_dev_s *priv,
                              FAR int16_t *offset)
{
  uint16_t temp;

  temp = vl53l1x_getreg16(priv, ALGO__PART_TO_PART_RANGE_OFFSET_MM);
  temp = temp << 3;
  temp = temp >> 5;
  *offset = (int16_t)(temp);
}

/****************************************************************************
 * Name: vl53l1x_setxtalk
 *
 * Description:
 *  This function programs the xtalk correction
 *  value in cps (count per second).
 *  This is the number of photons reflected back from the cover glass in cps.
 *
 ****************************************************************************/

static void vl53l1x_setxtalk(FAR struct vl53l1x_dev_s *priv,
                             uint16_t xtalkvalue)
{
  /* xtalkvalue in count per second to avoid float type */

  vl53l1x_putreg16(priv, ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
                   0x0000);
  vl53l1x_putreg16(priv, ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
                   0x0000);
  vl53l1x_putreg16(priv, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
                   (xtalkvalue << 9) / 1000);
}

/****************************************************************************
 * Name: vl53l1x_getxtalk
 *
 * Description:
 *  This function returns the current programmed xtalk correction
 *  value in cps.
 *
 ****************************************************************************/

static void vl53l1x_getxtalk(FAR struct vl53l1x_dev_s *priv,
                             FAR uint16_t *xtalk)
{
  uint16_t tmp;

  tmp = vl53l1x_getreg16(priv, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS);
  *xtalk = (tmp * 1000) >> 9;
}

/****************************************************************************
 * Name: vl53l1x_setdistancethreshold
 *
 * Description:
 *  This function programs the threshold detection mode.
 *
 ****************************************************************************/

static void vl53l1x_setdistancethreshold(FAR struct vl53l1x_dev_s *priv,
                                         uint16_t threshlow,
                                         uint16_t threshhigh, uint8_t window,
                                         uint8_t intonnotarget)
{
  uint8_t temp = 0;

  temp = vl53l1x_getreg8(priv, SYSTEM__INTERRUPT_CONFIG_GPIO);
  temp = temp & 0x47;
  if (intonnotarget == 0)
    {
      vl53l1x_putreg8(priv, SYSTEM__INTERRUPT_CONFIG_GPIO,
                      (temp | (window & 0x07)));
    }
  else
    {
      vl53l1x_putreg8(priv, SYSTEM__INTERRUPT_CONFIG_GPIO,
                      ((temp | (window & 0x07)) | 0x40));
    }

  vl53l1x_putreg16(priv, SYSTEM__THRESH_HIGH, threshhigh);
  vl53l1x_putreg16(priv, SYSTEM__THRESH_LOW, threshlow);
}

/****************************************************************************
 * Name: vl53l1x_getdistancethresholdwindow
 *
 * Description:
 *  This function returns the window detection mode (0=below; 1=above;
 *  2=out; 3=in).
 *
 ****************************************************************************/

static void vl53l1x_getdistancethresholdwindow(FAR struct vl53l1x_dev_s *priv,
                                               FAR uint16_t *window)
{
  uint8_t tmp;
  tmp = vl53l1x_getreg8(priv, SYSTEM__INTERRUPT_CONFIG_GPIO);
  *window = (uint16_t)(tmp & 0x7);
}

/****************************************************************************
 * Name: vl53l1x_getdistancethresholdlow
 *
 * Description:
 *  This function returns the low threshold in mm.
 *
 ****************************************************************************/

static void vl53l1x_getdistancethresholdlow(FAR struct vl53l1x_dev_s *priv,
                                            FAR uint16_t *low)
{
  uint16_t tmp;

  tmp = vl53l1x_getreg16(priv, SYSTEM__THRESH_LOW);
  *low = tmp;
}

/****************************************************************************
 * Name: vl53l1x_getdistancethresholdhigh
 *
 * Description:
 *  This function returns the high threshold in mm.
 *
 ****************************************************************************/

static void vl53l1x_getdistancethresholdhigh(FAR struct vl53l1x_dev_s *priv,
                                             FAR uint16_t *high)
{
  uint16_t tmp;

  tmp = vl53l1x_getreg16(priv, SYSTEM__THRESH_HIGH);
  *high = tmp;
}

/****************************************************************************
 * Name: vl53l1x_setroi
 *
 * Description:
 *  This function programs the roi (region of interest).
 *
 ****************************************************************************/

static void vl53l1x_setroi(FAR struct vl53l1x_dev_s *priv, uint16_t x,
                           uint16_t y)
{
  uint8_t opticalcenter;

  opticalcenter =
    vl53l1x_getreg8(priv, VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD);
  if (x > 16)
    {
      x = 16;
    }

  if (y > 16)
    {
      y = 16;
    }

  if (x > 10 || y > 10)
    {
      opticalcenter = 199;
    }

  vl53l1x_putreg8(priv, ROI_CONFIG__USER_ROI_CENTRE_SPAD, opticalcenter);
  vl53l1x_putreg8(priv, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
                  (y - 1) << 4 | (x - 1));
}

/****************************************************************************
 * Name: vl53l1x_getroi_xy
 *
 * Description:
 *  This function returns width x and height y.
 *
 ****************************************************************************/

static void vl53l1x_getroi_xy(FAR struct vl53l1x_dev_s *priv,
                              FAR uint16_t *roi_x, FAR uint16_t *roi_y)
{
  uint8_t tmp;

  tmp = vl53l1x_getreg8(priv, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE);
  *roi_x = ((uint16_t) tmp & 0x0f) + 1;
  *roi_y = (((uint16_t) tmp & 0xf0) >> 4) + 1;
}

/****************************************************************************
 * Name: vl53l1x_setsignalthreshold
 *
 * Description:
 *  This function programs a new signal threshold in kcps.
 *
 ****************************************************************************/

static void vl53l1x_setsignalthreshold(FAR struct vl53l1x_dev_s *priv,
                                       uint16_t signal)
{
  vl53l1x_putreg16(priv, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,
                   signal >> 3);
}

/****************************************************************************
 * Name: vl53l1x_getsignalthreshold
 *
 * Description:
 *  This function returns the current signal threshold in kcps.
 *
 ****************************************************************************/

static void vl53l1x_getsignalthreshold(FAR struct vl53l1x_dev_s *priv,
                                       FAR uint16_t *signal)
{
  uint16_t tmp;

  tmp = vl53l1x_getreg16(priv, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS);
  *signal = tmp << 3;
}

/****************************************************************************
 * Name: vl53l1x_setsigmathreshold
 *
 * Description:
 *  This function programs a new sigma threshold in mm (default=15 mm).
 *
 ****************************************************************************/

static void vl53l1x_setsigmathreshold(FAR struct vl53l1x_dev_s *priv,
                                      uint16_t sigma)
{
  if (sigma <= (0xffff >> 2))
    {
      /* 16 bits register 14.2 format */

      vl53l1x_putreg16(priv, RANGE_CONFIG__SIGMA_THRESH, sigma << 2);
    }
}

/****************************************************************************
 * Name: vl53l1x_getsigmathreshold
 *
 * Description:
 *  This function returns the current sigma threshold in mm.
 *
 ****************************************************************************/

static void vl53l1x_getsigmathreshold(FAR struct vl53l1x_dev_s *priv,
                                      FAR uint16_t *sigma)
{
  uint16_t tmp;

  tmp = vl53l1x_getreg16(priv, RANGE_CONFIG__SIGMA_THRESH);
  *sigma = tmp >> 2;
}

/****************************************************************************
 * Name: vl53l1x_starttemperatureupdate
 *
 * Description:
 *  This function performs the temperature calibration. it is recommended to
 *  call This function any time the temperature might have changed by more
 *  than 8 deg c without sensor ranging activity for an extended period.
 *
 ****************************************************************************/

static void vl53l1x_starttemperatureupdate(FAR struct vl53l1x_dev_s *priv)
{
  uint8_t tmp = 0;

  vl53l1x_putreg8(priv, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x81);

  vl53l1x_putreg8(priv, 0x0b, 0x92);
  vl53l1x_startranging(priv);
  while (tmp == 0)
    {
      vl53l1x_checkfordataready(priv, &tmp);
    }

  tmp = 0;
  vl53l1x_clearinterrupt(priv);
  vl53l1x_stopranging(priv);
  vl53l1x_putreg8(priv, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);

  vl53l1x_putreg8(priv, 0x0b, 0);
}

/****************************************************************************
 * Name: vl53l1x_calibrateoffset
 *
 * Description:
 *  the function returns the offset value found and programs the offset
 *  compensation into the device.
 *
 ****************************************************************************/

static void vl53l1x_calibrateoffset(FAR struct vl53l1x_dev_s *priv,
                                    uint16_t targetdistinmm,
                                    FAR int16_t *offset)
{
  uint8_t i = 0;
  uint8_t tmp;
  int16_t averagedistance = 0;
  uint16_t distance;

  vl53l1x_putreg16(priv, ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
  vl53l1x_putreg16(priv, MM_CONFIG__INNER_OFFSET_MM, 0x0);
  vl53l1x_putreg16(priv, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
  vl53l1x_startranging(priv);   /* Enable vl53l1x sensor */

  for (i = 0; i < 50; i++)
    {
      while (tmp == 0)
        {
          vl53l1x_checkfordataready(priv, &tmp);
        }

      tmp = 0;
      vl53l1x_getdistance(priv, &distance);
      vl53l1x_clearinterrupt(priv);
      averagedistance = averagedistance + distance;
    }

  vl53l1x_stopranging(priv);
  averagedistance = averagedistance / 50;
  *offset = targetdistinmm - averagedistance;
  vl53l1x_putreg16(priv, ALGO__PART_TO_PART_RANGE_OFFSET_MM, *offset * 4);
}

/****************************************************************************
 * Name: vl53l1x_calibratextalk
 *
 * Description:
 *  This function returns the xtalk value found and programs the xtalk
 *  compensation to the device.
 *
 ****************************************************************************/

static void vl53l1x_calibratextalk(FAR struct vl53l1x_dev_s *priv,
                                   uint16_t targetdistinmm,
                                   FAR uint16_t *xtalk)
{
  uint8_t i;
  uint8_t tmp = 0;
  float averagesignalrate = 0;
  float averagedistance = 0;
  float averagespadnb = 0;
  uint16_t distance = 0;
  uint16_t spadnum;
  uint16_t sr;

  vl53l1x_putreg16(priv, 0x0016, 0);
  vl53l1x_startranging(priv);

  for (i = 0; i < 50; i++)
    {
      while (tmp == 0)
        {
          vl53l1x_checkfordataready(priv, &tmp);
        }

      tmp = 0;
      vl53l1x_getsignalrate(priv, &sr);
      vl53l1x_getdistance(priv, &distance);
      vl53l1x_clearinterrupt(priv);
      averagedistance = averagedistance + distance;
      vl53l1x_getspadnb(priv, &spadnum);
      averagespadnb = averagespadnb + spadnum;
      averagesignalrate = averagesignalrate + sr;
    }

  vl53l1x_stopranging(priv);
  averagedistance = averagedistance / 50;
  averagespadnb = averagespadnb / 50;
  averagesignalrate = averagesignalrate / 50;

  *xtalk =
    (uint16_t)(512 *
                (averagesignalrate * (1 - (averagedistance / targetdistinmm))) /
                averagespadnb);
  vl53l1x_putreg16(priv, 0x0016, *xtalk);
}

/****************************************************************************
 * Name: vl53l1x_getreg8
 *
 * Description:
 *   read from an 8-bit vl53l1x register
 *
 ****************************************************************************/

static uint8_t vl53l1x_getreg8(FAR struct vl53l1x_dev_s *priv,
                               uint16_t regaddr)
{
  struct i2c_config_s config;
  uint8_t regval = 0;
  uint8_t reg_addr_aux[2];
  int ret;

  /* Set up the i2c configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  /* Split the i2c direction */

  reg_addr_aux[0] = (regaddr >> 8) & 0xff;
  reg_addr_aux[1] = regaddr;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read the register value */

  ret = i2c_read(priv->i2c, &config, &regval, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  return regval;
}

/****************************************************************************
 * Name: vl53l1x_getreg16
 *
 * Description:
 *   read two 8-bit from a bmp180 register
 *
 ****************************************************************************/

static uint16_t vl53l1x_getreg16(FAR struct vl53l1x_dev_s *priv,
                                 uint16_t regaddr)
{
  struct i2c_config_s config;
  uint16_t msb;
  uint16_t lsb;
  uint16_t regval = 0;
  uint8_t reg_addr_aux[2];
  int ret;

  /* Set up the i2c configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  /* Split the i2c direction */

  reg_addr_aux[0] = (regaddr >> 8) & 0xff;
  reg_addr_aux[1] = regaddr;

  /* Register to read */

  sninfo("reg %02x % \r\n", reg_addr_aux[0], reg_addr_aux[1]);
  ret = i2c_write(priv->i2c, &config, &reg_addr_aux, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read register */

  ret = i2c_read(priv->i2c, &config, (FAR uint8_t *)&regval, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* msb and lsb are inverted */

  msb = (regval & 0xff);
  lsb = (regval & 0xff00) >> 8;

  regval = (msb << 8) | lsb;

  return regval;
}

/****************************************************************************
 * Name: vl53l1x_getreg32
 *
 * Description:
 *   read 4 8-bit from a vl53l1x register
 *
 ****************************************************************************/

static uint32_t vl53l1x_getreg32(FAR struct vl53l1x_dev_s *priv,
                                 uint16_t regaddr)
{
  struct i2c_config_s config;
  uint16_t byte1;
  uint16_t byte2;
  uint16_t byte3;
  uint16_t byte4;

  uint32_t regval = 0;
  int ret;
  uint8_t reg_addr_aux[2];

  reg_addr_aux[0] = (regaddr >> 8) & 0xff;
  reg_addr_aux[1] = regaddr;

  /* Set up the i2c configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  /* Register to read */

  ret = i2c_write(priv->i2c, &config, &regaddr, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read register */

  ret = i2c_read(priv->i2c, &config, (FAR uint8_t *) & regval, 4);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* The bytes are in the opposite order of importance */

  byte1 = (regval & 0xff);
  byte2 = (regval & 0xff00) >> 8;
  byte3 = (regval & 0xffff00) >> 16;
  byte4 = (regval & 0xffffff00) >> 24;

  regval = (byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4;

  return regval;
}

/****************************************************************************
 * Name: vl53l1x_putreg8
 *
 * Description:
 *   write to an 8-bit vl53l1x register
 *
 ****************************************************************************/

static void vl53l1x_putreg8(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                            uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t data[3];
  int ret;

  /* Set up the i2c configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  data[0] = (regaddr >> 8) & 0xff;
  data[1] = regaddr;
  data[2] = regval & 0xff;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (FAR uint8_t *) & data, 3);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }

  return;
}

/****************************************************************************
 * Name: vl53l1x_putreg16
 *
 * Description:
 *   write to an 16-bit vl53l1x register
 *
 ****************************************************************************/

static void vl53l1x_putreg16(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                             uint16_t regval)
{
  struct i2c_config_s config;
  uint8_t data[4];
  int ret;

  /* Set up the i2c configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  data[0] = (regaddr >> 8) & 0xff;
  data[1] = regaddr;
  data[2] = (regval >> 8) & 0xff;
  data[3] = regval & 0xff;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (FAR uint8_t *) & data, 4);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }

  return;
}

/****************************************************************************
 * Name: vl53l1x_putreg32
 *
 * Description:
 *   write to an 32-bit vl53l1x register
 *
 ****************************************************************************/

static void vl53l1x_putreg32(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                             uint32_t regval)
{
  struct i2c_config_s config;
  uint8_t data[7];
  int ret;

  /* Set up the i2c configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  data[0] = (regaddr >> 8) & 0xff;
  data[1] = regaddr;
  data[2] = (regval >> 24) & 0xff;
  data[4] = (regval >> 16) & 0xff;
  data[5] = (regval >> 8) & 0xff;
  data[6] = regval & 0xff;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (FAR uint8_t *)&data, 7);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }

  return;
}

/****************************************************************************
 * Name: vl53l1x_open
 *
 * Description:
 *   This function is called whenever the vl53l1x device is opened.
 *
 ****************************************************************************/

static int vl53l1x_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: vl53l1x_close
 *
 * Description:
 *   This routine is called when the vl53l1x device is closed.
 *
 ****************************************************************************/

static int vl53l1x_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: vl53l1x_read
 ****************************************************************************/

static ssize_t vl53l1x_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct vl53l1x_dev_s *priv = inode->i_private;
  FAR FAR uint16_t *aux = (FAR FAR uint16_t *)buffer;

  vl53l1x_startranging(priv);
  vl53l1x_getdistance(priv, aux);
  vl53l1x_stopranging(priv);

  return sizeof(uint16_t);
}

/****************************************************************************
 * Name: vl53l1x_write
 ****************************************************************************/

static ssize_t vl53l1x_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: vl53l1x_ioctl
 ****************************************************************************/

static ssize_t vl53l1x_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct vl53l1x_dev_s *priv = inode->i_private;

  switch (cmd)
    {
    case snioc_distanceshort:
      {
        sninfo("set distance up to 1.3m\n");
        vl53l1x_setdistancemode(priv, 1);
      }
      break;

    case snioc_distancelong:
      {
        sninfo("set distance up to 4m\n");
        vl53l1x_setdistancemode(priv, 2);
      }
      break;

    case snioc_calibrate:
      {
        sninfo("calibrating distance\n");
        int16_t offset;
        vl53l1x_getoffset(priv, offset);
        vl53l1x_calibrateoffset(priv, arg, offset);
      }
      break;

    case snioc_tempupdate:
      {
        sninfo("recalculating due to temperature change\n");
        vl53l1x_starttemperatureupdate(priv);
      }
      break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vl53l1x_register
 *
 * Description:
 *   register the vl53l1x character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - the full path to the driver to register. e.g., "/dev/tof"
 *   i2c     - an instance of the i2c interface to use to communicate with
 *             vl53l1x tof
 *
 * Returned Value:
 *   zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vl53l1x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct vl53l1x_dev_s *priv;
  int ret;
  uint16_t sensorid;

  /* Initialize the vl53l1x device structure */

  priv = (FAR struct vl53l1x_dev_s *)kmm_malloc(sizeof(struct vl53l1x_dev_s));
  if (!priv)
    {
      snerr("ERROR: failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = 0x29;
  priv->freq = VLM53L1X_FREQ;

  vl53l1x_sensorinit(priv);

  vl53l1x_getsensorid(priv, &sensorid);
  if (sensorid != 0xeacc)
    {
      snerr("ERROR: failed sensor id %04x\n", sensorid);
      kmm_free(priv);
      return 0;
    }

  register_driver(devpath, &g_vl53l1xfops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: failed to register driver: %d\n", ret);
      kmm_free(priv);
      return 0;
    }

  sninfo("vl53l1x driver loaded successfully!\n");
  return 1;
}

#endif  /* CONFIG_I2C && CONFIG_SENSORS_VL53L1X */
