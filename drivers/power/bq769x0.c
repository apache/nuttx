/****************************************************************************
 * drivers/power/bq769x0.c
 * Lower half driver for BQ769x0 battery monitor
 *
 *   Copyright (C) 2019 2G Engineering. All rights reserved.
 *   Author: Josh Lange <jlange@2g-eng.com>
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

/* The bq76920/bq76930/bq76940 battery monitor ICs provide voltage, current,
 * and temperature monitoring of up to 15-series cells.  These ICs also provide
 * Coulomb counting for state-of-charge measurement, balance drivers for all
 * cells, and drivers for external cell protection switches.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <crc8.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_monitor.h>
#include <nuttx/power/battery_ioctl.h>

#include <nuttx/power/bq769x0.h>

/* This driver requires:
 *
 * CONFIG_BATTERY_MONITOR- Upper half battery driver support
 * CONFIG_I2C - I2C support
 * CONFIG_I2C_BQ769X0 - And the driver must be explicitly selected.
 */

#if defined(CONFIG_BATTERY_MONITOR) && defined(CONFIG_I2C) && \
    defined(CONFIG_I2C_BQ769X0)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif
#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_BQ769X0
#  define baterr    _err
#  define batreg    _err
#  define batinfo   _info
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define baterr(x...)
#    define batreg(x...)
#    define batinfo(x...)
#  else
#    define baterr(void)
#    define batreg(void)
#    define batinfo(void)
#  endif
#endif

/****************************************************************************
 * Private
 ****************************************************************************/

struct bq769x0_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  FAR const struct battery_monitor_operations_s *ops; /* Battery operations */
  sem_t batsem;                /* Enforce mutually exclusive access */

  /* Data fields specific to the lower half BQ769x0 driver follow */

  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  uint8_t chip;                 /* Chip Type (e.g. CHIP_76920) */
  uint32_t frequency;           /* I2C frequency */
  uint32_t gain;                /* ADC gain value in uV */
  uint32_t offset;              /* ADC offset value in uV */
  bool crc;                     /* Whether or not the device has CRC enabled */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C support functions */

static int bq769x0_getreg8(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval);
static int bq769x0_putreg8(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);
static int bq769x0_getreg16(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                            FAR uint16_t *regval);
static int bq769x0_getnreg16(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                             FAR uint16_t *regvals, unsigned int count);

/* Device functions */
static inline int bq769x0_getreport(FAR struct bq769x0_dev_s *priv,
                                    uint8_t *report);
static inline int bq769x0_getvolt(FAR struct bq769x0_dev_s *priv, int *volts);
static inline int bq769x0_getcc(FAR struct bq769x0_dev_s *priv, int *cc);
static inline int bq769x0_getcellvolt(FAR struct bq769x0_dev_s *priv,
    struct battery_monitor_voltage_s *voltages);
static inline int bq769x0_gettemperature(FAR struct bq769x0_dev_s *priv,
    struct battery_monitor_temperature_s *temps);

static inline int bq769x0_updategain(FAR struct bq769x0_dev_s *priv);

/* Battery driver lower half methods */

static int bq769x0_state(struct battery_monitor_dev_s *dev, int *status);
static int bq769x0_health(struct battery_monitor_dev_s *dev, int *health);
static int bq769x0_online(struct battery_monitor_dev_s *dev, bool *status);
static int bq769x0_voltage(struct battery_monitor_dev_s *dev, int *value);
static int bq769x0_cellvoltage(struct battery_monitor_dev_s *dev,
    struct battery_monitor_voltage_s *cellv);
static int bq769x0_current(struct battery_monitor_dev_s *dev, int *value);
static int bq769x0_soc(struct battery_monitor_dev_s *dev, b16_t *value);
static int bq769x0_coulombs(struct battery_monitor_dev_s *dev, uintptr_t param);
static int bq769x0_temp(struct battery_monitor_dev_s *dev,
    struct battery_monitor_temperature_s *temps);
static int bq769x0_balance(struct battery_monitor_dev_s *dev,
    struct battery_monitor_balance_s *bal);
static int bq769x0_operate(struct battery_monitor_dev_s *dev, uintptr_t param);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_monitor_operations_s g_bq769x0ops =
{
  bq769x0_state,
  bq769x0_health,
  bq769x0_online,
  bq769x0_voltage,
  bq769x0_cellvoltage,
  bq769x0_current,
  bq769x0_soc,
  bq769x0_coulombs,
  bq769x0_temp,
  bq769x0_balance,
  bq769x0_operate,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq769x0_getreg8
 *
 * Description:
 *   Read a 8-bit value from a BQ769x0 register.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 NO-ACK STOP
 *
 ****************************************************************************/

static int bq769x0_getreg8(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval)
{
  struct i2c_config_s config;
  uint8_t val[2];
  int ret;
  int datalen;
  uint8_t crc;

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      baterr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Our expected data length varies depending on whether or not a CRC is used */

  if (priv->crc)
    {
      datalen = 2;
    }
  else
    {
      datalen = 1;
    }

  /* Restart and read 8-bits from the register */

  ret = i2c_read(priv->i2c, &config, val, datalen);
  if (ret < 0)
    {
      baterr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* If CRC is used, verify that it is correct */
  if (priv->crc)
    {
      crc = crc8ccittpart(&priv->addr, 1, 0);
      crc = crc8ccittpart(val, 1, crc);
      if (crc != val[1])
        {
          baterr("ERROR: CRC mismatch: Got %02x, Expected %02x\n", val[1], crc);
          return ERROR;
        }
    }

  /* Copy 8-bit value to be returned */

  *regval = val[0];
  return OK;
}

/****************************************************************************
 * Name: bq769x0_putreg8
 *
 * Description:
 *   Write a 8-bit value to a BQ769x0 register.
 *
 *   START <I2C write address> ACK <Reg address> ACK Data0 ACK STOP
 *
 ****************************************************************************/

static int bq769x0_putreg8(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[3];
  int datalen;
  uint8_t crc;

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  batreg("addr: %02x regval: %08x\n", regaddr, regval);

  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Our expected data length varies depending on whether or not a CRC is used */
  if (priv->crc)
    {
      datalen = 3;
      crc = crc8ccittpart(&priv->addr, 1, 0);
      crc = crc8ccittpart(buffer, 2, crc);
      buffer[2] = crc;
    }
  else
    {
      datalen = 2;
    }

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, datalen);
}


/****************************************************************************
 * Name: bq769x0_getreg16
 *
 * Description:
 *   Read a 16-bit value from a BQ769x0 register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int bq769x0_getreg16(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           FAR uint16_t *regval)
{
    return bq769x0_getnreg16(priv, regaddr, regval, 1);
}
/****************************************************************************
 * Name: bq769x0_getnreg16
 *
 * Description:
 *   Read an array of 16-bit values from BQ769x0 register pairs.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int bq769x0_getnreg16(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           FAR uint16_t *regvals, unsigned int count)
{
  struct i2c_config_s config;
  uint8_t val[(2 * 22) + 1];
  int ret;
  int datalen;
  uint8_t crc;
  int i;

  if (count >= (sizeof(val) / 2))
    {
      count = sizeof(val) / 2 - 1;
    }

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      baterr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Our expected data length varies depending on whether or not a CRC is used */

  if (priv->crc)
    {
      datalen = (2 * count) + 1;
    }
  else
    {
      datalen = (2 * count);
    }

  /* Restart and read 16-bits from the register */

  ret = i2c_read(priv->i2c, &config, val, datalen);
  if (ret < 0)
    {
      baterr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* If CRC is used, verify that it is correct */
  if (priv->crc)
    {
      crc = crc8ccittpart(&priv->addr, 1, 0);
      crc = crc8ccittpart(val, datalen, crc);
      if (crc != val[datalen - 1])
        {
          baterr("ERROR: CRC mismatch: Got %02x, Expected %02x\n", val[2], crc);
          return ERROR;
        }
    }

  /* Copy 16-bit values to be returned */
  for (i = 0; i < datalen; i += 1) {
      *regvals = (uint16_t)val[2 * i] << 8 | (uint16_t)val[(2 * i) + 1];
      regvals += 1;
  }
  return OK;
}

/****************************************************************************
 * Name: bq769x0_getreport
 *
 * Description:
 *   Read the BQ769X0 SYS_STAT register
 *
 ****************************************************************************/

static inline int bq769x0_getreport(FAR struct bq769x0_dev_s *priv,
                                    uint8_t *report)
{
  uint8_t regval = 0;
  int ret;

  ret = bq769x0_getreg8(priv, BQ769X0_REG_SYS_STAT, &regval);
  if (ret == OK)
    {
      *report = regval;
    }

  return ret;
}

/****************************************************************************
 * Name: bq769x0_updategain
 *
 * Description:
 *   Updates the local copies of the BQ769x0 ADC gain registers.
 *   These are used when converting ADC values to actual voltages.
 *
 ****************************************************************************/

static inline int bq769x0_updategain(FAR struct bq769x0_dev_s *priv)
{
  int ret;
  uint8_t gainreg1;
  uint8_t gainreg2;
  uint8_t gain;
  int8_t offset;

  /* Read current register values */

  ret = bq769x0_getreg8(priv, BQ769X0_REG_ADCGAIN1, &gainreg1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }
  ret = bq769x0_getreg8(priv, BQ769X0_REG_ADCGAIN2, &gainreg2);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }
  ret = bq769x0_getreg8(priv, BQ769X0_REG_ADCOFFSET, (uint8_t *)&offset);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Calculate actual gain & offset values
   * gainreg1 contains gain bits 4-3
   * gainreg2 contains gain bits 2-0
   */

  gainreg1 &= BQ769X0_ADCGAIN1_MASK;
  gainreg2 &= BQ769X0_ADCGAIN2_MASK;
  gain = (gainreg1 << 1) | (gainreg2 >> 5);

  priv->gain = gain + BQ769X0_BASE_GAIN;
  priv->offset = offset * 1000; /* Convert mV to uV */

  batinfo("Battery monitor gain: %d, offset: %d.%\n", priv->gain, priv->offset);

  return OK;
}

/****************************************************************************
 * Name: bq769x0_state
 *
 * Description:
 *   Return the current battery state
 *
 ****************************************************************************/

static int bq769x0_state(struct battery_monitor_dev_s *dev, int *status)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  uint8_t regval = 0;
  int ret;

  ret = bq769x0_getreport(priv, &regval);
  if (ret < 0)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

#warning fixme
  *status = regval;

  return OK;
}

/****************************************************************************
 * Name: bq769x0_health
 *
 * Description:
 *   Return the current battery health state
 *
 * Note: if more than one fault happened the user needs to call this ioctl
 * again to read a new fault, repeat until receive a BATTERY_HEALTH_GOOD.
 *
 ****************************************************************************/

static int bq769x0_health(struct battery_monitor_dev_s *dev, int *health)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  uint8_t regval = 0;
  int ret;

  ret = bq769x0_getreport(priv, &regval);
  if (ret < 0)
    {
      *health = BATTERY_HEALTH_UNKNOWN;
      return ret;
    }

  switch (regval)
  {

  default:
    *health = BATTERY_HEALTH_UNKNOWN;
    break;
  }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_online
 *
 * Description:
 *   Return true if the battery is online
 *
 ****************************************************************************/

static int bq769x0_online(struct battery_monitor_dev_s *dev, bool *status)
{
  /* There is no concept of online/offline in this driver */

  *status = true;
  return OK;
}

/****************************************************************************
 * Name: bq769x0_getvolt
 *
 * Description:
 *   Gets the battery stack voltage in uV.
 *
 ****************************************************************************/

static inline int bq769x0_getvolt(FAR struct bq769x0_dev_s *priv, int *volts)
{
  uint16_t regval;
  int ret;
  int idx;

  ret = bq769x0_getreg16(priv, BQ769X0_REG_BAT_HI, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Voltage is returned from the chip in units of <gain>uV/LSB
   * An offset also needs to be added.
   */

#warning fixme
  regval = regval * priv->gain;
  regval = regval + priv->offset;

  *volts = regval;

  return OK;
}

/****************************************************************************
 * Name: bq769x0_getvolt
 *
 * Description:
 *   Gets one or more battery cell voltages from the monitor.
 *
 ****************************************************************************/

static inline int bq769x0_getcellvolt(FAR struct bq769x0_dev_s *priv,
                                      struct battery_monitor_voltage_s *voltages)
  {
  uint16_t regvals[BQ769X0_MAX_CELLS];
  int ret
  int idx;

  if (voltages)
    {
     if (voltages->cell_count > BQ769X0_MAX_CELLS)
       {
         voltages->cell_count = BQ769X0_MAX_CELLS;
       }
    }

  ret = bq769x0_getnreg16(priv, BQ769X0_REG_VC1_HI, regvals, voltages->cell_count);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  for (idx = 0; idx < voltages->cell_count; idx += 1) {

      /* Voltage is returned from the chip in units of <gain>uV/LSB
       * An offset also needs to be added.
       */

      voltages->cell_voltages[idx] = ((uint32_t) regvals[i] * priv->gain) + priv->offset;
  }
}

/****************************************************************************
 * Name: bq769x0_gettemperature
 *
 * Description:
 *   Gets the voltage(s) at the temperature sensor input(s) of the chip
 *   It is up to the user to convert these voltage values into temperature
 *   values, as many types of temperature sensors exist.
 *
 ****************************************************************************/
static inline int bq769x0_gettemperature(FAR struct bq769x0_dev_s *priv,
                                         struct battery_monitor_temperature_s *temps)
{
  int chip_sensors;
  int ret;
  int i;
  uint16_t regvals[3];

  /* The number of temperature registers varies depending on the chip variant */

  switch (priv->chip)
    {
    case CHIP_BQ76920:
      chip_sensors = 1;
      break;
    case CHIP_BQ76930:
      chip_sensors = 2;
      break;
    default:
    case CHIP_BQ76940:
      chip_sensors = 3;
      break;
    }

  /* Read the number of sensors requested or available, whichever is smaller */

  temps->sensor_count = MIN(chip_sensors, temps->sensor_count);
  ret = bq769x0_getnreg16(priv, BQ769X0_REG_TS1_HI, regvals, temps->sensor_count);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Convert temp sensor ADC values to microvolts */

  for (i = 0; i < temps->sensor_count; i += 1)
    {
      temps->temperatures[i] = ((uint32_t) regvals[i] * priv->gain) + priv->offset;
    }
  return OK;

}

/****************************************************************************
 * Name: bq769x0_getcc
 *
 * Description:
 *   Gets the value of the Coulomb counter from the BQ769X0
 *
 ****************************************************************************/

static inline int bq769x0_getcc(FAR struct bq769x0_dev_s *priv, int *cc)
{
  int16_t regval;
  int ret
  int idx;

#warning scaling
  ret = bq769x0_getreg16(priv, BQ769X0_REG_CC_HI, (uint16_t *)&regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  *cc = regval;
  return OK;
}


/****************************************************************************
 * Name: bq769x0_voltage
 *
 * Description:
 *   Get the pack voltage
 *
 ****************************************************************************/

static int bq769x0_voltage(struct battery_monitor_dev_s *dev, int *value)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;

  /* Get pack voltage from battery monitor */

  ret = bq769x0_getvolt(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Error getting voltage from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_current
 *
 * Description:
 *   Get the pack current
 *
 ****************************************************************************/

static int bq769x0_current(struct battery_monitor_dev_s *dev, int *value)
{

    /* The BQ769X0 does not support directly reporting pack curent, although you could
     * come up with an average value by looking at the Coulomb counter over time.
     */

    return -ENOSYS;
}

static int bq769x0_temp(struct battery_monitor_dev_s *dev,
    struct battery_monitor_temperature_s *temps)
{
    return bq769x0_gettemperature(dev, temps);
}

/****************************************************************************
 * Name: bq769x0_operate
 *
 * Description:
 *   Do miscellaneous battery ioctl()
 *
 ****************************************************************************/

static int bq769x0_operate(struct battery_monitor_dev_s *dev, uintptr_t param)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq769x0_initialize
 *
 * Description:
 *   Initialize the BQ769x0 battery driver and return an instance of the
 *   lower_half interface that may be used with battery_monitor_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_MONITOR - Upper half battery driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_I2C_BQ769X0 - And the driver must be explictly selected.
 *
 * Input Parameters:
 *   i2c       - An instance of the I2C interface to use to communicate with
 *               the BQ769x0
 *   addr      - The I2C address of the BQ769X0 (Can be 0x08 or 0x18).
 *   frequency - The I2C frequency
 *   crc       - True if the device has CRC enabled (see TI datasheet)
 *   cellcount - The number of battery cells attached to the BQ769X0.  The
 *               mapping of the cells changes based on count - see datasheet.
 *   chip      - The chip type (either CHIP_BQ76920, CHIP_BQ76930, or
 *               CHIP_BQ76940).  This is used to map cell numbers when the
 *               full capacity of the chip is not used.  See the TI datasheet
 *               for cell wiring information.
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance.  A NULL pointer
 *   is returned on a failure to initialize the BQ769x0 lower half.
 *
 ****************************************************************************/

FAR struct battery_monitor_dev_s *
  bq769x0_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                     uint32_t frequency, bool crc, uint8_t cellcount,
                     uint8_t chip)
{
  FAR struct bq769x0_dev_s *priv;
  int ret;

  /* Initialize the BQ769x0 device structure */

  priv = (FAR struct bq769x0_dev_s *)kmm_zalloc(sizeof(struct bq769x0_dev_s));
  if (priv)
    {
      /* Initialize the BQ769x0 device structure */

      nxsem_init(&priv->batsem, 0, 1);
      priv->ops       = &g_bq769x0ops;
      priv->i2c       = i2c;
      priv->addr      = addr;
      priv->frequency = frequency;
      priv->crc       = crc;
      priv->chip      = chip;

      /* Configure the BQ769x0 */


      /* Pull the factory-calibrated gain and offset values from the chip. */

      ret = bq769x0_updategain(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to get gain/offset values from the BQ769x0: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }


    }

  return (FAR struct battery_monitor_dev_s *)priv;
}

#endif /* CONFIG_BATTERY_MONITOR && CONFIG_I2C && CONFIG_I2C_BQ769X0 */
