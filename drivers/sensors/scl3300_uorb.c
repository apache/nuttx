/****************************************************************************
 * drivers/sensors/scl3300_uorb.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Driver for the Murata SCL3300-D01 3-axis inclinometer.
 *
 * All section, table and figure numbers refer to the Murata SCL3300-D01
 * Data Sheet, Doc.No. 4921, Rev. 4.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/scl3300.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI link (§5.1.1, §5.1.2, Table 8): mode 0, MSB first, 32-bit frames
 * and at least 10 us with CS high between two frames (TLH).
 */

#define SCL3300_SPI_MODE          SPIDEV_MODE0
#define SCL3300_SPI_NBITS         8
#define SCL3300_FRAME_BYTES       4
#define SCL3300_TLH_US            10

/* SPI frame layout (Table 14): OP[31:26] = RW + ADDR[4:0], RS[25:24],
 * DATA[23:8], CRC[7:0].
 */

#define SCL3300_FRAME_RW          (1u << 31)
#define SCL3300_FRAME_ADDR_SHIFT  26
#define SCL3300_FRAME_ADDR_MASK   0x1f
#define SCL3300_FRAME_OP(f)       (((f) >> 26) & 0x3f)
#define SCL3300_FRAME_RS(f)       (((f) >> 24) & 0x03)
#define SCL3300_FRAME_DATA(f)     ((uint16_t)(((f) >> 8) & 0xffff))
#define SCL3300_FRAME_CRC(f)      ((uint8_t)((f) & 0xff))

/* Return status (§5.1.5, Table 15) */

#define SCL3300_RS_STARTUP        0
#define SCL3300_RS_NORMAL         1
#define SCL3300_RS_ERROR          3

/* CRC-8 (§5.2, Table 17, Fig. 15): polynomial x^8 + x^4 + x^3 + x^2 + 1,
 * seed 0xff, result inverted, computed over frame bits 31..8.
 */

#define SCL3300_CRC_POLY          0x1d
#define SCL3300_CRC_SEED          0xff

/* Register addresses, bank 0 unless noted (Table 18, Table 19) */

#define SCL3300_REG_ACC_X         0x01
#define SCL3300_REG_ACC_Y         0x02
#define SCL3300_REG_ACC_Z         0x03
#define SCL3300_REG_STO           0x04
#define SCL3300_REG_TEMP          0x05
#define SCL3300_REG_STATUS        0x06
#define SCL3300_REG_ERR_FLAG1     0x07
#define SCL3300_REG_ERR_FLAG2     0x08
#define SCL3300_REG_ANG_X         0x09
#define SCL3300_REG_ANG_Y         0x0a
#define SCL3300_REG_ANG_Z         0x0b
#define SCL3300_REG_ANG_CTRL      0x0c
#define SCL3300_REG_MODE          0x0d
#define SCL3300_REG_WHOAMI        0x10
#define SCL3300_REG_SERIAL1       0x19  /* Bank 1 */
#define SCL3300_REG_SERIAL2       0x1a  /* Bank 1 */
#define SCL3300_REG_SELBANK       0x1f

/* Register values (Table 19, §6.5, §6.7, §6.9) */

#define SCL3300_ANG_CTRL_ENABLE   0x001f
#define SCL3300_MODE_CMD(m)       ((uint16_t)((m) - 1)) /* Mode 1..4 */
#define SCL3300_MODE_POWERDOWN    0x0004
#define SCL3300_MODE_SWRESET      0x0020
#define SCL3300_WAKEUP_DATA       0x0000  /* Same frame as Mode 1 */
#define SCL3300_WHOAMI_VALUE      0xc1

/* STATUS register bits (Table 26, Table 27) */

#define SCL3300_STATUS_PIN_CONT   (1 << 0)
#define SCL3300_STATUS_MODE_CHG   (1 << 1)
#define SCL3300_STATUS_PD         (1 << 2)
#define SCL3300_STATUS_MEM        (1 << 3)
#define SCL3300_STATUS_PWR        (1 << 4)
#define SCL3300_STATUS_TEMP       (1 << 5)
#define SCL3300_STATUS_SAT        (1 << 6)
#define SCL3300_STATUS_CLK        (1 << 7)
#define SCL3300_STATUS_DIGI2      (1 << 8)
#define SCL3300_STATUS_DIGI1      (1 << 9)

/* Flags that require a SW reset and a full start-up (§4.7 of the driver
 * contract, derived from Table 27).
 */

#define SCL3300_STATUS_RESET      (SCL3300_STATUS_DIGI1 | \
                                   SCL3300_STATUS_DIGI2 | \
                                   SCL3300_STATUS_CLK | \
                                   SCL3300_STATUS_MEM | \
                                   SCL3300_STATUS_PWR | \
                                   SCL3300_STATUS_PD | \
                                   SCL3300_STATUS_MODE_CHG | \
                                   SCL3300_STATUS_PIN_CONT)

/* Flags expected right after a start-up (Table 27: PWR after power-up or
 * reset, MODE_CHANGE after a mode change, PD after a wake-up).
 */

#define SCL3300_STATUS_STARTUP    (SCL3300_STATUS_PWR | \
                                   SCL3300_STATUS_MODE_CHG | \
                                   SCL3300_STATUS_PD)

/* Timing (Table 11) */

#define SCL3300_WAKEUP_US         3000
#define SCL3300_RESET_US          3000

/* Conversions.
 *
 * Angle (§2.5, §6.1.3): two's complement, deg = raw / 2^14 * 90.  Angles
 * are reported signed: Table 9 and Table 18 show an unsigned 0..360 view,
 * but §2.5 and §6.1.3 define the register as two's complement.
 *
 * Temperature (§2.4): degC = -273 + raw / 18.9.
 *
 * Acceleration (Table 2): m/s^2 = raw / sensitivity * g, with the
 * datasheet definition of g ("Definition of gravitational acceleration:
 * g = 9.819 m/s^2").
 */

#define SCL3300_G_MS2             9.819f
#define SCL3300_G_UMS2            9819    /* g in mm/s^2 */
#define SCL3300_ANGLE_FS_DEG      90
#define SCL3300_ANGLE_FS_LSB      16384

/* Self-test: number of consecutive STO samples checked (Table 23) */

#define SCL3300_STO_SAMPLES       16

/* Device info (Table 1, Table 2) */

#define SCL3300_INFO_VERSION      1
#define SCL3300_MIN_INTERVAL      500       /* us */
#define SCL3300_MAX_INTERVAL      INT32_MAX /* us, no limit when polled */

/* Consecutive failed bursts before a reset (driver policy) */

#define SCL3300_MAX_BAD_BURSTS    3

/* Minimum time between two saturation warnings */

#define SCL3300_SAT_WARN_US       1000000

#ifdef CONFIG_SENSORS_USE_B16
#  define SCL3300_DATA_UNUSED     ((b16_t)b16MIN)
#else
#  define SCL3300_DATA_UNUSED     NAN
#endif

#ifndef CONFIG_SENSORS_SCL3300_POLL_INTERVAL
#  define CONFIG_SENSORS_SCL3300_POLL_INTERVAL 10000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Per-mode parameters (Table 2, Table 12, Table 23, §2.11.1) */

struct scl3300_mode_s
{
  uint16_t sens;            /* Acceleration sensitivity, LSB/g */
  uint16_t fs;              /* Acceleration full scale, LSB */
  uint16_t settle_ms;       /* Settling time after mode change, ms */
  uint16_t sto_thr;         /* Self-test output threshold, +/-LSB */
};

struct scl3300_dev_s;

struct scl3300_sensor_s
{
  struct sensor_lowerhalf_s lower;       /* Must be first */
  FAR struct scl3300_dev_s *dev;
  uint32_t                  interval;    /* us */
  uint64_t                  last;        /* Last publish timestamp */
  bool                      enabled;
};

struct scl3300_dev_s
{
  struct scl3300_sensor_s   incl;
#ifdef CONFIG_SENSORS_SCL3300_ACCEL
  struct scl3300_sensor_s   accel;
#endif
  FAR struct spi_dev_s     *spi;
  int                       devno;
  mutex_t                   lock;        /* Serializes SPI bursts and state */
  uint8_t                   mode;        /* 1..4 */
  uint8_t                   power;       /* enum scl3300_power_e */
  bool                      powered;     /* Awake and started up */
  bool                      failed;      /* Unrecoverable device fault */
  uint8_t                   badbursts;   /* Consecutive failed bursts */
  uint8_t                   recoveries;  /* Resets without a good sample */
  uint16_t                  expected;    /* STATUS flags expected now */
  uint32_t                  satdrops;    /* Samples dropped on saturation */
  uint64_t                  satwarn;     /* Last saturation warning */
#ifdef CONFIG_SENSORS_SCL3300_POLL
  sem_t                     run;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor ops functions */

static int scl3300_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
static int scl3300_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us);
#ifndef CONFIG_SENSORS_SCL3300_POLL
static int scl3300_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen);
#endif
static int scl3300_selftest(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, unsigned long arg);
static int scl3300_get_info(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR struct sensor_device_info_s *info);
static int scl3300_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_scl3300_ops =
{
  NULL,                   /* open */
  NULL,                   /* close */
  scl3300_activate,       /* activate */
  scl3300_set_interval,   /* set_interval */
  NULL,                   /* batch */
#ifdef CONFIG_SENSORS_SCL3300_POLL
  NULL,                   /* fetch */
#else
  scl3300_fetch,          /* fetch */
#endif
  NULL,                   /* flush */
  scl3300_selftest,       /* selftest */
  NULL,                   /* set_calibvalue */
  NULL,                   /* calibrate */
  scl3300_get_info,       /* get_info */
  NULL,                   /* set_nonwakeup */
  scl3300_control,        /* control */
};

/* Indexed by mode - 1.  Sensitivities and settling times from Table 2 and
 * Table 12, STO thresholds from Table 23.  Modes 3 and 4 are inclination
 * modes valid for about +/-10 degrees of tilt only (§2.11.1); the full
 * scale reported for them is 1 g, the largest acceleration seen in a
 * static measurement.
 */

static const struct scl3300_mode_s g_scl3300_modes[] =
{
  {
    6000, 7200, 25, 1800       /* Mode 1: +/-1.2 g, 40 Hz LPF */
  },
  {
    3000, 7200, 15, 900        /* Mode 2: +/-2.4 g, 70 Hz LPF */
  },
  {
    12000, 12000, 100, 3600    /* Mode 3: inclination, 10 Hz LPF */
  },
  {
    12000, 12000, 100, 3600    /* Mode 4: inclination, low noise */
  }
};

/* Current consumption in 0.1 mA per mode (Table 1) */

static const uint8_t g_scl3300_power[] =
{
  12, 12, 12, 12
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: scl3300_crc8
 *
 * Description:
 *   Compute the CRC-8 of an SPI frame over bits 31..8 (§5.2, Table 17).
 *
 ****************************************************************************/

static uint8_t scl3300_crc8(uint32_t frame)
{
  uint8_t crc = SCL3300_CRC_SEED;
  bool in;
  bool msb;
  int bit;

  for (bit = 31; bit > 7; bit--)
    {
      in  = ((frame >> bit) & 1) != 0;
      msb = (crc & 0x80) != 0;
      crc = (uint8_t)(crc << 1);
      if (in != msb)
        {
          crc ^= SCL3300_CRC_POLY;
        }
    }

  return (uint8_t)~crc;
}

/****************************************************************************
 * Name: scl3300_frame
 *
 * Description:
 *   Build a MOSI frame (Table 14) including its CRC.
 *
 ****************************************************************************/

static uint32_t scl3300_frame(bool rw, uint8_t addr, uint16_t data)
{
  uint32_t frame;

  frame = ((uint32_t)(addr & SCL3300_FRAME_ADDR_MASK) <<
           SCL3300_FRAME_ADDR_SHIFT) | ((uint32_t)data << 8);
  if (rw)
    {
      frame |= SCL3300_FRAME_RW;
    }

  return frame | scl3300_crc8(frame);
}

/****************************************************************************
 * Name: scl3300_response_ok
 *
 * Description:
 *   Check that a MISO frame has a valid CRC and answers the request
 *   'request', i.e. echoes its OP field (§6.1.1).
 *
 ****************************************************************************/

static bool scl3300_response_ok(uint32_t response, uint32_t request)
{
  return scl3300_crc8(response) == SCL3300_FRAME_CRC(response) &&
         SCL3300_FRAME_OP(response) == SCL3300_FRAME_OP(request);
}

#ifdef CONFIG_SENSORS_USE_B16
/****************************************************************************
 * Name: scl3300_divround
 *
 * Description:
 *   Signed integer division rounded to nearest, den > 0.
 *
 ****************************************************************************/

static b16_t scl3300_divround(int64_t num, int64_t den)
{
  if (num >= 0)
    {
      return (b16_t)((num + den / 2) / den);
    }

  return (b16_t)((num - den / 2) / den);
}
#endif

/****************************************************************************
 * Name: scl3300_angle
 *
 * Description:
 *   Convert an ANG_X/Y/Z register value to degrees (§2.5, §6.1.3).
 *
 ****************************************************************************/

static sensor_data_t scl3300_angle(int16_t raw)
{
#ifdef CONFIG_SENSORS_USE_B16
  /* raw * 90 / 2^14 * 2^16 = raw * 360, exact */

  return (b16_t)raw * (SCL3300_ANGLE_FS_DEG * 65536 / SCL3300_ANGLE_FS_LSB);
#else
  return (float)raw * ((float)SCL3300_ANGLE_FS_DEG /
                       (float)SCL3300_ANGLE_FS_LSB);
#endif
}

/****************************************************************************
 * Name: scl3300_temp
 *
 * Description:
 *   Convert a TEMPERATURE register value to degrees Celsius (§2.4).
 *
 ****************************************************************************/

static sensor_data_t scl3300_temp(int16_t raw)
{
#ifdef CONFIG_SENSORS_USE_B16
  /* raw / 18.9 * 2^16 = raw * 655360 / 189 */

  return -itob16(273) + scl3300_divround((int64_t)raw * 655360, 189);
#else
  return -273.0f + (float)raw / 18.9f;
#endif
}

/****************************************************************************
 * Name: scl3300_accel
 *
 * Description:
 *   Convert an ACC_X/Y/Z register value to m/s^2 (Table 2).
 *
 ****************************************************************************/

static sensor_data_t scl3300_accel(int16_t raw, uint16_t sens)
{
#ifdef CONFIG_SENSORS_USE_B16
  return scl3300_divround((int64_t)raw * SCL3300_G_UMS2 * 65536,
                          (int64_t)sens * 1000);
#else
  return (float)raw * (SCL3300_G_MS2 / (float)sens);
#endif
}

/****************************************************************************
 * Name: scl3300_serial
 *
 * Description:
 *   Compose the serial number from SERIAL1 and SERIAL2 (§6.8).
 *
 ****************************************************************************/

static void scl3300_serial(uint16_t serial1, uint16_t serial2,
                           FAR char *buf, size_t len)
{
  snprintf(buf, len, "%" PRIu32 "B33",
           ((uint32_t)serial2 << 16) | serial1);
}

/****************************************************************************
 * Name: scl3300_xfer
 *
 * Description:
 *   Exchange 'n' frames in one SPI burst.  The protocol is off-frame
 *   (§5.1.2, Fig. 13): rx[k] answers tx[k - 1] and rx[0] answers the last
 *   frame of the previous burst.
 *
 ****************************************************************************/

static void scl3300_xfer(FAR struct scl3300_dev_s *dev,
                         FAR const uint32_t *tx, FAR uint32_t *rx, int n)
{
  uint8_t txbuf[SCL3300_FRAME_BYTES];
  uint8_t rxbuf[SCL3300_FRAME_BYTES];
  int i;

  SPI_LOCK(dev->spi, true);
  SPI_SETMODE(dev->spi, SCL3300_SPI_MODE);
  SPI_SETBITS(dev->spi, SCL3300_SPI_NBITS);
  SPI_SETFREQUENCY(dev->spi, CONFIG_SENSORS_SCL3300_SPI_FREQUENCY);

  for (i = 0; i < n; i++)
    {
      txbuf[0] = (uint8_t)(tx[i] >> 24);
      txbuf[1] = (uint8_t)(tx[i] >> 16);
      txbuf[2] = (uint8_t)(tx[i] >> 8);
      txbuf[3] = (uint8_t)tx[i];

      SPI_SELECT(dev->spi, SPIDEV_ACCELEROMETER(dev->devno), true);
      SPI_EXCHANGE(dev->spi, txbuf, rxbuf, SCL3300_FRAME_BYTES);
      SPI_SELECT(dev->spi, SPIDEV_ACCELEROMETER(dev->devno), false);

      /* Keep CS high for at least TLH before the next frame (Table 8) */

      up_udelay(SCL3300_TLH_US);

      if (rx != NULL)
        {
          rx[i] = ((uint32_t)rxbuf[0] << 24) | ((uint32_t)rxbuf[1] << 16) |
                  ((uint32_t)rxbuf[2] << 8) | rxbuf[3];
        }
    }

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: scl3300_write
 *
 * Description:
 *   Send a single write command, discarding the response.
 *
 ****************************************************************************/

static void scl3300_write(FAR struct scl3300_dev_s *dev, uint8_t addr,
                          uint16_t data)
{
  uint32_t frame = scl3300_frame(true, addr, data);

  scl3300_xfer(dev, &frame, NULL, 1);
}

/****************************************************************************
 * Name: scl3300_startup
 *
 * Description:
 *   Run the start-up sequence of Table 11 with the stored mode.  Also used
 *   after every wake-up, reset and mode change, because power-down, SW
 *   reset and power cycle all reset the written settings.
 *
 * Input Parameters:
 *   dev  - Device state, locked by the caller.
 *   wake - Send the wake-up command first.
 *
 * Returned Value:
 *   OK, -EIO if the device does not report a successful start-up or
 *   -ENODEV if WHOAMI does not match.
 *
 ****************************************************************************/

static int scl3300_startup(FAR struct scl3300_dev_s *dev, bool wake)
{
  uint32_t tx[5];
  uint32_t rx[5];
  uint16_t whoami;

  dev->powered = false;

  /* Step 1: wake-up from power-down mode */

  if (wake)
    {
      scl3300_write(dev, SCL3300_REG_MODE, SCL3300_WAKEUP_DATA);
      nxsched_usleep(SCL3300_WAKEUP_US);
    }

  /* Step 2: SW reset.  The response of the next frame is undefined and is
   * discarded below.
   */

  scl3300_write(dev, SCL3300_REG_MODE, SCL3300_MODE_SWRESET);
  nxsched_usleep(SCL3300_RESET_US);

  /* Steps 3 and 4: select the mode and enable the angle outputs */

  tx[0] = scl3300_frame(true, SCL3300_REG_MODE,
                        SCL3300_MODE_CMD(dev->mode));
  tx[1] = scl3300_frame(true, SCL3300_REG_ANG_CTRL,
                        SCL3300_ANG_CTRL_ENABLE);
  scl3300_xfer(dev, tx, NULL, 2);

  /* Step 5: wait for the output to settle */

  nxsched_usleep(g_scl3300_modes[dev->mode - 1].settle_ms * 1000);

  /* Steps 6 to 9: STATUS (clears the summary), STATUS, STATUS, WHOAMI and
   * a trailing WHOAMI that clocks out the answer to the previous one.
   */

  tx[0] = scl3300_frame(false, SCL3300_REG_STATUS, 0);
  tx[1] = tx[0];
  tx[2] = tx[0];
  tx[3] = scl3300_frame(false, SCL3300_REG_WHOAMI, 0);
  tx[4] = tx[3];
  scl3300_xfer(dev, tx, rx, 5);

  /* Step 8: the response received during the third STATUS read must
   * have RS = 01.
   */

  if (!scl3300_response_ok(rx[2], tx[1]) ||
      SCL3300_FRAME_RS(rx[2]) != SCL3300_RS_NORMAL)
    {
      snerr("ERROR: start-up failed, response %08" PRIx32 "\n", rx[2]);
      return -EIO;
    }

  /* Step 9: WHOAMI */

  whoami = SCL3300_FRAME_DATA(rx[4]) & 0xff;
  if (!scl3300_response_ok(rx[4], tx[3]) || whoami != SCL3300_WHOAMI_VALUE)
    {
      snerr("ERROR: wrong WHOAMI, response %08" PRIx32 "\n", rx[4]);
      return -ENODEV;
    }

  dev->powered   = true;
  dev->badbursts = 0;
  dev->expected  = SCL3300_STATUS_STARTUP;
  return OK;
}

/****************************************************************************
 * Name: scl3300_powerdown
 *
 * Description:
 *   Enter power-down mode (Table 19, §6.5).
 *
 ****************************************************************************/

static void scl3300_powerdown(FAR struct scl3300_dev_s *dev)
{
  scl3300_write(dev, SCL3300_REG_MODE, SCL3300_MODE_POWERDOWN);
  dev->powered = false;
}

/****************************************************************************
 * Name: scl3300_any_enabled
 ****************************************************************************/

static bool scl3300_any_enabled(FAR struct scl3300_dev_s *dev)
{
#ifdef CONFIG_SENSORS_SCL3300_ACCEL
  return dev->incl.enabled || dev->accel.enabled;
#else
  return dev->incl.enabled;
#endif
}

/****************************************************************************
 * Name: scl3300_fail
 *
 * Description:
 *   Mark the device as failed: stop publishing and log once.
 *
 ****************************************************************************/

static void scl3300_fail(FAR struct scl3300_dev_s *dev)
{
  if (!dev->failed)
    {
      snerr("ERROR: SCL3300 %d failed, device disabled\n", dev->devno);
      dev->failed = true;
    }
}

/****************************************************************************
 * Name: scl3300_recover
 *
 * Description:
 *   SW reset and full start-up after a device fault.  If the fault came
 *   back before a good sample was read since the last recovery, the reset
 *   did not clear it and the device is marked failed.
 *
 ****************************************************************************/

static void scl3300_recover(FAR struct scl3300_dev_s *dev)
{
  uint32_t tx[3];
  uint32_t rx[3];

  /* Read ERR_FLAG1 and ERR_FLAG2 for diagnostics only.  Note: the
   * ERR_FLAG1 read frame is 1C0000E3 (Table 19).
   */

  tx[0] = scl3300_frame(false, SCL3300_REG_ERR_FLAG1, 0);
  tx[1] = scl3300_frame(false, SCL3300_REG_ERR_FLAG2, 0);
  tx[2] = scl3300_frame(false, SCL3300_REG_WHOAMI, 0);
  scl3300_xfer(dev, tx, rx, 3);
  snwarn("WARNING: resetting, ERR_FLAG1 %04x ERR_FLAG2 %04x\n",
         SCL3300_FRAME_DATA(rx[1]), SCL3300_FRAME_DATA(rx[2]));

  if (dev->recoveries++ > 0 || scl3300_startup(dev, false) < 0)
    {
      scl3300_fail(dev);
    }
}

/****************************************************************************
 * Name: scl3300_check_status
 *
 * Description:
 *   Called after a burst returned RS = 11.  Reads STATUS and reacts as
 *   required by Table 27.
 *
 * Returned Value:
 *   OK if the sample may be published, -EAGAIN if it must be dropped.
 *   'temp_ok' is cleared when the temperature is out of range.
 *
 ****************************************************************************/

static int scl3300_check_status(FAR struct scl3300_dev_s *dev,
                                FAR bool *temp_ok)
{
  uint32_t tx[3];
  uint32_t rx[3];
  uint16_t status;
  uint16_t unexpected;
  uint64_t now;

  tx[0] = scl3300_frame(false, SCL3300_REG_STATUS, 0);
  tx[1] = tx[0];
  tx[2] = scl3300_frame(false, SCL3300_REG_WHOAMI, 0);
  scl3300_xfer(dev, tx, rx, 3);

  if (!scl3300_response_ok(rx[1], tx[0]))
    {
      return -EAGAIN;
    }

  status        = SCL3300_FRAME_DATA(rx[1]);
  unexpected    = status & ~dev->expected;
  dev->expected = 0;

  /* RS = 11 with a clear STATUS means the error was caused by a CRC
   * error in a MOSI frame, not by a sensor fault (§5.1.5).
   */

  if (status == 0)
    {
      return -EAGAIN;
    }

  if ((unexpected & SCL3300_STATUS_RESET) != 0)
    {
      snwarn("WARNING: STATUS %04x\n", status);
      scl3300_recover(dev);
      return -EAGAIN;
    }

  if ((status & SCL3300_STATUS_SAT) != 0)
    {
      /* Acceleration, angle and STO outputs are invalid (Table 27) */

      dev->satdrops++;
      now = sensor_get_timestamp();
      if (now - dev->satwarn >= SCL3300_SAT_WARN_US)
        {
          snwarn("WARNING: saturated, %" PRIu32 " samples dropped\n",
                 dev->satdrops);
          dev->satwarn = now;
        }

      return -EAGAIN;
    }

  if ((status & SCL3300_STATUS_TEMP) != 0)
    {
      *temp_ok = false;
    }

  return OK;
}

/****************************************************************************
 * Name: scl3300_sample
 *
 * Description:
 *   Read one sample in a single SPI burst.  Either output may be NULL.
 *
 * Returned Value:
 *   OK if the sample is valid, a negated errno value if it was dropped.
 *
 ****************************************************************************/

static int scl3300_sample(FAR struct scl3300_dev_s *dev,
                          FAR struct sensor_inclinometer *incl,
                          FAR struct sensor_accel *accel)
{
  uint32_t tx[8];
  uint32_t rx[8];
  int16_t raw[7];
  uint64_t timestamp;
  sensor_data_t temp;
  uint16_t sens;
  bool rserror = false;
  bool temp_ok = true;
  bool bad = false;
  int itemp;
  int iacc;
  int iang;
  int n = 0;
  int ret;
  int k;

  if (dev->failed)
    {
      return -EIO;
    }

  if (!dev->powered)
    {
      return -EAGAIN;
    }

  iacc = n;
  if (accel != NULL)
    {
      tx[n++] = scl3300_frame(false, SCL3300_REG_ACC_X, 0);
      tx[n++] = scl3300_frame(false, SCL3300_REG_ACC_Y, 0);
      tx[n++] = scl3300_frame(false, SCL3300_REG_ACC_Z, 0);
    }

  itemp = n;
  tx[n++] = scl3300_frame(false, SCL3300_REG_TEMP, 0);

  iang = n;
  if (incl != NULL)
    {
      tx[n++] = scl3300_frame(false, SCL3300_REG_ANG_X, 0);
      tx[n++] = scl3300_frame(false, SCL3300_REG_ANG_Y, 0);
      tx[n++] = scl3300_frame(false, SCL3300_REG_ANG_Z, 0);
    }

  /* Trailing frame that clocks out the answer to the last read */

  tx[n] = scl3300_frame(false, SCL3300_REG_WHOAMI, 0);

  scl3300_xfer(dev, tx, rx, n + 1);
  timestamp = sensor_get_timestamp();

  for (k = 0; k < n; k++)
    {
      if (!scl3300_response_ok(rx[k + 1], tx[k]))
        {
          bad = true;
          break;
        }

      switch (SCL3300_FRAME_RS(rx[k + 1]))
        {
          case SCL3300_RS_NORMAL:
            break;

          case SCL3300_RS_ERROR:
            rserror = true;
            break;

          default:

            /* Start-up in progress: drop the sample silently */

            return -EAGAIN;
        }

      raw[k] = (int16_t)SCL3300_FRAME_DATA(rx[k + 1]);
    }

  if (bad)
    {
      if (++dev->badbursts >= SCL3300_MAX_BAD_BURSTS)
        {
          snwarn("WARNING: %d consecutive bad bursts\n", dev->badbursts);
          scl3300_recover(dev);
        }

      return -EIO;
    }

  dev->badbursts = 0;

  if (rserror)
    {
      ret = scl3300_check_status(dev, &temp_ok);
      if (ret < 0)
        {
          return ret;
        }
    }
  else
    {
      dev->expected = 0;
    }

  dev->recoveries = 0;

  temp = temp_ok ? scl3300_temp(raw[itemp]) : SCL3300_DATA_UNUSED;
  sens = g_scl3300_modes[dev->mode - 1].sens;

  if (accel != NULL)
    {
      accel->timestamp   = timestamp;
      accel->x           = scl3300_accel(raw[iacc], sens);
      accel->y           = scl3300_accel(raw[iacc + 1], sens);
      accel->z           = scl3300_accel(raw[iacc + 2], sens);
      accel->temperature = temp;
    }

  if (incl != NULL)
    {
      incl->timestamp   = timestamp;
      incl->x           = scl3300_angle(raw[iang]);
      incl->y           = scl3300_angle(raw[iang + 1]);
      incl->z           = scl3300_angle(raw[iang + 2]);
      incl->temperature = temp;
    }

  return OK;
}

/****************************************************************************
 * Name: scl3300_wake
 *
 * Description:
 *   Make sure the device is awake.  'wasdown' tells whether it was
 *   powered down before, so that scl3300_restore() can put it back.
 *
 ****************************************************************************/

static int scl3300_wake(FAR struct scl3300_dev_s *dev, FAR bool *wasdown)
{
  *wasdown = !dev->powered;
  if (dev->powered)
    {
      return OK;
    }

  return scl3300_startup(dev, true);
}

/****************************************************************************
 * Name: scl3300_idle
 *
 * Description:
 *   Apply the idle power policy when no topic is active.
 *
 ****************************************************************************/

static void scl3300_idle(FAR struct scl3300_dev_s *dev)
{
  if (dev->power == SCL3300_POWER_DOWN_IDLE && !scl3300_any_enabled(dev) &&
      dev->powered)
    {
      scl3300_powerdown(dev);
    }
}

/****************************************************************************
 * Name: scl3300_restore
 *
 * Description:
 *   Undo scl3300_wake().
 *
 ****************************************************************************/

static void scl3300_restore(FAR struct scl3300_dev_s *dev, bool wasdown)
{
  if (wasdown && !scl3300_any_enabled(dev) && dev->powered)
    {
      scl3300_powerdown(dev);
    }
}

/****************************************************************************
 * Name: scl3300_activate
 ****************************************************************************/

static int scl3300_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct scl3300_sensor_s *sensor = (FAR struct scl3300_sensor_s *)lower;
  FAR struct scl3300_dev_s *dev = sensor->dev;
  bool wasidle;
  int ret = OK;

  nxmutex_lock(&dev->lock);

  wasidle = !scl3300_any_enabled(dev);

  if (enable)
    {
      if (dev->failed)
        {
          ret = -EIO;
          goto out;
        }

      if (!dev->powered)
        {
          ret = scl3300_startup(dev, true);
          if (ret < 0)
            {
              goto out;
            }

          dev->recoveries = 0;
        }

      sensor->last    = 0;
      sensor->enabled = true;

#ifdef CONFIG_SENSORS_SCL3300_POLL
      if (wasidle)
        {
          nxsem_post(&dev->run);
        }
#endif
    }
  else
    {
      sensor->enabled = false;
      scl3300_idle(dev);
    }

out:
  nxmutex_unlock(&dev->lock);
  UNUSED(wasidle);
  return ret;
}

/****************************************************************************
 * Name: scl3300_set_interval
 ****************************************************************************/

static int scl3300_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
  FAR struct scl3300_sensor_s *sensor = (FAR struct scl3300_sensor_s *)lower;
  FAR struct scl3300_dev_s *dev = sensor->dev;

  if (*period_us < SCL3300_MIN_INTERVAL)
    {
      *period_us = SCL3300_MIN_INTERVAL;
    }

  nxmutex_lock(&dev->lock);
  sensor->interval = *period_us;
  nxmutex_unlock(&dev->lock);

  return OK;
}

#ifndef CONFIG_SENSORS_SCL3300_POLL
/****************************************************************************
 * Name: scl3300_fetch
 ****************************************************************************/

static int scl3300_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen)
{
  FAR struct scl3300_sensor_s *sensor = (FAR struct scl3300_sensor_s *)lower;
  FAR struct scl3300_dev_s *dev = sensor->dev;
  bool wasdown;
  int ret;

  if (buflen < (lower->type == SENSOR_TYPE_INCLINOMETER ?
                 sizeof(struct sensor_inclinometer) :
                 sizeof(struct sensor_accel)))
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  if (dev->failed)
    {
      ret = -EIO;
      goto out;
    }

  ret = scl3300_wake(dev, &wasdown);
  if (ret < 0)
    {
      goto out;
    }

#ifdef CONFIG_SENSORS_SCL3300_ACCEL
  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      ret = scl3300_sample(dev, NULL, (FAR struct sensor_accel *)buffer);
      if (ret >= 0)
        {
          ret = sizeof(struct sensor_accel);
        }
    }
  else
#endif
    {
      ret = scl3300_sample(dev, (FAR struct sensor_inclinometer *)buffer,
                           NULL);
      if (ret >= 0)
        {
          ret = sizeof(struct sensor_inclinometer);
        }
    }

  scl3300_restore(dev, wasdown);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}
#endif

/****************************************************************************
 * Name: scl3300_selftest
 *
 * Description:
 *   Check WHOAMI, check that STATUS reports no fault and that
 *   SCL3300_STO_SAMPLES consecutive self-test outputs are within the
 *   threshold of the current mode (Table 23).
 *
 ****************************************************************************/

static int scl3300_selftest(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, unsigned long arg)
{
  FAR struct scl3300_sensor_s *sensor = (FAR struct scl3300_sensor_s *)lower;
  FAR struct scl3300_dev_s *dev = sensor->dev;
  uint32_t tx[SCL3300_STO_SAMPLES + 4];
  uint32_t rx[SCL3300_STO_SAMPLES + 4];
  int16_t thr;
  int16_t sto;
  bool wasdown;
  int ret;
  int k;

  nxmutex_lock(&dev->lock);

  if (dev->failed)
    {
      ret = -EIO;
      goto out;
    }

  ret = scl3300_wake(dev, &wasdown);
  if (ret < 0)
    {
      ret = -EIO;
      goto out;
    }

  /* WHOAMI, STATUS (clears the summary), STATUS, STO x N, trailer */

  tx[0] = scl3300_frame(false, SCL3300_REG_WHOAMI, 0);
  tx[1] = scl3300_frame(false, SCL3300_REG_STATUS, 0);
  tx[2] = tx[1];
  for (k = 0; k < SCL3300_STO_SAMPLES; k++)
    {
      tx[3 + k] = scl3300_frame(false, SCL3300_REG_STO, 0);
    }

  tx[3 + k] = tx[0];
  scl3300_xfer(dev, tx, rx, SCL3300_STO_SAMPLES + 4);

  ret = OK;

  if (!scl3300_response_ok(rx[1], tx[0]) ||
      (SCL3300_FRAME_DATA(rx[1]) & 0xff) != SCL3300_WHOAMI_VALUE)
    {
      snerr("ERROR: self-test WHOAMI %08" PRIx32 "\n", rx[1]);
      ret = -EIO;
    }

  if (!scl3300_response_ok(rx[3], tx[2]) ||
      (SCL3300_FRAME_DATA(rx[3]) &
       (SCL3300_STATUS_RESET | SCL3300_STATUS_SAT)) != 0)
    {
      snerr("ERROR: self-test STATUS %08" PRIx32 "\n", rx[3]);
      ret = -EIO;
    }

  thr = (int16_t)g_scl3300_modes[dev->mode - 1].sto_thr;
  for (k = 0; k < SCL3300_STO_SAMPLES; k++)
    {
      sto = (int16_t)SCL3300_FRAME_DATA(rx[4 + k]);
      if (!scl3300_response_ok(rx[4 + k], tx[3 + k]) ||
          SCL3300_FRAME_RS(rx[4 + k]) != SCL3300_RS_NORMAL ||
          sto > thr || sto < -thr)
        {
          snerr("ERROR: self-test STO %08" PRIx32 "\n", rx[4 + k]);
          ret = -EIO;
          break;
        }
    }

  scl3300_restore(dev, wasdown);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: scl3300_get_info
 ****************************************************************************/

static int scl3300_get_info(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR struct sensor_device_info_s *info)
{
  FAR struct scl3300_sensor_s *sensor = (FAR struct scl3300_sensor_s *)lower;
  FAR struct scl3300_dev_s *dev = sensor->dev;
  FAR const struct scl3300_mode_s *mode;

  nxmutex_lock(&dev->lock);
  mode = &g_scl3300_modes[dev->mode - 1];

  memset(info, 0, sizeof(*info));
  info->version   = SCL3300_INFO_VERSION;
  info->power     = sensor_data_divi(
                      sensor_data_itof(g_scl3300_power[dev->mode - 1]), 10);
  info->min_delay = SCL3300_MIN_INTERVAL;
  info->max_delay = SCL3300_MAX_INTERVAL;
  strlcpy(info->name, "SCL3300", sizeof(info->name));
  strlcpy(info->vendor, "Murata", sizeof(info->vendor));

  if (lower->type == SENSOR_TYPE_INCLINOMETER)
    {
      info->max_range  = sensor_data_itof(SCL3300_ANGLE_FS_DEG);
      info->resolution = scl3300_angle(1);
    }
  else
    {
      info->max_range  = scl3300_accel((int16_t)mode->fs, mode->sens);
      info->resolution = scl3300_accel(1, mode->sens);
    }

  nxmutex_unlock(&dev->lock);
  return OK;
}

/****************************************************************************
 * Name: scl3300_control
 ****************************************************************************/

static int scl3300_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct scl3300_sensor_s *sensor = (FAR struct scl3300_sensor_s *)lower;
  FAR struct scl3300_dev_s *dev = sensor->dev;
  FAR uint8_t *whoami;
  uint32_t tx[2];
  uint32_t rx[2];
  bool wasdown;
  int ret = OK;

  nxmutex_lock(&dev->lock);

  switch (cmd)
    {
      /* Arg: enum scl3300_mode_e by value */

      case SNIOC_SET_OPERATIONAL_MODE:
        if (arg < SCL3300_MODE_1 || arg > SCL3300_MODE_4)
          {
            ret = -EINVAL;
            break;
          }

        dev->mode = (uint8_t)arg;
        if (dev->powered)
          {
            ret = scl3300_startup(dev, false);
            if (ret < 0)
              {
                scl3300_fail(dev);
              }
          }
        break;

      /* Arg: enum scl3300_power_e by value */

      case SNIOC_SET_POWER_MODE:
        if (arg != SCL3300_POWER_ALWAYS_ON && arg != SCL3300_POWER_DOWN_IDLE)
          {
            ret = -EINVAL;
            break;
          }

        dev->power = (uint8_t)arg;
        if (dev->power == SCL3300_POWER_ALWAYS_ON)
          {
            if (!dev->powered && !dev->failed)
              {
                ret = scl3300_startup(dev, true);
                if (ret < 0)
                  {
                    scl3300_fail(dev);
                  }
              }
          }
        else
          {
            scl3300_idle(dev);
          }
        break;

      /* Arg: none.  Also clears a failed state if the start-up succeeds. */

      case SNIOC_RESET:
        ret = scl3300_startup(dev, true);
        if (ret >= 0)
          {
            dev->failed     = false;
            dev->recoveries = 0;
            scl3300_idle(dev);
          }
        else
          {
            scl3300_fail(dev);
          }
        break;

      /* Arg: FAR uint8_t * */

      case SNIOC_WHO_AM_I:
        whoami = (FAR uint8_t *)(uintptr_t)arg;
        if (whoami == NULL)
          {
            ret = -EINVAL;
            break;
          }

        if (dev->failed)
          {
            ret = -EIO;
            break;
          }

        ret = scl3300_wake(dev, &wasdown);
        if (ret < 0)
          {
            break;
          }

        tx[0] = scl3300_frame(false, SCL3300_REG_WHOAMI, 0);
        tx[1] = tx[0];
        scl3300_xfer(dev, tx, rx, 2);
        if (scl3300_response_ok(rx[1], tx[0]))
          {
            *whoami = SCL3300_FRAME_DATA(rx[1]) & 0xff;
          }
        else
          {
            ret = -EIO;
          }

        scl3300_restore(dev, wasdown);
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: scl3300_read_serial
 *
 * Description:
 *   Read and log the serial number (§6.8).  Note: §6.8 refers to "6.5 CMD"
 *   for the bank switch, but the bank is selected with SELBANK (§6.9).
 *   The switch back to bank 0 is part of the same burst, so bank 1 is
 *   never left selected.
 *
 ****************************************************************************/

static void scl3300_read_serial(FAR struct scl3300_dev_s *dev)
{
  uint32_t tx[5];
  uint32_t rx[5];
  char serial[24];

  tx[0] = scl3300_frame(true, SCL3300_REG_SELBANK, 1);
  tx[1] = scl3300_frame(false, SCL3300_REG_SERIAL1, 0);
  tx[2] = scl3300_frame(false, SCL3300_REG_SERIAL2, 0);
  tx[3] = scl3300_frame(true, SCL3300_REG_SELBANK, 0);
  tx[4] = scl3300_frame(false, SCL3300_REG_WHOAMI, 0);
  scl3300_xfer(dev, tx, rx, 5);

  if (scl3300_response_ok(rx[2], tx[1]) &&
      scl3300_response_ok(rx[3], tx[2]))
    {
      scl3300_serial(SCL3300_FRAME_DATA(rx[2]), SCL3300_FRAME_DATA(rx[3]),
                     serial, sizeof(serial));
      sninfo("SCL3300 %d: WHOAMI ok, serial number %s\n", dev->devno,
             serial);
    }

  UNUSED(serial);
}

#ifdef CONFIG_SENSORS_SCL3300_POLL
/****************************************************************************
 * Name: scl3300_due
 *
 * Description:
 *   Tell whether a topic must be published now.  'slack' absorbs the
 *   wake-up jitter of the polling thread.
 *
 ****************************************************************************/

static bool scl3300_due(FAR struct scl3300_sensor_s *sensor, uint64_t now,
                        uint32_t slack)
{
  return sensor->enabled &&
         (sensor->last == 0 || now + slack >= sensor->last +
                                              sensor->interval);
}

/****************************************************************************
 * Name: scl3300_thread
 *
 * Description:
 *   Polling thread.  Sleeps for the shortest interval of the active topics
 *   and publishes each topic when its own interval has elapsed.
 *
 ****************************************************************************/

static int scl3300_thread(int argc, FAR char **argv)
{
  FAR struct scl3300_dev_s *dev =
    (FAR struct scl3300_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  struct sensor_inclinometer incl;
  FAR struct sensor_inclinometer *pincl;
#ifdef CONFIG_SENSORS_SCL3300_ACCEL
  struct sensor_accel accel;
#endif
  FAR struct sensor_accel *paccel;
  struct timespec ts;
  uint32_t interval;
  uint64_t next = 0;
  uint64_t now;
  int ret;

  while (true)
    {
      nxmutex_lock(&dev->lock);

      if (!scl3300_any_enabled(dev))
        {
          nxmutex_unlock(&dev->lock);
          nxsem_wait_uninterruptible(&dev->run);
          next = 0;
          continue;
        }

      interval = UINT32_MAX;
      if (dev->incl.enabled)
        {
          interval = dev->incl.interval;
        }

#ifdef CONFIG_SENSORS_SCL3300_ACCEL
      if (dev->accel.enabled && dev->accel.interval < interval)
        {
          interval = dev->accel.interval;
        }
#endif

      now    = sensor_get_timestamp();
      pincl  = scl3300_due(&dev->incl, now, interval / 2) ? &incl : NULL;
      paccel = NULL;
#ifdef CONFIG_SENSORS_SCL3300_ACCEL
      if (scl3300_due(&dev->accel, now, interval / 2))
        {
          paccel = &accel;
        }
#endif

      ret = -EAGAIN;
      if (pincl != NULL || paccel != NULL)
        {
          ret = scl3300_sample(dev, pincl, paccel);
          if (ret >= 0 && pincl != NULL)
            {
              dev->incl.last = incl.timestamp;
            }

#ifdef CONFIG_SENSORS_SCL3300_ACCEL
          if (ret >= 0 && paccel != NULL)
            {
              dev->accel.last = accel.timestamp;
            }
#endif
        }

      nxmutex_unlock(&dev->lock);

      /* Publish without holding dev->lock: the upper half calls
       * activate() with its own lock held, and push_event() takes it.
       */

      if (ret >= 0 && pincl != NULL)
        {
          dev->incl.lower.push_event(dev->incl.lower.priv, &incl,
                                     sizeof(incl));
        }

#ifdef CONFIG_SENSORS_SCL3300_ACCEL
      if (ret >= 0 && paccel != NULL)
        {
          dev->accel.lower.push_event(dev->accel.lower.priv, &accel,
                                      sizeof(accel));
        }
#endif

      /* Wait for an absolute deadline, so that neither the time spent
       * above nor the tick rounding of a relative sleep accumulates.  If
       * a whole period was missed, restart from now instead of catching
       * up.  An activation posts 'run' and ends the wait early.
       */

      next += interval;
      if (next <= now)
        {
          next = now + interval;
        }

      ts.tv_sec  = next / USEC_PER_SEC;
      ts.tv_nsec = (next % USEC_PER_SEC) * NSEC_PER_USEC;
      nxsem_clockwait_uninterruptible(&dev->run, CLOCK_MONOTONIC, &ts);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: scl3300_init_sensor
 ****************************************************************************/

static void scl3300_init_sensor(FAR struct scl3300_dev_s *dev,
                                FAR struct scl3300_sensor_s *sensor,
                                int type)
{
  sensor->lower.ops     = &g_scl3300_ops;
  sensor->lower.type    = type;
  sensor->lower.nbuffer = 1;
  sensor->dev           = dev;
  sensor->interval      = CONFIG_SENSORS_SCL3300_POLL_INTERVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: scl3300_register
 *
 * Description:
 *   Register the Murata SCL3300 inclinometer as the uORB topics
 *   sensor_inclinometer<devno> and, if CONFIG_SENSORS_SCL3300_ACCEL is
 *   enabled, sensor_accel<devno>.  The chip select used is
 *   SPIDEV_ACCELEROMETER(devno).
 *
 * Input Parameters:
 *   devno  - Instance number of the uORB topics and of the chip select.
 *   spi    - An instance of the SPI interface to use to communicate with
 *            the SCL3300.
 *   config - Platform data, copied by the driver.  NULL selects Mode 1
 *            and SCL3300_POWER_ALWAYS_ON.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int scl3300_register(int devno, FAR struct spi_dev_s *spi,
                     FAR const struct scl3300_config_s *config)
{
  FAR struct scl3300_dev_s *dev;
  uint8_t mode = SCL3300_MODE_1;
  uint8_t power = SCL3300_POWER_ALWAYS_ON;
#ifdef CONFIG_SENSORS_SCL3300_POLL
  FAR char *argv[2];
  char arg1[32];
#endif
  int ret;

  DEBUGASSERT(spi != NULL);

  if (config != NULL)
    {
      if (config->mode > SCL3300_MODE_4 ||
          (config->power != SCL3300_POWER_ALWAYS_ON &&
           config->power != SCL3300_POWER_DOWN_IDLE))
        {
          return -EINVAL;
        }

      if (config->mode != 0)
        {
          mode = config->mode;
        }

      power = config->power;
    }

  dev = kmm_zalloc(sizeof(struct scl3300_dev_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->spi   = spi;
  dev->devno = devno;
  dev->mode  = mode;
  dev->power = power;
  nxmutex_init(&dev->lock);

  scl3300_init_sensor(dev, &dev->incl, SENSOR_TYPE_INCLINOMETER);
#ifdef CONFIG_SENSORS_SCL3300_ACCEL
  scl3300_init_sensor(dev, &dev->accel, SENSOR_TYPE_ACCELEROMETER);
#endif

  /* The device may have been left in power-down by a previous run, so
   * always start with a wake-up.
   */

  ret = scl3300_startup(dev, true);
  if (ret < 0)
    {
      goto err_dev;
    }

  scl3300_read_serial(dev);
  scl3300_idle(dev);

  ret = sensor_register(&dev->incl.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: failed to register inclinometer: %d\n", ret);
      goto err_dev;
    }

#ifdef CONFIG_SENSORS_SCL3300_ACCEL
  ret = sensor_register(&dev->accel.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: failed to register accelerometer: %d\n", ret);
      goto err_incl;
    }
#endif

#ifdef CONFIG_SENSORS_SCL3300_POLL
  nxsem_init(&dev->run, 0, 0);

  snprintf(arg1, sizeof(arg1), "%p", dev);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("scl3300_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_SCL3300_THREAD_STACKSIZE,
                       scl3300_thread, argv);
  if (ret < 0)
    {
      snerr("ERROR: failed to create the poll thread: %d\n", ret);
      goto err_sem;
    }
#endif

  return OK;

#ifdef CONFIG_SENSORS_SCL3300_POLL
err_sem:
  nxsem_destroy(&dev->run);
#  ifdef CONFIG_SENSORS_SCL3300_ACCEL
  sensor_unregister(&dev->accel.lower, devno);
#  endif
#endif
#ifdef CONFIG_SENSORS_SCL3300_ACCEL
err_incl:
#endif
  sensor_unregister(&dev->incl.lower, devno);
err_dev:
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
  return ret;
}
