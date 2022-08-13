/****************************************************************************
 * drivers/sensors/lsm6dso.c
 * Character driver for lsm6dso.
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

#include <stdio.h>
#include <stdlib.h>
#include <debug.h>
#include <errno.h>
#include <math.h>
#include <assert.h>

#include <nuttx/nuttx.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>
#include <nuttx/sensors/lsm6dso.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Roundup func define */

#define LSM6DSO_ROUNDUP(x, esize) ((x + (esize - 1)) / (esize)) * (esize)

#define LSM6DSO_WAIT_COUNT_MAX    300        /* Max count wait for reset */
#define LSM6DSO_SET_DELAY         10         /* Delay after set(ms) */
#define LSM6DSO_READ_DELAY        10         /* Selftest data read delay (ms) */
#define LSM6DSO_SPI_MAX_BUFFER    10         /* SPI exchange buffer size */
#define LSM6DSO_VECTOR_REMAP      2          /* Vector remap of lsm6dso */
#define LSM6DSO_ODR_FLT_EPSILON   0.1f       /* ODR float epsilon */

/* Multi sensor index */

#define LSM6DSO_XL_IDX            0          /* Accelerator index */
#define LSM6DSO_GY_IDX            1          /* Gyroscope index */
#define LSM6DSO_FSM_IDX           2          /* FSM index */
#define LSM6DSO_IDX_NUM           3          /* Max index */

/* Self-testmode selection */

#define LSM6DSO_NORMAL_MODE       0x00       /* Normal mode*/
#define LSM6DSO_POSIGN_MODE       0x01       /* Positive mode */
#define LSM6DSO_NESIGN_MODE       0x11       /* Negative mode */

/* Sensor ODR */

#define LSM6DSO_UNIT_TIME         1000000.0f /* Unit time 1000000us */

#define LSM6DSO_XL_ODR_OFF        0x00       /* Accelerator OFF */
#define LSM6DSO_XL_ODR_12p5HZ     0x01       /* Accelerator ODR 12.5Hz */
#define LSM6DSO_XL_ODR_26HZ       0x02       /* Accelerator ODR 26Hz */
#define LSM6DSO_XL_ODR_52HZ       0x03       /* Accelerator ODR 52Hz */
#define LSM6DSO_XL_ODR_104HZ      0x04       /* Accelerator ODR 104Hz */
#define LSM6DSO_XL_ODR_208HZ      0x05       /* Accelerator ODR 208Hz */
#define LSM6DSO_XL_ODR_416HZ      0x06       /* Accelerator ODR 416Hz */
#define LSM6DSO_XL_ODR_833HZ      0x07       /* Accelerator ODR 833Hz */

#define LSM6DSO_GY_ODR_OFF        0x00       /* Gyroscope OFF */
#define LSM6DSO_GY_ODR_12p5HZ     0x01       /* Gyroscope ODR 12.5Hz */
#define LSM6DSO_GY_ODR_26HZ       0x02       /* Gyroscope ODR 26Hz */
#define LSM6DSO_GY_ODR_52HZ       0x03       /* Gyroscope ODR 52Hz */
#define LSM6DSO_GY_ODR_104HZ      0x04       /* Gyroscope ODR 104Hz */
#define LSM6DSO_GY_ODR_208HZ      0x05       /* Gyroscope ODR 208Hz */
#define LSM6DSO_GY_ODR_416HZ      0x06       /* Gyroscope ODR 416Hz */
#define LSM6DSO_GY_ODR_833HZ      0x07       /* Gyroscope ODR 833Hz */

#define LSM6DSO_FSM_ODR_12p5HZ    0x00       /* FSM ODR 12.5Hz */
#define LSM6DSO_FSM_ODR_26HZ      0x01       /* FSM ODR 26Hz */
#define LSM6DSO_FSM_ODR_52HZ      0x02       /* FSM ODR 52Hz */
#define LSM6DSO_FSM_ODR_104HZ     0x03       /* FSM ODR 104Hz */

/* Sensor batch BDR */

#define LSM6DSO_XL_BDR_OFF        0x00       /* Accelerator batch OFF */
#define LSM6DSO_XL_BDR_12p5Hz     0x01       /* Accelerator batch 12.5Hz */
#define LSM6DSO_XL_BDR_26Hz       0x02       /* Accelerator batch 26Hz */
#define LSM6DSO_XL_BDR_52Hz       0x03       /* Accelerator batch 52Hz */
#define LSM6DSO_XL_BDR_104Hz      0x04       /* Accelerator batch 104Hz */
#define LSM6DSO_XL_BDR_208Hz      0x05       /* Accelerator batch 208Hz */
#define LSM6DSO_XL_BDR_417Hz      0x06       /* Accelerator batch 417Hz */
#define LSM6DSO_XL_BDR_833Hz      0x07       /* Accelerator batch 833Hz */

#define LSM6DSO_GY_BDR_OFF        0x00       /* Gyroscope batch OFF */
#define LSM6DSO_GY_BDR_12p5Hz     0x01       /* Gyroscope batch 12.5Hz */
#define LSM6DSO_GY_BDR_26Hz       0x02       /* Gyroscope batch 26Hz */
#define LSM6DSO_GY_BDR_52Hz       0x03       /* Gyroscope batch 52Hz */
#define LSM6DSO_GY_BDR_104Hz      0x04       /* Gyroscope batch 104Hz */
#define LSM6DSO_GY_BDR_208Hz      0x05       /* Gyroscope batch 208Hz */
#define LSM6DSO_GY_BDR_417Hz      0x06       /* Gyroscope batch 417Hz */
#define LSM6DSO_GY_BDR_833Hz      0x07       /* Gyroscope batch 833Hz */

/* Sensor scale */

#define LSM6DSO_2G                0x00       /* Accelerator scale 2g */
#define LSM6DSO_4G                0x02       /* Accelerator scale 4g */
#define LSM6DSO_8G                0x03       /* Accelerator scale 8g */
#define LSM6DSO_16G               0x01       /* Accelerator scale 16g */

#define LSM6DSO_2G_FACTOR         0.061f     /* Accelerator 2g factor (mg) */
#define LSM6DSO_4G_FACTOR         0.122f     /* Accelerator 4g factor (mg) */
#define LSM6DSO_8G_FACTOR         0.244f     /* Accelerator 8g factor (mg) */
#define LSM6DSO_16G_FACTOR        0.488f     /* Accelerator 16g factor (mg) */
#define LSM6DSO_MG2MS_FACTOR      0.0098f    /* Convert mg to m/s² factor */

#define LSM6DSO_125DPS            0x01       /* Gyroscope scale 125dps */
#define LSM6DSO_250DPS            0x00       /* Gyroscope scale 250dps */
#define LSM6DSO_500DPS            0x02       /* Gyroscope scale 500dps */
#define LSM6DSO_1000DPS           0x04       /* Gyroscope scale 1000dps */
#define LSM6DSO_2000DPS           0x06       /* Gyroscope scale 2000dps */

#define LSM6DSO_125DPS_FACTOR     4.375f     /* Gyroscope 125dps factor (mdps/LSB) */
#define LSM6DSO_250DPS_FACTOR     8.75f      /* Gyroscope 250dps factor (mdps/LSB) */
#define LSM6DSO_500DPS_FACTOR     17.5f      /* Gyroscope 500dps factor (mdps/LSB) */
#define LSM6DSO_1000DPS_FACTOR    35.0f      /* Gyroscope 1000dps factor (mdps/LSB) */
#define LSM6DSO_2000DPS_FACTOR    70.0f      /* Gyroscope 2000dps factor (mdps/LSB) */
#define LSM6DSO_MDPS2DPS_FACTOR   0.001f     /* Convert mdps to dps factor */
#define LSM6DSO_DPS2RPS_FACTOR    (M_PI/180) /* Convert dps to rad/s factor */

/* IO control command. */

#define LSM6DSO_FSM_MANAGE_CMD    0xf1       /* Finite state machine manage command */
#define LSM6DSO_SET_SCALE_XL_CMD  0xf2       /* Set accelerator scale command */

#define LSM6DSO_XL_SET_2G         2          /* Accelerometer set 2G */
#define LSM6DSO_XL_SET_4G         4          /* Accelerometer set 4G */
#define LSM6DSO_XL_SET_8G         8          /* Accelerometer set 8G */
#define LSM6DSO_XL_SET_16G        16         /* Accelerometer set 16G */

/* Factory test instructions. */

#define LSM6DSO_SIMPLE_CHECK      0x00       /* Simple check */
#define LSM6DSO_FULL_CHECK        0x01       /* Full check */

/* Self test results. */

#define LSM6DSO_ST_PASS           0          /* Pass self test */
#define LSM6DSO_ST_FAIL           -1         /* Failed self test */

/* Self test limits. */

#define LSM6DSO_MIN_ST_LIMIT_MG   50.0f      /* Accelerator min limit */
#define LSM6DSO_MAX_ST_LIMIT_MG   1700.0f    /* Accelerator max limit */
#define LSM6DSO_MIN_ST_LIMIT_MDPS 150000.0f  /* Gyroscope min limit */
#define LSM6DSO_MAX_ST_LIMIT_MDPS 700000.0f  /* Gyroscope max limit */

/* Sensor device info */

#define LSM6DSO_DEVICE_ID         0x6c       /* Device Identification */
#define LSM6DSO_DEFAULT_INTERVAL  40000      /* Default conversion interval(us) */
#define LSM6DSO_WAITBOOT_TIME     10         /* Wait Sensor boot time(ms) */

/* I3C status */

#define LSM6DSO_I3C_DISABLE       0x80       /* Disable the I3C */

/* Block data update */

#define LSM6DSO_BDU_CONTINUOUS    0x00       /* Continuous update */
#define LSM6DSO_BDU_UNTILREAD     0x01       /* Update until data read */

/* Timestamp counter */

#define LSM6DSO_TIMESTAMP_DISABLE 0x00       /* Disabled timestamp */
#define LSM6DSO_TIMESTAMP_ENABLE  0x01       /* Enable timestamp */

/* High-performance operating mode disable for accelerometer */

#define LSM6DSO_HIGHT_ENABLE      0x00       /* High-performance enable */
#define LSM6DSO_HIGHT_DISABLE     0x01       /* High-performance disable */

/* Interrupt set */

#define LSM6DSO_XL_INT_ENABLE     0x01       /* Accel interrupt enable */
#define LSM6DSO_XL_INT_DISABLE    0x00       /* Accel interrupt disable */

#define LSM6DSO_GY_INT_ENABLE     0x01       /* Gyro interrupt enable */
#define LSM6DSO_GY_INT_DISABLE    0x00       /* Gyro interrupt disable */

/* Interrupt set */

#define LSM6DSO_FIFO_INT_ENABLE   0x01       /* FIFO interrupt enable */
#define LSM6DSO_FIFO_INT_DISABLE  0x00       /* FIFO interrupt disable */

/* Batch mode */

#define LSM6DSO_BYPASS_MODE       0x00       /* Batch bypass mode */
#define LSM6DSO_FIFO_MODE         0x01       /* Batch FIFO mode */
#define LSM6DSO_STREAM_MODE       0x06       /* Batch stream mode */

/* FIFO output data tag */

#define LSM6DSO_GYRO_NC_TAG       0x01       /* Gyro uncompressed data */
#define LSM6DSO_XL_NC_TAG         0x02       /* Accel uncompressed data */

/* CFG access mode */

#define LSM6DSO_USER_BANK         0x00       /* User bank */
#define LSM6DSO_SENSOR_HUB_BANK   0x01       /* Sensor hub bank */
#define LSM6DSO_EMBEDDEDFUNC_BANK 0x02       /* Embedded func bank */

/* Finite state machine define */

#define LSM6DSO_PG_WRITE_ENABLE   0x02       /* Page write enable */
#define LSM6DSO_PG_WRITE_DISABLE  0x00       /* Page write disable */
#define LSM6DSO_LONGCNT_RESET     0x00       /* Reset Long Counter */
#define LSM6DSO_START_FSM_ADD     0x0400     /* FSM start address */
#define LSM6DSO_FSM_PROGRAMNUM    0x03       /* FSM program number */
#define LSM6DSO_FSM_ODR_CFG_NU1   0x03       /* FSM odr config register not used bit */
#define LSM6DSO_FSM_ODR_CFG_NU2   0x02       /* FSM odr config register not used bit */
#define LSM6DSO_FSM_PAGE_SEL_NU   0x01       /* FSM page sel config register not used bit */
#define LSM6DSO_DEFAULT_FSM_EN    0x0001lu   /* Default FSM enable */

/* Property status */

#define LSM6DSO_PROPERTY_DISABLE  0x00       /* Property disable */
#define LSM6DSO_PROPERTY_ENABLE   0x01       /* Property enable */

/* Finite state machine index */

#define LSM6DSO_FSM_INDEX1        0x0001     /* Final state machine 1 index */
#define LSM6DSO_FSM_INDEX2        0x0002     /* Final state machine 2 index */
#define LSM6DSO_FSM_INDEX3        0x0004     /* Final state machine 3 index */
#define LSM6DSO_FSM_INDEX4        0x0008     /* Final state machine 4 index */
#define LSM6DSO_FSM_INDEX5        0x0010     /* Final state machine 5 index */
#define LSM6DSO_FSM_INDEX6        0x0020     /* Final state machine 6 index */
#define LSM6DSO_FSM_INDEX7        0x0040     /* Final state machine 7 index */
#define LSM6DSO_FSM_INDEX8        0x0080     /* Final state machine 8 index */
#define LSM6DSO_FSM_INDEX9        0x0100     /* Final state machine 9 index */
#define LSM6DSO_FSM_INDEX10       0x0200     /* Final state machine 10 index */
#define LSM6DSO_FSM_INDEX11       0x0400     /* Final state machine 11 index */
#define LSM6DSO_FSM_INDEX12       0x0800     /* Final state machine 12 index */
#define LSM6DSO_FSM_INDEX13       0x1000     /* Final state machine 13 index */
#define LSM6DSO_FSM_INDEX14       0x2000     /* Final state machine 14 index */
#define LSM6DSO_FSM_INDEX15       0x4000     /* Final state machine 15 index */
#define LSM6DSO_FSM_INDEX16       0x8000     /* Final state machine 16 index */

/* Device Register */

#define LSM6DSO_FIFO_CTRL1        0x07       /* FIFO control register (r/w) */
#define LSM6DSO_FIFO_CTRL2        0x08       /* FIFO control register (r/w) */
#define LSM6DSO_FIFO_CTRL3        0x09       /* FIFO control register (r/w) */
#define LSM6DSO_FIFO_CTRL4        0x0a       /* FIFO control register (r/w) */
#define LSM6DSO_COUNTER_BDR_REG1  0x0b       /* DataReady configuration register (r/w)*/
#define LSM6DSO_INT1_CTRL         0x0d       /* INT1 pad control register (r/w) */
#define LSM6DSO_INT2_CTRL         0x0e       /* INT2 pad control register (r/w). */
#define LSM6DSO_WHO_AM_I          0x0f       /* Who_AM_I register (r). This register is a read-only register */
#define LSM6DSO_CTRL1_XL          0x10       /* Linear acceleration sensor control register 1 (r/w) */
#define LSM6DSO_CTRL3_C           0x12       /* Control register 3 (r/w) */
#define LSM6DSO_CTRL2_G           0x11       /* Angular rate sensor control register 2 (r/w) */
#define LSM6DSO_CTRL6_C           0x15       /* Angular rate sensor control register 6 (r/w) */
#define LSM6DSO_CTRL4_C           0x13       /* Control register 4 (r/w) */
#define LSM6DSO_CTRL5_C           0x14       /* Control register 5 (r/w) */
#define LSM6DSO_CTRL7_G           0x16       /* Angular rate sensor control register 7 (r/w) */
#define LSM6DSO_CTRL10_C          0x19       /* Control register 10 (r/w) */
#define LSM6DSO_I3C_BUS_AVB       0x62       /* I3C bus available register */
#define LSM6DSO_CTRL9_XL          0x18       /* Linear acceleration sensor control register 9 (r/w) */
#define LSM6DSO_CTRL8_XL          0x17       /* Linear acceleration sensor control register 8 (r/w) */
#define LSM6DSO_STATUS_REG        0x1e       /* The STATUS_REG register is read by the SPI/I2C interface (r) */
#define LSM6DSO_FSM_ENABLE_A      0x46       /* FSM enable register (r/w) */
#define LSM6DSO_FSM_ENABLE_B      0x47       /* FSM enable register (r/w) */
#define LSM6DSO_FIFO_DATA_OUT_TAG 0x78       /* FIFO out data tag register (r) */
#define LSM6DSO_FIFO_DATA_OUT_X_L 0x79       /* FIFO out data x low register (r) */
#define LSM6DSO_FIFO_DATA_OUT_X_H 0x7a       /* FIFO out data x high register (r) */
#define LSM6DSO_FIFO_DATA_OUT_Y_L 0x7b       /* FIFO out data y low register (r) */
#define LSM6DSO_FIFO_DATA_OUT_Y_H 0x7c       /* FIFO out data y high register (r) */
#define LSM6DSO_FIFO_DATA_OUT_Z_L 0x7d       /* FIFO out data z low register (r) */
#define LSM6DSO_FIFO_DATA_OUT_Z_H 0x7e       /* FIFO out data z high register (r) */

#define LSM6DSO_OUT_TEMP_L        0x20       /* Temperature data output register (r) */
#define LSM6DSO_OUT_TEMP_H        0x21       /* Temperature data output register (r) */
#define LSM6DSO_OUTX_L_G          0x22       /* Angular rate sensor pitch axis (X) angular rate output register (r) */
#define LSM6DSO_OUTX_H_G          0x23       /* Angular rate sensor pitch axis (X) angular rate output register (r) */
#define LSM6DSO_OUTY_L_G          0x24       /* Angular rate sensor roll axis (Y) angular rate output register (r) */
#define LSM6DSO_OUTY_H_G          0x25       /* Angular rate sensor roll axis (Y) angular rate output register (r) */
#define LSM6DSO_OUTZ_L_G          0x26       /* Angular rate sensor yaw axis (Z) angular rate output register (r) */
#define LSM6DSO_OUTZ_H_G          0x27       /* Angular rate sensor yaw axis (Z) angular rate output register (r) */
#define LSM6DSO_OUTX_L_XL         0x28       /* Linear acceleration sensor X-axis output register (r) */
#define LSM6DSO_OUTX_H_XL         0x29       /* Linear acceleration sensor X-axis output register (r) */
#define LSM6DSO_OUTY_L_XL         0x2a       /* Linear acceleration sensor Y-axis output register (r) */
#define LSM6DSO_OUTY_H_XL         0x2b       /* Linear acceleration sensor Y-axis output register (r) */
#define LSM6DSO_OUTZ_L_XL         0x2c       /* Linear acceleration sensor Z-axis output register (r) */
#define LSM6DSO_OUTZ_H_XL         0x2d       /* Linear acceleration sensor Z-axis output register (r) */
#define LSM6DSO_FIFO_STATUS1      0x3a       /* FIFO status control register (r) */
#define LSM6DSO_FIFO_STATUS2      0x3b       /* FIFO status control register (r) */
#define LSM6DSO_TIMESTAMP0_REG    0x40       /* Timestamp first (least significant) byte data output register (r) */
#define LSM6DSO_TIMESTAMP1_REG    0x41       /* Timestamp second byte data output register (r) */
#define LSM6DSO_TIMESTAMP2_REG    0x42       /* Timestamp third (most significant) byte data output register (r) */

#define LSM6DSO_FUNC_CFG_ACCESS   0x01       /* Enable embedded functions register (r/w) */
#define LSM6DSO_PAGE_SEL          0x02       /* Enable advanced features dedicated page (r/w) */
#define LSM6DSO_EMB_FUNC_EN_A     0x04       /* Embedded functions enable register (r/w) */
#define LSM6DSO_EMB_FUNC_EN_B     0x05       /* Embedded functions enable register (r/w) */
#define LSM6DSO_PAGE_ADDRESS      0x08       /* Page address register (r/w) */
#define LSM6DSO_PAGE_VALUE        0x09       /* Page value register (r/w) */
#define LSM6DSO_EMB_FUNC_INT1     0x0a       /* INT1 pin control register (r/w) */
#define LSM6DSO_FSM_INT1_A        0x0b       /* INT1 pin control register (r/w) */
#define LSM6DSO_FSM_INT1_B        0x0c       /* INT1 pin control register (r/w) */
#define LSM6DSO_EMB_FUNC_INT2     0x0e       /* INT2 pin control register (r/w) */
#define LSM6DSO_FSM_INT2_A        0x0f       /* INT2 pin control register (r/w) */
#define LSM6DSO_FSM_INT2_B        0x10       /* INT2 pin control register (r/w) */
#define LSM6DSO_PAGE_RW           0x17       /* Enable read and write mode of advanced features dedicated page (r/w) */
#define LSM6DSO_ALL_INT_SRC       0x1a       /* Source register for all interrupts (r) */
#define LSM6DSO_WAKE_UP_SRC       0x1b       /* Wake-up interrupt source register (r) */
#define LSM6DSO_TAP_SRC           0x1c       /* Tap source register (r) */
#define LSM6DSO_D6D_SRC           0x1d       /* Portrait, landscape, face-up and face-down source register (r) */
#define LSM6DSO_EMB_FUNC_MPSTATUS 0x35       /* Embedded function status register (r) */
#define LSM6DSO_FSM_MPSTATUS_A    0x36       /* Finite State Machine status register (r) */
#define LSM6DSO_FSM_MPSTATUS_B    0x37       /* Finite State Machine status register (r) */
#define LSM6DSO_STATUS_MPMASTER   0x39       /* Sensor hub source register (r)*/
#define LSM6DSO_TAP_CFG0          0x56       /* Configuration of filtering, and tap recognition functions (r/w) */
#define LSM6DSO_TAP_CFG2          0x58       /* Enables interrupt and inactivity functions, and tap recognition functions (r/w) */
#define LSM6DSO_MD1_CFG           0x5e       /* Functions routing on INT1 register (r/w) */
#define LSM6DSO_MD2_CFG           0x5f       /* Functions routing on INT2 register (r/w) */
#define LSM6DSO_EMB_FUNC_ODRCFG_B 0x5f       /* Finite State Machine output data rate configuration register (r/w) */
#define LSM6DSO_INT_OIS           0x6f       /* OIS interrupt configuration register and accelerometer self-test enable setting */
#define LSM6DSO_FSM_LC_TIMEOUT_L  0x17a      /* FSM long counter timeout register (r/w) */
#define LSM6DSO_FSM_LC_TIMEOUT_H  0x17b      /* FSM long counter timeout register (r/w) */
#define LSM6DSO_FSM_PROGRAMS      0x17c      /* FSM number of programs register (r/w) */
#define LSM6DSO_FSM_START_ADD_L   0x17e      /* FSM start address register (r/w) */
#define LSM6DSO_FSM_START_ADD_H   0x17f      /* FSM start address register (r/w) */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Device Register Bit */

struct lsm6dso_fifo_ctrl1_s
{
  uint8_t wtm                      : 8;
};

typedef struct lsm6dso_fifo_ctrl1_s lsm6dso_fifo_ctrl1_t;

struct lsm6dso_fifo_ctrl2_s
{
  uint8_t wtm                      : 1;
  uint8_t uncoptr_rate             : 2;
  uint8_t not_used_01              : 1;
  uint8_t odrchg_en                : 1;
  uint8_t not_used_02              : 1;
  uint8_t fifo_compr_rt_en         : 1;
  uint8_t stop_on_wtm              : 1;
};

typedef struct lsm6dso_fifo_ctrl2_s lsm6dso_fifo_ctrl2_t;

struct lsm6dso_fifo_ctrl3_s
{
  uint8_t bdr_xl                   : 4;
  uint8_t bdr_gy                   : 4;
};

typedef struct lsm6dso_fifo_ctrl3_s lsm6dso_fifo_ctrl3_t;

struct lsm6dso_fifo_ctrl4_s
{
  uint8_t fifo_mode                : 3;
  uint8_t not_used_01              : 1;
  uint8_t odr_t_batch              : 2;
  uint8_t odr_ts_batch             : 2;
};

typedef struct lsm6dso_fifo_ctrl4_s lsm6dso_fifo_ctrl4_t;

struct lsm6dso_counter_bdr_reg1_s
{
  uint8_t cnt_bdr_th               : 3;
  uint8_t not_used_01              : 2;
  uint8_t trig_counter_bdr         : 1;
  uint8_t rst_counter_bdr          : 1;
  uint8_t dataready_pulsed         : 1;
};

typedef struct lsm6dso_counter_bdr_reg1_s lsm6dso_counter_bdr_reg1_t;

struct lsm6dso_int1_ctrl_s
{
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fifo_th             : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_fifo_full           : 1;
  uint8_t int1_cnt_bdr             : 1;
  uint8_t den_drdy_flag            : 1;
};

typedef struct lsm6dso_int1_ctrl_s lsm6dso_int1_ctrl_t;

struct lsm6dso_int2_ctrl_s
{
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fifo_th             : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_fifo_full           : 1;
  uint8_t int2_cnt_bdr             : 1;
  uint8_t not_used_01              : 1;
};

typedef struct lsm6dso_int2_ctrl_s lsm6dso_int2_ctrl_t;

struct lsm6dso_ctrl1_xl_s
{
  uint8_t not_used_01              : 1;
  uint8_t lpf2_xl_en               : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 4;
};

typedef struct lsm6dso_ctrl1_xl_s lsm6dso_ctrl1_xl_t;

struct lsm6dso_ctrl2_g_s
{
  uint8_t not_used_01              : 1;
  uint8_t fs_g                     : 3;
  uint8_t odr_g                    : 4;
};

typedef struct lsm6dso_ctrl2_g_s lsm6dso_ctrl2_g_t;

struct lsm6dso_ctrl3_c_s
{
  uint8_t sw_reset                 : 1;
  uint8_t not_used_01              : 1;
  uint8_t if_inc                   : 1;
  uint8_t sim                      : 1;
  uint8_t pp_od                    : 1;
  uint8_t h_lactive                : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
};

typedef struct lsm6dso_ctrl3_c_s lsm6dso_ctrl3_c_t;

struct lsm6dso_ctrl4_c_s
{
  uint8_t not_used_01              : 1;
  uint8_t lpf1_sel_g               : 1;
  uint8_t i2c_disable              : 1;
  uint8_t drdy_mask                : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_on_int1             : 1;
  uint8_t sleep_g                  : 1;
  uint8_t not_used_03              : 1;
};

typedef struct lsm6dso_ctrl4_c_s lsm6dso_ctrl4_c_t;

struct lsm6dso_ctrl5_c_s
{
  uint8_t st_xl                    : 2;
  uint8_t st_g                     : 2;
  uint8_t not_used_01              : 1;
  uint8_t rounding                 : 2;
  uint8_t xl_ulp_en                : 1;
};

typedef struct lsm6dso_ctrl5_c_s lsm6dso_ctrl5_c_t;

struct lsm6dso_ctrl6_c_s
{
  uint8_t ftype                    : 3;
  uint8_t usr_off_w                : 1;
  uint8_t xl_hm_mode               : 1;
  uint8_t den_mode                 : 3;
};

typedef struct lsm6dso_ctrl6_c_s lsm6dso_ctrl6_c_t;

struct lsm6dso_ctrl7_g_s
{
  uint8_t ois_on                   : 1;
  uint8_t usr_off_on_out           : 1;
  uint8_t ois_on_en                : 1;
  uint8_t not_used_01              : 1;
  uint8_t hpm_g                    : 2;
  uint8_t hp_en_g                  : 1;
  uint8_t g_hm_mode                : 1;
};

typedef struct lsm6dso_ctrl7_g_s lsm6dso_ctrl7_g_t;

struct lsm6dso_ctrl10_c_s
{
  uint8_t not_used_01              : 5;
  uint8_t timestamp_en             : 1;
  uint8_t not_used_02              : 2;
};

typedef struct lsm6dso_ctrl10_c_s lsm6dso_ctrl10_c_t;

struct lsm6dso_i3c_bus_avb_s
{
  uint8_t pd_dis_int1              : 1;
  uint8_t not_used_01              : 2;
  uint8_t i3c_bus_avb_sel          : 2;
  uint8_t not_used_02              : 3;
};

typedef struct lsm6dso_i3c_bus_avb_s lsm6dso_i3c_bus_avb_t;

struct lsm6dso_ctrl9_xl_s
{
  uint8_t not_used_01              : 1;
  uint8_t i3c_disable              : 1;
  uint8_t den_lh                   : 1;
  uint8_t den_xl_g                 : 2;
  uint8_t den_z                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_x                    : 1;
};

typedef struct lsm6dso_ctrl9_xl_s lsm6dso_ctrl9_xl_t;

struct lsm6dso_ctrl8_xl_s
{
  uint8_t low_pass_on_6d           : 1;
  uint8_t xl_fs_mode               : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t fastsettl_mode_xl        : 1;
  uint8_t hp_ref_mode_xl           : 1;
  uint8_t hpcf_xl                  : 3;
};

typedef struct lsm6dso_ctrl8_xl_s lsm6dso_ctrl8_xl_t;

struct lsm6dso_status_reg_s
{
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t tda                      : 1;
  uint8_t not_used_01              : 5;
};

typedef struct lsm6dso_status_reg_s lsm6dso_status_reg_t;

struct lsm6dso_fifo_status1_s
{
  uint8_t diff_fifo                : 8;
};

typedef struct lsm6dso_fifo_status1_s lsm6dso_fifo_status1_t;

struct lsm6dso_fifo_status2_s
{
  uint8_t diff_fifo                : 2;
  uint8_t not_used_01              : 1;
  uint8_t over_run_latched         : 1;
  uint8_t counter_bdr_ia           : 1;
  uint8_t fifo_full_ia             : 1;
  uint8_t fifo_ovr_ia              : 1;
  uint8_t fifo_wtm_ia              : 1;
};

typedef struct lsm6dso_fifo_status2_s lsm6dso_fifo_status2_t;

struct lsm6dso_fsm_enable_a_s
{
  uint8_t fsm1_en                  : 1;
  uint8_t fsm2_en                  : 1;
  uint8_t fsm3_en                  : 1;
  uint8_t fsm4_en                  : 1;
  uint8_t fsm5_en                  : 1;
  uint8_t fsm6_en                  : 1;
  uint8_t fsm7_en                  : 1;
  uint8_t fsm8_en                  : 1;
};

typedef struct lsm6dso_fsm_enable_a_s lsm6dso_fsm_enable_a_t;

struct lsm6dso_fsm_enable_b_s
{
  uint8_t fsm9_en                  : 1;
  uint8_t fsm10_en                 : 1;
  uint8_t fsm11_en                 : 1;
  uint8_t fsm12_en                 : 1;
  uint8_t fsm13_en                 : 1;
  uint8_t fsm14_en                 : 1;
  uint8_t fsm15_en                 : 1;
  uint8_t fsm16_en                 : 1;
};

typedef struct lsm6dso_fsm_enable_b_s lsm6dso_fsm_enable_b_t;

struct lsm6dso_fifo_data_out_tag_s
{
  uint8_t tag_parity               : 1;
  uint8_t tag_cnt                  : 2;
  uint8_t tag_sensor               : 5;
};

typedef struct lsm6dso_fifo_data_out_tag_s lsm6dso_fifo_data_out_tag_t;

struct lsm6dso_func_cfg_access_s
{
  uint8_t not_used_01              : 6;
  uint8_t reg_access               : 2;
};

typedef struct lsm6dso_func_cfg_access_s lsm6dso_func_cfg_access_t;

struct lsm6dso_page_sel_s
{
  uint8_t not_used_01              : 4;
  uint8_t page_sel                 : 4;
};

typedef struct lsm6dso_page_sel_s lsm6dso_page_sel_t;

struct lsm6dso_emb_func_en_a_s
{
  uint8_t not_used_01              : 3;
  uint8_t pedo_en                  : 1;
  uint8_t tilt_en                  : 1;
  uint8_t sign_motion_en           : 1;
  uint8_t not_used_02              : 2;
};

typedef struct lsm6dso_emb_func_en_a_s lsm6dso_emb_func_en_a_t;

struct lsm6dso_emb_func_en_b_s
{
  uint8_t fsm_en                   : 1;
  uint8_t not_used_01              : 2;
  uint8_t fifo_compr_en            : 1;
  uint8_t pedo_adv_en              : 1;
  uint8_t not_used_02              : 3;
};

typedef struct lsm6dso_emb_func_en_b_s lsm6dso_emb_func_en_b_t;

struct lsm6dso_page_address_s
{
  uint8_t page_addr                : 8;
};

typedef struct lsm6dso_page_address_s lsm6dso_page_address_t;

struct lsm6dso_emb_func_int1_s
{
  uint8_t not_used_01              : 3;
  uint8_t int1_step_detector       : 1;
  uint8_t int1_tilt                : 1;
  uint8_t int1_sig_mot             : 1;
  uint8_t not_used_02              : 1;
  uint8_t int1_fsm_lc              : 1;
};

typedef struct lsm6dso_emb_func_int1_s lsm6dso_emb_func_int1_t;

struct lsm6dso_fsm_int1_a_s
{
  uint8_t int1_fsm1                : 1;
  uint8_t int1_fsm2                : 1;
  uint8_t int1_fsm3                : 1;
  uint8_t int1_fsm4                : 1;
  uint8_t int1_fsm5                : 1;
  uint8_t int1_fsm6                : 1;
  uint8_t int1_fsm7                : 1;
  uint8_t int1_fsm8                : 1;
};

typedef struct lsm6dso_fsm_int1_a_s lsm6dso_fsm_int1_a_t;

struct lsm6dso_fsm_int1_b_s
{
  uint8_t int1_fsm9                : 1;
  uint8_t int1_fsm10               : 1;
  uint8_t int1_fsm11               : 1;
  uint8_t int1_fsm12               : 1;
  uint8_t int1_fsm13               : 1;
  uint8_t int1_fsm14               : 1;
  uint8_t int1_fsm15               : 1;
  uint8_t int1_fsm16               : 1;
};

typedef struct lsm6dso_fsm_int1_b_s lsm6dso_fsm_int1_b_t;

struct lsm6dso_emb_func_int2_s
{
  uint8_t not_used_01              : 3;
  uint8_t int2_step_detector       : 1;
  uint8_t int2_tilt                : 1;
  uint8_t int2_sig_mot             : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_fsm_lc              : 1;
};

typedef struct lsm6dso_emb_func_int2_s lsm6dso_emb_func_int2_t;

struct lsm6dso_fsm_int2_a_s
{
  uint8_t int2_fsm1                : 1;
  uint8_t int2_fsm2                : 1;
  uint8_t int2_fsm3                : 1;
  uint8_t int2_fsm4                : 1;
  uint8_t int2_fsm5                : 1;
  uint8_t int2_fsm6                : 1;
  uint8_t int2_fsm7                : 1;
  uint8_t int2_fsm8                : 1;
};

typedef struct lsm6dso_fsm_int2_a_s lsm6dso_fsm_int2_a_t;

struct lsm6dso_fsm_int2_b_s
{
  uint8_t int2_fsm9                : 1;
  uint8_t int2_fsm10               : 1;
  uint8_t int2_fsm11               : 1;
  uint8_t int2_fsm12               : 1;
  uint8_t int2_fsm13               : 1;
  uint8_t int2_fsm14               : 1;
  uint8_t int2_fsm15               : 1;
  uint8_t int2_fsm16               : 1;
};

typedef struct lsm6dso_fsm_int2_b_s lsm6dso_fsm_int2_b_t;

struct lsm6dso_page_rw_s
{
  uint8_t not_used_01              : 5;
  uint8_t page_rw                  : 2;  /* page_write + page_read */
  uint8_t emb_func_lir             : 1;
};

typedef struct lsm6dso_page_rw_s lsm6dso_page_rw_t;

struct lsm6dso_all_int_src_s
{
  uint8_t ff_ia                    : 1;
  uint8_t wu_ia                    : 1;
  uint8_t single_tap               : 1;
  uint8_t double_tap               : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_01              : 1;
  uint8_t timestamp_endcount       : 1;
};

typedef struct lsm6dso_all_int_src_s lsm6dso_all_int_src_t;

struct lsm6dso_wake_up_src_s
{
  uint8_t z_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t x_wu                     : 1;
  uint8_t wu_ia                    : 1;
  uint8_t sleep_state              : 1;
  uint8_t ff_ia                    : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_01              : 1;
};

typedef struct lsm6dso_wake_up_src_s lsm6dso_wake_up_src_t;

struct lsm6dso_tap_src_s
{
  uint8_t z_tap                    : 1;
  uint8_t y_tap                    : 1;
  uint8_t x_tap                    : 1;
  uint8_t tap_sign                 : 1;
  uint8_t double_tap               : 1;
  uint8_t single_tap               : 1;
  uint8_t tap_ia                   : 1;
  uint8_t not_used_02              : 1;
};

typedef struct lsm6dso_tap_src_s lsm6dso_tap_src_t;

struct lsm6dso_d6d_src_s
{
  uint8_t xl                       : 1;
  uint8_t xh                       : 1;
  uint8_t yl                       : 1;
  uint8_t yh                       : 1;
  uint8_t zl                       : 1;
  uint8_t zh                       : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t den_drdy                 : 1;
};

typedef struct lsm6dso_d6d_src_s lsm6dso_d6d_src_t;

struct lsm6dso_emb_func_mpstatus_s
{
  uint8_t not_used_01             : 3;
  uint8_t is_step_det             : 1;
  uint8_t is_tilt                 : 1;
  uint8_t is_sigmot               : 1;
  uint8_t not_used_02             : 1;
  uint8_t is_fsm_lc               : 1;
};

typedef struct lsm6dso_emb_func_mpstatus_s lsm6dso_emb_func_mpstatus_t;

struct lsm6dso_status_mpmaster_s
{
  uint8_t sens_hub_endop          : 1;
  uint8_t not_used_01             : 2;
  uint8_t slave0_nack             : 1;
  uint8_t slave1_nack             : 1;
  uint8_t slave2_nack             : 1;
  uint8_t slave3_nack             : 1;
  uint8_t wr_once_done            : 1;
};

typedef struct lsm6dso_status_mpmaster_s lsm6dso_status_mpmaster_t;

struct lsm6dso_fsm_mpstatus_a_s
{
  uint8_t is_fsm1                 : 1;
  uint8_t is_fsm2                 : 1;
  uint8_t is_fsm3                 : 1;
  uint8_t is_fsm4                 : 1;
  uint8_t is_fsm5                 : 1;
  uint8_t is_fsm6                 : 1;
  uint8_t is_fsm7                 : 1;
  uint8_t is_fsm8                 : 1;
};

typedef struct lsm6dso_fsm_mpstatus_a_s lsm6dso_fsm_mpstatus_a_t;

struct lsm6dso_fsm_mpstatus_b_s
{
  uint8_t is_fsm9                 : 1;
  uint8_t is_fsm10                : 1;
  uint8_t is_fsm11                : 1;
  uint8_t is_fsm12                : 1;
  uint8_t is_fsm13                : 1;
  uint8_t is_fsm14                : 1;
  uint8_t is_fsm15                : 1;
  uint8_t is_fsm16                : 1;
};

typedef struct lsm6dso_fsm_mpstatus_b_s lsm6dso_fsm_mpstatus_b_t;

struct lsm6dso_tap_cfg0_s
{
  uint8_t lir                      : 1;
  uint8_t tap_z_en                 : 1;
  uint8_t tap_y_en                 : 1;
  uint8_t tap_x_en                 : 1;
  uint8_t slope_fds                : 1;
  uint8_t sleep_status_on_int      : 1;
  uint8_t int_clr_on_read          : 1;
  uint8_t not_used_01              : 1;
};

typedef struct lsm6dso_tap_cfg0_s lsm6dso_tap_cfg0_t;

struct lsm6dso_tap_cfg2_s
{
  uint8_t tap_ths_y                : 5;
  uint8_t inact_en                 : 2;
  uint8_t interrupts_enable        : 1;
};

typedef struct lsm6dso_tap_cfg2_s lsm6dso_tap_cfg2_t;

struct lsm6dso_md1_cfg_s
{
  uint8_t int1_shub                : 1;
  uint8_t int1_emb_func            : 1;
  uint8_t int1_6d                  : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_sleep_change        : 1;
};

typedef struct lsm6dso_md1_cfg_s lsm6dso_md1_cfg_t;

struct lsm6dso_md2_cfg_s
{
  uint8_t int2_timestamp           : 1;
  uint8_t int2_emb_func            : 1;
  uint8_t int2_6d                  : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_sleep_change        : 1;
};

typedef struct lsm6dso_md2_cfg_s lsm6dso_md2_cfg_t;

struct lsm6dso_emb_func_odr_cfg_b_s
{
  uint8_t not_used_01              : 3;
  uint8_t fsm_odr                  : 2;
  uint8_t not_used_02              : 3;
};

typedef struct lsm6dso_emb_func_odr_cfg_b_s lsm6dso_emb_func_odr_cfg_b_t;

struct lsm6dso_int_ois_s
{
  uint8_t st_xl_ois                : 2;
  uint8_t not_used_01              : 3;
  uint8_t den_lh_ois               : 1;
  uint8_t lvl2_ois                 : 1;
  uint8_t int2_drdy_ois            : 1;
};

typedef struct lsm6dso_int_ois_s lsm6dso_int_ois_t;

struct lsm6dso_pin_int1_route_s
{
  uint8_t drdy_xl       : 1;        /* Accelerometer data ready */
  uint8_t drdy_g        : 1;        /* Gyroscope data ready */
  uint8_t drdy_temp     : 1;        /* Temperature data ready (1 = int2 pin disable) */
  uint8_t boot          : 1;        /* Restoring calibration parameters */
  uint8_t fifo_th       : 1;        /* FIFO threshold reached */
  uint8_t fifo_ovr      : 1;        /* FIFO overrun */
  uint8_t fifo_full     : 1;        /* FIFO full */
  uint8_t fifo_bdr      : 1;        /* FIFO Batch counter threshold reached */
  uint8_t den_flag      : 1;        /* external trigger level recognition (DEN) */
  uint8_t sh_endop      : 1;        /* sensor hub end operation */
  uint8_t timestamp     : 1;        /* timestamp overflow (1 = int2 pin disable) */
  uint8_t six_d         : 1;        /* orientation change (6D/4D detection) */
  uint8_t double_tap    : 1;        /* double-tap event */
  uint8_t free_fall     : 1;        /* free fall event */
  uint8_t wake_up       : 1;        /* wake up event */
  uint8_t single_tap    : 1;        /* single-tap event */
  uint8_t sleep_change  : 1;        /* Act/Inact (or Vice-versa) status changed */
  uint8_t step_detector : 1;        /* Step detected */
  uint8_t tilt          : 1;        /* Relative tilt event detected */
  uint8_t sig_mot       : 1;        /* "significant motion" event detected */
  uint8_t fsm_lc        : 1;        /* fsm long counter timeout interrupt event */
  uint8_t fsm1          : 1;        /* fsm 1 interrupt event */
  uint8_t fsm2          : 1;        /* fsm 2 interrupt event */
  uint8_t fsm3          : 1;        /* fsm 3 interrupt event */
  uint8_t fsm4          : 1;        /* fsm 4 interrupt event */
  uint8_t fsm5          : 1;        /* fsm 5 interrupt event */
  uint8_t fsm6          : 1;        /* fsm 6 interrupt event */
  uint8_t fsm7          : 1;        /* fsm 7 interrupt event */
  uint8_t fsm8          : 1;        /* fsm 8 interrupt event */
  uint8_t fsm9          : 1;        /* fsm 9 interrupt event */
  uint8_t fsm10         : 1;        /* fsm 10 interrupt event */
  uint8_t fsm11         : 1;        /* fsm 11 interrupt event */
  uint8_t fsm12         : 1;        /* fsm 12 interrupt event */
  uint8_t fsm13         : 1;        /* fsm 13 interrupt event */
  uint8_t fsm14         : 1;        /* fsm 14 interrupt event */
  uint8_t fsm15         : 1;        /* fsm 15 interrupt event */
  uint8_t fsm16         : 1;        /* fsm 16 interrupt event */
  uint8_t mlc1          : 1;        /* mlc 1 interrupt event */
  uint8_t mlc2          : 1;        /* mlc 2 interrupt event */
  uint8_t mlc3          : 1;        /* mlc 3 interrupt event */
  uint8_t mlc4          : 1;        /* mlc 4 interrupt event */
  uint8_t mlc5          : 1;        /* mlc 5 interrupt event */
  uint8_t mlc6          : 1;        /* mlc 6 interrupt event */
  uint8_t mlc7          : 1;        /* mlc 7 interrupt event */
  uint8_t mlc8          : 1;        /* mlc 8 interrupt event */
};

typedef struct lsm6dso_pin_int1_route_s lsm6dso_pin_int1_route_t;

struct lsm6dso_pin_int2_route_s
{
  uint8_t drdy_ois      : 1;         /* OIS chain data ready */
  uint8_t drdy_xl       : 1;         /* Accelerometer data ready */
  uint8_t drdy_g        : 1;         /* Gyroscope data ready */
  uint8_t drdy_temp     : 1;         /* Temperature data ready */
  uint8_t fifo_th       : 1;         /* FIFO threshold reached */
  uint8_t fifo_ovr      : 1;         /* FIFO overrun */
  uint8_t fifo_full     : 1;         /* FIFO full */
  uint8_t fifo_bdr      : 1;         /* FIFO Batch counter threshold reached */
  uint8_t timestamp     : 1;         /* timestamp overflow */
  uint8_t six_d         : 1;         /* orientation change (6D/4D detection) */
  uint8_t double_tap    : 1;         /* double-tap event */
  uint8_t free_fall     : 1;         /* free fall event */
  uint8_t wake_up       : 1;         /* wake up event */
  uint8_t single_tap    : 1;         /* single-tap event */
  uint8_t sleep_change  : 1;         /* Act/Inact (or Vice-versa) status changed */
  uint8_t step_detector : 1;         /* Step detected */
  uint8_t tilt          : 1;         /* Relative tilt event detected */
  uint8_t sig_mot       : 1;         /* "significant motion" event detected */
  uint8_t fsm_lc        : 1;         /* fsm long counter timeout interrupt event */
  uint8_t fsm1          : 1;         /* fsm 1 interrupt event */
  uint8_t fsm2          : 1;         /* fsm 2 interrupt event */
  uint8_t fsm3          : 1;         /* fsm 3 interrupt event */
  uint8_t fsm4          : 1;         /* fsm 4 interrupt event */
  uint8_t fsm5          : 1;         /* fsm 5 interrupt event */
  uint8_t fsm6          : 1;         /* fsm 6 interrupt event */
  uint8_t fsm7          : 1;         /* fsm 7 interrupt event */
  uint8_t fsm8          : 1;         /* fsm 8 interrupt event */
  uint8_t fsm9          : 1;         /* fsm 9 interrupt event */
  uint8_t fsm10         : 1;         /* fsm 10 interrupt event */
  uint8_t fsm11         : 1;         /* fsm 11 interrupt event */
  uint8_t fsm12         : 1;         /* fsm 12 interrupt event */
  uint8_t fsm13         : 1;         /* fsm 13 interrupt event */
  uint8_t fsm14         : 1;         /* fsm 14 interrupt event */
  uint8_t fsm15         : 1;         /* fsm 15 interrupt event */
  uint8_t fsm16         : 1;         /* fsm 16 interrupt event */
  uint8_t mlc1          : 1;         /* mlc 1 interrupt event */
  uint8_t mlc2          : 1;         /* mlc 2 interrupt event */
  uint8_t mlc3          : 1;         /* mlc 3 interrupt event */
  uint8_t mlc4          : 1;         /* mlc 4 interrupt event */
  uint8_t mlc5          : 1;         /* mlc 5 interrupt event */
  uint8_t mlc6          : 1;         /* mlc 6 interrupt event */
  uint8_t mlc7          : 1;         /* mlc 7 interrupt event */
  uint8_t mlc8          : 1;         /* mlc 8 interrupt event */
};

typedef struct lsm6dso_pin_int2_route_s lsm6dso_pin_int2_route_t;

struct lsm6dso_all_sources_s
{
  uint8_t drdy_xl          : 1;      /* Accelerometer data ready */
  uint8_t drdy_g           : 1;      /* Gyroscope data ready */
  uint8_t drdy_temp        : 1;      /* Temperature data ready */
  uint8_t den_flag         : 1;      /* external trigger level recognition (DEN) */
  uint8_t timestamp        : 1;      /* timestamp overflow (1 = int2 pin disable) */
  uint8_t free_fall        : 1;      /* free fall event */
  uint8_t wake_up          : 1;      /* wake up event */
  uint8_t wake_up_z        : 1;      /* wake up on Z axis event */
  uint8_t wake_up_y        : 1;      /* wake up on Y axis event */
  uint8_t wake_up_x        : 1;      /* wake up on X axis event */
  uint8_t single_tap       : 1;      /* single-tap event */
  uint8_t double_tap       : 1;      /* double-tap event */
  uint8_t tap_z            : 1;      /* single-tap on Z axis event */
  uint8_t tap_y            : 1;      /* single-tap on Y axis event */
  uint8_t tap_x            : 1;      /* single-tap on X axis event */
  uint8_t tap_sign         : 1;      /* sign of tap event (0-pos / 1-neg) */
  uint8_t six_d            : 1;      /* orientation change (6D/4D detection) */
  uint8_t six_d_xl         : 1;      /* X-axis low 6D/4D event (under threshold) */
  uint8_t six_d_xh         : 1;      /* X-axis high 6D/4D event (over threshold) */
  uint8_t six_d_yl         : 1;      /* Y-axis low 6D/4D event (under threshold) */
  uint8_t six_d_yh         : 1;      /* Y-axis high 6D/4D event (over threshold) */
  uint8_t six_d_zl         : 1;      /* Z-axis low 6D/4D event (under threshold) */
  uint8_t six_d_zh         : 1;      /* Z-axis high 6D/4D event (over threshold) */
  uint8_t sleep_change     : 1;      /* Act/Inact (or Vice-versa) status changed */
  uint8_t sleep_state      : 1;      /* Act/Inact status flag (0-Act / 1-Inact) */
  uint8_t step_detector    : 1;      /* Step detected */
  uint8_t tilt             : 1;      /* Relative tilt event detected */
  uint8_t sig_mot          : 1;      /* "significant motion" event detected */
  uint8_t fsm_lc           : 1;      /* fsm long counter timeout interrupt event */
  uint8_t fsm1             : 1;      /* fsm 1 interrupt event */
  uint8_t fsm2             : 1;      /* fsm 2 interrupt event */
  uint8_t fsm3             : 1;      /* fsm 3 interrupt event */
  uint8_t fsm4             : 1;      /* fsm 4 interrupt event */
  uint8_t fsm5             : 1;      /* fsm 5 interrupt event */
  uint8_t fsm6             : 1;      /* fsm 6 interrupt event */
  uint8_t fsm7             : 1;      /* fsm 7 interrupt event */
  uint8_t fsm8             : 1;      /* fsm 8 interrupt event */
  uint8_t fsm9             : 1;      /* fsm 9 interrupt event */
  uint8_t fsm10            : 1;      /* fsm 10 interrupt event */
  uint8_t fsm11            : 1;      /* fsm 11 interrupt event */
  uint8_t fsm12            : 1;      /* fsm 12 interrupt event */
  uint8_t fsm13            : 1;      /* fsm 13 interrupt event */
  uint8_t fsm14            : 1;      /* fsm 14 interrupt event */
  uint8_t fsm15            : 1;      /* fsm 15 interrupt event */
  uint8_t fsm16            : 1;      /* fsm 16 interrupt event */
  uint8_t mlc1             : 1;      /* mlc 1 interrupt event */
  uint8_t mlc2             : 1;      /* mlc 2 interrupt event */
  uint8_t mlc3             : 1;      /* mlc 3 interrupt event */
  uint8_t mlc4             : 1;      /* mlc 4 interrupt event */
  uint8_t mlc5             : 1;      /* mlc 5 interrupt event */
  uint8_t mlc6             : 1;      /* mlc 6 interrupt event */
  uint8_t mlc7             : 1;      /* mlc 7 interrupt event */
  uint8_t mlc8             : 1;      /* mlc 8 interrupt event */
  uint8_t sh_endop         : 1;      /* sensor hub end operation */
  uint8_t sh_slave0_nack   : 1;      /* Not acknowledge on sensor hub slave 0 */
  uint8_t sh_slave1_nack   : 1;      /* Not acknowledge on sensor hub slave 1 */
  uint8_t sh_slave2_nack   : 1;      /* Not acknowledge on sensor hub slave 2 */
  uint8_t sh_slave3_nack   : 1;      /* Not acknowledge on sensor hub slave 3 */
  uint8_t sh_wr_once       : 1;      /* "WRITE_ONCE" end on sensor hub slave 0 */
  uint16_t fifo_diff       : 10;     /* Number of unread sensor data in FIFO */
  uint8_t fifo_ovr_latched : 1;      /* Latched FIFO overrun status */
  uint8_t fifo_bdr         : 1;      /* FIFO Batch counter threshold reached */
  uint8_t fifo_full        : 1;      /* FIFO full */
  uint8_t fifo_ovr         : 1;      /* FIFO overrun */
  uint8_t fifo_th          : 1;      /* FIFO threshold reached */
};

typedef struct lsm6dso_all_sources_s lsm6dso_all_sources_t;

struct lsm6dso_emb_sens_s
{
  uint8_t sig_mot       : 1;         /* significant motion */
  uint8_t tilt          : 1;         /* tilt detection  */
  uint8_t step          : 1;         /* step counter/detector */
  uint8_t step_adv      : 1;         /* step counter advanced mode */
  uint8_t fsm           : 1;         /* finite state machine */
  uint8_t fifo_compr    : 1;         /* FIFO compression */
};

typedef struct lsm6dso_emb_sens_s lsm6dso_emb_sens_t;

enum lsm6dso_lir_e
{
  LSM6DSO_ALL_INT_PULSED          = 0,
  LSM6DSO_BASE_LATCHED_EMB_PULSED = 1,
  LSM6DSO_BASE_PULSED_EMB_LATCHED = 2,
  LSM6DSO_ALL_INT_LATCHED         = 3,
};

typedef enum lsm6dso_lir_e lsm6dso_lir_t;

struct lsm6dso_emb_fsm_enable_s
{
  lsm6dso_fsm_enable_a_t fsm_enable_a;
  lsm6dso_fsm_enable_b_t fsm_enable_b;
};

typedef struct lsm6dso_emb_fsm_enable_s lsm6dso_emb_fsm_enable_t;

/* Convert data */

union axis3bit16_u
{
  int16_t i16bit[3];                            /* 16 bit int data */
  uint8_t u8bit[6];                             /* 8 bit unsigned int data */
};

typedef union axis3bit16_u axis3bit16_t;

/* Sensor struct */

struct lsm6dso_sensor_s
{
  struct sensor_lowerhalf_s lower;              /* Lower half sensor driver */
  struct work_s             work;               /* Sensor handler */
  unsigned long             interval;           /* Sensor interval */
  unsigned long             batch;              /* Sensor bat */
  unsigned int              fifowtm;            /* Sensor fifo water marker */
  float                     factor;             /* Data factor */
  bool                      fifoen;             /* Sensor fifo enable */
  bool                      activated;          /* Sensor working state */
};

/* Device struct */

struct lsm6dso_dev_s
{
  struct lsm6dso_sensor_s dev[LSM6DSO_IDX_NUM]; /* Sensor struct */
  uint64_t timestamp;                           /* Units is microseconds */
  uint64_t timestamp_fifolast;                  /* FIFO last timestamp, Units is microseconds */
  FAR const struct lsm6dso_config_s *config;    /* The board config */
  struct work_s work;                           /* Interrupt handler */
  unsigned int fifowtm;                         /* fifo water marker */
  unsigned int fsmen;                           /* FSM enable */
  bool fifoen;                                  /* Sensor fifo enable */
};

/* Sensor ODR */

struct lsm6dso_odr_s
{
  uint8_t regval;                               /* the data of register */
  float odr;                                    /* the unit is Hz */
};

/* Batch BDR */

struct lsm6dso_bdr_s
{
  uint8_t regval;                               /* the data of register */
  float   bdr;                                  /* the unit is Hz */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI functions */

static void lsm6dso_spi_read(FAR struct lsm6dso_dev_s *priv,
                             uint8_t regaddr,
                             FAR uint8_t *regval,
                             uint8_t len);
static void lsm6dso_spi_write(FAR struct lsm6dso_dev_s *priv,
                              uint8_t regaddr,
                              FAR uint8_t *value);

/* Sensor handle functions */

/* Lsm6dso handle functions */

static int lsm6dso_readdevid(FAR struct lsm6dso_dev_s *priv);
static int lsm6dso_datatest(FAR struct lsm6dso_dev_s *priv, int type);
static int lsm6dso_reset(FAR struct lsm6dso_dev_s *priv);
static int lsm6dso_resetwait(FAR struct lsm6dso_dev_s *priv);
static int lsm6dso_seti3c(FAR struct lsm6dso_dev_s *priv, uint8_t value);
static int lsm6dso_setupdate(FAR struct lsm6dso_dev_s *priv, uint8_t value);
static int lsm6dso_embedded_setsens(FAR struct lsm6dso_dev_s *priv,
                                    FAR lsm6dso_emb_sens_t *value);
static void lsm6dso_copybyte(FAR uint8_t *target, FAR uint8_t *source);
static int lsm6dso_mem_setbank(FAR struct lsm6dso_dev_s *priv,
                               uint8_t value);
static int lsm6dso_pg_writelnbyte(FAR struct lsm6dso_dev_s *priv,
                                  uint16_t address, FAR uint8_t *value);
static int lsm6dso_pg_writeln(FAR struct lsm6dso_dev_s *priv,
                              uint16_t address, FAR uint8_t *buf,
                              uint8_t len);
static int lsm6dso_temp_getdata(FAR struct lsm6dso_dev_s *priv,
                                uint8_t regaddr, FAR float *value);

/* Accelerator handle functions */

static int lsm6dso_xl_setselftest(FAR struct lsm6dso_dev_s *priv,
                                  uint8_t value);
static int lsm6dso_xl_findodr(FAR float *freq);
static int lsm6dso_xl_setodr(FAR struct lsm6dso_dev_s *priv, uint8_t value);
static int lsm6dso_xl_setfullscale(FAR struct lsm6dso_dev_s *priv,
                                   uint8_t value);
static int lsm6dso_xl_enable(FAR struct lsm6dso_dev_s *priv,
                             bool enable);
static int lsm6dso_xl_isready(FAR struct lsm6dso_dev_s *priv,
                              FAR uint8_t *value);
static int lsm6dso_xl_getdata(FAR struct lsm6dso_dev_s *priv,
                              uint8_t regaddr,
                              FAR struct sensor_accel *value);
static void lsm6dso_xl_worker(FAR void *arg);

/* Gyroscope handle functions */

static int lsm6dso_gy_setselftest(FAR struct lsm6dso_dev_s *priv,
                                  uint8_t value);
static int lsm6dso_gy_findodr(FAR float *freq);
static int lsm6dso_gy_setodr(FAR struct lsm6dso_dev_s *priv, uint8_t value);
static int lsm6dso_gy_setfullscale(FAR struct lsm6dso_dev_s *priv,
                                   uint8_t value);
static int lsm6dso_gy_enable(FAR struct lsm6dso_dev_s *priv,
                             bool enable);
static int lsm6dso_gy_isready(FAR struct lsm6dso_dev_s *priv,
                              FAR uint8_t *value);
static int lsm6dso_gy_getdata(FAR struct lsm6dso_dev_s *priv,
                              uint8_t regaddr,
                              FAR struct sensor_gyro *value);
static void lsm6dso_gy_worker(FAR void *arg);

/* FIFO handle functions */

static int lsm6dso_fifo_setmode(FAR struct lsm6dso_dev_s *priv,
                                uint8_t value);
static int lsm6dso_fifo_setwatermark(FAR struct lsm6dso_dev_s *priv,
                                     unsigned int value);
static int lsm6dso_fifo_xl_setbatch(FAR struct lsm6dso_dev_s *priv,
                                    uint8_t value);
static int lsm6dso_fifo_gy_setbatch(FAR struct lsm6dso_dev_s *priv,
                                    uint8_t value);
static int lsm6dso_fifo_findbdr(FAR struct lsm6dso_sensor_s *sensor);
static int lsm6dso_fifo_getlevel(FAR struct lsm6dso_dev_s *priv,
                                 unsigned int *value);
static int lsm6dso_fifo_gettag(FAR struct lsm6dso_dev_s *priv,
                               uint8_t *value);
static int lsm6dso_fifo_flushdata(FAR struct lsm6dso_dev_s *priv);
static int lsm6dso_fifo_readdata(FAR struct lsm6dso_dev_s *priv);

/* Finite state machine handle functions */

static int lsm6dso_fsm_enable(FAR struct lsm6dso_dev_s *priv,
                              bool enable);
static int lsm6dso_fsm_manage(FAR struct lsm6dso_dev_s *priv);
static int lsm6dso_fsm_findodr(FAR float *freq);
static int lsm6dso_fsm_setstartaddr(FAR struct lsm6dso_dev_s *priv,
                                    uint16_t value);
static int lsm6dso_fsm_setprogramnum(FAR struct lsm6dso_dev_s *priv,
                                     uint8_t value);
static int lsm6dso_fsm_setenable(FAR struct lsm6dso_dev_s *priv,
                                 FAR lsm6dso_emb_fsm_enable_t *value);
static int lsm6dso_fsm_setodr(FAR struct lsm6dso_dev_s *priv,
                              uint8_t value);
static int lsm6dso_fsm_handler(FAR struct lsm6dso_dev_s *priv,
                               FAR lsm6dso_all_sources_t *status);

/* Sensor ops functions */

static int lsm6dso_batch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR unsigned long *latency_us);
static int lsm6dso_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *period_us);
static int lsm6dso_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            bool enable);
static int lsm6dso_selftest(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            unsigned long arg);
static int lsm6dso_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg);

/* Sensor interrupt functions */

static int lsm6dso_int1_getroute(FAR struct lsm6dso_dev_s *priv,
                                 FAR lsm6dso_pin_int1_route_t *value);
static int lsm6dso_int1_setroute(FAR struct lsm6dso_dev_s *priv,
                                 lsm6dso_pin_int1_route_t value);
static int lsm6dso_int2_getroute(FAR struct lsm6dso_dev_s *priv,
                                 FAR lsm6dso_pin_int2_route_t *value);
static int lsm6dso_int_setnotification(FAR struct lsm6dso_dev_s *priv,
                                       lsm6dso_lir_t value);
static int lsm6dso_int_setlongcnt(FAR struct lsm6dso_dev_s *priv,
                                  uint16_t value);
static int lsm6dso_int_getall(FAR struct lsm6dso_dev_s *priv,
                              FAR lsm6dso_all_sources_t *value);
static int lsm6dso_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg);
static void lsm6dso_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_lsm6dso_xl_ops =
{
  .activate = lsm6dso_activate,         /* Enable/disable sensor */
  .set_interval = lsm6dso_set_interval, /* Set output data period */
  .batch = lsm6dso_batch,               /* Set maximum report latency */
  .selftest = lsm6dso_selftest,         /* Sensor selftest function */
  .control = lsm6dso_control            /* Set special config for sensor */
};

static const struct sensor_ops_s g_lsm6dso_gy_ops =
{
  .activate = lsm6dso_activate,         /* Enable/disable sensor */
  .set_interval = lsm6dso_set_interval, /* Set output data period */
  .batch = lsm6dso_batch,               /* Set maximum report latency */
  .selftest = lsm6dso_selftest          /* Sensor selftest function */
};

static const struct sensor_ops_s g_lsm6dso_fsm_ops =
{
  .activate = lsm6dso_activate,         /* Enable/disable sensor */
  .set_interval = lsm6dso_set_interval, /* Set output data period */
  .control = lsm6dso_control            /* Set special config for sensor */
};

static const struct lsm6dso_odr_s g_lsm6dso_xl_odr[] =
{
  {LSM6DSO_XL_ODR_12p5HZ, 12.5},        /* Sampling interval is 80ms */
  {LSM6DSO_XL_ODR_26HZ,   26},          /* Sampling interval is about 38.462ms */
  {LSM6DSO_XL_ODR_52HZ,   52},          /* Sampling interval is about 19.231ms */
  {LSM6DSO_XL_ODR_104HZ,  104},         /* Sampling interval is about 9.616ms */
  {LSM6DSO_XL_ODR_208HZ,  208},         /* Sampling interval is about 4.808ms */
  {LSM6DSO_XL_ODR_416HZ,  416},         /* Sampling interval is about 2.404ms */
  {LSM6DSO_XL_ODR_833HZ,  833},         /* Sampling interval is about 1.201ms */
};

static const struct lsm6dso_odr_s g_lsm6dso_gy_odr[] =
{
  {LSM6DSO_GY_ODR_12p5HZ, 12.5},        /* Sampling interval is 80ms */
  {LSM6DSO_GY_ODR_26HZ,   26},          /* Sampling interval is about 38.462ms */
  {LSM6DSO_GY_ODR_52HZ,   52},          /* Sampling interval is about 19.231ms */
  {LSM6DSO_GY_ODR_104HZ,  104},         /* Sampling interval is about 9.616ms */
  {LSM6DSO_GY_ODR_208HZ,  208},         /* Sampling interval is about 4.808ms */
  {LSM6DSO_GY_ODR_416HZ,  416},         /* Sampling interval is about 2.404ms */
  {LSM6DSO_GY_ODR_833HZ,  833},         /* Sampling interval is about 1.201ms */
};

static const struct lsm6dso_odr_s g_lsm6dso_fsm_odr[] =
{
  {LSM6DSO_FSM_ODR_12p5HZ, 12.5},       /* FSM interval is 80ms */
  {LSM6DSO_FSM_ODR_26HZ,   26},         /* FSM interval is about 38.462ms */
  {LSM6DSO_FSM_ODR_52HZ,   52},         /* FSM interval is about 19.231ms */
  {LSM6DSO_FSM_ODR_104HZ,  104},        /* FSM interval is about 9.616ms */
};

static const struct lsm6dso_bdr_s g_lsm6dso_xl_bdr[] =
{
  {LSM6DSO_XL_BDR_12p5Hz, 12.5},        /* Sampling interval is 80ms */
  {LSM6DSO_XL_BDR_26Hz,   26},          /* Sampling interval is about 38.462ms */
  {LSM6DSO_XL_BDR_52Hz,   52},          /* Sampling interval is about 19.231ms */
  {LSM6DSO_XL_BDR_104Hz,  104},         /* Sampling interval is about 9.616ms */
  {LSM6DSO_XL_BDR_208Hz,  208},         /* Sampling interval is about 4.808ms */
  {LSM6DSO_XL_BDR_417Hz,  417},         /* Sampling interval is about 2.398ms */
  {LSM6DSO_XL_BDR_833Hz,  833},         /* Sampling interval is about 1.201ms */
};

static const struct lsm6dso_bdr_s g_lsm6dso_gy_bdr[] =
{
  {LSM6DSO_GY_BDR_12p5Hz, 12.5},        /* Sampling interval is 80ms */
  {LSM6DSO_GY_BDR_26Hz,   26},          /* Sampling interval is about 38.462ms */
  {LSM6DSO_GY_BDR_52Hz,   52},          /* Sampling interval is about 19.231ms */
  {LSM6DSO_GY_BDR_104Hz,  104},         /* Sampling interval is about 9.616ms */
  {LSM6DSO_GY_BDR_208Hz,  208},         /* Sampling interval is about 4.808ms */
  {LSM6DSO_GY_BDR_417Hz,  417},         /* Sampling interval is about 2.398ms */
  {LSM6DSO_GY_BDR_833Hz,  833},         /* Sampling interval is about 1.201ms */
};

/* Programs can be extracted from ".ucf" configuration file generated
 * by Unico / Unicleo tool.
 */

/* Program: wrist tilt(only xl) */

const uint8_t g_lsm6so_prg_wrist_tilt_xl[] =
{
  0x52, 0x00, 0x14, 0x00, 0x0d, 0x00, 0x8e, 0x31, 0x10, 0x00, 0x00, 0x0d,
  0x06, 0x23, 0x00, 0x53, 0x33, 0x74, 0x44, 0x22
};

/* Program: wrist tilt (left) */

const uint8_t g_lsm6so_prg_wrist_tilt_left[] =
{
  0xb1, 0x30, 0x34, 0x00, 0x1d, 0x00, 0x96, 0x31, 0x00, 0xb8, 0x80, 0x00,
  0x20, 0x00, 0x80, 0x00, 0xec, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x08, 0x00, 0x66, 0xaa, 0x96, 0x31, 0x23, 0x01, 0xa5,
  0xff, 0x42, 0x0a, 0xaa, 0x96, 0x39, 0x23, 0x07, 0xa5, 0x23, 0x00, 0x77,
  0x83, 0x88, 0x86, 0x22
};

/* Program: wrist tilt (right) */

const uint8_t g_lsm6so_prg_wrist_tilt_right[] =
{
  0xb1, 0x30, 0x34, 0x00, 0x1d, 0x00, 0x96, 0x31, 0x00, 0xb8, 0x80, 0x00,
  0x20, 0x00, 0x40, 0x00, 0xec, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x08, 0x00, 0x66, 0xaa, 0x96, 0x31, 0x23, 0x01, 0xa5,
  0xff, 0x42, 0x0a, 0xaa, 0x96, 0x39, 0x23, 0x07, 0xa5, 0x23, 0x00, 0x77,
  0x83, 0x88, 0x86, 0x22
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* SPI functions */

/****************************************************************************
 * Name: lsm6dso_spi_read
 *
 * Description:
 *   Read 16-bit register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Register address.
 *   regval  - Register value.
 *   len     - Read data lenth.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void lsm6dso_spi_read(FAR struct lsm6dso_dev_s *priv,
                             uint8_t regaddr,
                             FAR uint8_t *regval,
                             uint8_t len)
{
  uint8_t sendbuffer[LSM6DSO_SPI_MAX_BUFFER];
  uint8_t revbuffer[LSM6DSO_SPI_MAX_BUFFER];

  /* Lock the SPI bus so that only one device can access it at the same
   * time.
   */

  SPI_LOCK(priv->config->spi, true);

  /* Set SPI frequency and mode. */

  SPI_SETFREQUENCY(priv->config->spi, priv->config->freq);
  SPI_SETMODE(priv->config->spi, SPIDEV_MODE3);

  /* Set CS to low which selects the LSM6DSO. */

  SPI_SELECT(priv->config->spi, priv->config->cs, true);

  /* Transmit the register address from where we want to read - the MSB
   * needs to be set to indicate the read indication. Then Write some
   * idle byte while receiving the required data.
   */

  sendbuffer[0] = regaddr | 0x80;
  SPI_EXCHANGE(priv->config->spi, sendbuffer, revbuffer, (len + 1));

  /* Copy data from buffer to receive array. */

  memcpy(regval, revbuffer + 1, len);

  /* Set CS to high which deselects the LSM6DSO. */

  SPI_SELECT(priv->config->spi, priv->config->cs, false);

  /* Unlock the SPI bus. */

  SPI_LOCK(priv->config->spi, false);
}

/****************************************************************************
 * Name: lsm6dso_spi_write
 *
 * Description:
 *   write 16-bit register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Register address.
 *   value   - To be write value.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void lsm6dso_spi_write(FAR struct lsm6dso_dev_s *priv,
                              uint8_t regaddr,
                              FAR uint8_t *value)
{
  uint8_t sendbuffer[2];
  uint8_t revbuffer[2];

  /* Lock the SPI bus so that only one device can access it at the same
   * time.
   */

  SPI_LOCK(priv->config->spi, true);

  /* Set SPI frequency and mode. */

  SPI_SETFREQUENCY(priv->config->spi, priv->config->freq);
  SPI_SETMODE(priv->config->spi, SPIDEV_MODE3);

  /* Set CS to low which selects the LIS3MDL. */

  SPI_SELECT(priv->config->spi, priv->config->cs, true);

  /* Transmit the register address from where we want to read. Then
   * transmit the content which should be written in the register.
   */

  sendbuffer[0] = regaddr;
  sendbuffer[1] = *value;
  SPI_EXCHANGE(priv->config->spi, sendbuffer, revbuffer, 2);

  /* Set CS to high which deselects the LIS3MDL. */

  SPI_SELECT(priv->config->spi, priv->config->cs, false);

  /* Unlock the SPI bus. */

  SPI_LOCK(priv->config->spi, false);
}

/* Sensor handle functions */

/****************************************************************************
 * Name: lsm6dso_readdevid
 *
 * Description:
 *   Read the device ID. The device ID reads 0x6Ch.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int lsm6dso_readdevid(FAR struct lsm6dso_dev_s *priv)
{
  uint8_t regval;
  int ret;

  lsm6dso_spi_read(priv, LSM6DSO_WHO_AM_I, &regval, 1);
  if (regval != LSM6DSO_DEVICE_ID)
    {
      ret = -ENODEV;
      snerr("Wrong device ID! : %d\n", regval);
    }
  else
    {
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_datatest
 *
 * Description:
 *   Selftesting the sensor.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   Return LSM6DSO_ST_PASS if the selftest was success;
 *   Return LSM6DSO_ST_FAIL if the selftest was failed.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_datatest(FAR struct lsm6dso_dev_s *priv, int type)
{
  struct sensor_accel temp_xl;
  struct sensor_gyro temp_gy;
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];
  int st_result;
  uint8_t valuezero;
  uint8_t drdy;
  uint8_t i;

  /* Ensure these registers set to zero. */

  valuezero = 0;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL2_G, &valuezero);
  lsm6dso_spi_write(priv, LSM6DSO_CTRL4_C, &valuezero);
  lsm6dso_spi_write(priv, LSM6DSO_CTRL5_C, &valuezero);
  lsm6dso_spi_write(priv, LSM6DSO_CTRL6_C, &valuezero);
  lsm6dso_spi_write(priv, LSM6DSO_CTRL7_G, &valuezero);
  lsm6dso_spi_write(priv, LSM6DSO_CTRL8_XL, &valuezero);
  lsm6dso_spi_write(priv, LSM6DSO_CTRL9_XL, &valuezero);
  lsm6dso_spi_write(priv, LSM6DSO_CTRL10_C, &valuezero);

  if (type == SENSOR_TYPE_ACCELEROMETER)        /* Accelerometer Self Test */
    {
      /* Set Output Data Rate. */

      lsm6dso_xl_setodr(priv, LSM6DSO_XL_ODR_52HZ);

      /* Set full scale. */

      lsm6dso_xl_setfullscale(priv, LSM6DSO_4G);
      priv->dev[LSM6DSO_XL_IDX].factor = LSM6DSO_4G_FACTOR;

      /* Wait stable output. */

      up_mdelay(LSM6DSO_SET_DELAY);             /* 100ms */

      /* Check if new value available. */

      do
        {
          up_mdelay(LSM6DSO_READ_DELAY);
          lsm6dso_xl_isready(priv, &drdy);
        }
      while (!drdy);

      /* Read dummy data and discard it. */

      lsm6dso_xl_getdata(priv, LSM6DSO_OUTX_L_XL, &temp_xl);

      /* Read 5 sample and get the average vale for each axis. */

      memset(val_st_off, 0x00, sizeof(val_st_off));

      for (i = 0; i < 5; i++)
        {
          /* Check if new value available. */

          do
            {
              up_mdelay(LSM6DSO_READ_DELAY);
              lsm6dso_xl_isready(priv, &drdy);
            }
          while (!drdy);

          /* Read data and accumulate the mg value. */

          lsm6dso_xl_getdata(priv, LSM6DSO_OUTX_L_XL, &temp_xl);

          val_st_off[0] = val_st_off[0] + temp_xl.x;
          val_st_off[1] = val_st_off[1] + temp_xl.y;
          val_st_off[2] = val_st_off[2] + temp_xl.z;

          sninfo("accel x normal data ->: %d:%f mg\n", i, temp_xl.x);
          sninfo("accel y normal data ->: %d:%f mg\n", i, temp_xl.y);
          sninfo("accel z normal data ->: %d:%f mg\n", i, temp_xl.z);
        }

      /* Calculate the mg average values. */

      for (i = 0; i < 3; i++)
        {
          val_st_off[i] = val_st_off[i] / 5.0f;
        }

      /* Enable Self Test positive (or negative). */

      lsm6dso_xl_setselftest(priv, LSM6DSO_POSIGN_MODE);

      /* Wait stable output. */

      up_mdelay(LSM6DSO_SET_DELAY);         /* 100ms */

      /* Check if new value available. */

      do
        {
          up_mdelay(LSM6DSO_READ_DELAY);
          lsm6dso_xl_isready(priv, &drdy);
        }
      while (!drdy);

      /* Read dummy data and discard it. */

      lsm6dso_xl_getdata(priv, LSM6DSO_OUTX_L_XL, &temp_xl);

      /* Read 5 sample and get the average vale for each axis. */

      memset(val_st_on, 0x00, sizeof(val_st_on));

      for (i = 0; i < 5; i++)
        {
          /* Check if new value available. */

          do
            {
              up_mdelay(LSM6DSO_READ_DELAY);
              lsm6dso_xl_isready(priv, &drdy);
            }
          while (!drdy);

          /* Read data and accumulate the mg value. */

          lsm6dso_xl_getdata(priv, LSM6DSO_OUTX_L_XL, &temp_xl);

          val_st_on[0] = val_st_on[0] + temp_xl.x;
          val_st_on[1] = val_st_on[1] + temp_xl.y;
          val_st_on[2] = val_st_on[2] + temp_xl.z;

          sninfo("accel x selftest data ->: %d:%f mg\n", i, temp_xl.x);
          sninfo("accel y selftest data ->: %d:%f mg\n", i, temp_xl.y);
          sninfo("accel z selftest data ->: %d:%f mg\n", i, temp_xl.z);
        }

      /* Calculate the mg average values. */

      for (i = 0; i < 3; i++)
        {
          val_st_on[i] = val_st_on[i] / 5.0f;
        }

      /* Calculate the mg values for self test. */

      for (i = 0; i < 3; i++)
        {
          test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
        }

      /* Check self test limit. */

      st_result = LSM6DSO_ST_PASS;

      for (i = 0; i < 3; i++)
        {
          if ((LSM6DSO_MIN_ST_LIMIT_MG > test_val[i])
            || (test_val[i] > LSM6DSO_MAX_ST_LIMIT_MG))
            {
              st_result = LSM6DSO_ST_FAIL;
              sninfo("acceleration %d LSM6DSO_ST_FAIL : "
              "min limit:%f  test_val[%d]:%f  max limit:%f\n",
              i, LSM6DSO_MIN_ST_LIMIT_MG,
              i, test_val[i], LSM6DSO_MAX_ST_LIMIT_MG);
            }
        }

      /* Disable Self Test. */

      lsm6dso_xl_setselftest(priv, LSM6DSO_NORMAL_MODE);

      /* Disable sensor. */

      lsm6dso_xl_setodr(priv, LSM6DSO_XL_ODR_OFF);
    }
  else if (type == SENSOR_TYPE_GYROSCOPE)   /* Gyroscope Self Test */
    {
      /* Set Output Data Rate. */

      lsm6dso_gy_setodr(priv, LSM6DSO_GY_ODR_208HZ);

      /* Set full scale. */

      lsm6dso_gy_setfullscale(priv, LSM6DSO_2000DPS);

      priv->dev[LSM6DSO_GY_IDX].factor = LSM6DSO_2000DPS_FACTOR;

      /* Wait stable output. */

      up_mdelay(LSM6DSO_SET_DELAY);         /* 100ms */

      /* Check if new value available. */

      do
        {
          up_mdelay(LSM6DSO_READ_DELAY);
          lsm6dso_gy_isready(priv, &drdy);
        }
      while (!drdy);

      /* Read dummy data and discard it. */

      lsm6dso_gy_getdata(priv, LSM6DSO_OUTX_L_G, &temp_gy);

      /* Read 5 sample and get the average vale for each axis. */

      memset(val_st_off, 0x00, sizeof(val_st_off));

      for (i = 0; i < 5; i++)
        {
          /* Check if new value available. */

          do
            {
              up_mdelay(LSM6DSO_READ_DELAY);
              lsm6dso_gy_isready(priv, &drdy);
            }
          while (!drdy);

          /* Read data and accumulate the mdps value. */

          lsm6dso_gy_getdata(priv, LSM6DSO_OUTX_L_G, &temp_gy);

          val_st_off[0] = val_st_off[0] + temp_gy.x;
          val_st_off[1] = val_st_off[1] + temp_gy.y;
          val_st_off[2] = val_st_off[2] + temp_gy.z;

          sninfo("gyro x normal data ->: %d:%f mdps\n", i, temp_gy.x);
          sninfo("gyro y normal data ->: %d:%f mdps\n", i, temp_gy.y);
          sninfo("gyro z normal data ->: %d:%f mdps\n", i, temp_gy.z);
        }

      /* Calculate the mdps average values. */

      for (i = 0; i < 3; i++)
        {
          val_st_off[i] = val_st_off[i] / 5.0f;
        }

      /* Enable Self Test positive (or negative). */

      lsm6dso_gy_setselftest(priv, LSM6DSO_POSIGN_MODE);

      /* Wait stable output. */

      up_mdelay(LSM6DSO_SET_DELAY);               /* 100ms */

      /* Check if new value available. */

      do
        {
          up_mdelay(LSM6DSO_READ_DELAY);
          lsm6dso_gy_isready(priv, &drdy);
        }
      while (!drdy);

      /* Read dummy data and discard it. */

      lsm6dso_gy_getdata(priv, LSM6DSO_OUTX_L_G, &temp_gy);

      /* Read 5 sample and get the average vale for each axis. */

      memset(val_st_on, 0x00, sizeof(val_st_on));

      for (i = 0; i < 5; i++)
        {
          /* Check if new value available. */

          do
            {
              up_mdelay(LSM6DSO_READ_DELAY);
              lsm6dso_gy_isready(priv, &drdy);
            }
          while (!drdy);

          /* Read data and accumulate the mdps value. */

          lsm6dso_gy_getdata(priv, LSM6DSO_OUTX_L_G, &temp_gy);

          val_st_on[0] = val_st_on[0] + temp_gy.x;
          val_st_on[1] = val_st_on[1] + temp_gy.y;
          val_st_on[2] = val_st_on[2] + temp_gy.z;

          sninfo("gyro x selftest data ->: %d:%f mdps\n", i, temp_gy.x);
          sninfo("gyro y selftest data ->: %d:%f mdps\n", i, temp_gy.y);
          sninfo("gyro z selftest data ->: %d:%f mdps\n", i, temp_gy.z);
        }

      /* Calculate the mdps average values. */

      for (i = 0; i < 3; i++)
        {
          val_st_on[i] = val_st_on[i] / 5.0f;
        }

      /* Calculate the mg values for self test. */

      for (i = 0; i < 3; i++)
        {
          test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
        }

      /* Check self test limit. */

      st_result = LSM6DSO_ST_PASS;

      for (i = 0; i < 3; i++)
        {
          if ((LSM6DSO_MIN_ST_LIMIT_MDPS > test_val[i])
            || (test_val[i] > LSM6DSO_MAX_ST_LIMIT_MDPS))
            {
              st_result = LSM6DSO_ST_FAIL;
              sninfo("angular %d LSM6DSO_ST_FAIL : "
              "min limit:%f  test_val[%d]:%f  max limit:%f\n",
              i, LSM6DSO_MIN_ST_LIMIT_MDPS, i,
              test_val[i], LSM6DSO_MAX_ST_LIMIT_MDPS);
            }
        }

      /* Disable Self Test. */

      lsm6dso_gy_setselftest(priv, LSM6DSO_NORMAL_MODE);

      /* Disable sensor. */

      lsm6dso_gy_setodr(priv, LSM6DSO_GY_ODR_OFF);
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  if (st_result == LSM6DSO_ST_PASS)
    {
      sninfo("Self Test - PASS\n");
    }
  else
    {
      sninfo("Self Test - FAIL\n");
    }

  return st_result;
}

/****************************************************************************
 * Name: lsm6dso_reset
 *
 * Description:
 *   Reset the sensor to re-load the trimming parameters.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_reset(FAR struct lsm6dso_dev_s *priv)
{
  lsm6dso_ctrl3_c_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL3_C, (FAR uint8_t *)&reg, 1);

  reg.sw_reset = 1;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL3_C, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_resetwait
 *
 * Description:
 *   Waitting for sensor reset.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_resetwait(FAR struct lsm6dso_dev_s *priv)
{
  lsm6dso_ctrl3_c_t reg;
  uint16_t maxcount = LSM6DSO_WAIT_COUNT_MAX;

  do
    {
      up_mdelay(LSM6DSO_SET_DELAY);
      lsm6dso_spi_read(priv, LSM6DSO_CTRL3_C, (FAR uint8_t *)&reg, 1);
      maxcount--;
    }
  while (reg.sw_reset && maxcount);

  if (maxcount == 0)
    {
      snerr("lsm6dso reset wait timeout!\n");
    }

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_seti3c
 *
 * Description:
 *   Disables MIPI I3C communication protocol. It is recommended to set this
 *   bit to '1' during the initial device configuration phase, when the I3C
 *   interface is not used.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Data state.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_seti3c(FAR struct lsm6dso_dev_s *priv, uint8_t value)
{
  lsm6dso_i3c_bus_avb_t i3c_bus_avb;
  lsm6dso_ctrl9_xl_t ctrl9_xl;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL9_XL, (FAR uint8_t *)&ctrl9_xl, 1);

  ctrl9_xl.i3c_disable = (value & 0x80) >> 7;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL9_XL, (FAR uint8_t *)&ctrl9_xl);

  lsm6dso_spi_read(priv, LSM6DSO_I3C_BUS_AVB,
                  (FAR uint8_t *)&i3c_bus_avb, 1);

  i3c_bus_avb.i3c_bus_avb_sel = value & 0x03;
  lsm6dso_spi_write(priv, LSM6DSO_I3C_BUS_AVB,
                   (FAR uint8_t *)&i3c_bus_avb);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_setupdate
 *
 * Description:
 *   Set block data update mode.
 *   0: continuous update;
 *   1: output registers are not updated until MSB and LSB have been read.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - The mode of block data update.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_setupdate(FAR struct lsm6dso_dev_s *priv, uint8_t value)
{
  lsm6dso_ctrl3_c_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL3_C, (FAR uint8_t *)&reg, 1);

  reg.bdu = value;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL3_C, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_embedded_setsens
 *
 * Description:
 *   Embedded functions.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   value   - Change the values of registers.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_embedded_setsens(FAR struct lsm6dso_dev_s *priv,
                                    FAR lsm6dso_emb_sens_t *value)
{
  lsm6dso_emb_func_en_a_t emb_func_en_a;
  lsm6dso_emb_func_en_b_t emb_func_en_b;
  int ret;

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_EMBEDDEDFUNC_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
            ret);
      return ret;
    }

  lsm6dso_spi_read(priv, LSM6DSO_EMB_FUNC_EN_A,
                   (FAR uint8_t *)&emb_func_en_a, 1);
  lsm6dso_spi_read(priv, LSM6DSO_EMB_FUNC_EN_B,
                   (FAR uint8_t *)&emb_func_en_b, 1);
  emb_func_en_b.fsm_en = value->fsm;
  emb_func_en_a.tilt_en = value->tilt;
  emb_func_en_a.pedo_en = value->step;
  emb_func_en_b.pedo_adv_en = value->step_adv;
  emb_func_en_a.sign_motion_en = value->sig_mot;
  emb_func_en_b.fifo_compr_en = value->fifo_compr;
  lsm6dso_spi_write(priv, LSM6DSO_EMB_FUNC_EN_A,
                    (FAR uint8_t *)&emb_func_en_a);
  lsm6dso_spi_write(priv, LSM6DSO_EMB_FUNC_EN_B,
                    (FAR uint8_t *)&emb_func_en_b);

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_USER_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
            ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_copybyte
 *
 * Description:
 *   Copy a byte from source to target.
 *
 * Input Parameters:
 *   target   - Target address.
 *   source   - Source address.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void lsm6dso_copybyte(FAR uint8_t *target, FAR uint8_t *source)
{
  if ((target != NULL) && (source != NULL))
    {
      *target = *source;
    }
}

/****************************************************************************
 * Name: lsm6dso_mem_setbank
 *
 * Description:
 *   Enable access to the embedded functions/sensor hub
 *   configuration registers.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Change the values of reg_access.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_mem_setbank(FAR struct lsm6dso_dev_s *priv,
                               uint8_t value)
{
  lsm6dso_func_cfg_access_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_FUNC_CFG_ACCESS, (FAR uint8_t *)&reg, 1);

  reg.reg_access = (uint8_t)value;
  lsm6dso_spi_write(priv, LSM6DSO_FUNC_CFG_ACCESS, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_pg_writelnbyte
 *
 * Description:
 *   Write a line(byte) in a page.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   address - page line address.
 *   value   - value to write.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_pg_writelnbyte(FAR struct lsm6dso_dev_s *priv,
                                  uint16_t address, FAR uint8_t *value)
{
  lsm6dso_page_rw_t page_rw;
  lsm6dso_page_sel_t page_sel;
  lsm6dso_page_address_t page_address;
  int ret;

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_EMBEDDEDFUNC_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
            ret);
      return ret;
    }

  lsm6dso_spi_read(priv, LSM6DSO_PAGE_RW, (FAR uint8_t *)&page_rw, 1);
  page_rw.page_rw = LSM6DSO_PG_WRITE_ENABLE;
  lsm6dso_spi_write(priv, LSM6DSO_PAGE_RW, (FAR uint8_t *)&page_rw);

  lsm6dso_spi_read(priv, LSM6DSO_PAGE_SEL, (FAR uint8_t *)&page_sel, 1);
  page_sel.page_sel = ((uint8_t)(address >> 8) & 0x0fu);
  page_sel.not_used_01 = 1;
  lsm6dso_spi_write(priv, LSM6DSO_PAGE_SEL, (FAR uint8_t *)&page_sel);

  page_address.page_addr = (uint8_t)address & 0xffu;
  lsm6dso_spi_write(priv, LSM6DSO_PAGE_ADDRESS,
                    (FAR uint8_t *)&page_address);

  lsm6dso_spi_write(priv, LSM6DSO_PAGE_VALUE, value);

  lsm6dso_spi_read(priv, LSM6DSO_PAGE_RW, (FAR uint8_t *)&page_rw, 1);
  page_rw.page_rw = LSM6DSO_PG_WRITE_DISABLE;
  lsm6dso_spi_write(priv, LSM6DSO_PAGE_RW, (FAR uint8_t *)&page_rw);

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_USER_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
            ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_pg_writeln
 *
 * Description:
 *   Write buffer in a page.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   address - page line address.
 *   buf   - buffer to write.
 *   len   - buffer len.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_pg_writeln(FAR struct lsm6dso_dev_s *priv,
                              uint16_t address, FAR uint8_t *buf,
                              uint8_t len)
{
  lsm6dso_page_rw_t page_rw;
  lsm6dso_page_sel_t page_sel;
  lsm6dso_page_address_t  page_address;
  uint16_t addr_pointed;
  int ret;
  uint8_t i ;

  addr_pointed = address;

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_EMBEDDEDFUNC_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
            ret);
      return ret;
    }

  lsm6dso_spi_read(priv, LSM6DSO_PAGE_RW, (FAR uint8_t *)&page_rw, 1);
  page_rw.page_rw = LSM6DSO_PG_WRITE_ENABLE;
  lsm6dso_spi_write(priv, LSM6DSO_PAGE_RW, (FAR uint8_t *)&page_rw);

  lsm6dso_spi_read(priv, LSM6DSO_PAGE_SEL, (FAR uint8_t *)&page_sel, 1);
  page_sel.page_sel = ((uint8_t)(addr_pointed >> 8) & 0x0fu);
  page_sel.not_used_01 = LSM6DSO_FSM_PAGE_SEL_NU;
  lsm6dso_spi_write(priv, LSM6DSO_PAGE_SEL, (FAR uint8_t *)&page_sel);

  page_address.page_addr = (uint8_t)(addr_pointed & 0x00ffu);
  lsm6dso_spi_write(priv, LSM6DSO_PAGE_ADDRESS,
                    (FAR uint8_t *)&page_address);

  for (i = 0; i < len; i++)
    {
      lsm6dso_spi_write(priv, LSM6DSO_PAGE_VALUE, &buf[i]);
      addr_pointed++;

      /* Check if page wrap */

      if (addr_pointed % 0x0100u == 0x00u && ret == 0)
        {
          lsm6dso_spi_read(priv, LSM6DSO_PAGE_SEL,
                           (FAR uint8_t *)&page_sel, 1);
          page_sel.page_sel = ((uint8_t)(addr_pointed >> 8) & 0x0fu);
          page_sel.not_used_01 = LSM6DSO_FSM_PAGE_SEL_NU;
          lsm6dso_spi_write(priv, LSM6DSO_PAGE_SEL,
                            (FAR uint8_t *)&page_sel);
        }
    }

  page_sel.page_sel = 0;
  page_sel.not_used_01 = LSM6DSO_FSM_PAGE_SEL_NU;
  lsm6dso_spi_write(priv, LSM6DSO_PAGE_SEL, (FAR uint8_t *)&page_sel);

  lsm6dso_spi_read(priv, LSM6DSO_PAGE_RW, (FAR uint8_t *)&page_rw, 1);
  page_rw.page_rw = LSM6DSO_PG_WRITE_DISABLE;
  lsm6dso_spi_write(priv, LSM6DSO_PAGE_RW, (FAR uint8_t *)&page_rw);

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_USER_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_temp_getdata
 *
 * Description:
 *   Read the temperature data.
 *
 * Input Parameters:
 *   priv     -  Device struct.
 *   regaddr  -  Out put data start register address.
 *   value    -  Out put data(celsius).
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_temp_getdata(FAR struct lsm6dso_dev_s *priv,
                                uint8_t regaddr, FAR float *value)
{
  uint8_t buff[2];
  int16_t temp;

  /* Read temperature data. */

  lsm6dso_spi_read(priv, regaddr, buff, 2);
  temp = buff[1] << 8 | buff[0];

  /* Convert lsb to celsius. */

  *value = temp / 256.0f + 25.0f;

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_xl_setselftest
 *
 * Description:
 *   Set the mode of selftest for accelerometer.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Mode of selftest.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_xl_setselftest(FAR struct lsm6dso_dev_s *priv,
                                  uint8_t value)
{
  lsm6dso_ctrl5_c_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL5_C, (FAR uint8_t *)&reg, 1);

  reg.st_xl = value;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL5_C, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_xl_findodr
 *
 * Description:
 *   Find the best matching odr for accelerometer.
 *
 * Input Parameters:
 *   freq  - Desired frequency.
 *
 * Returned Value:
 *   Index of the best fit ODR.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_xl_findodr(FAR float *freq)
{
  int i;
  int num = sizeof(g_lsm6dso_xl_odr) / sizeof(struct lsm6dso_odr_s);

  for (i = 0; i < num; i++)
    {
      if (((*freq < g_lsm6dso_xl_odr[i].odr + LSM6DSO_ODR_FLT_EPSILON)
          && (*freq > g_lsm6dso_xl_odr[i].odr - LSM6DSO_ODR_FLT_EPSILON))
          || *freq < g_lsm6dso_xl_odr[i].odr)
        {
          *freq = g_lsm6dso_xl_odr[i].odr;
          return i;
        }
    }

  return num - 1;
}

/****************************************************************************
 * Name: lsm6dso_xl_setodr
 *
 * Description:
 *   Set odr for accelerometer.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - The value set to register.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_xl_setodr(FAR struct lsm6dso_dev_s *priv, uint8_t value)
{
  lsm6dso_ctrl1_xl_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL1_XL, (FAR uint8_t *)&reg, 1);

  reg.odr_xl = value;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL1_XL, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_xl_setfullscale
 *
 * Description:
 *   Set full scale for accelerometer.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - value of full scale.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_xl_setfullscale(FAR struct lsm6dso_dev_s *priv,
                                   uint8_t value)
{
  lsm6dso_ctrl1_xl_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL1_XL, (FAR uint8_t *)&reg, 1);

  reg.fs_xl = value;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL1_XL, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_xl_enable
 *
 * Description:
 *   Enable or disable sensor device.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int lsm6dso_xl_enable(FAR struct lsm6dso_dev_s *priv,
                             bool enable)
{
  if (enable)
    {
      /* Accelerometer config registers:
       * Turn on the accelerometer: +-8g.
       */

      lsm6dso_xl_setfullscale(priv, LSM6DSO_8G);
      priv->dev[LSM6DSO_XL_IDX].factor = LSM6DSO_8G_FACTOR
                                       * LSM6DSO_MG2MS_FACTOR;

      /* Set worker for accelerometer. */

      if (priv->dev[LSM6DSO_XL_IDX].fifoen)
        {
          work_cancel(HPWORK, &priv->dev[LSM6DSO_XL_IDX].work);
        }
      else
        {
          work_queue(HPWORK, &priv->dev[LSM6DSO_XL_IDX].work,
                     lsm6dso_xl_worker, priv,
                     priv->dev[LSM6DSO_XL_IDX].interval / USEC_PER_TICK);
        }
    }
  else
    {
      /* Set to Shut Down. */

      work_cancel(HPWORK, &priv->dev[LSM6DSO_XL_IDX].work);
      lsm6dso_xl_setodr(priv, LSM6DSO_XL_ODR_OFF);
    }

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_xl_isready
 *
 * Description:
 *   Read the accelerometer data ready flag.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Flag state.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_xl_isready(FAR struct lsm6dso_dev_s *priv,
                              FAR uint8_t *value)
{
  lsm6dso_status_reg_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_STATUS_REG, (FAR uint8_t *)&reg, 1);
  *value = reg.xlda;

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_xl_getdata
 *
 * Description:
 *   Read the accelerometer data.
 *
 * Input Parameters:
 *   priv     -  Device struct.
 *   regaddr  -  Out put data start register address.
 *   value    -  Out put data.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_xl_getdata(FAR struct lsm6dso_dev_s *priv,
                              uint8_t regaddr,
                              FAR struct sensor_accel *value)
{
  axis3bit16_t temp;

  lsm6dso_spi_read(priv, regaddr, temp.u8bit, 6);
  sensor_remap_vector_raw16(temp.i16bit, temp.i16bit, LSM6DSO_VECTOR_REMAP);

  value->x = temp.i16bit[0] *priv->dev[LSM6DSO_XL_IDX].factor;
  value->y = temp.i16bit[1] *priv->dev[LSM6DSO_XL_IDX].factor;
  value->z = temp.i16bit[2] *priv->dev[LSM6DSO_XL_IDX].factor;

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_xl_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void lsm6dso_xl_worker(FAR void *arg)
{
  FAR struct lsm6dso_dev_s *priv = arg;
  struct sensor_accel temp_xl;

  /* Sanity check. */

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp. */

  temp_xl.timestamp = sensor_get_timestamp();

  /* Set work queue. */

  work_queue(HPWORK, &priv->dev[LSM6DSO_XL_IDX].work, lsm6dso_xl_worker,
             priv, priv->dev[LSM6DSO_XL_IDX].interval / USEC_PER_TICK);

  /* Read out the latest sensor data. */

  lsm6dso_xl_getdata(priv, LSM6DSO_OUTX_L_XL, &temp_xl);

  /* Read out the latest temperature data. */

  lsm6dso_temp_getdata(priv, LSM6DSO_OUT_TEMP_L, &temp_xl.temperature);

  /* push data to upper half driver. */

  priv->dev[LSM6DSO_XL_IDX].lower.push_event(
        priv->dev[LSM6DSO_XL_IDX].lower.priv,
        &temp_xl,
        sizeof(struct sensor_accel));
}

/****************************************************************************
 * Name: lsm6dso_gy_setselftest
 *
 * Description:
 *   Set the mode of selftest for gyroscope.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Mode of selftest.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_gy_setselftest(FAR struct lsm6dso_dev_s *priv,
                                  uint8_t value)
{
  lsm6dso_ctrl5_c_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL5_C, (FAR uint8_t *)&reg, 1);

  reg.st_g  = value;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL5_C, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_gy_findodr
 *
 * Description:
 *   Find the best matching odr for gyroscope.
 *
 * Input Parameters:
 *   freq  - Desired frequency.
 *
 * Returned Value:
 *   Index of the best fit ODR.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_gy_findodr(FAR float *freq)
{
  int i;
  int num = sizeof(g_lsm6dso_gy_odr) / sizeof(struct lsm6dso_odr_s);

  for (i = 0; i < num; i++)
    {
      if (((*freq < g_lsm6dso_gy_odr[i].odr + LSM6DSO_ODR_FLT_EPSILON)
          && (*freq > g_lsm6dso_gy_odr[i].odr - LSM6DSO_ODR_FLT_EPSILON))
          || *freq < g_lsm6dso_gy_odr[i].odr)
        {
          *freq = g_lsm6dso_gy_odr[i].odr;
          return i;
        }
    }

  return num - 1;
}

/****************************************************************************
 * Name: lsm6dso_gy_setodr
 *
 * Description:
 *   Set odr for gyroscope.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - The value set to register.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_gy_setodr(FAR struct lsm6dso_dev_s *priv, uint8_t value)
{
  lsm6dso_ctrl2_g_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL2_G, (FAR uint8_t *)&reg, 1);

  reg.odr_g = value;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL2_G, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_gy_setfullscale
 *
 * Description:
 *   Set full scale for gyroscope.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - value of full scale.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_gy_setfullscale(FAR struct lsm6dso_dev_s *priv,
                                   uint8_t value)
{
  lsm6dso_ctrl2_g_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL2_G, (FAR uint8_t *)&reg, 1);

  reg.fs_g = value;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL2_G, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_gy_enable
 *
 * Description:
 *   Enable or disable sensor device.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - true(enable) and false(disable)
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int lsm6dso_gy_enable(FAR struct lsm6dso_dev_s *priv,
                             bool enable)
{
  if (enable)
    {
      /* Gyro config registers Turn on the gyro: FS=2000dps.
       * Not using modifyreg with empty value!!!! Then read value first!!!
       */

      lsm6dso_gy_setfullscale(priv, LSM6DSO_2000DPS);
      priv->dev[LSM6DSO_GY_IDX].factor = LSM6DSO_2000DPS_FACTOR
                                       * LSM6DSO_MDPS2DPS_FACTOR
                                       * LSM6DSO_DPS2RPS_FACTOR;

      /* Set interrupt for gyroscope. */

      if (priv->dev[LSM6DSO_GY_IDX].fifoen)
        {
          work_cancel(HPWORK, &priv->dev[LSM6DSO_GY_IDX].work);
        }
      else
        {
          work_queue(HPWORK, &priv->dev[LSM6DSO_GY_IDX].work,
                     lsm6dso_gy_worker, priv,
                     priv->dev[LSM6DSO_GY_IDX].interval / USEC_PER_TICK);
        }
    }
  else
    {
      /* Set to Shut Down */

      work_cancel(HPWORK, &priv->dev[LSM6DSO_GY_IDX].work);
      lsm6dso_gy_setodr(priv, LSM6DSO_GY_ODR_OFF);
    }

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_gy_isready
 *
 * Description:
 *   Read the gyroscope data flag.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Flag state.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_gy_isready(FAR struct lsm6dso_dev_s *priv,
                              FAR uint8_t *value)
{
  lsm6dso_status_reg_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_STATUS_REG, (FAR uint8_t *)&reg, 1);
  *value = reg.gda;

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_gy_getdata
 *
 * Description:
 *   Read the gyroscope data flag.
 *
 * Input Parameters:
 *   priv     -  Device struct.
 *   regaddr  -  Out put data start register address.
 *   value    -  Output data.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_gy_getdata(FAR struct lsm6dso_dev_s *priv,
                              uint8_t regaddr,
                              FAR struct sensor_gyro *value)
{
  axis3bit16_t temp;

  lsm6dso_spi_read(priv, regaddr, temp.u8bit, 6);
  sensor_remap_vector_raw16(temp.i16bit, temp.i16bit, LSM6DSO_VECTOR_REMAP);

  value->x = temp.i16bit[0] * priv->dev[LSM6DSO_GY_IDX].factor;
  value->y = temp.i16bit[1] * priv->dev[LSM6DSO_GY_IDX].factor;
  value->z = temp.i16bit[2] * priv->dev[LSM6DSO_GY_IDX].factor;

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_gy_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void lsm6dso_gy_worker(FAR void *arg)
{
  FAR struct lsm6dso_dev_s *priv = arg;
  struct sensor_gyro temp_gy;

  /* Sanity check. */

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp. */

  temp_gy.timestamp = sensor_get_timestamp();

  /* Set work queue. */

  work_queue(HPWORK, &priv->dev[LSM6DSO_GY_IDX].work, lsm6dso_gy_worker,
             priv, priv->dev[LSM6DSO_GY_IDX].interval / USEC_PER_TICK);

  /* Read out the latest sensor data. */

  lsm6dso_gy_getdata(priv, LSM6DSO_OUTX_L_G, &temp_gy);

  /* Read out the latest temperature data. */

  lsm6dso_temp_getdata(priv, LSM6DSO_OUT_TEMP_L, &temp_gy.temperature);

  /* push data to upper half driver. */

  priv->dev[LSM6DSO_GY_IDX].lower.push_event(
        priv->dev[LSM6DSO_GY_IDX].lower.priv,
        &temp_gy,
        sizeof(struct sensor_gyro));
}

/****************************************************************************
 * Name: lsm6dso_fifo_setmode
 *
 * Description:
 *   Set FIFO working mode.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   value  - FIFO mode.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fifo_setmode(FAR struct lsm6dso_dev_s *priv,
                                uint8_t value)
{
  lsm6dso_fifo_ctrl4_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_FIFO_CTRL4, (FAR uint8_t *)&reg, 1);

  reg.fifo_mode = value;
  lsm6dso_spi_write(priv, LSM6DSO_FIFO_CTRL4, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_fifo_setwatermark
 *
 * Description:
 *   Set FIFO watermark threshold.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - FIFO water mark(9bit).
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fifo_setwatermark(FAR struct lsm6dso_dev_s *priv,
                                     unsigned int value)
{
  lsm6dso_fifo_ctrl1_t fifo_ctrl1;
  lsm6dso_fifo_ctrl2_t fifo_ctrl2;

  lsm6dso_spi_read(priv, LSM6DSO_FIFO_CTRL2,
                  (FAR uint8_t *)&fifo_ctrl2, 1);

  fifo_ctrl1.wtm = 0x00ff & value;
  fifo_ctrl2.wtm = ((0x0100 & value) >> 8);
  lsm6dso_spi_write(priv, LSM6DSO_FIFO_CTRL1,
                   (FAR uint8_t *)&fifo_ctrl1);
  lsm6dso_spi_write(priv, LSM6DSO_FIFO_CTRL2,
                   (FAR uint8_t *)&fifo_ctrl2);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_fifo_xl_setbatch
 *
 * Description:
 *   Set FIFO batch accelerometer ODR.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Batch ODR.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fifo_xl_setbatch(FAR struct lsm6dso_dev_s *priv,
                                    uint8_t value)
{
  lsm6dso_fifo_ctrl3_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_FIFO_CTRL3, (FAR uint8_t *)&reg, 1);

  reg.bdr_xl = value;
  lsm6dso_spi_write(priv, LSM6DSO_FIFO_CTRL3, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_fifo_gy_setbatch
 *
 * Description:
 *   Set FIFO batch gyroscope ODR.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Batch ODR.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fifo_gy_setbatch(FAR struct lsm6dso_dev_s *priv,
                                    uint8_t value)
{
  lsm6dso_fifo_ctrl3_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_FIFO_CTRL3, (FAR uint8_t *)&reg, 1);

  reg.bdr_gy = value;
  lsm6dso_spi_write(priv, LSM6DSO_FIFO_CTRL3, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_fifo_findbdr
 *
 * Description:
 *   Find the best matching bdr for sensor.
 *
 * Input Parameters:
 *   sensor  - The instance of lower half sensor driver.
 *
 * Returned Value:
 *   Index of the best fit BDR or a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fifo_findbdr(FAR struct lsm6dso_sensor_s *sensor)
{
  int i;
  int num;
  float freq;

  if (sensor->lower.type == SENSOR_TYPE_ACCELEROMETER)
    {
      num = sizeof(g_lsm6dso_xl_bdr) / sizeof(struct lsm6dso_bdr_s);
      freq = LSM6DSO_UNIT_TIME / sensor->interval;
      for (i = 0; i < num; i++)
        {
          if (freq < g_lsm6dso_xl_bdr[i].bdr
             || freq == g_lsm6dso_xl_bdr[i].bdr)
            {
              freq = g_lsm6dso_xl_bdr[i].bdr;
              return i;
            }
        }

      return num - 1;
    }
  else if (sensor->lower.type == SENSOR_TYPE_GYROSCOPE)
    {
      num = sizeof(g_lsm6dso_gy_bdr) / sizeof(struct lsm6dso_bdr_s);
      freq = LSM6DSO_UNIT_TIME / sensor->interval;
      for (i = 0; i < num; i++)
        {
          if (freq < g_lsm6dso_gy_bdr[i].bdr
             || freq == g_lsm6dso_gy_bdr[i].bdr)
            {
              freq = g_lsm6dso_gy_bdr[i].bdr;
              return i;
            }
        }

      return num - 1;
    }
  else
    {
      snerr("Failed to match sensor type.\n");
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: lsm6dso_fifo_getlevel
 *
 * Description:
 *   Read number of samples in FIFO.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Number of samples in FIFO(9bit).
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fifo_getlevel(FAR struct lsm6dso_dev_s *priv,
                                 unsigned int *value)
{
  lsm6dso_fifo_status1_t fifo_status1;
  lsm6dso_fifo_status2_t fifo_status2;

  lsm6dso_spi_read(priv, LSM6DSO_FIFO_STATUS1,
                  (FAR uint8_t *)&fifo_status1, 1);
  lsm6dso_spi_read(priv, LSM6DSO_FIFO_STATUS2,
                  (FAR uint8_t *)&fifo_status2, 1);
  *value = ((uint16_t)fifo_status2.diff_fifo << 8) +
           (uint16_t)fifo_status1.diff_fifo;

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_fifo_gettag
 *
 * Description:
 *   Read the fifo data tag.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   value  - FIFO data tag.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fifo_gettag(FAR struct lsm6dso_dev_s *priv,
                               FAR uint8_t *value)
{
  lsm6dso_fifo_data_out_tag_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_FIFO_DATA_OUT_TAG, (FAR uint8_t *)&reg, 1);
  *value = reg.tag_sensor;

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_fifo_flushdata
 *
 * Description:
 *   Flush unused samples.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fifo_flushdata(FAR struct lsm6dso_dev_s *priv)
{
  axis3bit16_t temp;

  lsm6dso_spi_read(priv, LSM6DSO_FIFO_DATA_OUT_X_L, temp.u8bit, 6);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_fifo_readdata
 *
 * Description:
 *   Read all data in FIFO.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fifo_readdata(FAR struct lsm6dso_dev_s *priv)
{
  struct sensor_accel
         temp_xl[CONFIG_SENSORS_LSM6DSO_FIFO_SLOTS_NUMBER];
  struct sensor_gyro
         temp_gy[CONFIG_SENSORS_LSM6DSO_FIFO_SLOTS_NUMBER];
  unsigned int counter_xl = 0;
  unsigned int counter_gy = 0;
  unsigned int fifo_interval;
  unsigned int num;
  float temperature;
  int ret;
  int i;
  uint8_t reg_tag;

  memset(temp_xl, 0x00, sizeof(temp_xl));
  memset(temp_gy, 0x00, sizeof(temp_gy));

  ret = lsm6dso_fifo_getlevel(priv, &num);
  if (ret < 0)
    {
      snerr("Failed to get FIFO level!\n");
      return ret;
    }

  lsm6dso_temp_getdata(priv, LSM6DSO_OUT_TEMP_L, &temperature);

  for (; num > 0; num--)
    {
      /* Read FIFO tag */

      lsm6dso_fifo_gettag(priv, &reg_tag);
      switch (reg_tag)
        {
          case LSM6DSO_XL_NC_TAG:    /* Accelerator data tag */
            {
              lsm6dso_xl_getdata(priv,
                                 LSM6DSO_FIFO_DATA_OUT_X_L,
                                 &temp_xl[counter_xl]);
              temp_xl[counter_xl].temperature = temperature;
              counter_xl++;
            }
            break;

          case LSM6DSO_GYRO_NC_TAG:  /* Gyroscope data tag */
            {
              lsm6dso_gy_getdata(priv,
                                 LSM6DSO_FIFO_DATA_OUT_X_L,
                                 &temp_gy[counter_gy]);
              temp_gy[counter_gy].temperature = temperature;
              counter_gy++;
            }
            break;

          default:                   /* Other data tag */
            {
              lsm6dso_fifo_flushdata(priv);
            }

            break;
        }
    }

  if (counter_xl)
    {
      /* Inferred data timestamp. */

      fifo_interval = (priv->timestamp - priv->timestamp_fifolast)
                    / counter_xl;

      for (i = 0; i < counter_xl; i++)
        {
          temp_xl[i].timestamp
            = priv->timestamp
            - fifo_interval
            * (counter_xl - i - 1);
        }

      /* Push data to the upper layer. */

      priv->dev[LSM6DSO_XL_IDX].lower.push_event(
            priv->dev[LSM6DSO_XL_IDX].lower.priv,
            temp_xl,
            sizeof(struct sensor_accel) * counter_xl);
    }

  if (counter_gy)
    {
      /* Inferred data timestamp. */

      fifo_interval = (priv->timestamp - priv->timestamp_fifolast)
                    / counter_gy;

      for (i = 0; i < counter_gy; i++)
        {
          temp_gy[i].timestamp
            = priv->timestamp
            - fifo_interval
            * (counter_gy - i - 1);
        }

      /* Push data to the upper layer. */

      priv->dev[LSM6DSO_GY_IDX].lower.push_event(
            priv->dev[LSM6DSO_GY_IDX].lower.priv,
            temp_gy,
            sizeof(struct sensor_gyro) * counter_gy);
    }

  priv->timestamp_fifolast = priv->timestamp;

  return ret;
}

/* Finite state machine handle functions */

/****************************************************************************
 * Name: lsm6dso_fsm_enable
 *
 * Description:
 *   Start/stop finite state machine.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - fsm state.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fsm_enable(FAR struct lsm6dso_dev_s *priv,
                              bool enable)
{
  uint16_t fsm_addr;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Check the sensor is setting to active. */

  if (enable
      && priv->dev[LSM6DSO_XL_IDX].activated == false
      && priv->dev[LSM6DSO_GY_IDX].activated == false)
    {
      snerr("Sensor is not activated: %d\n", -ENXIO);
      return -ENXIO;
    }
  else if (!enable)
    {
      priv->fsmen = LSM6DSO_PROPERTY_DISABLE;
    }

  /* Set interrupt and finite state machine state. */

  ret = lsm6dso_fsm_manage(priv);
  if (ret < 0)
    {
      snerr("Failed to set finite state machine: %d\n", ret);
      return ret;
    }

  /* Disable finite state machine. */

  if (!enable)
    {
      return ret;
    }

  /* Set the first address where the programs are written. */

  lsm6dso_fsm_setstartaddr(priv, LSM6DSO_START_FSM_ADD);

  /* Set the number of the programs. */

  lsm6dso_fsm_setprogramnum(priv, LSM6DSO_FSM_PROGRAMNUM);

  /* Set finite state machine data rate. */

  lsm6dso_fsm_setodr(priv, LSM6DSO_FSM_ODR_26HZ);

  /* Write programs. */

  fsm_addr = LSM6DSO_START_FSM_ADD;

  /* Wrist tilt xl. */

  lsm6dso_pg_writeln(priv, fsm_addr,
                     (FAR uint8_t *)g_lsm6so_prg_wrist_tilt_xl,
                     sizeof(g_lsm6so_prg_wrist_tilt_xl));
  fsm_addr += sizeof(g_lsm6so_prg_wrist_tilt_xl);

  /* Wrist tilt left. */

  lsm6dso_pg_writeln(priv, fsm_addr,
                     (FAR uint8_t *)g_lsm6so_prg_wrist_tilt_left,
                     sizeof(g_lsm6so_prg_wrist_tilt_left));
  fsm_addr += sizeof(g_lsm6so_prg_wrist_tilt_left);

  /* Wrist tilt right. */

  lsm6dso_pg_writeln(priv, fsm_addr,
                     (FAR uint8_t *)g_lsm6so_prg_wrist_tilt_right,
                     sizeof(g_lsm6so_prg_wrist_tilt_right));

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_fsm_manage
 *
 * Description:
 *   Manage finite state machine. Control which finite state
 *   machines are started.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fsm_manage(FAR struct lsm6dso_dev_s *priv)
{
  lsm6dso_pin_int1_route_t pin_int1_route;
  lsm6dso_emb_fsm_enable_t fsm_enable;
  lsm6dso_emb_sens_t emb_sens;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Route signals on interrupt pin 1. */

  ret = lsm6dso_int1_getroute(priv, &pin_int1_route);
  if (ret < 0)
    {
      snerr("Failed to route signals on interrupt pin 1: %d\n", ret);
      return ret;
    }

  pin_int1_route.fsm1  = priv->fsmen & LSM6DSO_FSM_INDEX1;
  pin_int1_route.fsm2  = priv->fsmen & LSM6DSO_FSM_INDEX2;
  pin_int1_route.fsm3  = priv->fsmen & LSM6DSO_FSM_INDEX3;
  pin_int1_route.fsm4  = priv->fsmen & LSM6DSO_FSM_INDEX4;
  pin_int1_route.fsm5  = priv->fsmen & LSM6DSO_FSM_INDEX5;
  pin_int1_route.fsm6  = priv->fsmen & LSM6DSO_FSM_INDEX6;
  pin_int1_route.fsm7  = priv->fsmen & LSM6DSO_FSM_INDEX7;
  pin_int1_route.fsm8  = priv->fsmen & LSM6DSO_FSM_INDEX8;
  pin_int1_route.fsm9  = priv->fsmen & LSM6DSO_FSM_INDEX9;
  pin_int1_route.fsm10 = priv->fsmen & LSM6DSO_FSM_INDEX10;
  pin_int1_route.fsm11 = priv->fsmen & LSM6DSO_FSM_INDEX11;
  pin_int1_route.fsm12 = priv->fsmen & LSM6DSO_FSM_INDEX12;
  pin_int1_route.fsm13 = priv->fsmen & LSM6DSO_FSM_INDEX13;
  pin_int1_route.fsm14 = priv->fsmen & LSM6DSO_FSM_INDEX14;
  pin_int1_route.fsm15 = priv->fsmen & LSM6DSO_FSM_INDEX15;
  pin_int1_route.fsm16 = priv->fsmen & LSM6DSO_FSM_INDEX16;

  lsm6dso_int1_setroute(priv, pin_int1_route);

  /* Configure interrupt pin mode notification. */

  lsm6dso_int_setnotification(priv, LSM6DSO_BASE_PULSED_EMB_LATCHED);

  /* Enable final state machine */

  fsm_enable.fsm_enable_a.fsm1_en  = priv->fsmen & LSM6DSO_FSM_INDEX1;
  fsm_enable.fsm_enable_a.fsm2_en  = priv->fsmen & LSM6DSO_FSM_INDEX2;
  fsm_enable.fsm_enable_a.fsm3_en  = priv->fsmen & LSM6DSO_FSM_INDEX3;
  fsm_enable.fsm_enable_a.fsm4_en  = priv->fsmen & LSM6DSO_FSM_INDEX4;
  fsm_enable.fsm_enable_a.fsm5_en  = priv->fsmen & LSM6DSO_FSM_INDEX5;
  fsm_enable.fsm_enable_a.fsm6_en  = priv->fsmen & LSM6DSO_FSM_INDEX6;
  fsm_enable.fsm_enable_a.fsm7_en  = priv->fsmen & LSM6DSO_FSM_INDEX7;
  fsm_enable.fsm_enable_a.fsm8_en  = priv->fsmen & LSM6DSO_FSM_INDEX8;
  fsm_enable.fsm_enable_b.fsm9_en  = priv->fsmen & LSM6DSO_FSM_INDEX9;
  fsm_enable.fsm_enable_b.fsm10_en = priv->fsmen & LSM6DSO_FSM_INDEX10;
  fsm_enable.fsm_enable_b.fsm11_en = priv->fsmen & LSM6DSO_FSM_INDEX11;
  fsm_enable.fsm_enable_b.fsm12_en = priv->fsmen & LSM6DSO_FSM_INDEX12;
  fsm_enable.fsm_enable_b.fsm13_en = priv->fsmen & LSM6DSO_FSM_INDEX13;
  fsm_enable.fsm_enable_b.fsm14_en = priv->fsmen & LSM6DSO_FSM_INDEX14;
  fsm_enable.fsm_enable_b.fsm15_en = priv->fsmen & LSM6DSO_FSM_INDEX15;
  fsm_enable.fsm_enable_b.fsm16_en = priv->fsmen & LSM6DSO_FSM_INDEX16;

  lsm6dso_fsm_setenable(priv, &fsm_enable);

  /* Disable Finite State Machine */

  memset(&emb_sens, 0, sizeof(emb_sens));
  if (priv->fsmen == LSM6DSO_PROPERTY_DISABLE)
    {
      emb_sens.fsm = LSM6DSO_PROPERTY_DISABLE;
    }
  else
    {
      /* Reset Long Counter. */

      lsm6dso_int_setlongcnt(priv, LSM6DSO_LONGCNT_RESET);
      emb_sens.fsm = LSM6DSO_PROPERTY_ENABLE;
    }

  lsm6dso_embedded_setsens(priv, &emb_sens);

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_fsm_findodr
 *
 * Description:
 *   Find the best matching odr for FSM.
 *
 * Input Parameters:
 *   freq  - Desired frequency.
 *
 * Returned Value:
 *   Index of the best fit ODR.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fsm_findodr(FAR float *freq)
{
  int i;
  int num = sizeof(g_lsm6dso_fsm_odr) / sizeof(struct lsm6dso_odr_s);

  for (i = 0; i < num; i++)
    {
      if (((*freq < g_lsm6dso_fsm_odr[i].odr + LSM6DSO_ODR_FLT_EPSILON)
          && (*freq > g_lsm6dso_fsm_odr[i].odr - LSM6DSO_ODR_FLT_EPSILON))
          || *freq < g_lsm6dso_fsm_odr[i].odr)
        {
          *freq = g_lsm6dso_fsm_odr[i].odr;
          return i;
        }
    }

  return num - 1;
}

/****************************************************************************
 * Name: lsm6dso_fsm_setstartaddr
 *
 * Description:
 *   FSM start address register (r/w).
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   value   - The value of start address
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fsm_setstartaddr(FAR struct lsm6dso_dev_s *priv,
                                    uint16_t value)
{
  int ret;
  uint8_t buff[2];

  buff[1] = (uint8_t)(value / 256U);
  buff[0] = (uint8_t)(value - (buff[1] * 256U));
  ret = lsm6dso_pg_writelnbyte(priv, LSM6DSO_FSM_START_ADD_L, &buff[0]);

  if (ret == 0)
    {
      ret = lsm6dso_pg_writelnbyte(priv, LSM6DSO_FSM_START_ADD_H,
                                   &buff[1]);
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_fsm_setprogramnum
 *
 * Description:
 *   FSM number of programs register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   value   - Value to write.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fsm_setprogramnum(FAR struct lsm6dso_dev_s *priv,
                                     uint8_t value)
{
  return lsm6dso_pg_writelnbyte(priv, LSM6DSO_FSM_PROGRAMS, &value);
}

/****************************************************************************
 * Name: lsm6dso_fsm_setenable
 *
 * Description:
 *   Final State Machine enable.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   value   - Union of registers from FSM_ENABLE_A to FSM_ENABLE_B.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fsm_setenable(FAR struct lsm6dso_dev_s *priv,
                                 FAR lsm6dso_emb_fsm_enable_t *value)
{
  int ret;

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_EMBEDDEDFUNC_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
      return ret;
    }

  lsm6dso_spi_write(priv, LSM6DSO_FSM_ENABLE_A,
                    (FAR uint8_t *)&value->fsm_enable_a);
  lsm6dso_spi_write(priv, LSM6DSO_FSM_ENABLE_B,
                    (FAR uint8_t *)&value->fsm_enable_b);

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_USER_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_fsm_setodr
 *
 * Description:
 *   Finite State Machine ODR configuration.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   value   - Change the values of fsm_odr in reg EMB_FUNC_ODR_CFG_B.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fsm_setodr(FAR struct lsm6dso_dev_s *priv,
                              uint8_t value)
{
  lsm6dso_emb_func_odr_cfg_b_t reg;
  int ret;

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_EMBEDDEDFUNC_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
      return ret;
    }

  lsm6dso_spi_read(priv, LSM6DSO_EMB_FUNC_ODRCFG_B,
                   (FAR uint8_t *)&reg, 1);
  reg.not_used_01 = LSM6DSO_FSM_ODR_CFG_NU1;
  reg.not_used_02 = LSM6DSO_FSM_ODR_CFG_NU2;
  reg.fsm_odr = value;
  lsm6dso_spi_write(priv, LSM6DSO_EMB_FUNC_ODRCFG_B,
                    (FAR uint8_t *)&reg);

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_USER_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_fsm_handler
 *
 * Description:
 *   Finite state machine result handler.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   status  - All interrupt status.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fsm_handler(FAR struct lsm6dso_dev_s *priv,
                               lsm6dso_all_sources_t *status)
{
  struct sensor_wake_gesture temp_fsm;
  int ret = 0;

  temp_fsm.timestamp = priv->timestamp;

  if (status->fsm1)
    {
      temp_fsm.event = LSM6DSO_FSM_INDEX1;

      /* push data to upper half driver. */

      priv->dev[LSM6DSO_FSM_IDX].lower.push_event(
            priv->dev[LSM6DSO_FSM_IDX].lower.priv,
            &temp_fsm,
            sizeof(struct sensor_wake_gesture));
    }

  if (status->fsm2)
    {
      temp_fsm.event = LSM6DSO_FSM_INDEX2;

      /* push data to upper half driver. */

      priv->dev[LSM6DSO_FSM_IDX].lower.push_event(
            priv->dev[LSM6DSO_FSM_IDX].lower.priv,
            &temp_fsm,
            sizeof(struct sensor_wake_gesture));
    }

  if (status->fsm3)
    {
      temp_fsm.event = LSM6DSO_FSM_INDEX3;

      /* push data to upper half driver. */

      priv->dev[LSM6DSO_FSM_IDX].lower.push_event(
            priv->dev[LSM6DSO_FSM_IDX].lower.priv,
            &temp_fsm,
            sizeof(struct sensor_wake_gesture));
    }

  return ret;
}

/* Sensor ops functions */

/****************************************************************************
 * Name: lsm6dso_batch
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

static int lsm6dso_batch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR unsigned long *latency_us)
{
  FAR struct lsm6dso_sensor_s *sensor = (FAR struct lsm6dso_sensor_s *)lower;
  lsm6dso_pin_int1_route_t pin_int1_route;
  FAR struct lsm6dso_dev_s * priv;
  unsigned long max_latency;
  int idx;
  int ret;

  /* Sanity check. */

  DEBUGASSERT(sensor != NULL && latency_us != NULL);

  max_latency = sensor->lower.nbuffer * sensor->interval;
  if (*latency_us > max_latency)
    {
      *latency_us = max_latency;
    }
  else if (*latency_us < sensor->interval && *latency_us > 0)
    {
      *latency_us = sensor->interval;
    }

  sensor->fifowtm = LSM6DSO_ROUNDUP(*latency_us, sensor->interval)
                  / sensor->interval;
  *latency_us = sensor->fifowtm * sensor->interval;
  sensor->batch = *latency_us;

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_XL_IDX);
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_GY_IDX);
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  if (sensor->fifowtm > 1)
    {
      sensor->fifoen = true;
      idx = lsm6dso_fifo_findbdr(sensor);
      if (idx < 0)
        {
          snerr("Failed to match bdr.\n");
          return -EINVAL;
        }

      /* Get the fifo start timestamp. */

      priv->timestamp_fifolast = sensor_get_timestamp();

      if (lower->type == SENSOR_TYPE_ACCELEROMETER)
        {
          lsm6dso_fifo_xl_setbatch(priv, g_lsm6dso_xl_bdr[idx].regval);
          work_cancel(HPWORK, &priv->dev[LSM6DSO_XL_IDX].work);
        }
      else if (lower->type == SENSOR_TYPE_GYROSCOPE)
        {
          lsm6dso_fifo_gy_setbatch(priv, g_lsm6dso_gy_bdr[idx].regval);
          work_cancel(HPWORK, &priv->dev[LSM6DSO_GY_IDX].work);
        }
      else
        {
          snerr("Failed to match sensor type.\n");
          return -EINVAL;
        }

      IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                      IOEXPANDER_OPTION_INTCFG,
                      (FAR void *)IOEXPANDER_VAL_RISING);
    }
  else
    {
      sensor->fifoen = false;
    }

  /* Get all interrupt setting. */

  ret = lsm6dso_int1_getroute(priv, &pin_int1_route);
  if (ret < 0)
    {
      snerr("Failed to route signals on interrupt pin 1: %d\n", ret);
      return ret;
    }

  if (priv->dev[LSM6DSO_XL_IDX].fifoen == false
      && priv->dev[LSM6DSO_GY_IDX].fifoen == false)
    {
      /* Read the remaining FIFO data. */

      if (priv->fifoen)
        {
          lsm6dso_fifo_readdata(priv);
        }

      priv->fifoen = false;
      pin_int1_route.fifo_full = LSM6DSO_FIFO_INT_DISABLE;
      pin_int1_route.fifo_ovr = LSM6DSO_FIFO_INT_DISABLE;
      pin_int1_route.fifo_th = LSM6DSO_FIFO_INT_DISABLE;
      lsm6dso_fifo_setmode(priv, LSM6DSO_BYPASS_MODE);
    }
  else
    {
      priv->fifoen = true;
      pin_int1_route.fifo_full = LSM6DSO_FIFO_INT_ENABLE;
      pin_int1_route.fifo_ovr = LSM6DSO_FIFO_INT_ENABLE;
      pin_int1_route.fifo_th = LSM6DSO_FIFO_INT_ENABLE;

      if (priv->dev[LSM6DSO_XL_IDX].fifowtm
          > priv->dev[LSM6DSO_XL_IDX].lower.nbuffer)
        {
          priv->dev[LSM6DSO_XL_IDX].fifowtm
          = priv->dev[LSM6DSO_XL_IDX].lower.nbuffer;
        }

      if (priv->dev[LSM6DSO_GY_IDX].fifowtm
         > priv->dev[LSM6DSO_GY_IDX].lower.nbuffer)
        {
          priv->dev[LSM6DSO_GY_IDX].fifowtm
          = priv->dev[LSM6DSO_GY_IDX].lower.nbuffer;
        }

      priv->fifowtm = priv->dev[LSM6DSO_XL_IDX].fifowtm
                    + priv->dev[LSM6DSO_GY_IDX].fifowtm;

      lsm6dso_fifo_setwatermark(priv, priv->fifowtm);
      lsm6dso_fifo_setmode(priv, LSM6DSO_STREAM_MODE);
    }

  /* Set interrupt route. */

  lsm6dso_int1_setroute(priv, pin_int1_route);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_set_interval
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

static int lsm6dso_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *period_us)
{
  FAR struct lsm6dso_sensor_s *sensor = (FAR struct lsm6dso_sensor_s *)lower;
  FAR struct lsm6dso_dev_s * priv;
  float freq;
  int ret;
  int idx;

  /* Sanity check. */

  DEBUGASSERT(sensor != NULL && period_us != NULL);

  freq = LSM6DSO_UNIT_TIME / *period_us;

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_XL_IDX);

      /* Find the period that matches best.  */

      idx = lsm6dso_xl_findodr(&freq);
      ret = lsm6dso_xl_setodr(priv, g_lsm6dso_xl_odr[idx].regval);
      if (ret < 0)
        {
          snerr("Failed to set interval: %d\n", ret);
          return ret;
        }

      *period_us = LSM6DSO_UNIT_TIME / freq;
      priv->dev[LSM6DSO_XL_IDX].interval = *period_us;
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_GY_IDX);

      /* Find the period that matches best.  */

      idx = lsm6dso_gy_findodr(&freq);
      ret = lsm6dso_gy_setodr(priv, g_lsm6dso_gy_odr[idx].regval);
      if (ret < 0)
        {
          snerr("Failed to set interval: %d\n", ret);
          return ret;
        }

      *period_us = LSM6DSO_UNIT_TIME / freq;
      priv->dev[LSM6DSO_GY_IDX].interval = *period_us;
    }
  else if (lower->type == SENSOR_TYPE_WAKE_GESTURE)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_FSM_IDX);

      /* Find the period that matches best.  */

      idx = lsm6dso_fsm_findodr(&freq);
      ret = lsm6dso_fsm_setodr(priv, g_lsm6dso_fsm_odr[idx].regval);
      if (ret < 0)
        {
          snerr("Failed to set interval: %d\n", ret);
          return ret;
        }

      *period_us = LSM6DSO_UNIT_TIME / freq;
      priv->dev[LSM6DSO_FSM_IDX].interval = *period_us;
    }
  else
    {
      snerr("Failed to match sensor type: %d\n", -EINVAL);
      return -EINVAL;
    }

  if ((priv->dev[LSM6DSO_FSM_IDX].interval
      < priv->dev[LSM6DSO_XL_IDX].interval)
      && (priv->dev[LSM6DSO_FSM_IDX].interval
      < priv->dev[LSM6DSO_GY_IDX].interval))
    {
      ret = -EINVAL;
      snerr("FSM odr should not be larger than the sensor: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_activate
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   lower  - The instance of lower half sensor driver.
 *   filep  - The pointer of file, represents each user using the sensor.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int lsm6dso_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            bool enable)
{
  FAR struct lsm6dso_sensor_s *sensor = (FAR struct lsm6dso_sensor_s *)lower;
  FAR struct lsm6dso_dev_s * priv;
  int ret = OK;

  DEBUGASSERT(lower != NULL);

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_XL_IDX);
      if (sensor->activated != enable)
        {
          ret = lsm6dso_xl_enable(priv, enable);
          if (ret < 0)
            {
              snerr("Failed to enable light sensor: %d\n", ret);
              return ret;
            }

          sensor->activated = enable;
        }
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_GY_IDX);
      if (sensor->activated != enable)
        {
          ret = lsm6dso_gy_enable(priv, enable);
          if (ret < 0)
            {
              snerr("Failed to enable light sensor: %d\n", ret);
              return ret;
            }

          sensor->activated = enable;
        }
    }
  else if (lower->type == SENSOR_TYPE_WAKE_GESTURE)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_FSM_IDX);
      if (sensor->activated != enable)
        {
          if (enable)
            {
              IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                              IOEXPANDER_OPTION_INTCFG,
                              (FAR void *)IOEXPANDER_VAL_RISING);
            }

          ret = lsm6dso_fsm_enable(priv, enable);
          if (ret < 0)
            {
              snerr("Failed to enable light sensor: %d\n", ret);
              return ret;
            }

          sensor->activated = enable;
        }
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  if (priv->dev[LSM6DSO_FSM_IDX].activated == false
      && priv->dev[LSM6DSO_XL_IDX].fifoen == false
      && priv->dev[LSM6DSO_GY_IDX].fifoen == false)
    {
      IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                      IOEXPANDER_OPTION_INTCFG,
                      (FAR void *)IOEXPANDER_VAL_DISABLE);
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_selftest
 *
 * Selftest allows for the testing of the mechanical and electrical
 * portions of the sensors. When the selftest is activated, the
 * electronics cause the sensors to be actuated and produce an output
 * signal. The output signal is used to observe the selftest response.
 * When the selftest response exceeds the min/max values,
 * the part is deemed to have failed selftest.
 *
 * Input Parameters:
 *   filep      - The pointer of file, represents each user using the sensor.
 *   lower      - The instance of lower half sensor driver.
 *   arg        - The parameters associated with selftest.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int lsm6dso_selftest(FAR struct file *filep,
                            FAR struct sensor_lowerhalf_s *lower,
                            unsigned long arg)
{
  FAR struct lsm6dso_sensor_s *sensor = (FAR struct lsm6dso_sensor_s *)lower;
  FAR struct lsm6dso_dev_s * priv;
  int ret;

  DEBUGASSERT(lower != NULL);

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_XL_IDX);
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_GY_IDX);
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  /* Process ioctl commands. */

  switch (arg)
    {
      case LSM6DSO_SIMPLE_CHECK:    /* Simple check tag */
        {
          /* Read device ID. */

          ret = lsm6dso_readdevid(priv);
        }
        break;

      case LSM6DSO_FULL_CHECK:      /* Full check tag */
        {
          /* Run selftest. */

          ret = lsm6dso_datatest(priv, lower->type);
        }
        break;

      default:                      /* Other cmd tag */
        {
          ret = -ENOTTY;
          snerr("ERROR: The cmd don't support: %d\n", ret);
        }
        break;
    }

    return ret;
}

/****************************************************************************
 * Name: lsm6dso_control
 *
 * Description:
 *   With this method, the user can set some special config for the sensor,
 *   such as changing the custom mode, setting the custom resolution, reset,
 *   etc, which are all parsed and implemented by lower half driver.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   filep      - The pointer of file, represents each user using the sensor.
 *   cmd        - The special cmd for sensor.
 *   arg        - The parameters associated with cmd.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *   -ENOTTY    - The cmd don't support.
 *   -EINVAL    - Failed to match sensor type.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int lsm6dso_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg)
{
  FAR struct lsm6dso_sensor_s *sensor = (FAR struct lsm6dso_sensor_s *)lower;
  FAR struct lsm6dso_dev_s * priv;
  int ret;

  DEBUGASSERT(lower != NULL);

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_XL_IDX);
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_GY_IDX);
    }
  else if (lower->type == SENSOR_TYPE_WAKE_GESTURE)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_FSM_IDX);
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  /* Process ioctl commands. */

  switch (cmd)
    {
      case LSM6DSO_FSM_MANAGE_CMD:    /* Finite state machine cmd tag */
        {
          priv->fsmen = (unsigned int)arg;
          ret = lsm6dso_fsm_manage(priv);
          if (ret < 0)
            {
              snerr("ERROR: Failed to selftest: %d\n", ret);
            }
        }
        break;

      case LSM6DSO_SET_SCALE_XL_CMD:  /* Set accelerator scale command tag */
        {
          switch (arg)
            {
              case LSM6DSO_XL_SET_2G:
                {
                  ret = lsm6dso_xl_setfullscale(priv, LSM6DSO_2G);
                  priv->dev[LSM6DSO_XL_IDX].factor = LSM6DSO_2G_FACTOR
                                                   * LSM6DSO_MG2MS_FACTOR;
                }
                break;

              case LSM6DSO_XL_SET_4G:
                {
                  ret = lsm6dso_xl_setfullscale(priv, LSM6DSO_4G);
                  priv->dev[LSM6DSO_XL_IDX].factor = LSM6DSO_4G_FACTOR
                                                   * LSM6DSO_MG2MS_FACTOR;
                }
                break;

              case LSM6DSO_XL_SET_8G:
                {
                  ret = lsm6dso_xl_setfullscale(priv, LSM6DSO_8G);
                  priv->dev[LSM6DSO_XL_IDX].factor = LSM6DSO_8G_FACTOR
                                                   * LSM6DSO_MG2MS_FACTOR;
                }
                break;

              case LSM6DSO_XL_SET_16G:
                {
                  ret = lsm6dso_xl_setfullscale(priv, LSM6DSO_16G);
                  priv->dev[LSM6DSO_XL_IDX].factor = LSM6DSO_16G_FACTOR
                                                   * LSM6DSO_MG2MS_FACTOR;
                }
                break;

              default:
                {
                  ret = -EINVAL;
                }
                break;
            }

          if (ret < 0)
            {
              snerr("ERROR: Failed to set scale for accelerator: %d\n", ret);
            }
        }
        break;

      default:                        /* Other cmd tag */
        {
          ret = -ENOTTY;
        }
        break;
    }

    return ret;
}

/* Sensor interrupt functions */

/****************************************************************************
 * Name: lsm6dso_int1_getroute
 *
 * Description:
 *   Route interrupt signals on int1 pin.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - The signals that are routed on int1 pin.
 *   type  - Init type.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_int1_getroute(FAR struct lsm6dso_dev_s *priv,
                                 FAR lsm6dso_pin_int1_route_t *value)
{
  lsm6dso_emb_func_int1_t emb_func_int1;
  lsm6dso_fsm_int1_a_t fsm_int1_a;
  lsm6dso_fsm_int1_b_t fsm_int1_b;
  lsm6dso_int1_ctrl_t int1_ctrl;
  lsm6dso_int2_ctrl_t int2_ctrl;
  lsm6dso_md1_cfg_t md1_cfg;
  lsm6dso_md2_cfg_t md2_cfg;
  lsm6dso_ctrl4_c_t ctrl4_c;
  int ret;

  /* Get all interrupt status */

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_EMBEDDEDFUNC_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
      return ret;
    }

  lsm6dso_spi_read(priv, LSM6DSO_EMB_FUNC_INT1,
                   (FAR uint8_t *)&emb_func_int1, 1);
  lsm6dso_spi_read(priv, LSM6DSO_FSM_INT1_A,
                   (FAR uint8_t *)&fsm_int1_a, 1);
  lsm6dso_spi_read(priv, LSM6DSO_FSM_INT1_B,
                   (FAR uint8_t *)&fsm_int1_b, 1);

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_USER_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
      return ret;
    }

  lsm6dso_spi_read(priv, LSM6DSO_INT1_CTRL,
                   (FAR uint8_t *)&int1_ctrl, 1);
  lsm6dso_spi_read(priv, LSM6DSO_MD1_CFG,
                   (FAR uint8_t *)&md1_cfg, 1);
  lsm6dso_spi_read(priv, LSM6DSO_CTRL4_C,
                   (FAR uint8_t *)&ctrl4_c, 1);

  if (ctrl4_c.int2_on_int1 == LSM6DSO_PROPERTY_ENABLE)
    {
      lsm6dso_spi_read(priv, LSM6DSO_INT2_CTRL,
                       (FAR uint8_t *)&int2_ctrl, 1);
      value->drdy_temp = int2_ctrl.int2_drdy_temp;
      lsm6dso_spi_read(priv, LSM6DSO_MD2_CFG,
                       (FAR uint8_t *)&md2_cfg, 1);
      value->timestamp = md2_cfg.int2_timestamp;
    }
  else
    {
      value->drdy_temp = LSM6DSO_PROPERTY_DISABLE;
      value->timestamp = LSM6DSO_PROPERTY_DISABLE;
    }

  value->drdy_xl       = int1_ctrl.int1_drdy_xl;
  value->drdy_g        = int1_ctrl.int1_drdy_g;
  value->boot          = int1_ctrl.int1_boot;
  value->fifo_th       = int1_ctrl.int1_fifo_th;
  value->fifo_ovr      = int1_ctrl.int1_fifo_ovr;
  value->fifo_full     = int1_ctrl.int1_fifo_full;
  value->fifo_bdr      = int1_ctrl.int1_cnt_bdr;
  value->den_flag      = int1_ctrl.den_drdy_flag;
  value->sh_endop      = md1_cfg.int1_shub;
  value->six_d         = md1_cfg.int1_6d;
  value->double_tap    = md1_cfg.int1_double_tap;
  value->free_fall     = md1_cfg.int1_ff;
  value->wake_up       = md1_cfg.int1_wu;
  value->single_tap    = md1_cfg.int1_single_tap;
  value->sleep_change  = md1_cfg.int1_sleep_change;
  value->step_detector = emb_func_int1.int1_step_detector;
  value->tilt          = emb_func_int1.int1_tilt;
  value->sig_mot       = emb_func_int1.int1_sig_mot;
  value->fsm_lc        = emb_func_int1.int1_fsm_lc;
  value->fsm1          = fsm_int1_a.int1_fsm1;
  value->fsm2          = fsm_int1_a.int1_fsm2;
  value->fsm3          = fsm_int1_a.int1_fsm3;
  value->fsm4          = fsm_int1_a.int1_fsm4;
  value->fsm5          = fsm_int1_a.int1_fsm5;
  value->fsm6          = fsm_int1_a.int1_fsm6;
  value->fsm7          = fsm_int1_a.int1_fsm7;
  value->fsm8          = fsm_int1_a.int1_fsm8;
  value->fsm9          = fsm_int1_b.int1_fsm9;
  value->fsm10         = fsm_int1_b.int1_fsm10;
  value->fsm11         = fsm_int1_b.int1_fsm11;
  value->fsm12         = fsm_int1_b.int1_fsm12;
  value->fsm13         = fsm_int1_b.int1_fsm13;
  value->fsm14         = fsm_int1_b.int1_fsm14;
  value->fsm15         = fsm_int1_b.int1_fsm15;
  value->fsm16         = fsm_int1_b.int1_fsm16;

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_int1_setroute
 *
 * Description:
 *   Route interrupt signals on int1 pin.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - The signals to route on int1 pin.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_int1_setroute(FAR struct lsm6dso_dev_s *priv,
                                 lsm6dso_pin_int1_route_t value)
{
  lsm6dso_pin_int2_route_t pin_int2_route;
  lsm6dso_emb_func_int1_t emb_func_int1;
  lsm6dso_fsm_int1_a_t fsm_int1_a;
  lsm6dso_fsm_int1_b_t fsm_int1_b;
  lsm6dso_int1_ctrl_t int1_ctrl;
  lsm6dso_int2_ctrl_t int2_ctrl;
  lsm6dso_tap_cfg2_t tap_cfg2;
  lsm6dso_md2_cfg_t md2_cfg;
  lsm6dso_md1_cfg_t md1_cfg;
  lsm6dso_ctrl4_c_t ctrl4_c;
  int ret;

  int1_ctrl.int1_drdy_xl           = value.drdy_xl;
  int1_ctrl.int1_drdy_g            = value.drdy_g;
  int1_ctrl.int1_boot              = value.boot;
  int1_ctrl.int1_fifo_th           = value.fifo_th;
  int1_ctrl.int1_fifo_ovr          = value.fifo_ovr;
  int1_ctrl.int1_fifo_full         = value.fifo_full;
  int1_ctrl.int1_cnt_bdr           = value.fifo_bdr;
  int1_ctrl.den_drdy_flag          = value.den_flag;
  md1_cfg.int1_shub                = value.sh_endop;
  md1_cfg.int1_6d                  = value.six_d;
  md1_cfg.int1_double_tap          = value.double_tap;
  md1_cfg.int1_ff                  = value.free_fall;
  md1_cfg.int1_wu                  = value.wake_up;
  md1_cfg.int1_single_tap          = value.single_tap;
  md1_cfg.int1_sleep_change        = value.sleep_change;
  emb_func_int1.not_used_01        = 0;
  emb_func_int1.int1_step_detector = value.step_detector;
  emb_func_int1.int1_tilt          = value.tilt;
  emb_func_int1.int1_sig_mot       = value.sig_mot;
  emb_func_int1.not_used_02        = 0;
  emb_func_int1.int1_fsm_lc        = value.fsm_lc;
  fsm_int1_a.int1_fsm1             = value.fsm1;
  fsm_int1_a.int1_fsm2             = value.fsm2;
  fsm_int1_a.int1_fsm3             = value.fsm3;
  fsm_int1_a.int1_fsm4             = value.fsm4;
  fsm_int1_a.int1_fsm5             = value.fsm5;
  fsm_int1_a.int1_fsm6             = value.fsm6;
  fsm_int1_a.int1_fsm7             = value.fsm7;
  fsm_int1_a.int1_fsm8             = value.fsm8;
  fsm_int1_b.int1_fsm9             = value.fsm9;
  fsm_int1_b.int1_fsm10            = value.fsm10;
  fsm_int1_b.int1_fsm11            = value.fsm11;
  fsm_int1_b.int1_fsm12            = value.fsm12;
  fsm_int1_b.int1_fsm13            = value.fsm13;
  fsm_int1_b.int1_fsm14            = value.fsm14;
  fsm_int1_b.int1_fsm15            = value.fsm15;
  fsm_int1_b.int1_fsm16            = value.fsm16;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL4_C, (FAR uint8_t *)&ctrl4_c, 1);

  if ((value.drdy_temp | value.timestamp) != LSM6DSO_PROPERTY_DISABLE)
    {
      ctrl4_c.int2_on_int1 = LSM6DSO_PROPERTY_ENABLE;
    }
  else
    {
      ctrl4_c.int2_on_int1 = LSM6DSO_PROPERTY_DISABLE;
    }

  lsm6dso_spi_write(priv, LSM6DSO_CTRL4_C, (FAR uint8_t *)&ctrl4_c);

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_EMBEDDEDFUNC_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
      return ret;
    }

  lsm6dso_spi_write(priv, LSM6DSO_EMB_FUNC_INT1,
                    (FAR uint8_t *)&emb_func_int1);
  lsm6dso_spi_write(priv, LSM6DSO_FSM_INT1_A, (FAR uint8_t *)&fsm_int1_a);
  lsm6dso_spi_write(priv, LSM6DSO_FSM_INT1_B, (FAR uint8_t *)&fsm_int1_b);

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_USER_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
      return ret;
    }

  if ((emb_func_int1.int1_fsm_lc
      | emb_func_int1.int1_sig_mot
      | emb_func_int1.int1_step_detector
      | emb_func_int1.int1_tilt
      | fsm_int1_a.int1_fsm1
      | fsm_int1_a.int1_fsm2
      | fsm_int1_a.int1_fsm3
      | fsm_int1_a.int1_fsm4
      | fsm_int1_a.int1_fsm5
      | fsm_int1_a.int1_fsm6
      | fsm_int1_a.int1_fsm7
      | fsm_int1_a.int1_fsm8
      | fsm_int1_b.int1_fsm9
      | fsm_int1_b.int1_fsm10
      | fsm_int1_b.int1_fsm11
      | fsm_int1_b.int1_fsm12
      | fsm_int1_b.int1_fsm13
      | fsm_int1_b.int1_fsm14
      | fsm_int1_b.int1_fsm15
      | fsm_int1_b.int1_fsm16) != LSM6DSO_PROPERTY_DISABLE)
    {
      md1_cfg.int1_emb_func = LSM6DSO_PROPERTY_ENABLE;
    }
  else
    {
      md1_cfg.int1_emb_func = LSM6DSO_PROPERTY_DISABLE;
    }

  lsm6dso_spi_write(priv, LSM6DSO_INT1_CTRL, (FAR uint8_t *)&int1_ctrl);
  lsm6dso_spi_write(priv, LSM6DSO_MD1_CFG, (FAR uint8_t *)&md1_cfg);

  lsm6dso_spi_read(priv, LSM6DSO_INT2_CTRL, (FAR uint8_t *)&int2_ctrl, 1);
  int2_ctrl.int2_drdy_temp = value.drdy_temp;
  lsm6dso_spi_write(priv, LSM6DSO_INT2_CTRL, (FAR uint8_t *)&int2_ctrl);

  lsm6dso_spi_read(priv, LSM6DSO_MD2_CFG, (FAR uint8_t *)&md2_cfg, 1);
  md2_cfg.int2_timestamp = value.timestamp;
  lsm6dso_spi_write(priv, LSM6DSO_MD2_CFG, (FAR uint8_t *)&md2_cfg);

  lsm6dso_spi_read(priv, LSM6DSO_TAP_CFG2, (FAR uint8_t *)&tap_cfg2, 1);

  ret = lsm6dso_int2_getroute(priv, &pin_int2_route);
  if (ret < 0)
    {
      snerr("Failed to get int2 route.\n");
      return ret;
    }

  if ((pin_int2_route.fifo_bdr
      | pin_int2_route.drdy_g
      | pin_int2_route.drdy_temp
      | pin_int2_route.drdy_xl
      | pin_int2_route.fifo_full
      | pin_int2_route.fifo_ovr
      | pin_int2_route.fifo_th
      | pin_int2_route.six_d
      | pin_int2_route.double_tap
      | pin_int2_route.free_fall
      | pin_int2_route.wake_up
      | pin_int2_route.single_tap
      | pin_int2_route.sleep_change
      | int1_ctrl.den_drdy_flag
      | int1_ctrl.int1_boot
      | int1_ctrl.int1_cnt_bdr
      | int1_ctrl.int1_drdy_g
      | int1_ctrl.int1_drdy_xl
      | int1_ctrl.int1_fifo_full
      | int1_ctrl.int1_fifo_ovr
      | int1_ctrl.int1_fifo_th
      | md1_cfg.int1_shub
      | md1_cfg.int1_6d
      | md1_cfg.int1_double_tap
      | md1_cfg.int1_ff
      | md1_cfg.int1_wu
      | md1_cfg.int1_single_tap
      | md1_cfg.int1_sleep_change) != LSM6DSO_PROPERTY_DISABLE)
    {
      tap_cfg2.interrupts_enable = LSM6DSO_PROPERTY_ENABLE;
    }
  else
    {
      tap_cfg2.interrupts_enable = LSM6DSO_PROPERTY_DISABLE;
    }

  lsm6dso_spi_write(priv, LSM6DSO_TAP_CFG2, (FAR uint8_t *)&tap_cfg2);

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_int2_getroute
 *
 * Description:
 *   Route interrupt signals on int2 pin.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - The signals that are routed on int2 pin.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_int2_getroute(FAR struct lsm6dso_dev_s *priv,
                                 FAR lsm6dso_pin_int2_route_t *value)
{
  lsm6dso_emb_func_int2_t emb_func_int2;
  lsm6dso_fsm_int2_a_t fsm_int2_a;
  lsm6dso_fsm_int2_b_t fsm_int2_b;
  lsm6dso_int2_ctrl_t int2_ctrl;
  lsm6dso_md2_cfg_t md2_cfg;
  lsm6dso_ctrl4_c_t ctrl4_c;
  lsm6dso_int_ois_t int_ois;
  int ret;

  lsm6dso_spi_read(priv, LSM6DSO_INT_OIS, (FAR uint8_t *)&int_ois, 1);
  value->drdy_ois = int_ois.int2_drdy_ois;

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_EMBEDDEDFUNC_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
      return ret;
    }

  lsm6dso_spi_read(priv, LSM6DSO_EMB_FUNC_INT2,
                   (FAR uint8_t *)&emb_func_int2, 1);
  lsm6dso_spi_read(priv, LSM6DSO_FSM_INT2_A, (FAR uint8_t *)&fsm_int2_a, 1);
  lsm6dso_spi_read(priv, LSM6DSO_FSM_INT2_B, (FAR uint8_t *)&fsm_int2_b, 1);

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_USER_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
      return ret;
    }

  lsm6dso_spi_read(priv, LSM6DSO_INT2_CTRL, (FAR uint8_t *)&int2_ctrl, 1);
  lsm6dso_spi_read(priv, LSM6DSO_MD2_CFG, (FAR uint8_t *)&md2_cfg, 1);
  lsm6dso_spi_read(priv, LSM6DSO_CTRL4_C, (FAR uint8_t *)&ctrl4_c, 1);

  if (ctrl4_c.int2_on_int1 == LSM6DSO_PROPERTY_DISABLE)
    {
      lsm6dso_spi_read(priv, LSM6DSO_INT2_CTRL,
                       (FAR uint8_t *)&int2_ctrl, 1);
      value->drdy_temp = int2_ctrl.int2_drdy_temp;

      lsm6dso_spi_read(priv, LSM6DSO_MD2_CFG,
                       (FAR uint8_t *)&md2_cfg, 1);
      value->timestamp = md2_cfg.int2_timestamp;
    }
  else
    {
      value->drdy_temp = LSM6DSO_PROPERTY_DISABLE;
      value->timestamp = LSM6DSO_PROPERTY_DISABLE;
    }

  value->drdy_xl       = int2_ctrl.int2_drdy_xl;
  value->drdy_g        = int2_ctrl.int2_drdy_g;
  value->drdy_temp     = int2_ctrl.int2_drdy_temp;
  value->fifo_th       = int2_ctrl.int2_fifo_th;
  value->fifo_ovr      = int2_ctrl.int2_fifo_ovr;
  value->fifo_full     = int2_ctrl.int2_fifo_full;
  value->fifo_bdr      = int2_ctrl.int2_cnt_bdr;
  value->timestamp     = md2_cfg.int2_timestamp;
  value->six_d         = md2_cfg.int2_6d;
  value->double_tap    = md2_cfg.int2_double_tap;
  value->free_fall     = md2_cfg.int2_ff;
  value->wake_up       = md2_cfg.int2_wu;
  value->single_tap    = md2_cfg.int2_single_tap;
  value->sleep_change  = md2_cfg.int2_sleep_change;
  value->step_detector = emb_func_int2. int2_step_detector;
  value->tilt          = emb_func_int2.int2_tilt;
  value->fsm_lc        = emb_func_int2.int2_fsm_lc;
  value->fsm1          = fsm_int2_a.int2_fsm1;
  value->fsm2          = fsm_int2_a.int2_fsm2;
  value->fsm3          = fsm_int2_a.int2_fsm3;
  value->fsm4          = fsm_int2_a.int2_fsm4;
  value->fsm5          = fsm_int2_a.int2_fsm5;
  value->fsm6          = fsm_int2_a.int2_fsm6;
  value->fsm7          = fsm_int2_a.int2_fsm7;
  value->fsm8          = fsm_int2_a.int2_fsm8;
  value->fsm9          = fsm_int2_b.int2_fsm9;
  value->fsm10         = fsm_int2_b.int2_fsm10;
  value->fsm11         = fsm_int2_b.int2_fsm11;
  value->fsm12         = fsm_int2_b.int2_fsm12;
  value->fsm13         = fsm_int2_b.int2_fsm13;
  value->fsm14         = fsm_int2_b.int2_fsm14;
  value->fsm15         = fsm_int2_b.int2_fsm15;
  value->fsm16         = fsm_int2_b.int2_fsm16;

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_int_setnotification
 *
 * Description:
 *   Interrupt notification mode.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Int notification value.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_int_setnotification(FAR struct lsm6dso_dev_s *priv,
                                       lsm6dso_lir_t value)
{
  lsm6dso_tap_cfg0_t tap_cfg0;
  lsm6dso_page_rw_t page_rw;
  int ret;

  lsm6dso_spi_read(priv, LSM6DSO_TAP_CFG0, (FAR uint8_t *)&tap_cfg0, 1);
  tap_cfg0.lir = (uint8_t)value & 0x01u;
  tap_cfg0.int_clr_on_read = (uint8_t)value & 0x01u;
  lsm6dso_spi_write(priv, LSM6DSO_TAP_CFG0, (FAR uint8_t *)&tap_cfg0);

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_EMBEDDEDFUNC_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
      return ret;
    }

  lsm6dso_spi_read(priv, LSM6DSO_PAGE_RW, (FAR uint8_t *)&page_rw, 1);
  page_rw.emb_func_lir = ((uint8_t)value & 0x02u) >> 1;
  lsm6dso_spi_write(priv, LSM6DSO_PAGE_RW, (FAR uint8_t *)&page_rw);

  ret = lsm6dso_mem_setbank(priv, LSM6DSO_USER_BANK);
  if (ret < 0)
    {
      snerr("Failed to get access to the embedded functions/sensorhub: %d\n",
      ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_int_setlongcnt
 *
 * Description:
 *   FSM long counter timeout register (r/w). The long counter
 *   timeout value is an unsigned integer value (16-bit format).
 *   When the long counter value reached this value,
 *   the FSM generates an interrupt.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - The value of long counter.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_int_setlongcnt(FAR struct lsm6dso_dev_s *priv,
                                  uint16_t value)
{
  int ret;
  uint8_t buff[2];

  buff[1] = (uint8_t)(value / 256U);
  buff[0] = (uint8_t)(value - (buff[1] * 256U));
  ret = lsm6dso_pg_writelnbyte(priv, LSM6DSO_FSM_LC_TIMEOUT_L, &buff[0]);
  if (ret == 0)
    {
      ret = lsm6dso_pg_writelnbyte(priv, LSM6DSO_FSM_LC_TIMEOUT_H, &buff[1]);
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dso_int_getall
 *
 * Description:
 *   Get the status of all the interrupt sources.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   value   - The status of all the interrupt sources.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_int_getall(FAR struct lsm6dso_dev_s *priv,
                              FAR lsm6dso_all_sources_t *value)
{
  lsm6dso_emb_func_mpstatus_t emb_func_status_mainpage;
  lsm6dso_status_mpmaster_t status_master_mainpage;
  lsm6dso_fsm_mpstatus_a_t fsm_status_a_mainpage;
  lsm6dso_fsm_mpstatus_b_t fsm_status_b_mainpage;
  lsm6dso_fifo_status1_t fifo_status1;
  lsm6dso_fifo_status2_t fifo_status2;
  lsm6dso_all_int_src_t all_int_src;
  lsm6dso_wake_up_src_t wake_up_src;
  lsm6dso_status_reg_t status_reg;
  lsm6dso_tap_src_t tap_src;
  lsm6dso_d6d_src_t d6d_src;
  uint8_t reg[5];

  lsm6dso_spi_read(priv, LSM6DSO_ALL_INT_SRC, (FAR uint8_t *)&reg, 5);
  lsm6dso_copybyte((uint8_t *)&all_int_src, &reg[0]);
  lsm6dso_copybyte((uint8_t *)&wake_up_src, &reg[1]);
  lsm6dso_copybyte((uint8_t *)&tap_src, &reg[2]);
  lsm6dso_copybyte((uint8_t *)&d6d_src, &reg[3]);
  lsm6dso_copybyte((uint8_t *)&status_reg, &reg[4]);
  value->timestamp        = all_int_src.timestamp_endcount;
  value->wake_up_z        = wake_up_src.z_wu;
  value->wake_up_y        = wake_up_src.y_wu;
  value->wake_up_x        = wake_up_src.x_wu;
  value->wake_up          = wake_up_src.wu_ia;
  value->sleep_state      = wake_up_src.sleep_state;
  value->free_fall        = wake_up_src.ff_ia;
  value->sleep_change     = wake_up_src.sleep_change_ia;
  value->tap_x            = tap_src.x_tap;
  value->tap_y            = tap_src.y_tap;
  value->tap_z            = tap_src.z_tap;
  value->tap_sign         = tap_src.tap_sign;
  value->double_tap       = tap_src.double_tap;
  value->single_tap       = tap_src.single_tap;
  value->six_d_xl         = d6d_src.xl;
  value->six_d_xh         = d6d_src.xh;
  value->six_d_yl         = d6d_src.yl;
  value->six_d_yh         = d6d_src.yh;
  value->six_d_zl         = d6d_src.zl;
  value->six_d_zh         = d6d_src.zh;
  value->six_d            = d6d_src.d6d_ia;
  value->den_flag         = d6d_src.den_drdy;
  value->drdy_xl          = status_reg.xlda;
  value->drdy_g           = status_reg.gda;
  value->drdy_temp        = status_reg.tda;

  lsm6dso_spi_read(priv, LSM6DSO_EMB_FUNC_MPSTATUS, (FAR uint8_t *)&reg, 3);
  lsm6dso_copybyte((uint8_t *)&emb_func_status_mainpage, &reg[0]);
  lsm6dso_copybyte((uint8_t *)&fsm_status_a_mainpage, &reg[1]);
  lsm6dso_copybyte((uint8_t *)&fsm_status_b_mainpage, &reg[2]);
  value->step_detector    = emb_func_status_mainpage.is_step_det;
  value->tilt             = emb_func_status_mainpage.is_tilt;
  value->sig_mot          = emb_func_status_mainpage.is_sigmot;
  value->fsm_lc           = emb_func_status_mainpage.is_fsm_lc;
  value->fsm1             = fsm_status_a_mainpage.is_fsm1;
  value->fsm2             = fsm_status_a_mainpage.is_fsm2;
  value->fsm3             = fsm_status_a_mainpage.is_fsm3;
  value->fsm4             = fsm_status_a_mainpage.is_fsm4;
  value->fsm5             = fsm_status_a_mainpage.is_fsm5;
  value->fsm6             = fsm_status_a_mainpage.is_fsm6;
  value->fsm7             = fsm_status_a_mainpage.is_fsm7;
  value->fsm8             = fsm_status_a_mainpage.is_fsm8;
  value->fsm9             = fsm_status_b_mainpage.is_fsm9;
  value->fsm10            = fsm_status_b_mainpage.is_fsm10;
  value->fsm11            = fsm_status_b_mainpage.is_fsm11;
  value->fsm12            = fsm_status_b_mainpage.is_fsm12;
  value->fsm13            = fsm_status_b_mainpage.is_fsm13;
  value->fsm14            = fsm_status_b_mainpage.is_fsm14;
  value->fsm15            = fsm_status_b_mainpage.is_fsm15;
  value->fsm16            = fsm_status_b_mainpage.is_fsm16;

  lsm6dso_spi_read(priv, LSM6DSO_STATUS_MPMASTER, (FAR uint8_t *)&reg, 3);
  lsm6dso_copybyte((uint8_t *)&status_master_mainpage, &reg[0]);
  lsm6dso_copybyte((uint8_t *)&fifo_status1, &reg[1]);
  lsm6dso_copybyte((uint8_t *)&fifo_status2, &reg[2]);
  value->sh_endop         = status_master_mainpage.sens_hub_endop;
  value->sh_slave0_nack   = status_master_mainpage.slave0_nack;
  value->sh_slave1_nack   = status_master_mainpage.slave1_nack;
  value->sh_slave2_nack   = status_master_mainpage.slave2_nack;
  value->sh_slave3_nack   = status_master_mainpage.slave3_nack;
  value->sh_wr_once       = status_master_mainpage.wr_once_done;
  value->fifo_diff        = (256U * fifo_status2.diff_fifo)
                          + fifo_status1.diff_fifo;
  value->fifo_ovr_latched = fifo_status2.over_run_latched;
  value->fifo_bdr         = fifo_status2.counter_bdr_ia;
  value->fifo_full        = fifo_status2.fifo_full_ia;
  value->fifo_ovr         = fifo_status2.fifo_ovr_ia;
  value->fifo_th          = fifo_status2.fifo_wtm_ia;

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_interrupt_handler
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
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int lsm6dso_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the LSM6DSO
   * new data interrupt pin since it signals that new data has
   * been measured.
   */

  FAR struct lsm6dso_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp. */

  priv->timestamp = sensor_get_timestamp();

  /* Task the worker with retrieving the latest sensor data. We should not
   * do this in a interrupt since it might take too long. Also we cannot lock
   * the SPI bus from within an interrupt.
   */

  work_queue(LPWORK, &priv->work, lsm6dso_worker, priv, 0);
  IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void lsm6dso_worker(FAR void *arg)
{
  FAR struct lsm6dso_dev_s *priv = arg;
  lsm6dso_all_sources_t status;
  int ret;

  /* Sanity check. */

  DEBUGASSERT(priv != NULL);

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_RISING);

  /* Get all interrupt flag. */

  ret = lsm6dso_int_getall(priv, &status);
  if (ret < 0)
    {
      snerr("Failed to get interrupt source registers: %d\n", ret);
      return;
    }

  /* Get sensor FIFO data. */

  if (status.fifo_th || status.fifo_full || status.fifo_ovr)
    {
      if (status.fifo_full)
        {
          snwarn("FIFO is full.\n");
        }

      if (status.fifo_ovr)
        {
          snwarn("FIFO is overflow.\n");
        }

      lsm6dso_fifo_readdata(priv);
    }

  /* Get finite state machine result. */

  lsm6dso_fsm_handler(priv, &status);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dso_register
 *
 * Description:
 *   Register the LSM6DSO character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   i2c - An instance of the I2C interface to use to communicate with
 *         LSM6DSO
 *   addr - The I2C address of the LSM6DSO.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int lsm6dso_register(int devno, FAR const struct lsm6dso_config_s *config)
{
  FAR struct lsm6dso_dev_s *priv;
  FAR void *ioephandle;
  int ret;

  /* Sanity check. */

  DEBUGASSERT(config != NULL);

  /* Initialize the LSM6DSO device structure. */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config = config;

  priv->dev[LSM6DSO_XL_IDX].lower.ops = &g_lsm6dso_xl_ops;
  priv->dev[LSM6DSO_XL_IDX].lower.type = SENSOR_TYPE_ACCELEROMETER;
  priv->dev[LSM6DSO_XL_IDX].lower.uncalibrated = true;
  priv->dev[LSM6DSO_XL_IDX].interval = LSM6DSO_DEFAULT_INTERVAL;
  priv->dev[LSM6DSO_XL_IDX].lower.nbuffer
                            = CONFIG_SENSORS_LSM6DSO_FIFO_SLOTS_NUMBER;

  priv->dev[LSM6DSO_GY_IDX].lower.ops = &g_lsm6dso_gy_ops;
  priv->dev[LSM6DSO_GY_IDX].lower.type = SENSOR_TYPE_GYROSCOPE;
  priv->dev[LSM6DSO_GY_IDX].lower.uncalibrated = true;
  priv->dev[LSM6DSO_GY_IDX].interval = LSM6DSO_DEFAULT_INTERVAL;
  priv->dev[LSM6DSO_GY_IDX].lower.nbuffer
                            = CONFIG_SENSORS_LSM6DSO_FIFO_SLOTS_NUMBER;

  priv->fsmen = LSM6DSO_DEFAULT_FSM_EN;
  priv->dev[LSM6DSO_FSM_IDX].lower.ops = &g_lsm6dso_fsm_ops;
  priv->dev[LSM6DSO_FSM_IDX].lower.type = SENSOR_TYPE_WAKE_GESTURE;
  priv->dev[LSM6DSO_FSM_IDX].lower.uncalibrated = true;
  priv->dev[LSM6DSO_FSM_IDX].interval = LSM6DSO_DEFAULT_INTERVAL;
  priv->dev[LSM6DSO_FSM_IDX].lower.nbuffer
                            = CONFIG_SENSORS_LSM6DSO_FIFO_SLOTS_NUMBER;

  /* Wait sensor boot time. */

  up_mdelay(LSM6DSO_WAITBOOT_TIME);

  /* Read the deviceID. */

  ret = lsm6dso_readdevid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to get DeviceID: %d\n", ret);
      goto err;
    }

  /* Reset devices. */

  lsm6dso_reset(priv);
  lsm6dso_resetwait(priv);

  /* Disable I3C interface. */

  lsm6dso_seti3c(priv, LSM6DSO_I3C_DISABLE);

  /* Enable Block Data Update. */

  lsm6dso_setupdate(priv, LSM6DSO_BDU_UNTILREAD);

  /* Interrupt register. */

  ret = IOEXP_SETDIRECTION(priv->config->ioedev, priv->config->pin,
                           IOEXPANDER_DIRECTION_IN_PULLDOWN);
  if (ret < 0)
    {
      snerr("Failed to set direction: %d\n", ret);
      goto err;
    }

  ioephandle = IOEP_ATTACH(priv->config->ioedev, priv->config->pin,
                           lsm6dso_interrupt_handler, priv);
  if (ioephandle == NULL)
    {
      ret = -EIO;
      snerr("Failed to attach: %d\n", ret);
      goto err;
    }

  ret = IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                        IOEXPANDER_OPTION_INTCFG,
                        (FAR void *)IOEXPANDER_VAL_DISABLE);
  if (ret < 0)
    {
      snerr("Failed to set option: %d\n", ret);
      goto err_detach;
    }

  /* Register the character driver. */

  ret = sensor_register((&(priv->dev[LSM6DSO_XL_IDX].lower)), devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto err_detach;
    }

  ret = sensor_register((&(priv->dev[LSM6DSO_GY_IDX].lower)), devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto err_detach;
    }

  ret = sensor_register((&(priv->dev[LSM6DSO_FSM_IDX].lower)), devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto err_detach;
    }

  return ret;

err_detach:
  IOEP_DETACH(priv->config->ioedev, lsm6dso_interrupt_handler);

err:
  kmm_free(priv);
  return ret;
}
