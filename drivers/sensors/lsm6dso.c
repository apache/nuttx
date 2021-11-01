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

/* Multi sensor index */

#define LSM6DSO_XL_IDX            0          /* Accelerator index */
#define LSM6DSO_GY_IDX            1          /* Gyroscope index */
#define LSM6DSO_IDX_NUM           2          /* Max index */

/* Self-test mode selection */

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

#define LSM6DSO_2G_FACTOR         0.061f     /* Accelerator 2g factor (mg) */
#define LSM6DSO_4G_FACTOR         0.122f     /* Accelerator 4g factor (mg) */
#define LSM6DSO_8G_FACTOR         0.244f     /* Accelerator 8g factor (mg) */
#define LSM6DSO_MG2G_FACTOR       0.001f     /* Convert mg to g factor */

#define LSM6DSO_125DPS            0x01       /* Gyroscope scale 125dpg */
#define LSM6DSO_250DPS            0x00       /* Gyroscope scale 250dpg */
#define LSM6DSO_500DPS            0x02       /* Gyroscope scale 500dpg */
#define LSM6DSO_1000DPS           0x04       /* Gyroscope scale 1000dpg */
#define LSM6DSO_2000DPS           0x06       /* Gyroscope scale 2000dpg */

#define LSM6DSO_125DPS_FACTOR     4.375f     /* Gyroscope 125dpg factor (mdps/LSB) */
#define LSM6DSO_250DPS_FACTOR     8.75f      /* Gyroscope 250dpg factor (mdps/LSB) */
#define LSM6DSO_500DPS_FACTOR     17.5f      /* Gyroscope 500dpg factor (mdps/LSB) */
#define LSM6DSO_1000DPS_FACTOR    35.0f      /* Gyroscope 1000dpg factor (mdps/LSB) */
#define LSM6DSO_2000DPS_FACTOR    70.0f      /* Gyroscope 2000dpg factor (mdps/LSB) */
#define LSM6DSO_MDPS2DPS_FACTOR   0.001f     /* Convert mdps to dpg factor */

/* Factory test instructions. */

#define LSM6DSO_SIMPLE_CHECK      0x00       /* Simple check */
#define LSM6DSO_FULL_CHECK        0x01       /* Full check */

/* Factory test results. */

#define LSM6DSO_ST_PASS           0          /* Pass ST test */
#define LSM6DSO_ST_FAIL           -1         /* Failed ST test */

/* Self test results. */

#define LSM6DSO_ST_PASS           1          /* Pass ST test */
#define LSM6DSO_ST_FAIL           0          /* Failed ST test */

/* Self test limits. */

#define LSM6DSO_MIN_ST_LIMIT_MG   50.0f      /* Accelerator min limit */
#define LSM6DSO_MAX_ST_LIMIT_MG   1700.0f    /* Accelerator max limit */
#define LSM6DSO_MIN_ST_LIMIT_MDPS 150000.0f  /* Gyroscope min limit */
#define LSM6DSO_MAX_ST_LIMIT_MDPS 700000.0f  /* Gyroscope max limit */

/* Sensor device info */

#define LSM6DSO_DEVICE_ID         0x6c       /* Device Identification */
#define LSM6DSO_DEFAULT_INTERVAL  80000      /* Default conversion interval */
#define LSM6DSO_WAITBOOT_TIME     10         /* Wait Sensor boot time(ms) */

/* I3C status */

#define LSM6DSO_I3C_DISABLE       0x80       /* Disable the I3C */

/* Block data update */

#define LSM6DSO_BDU_CONTINUOUS    0x00       /* Continuous update */
#define LSM6DSO_BDU_UNTILREAD     0x01       /* Update until data read */

/* Timestamp counter */

#define LSM6DSO_TIMESTAMP_DISABLE 0x00       /* Disabled timestamp */
#define LSM6DSO_TIMESTAMP_ENABLE  0x01       /* Enable timestamp */

/* Accelerometer LP f2 enable */

#define LSM6DSO_LPF2_DISABLE      0x00       /* Disabled LPF2 */
#define LSM6DSO_LPF2_ENABLE       0x01       /* Enable LPF2 */

/* Accelerometer HP filter */

#define LSM6DSO_LPODR_DIV100      0x04       /* Accelerometer bandwidth 100 */

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

/* Device Register */

#define LSM6DSO_FIFO_CTRL1        0x07       /* FIFO control register (r/w) */
#define LSM6DSO_FIFO_CTRL2        0x08       /* FIFO control register (r/w) */
#define LSM6DSO_FIFO_CTRL3        0x09       /* FIFO control register (r/w) */
#define LSM6DSO_FIFO_CTRL4        0x0a       /* FIFO control register (r/w) */
#define LSM6DSO_COUNTER_BDR_REG1  0x0b       /* DataReady configuration register (r/w)*/
#define LSM6DSO_INT1_CTRL         0x0d       /* INT1 pad control register (r/w) */
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

/* Convert data */

union axis3bit16_u
{
  int16_t i16bit[3];                /* 16 bit int data */
  uint8_t u8bit[6];                 /* 8 bit unsigned int data */
};

typedef union axis3bit16_u axis3bit16_t;

/* Sensor struct */

struct lsm6dso_sensor_s
{
  struct sensor_lowerhalf_s lower;              /* Lower half sensor driver */
  unsigned int              interval;           /* Sensor interval */
  unsigned int              batch;              /* Sensor bat */
  unsigned int              fifowtm;            /* Sensor fifo water marker */
  unsigned int              last_update;        /* Last update flag */
  float                     factor;             /* Data factor */
  bool                      fifoen;             /* Sensor fifo enable */
  bool                      activated;          /* Sensor working state */
};

/* Device struct */

struct lsm6dso_dev_s
{
  struct lsm6dso_sensor_s     dev[LSM6DSO_IDX_NUM]; /* Sensor struct */
  uint64_t                    timestamp;            /* Units is microseconds */
  unsigned int                fifowtm;              /* fifo water marker */
  FAR struct lsm6dso_config_s *config;              /* The board config */
  struct work_s               work;                 /* Interrupt handler */
  bool                        fifoen;               /* Sensor fifo enable */
};

/* Sensor ODR */

struct lsm6dso_odr_s
{
  uint8_t regval;                  /* the data of register */
  float odr;                       /* the unit is Hz */
};

/* Batch BDR */

struct lsm6dso_bdr_s
{
  uint8_t regval;                   /* the data of register */
  float   bdr;                      /* the unit is Hz */
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

/* Accelerator handle functions */

static int lsm6dso_xl_setint1(FAR struct lsm6dso_dev_s *priv, uint8_t value);
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
                              FAR struct sensor_event_accel *value);
static int lsm6dso_xl_sethpfilter(FAR struct lsm6dso_dev_s *priv,
                                  uint8_t value);
static int lsm6dso_xl_setlp2filter(FAR struct lsm6dso_dev_s *priv,
                                   uint8_t value);

/* Gyroscope handle functions */

static int lsm6dso_gy_setint1(FAR struct lsm6dso_dev_s *priv, uint8_t value);
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
                              FAR struct sensor_event_gyro *value);

/* FIFO handle functions */

static int lsm6dso_fifo_setint1(FAR struct lsm6dso_dev_s *priv,
                                uint8_t value);
static int lsm6dso_fifo_iswtm(FAR struct lsm6dso_dev_s *priv,
                              FAR uint8_t *value);
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

/* Sensor ops functions */

static int lsm6dso_batch(FAR struct sensor_lowerhalf_s *lower,
                         FAR unsigned int *latency_us);
static int lsm6dso_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR unsigned int *period_us);
static int lsm6dso_activate(FAR struct sensor_lowerhalf_s *lower,
                            bool enable);
static int lsm6dso_selftest(FAR struct sensor_lowerhalf_s *lower,
                            unsigned long arg);

/* Sensor interrupt functions */

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
  .selftest = lsm6dso_selftest          /* Sensor selftest function */
};

static const struct sensor_ops_s g_lsm6dso_gy_ops =
{
  .activate = lsm6dso_activate,         /* Enable/disable sensor */
  .set_interval = lsm6dso_set_interval, /* Set output data period */
  .batch = lsm6dso_batch,               /* Set maximum report latency */
  .selftest = lsm6dso_selftest          /* Sensor selftest function */
};

static const struct lsm6dso_odr_s g_lsm6dso_xl_odr[] =
{
  {LSM6DSO_XL_ODR_12p5HZ, 12.5},   /* Sampling interval is 80ms */
  {LSM6DSO_XL_ODR_26HZ,   26},     /* Sampling interval is about 38.462ms */
  {LSM6DSO_XL_ODR_52HZ,   52},     /* Sampling interval is about 19.231ms */
  {LSM6DSO_XL_ODR_104HZ,  104},    /* Sampling interval is about 9.616ms */
  {LSM6DSO_XL_ODR_208HZ,  208},    /* Sampling interval is about 4.808ms */
  {LSM6DSO_XL_ODR_416HZ,  416},    /* Sampling interval is about 2.404ms */
  {LSM6DSO_XL_ODR_833HZ,  833},    /* Sampling interval is about 1.201ms */
};

static const struct lsm6dso_odr_s g_lsm6dso_gy_odr[] =
{
  {LSM6DSO_GY_ODR_12p5HZ, 12.5},    /* Sampling interval is 80ms */
  {LSM6DSO_GY_ODR_26HZ,   26},      /* Sampling interval is about 38.462ms */
  {LSM6DSO_GY_ODR_52HZ,   52},      /* Sampling interval is about 19.231ms */
  {LSM6DSO_GY_ODR_104HZ,  104},     /* Sampling interval is about 9.616ms */
  {LSM6DSO_GY_ODR_208HZ,  208},     /* Sampling interval is about 4.808ms */
  {LSM6DSO_GY_ODR_416HZ,  416},     /* Sampling interval is about 2.404ms */
  {LSM6DSO_GY_ODR_833HZ,  833},     /* Sampling interval is about 1.201ms */
};

static const struct lsm6dso_bdr_s g_lsm6dso_xl_bdr[] =
{
  {LSM6DSO_XL_BDR_12p5Hz, 12.5},    /* Sampling interval is 80ms */
  {LSM6DSO_XL_BDR_26Hz,   26},      /* Sampling interval is about 38.462ms */
  {LSM6DSO_XL_BDR_52Hz,   52},      /* Sampling interval is about 19.231ms */
  {LSM6DSO_XL_BDR_104Hz,  104},     /* Sampling interval is about 9.616ms */
  {LSM6DSO_XL_BDR_208Hz,  208},     /* Sampling interval is about 4.808ms */
  {LSM6DSO_XL_BDR_417Hz,  417},     /* Sampling interval is about 2.398ms */
  {LSM6DSO_XL_BDR_833Hz,  833},     /* Sampling interval is about 1.201ms */
};

static const struct lsm6dso_bdr_s g_lsm6dso_gy_bdr[] =
{
  {LSM6DSO_GY_BDR_12p5Hz, 12.5},    /* Sampling interval is 80ms */
  {LSM6DSO_GY_BDR_26Hz,   26},      /* Sampling interval is about 38.462ms */
  {LSM6DSO_GY_BDR_52Hz,   52},      /* Sampling interval is about 19.231ms */
  {LSM6DSO_GY_BDR_104Hz,  104},     /* Sampling interval is about 9.616ms */
  {LSM6DSO_GY_BDR_208Hz,  208},     /* Sampling interval is about 4.808ms */
  {LSM6DSO_GY_BDR_417Hz,  417},     /* Sampling interval is about 2.398ms */
  {LSM6DSO_GY_BDR_833Hz,  833},     /* Sampling interval is about 1.201ms */
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
  struct sensor_event_accel temp_xl;
  struct sensor_event_gyro temp_gy;
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];
  int st_result;
  uint8_t valuezero;
  uint8_t drdy;
  uint8_t i;
  uint8_t j;

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

      if (st_result == LSM6DSO_ST_PASS)
        {
          sninfo("Self Test - PASS\n");
        }
      else
        {
          sninfo("Self Test - FAIL\n");
        }
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
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
      sninfo("lsm6dso reset wait timeout!\n");
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
 * Name: lsm6dso_xl_setint1
 *
 * Description:
 *   Set interrupt for accelerometer.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - INT state.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_xl_setint1(FAR struct lsm6dso_dev_s *priv, uint8_t value)
{
  lsm6dso_int1_ctrl_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_INT1_CTRL, (FAR uint8_t *)&reg, 1);

  reg.int1_drdy_xl = value;
  lsm6dso_spi_write(priv, LSM6DSO_INT1_CTRL, (FAR uint8_t *)&reg);

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
      if (*freq < g_lsm6dso_xl_odr[i].odr
          || *freq == g_lsm6dso_xl_odr[i].odr)
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
       * Turn on the accelerometer: +-2g.
       */

      lsm6dso_xl_setfullscale(priv, LSM6DSO_2G);
      priv->dev[LSM6DSO_XL_IDX].factor = LSM6DSO_2G_FACTOR
                                       * LSM6DSO_MG2G_FACTOR;

      /* Configure filtering chain(No aux interface)
       * Accelerometer - LPF1 + LPF2 path.
       */

      lsm6dso_xl_sethpfilter(priv, LSM6DSO_LPODR_DIV100);
      lsm6dso_xl_setlp2filter(priv, LSM6DSO_LPF2_ENABLE);

      /* Set interrupt for accelerometer. */

      if (priv->dev[LSM6DSO_XL_IDX].fifoen)
        {
          lsm6dso_xl_setint1(priv, LSM6DSO_XL_INT_DISABLE);
        }
      else
        {
          lsm6dso_xl_setint1(priv, LSM6DSO_XL_INT_ENABLE);
        }
    }
  else
    {
      /* Set to Shut Down. */

      lsm6dso_xl_setodr(priv, LSM6DSO_XL_ODR_OFF);
      lsm6dso_xl_setint1(priv, LSM6DSO_XL_INT_DISABLE);
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
                              FAR struct sensor_event_accel *value)
{
  axis3bit16_t temp;

  lsm6dso_spi_read(priv, regaddr, temp.u8bit, 6);

  value->x = temp.i16bit[0] *priv->dev[LSM6DSO_XL_IDX].factor;
  value->y = temp.i16bit[1] *priv->dev[LSM6DSO_XL_IDX].factor;
  value->z = temp.i16bit[2] *priv->dev[LSM6DSO_XL_IDX].factor;

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_xl_sethpfilter
 *
 * Description:
 *   Accelerometer slope filter / high-pass filter selection on output.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Set value.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_xl_sethpfilter(FAR struct lsm6dso_dev_s *priv,
                                  uint8_t value)
{
  lsm6dso_ctrl8_xl_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL8_XL, (FAR uint8_t *)&reg, 1);

  reg.hp_slope_xl_en = (value & 0x10) >> 4;
  reg.hp_ref_mode_xl = (value & 0x20) >> 5;
  reg.hpcf_xl = value & 0x07;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL8_XL, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_xl_setlp2filter
 *
 * Description:
 *   Accelerometer output from LPF2 filtering stage selection.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Set value.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_xl_setlp2filter(FAR struct lsm6dso_dev_s *priv,
                                   uint8_t value)
{
  lsm6dso_ctrl1_xl_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_CTRL1_XL, (FAR uint8_t *)&reg, 1);

  reg.lpf2_xl_en = value;
  lsm6dso_spi_write(priv, LSM6DSO_CTRL1_XL, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_gy_setint1
 *
 * Description:
 *   Set interrupt for gyroscope.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - INT state.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_gy_setint1(FAR struct lsm6dso_dev_s *priv, uint8_t value)
{
  lsm6dso_int1_ctrl_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_INT1_CTRL, (FAR uint8_t *)&reg, 1);

  reg.int1_drdy_g = value;
  lsm6dso_spi_write(priv, LSM6DSO_INT1_CTRL, (FAR uint8_t *)&reg);

  return OK;
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
      if (*freq < g_lsm6dso_gy_odr[i].odr
         || *freq == g_lsm6dso_gy_odr[i].odr)
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
                                       * LSM6DSO_MDPS2DPS_FACTOR;

      /* Set interrupt for gyroscope. */

      if (priv->dev[LSM6DSO_GY_IDX].fifoen)
        {
          lsm6dso_gy_setint1(priv, LSM6DSO_GY_INT_DISABLE);
        }
      else
        {
          lsm6dso_gy_setint1(priv, LSM6DSO_GY_INT_ENABLE);
        }
    }
  else
    {
      /* Set to Shut Down */

      lsm6dso_gy_setodr(priv, LSM6DSO_GY_ODR_OFF);
      lsm6dso_gy_setint1(priv, LSM6DSO_GY_INT_DISABLE);
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
                              FAR struct sensor_event_gyro *value)
{
  axis3bit16_t temp;

  lsm6dso_spi_read(priv, regaddr, temp.u8bit, 6);

  value->x = temp.i16bit[0] * priv->dev[LSM6DSO_GY_IDX].factor;
  value->y = temp.i16bit[1] * priv->dev[LSM6DSO_GY_IDX].factor;
  value->z = temp.i16bit[2] * priv->dev[LSM6DSO_GY_IDX].factor;

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_fifo_setint1
 *
 * Description:
 *   Set interrupt for FIFO.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - INT state.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fifo_setint1(FAR struct lsm6dso_dev_s *priv,
                                uint8_t value)
{
  lsm6dso_int1_ctrl_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_INT1_CTRL, (FAR uint8_t *)&reg, 1);

  reg.int1_fifo_full = value;
  reg.int1_fifo_ovr = value;
  reg.int1_fifo_th = value;
  lsm6dso_spi_write(priv, LSM6DSO_INT1_CTRL, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_fifo_iswtm
 *
 * Description:
 *   Read the FIFO watermark status.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - INT state.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lsm6dso_fifo_iswtm(FAR struct lsm6dso_dev_s *priv,
                              FAR uint8_t *value)
{
  lsm6dso_fifo_status2_t reg;

  lsm6dso_spi_read(priv, LSM6DSO_FIFO_STATUS2, (FAR uint8_t *)&reg, 1);
  *value = reg.fifo_wtm_ia;

  return OK;
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
  struct sensor_event_accel
         temp_xl[CONFIG_SENSORS_LSM6DSO_FIFO_SLOTS_NUMBER];
  struct sensor_event_gyro
         temp_gy[CONFIG_SENSORS_LSM6DSO_FIFO_SLOTS_NUMBER];
  unsigned int counter_xl = 0;
  unsigned int counter_gy = 0;
  unsigned int num;
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
              counter_xl++;
            }
            break;

          case LSM6DSO_GYRO_NC_TAG:  /* Gyroscope data tag */
            {
              lsm6dso_gy_getdata(priv,
                                 LSM6DSO_FIFO_DATA_OUT_X_L,
                                 &temp_gy[counter_gy]);
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

        for (i = 0; i < counter_xl; i++)
          {
            temp_xl[i].timestamp
              = priv->timestamp
              - priv->dev[LSM6DSO_XL_IDX].interval
              * (counter_xl - i - 1);
          }

        /* Push data to the upper layer. */

        priv->dev[LSM6DSO_XL_IDX].lower.push_event(
              priv->dev[LSM6DSO_XL_IDX].lower.priv,
              temp_xl,
              sizeof(struct sensor_event_accel) * counter_xl);
      }

    if (counter_gy)
      {
        /* Inferred data timestamp. */

        for (i = 0; i < counter_gy; i++)
          {
            temp_gy[i].timestamp
              = priv->timestamp
              - priv->dev[LSM6DSO_GY_IDX].interval
              * (counter_gy - i - 1);
          }

        /* Push data to the upper layer. */

        priv->dev[LSM6DSO_GY_IDX].lower.push_event(
              priv->dev[LSM6DSO_GY_IDX].lower.priv,
              temp_gy,
              sizeof(struct sensor_event_gyro) * counter_gy);
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
                         FAR unsigned int *latency_us)
{
  FAR struct lsm6dso_sensor_s *sensor = (FAR struct lsm6dso_sensor_s *)lower;
  FAR struct lsm6dso_dev_s * priv;
  uint32_t max_latency;
  int idx;

  /* Sanity check. */

  DEBUGASSERT(sensor != NULL && latency_us != NULL);

  max_latency = sensor->lower.batch_number * sensor->interval;
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

      if (lower->type == SENSOR_TYPE_ACCELEROMETER)
        {
          lsm6dso_fifo_xl_setbatch(priv, g_lsm6dso_xl_bdr[idx].regval);
        }
      else if (lower->type == SENSOR_TYPE_GYROSCOPE)
        {
          lsm6dso_fifo_gy_setbatch(priv, g_lsm6dso_gy_bdr[idx].regval);
        }
      else
        {
          snerr("Failed to match sensor type.\n");
          return -EINVAL;
        }
    }
  else
    {
      sensor->fifoen = false;
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
      lsm6dso_fifo_setint1(priv, LSM6DSO_FIFO_INT_DISABLE);
      lsm6dso_fifo_setmode(priv, LSM6DSO_BYPASS_MODE);
    }
  else
    {
      priv->fifoen = true;
      lsm6dso_fifo_setint1(priv, LSM6DSO_FIFO_INT_ENABLE);

      if (priv->dev[LSM6DSO_XL_IDX].fifowtm
          > priv->dev[LSM6DSO_XL_IDX].lower.batch_number)
        {
          priv->dev[LSM6DSO_XL_IDX].fifowtm
          = priv->dev[LSM6DSO_XL_IDX].lower.batch_number;
        }

      if (priv->dev[LSM6DSO_GY_IDX].fifowtm
         > priv->dev[LSM6DSO_GY_IDX].lower.batch_number)
        {
          priv->dev[LSM6DSO_GY_IDX].fifowtm
          = priv->dev[LSM6DSO_GY_IDX].lower.batch_number;
        }

      priv->fifowtm = priv->dev[LSM6DSO_XL_IDX].fifowtm
                    + priv->dev[LSM6DSO_GY_IDX].fifowtm;

      lsm6dso_fifo_setwatermark(priv, priv->fifowtm);
      lsm6dso_fifo_setmode(priv, LSM6DSO_STREAM_MODE);
    }

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
                                FAR unsigned int *period_us)
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
  else
    {
      snerr("Failed to match sensor type.\n");
      ret = -EINVAL;
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
 *   lower  - The instance of lower half sensor driver
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

static int lsm6dso_activate(FAR struct sensor_lowerhalf_s *lower,
                            bool enable)
{
  FAR struct lsm6dso_sensor_s *sensor = (FAR struct lsm6dso_sensor_s *)lower;
  FAR struct lsm6dso_dev_s * priv;
  int ret;

  DEBUGASSERT(lower != NULL);

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      priv = (struct lsm6dso_dev_s *)(sensor - LSM6DSO_XL_IDX);
      if (sensor->activated != enable)
        {
          if (enable)
            {
              IOEXP_SETOPTION(priv->config->ioedev,
                              priv->config->pin,
                              IOEXPANDER_OPTION_INTCFG,
                              IOEXPANDER_VAL_RISING);
            }

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
          if (enable)
            {
              IOEXP_SETOPTION(priv->config->ioedev,
                              priv->config->pin,
                              IOEXPANDER_OPTION_INTCFG,
                              IOEXPANDER_VAL_RISING);
            }

          ret = lsm6dso_gy_enable(priv, enable);
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

  if (priv->dev[LSM6DSO_XL_IDX].activated == false
      && priv->dev[LSM6DSO_GY_IDX].activated == false)
    {
      IOEXP_SETOPTION(priv->config->ioedev,
                      priv->config->pin,
                      IOEXPANDER_OPTION_INTCFG,
                      IOEXPANDER_VAL_DISABLE);
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

static int lsm6dso_selftest(FAR struct sensor_lowerhalf_s *lower,
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

/* Sensor interrupt functions */

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
                      IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);

  return OK;
}

/****************************************************************************
 * Name: lsm6dso_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long. Also we cannot lock
 *   the I2C bus from within an interrupt.
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
  struct sensor_event_accel temp_xl;
  struct sensor_event_gyro temp_gy;
  uint8_t drdy;

  /* Sanity check. */

  DEBUGASSERT(priv != NULL);

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                      IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_RISING);

  if (priv->fifoen)
    {
      lsm6dso_fifo_iswtm(priv, &drdy);
      if (drdy)
        {
          lsm6dso_fifo_readdata(priv);
        }
    }
  else
    {
      /* Check if new value available. */

      lsm6dso_xl_isready(priv, &drdy);
      if (drdy)
        {
          /* Read out the latest sensor data. */

          lsm6dso_xl_getdata(priv, LSM6DSO_OUTX_L_XL, &temp_xl);
          temp_xl.timestamp = priv->timestamp;

          /* push data to upper half driver. */

          priv->dev[LSM6DSO_XL_IDX].lower.push_event(
                priv->dev[LSM6DSO_XL_IDX].lower.priv,
                &temp_xl,
                sizeof(struct sensor_event_accel));
        }

      /* Check if new value available. */

      lsm6dso_gy_isready(priv, &drdy);
      if (drdy)
        {
          /* Read out the latest sensor data. */

          lsm6dso_gy_getdata(priv, LSM6DSO_OUTX_L_G, &temp_gy);
          temp_gy.timestamp = priv->timestamp;

          /* push data to upper half driver. */

          priv->dev[LSM6DSO_GY_IDX].lower.push_event(
                priv->dev[LSM6DSO_GY_IDX].lower.priv,
                &temp_gy,
                sizeof(struct sensor_event_gyro));
        }
    }
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
  void *ioephandle;
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
  priv->dev[LSM6DSO_XL_IDX].lower.buffer_number
                            = CONFIG_SENSORS_LSM6DSO_BUFFER_NUMBER;
  priv->dev[LSM6DSO_XL_IDX].lower.batch_number
                            = CONFIG_SENSORS_LSM6DSO_FIFO_SLOTS_NUMBER;

  priv->dev[LSM6DSO_GY_IDX].lower.ops = &g_lsm6dso_gy_ops;
  priv->dev[LSM6DSO_GY_IDX].lower.type = SENSOR_TYPE_GYROSCOPE;
  priv->dev[LSM6DSO_GY_IDX].lower.uncalibrated = true;
  priv->dev[LSM6DSO_GY_IDX].interval = LSM6DSO_DEFAULT_INTERVAL;
  priv->dev[LSM6DSO_GY_IDX].lower.buffer_number
                            = CONFIG_SENSORS_LSM6DSO_BUFFER_NUMBER;
  priv->dev[LSM6DSO_GY_IDX].lower.batch_number
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
                        IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);
  if (ret < 0)
    {
      snerr("Failed to set option: %d\n", ret);
      IOEP_DETACH(priv->config->ioedev, lsm6dso_interrupt_handler);
      goto err;
    }

  /* Register the character driver. */

  ret = sensor_register((&(priv->dev[LSM6DSO_XL_IDX].lower)), devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      IOEP_DETACH(priv->config->ioedev, lsm6dso_interrupt_handler);
      sensor_unregister((&(priv->dev[LSM6DSO_XL_IDX].lower)), devno);
      goto err;
    }

  ret = sensor_register((&(priv->dev[LSM6DSO_GY_IDX].lower)), devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      IOEP_DETACH(priv->config->ioedev, lsm6dso_interrupt_handler);
      sensor_unregister((&(priv->dev[LSM6DSO_GY_IDX].lower)), devno);
      goto err;
    }

  return ret;

err:
  kmm_free(priv);
  return ret;
}
