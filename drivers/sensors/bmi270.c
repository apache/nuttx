/****************************************************************************
 * drivers/sensors/bmi270.c
 * Character driver for bmi270.
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
#include <nuttx/sensors/bmi270.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Roundup func define */

#define BMI270_ROUNDUP(x, esize)      ((x + (esize - 1)) / (esize)) * (esize)

#define BMI270_SPI_WRITE_MAX_BUFFER   50         /* SPI write buffer size */
#define BMI270_SPI_READ_MAX_BUFFER    20         /* SPI read buffer size */
#define BMI270_WAIT_COUNT_MAX         300        /* Max count wait for reset */
#define BMI270_WAIT_LOAD              140000     /* Wait for loading config file(us) */
#define BMI270_WAIT_SELFTEST          500000     /* Wait for selftest complete(us) */
#define BMI270_WAIT_SELFTEST_COUNT    5          /* Max count of wait for selftest complete */
#define BMI270_SET_DELAY              10000      /* Delay after set(us) */
#define BMI270_POWER_MODE_DELAY       1000       /* Delay after power mode switch(us) */
#define BMI270_READWRITE_LEN          46         /* Max read/write length */
#define BMI270_READWRITE_RLEN         2          /* Remaind read/write length */
#define BMI270_FEAT_SIZE_IN_BYTES     16         /* Bytes remaining to read */
#define BMI270_CONFIG_LOAD_SUCCESS    1          /* Config load success */
#define BMI270_VECTOR_REMAP           0          /* Vector remap of bmi270 */
#define BMI270_ODR_FLT_EPSILON        0.1f       /* ODR float epsilon */
#define GRAVITY_EARTH                 9.80665f   /* Gravity of earth */
#define BMI270_HALF_SCALE             32768.0f   /* Half scale of sensor((1 << 16) / 2.0f) */

#define BMI270_ENABLE                 1          /* Enable value */
#define BMI270_DISABLE                0          /* Disable value */

/* IO control command. */

#define BMI270_FEAT_MANAGE_CMD        0xf1       /* Feature manage command */
#define BMI270_SET_SCALE_XL_CMD       0xf2       /* Set accelerator scale command */

#define BMI270_XL_SET_2G              2          /* Accelerometer set 2G */
#define BMI270_XL_SET_4G              4          /* Accelerometer set 4G */
#define BMI270_XL_SET_8G              8          /* Accelerometer set 8G */
#define BMI270_XL_SET_16G             16         /* Accelerometer set 16G */

/* Available command */

#define BMI270_G_TRIGGER_CMD          0x02       /* Trigger special gyro operations */
#define BMI270_SOFT_RESET_CMD         0xb6       /* Software reset command */

/* Sensor ODR */

#define BMI270_UNIT_TIME              1000000.0f /* Unit time 1000000us */

#define BMI270_XL_ODR_12p5HZ          0x05       /* Accelerator ODR 12.5Hz */
#define BMI270_XL_ODR_25HZ            0x06       /* Accelerator ODR 25Hz */
#define BMI270_XL_ODR_50HZ            0x07       /* Accelerator ODR 50Hz */
#define BMI270_XL_ODR_100HZ           0x08       /* Accelerator ODR 100Hz */
#define BMI270_XL_ODR_200HZ           0x09       /* Accelerator ODR 200Hz */
#define BMI270_XL_ODR_400HZ           0x0a       /* Accelerator ODR 400Hz */
#define BMI270_XL_ODR_800HZ           0x0b       /* Accelerator ODR 800Hz */
#define BMI270_XL_ODR_1600HZ          0x0c       /* Accelerator ODR 1600Hz */

#define BMI270_GY_ODR_12p5HZ          0x05       /* Gyroscope ODR 12.5Hz */
#define BMI270_GY_ODR_25HZ            0x06       /* Gyroscope ODR 25Hz */
#define BMI270_GY_ODR_50HZ            0x07       /* Gyroscope ODR 50Hz */
#define BMI270_GY_ODR_100HZ           0x08       /* Gyroscope ODR 100Hz */
#define BMI270_GY_ODR_200HZ           0x09       /* Gyroscope ODR 200Hz */
#define BMI270_GY_ODR_400HZ           0x0a       /* Gyroscope ODR 400Hz */
#define BMI270_GY_ODR_800HZ           0x0b       /* Gyroscope ODR 800Hz */

/* Mask */

#define BMI270_WRIST_WAKE_UP_EN_MASK  0x10       /* Wrist wear wake up feature enable mask */
#define BMI270_CRT_MASK               0x01       /* Crt gyroscope self test mask */
#define BMI270_GYRO_CROSS_MASK        0x7f       /* Gyro cross axes sense mask */
#define BMI270_GYRO_CROSS_SIGN_MASK   0x40       /* Gyro cross axes sense sign bit mask */

/* Address */

#define BMI270_WRIST_WEAR_WAKE_UP     0x00       /* Bmi270 wrist wear wake up start address */
#define BMI270_CRT_GYRO_SELF_TEST     0x03       /* Bmi270 crt gyro self-test start address */
#define BMI270_AXIS_MAP               0x04       /* Axis map start address */
#define BMI270_GYRO_GAIN_UPDATE       0x06       /* Bmi270 gyro gain update start address */
#define BMI270_GYRO_CROSS_SENSE_ADDR  0x0c       /* Bmi270 context gyro cross sense start address */

#define BMI270_PAGE_0                 0          /* Page 0 address */
#define BMI270_PAGE_1                 1          /* Page 1 address */
#define BMI270_PAGE_7                 7          /* Page 7 address */

/* Error code */

#define BMI270_CONFIG_LOAD_ERROR      -9         /* Config load error code */

/* Sensor device info */

#define BMI270_DEVICE_ID              0x24       /* Device Identification */
#define BMI270_DEFAULT_INTERVAL       40000      /* Default conversion interval(us) */
#define BMI270_WAIT_TIME              10000      /* Sensor boot wait time(ms) */

/* Multi sensor index */

#define BMI270_XL_IDX                 0          /* Accelerator index */
#define BMI270_GY_IDX                 1          /* Gyroscope index */
#define BMI270_FEAT_IDX               2          /* Feature index */
#define BMI270_IDX_NUM                3          /* Max index */

/* Finite state machine define */

#define BMI270_DEFAULT_FSM_EN         0x07lu     /* Default FSM enable */

/* Accelerometer G Range */

#define BMI270_XL_RANGE_2G            0x00       /* Accelerometer range 2G */
#define BMI270_XL_RANGE_4G            0x01       /* Accelerometer range 4G */
#define BMI270_XL_RANGE_8G            0x02       /* Accelerometer range 8G */
#define BMI270_XL_RANGE_16G           0x03       /* Accelerometer range 16G */

#define BMI270_2G_FACTOR              2.0f       /* Accelerator 2g factor */
#define BMI270_4G_FACTOR              4.0f       /* Accelerator 4g factor */
#define BMI270_8G_FACTOR              8.0f       /* Accelerator 8g factor */
#define BMI270_16G_FACTOR             16.0f      /* Accelerator 16g factor */

/* Accelerometer bandwidth parameters */

#define BMI270_XL_OSR4_AVG1           0x00       /* Accelerometer bandwidth value */
#define BMI270_XL_OSR2_AVG2           0x01       /* Accelerometer bandwidth value */
#define BMI270_XL_NORMAL_AVG4         0x02       /* Accelerometer bandwidth value */
#define BMI270_XL_CIC_AVG8            0x03       /* Accelerometer bandwidth value */
#define BMI270_XL_RES_AVG16           0x04       /* Accelerometer bandwidth value */
#define BMI270_XL_RES_AVG32           0x05       /* Accelerometer bandwidth value */
#define BMI270_XL_RES_AVG64           0x06       /* Accelerometer bandwidth value */
#define BMI270_XL_RES_AVG128          0x07       /* Accelerometer bandwidth value */

/* Gyroscope Angular Rate Measurement Range */

#define BMI270_GY_RANGE_2000          0x00       /* Gyroscope angular rate measurement range, +/-2000dps, 16.4 LSB/dps */
#define BMI270_GY_RANGE_1000          0x01       /* Gyroscope angular rate measurement range, +/-1000dps, 32.8 LSB/dps */
#define BMI270_GY_RANGE_500           0x02       /* Gyroscope angular rate measurement range, +/-500dps, 65.6 LSB/dps */
#define BMI270_GY_RANGE_250           0x03       /* Gyroscope angular rate measurement range, +/-250dps, 131.2 LSB/dps */
#define BMI270_GY_RANGE_125           0x04       /* Gyroscope angular rate measurement range, +/-125dps, 262.4 LSB/dps */

#define BMI270_125DPS_FACTOR          125.0f     /* Gyroscope 125dps factor */
#define BMI270_250DPS_FACTOR          250.0f     /* Gyroscope 250dps factor */
#define BMI270_500DPS_FACTOR          500.0f     /* Gyroscope 500dps factor */
#define BMI270_1000DPS_FACTOR         1000.0f    /* Gyroscope 1000dps factor */
#define BMI270_2000DPS_FACTOR         2000.0f    /* Gyroscope 2000dps factor */
#define BMI270_DPS2RPS_FACTOR         (M_PI/180) /* Convert dps to rad/s factor */

/* Gyroscope Bandwidth parameters */

#define BMI270_GY_OSR4_MODE           0x00       /* Gyroscope bandwidth coefficient OSR4 mode */
#define BMI270_GY_OSR2_MODE           0x01       /* Gyroscope bandwidth coefficient OSR2 mode */
#define BMI270_GY_NORMAL_MODE         0x02       /* Gyroscope bandwidth coefficient normal mode */
#define BMI270_GY_CIC_MODE            0x03       /* reserved */

/* Gyroscope performance optimized  */

#define BMI270_PERF_OPT_MODE          1          /* High performance mode */

/* Factory test instructions. */

#define BMI270_SIMPLE_CHECK           0x00       /* Simple check */
#define BMI270_FULL_CHECK             0x01       /* Full check */

/* Self test. */

#define BMI270_SELECT_GYRO_SELF_TEST  0x00       /* Select gyroscope self test. */

/* Self test results. */

#define BMI270_ST_PASS                0          /* Pass self test */
#define BMI270_ST_FAIL                -1         /* Failed self test */

/* Self test: Resulting minimum difference signal. */

#define BMI270_ST_X_AXIS              16         /* x-axis signal should > +16g */
#define BMI270_ST_Y_AXIS              -15        /* y-axis signal should < -15g */
#define BMI270_ST_Z_AXIS              10         /* z-axis signal should > +10g */

/* FIFO */

#define BMI270_FIFO_DATA_BLOCK_SIZE   0x07       /* Data size of the fifo frame */
#define BMI270_FIFO_REGULAR_FRAME     0x02       /* fifo regular frame */
#define BMI270_FIFO_CONTROL_FRAME     0x01       /* fifo control frame */
#define BMI270_FIFO_PARM_XL           0x00       /* Accelerometer included in the data part of the frame */
#define BMI270_FIFO_PARM_GY           0x02       /* Gyroscope included in the data part of the frame */
#define BMI270_FIFO_PARM_AUX          0x04       /* Aux included in the data part of the frame */

/* Axes remap */

#define BMI270_MAP2X_AXIS             0x00       /* Map to x axis */
#define BMI270_MAP2Y_AXIS             0x01       /* Map to y axis */
#define BMI270_MAP2Z_AXIS             0x02       /* Map to z axis */

#define BMI270_MAP_SIGN_NOT_INVERT    0x00       /* Map the axis sign to not invert */
#define BMI270_MAP_SIGN_INVERT        0x01       /* Map the axis sign to invert */

/* Device Register */

#define BMI270_CHIP_ID                0x00       /* Chip identification code Register */
#define BMI270_DATA_8                 0x0c       /* ACC_X(LSB) */
#define BMI270_DATA_14                0x12       /* GYR_X(LSB) */
#define BMI270_INT_STATUS_0           0x1c       /* Interrupt/Feature Status. Will be cleared on read */
#define BMI270_INT_STATUS_1           0x1d       /* Interrupt Status 1. Will be cleared on read when bit 0 is sent out over the bus */
#define BMI270_INTERNAL_STATUS        0x21       /* Error bits and message indicating internal status */
#define BMI270_FIFO_LENGTH_0          0x24       /* FIFO byte count register (LSB) */
#define BMI270_FIFO_LENGTH_1          0x25       /* FIFO byte count register (MSB) */
#define BMI270_FIFO_DATA              0x26       /* FIFO data output register */
#define BMI270_FEAT_PAGE              0x2f       /* Page number for feature configuration and output registers */
#define BMI270_FEATURES               0x30       /* Input registers for feature configuration. Output for results */
#define BMI270_ACC_CONF               0x40       /* Sets the output data rate, the bandwidth, and the read mode of the acceleration sensor */
#define BMI270_ACC_RANGE              0x41       /* Selection of the Accelerometer g-range */
#define BMI270_GYR_CONF               0x42       /* Sets the output data rate and the bandwidth of the Gyroscope in the sensor */
#define BMI270_GYR_RANGE              0x43       /* Defines the Gyroscope angular rate measurement range */
#define BMI270_FIFO_WTM_0             0x46       /* FIFO Watermark level LSB */
#define BMI270_FIFO_WTM_1             0x47       /* FIFO Watermark level MSB and frame content configuration */
#define BMI270_FIFO_CONFIG_0          0x48       /* FIFO frame content configuration */
#define BMI270_FIFO_CONFIG_1          0x49       /* FIFO frame content configuration */
#define BMI270_INT1_IO_CTRL           0x53       /* Configure the electrical behavior of the interrupt pin */
#define BMI270_INT1_MAP_FEAT          0x56       /* Interrupt/Feature mapping on INT1 */
#define BMI270_INT2_MAP_FEAT          0x57       /* Interrupt/Feature mapping on INT2 */
#define BMI270_INT_MAP_DATA           0x58       /* Data Interrupt mapping for both INT pins */
#define BMI270_INIT_CTRL              0x59       /* Start initialization */
#define BMI270_INIT_ADDR_0            0x5b       /* Base address of the initialization data */
#define BMI270_INIT_DATA              0x5e       /* Initialization register */
#define BMI270_GYR_CRT_CONF           0x69       /* Component Retrimming for Gyroscope */
#define BMI270_ACC_SELF_TEST          0x6d       /* Settings for the accelerometer self-test configuration and trigger */
#define BMI270_GYR_SELF_TEST_AXES     0x6e       /* Settings for the gyroscope AXES self-test configuration and trigger */
#define BMI270_PWR_CONF               0x7c       /* Power mode configuration register */
#define BMI270_PWR_CTRL               0x7d       /* Power mode control register */
#define BMI270_CMD_REG                0x7e       /* Command Register */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Device Register Bit */

struct bmi270_int_status_0_s
{
  uint8_t sig_motion_out          : 1;
  uint8_t step_counter_out        : 1;
  uint8_t activity_out            : 1;
  uint8_t wrist_wear_weakeup_out  : 1;
  uint8_t wrist_gesture_out       : 1;
  uint8_t no_motion_out           : 1;
  uint8_t any_motion_out          : 1;
  uint8_t not_used_01             : 1;
};

typedef struct bmi270_int_status_0_s bmi270_int_status_0_t;

struct bmi270_int_status_1_s
{
  uint8_t ffull_int               : 1;
  uint8_t fwm_int                 : 1;
  uint8_t err_int                 : 1;
  uint8_t not_used_01             : 2;
  uint8_t aux_drdy_int            : 1;
  uint8_t gyr_drdy_int            : 1;
  uint8_t acc_drdy_int            : 1;
};

typedef struct bmi270_int_status_1_s bmi270_int_status_1_t;

struct bmi270_internal_status_s
{
  uint8_t message                 : 4;
  uint8_t not_used_01             : 1;
  uint8_t axes_remap_error        : 1;
  uint8_t odr_50hz_error          : 1;
  uint8_t not_used_02             : 1;
};

typedef struct bmi270_internal_status_s bmi270_internal_status_t;

struct bmi270_fifo_length_0_s
{
  uint8_t fifo_byte_counter_7_0   : 8;
};

typedef struct bmi270_fifo_length_0_s bmi270_fifo_length_0_t;

struct bmi270_fifo_length_1_s
{
  uint8_t fifo_byte_counter_13_8  : 6;
  uint8_t not_used_01             : 2;
};

typedef struct bmi270_fifo_length_1_s bmi270_fifo_length_1_t;

struct bmi270_acc_conf_s
{
  uint8_t acc_odr                 : 4;
  uint8_t acc_bwp                 : 3;
  uint8_t acc_filter_perf         : 1;
};

typedef struct bmi270_acc_conf_s bmi270_acc_conf_t;

struct bmi270_acc_range_s
{
  uint8_t acc_range               : 2;
  uint8_t not_used_01             : 6;
};

typedef struct bmi270_acc_range_s bmi270_acc_range_t;

struct bmi270_gyr_conf_s
{
  uint8_t gyr_odr                 : 4;
  uint8_t gyr_bwp                 : 2;
  uint8_t gyr_noise_perf          : 1;
  uint8_t gyr_filter_perf         : 1;
};

typedef struct bmi270_gyr_conf_s bmi270_gyr_conf_t;

struct bmi270_gyr_range_s
{
  uint8_t gyr_range               : 3;
  uint8_t ois_range               : 1;
  uint8_t not_used_01             : 4;
};

typedef struct bmi270_gyr_range_s bmi270_gyr_range_t;

struct bmi270_fifo_wtm_0_s
{
  uint8_t fifo_water_mark_7_0     : 8;
};

typedef struct bmi270_fifo_wtm_0_s bmi270_fifo_wtm_0_t;

struct bmi270_fifo_wtm_1_s
{
  uint8_t fifo_water_mark_12_8    : 5;
  uint8_t not_used_01             : 3;
};

typedef struct bmi270_fifo_wtm_1_s bmi270_fifo_wtm_1_t;

struct bmi270_fifo_config_0_s
{
  uint8_t fifo_stop_on_full       : 1;
  uint8_t fifo_time_en            : 1;
  uint8_t not_used_01             : 6;
};

typedef struct bmi270_fifo_config_0_s bmi270_fifo_config_0_t;

struct bmi270_fifo_config_1_s
{
  uint8_t fifo_tag_int1_en        : 2;
  uint8_t fifo_tag_int2_en        : 2;
  uint8_t fifo_header_en          : 1;
  uint8_t fifo_aux_en             : 1;
  uint8_t fifo_acc_en             : 1;
  uint8_t fifo_gyr_en             : 1;
};

typedef struct bmi270_fifo_config_1_s bmi270_fifo_config_1_t;

struct bmi270_int1_io_ctrl_s
{
  uint8_t not_used_01             : 1;
  uint8_t lvl                     : 1;
  uint8_t od                      : 1;
  uint8_t output_en               : 1;
  uint8_t input_en                : 1;
  uint8_t not_used_02             : 3;
};

typedef struct bmi270_int1_io_ctrl_s bmi270_int1_io_ctrl_t;

struct bmi270_int1_map_feat_s
{
  uint8_t sig_motion_out          : 1;
  uint8_t step_counter_out        : 1;
  uint8_t activity_out            : 1;
  uint8_t wrist_wear_wakeup_out   : 1;
  uint8_t wrist_gesture_out       : 1;
  uint8_t no_motion_out           : 1;
  uint8_t any_motion_out          : 1;
  uint8_t not_used_01             : 1;
};

typedef struct bmi270_int1_map_feat_s bmi270_int1_map_feat_t;

struct bmi270_int2_map_feat_s
{
  uint8_t sig_motion_out          : 1;
  uint8_t step_counter_out        : 1;
  uint8_t activity_out            : 1;
  uint8_t wrist_wear_wakeup_out   : 1;
  uint8_t wrist_gesture_out       : 1;
  uint8_t no_motion_out           : 1;
  uint8_t any_motion_out          : 1;
  uint8_t not_used_01             : 1;
};

typedef struct bmi270_int2_map_feat_s bmi270_int2_map_feat_t;

struct bmi270_int_map_data_s
{
  uint8_t ffull_int1              : 1;
  uint8_t fwm_int1                : 1;
  uint8_t drdy_int1               : 1;
  uint8_t err_int1                : 1;
  uint8_t ffull_int2              : 1;
  uint8_t fwm_int2                : 1;
  uint8_t drdy_int2               : 1;
  uint8_t err_int2                : 1;
};

typedef struct bmi270_int_map_data_s bmi270_int_map_data_t;

struct bmi270_init_ctrl_s
{
  uint8_t value                   : 8;
};

typedef struct bmi270_init_ctrl_s bmi270_init_ctrl_t;

struct bmi270_gyr_crt_conf_s
{
  uint8_t not_used_01             : 2;
  uint8_t crt_running             : 1;
  uint8_t rdy_for_dl              : 1;
  uint8_t not_used_02             : 4;
};

typedef struct bmi270_gyr_crt_conf_s bmi270_gyr_crt_conf_t;

struct bmi270_acc_self_test_s
{
  uint8_t acc_self_test_en        : 1;
  uint8_t not_used_01             : 1;
  uint8_t acc_self_test_sign      : 1;
  uint8_t acc_self_test_amp       : 1;
  uint8_t not_used_02             : 4;
};

typedef struct bmi270_acc_self_test_s bmi270_acc_self_test_t;

struct bmi270_gyr_self_test_axes_s
{
  uint8_t gyr_st_axes_done        : 1;
  uint8_t gyr_axis_x_ok           : 1;
  uint8_t gyr_axis_y_ok           : 1;
  uint8_t gyr_axis_z_ok           : 1;
  uint8_t not_used_01             : 4;
};

typedef struct bmi270_gyr_self_test_axes_s bmi270_gyr_self_test_axes_t;

struct bmi270_pwr_conf_s
{
  uint8_t adv_power_save          : 1;
  uint8_t fifo_self_wake_up       : 1;
  uint8_t fup_en                  : 1;
  uint8_t not_used_01             : 5;
};

typedef struct bmi270_pwr_conf_s bmi270_pwr_conf_t;

struct bmi270_pwr_ctrl_s
{
  uint8_t aux_en                  : 1;
  uint8_t gyr_en                  : 1;
  uint8_t acc_en                  : 1;
  uint8_t temp_en                 : 1;
  uint8_t not_used_01             : 4;
};

typedef struct bmi270_pwr_ctrl_s bmi270_pwr_ctrl_t;

struct bmi270_cmd_reg_s
{
  uint8_t value                   : 8;
};

typedef struct bmi270_cmd_reg_s bmi270_cmd_reg_t;

struct bmi270_fifo_header_s
{
  uint8_t fh_ext                  : 2;
  uint8_t fh_parm                 : 4;
  uint8_t fh_mode                 : 2;
};

typedef struct bmi270_fifo_header_s bmi270_fifo_header_t;

struct bmi270_gyr_user_gain_stat_s
{
  uint8_t sat_x                   : 1;
  uint8_t sat_y                   : 1;
  uint8_t sat_z                   : 1;
  uint8_t g_trigger_status        : 1;
  uint8_t not_used_01             : 4;
};

typedef struct bmi270_gyr_user_gain_stat_s bmi270_gyr_user_gain_stat_t;

struct bmi270_axes_remap_0_s
{
  uint8_t map_x_axis              : 2;
  uint8_t map_x_axis_sign         : 1;
  uint8_t map_y_axis              : 2;
  uint8_t map_y_axis_sign         : 1;
  uint8_t map_z_axis              : 2;
};

typedef struct bmi270_axes_remap_0_s bmi270_axes_remap_0_t;

struct bmi270_axes_remap_1_s
{
  uint8_t map_z_axis_sign         : 1;
  uint8_t gyr_self_off            : 1;
  uint8_t nvm_prog_prep           : 1;
  uint8_t not_used_01             : 5;
};

typedef struct bmi270_axes_remap_1_s bmi270_axes_remap_1_t;

/* Sensor struct */

struct bmi270_sensor_s
{
  struct sensor_lowerhalf_s lower;              /* Lower half sensor driver */
  struct work_s             work;               /* Sensor handler */
  unsigned int              interval;           /* Sensor interval */
  unsigned int              batch;              /* Sensor bat */
  unsigned int              fifowtm;            /* Sensor fifo water marker */
  unsigned int              last_update;        /* Last update flag */
  float                     factor;             /* Data factor */
  bool                      fifoen;             /* Sensor fifo enable */
  bool                      activated;          /* Sensor working state */
};

/* Device struct */

struct bmi270_dev_s
{
  struct bmi270_sensor_s dev[BMI270_IDX_NUM];   /* Sensor struct */
  FAR const struct bmi270_config_s *config;     /* The board config */
  uint64_t timestamp_fifolast;                  /* FIFO last timestamp, Units is microseconds */
  uint64_t timestamp;                           /* Units is microseconds */
  struct work_s work;                           /* Interrupt handler */
  unsigned int fifowtm;                         /* FIFO water marker */
  unsigned int featen;                          /* Feature enable */
  int16_t cross_sense;                          /* Gyroscope cross sensitivity coefficient */
  FAR uint8_t *fifo_buff;                       /* FIFO temp buffer */
  bool fifoen;                                  /* Sensor fifo enable */
};

/* Structure to store the value of re-mapped axis and its sign */

struct bmi270_axes_remap_s
{
  uint8_t x_axis;                               /* Re-mapped x-axis */
  uint8_t y_axis;                               /* Re-mapped y-axis */
  uint8_t z_axis;                               /* Re-mapped z-axis */
  uint8_t x_axis_sign;                          /* Re-mapped x-axis sign */
  uint8_t y_axis_sign;                          /* Re-mapped y-axis sign */
  uint8_t z_axis_sign;                          /* Re-mapped z-axis sign */
};

/* Wrist wear wake up struct */

struct bmi270_wakeup_s
{
  /* Variable to define the array offset */

  uint8_t idx;

  /* Cosine of minimum expected attitude change of the device
   * within 1 second time window when moving within focus position.
   * The parameter is scaled by 2048 i.e. 2048 * cos(angle).
   * Range is 1024 to 1774. Default is 1448.
   */

  uint16_t min_angle_focus;

  /* Cosine of minimum expected attitude change of the device
   * within 1 second time window when moving from non-focus to
   * focus position.
   * The parameter is scaled by 2048 i.e. 2048 * cos(angle).
   * Range is 1448 to 1856. Default value is 1774.
   */

  uint16_t min_angle_nonfocus;

  /* Sine of the maximum allowed downward tilt angle in
   * landscape right direction of the device, when it is in focus position
   * (i.e. user is able to comfortably look at the dial of wear device).
   * The configuration parameter is scaled by 2048 i.e. 2048 * sin(angle).
   * Range is 700 to 1024. Default value is 1024.
   */

  uint16_t max_tilt_lr;

  /* Sine of the maximum allowed downward tilt angle in
   * landscape left direction of the device, when it is in focus position
   * (i.e. user is able to comfortably look at the dial of wear device).
   * The configuration parameter is scaled by 2048 i.e. 2048 * sin(angle).
   * Range is 700 to 1024. Default value is 700.
   */

  uint16_t max_tilt_ll;

  /* Sine of the maximum allowed backward tilt angle in
   * portrait down direction of the device, when it is in focus position
   * (i.e. user is able to comfortably look at the dial of wear device).
   * The configuration parameter is scaled by 2048 i.e. 2048 * sin(angle).
   * Range is 0 to179. Default value is 179.
   */

  uint16_t max_tilt_pd;

  /* Sine of the maximum allowed forward tilt angle in
   * portrait up direction of the device, when it is in focus position
   * (i.e. user is able to comfortably look at the dial of wear device).
   * The configuration parameter is scaled by 2048 i.e. 2048 * sin(angle).
   * Range is 1774 to 1978. Default value is 1925.
   */

  uint16_t max_tilt_pu;
};

/* Sensor ODR */

struct bmi270_odr_s
{
  uint8_t regval;                               /* the data of register */
  float odr;                                    /* the unit is Hz */
};

/* Batch BDR */

struct bmi270_bdr_s
{
  uint8_t regval;                               /* the data of register */
  float   bdr;                                  /* the unit is Hz */
};

/* Convert data */

union axis3bit16_u
{
  int16_t i16bit[3];                            /* 16 bit int data */
  uint8_t u8bit[6];                             /* 8 bit unsigned int data */
};

typedef union axis3bit16_u axis3bit16_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI functions */

static void bmi270_spi_read(FAR struct bmi270_dev_s *priv,
                            uint8_t regaddr,
                            FAR uint8_t *regval,
                            uint8_t len);
static void bmi270_spi_read_enhance(FAR struct bmi270_dev_s *priv,
                                    uint8_t regaddr,
                                    FAR uint8_t *regval,
                                    uint8_t len);
static void bmi270_spi_write(FAR struct bmi270_dev_s *priv,
                             uint8_t regaddr,
                             FAR uint8_t *value);
static void bmi270_spi_write_len(FAR struct bmi270_dev_s *priv,
                                 uint8_t regaddr,
                                 FAR uint8_t *value,
                                 uint8_t len);

/* Sensor handle functions */

static int bmi270_readdevid(FAR struct bmi270_dev_s *priv);
static int bmi270_wait_selftest(FAR struct bmi270_dev_s *priv);
static int bmi270_datatest(FAR struct bmi270_dev_s *priv, int type);
static void bmi270_reset(FAR struct bmi270_dev_s *priv);
static void bmi270_resetwait(FAR struct bmi270_dev_s *priv);
static int bmi270_write_config(FAR struct bmi270_dev_s *priv);
static int bmi270_set_powersave(FAR struct bmi270_dev_s *priv,
                                uint8_t value);
static int bmi270_set_configload(FAR struct bmi270_dev_s *priv,
                                 uint8_t value);
static int bmi270_upload_file(FAR struct bmi270_dev_s *priv,
                              FAR uint8_t *config_data,
                              uint16_t index, uint16_t write_len);
static int bmi270_get_internal_status(FAR struct bmi270_dev_s *priv,
                                      FAR uint8_t *state);
static int bmi270_set_int(FAR struct bmi270_dev_s *priv,
                          uint8_t value);
static int bmi270_get_feat_config(FAR struct bmi270_dev_s *priv,
                                  uint8_t page, FAR uint8_t *feat_config);

/* Accelerator handle functions */

static int bmi270_xl_enable(FAR struct bmi270_dev_s *priv,
                            bool enable);
static void bmi270_xl_worker(FAR void *arg);
static int bmi270_xl_getdata(FAR struct bmi270_dev_s *priv,
                             uint8_t regaddr,
                             FAR struct sensor_event_accel *value);
static int bmi270_xl_findodr(FAR float *freq);
static int bmi270_xl_setodr(FAR struct bmi270_dev_s *priv, uint8_t value);
static int bmi270_xl_setfullscale(FAR struct bmi270_dev_s *priv,
                                  uint8_t value);
static int bmi270_xl_setfilter(FAR struct bmi270_dev_s *priv,
                               uint8_t value);

/* Gyroscope handle functions */

static int bmi270_get_gyro_crosssense(FAR struct bmi270_dev_s *priv,
                                      FAR int16_t *cross_sense);
static int bmi270_gy_enable(FAR struct bmi270_dev_s *priv,
                            bool enable);
static void bmi270_gy_worker(FAR void *arg);
static int bmi270_gy_getdata(FAR struct bmi270_dev_s *priv,
                             uint8_t regaddr,
                             FAR struct sensor_event_gyro *value);
static int bmi270_gy_findodr(FAR float *freq);
static int bmi270_gy_setodr(FAR struct bmi270_dev_s *priv, uint8_t value);
static int bmi270_gy_setfullscale(FAR struct bmi270_dev_s *priv,
                                   uint8_t value);
static int bmi270_gy_setfilter(FAR struct bmi270_dev_s *priv,
                               uint8_t value);

/* FIFO handle functions */

static int bmi270_fifo_setwatermark(FAR struct bmi270_dev_s *priv,
                                    unsigned int value);
static int bmi270_fifo_xl_enable(FAR struct bmi270_dev_s *priv,
                                 uint8_t value);
static int bmi270_fifo_gy_enable(FAR struct bmi270_dev_s *priv,
                                 uint8_t value);
static int bmi270_fifo_config(FAR struct bmi270_dev_s *priv);
static int bmi270_fifo_readdata(FAR struct bmi270_dev_s *priv);
static int bmi270_fifo_getlevel(FAR struct bmi270_dev_s *priv,
                                FAR unsigned int *value);

/* Feature handle functions */

static int bmi270_feat_enable(FAR struct bmi270_dev_s *priv,
                              bool enable);
static int bmi270_map_feat_int(FAR struct bmi270_dev_s *priv,
                               uint8_t value);
static int bmi270_get_wakeup_cfg(FAR struct bmi270_dev_s *priv,
                                 FAR struct bmi270_wakeup_s *weakup_config);
static int bmi270_set_wakeup_cfg(FAR struct bmi270_dev_s *priv,
                                 FAR struct bmi270_wakeup_s *weakup_config);
static int bmi270_set_axis_map(FAR struct bmi270_dev_s *priv,
                               FAR struct bmi270_axes_remap_s *axes_map);
static int bmi270_feat_handler(FAR struct bmi270_dev_s *priv,
                               bmi270_int_status_0_t regval_int0);
static int bmi270_feat_manage(FAR struct bmi270_dev_s *priv);

/* Sensor ops functions */

static int bmi270_batch(FAR struct sensor_lowerhalf_s *lower,
                        FAR unsigned int *latency_us);
static int bmi270_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR unsigned int *period_us);
static int bmi270_activate(FAR struct sensor_lowerhalf_s *lower,
                           bool enable);
static int bmi270_selftest(FAR struct sensor_lowerhalf_s *lower,
                           unsigned long arg);
static int bmi270_control(FAR struct sensor_lowerhalf_s *lower,
                          int cmd, unsigned long arg);

/* Sensor interrupt functions */

static int bmi270_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg);
static void bmi270_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_bmi270_xl_ops =
{
  .activate = bmi270_activate,         /* Enable/disable sensor */
  .set_interval = bmi270_set_interval, /* Set output data period */
  .batch = bmi270_batch,               /* Set maximum report latency */
  .selftest = bmi270_selftest,         /* Sensor selftest function */
  .control = bmi270_control            /* Set special config for sensor */
};

static const struct sensor_ops_s g_bmi270_gy_ops =
{
  .activate = bmi270_activate,         /* Enable/disable sensor */
  .set_interval = bmi270_set_interval, /* Set output data period */
  .batch = bmi270_batch,               /* Set maximum report latency */
  .selftest = bmi270_selftest          /* Sensor selftest function */
};

static const struct sensor_ops_s g_bmi270_fsm_ops =
{
  .activate = bmi270_activate,         /* Enable/disable sensor */
  .set_interval = bmi270_set_interval, /* Set output data period */
  .control = bmi270_control            /* Set special config for sensor */
};

static const struct bmi270_odr_s g_bmi270_xl_odr[] =
{
  {BMI270_XL_ODR_12p5HZ, 12.5},        /* Sampling interval is 80ms */
  {BMI270_XL_ODR_25HZ,   25},          /* Sampling interval is about 40ms */
  {BMI270_XL_ODR_50HZ,   50},          /* Sampling interval is about 20ms */
  {BMI270_XL_ODR_100HZ,  100},         /* Sampling interval is about 10ms */
  {BMI270_XL_ODR_200HZ,  200},         /* Sampling interval is about 5ms */
  {BMI270_XL_ODR_400HZ,  400},         /* Sampling interval is about 2.5ms */
  {BMI270_XL_ODR_800HZ,  800},         /* Sampling interval is about 1.25ms */
  {BMI270_XL_ODR_1600HZ, 1600},        /* Sampling interval is about 0.625ms */
};

static const struct bmi270_odr_s g_bmi270_gy_odr[] =
{
  {BMI270_GY_ODR_12p5HZ, 12.5},        /* Sampling interval is 80ms */
  {BMI270_GY_ODR_25HZ,   25},          /* Sampling interval is about 40ms */
  {BMI270_GY_ODR_50HZ,   50},          /* Sampling interval is about 20ms */
  {BMI270_GY_ODR_100HZ,  100},         /* Sampling interval is about 10ms */
  {BMI270_GY_ODR_200HZ,  200},         /* Sampling interval is about 5ms */
  {BMI270_GY_ODR_400HZ,  400},         /* Sampling interval is about 2.5ms */
  {BMI270_GY_ODR_800HZ,  800},         /* Sampling interval is about 1.25ms */
};

static uint8_t bmi270_config_file[] =
{
  0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x3d, 0xb1, 0xc8, 0x2e, 0x00, 0x2e,
  0x80, 0x2e, 0x91, 0x03, 0x80, 0x2e, 0xbc, 0xb0, 0x80, 0x2e, 0xa3, 0x03,
  0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x00, 0xb0, 0x50, 0x30, 0x21, 0x2e,
  0x59, 0xf5, 0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x80, 0x2e, 0x3b, 0x03,
  0x00, 0x00, 0x00, 0x00, 0x08, 0x19, 0x01, 0x00, 0x22, 0x00, 0x75, 0x00,
  0x00, 0x10, 0x00, 0x10, 0xd1, 0x00, 0xb3, 0x43, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0xe0, 0x5f, 0x00, 0x00,
  0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x19, 0x00, 0x00,
  0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xe0, 0xaa, 0x38,
  0x05, 0xe0, 0x90, 0x30, 0xfa, 0x00, 0x96, 0x00, 0x4b, 0x09, 0x11, 0x00,
  0x11, 0x00, 0x02, 0x00, 0x2d, 0x01, 0xd4, 0x7b, 0x3b, 0x01, 0xdb, 0x7a,
  0x04, 0x00, 0x3f, 0x7b, 0xcd, 0x6c, 0xc3, 0x04, 0x85, 0x09, 0xc3, 0x04,
  0xec, 0xe6, 0x0c, 0x46, 0x01, 0x00, 0x27, 0x00, 0x19, 0x00, 0x96, 0x00,
  0xa0, 0x00, 0x01, 0x00, 0x0c, 0x00, 0xf0, 0x3c, 0x00, 0x01, 0x01, 0x00,
  0x03, 0x00, 0x01, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x32, 0x00, 0x05, 0x00,
  0xee, 0x06, 0x04, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x04, 0x00, 0xa8, 0x05,
  0xee, 0x06, 0x00, 0x04, 0xbc, 0x02, 0xb3, 0x00, 0x85, 0x07, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb4, 0x00, 0x01, 0x00, 0xb9, 0x00,
  0x01, 0x00, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x80, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x80, 0x2e, 0x00, 0xc1, 0xfd, 0x2d, 0xde, 0x00, 0xeb, 0x00,
  0xda, 0x00, 0x00, 0x0c, 0xff, 0x0f, 0x00, 0x04, 0xc0, 0x00, 0x5b, 0xf5,
  0xc9, 0x01, 0x1e, 0xf2, 0x80, 0x00, 0x3f, 0xff, 0x19, 0xf4, 0x58, 0xf5,
  0x66, 0xf5, 0x64, 0xf5, 0xc0, 0xf1, 0xf0, 0x00, 0xe0, 0x00, 0xcd, 0x01,
  0xd3, 0x01, 0xdb, 0x01, 0xff, 0x7f, 0xff, 0x01, 0xe4, 0x00, 0x74, 0xf7,
  0xf3, 0x00, 0xfa, 0x00, 0xff, 0x3f, 0xca, 0x03, 0x6c, 0x38, 0x56, 0xfe,
  0x44, 0xfd, 0xbc, 0x02, 0xf9, 0x06, 0x00, 0xfc, 0x12, 0x02, 0xae, 0x01,
  0x58, 0xfa, 0x9a, 0xfd, 0x77, 0x05, 0xbb, 0x02, 0x96, 0x01, 0x95, 0x01,
  0x7f, 0x01, 0x82, 0x01, 0x89, 0x01, 0x87, 0x01, 0x88, 0x01, 0x8a, 0x01,
  0x8c, 0x01, 0x8f, 0x01, 0x8d, 0x01, 0x92, 0x01, 0x91, 0x01, 0xdd, 0x00,
  0x9f, 0x01, 0x7e, 0x01, 0xdb, 0x00, 0xb6, 0x01, 0x70, 0x69, 0x26, 0xd3,
  0x9c, 0x07, 0x1f, 0x05, 0x9d, 0x00, 0x00, 0x08, 0xbc, 0x05, 0x37, 0xfa,
  0xa2, 0x01, 0xaa, 0x01, 0xa1, 0x01, 0xa8, 0x01, 0xa0, 0x01, 0xa8, 0x05,
  0xb4, 0x01, 0xb4, 0x01, 0xce, 0x00, 0xd0, 0x00, 0xfc, 0x00, 0xc5, 0x01,
  0xff, 0xfb, 0xb1, 0x00, 0x00, 0x38, 0x00, 0x30, 0xfd, 0xf5, 0xfc, 0xf5,
  0xcd, 0x01, 0xa0, 0x00, 0x5f, 0xff, 0x00, 0x40, 0xff, 0x00, 0x00, 0x80,
  0x6d, 0x0f, 0xeb, 0x00, 0x7f, 0xff, 0xc2, 0xf5, 0x68, 0xf7, 0xb3, 0xf1,
  0x67, 0x0f, 0x5b, 0x0f, 0x61, 0x0f, 0x80, 0x0f, 0x58, 0xf7, 0x5b, 0xf7,
  0x83, 0x0f, 0x86, 0x00, 0x72, 0x0f, 0x85, 0x0f, 0xc6, 0xf1, 0x7f, 0x0f,
  0x6c, 0xf7, 0x00, 0xe0, 0x00, 0xff, 0xd1, 0xf5, 0x87, 0x0f, 0x8a, 0x0f,
  0xff, 0x03, 0xf0, 0x3f, 0x8b, 0x00, 0x8e, 0x00, 0x90, 0x00, 0xb9, 0x00,
  0x2d, 0xf5, 0xca, 0xf5, 0xcb, 0x01, 0x20, 0xf2, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x30, 0x50, 0x98, 0x2e, 0xd7, 0x0e, 0x50, 0x32, 0x98, 0x2e, 0xfa, 0x03,
  0x00, 0x30, 0xf0, 0x7f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x00, 0x2e,
  0x01, 0x80, 0x08, 0xa2, 0xfb, 0x2f, 0x98, 0x2e, 0xba, 0x03, 0x21, 0x2e,
  0x19, 0x00, 0x01, 0x2e, 0xee, 0x00, 0x00, 0xb2, 0x07, 0x2f, 0x01, 0x2e,
  0x19, 0x00, 0x00, 0xb2, 0x03, 0x2f, 0x01, 0x50, 0x03, 0x52, 0x98, 0x2e,
  0x07, 0xcc, 0x01, 0x2e, 0xdd, 0x00, 0x00, 0xb2, 0x27, 0x2f, 0x05, 0x2e,
  0x8a, 0x00, 0x05, 0x52, 0x98, 0x2e, 0xc7, 0xc1, 0x03, 0x2e, 0xe9, 0x00,
  0x40, 0xb2, 0xf0, 0x7f, 0x08, 0x2f, 0x01, 0x2e, 0x19, 0x00, 0x00, 0xb2,
  0x04, 0x2f, 0x00, 0x30, 0x21, 0x2e, 0xe9, 0x00, 0x98, 0x2e, 0xb4, 0xb1,
  0x01, 0x2e, 0x18, 0x00, 0x00, 0xb2, 0x10, 0x2f, 0x05, 0x50, 0x98, 0x2e,
  0x4d, 0xc3, 0x05, 0x50, 0x98, 0x2e, 0x5a, 0xc7, 0x98, 0x2e, 0xf9, 0xb4,
  0x98, 0x2e, 0x54, 0xb2, 0x98, 0x2e, 0x67, 0xb6, 0x98, 0x2e, 0x17, 0xb2,
  0x10, 0x30, 0x21, 0x2e, 0x77, 0x00, 0x01, 0x2e, 0xef, 0x00, 0x00, 0xb2,
  0x04, 0x2f, 0x98, 0x2e, 0x7a, 0xb7, 0x00, 0x30, 0x21, 0x2e, 0xef, 0x00,
  0x01, 0x2e, 0xd4, 0x00, 0x04, 0xae, 0x0b, 0x2f, 0x01, 0x2e, 0xdd, 0x00,
  0x00, 0xb2, 0x07, 0x2f, 0x05, 0x52, 0x98, 0x2e, 0x8e, 0x0e, 0x00, 0xb2,
  0x02, 0x2f, 0x10, 0x30, 0x21, 0x2e, 0x7d, 0x00, 0x01, 0x2e, 0x7d, 0x00,
  0x00, 0x90, 0x90, 0x2e, 0xf1, 0x02, 0x01, 0x2e, 0xd7, 0x00, 0x00, 0xb2,
  0x04, 0x2f, 0x98, 0x2e, 0x2f, 0x0e, 0x00, 0x30, 0x21, 0x2e, 0x7b, 0x00,
  0x01, 0x2e, 0x7b, 0x00, 0x00, 0xb2, 0x12, 0x2f, 0x01, 0x2e, 0xd4, 0x00,
  0x00, 0x90, 0x02, 0x2f, 0x98, 0x2e, 0x1f, 0x0e, 0x09, 0x2d, 0x98, 0x2e,
  0x81, 0x0d, 0x01, 0x2e, 0xd4, 0x00, 0x04, 0x90, 0x02, 0x2f, 0x50, 0x32,
  0x98, 0x2e, 0xfa, 0x03, 0x00, 0x30, 0x21, 0x2e, 0x7b, 0x00, 0x01, 0x2e,
  0x7c, 0x00, 0x00, 0xb2, 0x90, 0x2e, 0x09, 0x03, 0x01, 0x2e, 0x7c, 0x00,
  0x01, 0x31, 0x01, 0x08, 0x00, 0xb2, 0x04, 0x2f, 0x98, 0x2e, 0x47, 0xcb,
  0x10, 0x30, 0x21, 0x2e, 0x77, 0x00, 0x81, 0x30, 0x01, 0x2e, 0x7c, 0x00,
  0x01, 0x08, 0x00, 0xb2, 0x61, 0x2f, 0x03, 0x2e, 0x89, 0x00, 0x01, 0x2e,
  0xd4, 0x00, 0x98, 0xbc, 0x98, 0xb8, 0x05, 0xb2, 0x0f, 0x58, 0x23, 0x2f,
  0x07, 0x90, 0x09, 0x54, 0x00, 0x30, 0x37, 0x2f, 0x15, 0x41, 0x04, 0x41,
  0xdc, 0xbe, 0x44, 0xbe, 0xdc, 0xba, 0x2c, 0x01, 0x61, 0x00, 0x0f, 0x56,
  0x4a, 0x0f, 0x0c, 0x2f, 0xd1, 0x42, 0x94, 0xb8, 0xc1, 0x42, 0x11, 0x30,
  0x05, 0x2e, 0x6a, 0xf7, 0x2c, 0xbd, 0x2f, 0xb9, 0x80, 0xb2, 0x08, 0x22,
  0x98, 0x2e, 0xc3, 0xb7, 0x21, 0x2d, 0x61, 0x30, 0x23, 0x2e, 0xd4, 0x00,
  0x98, 0x2e, 0xc3, 0xb7, 0x00, 0x30, 0x21, 0x2e, 0x5a, 0xf5, 0x18, 0x2d,
  0xe1, 0x7f, 0x50, 0x30, 0x98, 0x2e, 0xfa, 0x03, 0x0f, 0x52, 0x07, 0x50,
  0x50, 0x42, 0x70, 0x30, 0x0d, 0x54, 0x42, 0x42, 0x7e, 0x82, 0xe2, 0x6f,
  0x80, 0xb2, 0x42, 0x42, 0x05, 0x2f, 0x21, 0x2e, 0xd4, 0x00, 0x10, 0x30,
  0x98, 0x2e, 0xc3, 0xb7, 0x03, 0x2d, 0x60, 0x30, 0x21, 0x2e, 0xd4, 0x00,
  0x01, 0x2e, 0xd4, 0x00, 0x06, 0x90, 0x18, 0x2f, 0x01, 0x2e, 0x76, 0x00,
  0x0b, 0x54, 0x07, 0x52, 0xe0, 0x7f, 0x98, 0x2e, 0x7a, 0xc1, 0xe1, 0x6f,
  0x08, 0x1a, 0x40, 0x30, 0x08, 0x2f, 0x21, 0x2e, 0xd4, 0x00, 0x20, 0x30,
  0x98, 0x2e, 0xaf, 0xb7, 0x50, 0x32, 0x98, 0x2e, 0xfa, 0x03, 0x05, 0x2d,
  0x98, 0x2e, 0x38, 0x0e, 0x00, 0x30, 0x21, 0x2e, 0xd4, 0x00, 0x00, 0x30,
  0x21, 0x2e, 0x7c, 0x00, 0x18, 0x2d, 0x01, 0x2e, 0xd4, 0x00, 0x03, 0xaa,
  0x01, 0x2f, 0x98, 0x2e, 0x45, 0x0e, 0x01, 0x2e, 0xd4, 0x00, 0x3f, 0x80,
  0x03, 0xa2, 0x01, 0x2f, 0x00, 0x2e, 0x02, 0x2d, 0x98, 0x2e, 0x5b, 0x0e,
  0x30, 0x30, 0x98, 0x2e, 0xce, 0xb7, 0x00, 0x30, 0x21, 0x2e, 0x7d, 0x00,
  0x50, 0x32, 0x98, 0x2e, 0xfa, 0x03, 0x01, 0x2e, 0x77, 0x00, 0x00, 0xb2,
  0x24, 0x2f, 0x98, 0x2e, 0xf5, 0xcb, 0x03, 0x2e, 0xd5, 0x00, 0x11, 0x54,
  0x01, 0x0a, 0xbc, 0x84, 0x83, 0x86, 0x21, 0x2e, 0xc9, 0x01, 0xe0, 0x40,
  0x13, 0x52, 0xc4, 0x40, 0x82, 0x40, 0xa8, 0xb9, 0x52, 0x42, 0x43, 0xbe,
  0x53, 0x42, 0x04, 0x0a, 0x50, 0x42, 0xe1, 0x7f, 0xf0, 0x31, 0x41, 0x40,
  0xf2, 0x6f, 0x25, 0xbd, 0x08, 0x08, 0x02, 0x0a, 0xd0, 0x7f, 0x98, 0x2e,
  0xa8, 0xcf, 0x06, 0xbc, 0xd1, 0x6f, 0xe2, 0x6f, 0x08, 0x0a, 0x80, 0x42,
  0x98, 0x2e, 0x58, 0xb7, 0x00, 0x30, 0x21, 0x2e, 0xee, 0x00, 0x21, 0x2e,
  0x77, 0x00, 0x21, 0x2e, 0xdd, 0x00, 0x80, 0x2e, 0xf4, 0x01, 0x1a, 0x24,
  0x22, 0x00, 0x80, 0x2e, 0xec, 0x01, 0x10, 0x50, 0xfb, 0x7f, 0x98, 0x2e,
  0xf3, 0x03, 0x57, 0x50, 0xfb, 0x6f, 0x01, 0x30, 0x71, 0x54, 0x11, 0x42,
  0x42, 0x0e, 0xfc, 0x2f, 0xc0, 0x2e, 0x01, 0x42, 0xf0, 0x5f, 0x80, 0x2e,
  0x00, 0xc1, 0xfd, 0x2d, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x9a, 0x01, 0x34, 0x03, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x20, 0x50, 0xe7, 0x7f, 0xf6, 0x7f, 0x06, 0x32, 0x0f, 0x2e,
  0x61, 0xf5, 0xfe, 0x09, 0xc0, 0xb3, 0x04, 0x2f, 0x17, 0x30, 0x2f, 0x2e,
  0xef, 0x00, 0x2d, 0x2e, 0x61, 0xf5, 0xf6, 0x6f, 0xe7, 0x6f, 0xe0, 0x5f,
  0xc8, 0x2e, 0x20, 0x50, 0xe7, 0x7f, 0xf6, 0x7f, 0x46, 0x30, 0x0f, 0x2e,
  0xa4, 0xf1, 0xbe, 0x09, 0x80, 0xb3, 0x06, 0x2f, 0x0d, 0x2e, 0xd4, 0x00,
  0x84, 0xaf, 0x02, 0x2f, 0x16, 0x30, 0x2d, 0x2e, 0x7b, 0x00, 0x86, 0x30,
  0x2d, 0x2e, 0x60, 0xf5, 0xf6, 0x6f, 0xe7, 0x6f, 0xe0, 0x5f, 0xc8, 0x2e,
  0x01, 0x2e, 0x77, 0xf7, 0x09, 0xbc, 0x0f, 0xb8, 0x00, 0xb2, 0x10, 0x50,
  0xfb, 0x7f, 0x10, 0x30, 0x0b, 0x2f, 0x03, 0x2e, 0x8a, 0x00, 0x96, 0xbc,
  0x9f, 0xb8, 0x40, 0xb2, 0x05, 0x2f, 0x03, 0x2e, 0x68, 0xf7, 0x9e, 0xbc,
  0x9f, 0xb8, 0x40, 0xb2, 0x07, 0x2f, 0x03, 0x2e, 0x7e, 0x00, 0x41, 0x90,
  0x01, 0x2f, 0x98, 0x2e, 0xdc, 0x03, 0x03, 0x2c, 0x00, 0x30, 0x21, 0x2e,
  0x7e, 0x00, 0xfb, 0x6f, 0xf0, 0x5f, 0xb8, 0x2e, 0x20, 0x50, 0xe0, 0x7f,
  0xfb, 0x7f, 0x00, 0x2e, 0x27, 0x50, 0x98, 0x2e, 0x3b, 0xc8, 0x29, 0x50,
  0x98, 0x2e, 0xa7, 0xc8, 0x01, 0x50, 0x98, 0x2e, 0x55, 0xcc, 0xe1, 0x6f,
  0x2b, 0x50, 0x98, 0x2e, 0xe0, 0xc9, 0xfb, 0x6f, 0x00, 0x30, 0xe0, 0x5f,
  0x21, 0x2e, 0x7e, 0x00, 0xb8, 0x2e, 0x73, 0x50, 0x01, 0x30, 0x57, 0x54,
  0x11, 0x42, 0x42, 0x0e, 0xfc, 0x2f, 0xb8, 0x2e, 0x21, 0x2e, 0x59, 0xf5,
  0x10, 0x30, 0xc0, 0x2e, 0x21, 0x2e, 0x4a, 0xf1, 0x90, 0x50, 0xf7, 0x7f,
  0xe6, 0x7f, 0xd5, 0x7f, 0xc4, 0x7f, 0xb3, 0x7f, 0xa1, 0x7f, 0x90, 0x7f,
  0x82, 0x7f, 0x7b, 0x7f, 0x98, 0x2e, 0x35, 0xb7, 0x00, 0xb2, 0x90, 0x2e,
  0x97, 0xb0, 0x03, 0x2e, 0x8f, 0x00, 0x07, 0x2e, 0x91, 0x00, 0x05, 0x2e,
  0xb1, 0x00, 0x3f, 0xba, 0x9f, 0xb8, 0x01, 0x2e, 0xb1, 0x00, 0xa3, 0xbd,
  0x4c, 0x0a, 0x05, 0x2e, 0xb1, 0x00, 0x04, 0xbe, 0xbf, 0xb9, 0xcb, 0x0a,
  0x4f, 0xba, 0x22, 0xbd, 0x01, 0x2e, 0xb3, 0x00, 0xdc, 0x0a, 0x2f, 0xb9,
  0x03, 0x2e, 0xb8, 0x00, 0x0a, 0xbe, 0x9a, 0x0a, 0xcf, 0xb9, 0x9b, 0xbc,
  0x01, 0x2e, 0x97, 0x00, 0x9f, 0xb8, 0x93, 0x0a, 0x0f, 0xbc, 0x91, 0x0a,
  0x0f, 0xb8, 0x90, 0x0a, 0x25, 0x2e, 0x18, 0x00, 0x05, 0x2e, 0xc1, 0xf5,
  0x2e, 0xbd, 0x2e, 0xb9, 0x01, 0x2e, 0x19, 0x00, 0x31, 0x30, 0x8a, 0x04,
  0x00, 0x90, 0x07, 0x2f, 0x01, 0x2e, 0xd4, 0x00, 0x04, 0xa2, 0x03, 0x2f,
  0x01, 0x2e, 0x18, 0x00, 0x00, 0xb2, 0x0c, 0x2f, 0x19, 0x50, 0x05, 0x52,
  0x98, 0x2e, 0x4d, 0xb7, 0x05, 0x2e, 0x78, 0x00, 0x80, 0x90, 0x10, 0x30,
  0x01, 0x2f, 0x21, 0x2e, 0x78, 0x00, 0x25, 0x2e, 0xdd, 0x00, 0x98, 0x2e,
  0x3e, 0xb7, 0x00, 0xb2, 0x02, 0x30, 0x01, 0x30, 0x04, 0x2f, 0x01, 0x2e,
  0x19, 0x00, 0x00, 0xb2, 0x00, 0x2f, 0x21, 0x30, 0x01, 0x2e, 0xea, 0x00,
  0x08, 0x1a, 0x0e, 0x2f, 0x23, 0x2e, 0xea, 0x00, 0x33, 0x30, 0x1b, 0x50,
  0x0b, 0x09, 0x01, 0x40, 0x17, 0x56, 0x46, 0xbe, 0x4b, 0x08, 0x4c, 0x0a,
  0x01, 0x42, 0x0a, 0x80, 0x15, 0x52, 0x01, 0x42, 0x00, 0x2e, 0x01, 0x2e,
  0x18, 0x00, 0x00, 0xb2, 0x1f, 0x2f, 0x03, 0x2e, 0xc0, 0xf5, 0xf0, 0x30,
  0x48, 0x08, 0x47, 0xaa, 0x74, 0x30, 0x07, 0x2e, 0x7a, 0x00, 0x61, 0x22,
  0x4b, 0x1a, 0x05, 0x2f, 0x07, 0x2e, 0x66, 0xf5, 0xbf, 0xbd, 0xbf, 0xb9,
  0xc0, 0x90, 0x0b, 0x2f, 0x1d, 0x56, 0x2b, 0x30, 0xd2, 0x42, 0xdb, 0x42,
  0x01, 0x04, 0xc2, 0x42, 0x04, 0xbd, 0xfe, 0x80, 0x81, 0x84, 0x23, 0x2e,
  0x7a, 0x00, 0x02, 0x42, 0x02, 0x32, 0x25, 0x2e, 0x62, 0xf5, 0x05, 0x2e,
  0xd6, 0x00, 0x81, 0x84, 0x25, 0x2e, 0xd6, 0x00, 0x02, 0x31, 0x25, 0x2e,
  0x60, 0xf5, 0x05, 0x2e, 0x8a, 0x00, 0x0b, 0x50, 0x90, 0x08, 0x80, 0xb2,
  0x0b, 0x2f, 0x05, 0x2e, 0xca, 0xf5, 0xf0, 0x3e, 0x90, 0x08, 0x25, 0x2e,
  0xca, 0xf5, 0x05, 0x2e, 0x59, 0xf5, 0xe0, 0x3f, 0x90, 0x08, 0x25, 0x2e,
  0x59, 0xf5, 0x90, 0x6f, 0xa1, 0x6f, 0xb3, 0x6f, 0xc4, 0x6f, 0xd5, 0x6f,
  0xe6, 0x6f, 0xf7, 0x6f, 0x7b, 0x6f, 0x82, 0x6f, 0x70, 0x5f, 0xc8, 0x2e,
  0xc0, 0x50, 0x90, 0x7f, 0xe5, 0x7f, 0xd4, 0x7f, 0xc3, 0x7f, 0xb1, 0x7f,
  0xa2, 0x7f, 0x87, 0x7f, 0xf6, 0x7f, 0x7b, 0x7f, 0x00, 0x2e, 0x01, 0x2e,
  0x60, 0xf5, 0x60, 0x7f, 0x98, 0x2e, 0x35, 0xb7, 0x02, 0x30, 0x63, 0x6f,
  0x15, 0x52, 0x50, 0x7f, 0x62, 0x7f, 0x5a, 0x2c, 0x02, 0x32, 0x1a, 0x09,
  0x00, 0xb3, 0x14, 0x2f, 0x00, 0xb2, 0x03, 0x2f, 0x09, 0x2e, 0x18, 0x00,
  0x00, 0x91, 0x0c, 0x2f, 0x43, 0x7f, 0x98, 0x2e, 0x97, 0xb7, 0x1f, 0x50,
  0x02, 0x8a, 0x02, 0x32, 0x04, 0x30, 0x25, 0x2e, 0x64, 0xf5, 0x15, 0x52,
  0x50, 0x6f, 0x43, 0x6f, 0x44, 0x43, 0x25, 0x2e, 0x60, 0xf5, 0xd9, 0x08,
  0xc0, 0xb2, 0x36, 0x2f, 0x98, 0x2e, 0x3e, 0xb7, 0x00, 0xb2, 0x06, 0x2f,
  0x01, 0x2e, 0x19, 0x00, 0x00, 0xb2, 0x02, 0x2f, 0x50, 0x6f, 0x00, 0x90,
  0x0a, 0x2f, 0x01, 0x2e, 0x79, 0x00, 0x00, 0x90, 0x19, 0x2f, 0x10, 0x30,
  0x21, 0x2e, 0x79, 0x00, 0x00, 0x30, 0x98, 0x2e, 0xdc, 0x03, 0x13, 0x2d,
  0x01, 0x2e, 0xc3, 0xf5, 0x0c, 0xbc, 0x0f, 0xb8, 0x12, 0x30, 0x10, 0x04,
  0x03, 0xb0, 0x26, 0x25, 0x21, 0x50, 0x03, 0x52, 0x98, 0x2e, 0x4d, 0xb7,
  0x10, 0x30, 0x21, 0x2e, 0xee, 0x00, 0x02, 0x30, 0x60, 0x7f, 0x25, 0x2e,
  0x79, 0x00, 0x60, 0x6f, 0x00, 0x90, 0x05, 0x2f, 0x00, 0x30, 0x21, 0x2e,
  0xea, 0x00, 0x15, 0x50, 0x21, 0x2e, 0x64, 0xf5, 0x15, 0x52, 0x23, 0x2e,
  0x60, 0xf5, 0x02, 0x32, 0x50, 0x6f, 0x00, 0x90, 0x02, 0x2f, 0x03, 0x30,
  0x27, 0x2e, 0x78, 0x00, 0x07, 0x2e, 0x60, 0xf5, 0x1a, 0x09, 0x00, 0x91,
  0xa3, 0x2f, 0x19, 0x09, 0x00, 0x91, 0xa0, 0x2f, 0x90, 0x6f, 0xa2, 0x6f,
  0xb1, 0x6f, 0xc3, 0x6f, 0xd4, 0x6f, 0xe5, 0x6f, 0x7b, 0x6f, 0xf6, 0x6f,
  0x87, 0x6f, 0x40, 0x5f, 0xc8, 0x2e, 0xc0, 0x50, 0xe7, 0x7f, 0xf6, 0x7f,
  0x26, 0x30, 0x0f, 0x2e, 0x61, 0xf5, 0x2f, 0x2e, 0x7c, 0x00, 0x0f, 0x2e,
  0x7c, 0x00, 0xbe, 0x09, 0xa2, 0x7f, 0x80, 0x7f, 0x80, 0xb3, 0xd5, 0x7f,
  0xc4, 0x7f, 0xb3, 0x7f, 0x91, 0x7f, 0x7b, 0x7f, 0x0b, 0x2f, 0x23, 0x50,
  0x1a, 0x25, 0x12, 0x40, 0x42, 0x7f, 0x74, 0x82, 0x12, 0x40, 0x52, 0x7f,
  0x00, 0x2e, 0x00, 0x40, 0x60, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x81, 0x30,
  0x01, 0x2e, 0x7c, 0x00, 0x01, 0x08, 0x00, 0xb2, 0x42, 0x2f, 0x03, 0x2e,
  0x89, 0x00, 0x01, 0x2e, 0x89, 0x00, 0x97, 0xbc, 0x06, 0xbc, 0x9f, 0xb8,
  0x0f, 0xb8, 0x00, 0x90, 0x23, 0x2e, 0xd8, 0x00, 0x10, 0x30, 0x01, 0x30,
  0x2a, 0x2f, 0x03, 0x2e, 0xd4, 0x00, 0x44, 0xb2, 0x05, 0x2f, 0x47, 0xb2,
  0x00, 0x30, 0x2d, 0x2f, 0x21, 0x2e, 0x7c, 0x00, 0x2b, 0x2d, 0x03, 0x2e,
  0xfd, 0xf5, 0x9e, 0xbc, 0x9f, 0xb8, 0x40, 0x90, 0x14, 0x2f, 0x03, 0x2e,
  0xfc, 0xf5, 0x99, 0xbc, 0x9f, 0xb8, 0x40, 0x90, 0x0e, 0x2f, 0x03, 0x2e,
  0x49, 0xf1, 0x25, 0x54, 0x4a, 0x08, 0x40, 0x90, 0x08, 0x2f, 0x98, 0x2e,
  0x35, 0xb7, 0x00, 0xb2, 0x10, 0x30, 0x03, 0x2f, 0x50, 0x30, 0x21, 0x2e,
  0xd4, 0x00, 0x10, 0x2d, 0x98, 0x2e, 0xaf, 0xb7, 0x00, 0x30, 0x21, 0x2e,
  0x7c, 0x00, 0x0a, 0x2d, 0x05, 0x2e, 0x69, 0xf7, 0x2d, 0xbd, 0x2f, 0xb9,
  0x80, 0xb2, 0x01, 0x2f, 0x21, 0x2e, 0x7d, 0x00, 0x23, 0x2e, 0x7c, 0x00,
  0xe0, 0x31, 0x21, 0x2e, 0x61, 0xf5, 0xf6, 0x6f, 0xe7, 0x6f, 0x80, 0x6f,
  0xa2, 0x6f, 0xb3, 0x6f, 0xc4, 0x6f, 0xd5, 0x6f, 0x7b, 0x6f, 0x91, 0x6f,
  0x40, 0x5f, 0xc8, 0x2e, 0x60, 0x51, 0x0a, 0x25, 0x36, 0x88, 0xf4, 0x7f,
  0xeb, 0x7f, 0x00, 0x32, 0x31, 0x52, 0x32, 0x30, 0x13, 0x30, 0x98, 0x2e,
  0x15, 0xcb, 0x0a, 0x25, 0x33, 0x84, 0xd2, 0x7f, 0x43, 0x30, 0x05, 0x50,
  0x2d, 0x52, 0x98, 0x2e, 0x95, 0xc1, 0xd2, 0x6f, 0x27, 0x52, 0x98, 0x2e,
  0xd7, 0xc7, 0x2a, 0x25, 0xb0, 0x86, 0xc0, 0x7f, 0xd3, 0x7f, 0xaf, 0x84,
  0x29, 0x50, 0xf1, 0x6f, 0x98, 0x2e, 0x4d, 0xc8, 0x2a, 0x25, 0xae, 0x8a,
  0xaa, 0x88, 0xf2, 0x6e, 0x2b, 0x50, 0xc1, 0x6f, 0xd3, 0x6f, 0xf4, 0x7f,
  0x98, 0x2e, 0xb6, 0xc8, 0xe0, 0x6e, 0x00, 0xb2, 0x32, 0x2f, 0x33, 0x54,
  0x83, 0x86, 0xf1, 0x6f, 0xc3, 0x7f, 0x04, 0x30, 0x30, 0x30, 0xf4, 0x7f,
  0xd0, 0x7f, 0xb2, 0x7f, 0xe3, 0x30, 0xc5, 0x6f, 0x56, 0x40, 0x45, 0x41,
  0x28, 0x08, 0x03, 0x14, 0x0e, 0xb4, 0x08, 0xbc, 0x82, 0x40, 0x10, 0x0a,
  0x2f, 0x54, 0x26, 0x05, 0x91, 0x7f, 0x44, 0x28, 0xa3, 0x7f, 0x98, 0x2e,
  0xd9, 0xc0, 0x08, 0xb9, 0x33, 0x30, 0x53, 0x09, 0xc1, 0x6f, 0xd3, 0x6f,
  0xf4, 0x6f, 0x83, 0x17, 0x47, 0x40, 0x6c, 0x15, 0xb2, 0x6f, 0xbe, 0x09,
  0x75, 0x0b, 0x90, 0x42, 0x45, 0x42, 0x51, 0x0e, 0x32, 0xbc, 0x02, 0x89,
  0xa1, 0x6f, 0x7e, 0x86, 0xf4, 0x7f, 0xd0, 0x7f, 0xb2, 0x7f, 0x04, 0x30,
  0x91, 0x6f, 0xd6, 0x2f, 0xeb, 0x6f, 0xa0, 0x5e, 0xb8, 0x2e, 0x03, 0x2e,
  0x97, 0x00, 0x1b, 0xbc, 0x60, 0x50, 0x9f, 0xbc, 0x0c, 0xb8, 0xf0, 0x7f,
  0x40, 0xb2, 0xeb, 0x7f, 0x2b, 0x2f, 0x03, 0x2e, 0x7f, 0x00, 0x41, 0x40,
  0x01, 0x2e, 0xc8, 0x00, 0x01, 0x1a, 0x11, 0x2f, 0x37, 0x58, 0x23, 0x2e,
  0xc8, 0x00, 0x10, 0x41, 0xa0, 0x7f, 0x38, 0x81, 0x01, 0x41, 0xd0, 0x7f,
  0xb1, 0x7f, 0x98, 0x2e, 0x64, 0xcf, 0xd0, 0x6f, 0x07, 0x80, 0xa1, 0x6f,
  0x11, 0x42, 0x00, 0x2e, 0xb1, 0x6f, 0x01, 0x42, 0x11, 0x30, 0x01, 0x2e,
  0xfc, 0x00, 0x00, 0xa8, 0x03, 0x30, 0xcb, 0x22, 0x4a, 0x25, 0x01, 0x2e,
  0x7f, 0x00, 0x3c, 0x89, 0x35, 0x52, 0x05, 0x54, 0x98, 0x2e, 0xc4, 0xce,
  0xc1, 0x6f, 0xf0, 0x6f, 0x98, 0x2e, 0x95, 0xcf, 0x04, 0x2d, 0x01, 0x30,
  0xf0, 0x6f, 0x98, 0x2e, 0x95, 0xcf, 0xeb, 0x6f, 0xa0, 0x5f, 0xb8, 0x2e,
  0x03, 0x2e, 0xb3, 0x00, 0x02, 0x32, 0xf0, 0x30, 0x03, 0x31, 0x30, 0x50,
  0x8a, 0x08, 0x08, 0x08, 0xcb, 0x08, 0xe0, 0x7f, 0x80, 0xb2, 0xf3, 0x7f,
  0xdb, 0x7f, 0x25, 0x2f, 0x03, 0x2e, 0xca, 0x00, 0x41, 0x90, 0x04, 0x2f,
  0x01, 0x30, 0x23, 0x2e, 0xca, 0x00, 0x98, 0x2e, 0x3f, 0x03, 0xc0, 0xb2,
  0x05, 0x2f, 0x03, 0x2e, 0xda, 0x00, 0x00, 0x30, 0x41, 0x04, 0x23, 0x2e,
  0xda, 0x00, 0x98, 0x2e, 0x92, 0xb2, 0x10, 0x25, 0xf0, 0x6f, 0x00, 0xb2,
  0x05, 0x2f, 0x01, 0x2e, 0xda, 0x00, 0x02, 0x30, 0x10, 0x04, 0x21, 0x2e,
  0xda, 0x00, 0x40, 0xb2, 0x01, 0x2f, 0x23, 0x2e, 0xc8, 0x01, 0xdb, 0x6f,
  0xe0, 0x6f, 0xd0, 0x5f, 0x80, 0x2e, 0x95, 0xcf, 0x01, 0x30, 0xe0, 0x6f,
  0x98, 0x2e, 0x95, 0xcf, 0x11, 0x30, 0x23, 0x2e, 0xca, 0x00, 0xdb, 0x6f,
  0xd0, 0x5f, 0xb8, 0x2e, 0xd0, 0x50, 0x0a, 0x25, 0x33, 0x84, 0x55, 0x50,
  0xd2, 0x7f, 0xe2, 0x7f, 0x03, 0x8c, 0xc0, 0x7f, 0xbb, 0x7f, 0x00, 0x30,
  0x05, 0x5a, 0x39, 0x54, 0x51, 0x41, 0xa5, 0x7f, 0x96, 0x7f, 0x80, 0x7f,
  0x98, 0x2e, 0xd9, 0xc0, 0x05, 0x30, 0xf5, 0x7f, 0x20, 0x25, 0x91, 0x6f,
  0x3b, 0x58, 0x3d, 0x5c, 0x3b, 0x56, 0x98, 0x2e, 0x67, 0xcc, 0xc1, 0x6f,
  0xd5, 0x6f, 0x52, 0x40, 0x50, 0x43, 0xc1, 0x7f, 0xd5, 0x7f, 0x10, 0x25,
  0x98, 0x2e, 0xfe, 0xc9, 0x10, 0x25, 0x98, 0x2e, 0x74, 0xc0, 0x86, 0x6f,
  0x30, 0x28, 0x92, 0x6f, 0x82, 0x8c, 0xa5, 0x6f, 0x6f, 0x52, 0x69, 0x0e,
  0x39, 0x54, 0xdb, 0x2f, 0x19, 0xa0, 0x15, 0x30, 0x03, 0x2f, 0x00, 0x30,
  0x21, 0x2e, 0x81, 0x01, 0x0a, 0x2d, 0x01, 0x2e, 0x81, 0x01, 0x05, 0x28,
  0x42, 0x36, 0x21, 0x2e, 0x81, 0x01, 0x02, 0x0e, 0x01, 0x2f, 0x98, 0x2e,
  0xf3, 0x03, 0x57, 0x50, 0x12, 0x30, 0x01, 0x40, 0x98, 0x2e, 0xfe, 0xc9,
  0x51, 0x6f, 0x0b, 0x5c, 0x8e, 0x0e, 0x3b, 0x6f, 0x57, 0x58, 0x02, 0x30,
  0x21, 0x2e, 0x95, 0x01, 0x45, 0x6f, 0x2a, 0x8d, 0xd2, 0x7f, 0xcb, 0x7f,
  0x13, 0x2f, 0x02, 0x30, 0x3f, 0x50, 0xd2, 0x7f, 0xa8, 0x0e, 0x0e, 0x2f,
  0xc0, 0x6f, 0x53, 0x54, 0x02, 0x00, 0x51, 0x54, 0x42, 0x0e, 0x10, 0x30,
  0x59, 0x52, 0x02, 0x30, 0x01, 0x2f, 0x00, 0x2e, 0x03, 0x2d, 0x50, 0x42,
  0x42, 0x42, 0x12, 0x30, 0xd2, 0x7f, 0x80, 0xb2, 0x03, 0x2f, 0x00, 0x30,
  0x21, 0x2e, 0x80, 0x01, 0x12, 0x2d, 0x01, 0x2e, 0xc9, 0x00, 0x02, 0x80,
  0x05, 0x2e, 0x80, 0x01, 0x11, 0x30, 0x91, 0x28, 0x00, 0x40, 0x25, 0x2e,
  0x80, 0x01, 0x10, 0x0e, 0x05, 0x2f, 0x01, 0x2e, 0x7f, 0x01, 0x01, 0x90,
  0x01, 0x2f, 0x98, 0x2e, 0xf3, 0x03, 0x00, 0x2e, 0xa0, 0x41, 0x01, 0x90,
  0xa6, 0x7f, 0x90, 0x2e, 0xe3, 0xb4, 0x01, 0x2e, 0x95, 0x01, 0x00, 0xa8,
  0x90, 0x2e, 0xe3, 0xb4, 0x5b, 0x54, 0x95, 0x80, 0x82, 0x40, 0x80, 0xb2,
  0x02, 0x40, 0x2d, 0x8c, 0x3f, 0x52, 0x96, 0x7f, 0x90, 0x2e, 0xc2, 0xb3,
  0x29, 0x0e, 0x76, 0x2f, 0x01, 0x2e, 0xc9, 0x00, 0x00, 0x40, 0x81, 0x28,
  0x45, 0x52, 0xb3, 0x30, 0x98, 0x2e, 0x0f, 0xca, 0x5d, 0x54, 0x80, 0x7f,
  0x00, 0x2e, 0xa1, 0x40, 0x72, 0x7f, 0x82, 0x80, 0x82, 0x40, 0x60, 0x7f,
  0x98, 0x2e, 0xfe, 0xc9, 0x10, 0x25, 0x98, 0x2e, 0x74, 0xc0, 0x62, 0x6f,
  0x05, 0x30, 0x87, 0x40, 0xc0, 0x91, 0x04, 0x30, 0x05, 0x2f, 0x05, 0x2e,
  0x83, 0x01, 0x80, 0xb2, 0x14, 0x30, 0x00, 0x2f, 0x04, 0x30, 0x05, 0x2e,
  0xc9, 0x00, 0x73, 0x6f, 0x81, 0x40, 0xe2, 0x40, 0x69, 0x04, 0x11, 0x0f,
  0xe1, 0x40, 0x16, 0x30, 0xfe, 0x29, 0xcb, 0x40, 0x02, 0x2f, 0x83, 0x6f,
  0x83, 0x0f, 0x22, 0x2f, 0x47, 0x56, 0x13, 0x0f, 0x12, 0x30, 0x77, 0x2f,
  0x49, 0x54, 0x42, 0x0e, 0x12, 0x30, 0x73, 0x2f, 0x00, 0x91, 0x0a, 0x2f,
  0x01, 0x2e, 0x8b, 0x01, 0x19, 0xa8, 0x02, 0x30, 0x6c, 0x2f, 0x63, 0x50,
  0x00, 0x2e, 0x17, 0x42, 0x05, 0x42, 0x68, 0x2c, 0x12, 0x30, 0x0b, 0x25,
  0x08, 0x0f, 0x50, 0x30, 0x02, 0x2f, 0x21, 0x2e, 0x83, 0x01, 0x03, 0x2d,
  0x40, 0x30, 0x21, 0x2e, 0x83, 0x01, 0x2b, 0x2e, 0x85, 0x01, 0x5a, 0x2c,
  0x12, 0x30, 0x00, 0x91, 0x2b, 0x25, 0x04, 0x2f, 0x63, 0x50, 0x02, 0x30,
  0x17, 0x42, 0x17, 0x2c, 0x02, 0x42, 0x98, 0x2e, 0xfe, 0xc9, 0x10, 0x25,
  0x98, 0x2e, 0x74, 0xc0, 0x05, 0x2e, 0xc9, 0x00, 0x81, 0x84, 0x5b, 0x30,
  0x82, 0x40, 0x37, 0x2e, 0x83, 0x01, 0x02, 0x0e, 0x07, 0x2f, 0x5f, 0x52,
  0x40, 0x30, 0x62, 0x40, 0x41, 0x40, 0x91, 0x0e, 0x01, 0x2f, 0x21, 0x2e,
  0x83, 0x01, 0x05, 0x30, 0x2b, 0x2e, 0x85, 0x01, 0x12, 0x30, 0x36, 0x2c,
  0x16, 0x30, 0x15, 0x25, 0x81, 0x7f, 0x98, 0x2e, 0xfe, 0xc9, 0x10, 0x25,
  0x98, 0x2e, 0x74, 0xc0, 0x19, 0xa2, 0x16, 0x30, 0x15, 0x2f, 0x05, 0x2e,
  0x97, 0x01, 0x80, 0x6f, 0x82, 0x0e, 0x05, 0x2f, 0x01, 0x2e, 0x86, 0x01,
  0x06, 0x28, 0x21, 0x2e, 0x86, 0x01, 0x0b, 0x2d, 0x03, 0x2e, 0x87, 0x01,
  0x5f, 0x54, 0x4e, 0x28, 0x91, 0x42, 0x00, 0x2e, 0x82, 0x40, 0x90, 0x0e,
  0x01, 0x2f, 0x21, 0x2e, 0x88, 0x01, 0x02, 0x30, 0x13, 0x2c, 0x05, 0x30,
  0xc0, 0x6f, 0x08, 0x1c, 0xa8, 0x0f, 0x16, 0x30, 0x05, 0x30, 0x5b, 0x50,
  0x09, 0x2f, 0x02, 0x80, 0x2d, 0x2e, 0x82, 0x01, 0x05, 0x42, 0x05, 0x80,
  0x00, 0x2e, 0x02, 0x42, 0x3e, 0x80, 0x00, 0x2e, 0x06, 0x42, 0x02, 0x30,
  0x90, 0x6f, 0x3e, 0x88, 0x01, 0x40, 0x04, 0x41, 0x4c, 0x28, 0x01, 0x42,
  0x07, 0x80, 0x10, 0x25, 0x24, 0x40, 0x00, 0x40, 0x00, 0xa8, 0xf5, 0x22,
  0x23, 0x29, 0x44, 0x42, 0x7a, 0x82, 0x7e, 0x88, 0x43, 0x40, 0x04, 0x41,
  0x00, 0xab, 0xf5, 0x23, 0xdf, 0x28, 0x43, 0x42, 0xd9, 0xa0, 0x14, 0x2f,
  0x00, 0x90, 0x02, 0x2f, 0xd2, 0x6f, 0x81, 0xb2, 0x05, 0x2f, 0x63, 0x54,
  0x06, 0x28, 0x90, 0x42, 0x85, 0x42, 0x09, 0x2c, 0x02, 0x30, 0x5b, 0x50,
  0x03, 0x80, 0x29, 0x2e, 0x7e, 0x01, 0x2b, 0x2e, 0x82, 0x01, 0x05, 0x42,
  0x12, 0x30, 0x2b, 0x2e, 0x83, 0x01, 0x45, 0x82, 0x00, 0x2e, 0x40, 0x40,
  0x7a, 0x82, 0x02, 0xa0, 0x08, 0x2f, 0x63, 0x50, 0x3b, 0x30, 0x15, 0x42,
  0x05, 0x42, 0x37, 0x80, 0x37, 0x2e, 0x7e, 0x01, 0x05, 0x42, 0x12, 0x30,
  0x01, 0x2e, 0xc9, 0x00, 0x02, 0x8c, 0x40, 0x40, 0x84, 0x41, 0x7a, 0x8c,
  0x04, 0x0f, 0x03, 0x2f, 0x01, 0x2e, 0x8b, 0x01, 0x19, 0xa4, 0x04, 0x2f,
  0x2b, 0x2e, 0x82, 0x01, 0x98, 0x2e, 0xf3, 0x03, 0x12, 0x30, 0x81, 0x90,
  0x61, 0x52, 0x08, 0x2f, 0x65, 0x42, 0x65, 0x42, 0x43, 0x80, 0x39, 0x84,
  0x82, 0x88, 0x05, 0x42, 0x45, 0x42, 0x85, 0x42, 0x05, 0x43, 0x00, 0x2e,
  0x80, 0x41, 0x00, 0x90, 0x90, 0x2e, 0xe1, 0xb4, 0x65, 0x54, 0xc1, 0x6f,
  0x80, 0x40, 0x00, 0xb2, 0x43, 0x58, 0x69, 0x50, 0x44, 0x2f, 0x55, 0x5c,
  0xb7, 0x87, 0x8c, 0x0f, 0x0d, 0x2e, 0x96, 0x01, 0xc4, 0x40, 0x36, 0x2f,
  0x41, 0x56, 0x8b, 0x0e, 0x2a, 0x2f, 0x0b, 0x52, 0xa1, 0x0e, 0x0a, 0x2f,
  0x05, 0x2e, 0x8f, 0x01, 0x14, 0x25, 0x98, 0x2e, 0xfe, 0xc9, 0x4b, 0x54,
  0x02, 0x0f, 0x69, 0x50, 0x05, 0x30, 0x65, 0x54, 0x15, 0x2f, 0x03, 0x2e,
  0x8e, 0x01, 0x4d, 0x5c, 0x8e, 0x0f, 0x3a, 0x2f, 0x05, 0x2e, 0x8f, 0x01,
  0x98, 0x2e, 0xfe, 0xc9, 0x4f, 0x54, 0x82, 0x0f, 0x05, 0x30, 0x69, 0x50,
  0x65, 0x54, 0x30, 0x2f, 0x6d, 0x52, 0x15, 0x30, 0x42, 0x8c, 0x45, 0x42,
  0x04, 0x30, 0x2b, 0x2c, 0x84, 0x43, 0x6b, 0x52, 0x42, 0x8c, 0x00, 0x2e,
  0x85, 0x43, 0x15, 0x30, 0x24, 0x2c, 0x45, 0x42, 0x8e, 0x0f, 0x20, 0x2f,
  0x0d, 0x2e, 0x8e, 0x01, 0xb1, 0x0e, 0x1c, 0x2f, 0x23, 0x2e, 0x8e, 0x01,
  0x1a, 0x2d, 0x0e, 0x0e, 0x17, 0x2f, 0xa1, 0x0f, 0x15, 0x2f, 0x23, 0x2e,
  0x8d, 0x01, 0x13, 0x2d, 0x98, 0x2e, 0x74, 0xc0, 0x43, 0x54, 0xc2, 0x0e,
  0x0a, 0x2f, 0x65, 0x50, 0x04, 0x80, 0x0b, 0x30, 0x06, 0x82, 0x0b, 0x42,
  0x79, 0x80, 0x41, 0x40, 0x12, 0x30, 0x25, 0x2e, 0x8c, 0x01, 0x01, 0x42,
  0x05, 0x30, 0x69, 0x50, 0x65, 0x54, 0x84, 0x82, 0x43, 0x84, 0xbe, 0x8c,
  0x84, 0x40, 0x86, 0x41, 0x26, 0x29, 0x94, 0x42, 0xbe, 0x8e, 0xd5, 0x7f,
  0x19, 0xa1, 0x43, 0x40, 0x0b, 0x2e, 0x8c, 0x01, 0x84, 0x40, 0xc7, 0x41,
  0x5d, 0x29, 0x27, 0x29, 0x45, 0x42, 0x84, 0x42, 0xc2, 0x7f, 0x01, 0x2f,
  0xc0, 0xb3, 0x1d, 0x2f, 0x05, 0x2e, 0x94, 0x01, 0x99, 0xa0, 0x01, 0x2f,
  0x80, 0xb3, 0x13, 0x2f, 0x80, 0xb3, 0x18, 0x2f, 0xc0, 0xb3, 0x16, 0x2f,
  0x12, 0x40, 0x01, 0x40, 0x92, 0x7f, 0x98, 0x2e, 0x74, 0xc0, 0x92, 0x6f,
  0x10, 0x0f, 0x20, 0x30, 0x03, 0x2f, 0x10, 0x30, 0x21, 0x2e, 0x7e, 0x01,
  0x0a, 0x2d, 0x21, 0x2e, 0x7e, 0x01, 0x07, 0x2d, 0x20, 0x30, 0x21, 0x2e,
  0x7e, 0x01, 0x03, 0x2d, 0x10, 0x30, 0x21, 0x2e, 0x7e, 0x01, 0xc2, 0x6f,
  0x01, 0x2e, 0xc9, 0x00, 0xbc, 0x84, 0x02, 0x80, 0x82, 0x40, 0x00, 0x40,
  0x90, 0x0e, 0xd5, 0x6f, 0x02, 0x2f, 0x15, 0x30, 0x98, 0x2e, 0xf3, 0x03,
  0x41, 0x91, 0x05, 0x30, 0x07, 0x2f, 0x67, 0x50, 0x3d, 0x80, 0x2b, 0x2e,
  0x8f, 0x01, 0x05, 0x42, 0x04, 0x80, 0x00, 0x2e, 0x05, 0x42, 0x02, 0x2c,
  0x00, 0x30, 0x00, 0x30, 0xa2, 0x6f, 0x98, 0x8a, 0x86, 0x40, 0x80, 0xa7,
  0x05, 0x2f, 0x98, 0x2e, 0xf3, 0x03, 0xc0, 0x30, 0x21, 0x2e, 0x95, 0x01,
  0x06, 0x25, 0x1a, 0x25, 0xe2, 0x6f, 0x76, 0x82, 0x96, 0x40, 0x56, 0x43,
  0x51, 0x0e, 0xfb, 0x2f, 0xbb, 0x6f, 0x30, 0x5f, 0xb8, 0x2e, 0x01, 0x2e,
  0xb8, 0x00, 0x01, 0x31, 0x41, 0x08, 0x40, 0xb2, 0x20, 0x50, 0xf2, 0x30,
  0x02, 0x08, 0xfb, 0x7f, 0x01, 0x30, 0x10, 0x2f, 0x05, 0x2e, 0xcc, 0x00,
  0x81, 0x90, 0xe0, 0x7f, 0x03, 0x2f, 0x23, 0x2e, 0xcc, 0x00, 0x98, 0x2e,
  0x55, 0xb6, 0x98, 0x2e, 0x1d, 0xb5, 0x10, 0x25, 0xfb, 0x6f, 0xe0, 0x6f,
  0xe0, 0x5f, 0x80, 0x2e, 0x95, 0xcf, 0x98, 0x2e, 0x95, 0xcf, 0x10, 0x30,
  0x21, 0x2e, 0xcc, 0x00, 0xfb, 0x6f, 0xe0, 0x5f, 0xb8, 0x2e, 0x00, 0x51,
  0x05, 0x58, 0xeb, 0x7f, 0x2a, 0x25, 0x89, 0x52, 0x6f, 0x5a, 0x89, 0x50,
  0x13, 0x41, 0x06, 0x40, 0xb3, 0x01, 0x16, 0x42, 0xcb, 0x16, 0x06, 0x40,
  0xf3, 0x02, 0x13, 0x42, 0x65, 0x0e, 0xf5, 0x2f, 0x05, 0x40, 0x14, 0x30,
  0x2c, 0x29, 0x04, 0x42, 0x08, 0xa1, 0x00, 0x30, 0x90, 0x2e, 0x52, 0xb6,
  0xb3, 0x88, 0xb0, 0x8a, 0xb6, 0x84, 0xa4, 0x7f, 0xc4, 0x7f, 0xb5, 0x7f,
  0xd5, 0x7f, 0x92, 0x7f, 0x73, 0x30, 0x04, 0x30, 0x55, 0x40, 0x42, 0x40,
  0x8a, 0x17, 0xf3, 0x08, 0x6b, 0x01, 0x90, 0x02, 0x53, 0xb8, 0x4b, 0x82,
  0xad, 0xbe, 0x71, 0x7f, 0x45, 0x0a, 0x09, 0x54, 0x84, 0x7f, 0x98, 0x2e,
  0xd9, 0xc0, 0xa3, 0x6f, 0x7b, 0x54, 0xd0, 0x42, 0xa3, 0x7f, 0xf2, 0x7f,
  0x60, 0x7f, 0x20, 0x25, 0x71, 0x6f, 0x75, 0x5a, 0x77, 0x58, 0x79, 0x5c,
  0x75, 0x56, 0x98, 0x2e, 0x67, 0xcc, 0xb1, 0x6f, 0x62, 0x6f, 0x50, 0x42,
  0xb1, 0x7f, 0xb3, 0x30, 0x10, 0x25, 0x98, 0x2e, 0x0f, 0xca, 0x84, 0x6f,
  0x20, 0x29, 0x71, 0x6f, 0x92, 0x6f, 0xa5, 0x6f, 0x76, 0x82, 0x6a, 0x0e,
  0x73, 0x30, 0x00, 0x30, 0xd0, 0x2f, 0xd2, 0x6f, 0xd1, 0x7f, 0xb4, 0x7f,
  0x98, 0x2e, 0x2b, 0xb7, 0x15, 0xbd, 0x0b, 0xb8, 0x02, 0x0a, 0xc2, 0x6f,
  0xc0, 0x7f, 0x98, 0x2e, 0x2b, 0xb7, 0x15, 0xbd, 0x0b, 0xb8, 0x42, 0x0a,
  0xc0, 0x6f, 0x08, 0x17, 0x41, 0x18, 0x89, 0x16, 0xe1, 0x18, 0xd0, 0x18,
  0xa1, 0x7f, 0x27, 0x25, 0x16, 0x25, 0x98, 0x2e, 0x79, 0xc0, 0x8b, 0x54,
  0x90, 0x7f, 0xb3, 0x30, 0x82, 0x40, 0x80, 0x90, 0x0d, 0x2f, 0x7d, 0x52,
  0x92, 0x6f, 0x98, 0x2e, 0x0f, 0xca, 0xb2, 0x6f, 0x90, 0x0e, 0x06, 0x2f,
  0x8b, 0x50, 0x14, 0x30, 0x42, 0x6f, 0x51, 0x6f, 0x14, 0x42, 0x12, 0x42,
  0x01, 0x42, 0x00, 0x2e, 0x31, 0x6f, 0x98, 0x2e, 0x74, 0xc0, 0x41, 0x6f,
  0x80, 0x7f, 0x98, 0x2e, 0x74, 0xc0, 0x82, 0x6f, 0x10, 0x04, 0x43, 0x52,
  0x01, 0x0f, 0x05, 0x2e, 0xcb, 0x00, 0x00, 0x30, 0x04, 0x30, 0x21, 0x2f,
  0x51, 0x6f, 0x43, 0x58, 0x8c, 0x0e, 0x04, 0x30, 0x1c, 0x2f, 0x85, 0x88,
  0x41, 0x6f, 0x04, 0x41, 0x8c, 0x0f, 0x04, 0x30, 0x16, 0x2f, 0x84, 0x88,
  0x00, 0x2e, 0x04, 0x41, 0x04, 0x05, 0x8c, 0x0e, 0x04, 0x30, 0x0f, 0x2f,
  0x82, 0x88, 0x31, 0x6f, 0x04, 0x41, 0x04, 0x05, 0x8c, 0x0e, 0x04, 0x30,
  0x08, 0x2f, 0x83, 0x88, 0x00, 0x2e, 0x04, 0x41, 0x8c, 0x0f, 0x04, 0x30,
  0x02, 0x2f, 0x21, 0x2e, 0xad, 0x01, 0x14, 0x30, 0x00, 0x91, 0x14, 0x2f,
  0x03, 0x2e, 0xa1, 0x01, 0x41, 0x90, 0x0e, 0x2f, 0x03, 0x2e, 0xad, 0x01,
  0x14, 0x30, 0x4c, 0x28, 0x23, 0x2e, 0xad, 0x01, 0x46, 0xa0, 0x06, 0x2f,
  0x81, 0x84, 0x8d, 0x52, 0x48, 0x82, 0x82, 0x40, 0x21, 0x2e, 0xa1, 0x01,
  0x42, 0x42, 0x5c, 0x2c, 0x02, 0x30, 0x05, 0x2e, 0xaa, 0x01, 0x80, 0xb2,
  0x02, 0x30, 0x55, 0x2f, 0x03, 0x2e, 0xa9, 0x01, 0x92, 0x6f, 0xb3, 0x30,
  0x98, 0x2e, 0x0f, 0xca, 0xb2, 0x6f, 0x90, 0x0f, 0x00, 0x30, 0x02, 0x30,
  0x4a, 0x2f, 0xa2, 0x6f, 0x87, 0x52, 0x91, 0x00, 0x85, 0x52, 0x51, 0x0e,
  0x02, 0x2f, 0x00, 0x2e, 0x43, 0x2c, 0x02, 0x30, 0xc2, 0x6f, 0x7f, 0x52,
  0x91, 0x0e, 0x02, 0x30, 0x3c, 0x2f, 0x51, 0x6f, 0x81, 0x54, 0x98, 0x2e,
  0xfe, 0xc9, 0x10, 0x25, 0xb3, 0x30, 0x21, 0x25, 0x98, 0x2e, 0x0f, 0xca,
  0x32, 0x6f, 0xc0, 0x7f, 0xb3, 0x30, 0x12, 0x25, 0x98, 0x2e, 0x0f, 0xca,
  0x42, 0x6f, 0xb0, 0x7f, 0xb3, 0x30, 0x12, 0x25, 0x98, 0x2e, 0x0f, 0xca,
  0xb2, 0x6f, 0x90, 0x28, 0x83, 0x52, 0x98, 0x2e, 0xfe, 0xc9, 0xc2, 0x6f,
  0x90, 0x0f, 0x00, 0x30, 0x02, 0x30, 0x1d, 0x2f, 0x05, 0x2e, 0xa1, 0x01,
  0x80, 0xb2, 0x12, 0x30, 0x0f, 0x2f, 0x42, 0x6f, 0x03, 0x2e, 0xab, 0x01,
  0x91, 0x0e, 0x02, 0x30, 0x12, 0x2f, 0x52, 0x6f, 0x03, 0x2e, 0xac, 0x01,
  0x91, 0x0f, 0x02, 0x30, 0x0c, 0x2f, 0x21, 0x2e, 0xaa, 0x01, 0x0a, 0x2c,
  0x12, 0x30, 0x03, 0x2e, 0xcb, 0x00, 0x8d, 0x58, 0x08, 0x89, 0x41, 0x40,
  0x11, 0x43, 0x00, 0x43, 0x25, 0x2e, 0xa1, 0x01, 0xd4, 0x6f, 0x8f, 0x52,
  0x00, 0x43, 0x3a, 0x89, 0x00, 0x2e, 0x10, 0x43, 0x10, 0x43, 0x61, 0x0e,
  0xfb, 0x2f, 0x03, 0x2e, 0xa0, 0x01, 0x11, 0x1a, 0x02, 0x2f, 0x02, 0x25,
  0x21, 0x2e, 0xa0, 0x01, 0xeb, 0x6f, 0x00, 0x5f, 0xb8, 0x2e, 0x91, 0x52,
  0x10, 0x30, 0x02, 0x30, 0x95, 0x56, 0x52, 0x42, 0x4b, 0x0e, 0xfc, 0x2f,
  0x8d, 0x54, 0x88, 0x82, 0x93, 0x56, 0x80, 0x42, 0x53, 0x42, 0x40, 0x42,
  0x42, 0x86, 0x83, 0x54, 0xc0, 0x2e, 0xc2, 0x42, 0x00, 0x2e, 0xa3, 0x52,
  0x00, 0x51, 0x52, 0x40, 0x47, 0x40, 0x1a, 0x25, 0x01, 0x2e, 0x97, 0x00,
  0x8f, 0xbe, 0x72, 0x86, 0xfb, 0x7f, 0x0b, 0x30, 0x7c, 0xbf, 0xa5, 0x50,
  0x10, 0x08, 0xdf, 0xba, 0x70, 0x88, 0xf8, 0xbf, 0xcb, 0x42, 0xd3, 0x7f,
  0x6c, 0xbb, 0xfc, 0xbb, 0xc5, 0x0a, 0x90, 0x7f, 0x1b, 0x7f, 0x0b, 0x43,
  0xc0, 0xb2, 0xe5, 0x7f, 0xb7, 0x7f, 0xa6, 0x7f, 0xc4, 0x7f, 0x90, 0x2e,
  0x1c, 0xb7, 0x07, 0x2e, 0xd2, 0x00, 0xc0, 0xb2, 0x0b, 0x2f, 0x97, 0x52,
  0x01, 0x2e, 0xcd, 0x00, 0x82, 0x7f, 0x98, 0x2e, 0xbb, 0xcc, 0x0b, 0x30,
  0x37, 0x2e, 0xd2, 0x00, 0x82, 0x6f, 0x90, 0x6f, 0x1a, 0x25, 0x00, 0xb2,
  0x8b, 0x7f, 0x14, 0x2f, 0xa6, 0xbd, 0x25, 0xbd, 0xb6, 0xb9, 0x2f, 0xb9,
  0x80, 0xb2, 0xd4, 0xb0, 0x0c, 0x2f, 0x99, 0x54, 0x9b, 0x56, 0x0b, 0x30,
  0x0b, 0x2e, 0xb1, 0x00, 0xa1, 0x58, 0x9b, 0x42, 0xdb, 0x42, 0x6c, 0x09,
  0x2b, 0x2e, 0xb1, 0x00, 0x8b, 0x42, 0xcb, 0x42, 0x86, 0x7f, 0x73, 0x84,
  0xa7, 0x56, 0xc3, 0x08, 0x39, 0x52, 0x05, 0x50, 0x72, 0x7f, 0x63, 0x7f,
  0x98, 0x2e, 0xc2, 0xc0, 0xe1, 0x6f, 0x62, 0x6f, 0xd1, 0x0a, 0x01, 0x2e,
  0xcd, 0x00, 0xd5, 0x6f, 0xc4, 0x6f, 0x72, 0x6f, 0x97, 0x52, 0x9d, 0x5c,
  0x98, 0x2e, 0x06, 0xcd, 0x23, 0x6f, 0x90, 0x6f, 0x99, 0x52, 0xc0, 0xb2,
  0x04, 0xbd, 0x54, 0x40, 0xaf, 0xb9, 0x45, 0x40, 0xe1, 0x7f, 0x02, 0x30,
  0x06, 0x2f, 0xc0, 0xb2, 0x02, 0x30, 0x03, 0x2f, 0x9b, 0x5c, 0x12, 0x30,
  0x94, 0x43, 0x85, 0x43, 0x03, 0xbf, 0x6f, 0xbb, 0x80, 0xb3, 0x20, 0x2f,
  0x06, 0x6f, 0x26, 0x01, 0x16, 0x6f, 0x6e, 0x03, 0x45, 0x42, 0xc0, 0x90,
  0x29, 0x2e, 0xce, 0x00, 0x9b, 0x52, 0x14, 0x2f, 0x9b, 0x5c, 0x00, 0x2e,
  0x93, 0x41, 0x86, 0x41, 0xe3, 0x04, 0xae, 0x07, 0x80, 0xab, 0x04, 0x2f,
  0x80, 0x91, 0x0a, 0x2f, 0x86, 0x6f, 0x73, 0x0f, 0x07, 0x2f, 0x83, 0x6f,
  0xc0, 0xb2, 0x04, 0x2f, 0x54, 0x42, 0x45, 0x42, 0x12, 0x30, 0x04, 0x2c,
  0x11, 0x30, 0x02, 0x2c, 0x11, 0x30, 0x11, 0x30, 0x02, 0xbc, 0x0f, 0xb8,
  0xd2, 0x7f, 0x00, 0xb2, 0x0a, 0x2f, 0x01, 0x2e, 0xfc, 0x00, 0x05, 0x2e,
  0xc7, 0x01, 0x10, 0x1a, 0x02, 0x2f, 0x21, 0x2e, 0xc7, 0x01, 0x03, 0x2d,
  0x02, 0x2c, 0x01, 0x30, 0x01, 0x30, 0xb0, 0x6f, 0x98, 0x2e, 0x95, 0xcf,
  0xd1, 0x6f, 0xa0, 0x6f, 0x98, 0x2e, 0x95, 0xcf, 0xe2, 0x6f, 0x9f, 0x52,
  0x01, 0x2e, 0xce, 0x00, 0x82, 0x40, 0x50, 0x42, 0x0c, 0x2c, 0x42, 0x42,
  0x11, 0x30, 0x23, 0x2e, 0xd2, 0x00, 0x01, 0x30, 0xb0, 0x6f, 0x98, 0x2e,
  0x95, 0xcf, 0xa0, 0x6f, 0x01, 0x30, 0x98, 0x2e, 0x95, 0xcf, 0x00, 0x2e,
  0xfb, 0x6f, 0x00, 0x5f, 0xb8, 0x2e, 0x83, 0x86, 0x01, 0x30, 0x00, 0x30,
  0x94, 0x40, 0x24, 0x18, 0x06, 0x00, 0x53, 0x0e, 0x4f, 0x02, 0xf9, 0x2f,
  0xb8, 0x2e, 0xa9, 0x52, 0x00, 0x2e, 0x60, 0x40, 0x41, 0x40, 0x0d, 0xbc,
  0x98, 0xbc, 0xc0, 0x2e, 0x01, 0x0a, 0x0f, 0xb8, 0xab, 0x52, 0x53, 0x3c,
  0x52, 0x40, 0x40, 0x40, 0x4b, 0x00, 0x82, 0x16, 0x26, 0xb9, 0x01, 0xb8,
  0x41, 0x40, 0x10, 0x08, 0x97, 0xb8, 0x01, 0x08, 0xc0, 0x2e, 0x11, 0x30,
  0x01, 0x08, 0x43, 0x86, 0x25, 0x40, 0x04, 0x40, 0xd8, 0xbe, 0x2c, 0x0b,
  0x22, 0x11, 0x54, 0x42, 0x03, 0x80, 0x4b, 0x0e, 0xf6, 0x2f, 0xb8, 0x2e,
  0x9f, 0x50, 0x10, 0x50, 0xad, 0x52, 0x05, 0x2e, 0xd3, 0x00, 0xfb, 0x7f,
  0x00, 0x2e, 0x13, 0x40, 0x93, 0x42, 0x41, 0x0e, 0xfb, 0x2f, 0x98, 0x2e,
  0xa5, 0xb7, 0x98, 0x2e, 0x87, 0xcf, 0x01, 0x2e, 0xd9, 0x00, 0x00, 0xb2,
  0xfb, 0x6f, 0x0b, 0x2f, 0x01, 0x2e, 0x69, 0xf7, 0xb1, 0x3f, 0x01, 0x08,
  0x01, 0x30, 0xf0, 0x5f, 0x23, 0x2e, 0xd9, 0x00, 0x21, 0x2e, 0x69, 0xf7,
  0x80, 0x2e, 0x7a, 0xb7, 0xf0, 0x5f, 0xb8, 0x2e, 0x01, 0x2e, 0xc0, 0xf8,
  0x03, 0x2e, 0xfc, 0xf5, 0x15, 0x54, 0xaf, 0x56, 0x82, 0x08, 0x0b, 0x2e,
  0x69, 0xf7, 0xcb, 0x0a, 0xb1, 0x58, 0x80, 0x90, 0xdd, 0xbe, 0x4c, 0x08,
  0x5f, 0xb9, 0x59, 0x22, 0x80, 0x90, 0x07, 0x2f, 0x03, 0x34, 0xc3, 0x08,
  0xf2, 0x3a, 0x0a, 0x08, 0x02, 0x35, 0xc0, 0x90, 0x4a, 0x0a, 0x48, 0x22,
  0xc0, 0x2e, 0x23, 0x2e, 0xfc, 0xf5, 0x10, 0x50, 0xfb, 0x7f, 0x98, 0x2e,
  0x56, 0xc7, 0x98, 0x2e, 0x49, 0xc3, 0x10, 0x30, 0xfb, 0x6f, 0xf0, 0x5f,
  0x21, 0x2e, 0xcc, 0x00, 0x21, 0x2e, 0xca, 0x00, 0xb8, 0x2e, 0x03, 0x2e,
  0xd3, 0x00, 0x16, 0xb8, 0x02, 0x34, 0x4a, 0x0c, 0x21, 0x2e, 0x2d, 0xf5,
  0xc0, 0x2e, 0x23, 0x2e, 0xd3, 0x00, 0x03, 0xbc, 0x21, 0x2e, 0xd5, 0x00,
  0x03, 0x2e, 0xd5, 0x00, 0x40, 0xb2, 0x10, 0x30, 0x21, 0x2e, 0x77, 0x00,
  0x01, 0x30, 0x05, 0x2f, 0x05, 0x2e, 0xd8, 0x00, 0x80, 0x90, 0x01, 0x2f,
  0x23, 0x2e, 0x6f, 0xf5, 0xc0, 0x2e, 0x21, 0x2e, 0xd9, 0x00, 0x11, 0x30,
  0x81, 0x08, 0x01, 0x2e, 0x6a, 0xf7, 0x71, 0x3f, 0x23, 0xbd, 0x01, 0x08,
  0x02, 0x0a, 0xc0, 0x2e, 0x21, 0x2e, 0x6a, 0xf7, 0x30, 0x25, 0x00, 0x30,
  0x21, 0x2e, 0x5a, 0xf5, 0x10, 0x50, 0x21, 0x2e, 0x7b, 0x00, 0x21, 0x2e,
  0x7c, 0x00, 0xfb, 0x7f, 0x98, 0x2e, 0xc3, 0xb7, 0x40, 0x30, 0x21, 0x2e,
  0xd4, 0x00, 0xfb, 0x6f, 0xf0, 0x5f, 0x03, 0x25, 0x80, 0x2e, 0xaf, 0xb7,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x01, 0x2e, 0x5d, 0xf7, 0x08, 0xbc, 0x80, 0xac, 0x0e, 0xbb, 0x02, 0x2f,
  0x00, 0x30, 0x41, 0x04, 0x82, 0x06, 0xc0, 0xa4, 0x00, 0x30, 0x11, 0x2f,
  0x40, 0xa9, 0x03, 0x2f, 0x40, 0x91, 0x0d, 0x2f, 0x00, 0xa7, 0x0b, 0x2f,
  0x80, 0xb3, 0xb3, 0x58, 0x02, 0x2f, 0x90, 0xa1, 0x26, 0x13, 0x20, 0x23,
  0x80, 0x90, 0x10, 0x30, 0x01, 0x2f, 0xcc, 0x0e, 0x00, 0x2f, 0x00, 0x30,
  0xb8, 0x2e, 0xb5, 0x50, 0x18, 0x08, 0x08, 0xbc, 0x88, 0xb6, 0x0d, 0x17,
  0xc6, 0xbd, 0x56, 0xbc, 0xb7, 0x58, 0xda, 0xba, 0x04, 0x01, 0x1d, 0x0a,
  0x10, 0x50, 0x05, 0x30, 0x32, 0x25, 0x45, 0x03, 0xfb, 0x7f, 0xf6, 0x30,
  0x21, 0x25, 0x98, 0x2e, 0x37, 0xca, 0x16, 0xb5, 0x9a, 0xbc, 0x06, 0xb8,
  0x80, 0xa8, 0x41, 0x0a, 0x0e, 0x2f, 0x80, 0x90, 0x02, 0x2f, 0x2d, 0x50,
  0x48, 0x0f, 0x09, 0x2f, 0xbf, 0xa0, 0x04, 0x2f, 0xbf, 0x90, 0x06, 0x2f,
  0xb7, 0x54, 0xca, 0x0f, 0x03, 0x2f, 0x00, 0x2e, 0x02, 0x2c, 0xb7, 0x52,
  0x2d, 0x52, 0xf2, 0x33, 0x98, 0x2e, 0xd9, 0xc0, 0xfb, 0x6f, 0xf1, 0x37,
  0xc0, 0x2e, 0x01, 0x08, 0xf0, 0x5f, 0xbf, 0x56, 0xb9, 0x54, 0xd0, 0x40,
  0xc4, 0x40, 0x0b, 0x2e, 0xfd, 0xf3, 0xbf, 0x52, 0x90, 0x42, 0x94, 0x42,
  0x95, 0x42, 0x05, 0x30, 0xc1, 0x50, 0x0f, 0x88, 0x06, 0x40, 0x04, 0x41,
  0x96, 0x42, 0xc5, 0x42, 0x48, 0xbe, 0x73, 0x30, 0x0d, 0x2e, 0xd8, 0x00,
  0x4f, 0xba, 0x84, 0x42, 0x03, 0x42, 0x81, 0xb3, 0x02, 0x2f, 0x2b, 0x2e,
  0x6f, 0xf5, 0x06, 0x2d, 0x05, 0x2e, 0x77, 0xf7, 0xbd, 0x56, 0x93, 0x08,
  0x25, 0x2e, 0x77, 0xf7, 0xbb, 0x54, 0x25, 0x2e, 0xc2, 0xf5, 0x07, 0x2e,
  0xfd, 0xf3, 0x42, 0x30, 0xb4, 0x33, 0xda, 0x0a, 0x4c, 0x00, 0x27, 0x2e,
  0xfd, 0xf3, 0x43, 0x40, 0xd4, 0x3f, 0xdc, 0x08, 0x43, 0x42, 0x00, 0x2e,
  0x00, 0x2e, 0x43, 0x40, 0x24, 0x30, 0xdc, 0x0a, 0x43, 0x42, 0x04, 0x80,
  0x03, 0x2e, 0xfd, 0xf3, 0x4a, 0x0a, 0x23, 0x2e, 0xfd, 0xf3, 0x61, 0x34,
  0xc0, 0x2e, 0x01, 0x42, 0x00, 0x2e, 0x60, 0x50, 0x1a, 0x25, 0x7a, 0x86,
  0xe0, 0x7f, 0xf3, 0x7f, 0x03, 0x25, 0xc3, 0x52, 0x41, 0x84, 0xdb, 0x7f,
  0x33, 0x30, 0x98, 0x2e, 0x16, 0xc2, 0x1a, 0x25, 0x7d, 0x82, 0xf0, 0x6f,
  0xe2, 0x6f, 0x32, 0x25, 0x16, 0x40, 0x94, 0x40, 0x26, 0x01, 0x85, 0x40,
  0x8e, 0x17, 0xc4, 0x42, 0x6e, 0x03, 0x95, 0x42, 0x41, 0x0e, 0xf4, 0x2f,
  0xdb, 0x6f, 0xa0, 0x5f, 0xb8, 0x2e, 0xb0, 0x51, 0xfb, 0x7f, 0x98, 0x2e,
  0xe8, 0x0d, 0x5a, 0x25, 0x98, 0x2e, 0x0f, 0x0e, 0xcb, 0x58, 0x32, 0x87,
  0xc4, 0x7f, 0x65, 0x89, 0x6b, 0x8d, 0xc5, 0x5a, 0x65, 0x7f, 0xe1, 0x7f,
  0x83, 0x7f, 0xa6, 0x7f, 0x74, 0x7f, 0xd0, 0x7f, 0xb6, 0x7f, 0x94, 0x7f,
  0x17, 0x30, 0xc7, 0x52, 0xc9, 0x54, 0x51, 0x7f, 0x00, 0x2e, 0x85, 0x6f,
  0x42, 0x7f, 0x00, 0x2e, 0x51, 0x41, 0x45, 0x81, 0x42, 0x41, 0x13, 0x40,
  0x3b, 0x8a, 0x00, 0x40, 0x4b, 0x04, 0xd0, 0x06, 0xc0, 0xac, 0x85, 0x7f,
  0x02, 0x2f, 0x02, 0x30, 0x51, 0x04, 0xd3, 0x06, 0x41, 0x84, 0x05, 0x30,
  0x5d, 0x02, 0xc9, 0x16, 0xdf, 0x08, 0xd3, 0x00, 0x8d, 0x02, 0xaf, 0xbc,
  0xb1, 0xb9, 0x59, 0x0a, 0x65, 0x6f, 0x11, 0x43, 0xa1, 0xb4, 0x52, 0x41,
  0x53, 0x41, 0x01, 0x43, 0x34, 0x7f, 0x65, 0x7f, 0x26, 0x31, 0xe5, 0x6f,
  0xd4, 0x6f, 0x98, 0x2e, 0x37, 0xca, 0x32, 0x6f, 0x75, 0x6f, 0x83, 0x40,
  0x42, 0x41, 0x23, 0x7f, 0x12, 0x7f, 0xf6, 0x30, 0x40, 0x25, 0x51, 0x25,
  0x98, 0x2e, 0x37, 0xca, 0x14, 0x6f, 0x20, 0x05, 0x70, 0x6f, 0x25, 0x6f,
  0x69, 0x07, 0xa2, 0x6f, 0x31, 0x6f, 0x0b, 0x30, 0x04, 0x42, 0x9b, 0x42,
  0x8b, 0x42, 0x55, 0x42, 0x32, 0x7f, 0x40, 0xa9, 0xc3, 0x6f, 0x71, 0x7f,
  0x02, 0x30, 0xd0, 0x40, 0xc3, 0x7f, 0x03, 0x2f, 0x40, 0x91, 0x15, 0x2f,
  0x00, 0xa7, 0x13, 0x2f, 0x00, 0xa4, 0x11, 0x2f, 0x84, 0xbd, 0x98, 0x2e,
  0x79, 0xca, 0x55, 0x6f, 0xb7, 0x54, 0x54, 0x41, 0x82, 0x00, 0xf3, 0x3f,
  0x45, 0x41, 0xcb, 0x02, 0xf6, 0x30, 0x98, 0x2e, 0x37, 0xca, 0x35, 0x6f,
  0xa4, 0x6f, 0x41, 0x43, 0x03, 0x2c, 0x00, 0x43, 0xa4, 0x6f, 0x35, 0x6f,
  0x17, 0x30, 0x42, 0x6f, 0x51, 0x6f, 0x93, 0x40, 0x42, 0x82, 0x00, 0x41,
  0xc3, 0x00, 0x03, 0x43, 0x51, 0x7f, 0x00, 0x2e, 0x94, 0x40, 0x41, 0x41,
  0x4c, 0x02, 0xc4, 0x6f, 0xd1, 0x56, 0x63, 0x0e, 0x74, 0x6f, 0x51, 0x43,
  0xa5, 0x7f, 0x8a, 0x2f, 0x09, 0x2e, 0xd8, 0x00, 0x01, 0xb3, 0x21, 0x2f,
  0xcb, 0x58, 0x90, 0x6f, 0x13, 0x41, 0xb6, 0x6f, 0xe4, 0x7f, 0x00, 0x2e,
  0x91, 0x41, 0x14, 0x40, 0x92, 0x41, 0x15, 0x40, 0x17, 0x2e, 0x6f, 0xf5,
  0xb6, 0x7f, 0xd0, 0x7f, 0xcb, 0x7f, 0x98, 0x2e, 0x00, 0x0c, 0x07, 0x15,
  0xc2, 0x6f, 0x14, 0x0b, 0x29, 0x2e, 0x6f, 0xf5, 0xc3, 0xa3, 0xc1, 0x8f,
  0xe4, 0x6f, 0xd0, 0x6f, 0xe6, 0x2f, 0x14, 0x30, 0x05, 0x2e, 0x6f, 0xf5,
  0x14, 0x0b, 0x29, 0x2e, 0x6f, 0xf5, 0x18, 0x2d, 0xcd, 0x56, 0x04, 0x32,
  0xb5, 0x6f, 0x1c, 0x01, 0x51, 0x41, 0x52, 0x41, 0xc3, 0x40, 0xb5, 0x7f,
  0xe4, 0x7f, 0x98, 0x2e, 0x1f, 0x0c, 0xe4, 0x6f, 0x21, 0x87, 0x00, 0x43,
  0x04, 0x32, 0xcf, 0x54, 0x5a, 0x0e, 0xef, 0x2f, 0x15, 0x54, 0x09, 0x2e,
  0x77, 0xf7, 0x22, 0x0b, 0x29, 0x2e, 0x77, 0xf7, 0xfb, 0x6f, 0x50, 0x5e,
  0xb8, 0x2e, 0x10, 0x50, 0x01, 0x2e, 0xd4, 0x00, 0x00, 0xb2, 0xfb, 0x7f,
  0x51, 0x2f, 0x01, 0xb2, 0x48, 0x2f, 0x02, 0xb2, 0x42, 0x2f, 0x03, 0x90,
  0x56, 0x2f, 0xd7, 0x52, 0x79, 0x80, 0x42, 0x40, 0x81, 0x84, 0x00, 0x40,
  0x42, 0x42, 0x98, 0x2e, 0x93, 0x0c, 0xd9, 0x54, 0xd7, 0x50, 0xa1, 0x40,
  0x98, 0xbd, 0x82, 0x40, 0x3e, 0x82, 0xda, 0x0a, 0x44, 0x40, 0x8b, 0x16,
  0xe3, 0x00, 0x53, 0x42, 0x00, 0x2e, 0x43, 0x40, 0x9a, 0x02, 0x52, 0x42,
  0x00, 0x2e, 0x41, 0x40, 0x15, 0x54, 0x4a, 0x0e, 0x3a, 0x2f, 0x3a, 0x82,
  0x00, 0x30, 0x41, 0x40, 0x21, 0x2e, 0x85, 0x0f, 0x40, 0xb2, 0x0a, 0x2f,
  0x98, 0x2e, 0xb1, 0x0c, 0x98, 0x2e, 0x45, 0x0e, 0x98, 0x2e, 0x5b, 0x0e,
  0xfb, 0x6f, 0xf0, 0x5f, 0x00, 0x30, 0x80, 0x2e, 0xce, 0xb7, 0xdd, 0x52,
  0xd3, 0x54, 0x42, 0x42, 0x4f, 0x84, 0x73, 0x30, 0xdb, 0x52, 0x83, 0x42,
  0x1b, 0x30, 0x6b, 0x42, 0x23, 0x30, 0x27, 0x2e, 0xd7, 0x00, 0x37, 0x2e,
  0xd4, 0x00, 0x21, 0x2e, 0xd6, 0x00, 0x7a, 0x84, 0x17, 0x2c, 0x42, 0x42,
  0x30, 0x30, 0x21, 0x2e, 0xd4, 0x00, 0x12, 0x2d, 0x21, 0x30, 0x00, 0x30,
  0x23, 0x2e, 0xd4, 0x00, 0x21, 0x2e, 0x7b, 0xf7, 0x0b, 0x2d, 0x17, 0x30,
  0x98, 0x2e, 0x51, 0x0c, 0xd5, 0x50, 0x0c, 0x82, 0x72, 0x30, 0x2f, 0x2e,
  0xd4, 0x00, 0x25, 0x2e, 0x7b, 0xf7, 0x40, 0x42, 0x00, 0x2e, 0xfb, 0x6f,
  0xf0, 0x5f, 0xb8, 0x2e, 0x70, 0x50, 0x0a, 0x25, 0x39, 0x86, 0xfb, 0x7f,
  0xe1, 0x32, 0x62, 0x30, 0x98, 0x2e, 0xc2, 0xc4, 0xb5, 0x56, 0xa5, 0x6f,
  0xab, 0x08, 0x91, 0x6f, 0x4b, 0x08, 0xdf, 0x56, 0xc4, 0x6f, 0x23, 0x09,
  0x4d, 0xba, 0x93, 0xbc, 0x8c, 0x0b, 0xd1, 0x6f, 0x0b, 0x09, 0xcb, 0x52,
  0xe1, 0x5e, 0x56, 0x42, 0xaf, 0x09, 0x4d, 0xba, 0x23, 0xbd, 0x94, 0x0a,
  0xe5, 0x6f, 0x68, 0xbb, 0xeb, 0x08, 0xbd, 0xb9, 0x63, 0xbe, 0xfb, 0x6f,
  0x52, 0x42, 0xe3, 0x0a, 0xc0, 0x2e, 0x43, 0x42, 0x90, 0x5f, 0xd1, 0x50,
  0x03, 0x2e, 0x25, 0xf3, 0x13, 0x40, 0x00, 0x40, 0x9b, 0xbc, 0x9b, 0xb4,
  0x08, 0xbd, 0xb8, 0xb9, 0x98, 0xbc, 0xda, 0x0a, 0x08, 0xb6, 0x89, 0x16,
  0xc0, 0x2e, 0x19, 0x00, 0x62, 0x02, 0x10, 0x50, 0xfb, 0x7f, 0x98, 0x2e,
  0x81, 0x0d, 0x01, 0x2e, 0xd4, 0x00, 0x31, 0x30, 0x08, 0x04, 0xfb, 0x6f,
  0x01, 0x30, 0xf0, 0x5f, 0x23, 0x2e, 0xd6, 0x00, 0x21, 0x2e, 0xd7, 0x00,
  0xb8, 0x2e, 0x01, 0x2e, 0xd7, 0x00, 0x03, 0x2e, 0xd6, 0x00, 0x48, 0x0e,
  0x01, 0x2f, 0x80, 0x2e, 0x1f, 0x0e, 0xb8, 0x2e, 0xe3, 0x50, 0x21, 0x34,
  0x01, 0x42, 0x82, 0x30, 0xc1, 0x32, 0x25, 0x2e, 0x62, 0xf5, 0x01, 0x00,
  0x22, 0x30, 0x01, 0x40, 0x4a, 0x0a, 0x01, 0x42, 0xb8, 0x2e, 0xe3, 0x54,
  0xf0, 0x3b, 0x83, 0x40, 0xd8, 0x08, 0xe5, 0x52, 0x83, 0x42, 0x00, 0x30,
  0x83, 0x30, 0x50, 0x42, 0xc4, 0x32, 0x27, 0x2e, 0x64, 0xf5, 0x94, 0x00,
  0x50, 0x42, 0x40, 0x42, 0xd3, 0x3f, 0x84, 0x40, 0x7d, 0x82, 0xe3, 0x08,
  0x40, 0x42, 0x83, 0x42, 0xb8, 0x2e, 0xdd, 0x52, 0x00, 0x30, 0x40, 0x42,
  0x7c, 0x86, 0xb9, 0x52, 0x09, 0x2e, 0x70, 0x0f, 0xbf, 0x54, 0xc4, 0x42,
  0xd3, 0x86, 0x54, 0x40, 0x55, 0x40, 0x94, 0x42, 0x85, 0x42, 0x21, 0x2e,
  0xd7, 0x00, 0x42, 0x40, 0x25, 0x2e, 0xfd, 0xf3, 0xc0, 0x42, 0x7e, 0x82,
  0x05, 0x2e, 0x7d, 0x00, 0x80, 0xb2, 0x14, 0x2f, 0x05, 0x2e, 0x89, 0x00,
  0x27, 0xbd, 0x2f, 0xb9, 0x80, 0x90, 0x02, 0x2f, 0x21, 0x2e, 0x6f, 0xf5,
  0x0c, 0x2d, 0x07, 0x2e, 0x71, 0x0f, 0x14, 0x30, 0x1c, 0x09, 0x05, 0x2e,
  0x77, 0xf7, 0xbd, 0x56, 0x47, 0xbe, 0x93, 0x08, 0x94, 0x0a, 0x25, 0x2e,
  0x77, 0xf7, 0xe7, 0x54, 0x50, 0x42, 0x4a, 0x0e, 0xfc, 0x2f, 0xb8, 0x2e,
  0x50, 0x50, 0x02, 0x30, 0x43, 0x86, 0xe5, 0x50, 0xfb, 0x7f, 0xe3, 0x7f,
  0xd2, 0x7f, 0xc0, 0x7f, 0xb1, 0x7f, 0x00, 0x2e, 0x41, 0x40, 0x00, 0x40,
  0x48, 0x04, 0x98, 0x2e, 0x74, 0xc0, 0x1e, 0xaa, 0xd3, 0x6f, 0x14, 0x30,
  0xb1, 0x6f, 0xe3, 0x22, 0xc0, 0x6f, 0x52, 0x40, 0xe4, 0x6f, 0x4c, 0x0e,
  0x12, 0x42, 0xd3, 0x7f, 0xeb, 0x2f, 0x03, 0x2e, 0x86, 0x0f, 0x40, 0x90,
  0x11, 0x30, 0x03, 0x2f, 0x23, 0x2e, 0x86, 0x0f, 0x02, 0x2c, 0x00, 0x30,
  0xd0, 0x6f, 0xfb, 0x6f, 0xb0, 0x5f, 0xb8, 0x2e, 0x40, 0x50, 0xf1, 0x7f,
  0x0a, 0x25, 0x3c, 0x86, 0xeb, 0x7f, 0x41, 0x33, 0x22, 0x30, 0x98, 0x2e,
  0xc2, 0xc4, 0xd3, 0x6f, 0xf4, 0x30, 0xdc, 0x09, 0x47, 0x58, 0xc2, 0x6f,
  0x94, 0x09, 0xeb, 0x58, 0x6a, 0xbb, 0xdc, 0x08, 0xb4, 0xb9, 0xb1, 0xbd,
  0xe9, 0x5a, 0x95, 0x08, 0x21, 0xbd, 0xf6, 0xbf, 0x77, 0x0b, 0x51, 0xbe,
  0xf1, 0x6f, 0xeb, 0x6f, 0x52, 0x42, 0x54, 0x42, 0xc0, 0x2e, 0x43, 0x42,
  0xc0, 0x5f, 0x50, 0x50, 0xf5, 0x50, 0x31, 0x30, 0x11, 0x42, 0xfb, 0x7f,
  0x7b, 0x30, 0x0b, 0x42, 0x11, 0x30, 0x02, 0x80, 0x23, 0x33, 0x01, 0x42,
  0x03, 0x00, 0x07, 0x2e, 0x80, 0x03, 0x05, 0x2e, 0xd3, 0x00, 0x23, 0x52,
  0xe2, 0x7f, 0xd3, 0x7f, 0xc0, 0x7f, 0x98, 0x2e, 0xb6, 0x0e, 0xd1, 0x6f,
  0x08, 0x0a, 0x1a, 0x25, 0x7b, 0x86, 0xd0, 0x7f, 0x01, 0x33, 0x12, 0x30,
  0x98, 0x2e, 0xc2, 0xc4, 0xd1, 0x6f, 0x08, 0x0a, 0x00, 0xb2, 0x0d, 0x2f,
  0xe3, 0x6f, 0x01, 0x2e, 0x80, 0x03, 0x51, 0x30, 0xc7, 0x86, 0x23, 0x2e,
  0x21, 0xf2, 0x08, 0xbc, 0xc0, 0x42, 0x98, 0x2e, 0xa5, 0xb7, 0x00, 0x2e,
  0x00, 0x2e, 0xd0, 0x2e, 0xb0, 0x6f, 0x0b, 0xb8, 0x03, 0x2e, 0x1b, 0x00,
  0x08, 0x1a, 0xb0, 0x7f, 0x70, 0x30, 0x04, 0x2f, 0x21, 0x2e, 0x21, 0xf2,
  0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x98, 0x2e, 0x6d, 0xc0, 0x98, 0x2e,
  0x5d, 0xc0, 0xed, 0x50, 0x98, 0x2e, 0x44, 0xcb, 0xef, 0x50, 0x98, 0x2e,
  0x46, 0xc3, 0xf1, 0x50, 0x98, 0x2e, 0x53, 0xc7, 0x35, 0x50, 0x98, 0x2e,
  0x64, 0xcf, 0x10, 0x30, 0x98, 0x2e, 0xdc, 0x03, 0x20, 0x26, 0xc0, 0x6f,
  0x02, 0x31, 0x12, 0x42, 0xab, 0x33, 0x0b, 0x42, 0x37, 0x80, 0x01, 0x30,
  0x01, 0x42, 0xf3, 0x37, 0xf7, 0x52, 0xfb, 0x50, 0x44, 0x40, 0xa2, 0x0a,
  0x42, 0x42, 0x8b, 0x31, 0x09, 0x2e, 0x5e, 0xf7, 0xf9, 0x54, 0xe3, 0x08,
  0x83, 0x42, 0x1b, 0x42, 0x23, 0x33, 0x4b, 0x00, 0xbc, 0x84, 0x0b, 0x40,
  0x33, 0x30, 0x83, 0x42, 0x0b, 0x42, 0xe0, 0x7f, 0xd1, 0x7f, 0x98, 0x2e,
  0x58, 0xb7, 0xd1, 0x6f, 0x80, 0x30, 0x40, 0x42, 0x03, 0x30, 0xe0, 0x6f,
  0xf3, 0x54, 0x04, 0x30, 0x00, 0x2e, 0x00, 0x2e, 0x01, 0x89, 0x62, 0x0e,
  0xfa, 0x2f, 0x43, 0x42, 0x11, 0x30, 0xfb, 0x6f, 0xc0, 0x2e, 0x01, 0x42,
  0xb0, 0x5f, 0xc1, 0x4a, 0x00, 0x00, 0x6d, 0x57, 0x00, 0x00, 0x77, 0x8e,
  0x00, 0x00, 0xe0, 0xff, 0xff, 0xff, 0xd3, 0xff, 0xff, 0xff, 0xe5, 0xff,
  0xff, 0xff, 0xee, 0xe1, 0xff, 0xff, 0x7c, 0x13, 0x00, 0x00, 0x46, 0xe6,
  0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* SPI functions */

/****************************************************************************
 * Name: bmi270_spi_read
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

static void bmi270_spi_read(FAR struct bmi270_dev_s *priv,
                            uint8_t regaddr,
                            FAR uint8_t *regval,
                            uint8_t len)
{
  uint8_t sendbuffer[BMI270_SPI_READ_MAX_BUFFER];
  uint8_t revbuffer[BMI270_SPI_READ_MAX_BUFFER];

  memset(sendbuffer, 0, BMI270_SPI_READ_MAX_BUFFER);
  memset(revbuffer, 0, BMI270_SPI_READ_MAX_BUFFER);

  /* Lock the SPI bus so that only one device can access it at the same
   * time.
   */

  SPI_LOCK(priv->config->spi, true);

  /* Set SPI frequency and mode. */

  SPI_SETFREQUENCY(priv->config->spi, priv->config->freq);
  SPI_SETMODE(priv->config->spi, SPIDEV_MODE3);

  /* Set CS to low which selects the BMI270. */

  SPI_SELECT(priv->config->spi, priv->config->cs, true);

  /* Transmit the register address from where we want to read - the MSB
   * needs to be set to indicate the read indication. Then Write some
   * idle byte while receiving the required data.
   */

  sendbuffer[0] = regaddr | 0x80;
  SPI_EXCHANGE(priv->config->spi, sendbuffer, revbuffer, (len + 2));

  /* Copy data from buffer to receive array. */

  memcpy(regval, revbuffer + 2, len);

  /* Set CS to high which deselects the BMI270. */

  SPI_SELECT(priv->config->spi, priv->config->cs, false);

  /* Unlock the SPI bus. */

  SPI_LOCK(priv->config->spi, false);
}

/****************************************************************************
 * Name: bmi270_spi_read_enhance
 *
 * Description:
 *   Read data from spi. This function is more suitable for
 *   reading multiple bytes.
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

static void bmi270_spi_read_enhance(FAR struct bmi270_dev_s *priv,
                                    uint8_t regaddr,
                                    FAR uint8_t *regval,
                                    uint8_t len)
{
  uint8_t sendbuffer[2];

  /* Lock the SPI bus so that only one device can access it at the same
   * time.
   */

  SPI_LOCK(priv->config->spi, true);

  /* Set SPI frequency and mode. */

  SPI_SETFREQUENCY(priv->config->spi, priv->config->freq);
  SPI_SETMODE(priv->config->spi, SPIDEV_MODE3);

  /* Set CS to low which selects the BMI270. */

  SPI_SELECT(priv->config->spi, priv->config->cs, true);

  /* Transmit the register address from where we want to read - the MSB
   * needs to be set to indicate the read indication. Then Write some
   * idle byte while receiving the required data.
   */

  sendbuffer[0] = regaddr | 0x80;
  SPI_EXCHANGE(priv->config->spi, sendbuffer, sendbuffer, 2);
  SPI_EXCHANGE(priv->config->spi, sendbuffer, regval, len);

  /* Set CS to high which deselects the BMI270. */

  SPI_SELECT(priv->config->spi, priv->config->cs, false);

  /* Unlock the SPI bus. */

  SPI_LOCK(priv->config->spi, false);
}

/****************************************************************************
 * Name: bmi270_spi_write
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

static void bmi270_spi_write(FAR struct bmi270_dev_s *priv,
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

/****************************************************************************
 * Name: bmi270_spi_write
 *
 * Description:
 *   write 16-bit register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Register address.
 *   value   - To be write value.
 *   len     - Write data lenth.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void bmi270_spi_write_len(FAR struct bmi270_dev_s *priv,
                                 uint8_t regaddr,
                                 FAR uint8_t *value,
                                 uint8_t len)
{
  uint8_t revbuffer[BMI270_SPI_WRITE_MAX_BUFFER];

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

  SPI_EXCHANGE(priv->config->spi, &regaddr, revbuffer, 1);
  SPI_EXCHANGE(priv->config->spi, value, revbuffer, len);

  /* Set CS to high which deselects the LIS3MDL. */

  SPI_SELECT(priv->config->spi, priv->config->cs, false);

  /* Unlock the SPI bus. */

  SPI_LOCK(priv->config->spi, false);
}

/* Sensor handle functions */

/****************************************************************************
 * Name: bmi270_readdevid
 *
 * Description:
 *   Read the device ID.
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

static int bmi270_readdevid(FAR struct bmi270_dev_s *priv)
{
  uint8_t regval;
  int ret;

  /* Performing a dummy read to bring interface back to SPI from I2C. */

  bmi270_spi_read(priv, BMI270_CHIP_ID, &regval, 1);

  usleep(BMI270_WAIT_TIME);

  /* Read the device ID. */

  bmi270_spi_read(priv, BMI270_CHIP_ID, &regval, 1);
  if (regval != BMI270_DEVICE_ID)
    {
      ret = -ENODEV;
      snerr("Wrong device ID! : %d\n", regval);
    }
  else
    {
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: bmi270_wait_selftest
 *
 * Description:
 *   Wait till crt status complete.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi270_wait_selftest(FAR struct bmi270_dev_s *priv)
{
  uint8_t count = BMI270_WAIT_SELFTEST_COUNT;
  bmi270_gyr_crt_conf_t reg_gyr_crt;

  while (count--)
    {
      bmi270_spi_read(priv, BMI270_GYR_CRT_CONF,
                      (FAR uint8_t *)&reg_gyr_crt, 1);

      if (reg_gyr_crt.crt_running == 0)
        {
          break;
        }

      usleep(BMI270_WAIT_SELFTEST);
    }

  if (count == 0)
    {
      return BMI270_ST_FAIL;
    }

  return OK;
}

/****************************************************************************
 * Name: bmi270_datatest
 *
 * Description:
 *   Selftesting the sensor.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   Return BMI270_ST_PASS if the selftest was success;
 *   Return BMI270_ST_FAIL if the selftest was failed.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_datatest(FAR struct bmi270_dev_s *priv, int type)
{
  uint8_t feat_config[BMI270_FEAT_SIZE_IN_BYTES];
  bmi270_acc_self_test_t regval_acc_self_test;
  bmi270_gyr_self_test_axes_t reg_gyr_self_test_axes;
  bmi270_pwr_ctrl_t regval_pwr_ctrl;
  bmi270_acc_conf_t regval_acc_conf;
  bmi270_gyr_crt_conf_t reg_gyr_crt;
  bmi270_cmd_reg_t reg_cmd;
  float positive_data[3];
  float negative_data[3];
  axis3bit16_t temp;
  int result;
  int ret;

  if (type == SENSOR_TYPE_ACCELEROMETER)        /* Accelerometer Self Test */
    {
      /* Disable advanced power save mode */

      bmi270_set_powersave(priv, BMI270_DISABLE);
      usleep(2000);

      /* Enable accelerometer. */

      bmi270_spi_read(priv, BMI270_PWR_CTRL,
                      (FAR uint8_t *)&regval_pwr_ctrl, 1);
      regval_pwr_ctrl.acc_en = BMI270_ENABLE;
      bmi270_spi_write(priv, BMI270_PWR_CTRL,
                       (FAR uint8_t *)&regval_pwr_ctrl);

      /* Set ±16g range. */

      bmi270_xl_setfullscale(priv, BMI270_XL_RANGE_16G);

      /* Set self test amplitude to high. */

      bmi270_spi_read(priv, BMI270_ACC_SELF_TEST,
                      (FAR uint8_t *)&regval_acc_self_test, 1);
      regval_acc_self_test.acc_self_test_amp = BMI270_ENABLE;
      bmi270_spi_write(priv, BMI270_ACC_SELF_TEST,
                       (FAR uint8_t *)&regval_acc_self_test);

      /* Set accel config. */

      bmi270_spi_read(priv, BMI270_ACC_CONF,
                      (FAR uint8_t *)&regval_acc_conf, 1);
      regval_acc_conf.acc_odr = BMI270_XL_ODR_1600HZ;
      regval_acc_conf.acc_bwp = BMI270_XL_NORMAL_AVG4;
      regval_acc_conf.acc_filter_perf = BMI270_ENABLE;
      bmi270_spi_write(priv, BMI270_ACC_CONF,
                       (FAR uint8_t *)&regval_acc_conf);

      /* Wait for more than 2 ms. */

      usleep(10000);

      /* Set positive self-test polarity. */

      bmi270_spi_read(priv, BMI270_ACC_SELF_TEST,
                      (FAR uint8_t *)&regval_acc_self_test, 1);
      regval_acc_self_test.acc_self_test_sign = 1;

      /* Enable self-test. */

      regval_acc_self_test.acc_self_test_en = BMI270_ENABLE;
      bmi270_spi_write(priv, BMI270_ACC_SELF_TEST,
                       (FAR uint8_t *)&regval_acc_self_test);

      /* Wait for more than 50 ms. */

      usleep(100000);

      /* Read and store positive acceleration value of each axis. */

      bmi270_spi_read(priv, BMI270_DATA_8, temp.u8bit, 6);
      positive_data[0] = (temp.i16bit[0] * 16) / BMI270_HALF_SCALE;
      positive_data[1] = (temp.i16bit[1] * 16) / BMI270_HALF_SCALE;
      positive_data[2] = (temp.i16bit[2] * 16) / BMI270_HALF_SCALE;

      sninfo("accel x positive data ->: %f g\n", positive_data[0]);
      sninfo("accel y positive data ->: %f g\n", positive_data[1]);
      sninfo("accel z positive data ->: %f g\n", positive_data[2]);

      /* Set positive self-test polarity. */

      bmi270_spi_read(priv, BMI270_ACC_SELF_TEST,
                      (FAR uint8_t *)&regval_acc_self_test, 1);

      regval_acc_self_test.acc_self_test_sign = 0;

      /* Enable self-test. */

      regval_acc_self_test.acc_self_test_en = BMI270_ENABLE;
      bmi270_spi_write(priv, BMI270_ACC_SELF_TEST,
                       (FAR uint8_t *)&regval_acc_self_test);

      /* Wait for more than 50 ms. */

      usleep(100000);

      /* Read and store negative acceleration value of each axis. */

      bmi270_spi_read(priv, BMI270_DATA_8, temp.u8bit, 6);
      negative_data[0] = (temp.i16bit[0] * 16) / BMI270_HALF_SCALE;
      negative_data[1] = (temp.i16bit[1] * 16) / BMI270_HALF_SCALE;
      negative_data[2] = (temp.i16bit[2] * 16) / BMI270_HALF_SCALE;

      sninfo("accel x negative data ->: %f g\n", negative_data[0]);
      sninfo("accel y negative data ->: %f g\n", negative_data[1]);
      sninfo("accel z negative data ->: %f g\n", negative_data[2]);

      /* Calculate difference of positive and negative acceleration
       * values and compare against minimum difference signal values
       * defined in the table below.
       */

      if ((positive_data[0] - negative_data[0] > BMI270_ST_X_AXIS)
          && (positive_data[1] - negative_data[1] < BMI270_ST_Y_AXIS)
          && (positive_data[2] - negative_data[2] > BMI270_ST_Z_AXIS))
        {
          result = BMI270_ST_PASS;
        }
      else
        {
          result = BMI270_ST_FAIL;
        }

      /* Disable self-test. */

      regval_acc_self_test.acc_self_test_en = BMI270_DISABLE;
      bmi270_spi_write(priv, BMI270_ACC_SELF_TEST,
                       (FAR uint8_t *)&regval_acc_self_test);

      /* Disable accelerometer. */

      regval_pwr_ctrl.acc_en = BMI270_DISABLE;
      bmi270_spi_write(priv, BMI270_PWR_CTRL,
                       (FAR uint8_t *)&regval_pwr_ctrl);
    }
  else if (type == SENSOR_TYPE_GYROSCOPE)       /* Gyroscope Self Test */
    {
      /* Enable the CRT running. */

      bmi270_spi_read(priv, BMI270_GYR_CRT_CONF,
                      (FAR uint8_t *)&reg_gyr_crt, 1);
      reg_gyr_crt.crt_running = BMI270_ENABLE;
      bmi270_spi_write(priv, BMI270_GYR_CRT_CONF,
                       (FAR uint8_t *)&reg_gyr_crt);

      /* Enable accelerometer. */

      bmi270_spi_read(priv, BMI270_PWR_CTRL,
                      (FAR uint8_t *)&regval_pwr_ctrl, 1);
      regval_pwr_ctrl.acc_en = BMI270_ENABLE;
      bmi270_spi_write(priv, BMI270_PWR_CTRL,
                       (FAR uint8_t *)&regval_pwr_ctrl);
      usleep(2000);

      /* Enable the CRT. */

      bmi270_get_feat_config(priv, BMI270_PAGE_1, feat_config);
      feat_config[BMI270_CRT_GYRO_SELF_TEST]
        = (feat_config[BMI270_CRT_GYRO_SELF_TEST] & (~BMI270_CRT_MASK))
        | (BMI270_SELECT_GYRO_SELF_TEST & BMI270_CRT_MASK);

      bmi270_spi_write_len(priv,
                           (BMI270_FEATURES + BMI270_CRT_GYRO_SELF_TEST - 1),
                           &feat_config[BMI270_CRT_GYRO_SELF_TEST - 1], 2);

      /* Send g_trigger command using the register CMD. */

      bmi270_spi_read(priv, BMI270_CMD_REG, (FAR uint8_t *)&reg_cmd, 1);
      reg_cmd.value = BMI270_G_TRIGGER_CMD;
      bmi270_spi_write(priv, BMI270_CMD_REG, (FAR uint8_t *)&reg_cmd);

      /* Wait until st_status = 0 or time out. */

      ret = bmi270_wait_selftest(priv);
      if (ret < 0)
        {
          result = BMI270_ST_FAIL;
        }

      /* Self-test is complete. */

      bmi270_spi_read(priv, BMI270_GYR_SELF_TEST_AXES,
                      (FAR uint8_t *)&reg_gyr_self_test_axes, 1);

      if (reg_gyr_self_test_axes.gyr_st_axes_done
          && reg_gyr_self_test_axes.gyr_axis_x_ok
          && reg_gyr_self_test_axes.gyr_axis_y_ok
          && reg_gyr_self_test_axes.gyr_axis_z_ok)
        {
          result = BMI270_ST_PASS;
        }
      else
        {
          result = BMI270_ST_FAIL;
        }
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  if (result == BMI270_ST_PASS)
    {
      snerr("Self Test - PASS\n");
    }
  else
    {
      snerr("Self Test - FAIL\n");
    }

  return result;
}

/****************************************************************************
 * Name: bmi270_reset
 *
 * Description:
 *   Reset the bmi270 sensor. All registers are overwritten with their
 *   default values.
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

static void bmi270_reset(FAR struct bmi270_dev_s *priv)
{
  bmi270_cmd_reg_t reg;

  reg.value = BMI270_SOFT_RESET_CMD;
  bmi270_spi_write(priv, BMI270_CMD_REG, (FAR uint8_t *)&reg);
}

/****************************************************************************
 * Name: bmi270_resetwait
 *
 * Description:
 *   Waitting for sensor reset.
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

static void bmi270_resetwait(FAR struct bmi270_dev_s *priv)
{
  uint16_t maxcount = BMI270_WAIT_COUNT_MAX;
  uint8_t regval;

  do
    {
      usleep(BMI270_SET_DELAY);

      /* Performing a dummy read to bring interface back to SPI from I2C. */

      bmi270_spi_read(priv, BMI270_CHIP_ID, (FAR uint8_t *)&regval, 1);

      /* Read the device ID. */

      bmi270_spi_read(priv, BMI270_CHIP_ID, (FAR uint8_t *)&regval, 1);

      maxcount--;
    }
  while ((regval != BMI270_DEVICE_ID) && maxcount);

  if (maxcount == 0)
    {
      snerr("bmi270 reset wait timeout!\n");
    }
}

/****************************************************************************
 * Name: bmi270_write_config
 *
 * Description:
 *   Writes the configuration file.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_write_config(FAR struct bmi270_dev_s *priv)
{
  bmi270_internal_status_t inte_status;
  uint16_t config_size;
  uint16_t bal_byte;
  uint16_t index;
  uint8_t remain;
  int ret = OK;

  /* Get the size of config array. */

  config_size = sizeof(bmi270_config_file);

  /* To get the remainder. */

  remain = (uint8_t)(config_size % BMI270_READWRITE_LEN);

  /* Disable advanced power save mode. */

  bmi270_set_powersave(priv, BMI270_DISABLE);

  usleep(BMI270_POWER_MODE_DELAY);

  /* Write the configuration file. */

  bmi270_set_configload(priv, BMI270_DISABLE);

  if (!remain)
    {
      for (index = 0; index < config_size;
           index = index + BMI270_READWRITE_LEN)
        {
          bmi270_upload_file(priv, bmi270_config_file + index, index,
                             BMI270_READWRITE_LEN);
        }
    }
  else
    {
      bal_byte = config_size - remain;
      for (index = 0; index < bal_byte;
           index = index + BMI270_READWRITE_LEN)
        {
          bmi270_upload_file(priv, bmi270_config_file + index, index,
                             BMI270_READWRITE_LEN);
        }

      for (index = bal_byte; index < config_size;
           index = index + BMI270_READWRITE_RLEN)
        {
          bmi270_upload_file(priv, bmi270_config_file + index, index,
                             BMI270_READWRITE_RLEN);
        }
    }

  bmi270_set_configload(priv, BMI270_ENABLE);

  /* Check the configuration load status */

  bmi270_get_internal_status(priv, (FAR uint8_t *)&inte_status);

  if (!(inte_status.message & BMI270_CONFIG_LOAD_SUCCESS))
    {
      ret = BMI270_CONFIG_LOAD_ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: bmi270_set_powersave
 *
 * Description:
 *   Disable advanced power save mode.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Enable/disable
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_set_powersave(FAR struct bmi270_dev_s *priv,
                                uint8_t value)
{
  bmi270_pwr_conf_t reg;

  bmi270_spi_read(priv, BMI270_PWR_CONF, (FAR uint8_t *)&reg, 1);

  reg.adv_power_save = 0;
  reg.fifo_self_wake_up = value;
  bmi270_spi_write(priv, BMI270_PWR_CONF, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: bmi270_set_configload
 *
 * Description:
 *   Enables/disables the loading of the configuration file.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Enable/disable
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_set_configload(FAR struct bmi270_dev_s *priv,
                                 uint8_t value)
{
  bmi270_init_ctrl_t reg;

  bmi270_spi_read(priv, BMI270_INIT_CTRL, (FAR uint8_t *)&reg, 1);

  reg.value = value;
  bmi270_spi_write(priv, BMI270_INIT_CTRL, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: bmi270_upload_file
 *
 * Description:
 *   Loads the configuration file.
 *
 * Input Parameters:
 *   priv        - Device struct.
 *   config_data - Configuration data buffer address.
 *   index       - Configuration file index.
 *   write_len   - Temporary write length.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_upload_file(FAR struct bmi270_dev_s *priv,
                              FAR uint8_t *config_data,
                              uint16_t index, uint16_t write_len)
{
  uint8_t addr_array[2];

  DEBUGASSERT(config_data != NULL);

  /* Store 0 to 3 bits of address in first byte. */

  addr_array[0] = (uint8_t)((index / 2) & 0x0f);

  /* Store 4 to 11 bits of address in the second byte. */

  addr_array[1] = (uint8_t)((index / 2) >> 4);

  /* Write the 2 bytes of address in consecutive locations. */

  bmi270_spi_write_len(priv, BMI270_INIT_ADDR_0, addr_array, 2);

  /* Burst write configuration file data corresponding to user set length */

  bmi270_spi_write_len(priv, BMI270_INIT_DATA, config_data, write_len);

  return OK;
}

/****************************************************************************
 * Name: bmi270_get_internal_status
 *
 * Description:
 *   Gets error bits and message indicating internal status.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   state - Configuration data buffer address.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_get_internal_status(FAR struct bmi270_dev_s *priv,
                                      FAR uint8_t *state)
{
  /* Wait till ASIC is initialized */

  usleep(BMI270_WAIT_LOAD);

  /* Get the error bits and message */

  bmi270_spi_read(priv, BMI270_INTERNAL_STATUS, state, 1);

  return OK;
}

/****************************************************************************
 * Name: bmi270_set_int
 *
 * Description:
 *   Enable/Disable interrupt on int1.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Enable/Disable.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_set_int(FAR struct bmi270_dev_s *priv,
                          uint8_t value)
{
  bmi270_int1_io_ctrl_t regval_int_ctrl;

  bmi270_spi_read(priv, BMI270_INT1_IO_CTRL,
                  (FAR uint8_t *)&regval_int_ctrl, 1);

  regval_int_ctrl.lvl = 1;
  regval_int_ctrl.od = 0;
  regval_int_ctrl.input_en = 0;
  regval_int_ctrl.output_en = value;
  bmi270_spi_write(priv, BMI270_INT1_IO_CTRL,
                   (FAR uint8_t *)&regval_int_ctrl);

  return OK;
}

/****************************************************************************
 * Name: bmi270_get_feat_config
 *
 * Description:
 *   get the feature configuration from the selected page.
 *
 * Input Parameters:
 *   priv         - Device struct.
 *   page         - Feature page.
 *   feat_config  - Feature config.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_get_feat_config(FAR struct bmi270_dev_s *priv,
                                  uint8_t page, FAR uint8_t *feat_config)
{
  /* Switch page */

  bmi270_spi_write(priv, BMI270_FEAT_PAGE, (FAR uint8_t *)&page);

  /* Read from the page */

  bmi270_spi_read(priv, BMI270_FEATURES, feat_config,
                  BMI270_FEAT_SIZE_IN_BYTES);

  return OK;
}

/* Accelerator handle functions */

/****************************************************************************
 * Name: bmi270_xl_enable
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

static int bmi270_xl_enable(FAR struct bmi270_dev_s *priv,
                            bool enable)
{
  bmi270_pwr_ctrl_t regval_pwr_ctrl;

  bmi270_xl_setfullscale(priv, BMI270_XL_RANGE_8G);
  priv->dev[BMI270_XL_IDX].factor = BMI270_8G_FACTOR;

  bmi270_xl_setfilter(priv, BMI270_XL_OSR4_AVG1);

  /* Enable accelerometer */

  bmi270_spi_read(priv, BMI270_PWR_CTRL, (FAR uint8_t *)&regval_pwr_ctrl, 1);

  if (enable)
    {
      regval_pwr_ctrl.acc_en = BMI270_ENABLE;

      /* Set worker for accelerometer. */

      if (priv->dev[BMI270_XL_IDX].fifoen)
        {
          work_cancel(HPWORK, &priv->dev[BMI270_XL_IDX].work);
        }
      else
        {
          work_queue(HPWORK, &priv->dev[BMI270_XL_IDX].work,
                     bmi270_xl_worker, priv,
                     priv->dev[BMI270_XL_IDX].interval / USEC_PER_TICK);
        }
    }
  else
    {
      /* Set to Shut Down. */

      work_cancel(HPWORK, &priv->dev[BMI270_XL_IDX].work);
      regval_pwr_ctrl.acc_en = BMI270_DISABLE;
    }

  bmi270_spi_write(priv, BMI270_PWR_CTRL, (FAR uint8_t *)&regval_pwr_ctrl);

  return OK;
}

/****************************************************************************
 * Name: bmi270_xl_worker
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

static void bmi270_xl_worker(FAR void *arg)
{
  FAR struct bmi270_dev_s *priv = arg;
  struct sensor_event_accel temp_xl;

  /* Sanity check. */

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp. */

  temp_xl.timestamp = sensor_get_timestamp();

  /* Set work queue. */

  work_queue(HPWORK, &priv->dev[BMI270_XL_IDX].work, bmi270_xl_worker,
             priv, priv->dev[BMI270_XL_IDX].interval / USEC_PER_TICK);

  /* Read out the latest sensor data. */

  bmi270_xl_getdata(priv, BMI270_DATA_8, &temp_xl);

  /* push data to upper half driver. */

  priv->dev[BMI270_XL_IDX].lower.push_event(
        priv->dev[BMI270_XL_IDX].lower.priv,
        &temp_xl,
        sizeof(struct sensor_event_accel));
}

/****************************************************************************
 * Name: bmi270_xl_getdata
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

static int bmi270_xl_getdata(FAR struct bmi270_dev_s *priv,
                             uint8_t regaddr,
                             FAR struct sensor_event_accel *value)
{
  axis3bit16_t temp;

  /* Get accel and gyro data for x, y and z axis. */

  bmi270_spi_read(priv, regaddr, temp.u8bit, 6);

  sensor_remap_vector_raw16(temp.i16bit, temp.i16bit, BMI270_VECTOR_REMAP);

  value->x = (GRAVITY_EARTH * temp.i16bit[0]
           * priv->dev[BMI270_XL_IDX].factor) / BMI270_HALF_SCALE;
  value->y = (GRAVITY_EARTH * temp.i16bit[1]
           * priv->dev[BMI270_XL_IDX].factor) / BMI270_HALF_SCALE;
  value->z = (GRAVITY_EARTH * temp.i16bit[2]
           * priv->dev[BMI270_XL_IDX].factor) / BMI270_HALF_SCALE;

  return OK;
}

/****************************************************************************
 * Name: bmi270_xl_findodr
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

static int bmi270_xl_findodr(FAR float *freq)
{
  int i;
  int num = sizeof(g_bmi270_xl_odr) / sizeof(struct bmi270_odr_s);

  for (i = 0; i < num; i++)
    {
      if (((*freq < g_bmi270_xl_odr[i].odr + BMI270_ODR_FLT_EPSILON)
          && (*freq > g_bmi270_xl_odr[i].odr - BMI270_ODR_FLT_EPSILON))
          || *freq < g_bmi270_xl_odr[i].odr)
        {
          *freq = g_bmi270_xl_odr[i].odr;
          return i;
        }
    }

  return num - 1;
}

/****************************************************************************
 * Name: bmi270_xl_setodr
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

static int bmi270_xl_setodr(FAR struct bmi270_dev_s *priv, uint8_t value)
{
  bmi270_acc_conf_t regval;

  bmi270_spi_read(priv, BMI270_ACC_CONF, (FAR uint8_t *)&regval, 1);

  regval.acc_odr = value;
  bmi270_spi_write(priv, BMI270_ACC_CONF, (FAR uint8_t *)&regval);

  return OK;
}

/****************************************************************************
 * Name: bmi270_xl_setfullscale
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

static int bmi270_xl_setfullscale(FAR struct bmi270_dev_s *priv,
                                  uint8_t value)
{
  bmi270_acc_range_t regval;

  bmi270_spi_read(priv, BMI270_ACC_RANGE, (FAR uint8_t *)&regval, 1);

  regval.acc_range = value;
  bmi270_spi_write(priv, BMI270_ACC_RANGE, (FAR uint8_t *)&regval);

  return OK;
}

/****************************************************************************
 * Name: bmi270_xl_setfilter
 *
 * Description:
 *   Set filter for accelerometer.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Accelerometer Bandwidth parameters.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_xl_setfilter(FAR struct bmi270_dev_s *priv,
                               uint8_t value)
{
  bmi270_acc_conf_t regval;

  bmi270_spi_read(priv, BMI270_ACC_CONF, (FAR uint8_t *)&regval, 1);

  regval.acc_bwp = value;
  regval.acc_filter_perf = BMI270_PERF_OPT_MODE;

  bmi270_spi_write(priv, BMI270_ACC_CONF, (FAR uint8_t *)&regval);

  return OK;
}

/* Gyroscope handle functions */

/****************************************************************************
 * Name: bmi270_get_gyro_crosssense
 *
 * Description:
 *   Gets the cross sensitivity coefficient between gyroscope's X and Z axes.
 *
 * Input Parameters:
 *   priv        - Device struct.
 *   cross_sense - Cross sensitivity coefficient.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_get_gyro_crosssense(FAR struct bmi270_dev_s *priv,
                                      FAR int16_t *cross_sense)
{
  uint8_t feat_config[BMI270_FEAT_SIZE_IN_BYTES];

  bmi270_get_feat_config(priv, BMI270_PAGE_0, feat_config);

  /* discard the MSB as GYR_CAS is of only 7 bit */

  feat_config[BMI270_GYRO_CROSS_SENSE_ADDR]
    = feat_config[BMI270_GYRO_CROSS_SENSE_ADDR] & BMI270_GYRO_CROSS_MASK;

  /* Get the gyroscope cross sensitivity coefficient */

  if (feat_config[BMI270_GYRO_CROSS_SENSE_ADDR]
      & BMI270_GYRO_CROSS_SIGN_MASK)
    {
      *cross_sense
        = (int16_t)(((int16_t)feat_config[BMI270_GYRO_CROSS_SENSE_ADDR])
        - 128);
    }
  else
    {
      *cross_sense = (int16_t)(feat_config[BMI270_GYRO_CROSS_SENSE_ADDR]);
    }

  return OK;
}

/****************************************************************************
 * Name: bmi270_gy_enable
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

static int bmi270_gy_enable(FAR struct bmi270_dev_s *priv,
                            bool enable)
{
  bmi270_pwr_ctrl_t regval_pwr_ctrl;

  bmi270_gy_setfullscale(priv, BMI270_GY_RANGE_2000);
  priv->dev[BMI270_GY_IDX].factor = BMI270_2000DPS_FACTOR
                                  * BMI270_DPS2RPS_FACTOR;

  bmi270_gy_setfilter(priv, BMI270_GY_NORMAL_MODE);

  /* Enable accelerometer */

  bmi270_spi_read(priv, BMI270_PWR_CTRL, (FAR uint8_t *)&regval_pwr_ctrl, 1);

  if (enable)
    {
      regval_pwr_ctrl.gyr_en = BMI270_ENABLE;

      /* Set worker for accelerometer. */

      if (priv->dev[BMI270_GY_IDX].fifoen)
        {
          work_cancel(HPWORK, &priv->dev[BMI270_GY_IDX].work);
        }
      else
        {
          work_queue(HPWORK, &priv->dev[BMI270_GY_IDX].work,
                     bmi270_gy_worker, priv,
                     priv->dev[BMI270_GY_IDX].interval / USEC_PER_TICK);
        }
    }
  else
    {
      /* Set to Shut Down. */

      work_cancel(HPWORK, &priv->dev[BMI270_GY_IDX].work);
      regval_pwr_ctrl.acc_en = BMI270_DISABLE;
    }

  bmi270_spi_write(priv, BMI270_PWR_CTRL, (FAR uint8_t *)&regval_pwr_ctrl);

  return OK;
}

/****************************************************************************
 * Name: bmi270_gy_worker
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

static void bmi270_gy_worker(FAR void *arg)
{
  FAR struct bmi270_dev_s *priv = arg;
  struct sensor_event_gyro temp_gy;

  /* Sanity check. */

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp. */

  temp_gy.timestamp = sensor_get_timestamp();

  /* Set work queue. */

  work_queue(HPWORK, &priv->dev[BMI270_GY_IDX].work, bmi270_gy_worker,
             priv, priv->dev[BMI270_GY_IDX].interval / USEC_PER_TICK);

  /* Read out the latest sensor data. */

  bmi270_gy_getdata(priv, BMI270_DATA_14, &temp_gy);

  /* push data to upper half driver. */

  priv->dev[BMI270_GY_IDX].lower.push_event(
        priv->dev[BMI270_GY_IDX].lower.priv,
        &temp_gy,
        sizeof(struct sensor_event_gyro));
}

/****************************************************************************
 * Name: bmi270_gy_getdata
 *
 * Description:
 *   Read the gyroscope data.
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

static int bmi270_gy_getdata(FAR struct bmi270_dev_s *priv,
                             uint8_t regaddr,
                             FAR struct sensor_event_gyro *value)
{
  axis3bit16_t temp;

  /* Get accel and gyro data for x, y and z axis. */

  bmi270_spi_read(priv, regaddr, temp.u8bit, 6);

  temp.i16bit[0] = temp.i16bit[0] - priv->cross_sense
                 * (temp.i16bit[2]) / (1 << 9);

  sensor_remap_vector_raw16(temp.i16bit, temp.i16bit, BMI270_VECTOR_REMAP);

  value->x = (priv->dev[BMI270_GY_IDX].factor / BMI270_HALF_SCALE)
           * temp.i16bit[0];
  value->y = (priv->dev[BMI270_GY_IDX].factor / BMI270_HALF_SCALE)
           * temp.i16bit[1];
  value->z = (priv->dev[BMI270_GY_IDX].factor / BMI270_HALF_SCALE)
           * temp.i16bit[2];

  return OK;
}

/****************************************************************************
 * Name: bmi270_gy_findodr
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

static int bmi270_gy_findodr(FAR float *freq)
{
  int i;
  int num = sizeof(g_bmi270_gy_odr) / sizeof(struct bmi270_odr_s);

  for (i = 0; i < num; i++)
    {
      if (((*freq < g_bmi270_gy_odr[i].odr + BMI270_ODR_FLT_EPSILON)
          && (*freq > g_bmi270_gy_odr[i].odr - BMI270_ODR_FLT_EPSILON))
          || *freq < g_bmi270_gy_odr[i].odr)
        {
          *freq = g_bmi270_gy_odr[i].odr;
          return i;
        }
    }

  return num - 1;
}

/****************************************************************************
 * Name: bmi270_gy_setodr
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

static int bmi270_gy_setodr(FAR struct bmi270_dev_s *priv, uint8_t value)
{
  bmi270_gyr_conf_t regval;

  bmi270_spi_read(priv, BMI270_GYR_CONF, (FAR uint8_t *)&regval, 1);

  regval.gyr_odr = value;
  bmi270_spi_write(priv, BMI270_GYR_CONF, (FAR uint8_t *)&regval);

  return OK;
}

/****************************************************************************
 * Name: bmi270_gy_setfullscale
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

static int bmi270_gy_setfullscale(FAR struct bmi270_dev_s *priv,
                                  uint8_t value)
{
  bmi270_gyr_range_t regval;

  bmi270_spi_read(priv, BMI270_GYR_RANGE, (FAR uint8_t *)&regval, 1);

  regval.gyr_range = value;
  bmi270_spi_write(priv, BMI270_GYR_RANGE, (FAR uint8_t *)&regval);

  return OK;
}

/****************************************************************************
 * Name: bmi270_gy_setfilter
 *
 * Description:
 *   Set filter for accelerometer.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Accelerometer Bandwidth parameters.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_gy_setfilter(FAR struct bmi270_dev_s *priv,
                               uint8_t value)
{
  bmi270_gyr_conf_t regval;

  bmi270_spi_read(priv, BMI270_GYR_CONF, (FAR uint8_t *)&regval, 1);

  regval.gyr_bwp = value;
  regval.gyr_filter_perf = BMI270_PERF_OPT_MODE;

  bmi270_spi_write(priv, BMI270_ACC_CONF, (FAR uint8_t *)&regval);

  return OK;
}

/* FIFO handle functions */

/****************************************************************************
 * Name: bmi270_fifo_setwatermark
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

static int bmi270_fifo_setwatermark(FAR struct bmi270_dev_s *priv,
                                    unsigned int value)
{
  bmi270_fifo_wtm_0_t fifo_wtm0;
  bmi270_fifo_wtm_1_t fifo_wtm1;

  value = value * BMI270_FIFO_DATA_BLOCK_SIZE;
  if (priv->fifo_buff)
    {
      kmm_free(priv->fifo_buff);
    }

  priv->fifo_buff = kmm_zalloc(value);

  if (priv->fifo_buff == NULL)
    {
      snerr("ERROR: Failed to allocate fifo temp buffer\n");
      return -ENOMEM;
    }

  fifo_wtm0.fifo_water_mark_7_0 = 0x00ff & value;
  fifo_wtm1.fifo_water_mark_12_8 = ((0x1f00 & value) >> 8);

  bmi270_spi_write(priv, BMI270_FIFO_WTM_0,
                   (FAR uint8_t *)&fifo_wtm0);
  bmi270_spi_write(priv, BMI270_FIFO_WTM_1,
                   (FAR uint8_t *)&fifo_wtm1);

  return OK;
}

/****************************************************************************
 * Name: bmi270_fifo_xl_enable
 *
 * Description:
 *   Set FIFO enable/disable of accelerometer.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Enable/disable.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_fifo_xl_enable(FAR struct bmi270_dev_s *priv,
                                 uint8_t value)
{
  bmi270_fifo_config_1_t reg;

  bmi270_spi_read(priv, BMI270_FIFO_CONFIG_1, (FAR uint8_t *)&reg, 1);

  reg.fifo_acc_en = value;
  bmi270_spi_write(priv, BMI270_FIFO_CONFIG_1, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: bmi270_fifo_gy_enable
 *
 * Description:
 *   Set FIFO enable/disable of accelerometer.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Enable/disable.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_fifo_gy_enable(FAR struct bmi270_dev_s *priv,
                                 uint8_t value)
{
  bmi270_fifo_config_1_t reg;

  bmi270_spi_read(priv, BMI270_FIFO_CONFIG_1, (FAR uint8_t *)&reg, 1);

  reg.fifo_gyr_en = value;
  bmi270_spi_write(priv, BMI270_FIFO_CONFIG_1, (FAR uint8_t *)&reg);

  return OK;
}

/****************************************************************************
 * Name: bmi270_fifo_config
 *
 * Description:
 *   Config FIFO.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int bmi270_fifo_config(FAR struct bmi270_dev_s *priv)
{
  bmi270_fifo_config_0_t reg_conf0;
  bmi270_fifo_config_1_t reg_conf1;

  bmi270_spi_read(priv, BMI270_FIFO_CONFIG_1, (FAR uint8_t *)&reg_conf1, 1);

  reg_conf0.fifo_stop_on_full = BMI270_ENABLE;
  reg_conf0.fifo_time_en = BMI270_DISABLE;
  reg_conf1.fifo_tag_int1_en = 0;
  reg_conf1.fifo_header_en = BMI270_ENABLE;
  bmi270_spi_write(priv, BMI270_FIFO_CONFIG_0, (FAR uint8_t *)&reg_conf0);
  bmi270_spi_write(priv, BMI270_FIFO_CONFIG_1, (FAR uint8_t *)&reg_conf1);

  return OK;
}

/****************************************************************************
 * Name: bmi270_fifo_readdata
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

static int bmi270_fifo_readdata(FAR struct bmi270_dev_s *priv)
{
  bmi270_fifo_header_t header;
  axis3bit16_t temp;
  struct sensor_event_accel
         temp_xl[CONFIG_SENSORS_BMI270_FIFO_SLOTS_NUMBER];
  struct sensor_event_gyro
         temp_gy[CONFIG_SENSORS_BMI270_FIFO_SLOTS_NUMBER];
  unsigned int counter_xl = 0;
  unsigned int counter_gy = 0;
  unsigned int fifo_interval;
  unsigned int num;
  unsigned int i;
  int ret;

  memset(temp_xl, 0x00, sizeof(temp_xl));
  memset(temp_gy, 0x00, sizeof(temp_gy));
  memset(priv->fifo_buff, 0, (priv->fifowtm * BMI270_FIFO_DATA_BLOCK_SIZE));

  ret = bmi270_fifo_getlevel(priv, &num);

  if (ret < 0)
    {
      snerr("Failed to get FIFO level!\n");
      return ret;
    }

  if (num > 0)
    {
      bmi270_spi_read_enhance(priv, BMI270_FIFO_DATA, priv->fifo_buff, num);
    }

  for (i = 0; i < num; i = i + BMI270_FIFO_DATA_BLOCK_SIZE)
    {
      header = *((FAR bmi270_fifo_header_t *)&(priv->fifo_buff[i]));

      switch (header.fh_parm)
        {
          case BMI270_FIFO_PARM_XL:    /* Accelerator data tag */
            {
              memcpy(temp.u8bit, &(priv->fifo_buff[i + 1]), 6);
              sensor_remap_vector_raw16(temp.i16bit, temp.i16bit,
                                        BMI270_VECTOR_REMAP);

              temp_xl[counter_xl].x = (GRAVITY_EARTH * temp.i16bit[0]
                                    * priv->dev[BMI270_XL_IDX].factor)
                                    / BMI270_HALF_SCALE;
              temp_xl[counter_xl].y = (GRAVITY_EARTH * temp.i16bit[1]
                                    * priv->dev[BMI270_XL_IDX].factor)
                                    / BMI270_HALF_SCALE;
              temp_xl[counter_xl].z = (GRAVITY_EARTH * temp.i16bit[2]
                                    * priv->dev[BMI270_XL_IDX].factor)
                                    / BMI270_HALF_SCALE;
              counter_xl++;
            }
            break;

          case BMI270_FIFO_PARM_GY:    /* Gyroscope data tag */
            {
              memcpy(temp.u8bit, &(priv->fifo_buff[i + 1]), 6);
              temp.i16bit[0] = temp.i16bit[0] - priv->cross_sense
                             * (temp.i16bit[2]) / (1 << 9);

              sensor_remap_vector_raw16(temp.i16bit, temp.i16bit,
                                        BMI270_VECTOR_REMAP);

              temp_gy[counter_gy].x = (priv->dev[BMI270_GY_IDX].factor
                                    / BMI270_HALF_SCALE) * temp.i16bit[0];
              temp_gy[counter_gy].y = (priv->dev[BMI270_GY_IDX].factor
                                    / BMI270_HALF_SCALE) * temp.i16bit[1];
              temp_gy[counter_gy].z = (priv->dev[BMI270_GY_IDX].factor
                                    / BMI270_HALF_SCALE) * temp.i16bit[2];
              counter_gy++;
            }
            break;

          default:              /* Other data tag */
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

      priv->dev[BMI270_XL_IDX].lower.push_event(
            priv->dev[BMI270_XL_IDX].lower.priv,
            temp_xl,
            sizeof(struct sensor_event_accel) * counter_xl);
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

      priv->dev[BMI270_GY_IDX].lower.push_event(
            priv->dev[BMI270_GY_IDX].lower.priv,
            temp_gy,
            sizeof(struct sensor_event_gyro) * counter_gy);
    }

  priv->timestamp_fifolast = priv->timestamp;

  return ret;
}

/****************************************************************************
 * Name: bmi270_fifo_getlevel
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

static int bmi270_fifo_getlevel(FAR struct bmi270_dev_s *priv,
                                FAR unsigned int *value)
{
  bmi270_fifo_length_0_t fifo_length0;
  bmi270_fifo_length_1_t fifo_length1;

  bmi270_spi_read(priv, BMI270_FIFO_LENGTH_0,
                  (FAR uint8_t *)&fifo_length0, 1);
  bmi270_spi_read(priv, BMI270_FIFO_LENGTH_1,
                  (FAR uint8_t *)&fifo_length1, 1);
  *value = ((uint16_t)fifo_length1.fifo_byte_counter_13_8 << 8) +
           (uint16_t)fifo_length0.fifo_byte_counter_7_0;

  return OK;
}

/* Feature handle functions */

/****************************************************************************
 * Name: bmi270_feat_enable
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

static int bmi270_feat_enable(FAR struct bmi270_dev_s *priv,
                              bool enable)
{
  uint8_t feat_config[BMI270_FEAT_SIZE_IN_BYTES];
  struct bmi270_wakeup_s weakup_config;
  struct bmi270_axes_remap_s axes_map;

  /* Disable advance power save if enabled. */

  bmi270_set_powersave(priv, BMI270_DISABLE);

  /* Remap axis. */

  axes_map.x_axis = BMI270_MAP2X_AXIS;
  axes_map.y_axis = BMI270_MAP2Y_AXIS;
  axes_map.z_axis = BMI270_MAP2Z_AXIS;

  axes_map.x_axis_sign = BMI270_MAP_SIGN_NOT_INVERT;
  axes_map.y_axis_sign = BMI270_MAP_SIGN_NOT_INVERT;
  axes_map.z_axis_sign = BMI270_MAP_SIGN_NOT_INVERT;

  bmi270_set_axis_map(priv, &axes_map);

  /* Enable the wrist wear wake up feature. */

  bmi270_get_feat_config(priv, BMI270_PAGE_7, feat_config);

  if (enable)
    {
      feat_config[BMI270_WRIST_WEAR_WAKE_UP]
        = (feat_config[BMI270_WRIST_WEAR_WAKE_UP]
        & (~BMI270_WRIST_WAKE_UP_EN_MASK))
        | (0x10 & BMI270_WRIST_WAKE_UP_EN_MASK);

      bmi270_spi_write_len(priv, BMI270_FEATURES, feat_config,
                           BMI270_FEAT_SIZE_IN_BYTES);

      /* Enable the wrist wear wake up interrupt. */

      bmi270_set_int(priv, BMI270_ENABLE);
      bmi270_get_wakeup_cfg(priv, &weakup_config);

      /* Sets wrist wear wake-up configurations. */

      weakup_config.max_tilt_pu = 1925;

      bmi270_set_wakeup_cfg(priv, &weakup_config);
      bmi270_map_feat_int(priv, BMI270_ENABLE);

      priv->featen = BMI270_ENABLE;
    }
  else
    {
      feat_config[BMI270_WRIST_WEAR_WAKE_UP]
        = (feat_config[BMI270_WRIST_WEAR_WAKE_UP]
        & (~BMI270_WRIST_WAKE_UP_EN_MASK))
        | (BMI270_DISABLE & BMI270_WRIST_WAKE_UP_EN_MASK);

      bmi270_spi_write_len(priv, BMI270_FEATURES, feat_config,
                           BMI270_FEAT_SIZE_IN_BYTES);

      /* Enable the wrist wear wake up interrupt. */

      bmi270_map_feat_int(priv, BMI270_DISABLE);
      bmi270_set_int(priv, BMI270_DISABLE);

      priv->featen = BMI270_DISABLE;
    }

  return OK;
}

/****************************************************************************
 * Name: bmi270_feat_enable
 *
 * Description:
 *   Maps/unmaps feature interrupts to that of interrupt pins.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - Enable/disable.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi270_map_feat_int(FAR struct bmi270_dev_s *priv,
                               uint8_t value)
{
  bmi270_int1_map_feat_t reg_map_feat_int1;
  bmi270_int2_map_feat_t reg_map_feat_int2;

  bmi270_spi_read(priv, BMI270_INT1_MAP_FEAT,
                  (FAR uint8_t *)&reg_map_feat_int1, 1);
  bmi270_spi_read(priv, BMI270_INT2_MAP_FEAT,
                  (FAR uint8_t *)&reg_map_feat_int2, 1);

  reg_map_feat_int1.wrist_wear_wakeup_out = value;
  reg_map_feat_int2.wrist_wear_wakeup_out = 0;
  bmi270_spi_write(priv, BMI270_INT1_MAP_FEAT,
                   (FAR uint8_t *)&reg_map_feat_int1);
  bmi270_spi_write(priv, BMI270_INT2_MAP_FEAT,
                   (FAR uint8_t *)&reg_map_feat_int2);

  return OK;
}

/****************************************************************************
 * Name: bmi270_get_wakeup_cfg
 *
 * Description:
 *   Get default configurations for the wake up gesture.
 *
 * Input Parameters:
 *   priv          - Device struct.
 *   weakup_config - Config struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi270_get_wakeup_cfg(FAR struct bmi270_dev_s *priv,
                                 FAR struct bmi270_wakeup_s *weakup_config)
{
  uint8_t feat_config[BMI270_FEAT_SIZE_IN_BYTES];
  uint16_t *data_p = (uint16_t *) (void *)feat_config;

  /* Disable advance power save if enabled */

  bmi270_set_powersave(priv, BMI270_DISABLE);

  /* Enable the wrist wear wake up feature. */

  bmi270_get_feat_config(priv, BMI270_PAGE_7, feat_config);

  /* Define the offset in bytes for wrist wear wake-up select */

  weakup_config->idx = BMI270_WRIST_WEAR_WAKE_UP;

  /* Get offset in words since all the features are set in words length */

  weakup_config->idx = weakup_config->idx / 2;

  /* Increment the offset value by 1 word to get config */

  weakup_config->idx++;
  weakup_config->min_angle_focus = *(data_p + weakup_config->idx);

  weakup_config->idx++;
  weakup_config->min_angle_nonfocus = *(data_p + weakup_config->idx);

  weakup_config->idx++;
  weakup_config->max_tilt_lr = *(data_p + weakup_config->idx);

  weakup_config->idx++;
  weakup_config->max_tilt_ll = *(data_p + weakup_config->idx);

  weakup_config->idx++;
  weakup_config->max_tilt_pd = *(data_p + weakup_config->idx);

  weakup_config->idx++;
  weakup_config->max_tilt_pu = *(data_p + weakup_config->idx);

  return OK;
}

/****************************************************************************
 * Name: bmi270_set_wakeup_cfg
 *
 * Description:
 *   Set configurations for the wake up gesture.
 *
 * Input Parameters:
 *   priv          - Device struct.
 *   weakup_config - Config struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi270_set_wakeup_cfg(FAR struct bmi270_dev_s *priv,
                                 FAR struct bmi270_wakeup_s *weakup_config)
{
  uint8_t feat_config[BMI270_FEAT_SIZE_IN_BYTES];
  uint16_t *data_p = (uint16_t *) (void *)feat_config;
  uint8_t index;

  bmi270_get_feat_config(priv, BMI270_PAGE_7, feat_config);

  weakup_config->idx = BMI270_WRIST_WEAR_WAKE_UP;
  weakup_config->idx = weakup_config->idx / 2;

  weakup_config->idx++;
  *(data_p + weakup_config->idx) = weakup_config->min_angle_focus;

  weakup_config->idx++;
  *(data_p + weakup_config->idx) = weakup_config->min_angle_nonfocus;

  weakup_config->idx++;
  *(data_p + weakup_config->idx) = weakup_config->max_tilt_lr;

  weakup_config->idx++;
  *(data_p + weakup_config->idx) = weakup_config->max_tilt_ll;

  weakup_config->idx++;
  *(data_p + weakup_config->idx) = weakup_config->max_tilt_pd;

  weakup_config->idx++;
  *(data_p + weakup_config->idx) = weakup_config->max_tilt_pu;

  weakup_config->idx++;
  weakup_config->idx = (uint8_t)(weakup_config->idx * 2)
                    - BMI270_WRIST_WEAR_WAKE_UP;

  /* Copy the bytes to be set back to the array */

  for (index = 0; index < weakup_config->idx; index++)
    {
      feat_config[BMI270_WRIST_WEAR_WAKE_UP + index]
        = *((FAR uint8_t *) data_p + BMI270_WRIST_WEAR_WAKE_UP + index);
    }

  bmi270_spi_write_len(priv, BMI270_FEATURES, feat_config,
                       BMI270_FEAT_SIZE_IN_BYTES);

  return OK;
}

/****************************************************************************
 * Name: bmi270_set_axis_map
 *
 * Description:
 *   Sets the re-mapped x, y and z axes in the sensor.
 *
 * Input Parameters:
 *   priv     - Device struct.
 *   axes_map - Remap axis config.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi270_set_axis_map(FAR struct bmi270_dev_s *priv,
                               FAR struct bmi270_axes_remap_s *axes_map)
{
  uint8_t feat_config[BMI270_FEAT_SIZE_IN_BYTES];
  bmi270_axes_remap_0_t reg_axes_remap_0;
  bmi270_axes_remap_1_t reg_axes_remap_1;

  bmi270_get_feat_config(priv, BMI270_PAGE_1, feat_config);

  /* Get the axis map. */

  reg_axes_remap_0
    = *((FAR bmi270_axes_remap_0_t *)&feat_config[BMI270_AXIS_MAP]);
  reg_axes_remap_1
    = *((FAR bmi270_axes_remap_1_t *)&feat_config[BMI270_AXIS_MAP + 1]);

  /* Remap the axis. */

  reg_axes_remap_0.map_x_axis = axes_map->x_axis;
  reg_axes_remap_0.map_y_axis = axes_map->y_axis;
  reg_axes_remap_0.map_z_axis = axes_map->z_axis;

  reg_axes_remap_0.map_x_axis_sign = axes_map->x_axis_sign;
  reg_axes_remap_0.map_y_axis_sign = axes_map->y_axis_sign;
  reg_axes_remap_1.map_z_axis_sign = axes_map->z_axis_sign;

  /* Set the axis map. */

  feat_config[BMI270_AXIS_MAP]
    = *((FAR uint8_t *)&reg_axes_remap_0);
  feat_config[BMI270_AXIS_MAP + 1]
    = *((FAR uint8_t *)&reg_axes_remap_1);

  bmi270_spi_write_len(priv, BMI270_FEATURES, feat_config,
                       BMI270_FEAT_SIZE_IN_BYTES);

  return OK;
}

/****************************************************************************
 * Name: bmi270_feat_handler
 *
 * Description:
 *   Feature result handler.
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

static int bmi270_feat_handler(FAR struct bmi270_dev_s *priv,
                               bmi270_int_status_0_t regval_int0)
{
  struct sensor_event_wake_gesture temp_feat;

  temp_feat.timestamp = priv->timestamp;

  if (regval_int0.wrist_wear_weakeup_out)
    {
      temp_feat.event = BMI270_ENABLE;

      /* push data to upper half driver. */

      priv->dev[BMI270_FEAT_IDX].lower.push_event(
            priv->dev[BMI270_FEAT_IDX].lower.priv,
            &temp_feat,
            sizeof(struct sensor_event_wake_gesture));
    }

  return OK;
}

/****************************************************************************
 * Name: bmi270_feat_manage
 *
 * Description:
 *   Manage feature of bmi270. Control feature state by io control.
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

static int bmi270_feat_manage(FAR struct bmi270_dev_s *priv)
{
  int ret = OK;

  DEBUGASSERT(priv != NULL);

  if (priv->featen)
    {
      ret = bmi270_feat_enable(priv, true);
    }
  else
    {
      ret = bmi270_feat_enable(priv, false);
    }

  return ret;
}

/* Sensor ops functions */

/****************************************************************************
 * Name: bmi270_batch
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

static int bmi270_batch(FAR struct sensor_lowerhalf_s *lower,
                        FAR unsigned int *latency_us)
{
  FAR struct bmi270_sensor_s *sensor = (FAR struct bmi270_sensor_s *)lower;
  bmi270_int_map_data_t regval_int_map_data;
  FAR struct bmi270_dev_s * priv;
  uint32_t max_latency;

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

  sensor->fifowtm = BMI270_ROUNDUP(*latency_us, sensor->interval)
                  / sensor->interval;
  *latency_us = sensor->fifowtm * sensor->interval;
  sensor->batch = *latency_us;

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_XL_IDX);
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_GY_IDX);
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  if (sensor->fifowtm > 1)
    {
      sensor->fifoen = true;
      bmi270_set_powersave(priv, BMI270_DISABLE);

      /* Get the fifo start timestamp. */

      priv->timestamp_fifolast = sensor_get_timestamp();

      if (lower->type == SENSOR_TYPE_ACCELEROMETER)
        {
          bmi270_fifo_xl_enable(priv, BMI270_ENABLE);
          work_cancel(HPWORK, &priv->dev[BMI270_XL_IDX].work);
        }
      else if (lower->type == SENSOR_TYPE_GYROSCOPE)
        {
          bmi270_fifo_gy_enable(priv, BMI270_ENABLE);
          work_cancel(HPWORK, &priv->dev[BMI270_GY_IDX].work);
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
      if (lower->type == SENSOR_TYPE_ACCELEROMETER)
        {
          bmi270_fifo_xl_enable(priv, BMI270_DISABLE);
          work_queue(HPWORK, &priv->dev[BMI270_XL_IDX].work,
                     bmi270_xl_worker, priv,
                     priv->dev[BMI270_XL_IDX].interval / USEC_PER_TICK);
        }
      else if (lower->type == SENSOR_TYPE_GYROSCOPE)
        {
          bmi270_fifo_gy_enable(priv, BMI270_DISABLE);
          work_queue(HPWORK, &priv->dev[BMI270_GY_IDX].work,
                     bmi270_gy_worker, priv,
                     priv->dev[BMI270_GY_IDX].interval / USEC_PER_TICK);
        }
      else
        {
          snerr("Failed to match sensor type.\n");
          return -EINVAL;
        }

      sensor->fifoen = false;
    }

  /* Map interrupt to int1. */

  bmi270_spi_read(priv, BMI270_INT_MAP_DATA,
                  (FAR uint8_t *)&regval_int_map_data, 1);

  if (priv->dev[BMI270_XL_IDX].fifoen == false
      && priv->dev[BMI270_GY_IDX].fifoen == false)
    {
      /* Read the remaining FIFO data. */

      if (priv->fifoen)
        {
          bmi270_fifo_readdata(priv);
        }

      bmi270_set_int(priv, BMI270_DISABLE);

      priv->fifoen = false;
      regval_int_map_data.ffull_int1 = BMI270_DISABLE;
      regval_int_map_data.err_int1 = BMI270_DISABLE;
      regval_int_map_data.fwm_int1 = BMI270_DISABLE;
    }
  else
    {
      bmi270_set_int(priv, BMI270_ENABLE);

      priv->fifoen = true;
      regval_int_map_data.ffull_int1 = BMI270_ENABLE;
      regval_int_map_data.err_int1 = BMI270_ENABLE;
      regval_int_map_data.fwm_int1 = BMI270_ENABLE;

      if (priv->dev[BMI270_XL_IDX].fifowtm
          > priv->dev[BMI270_XL_IDX].lower.batch_number)
        {
          priv->dev[BMI270_XL_IDX].fifowtm
          = priv->dev[BMI270_XL_IDX].lower.batch_number;
        }

      if (priv->dev[BMI270_GY_IDX].fifowtm
          > priv->dev[BMI270_GY_IDX].lower.batch_number)
        {
          priv->dev[BMI270_GY_IDX].fifowtm
          = priv->dev[BMI270_GY_IDX].lower.batch_number;
        }

      priv->fifowtm = priv->dev[BMI270_XL_IDX].fifowtm
                    + priv->dev[BMI270_GY_IDX].fifowtm;

      bmi270_fifo_setwatermark(priv, priv->fifowtm);
    }

  bmi270_fifo_config(priv);

  /* Set interrupt route. */

  bmi270_spi_write(priv, BMI270_INT_MAP_DATA,
                   (FAR uint8_t *)&regval_int_map_data);

  return OK;
}

/****************************************************************************
 * Name: bmi270_set_interval
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

static int bmi270_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR unsigned int *period_us)
{
  FAR struct bmi270_sensor_s *sensor = (FAR struct bmi270_sensor_s *)lower;
  FAR struct bmi270_dev_s * priv;
  float freq;
  int ret;
  int idx;

  /* Sanity check. */

  DEBUGASSERT(sensor != NULL && period_us != NULL);

  freq = BMI270_UNIT_TIME / *period_us;

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_XL_IDX);

      /* Find the period that matches best.  */

      idx = bmi270_xl_findodr(&freq);
      ret = bmi270_xl_setodr(priv, g_bmi270_xl_odr[idx].regval);
      if (ret < 0)
        {
          snerr("Failed to set interval: %d\n", ret);
          return ret;
        }

      *period_us = BMI270_UNIT_TIME / freq;
      priv->dev[BMI270_XL_IDX].interval = *period_us;
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_GY_IDX);

      /* Find the period that matches best.  */

      idx = bmi270_gy_findodr(&freq);
      ret = bmi270_gy_setodr(priv, g_bmi270_gy_odr[idx].regval);
      if (ret < 0)
        {
          snerr("Failed to set interval: %d\n", ret);
          return ret;
        }

      *period_us = BMI270_UNIT_TIME / freq;
      priv->dev[BMI270_GY_IDX].interval = *period_us;
    }
  else if (lower->type == SENSOR_TYPE_WAKE_GESTURE)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_FEAT_IDX);
      ret = OK;
    }
  else
    {
      snerr("Failed to match sensor type: %d\n", -EINVAL);
      return -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: bmi270_activate
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

static int bmi270_activate(FAR struct sensor_lowerhalf_s *lower,
                           bool enable)
{
  FAR struct bmi270_sensor_s *sensor = (FAR struct bmi270_sensor_s *)lower;
  FAR struct bmi270_dev_s * priv;
  int ret = OK;

  DEBUGASSERT(lower != NULL);

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_XL_IDX);
      if (sensor->activated != enable)
        {
          ret = bmi270_xl_enable(priv, enable);
          if (ret < 0)
            {
              snerr("Failed to enable accelerometer sensor: %d\n", ret);
              return ret;
            }

          sensor->activated = enable;
        }
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_GY_IDX);
      if (sensor->activated != enable)
        {
          ret = bmi270_gy_enable(priv, enable);
          if (ret < 0)
            {
              snerr("Failed to enable gyroscope sensor: %d\n", ret);
              return ret;
            }

          sensor->activated = enable;
        }
    }
  else if (lower->type == SENSOR_TYPE_WAKE_GESTURE)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_FEAT_IDX);
      if (sensor->activated != enable)
        {
          if (enable)
            {
              IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                              IOEXPANDER_OPTION_INTCFG,
                              (FAR void *)IOEXPANDER_VAL_RISING);
            }

          ret = bmi270_feat_enable(priv, enable);
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

  if (priv->dev[BMI270_FEAT_IDX].activated == false
      && priv->dev[BMI270_XL_IDX].fifoen == false
      && priv->dev[BMI270_GY_IDX].fifoen == false)
    {
      IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                      IOEXPANDER_OPTION_INTCFG,
                      (FAR void *)IOEXPANDER_VAL_DISABLE);
    }

  return ret;
}

/****************************************************************************
 * Name: bmi270_selftest
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

static int bmi270_selftest(FAR struct sensor_lowerhalf_s *lower,
                           unsigned long arg)
{
  FAR struct bmi270_sensor_s *sensor = (FAR struct bmi270_sensor_s *)lower;
  FAR struct bmi270_dev_s * priv;
  int ret;

  DEBUGASSERT(lower != NULL);

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_XL_IDX);
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_GY_IDX);
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  /* Process ioctl commands. */

  switch (arg)
    {
      case BMI270_SIMPLE_CHECK:    /* Simple check tag */
        {
          /* Read device ID. */

          ret = bmi270_readdevid(priv);
        }
        break;

      case BMI270_FULL_CHECK:      /* Full check tag */
        {
          /* Run selftest. */

          ret = bmi270_datatest(priv, lower->type);
        }
        break;

      default:                     /* Other cmd tag */
        {
          ret = -ENOTTY;
          snerr("ERROR: The cmd don't support: %d\n", ret);
        }
        break;
    }

    return ret;
}

/****************************************************************************
 * Name: bmi270_control
 *
 * Description:
 *   With this method, the user can set some special config for the sensor,
 *   such as changing the custom mode, setting the custom resolution, reset,
 *   etc, which are all parsed and implemented by lower half driver.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
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

static int bmi270_control(FAR struct sensor_lowerhalf_s *lower,
                          int cmd, unsigned long arg)
{
  FAR struct bmi270_sensor_s *sensor = (FAR struct bmi270_sensor_s *)lower;
  FAR struct bmi270_dev_s * priv;
  int ret;

  DEBUGASSERT(lower != NULL);

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_XL_IDX);
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_GY_IDX);
    }
  else if (lower->type == SENSOR_TYPE_WAKE_GESTURE)
    {
      priv = (struct bmi270_dev_s *)(sensor - BMI270_FEAT_IDX);
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  /* Process ioctl commands. */

  switch (cmd)
    {
      case BMI270_FEAT_MANAGE_CMD:    /* Feature manage cmd tag */
        {
          priv->featen = (unsigned int)arg;
          ret = bmi270_feat_manage(priv);
          if (ret < 0)
            {
              snerr("ERROR: Failed to set feature: %d\n", ret);
            }
        }
        break;

      case BMI270_SET_SCALE_XL_CMD:   /* Set accelerator scale command tag */
        {
          switch (arg)
          {
            case BMI270_XL_SET_2G:
              {
                ret = bmi270_xl_setfullscale(priv, BMI270_XL_RANGE_2G);
                priv->dev[BMI270_XL_IDX].factor = BMI270_2G_FACTOR;
              }
              break;

            case BMI270_XL_SET_4G:
              {
                ret = bmi270_xl_setfullscale(priv, BMI270_XL_RANGE_4G);
                priv->dev[BMI270_XL_IDX].factor = BMI270_4G_FACTOR;
              }
              break;

            case BMI270_XL_SET_8G:
              {
                ret = bmi270_xl_setfullscale(priv, BMI270_XL_RANGE_8G);
                priv->dev[BMI270_XL_IDX].factor = BMI270_8G_FACTOR;
              }
              break;

            case BMI270_XL_SET_16G:
              {
                ret = bmi270_xl_setfullscale(priv, BMI270_XL_RANGE_16G);
                priv->dev[BMI270_XL_IDX].factor = BMI270_16G_FACTOR;
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
          snerr("ERROR: The cmd don't support: %d\n", ret);
        }
        break;
    }

    return ret;
}

/* Sensor interrupt functions */

/****************************************************************************
 * Name: bmi270_interrupt_handler
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

static int bmi270_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the BMI270
   * new data interrupt pin since it signals that new data has
   * been measured.
   */

  FAR struct bmi270_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp. */

  priv->timestamp = sensor_get_timestamp();

  /* Task the worker with retrieving the latest sensor data. We should not
   * do this in a interrupt since it might take too long. Also we cannot lock
   * the SPI bus from within an interrupt.
   */

  work_queue(LPWORK, &priv->work, bmi270_worker, priv, 0);
  IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);

  return OK;
}

/****************************************************************************
 * Name: bmi270_worker
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

static void bmi270_worker(FAR void *arg)
{
  FAR struct bmi270_dev_s *priv = arg;
  bmi270_int_status_0_t regval_int0;
  bmi270_int_status_1_t regval_int1;

  /* Sanity check. */

  DEBUGASSERT(priv != NULL);

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_RISING);

  bmi270_spi_read(priv, BMI270_INT_STATUS_0, (FAR uint8_t *)&regval_int0, 1);
  bmi270_spi_read(priv, BMI270_INT_STATUS_1, (FAR uint8_t *)&regval_int1, 1);

  if (regval_int0.wrist_wear_weakeup_out)
    {
      bmi270_feat_handler(priv, regval_int0);
    }

  if (regval_int1.fwm_int || regval_int1.ffull_int)
    {
      bmi270_fifo_readdata(priv);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi270_register
 *
 * Description:
 *   Register the BMI270 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   i2c - An instance of the I2C interface to use to communicate with
 *         BMI270
 *   addr - The I2C address of the BMI270.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int bmi270_register(int devno, FAR const struct bmi270_config_s *config)
{
  FAR struct bmi270_dev_s *priv;
  FAR void *ioephandle;
  int ret;

  /* Sanity check. */

  DEBUGASSERT(config != NULL);

  /* Initialize the BMI270 device structure. */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config = config;

  priv->dev[BMI270_XL_IDX].lower.ops = &g_bmi270_xl_ops;
  priv->dev[BMI270_XL_IDX].lower.type = SENSOR_TYPE_ACCELEROMETER;
  priv->dev[BMI270_XL_IDX].lower.uncalibrated = true;
  priv->dev[BMI270_XL_IDX].interval = BMI270_DEFAULT_INTERVAL;
  priv->dev[BMI270_XL_IDX].lower.buffer_number
                            = CONFIG_SENSORS_BMI270_BUFFER_NUMBER;
  priv->dev[BMI270_XL_IDX].lower.batch_number
                            = CONFIG_SENSORS_BMI270_FIFO_SLOTS_NUMBER;

  priv->dev[BMI270_GY_IDX].lower.ops = &g_bmi270_gy_ops;
  priv->dev[BMI270_GY_IDX].lower.type = SENSOR_TYPE_GYROSCOPE;
  priv->dev[BMI270_GY_IDX].lower.uncalibrated = true;
  priv->dev[BMI270_GY_IDX].interval = BMI270_DEFAULT_INTERVAL;
  priv->dev[BMI270_GY_IDX].lower.buffer_number
                            = CONFIG_SENSORS_BMI270_BUFFER_NUMBER;
  priv->dev[BMI270_GY_IDX].lower.batch_number
                            = CONFIG_SENSORS_BMI270_FIFO_SLOTS_NUMBER;

  priv->featen = BMI270_DEFAULT_FSM_EN;
  priv->dev[BMI270_FEAT_IDX].lower.ops = &g_bmi270_fsm_ops;
  priv->dev[BMI270_FEAT_IDX].lower.type = SENSOR_TYPE_WAKE_GESTURE;
  priv->dev[BMI270_FEAT_IDX].lower.uncalibrated = true;
  priv->dev[BMI270_FEAT_IDX].interval = BMI270_DEFAULT_INTERVAL;
  priv->dev[BMI270_FEAT_IDX].lower.buffer_number
                            = CONFIG_SENSORS_BMI270_BUFFER_NUMBER;
  priv->dev[BMI270_FEAT_IDX].lower.batch_number
                            = CONFIG_SENSORS_BMI270_FIFO_SLOTS_NUMBER;

  /* Wait sensor boot time. */

  usleep(BMI270_WAIT_TIME);

  /* Read the deviceID. */

  ret = bmi270_readdevid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to get DeviceID: %d\n", ret);
      goto err;
    }

  /* Reset devices. */

  bmi270_reset(priv);
  bmi270_resetwait(priv);

  ret = bmi270_write_config(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to load config file: %d\n", ret);
      goto err;
    }

  /* Get the gyroscope cross axis sensitivity */

  bmi270_get_gyro_crosssense(priv, &(priv->cross_sense));

  /* Interrupt register. */

  ret = IOEXP_SETDIRECTION(priv->config->ioedev, priv->config->pin,
                           IOEXPANDER_DIRECTION_IN_PULLDOWN);
  if (ret < 0)
    {
      snerr("Failed to set direction: %d\n", ret);
      goto err;
    }

  ioephandle = IOEP_ATTACH(priv->config->ioedev, priv->config->pin,
                           bmi270_interrupt_handler, priv);
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

  ret = sensor_register((&(priv->dev[BMI270_XL_IDX].lower)), devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto err_detach;
    }

  ret = sensor_register((&(priv->dev[BMI270_GY_IDX].lower)), devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      sensor_unregister((&(priv->dev[BMI270_XL_IDX].lower)), devno);
      goto err_detach;
    }

  ret = sensor_register((&(priv->dev[BMI270_FEAT_IDX].lower)), devno);
  if (ret < 0)
    {
      sensor_unregister((&(priv->dev[BMI270_XL_IDX].lower)), devno);
      sensor_unregister((&(priv->dev[BMI270_GY_IDX].lower)), devno);
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto err_detach;
    }

  return ret;

err_detach:
  IOEP_DETACH(priv->config->ioedev, bmi270_interrupt_handler);

err:
  kmm_free(priv);
  return ret;
}
