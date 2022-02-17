/****************************************************************************
 * include/nuttx/sensors/lis2dh.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LIS2DH_H
#define __INCLUDE_NUTTX_SENSORS_LIS2DH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#define ST_LIS2DH_WHOAMI_VALUE          0x33    /* Valid WHOAMI register value */

/* LIS2DH Internal Registers ************************************************/

#define ST_LIS2DH_WHOAMI_REG            0x0f    /* WHOAMI register */

#define ST_LIS2DH_STATUS_AUX_REG        0x07    /* Temperature status */
#define ST_LIS2DH_OUT_TEMP_L_REG        0x0c    /* Temperature data */
#define ST_LIS2DH_OUT_TEMP_H_REG        0x0d    /* Temperature data */

#define ST_LIS2DH_TEMP_CFG_REG          0x1f

#define ST_LIS2DH_CTRL_REG1             0x20

/* CR1 ODR 4 MSBs (XXXX---) */

#define ST_LIS2DH_CR1_ODR_PWR_DWN       0x00
#define ST_LIS2DH_CR1_ODR_1HZ           0x10   /* HR / Normal / Low Power */
#define ST_LIS2DH_CR1_ODR_10HZ          0x20   /* HR / Normal / Low Power */
#define ST_LIS2DH_CR1_ODR_25HZ          0x30   /* HR / Normal / Low Power */
#define ST_LIS2DH_CR1_ODR_50HZ          0x40   /* HR / Normal / Low Power */
#define ST_LIS2DH_CR1_ODR_100HZ         0x50   /* HR / Normal / Low Power */
#define ST_LIS2DH_CR1_ODR_200HZ         0x60   /* HR / Normal / Low Power */
#define ST_LIS2DH_CR1_ODR_400HZ         0x70   /* HR / Normal / Low Power */
#define ST_LIS2DH_CR1_ODR_1620HZ        0x80   /* Low Power */
#define ST_LIS2DH_CR1_ODR_1344_5376HZ   0x90   /* HR / Normal: 1344Hz, Low power: 5376Hz*/
#define ST_LIS2DH_CR1_LOWP_ENABLE       0x08   /* Low power mode enable */
#define ST_LIS2DH_CR1_ZEN               0x04   /* Z-Axis Enable */
#define ST_LIS2DH_CR1_YEN               0x02   /* Y-Axis Enable */
#define ST_LIS2DH_CR1_XEN               0x01   /* X-Axis Enable */

#define ST_LIS2DH_CTRL_REG2             0x21

/* HPM1 .... HP FILT_MODE (XX------) */

#define ST_LIS2DH_CR2_HPFILT_M_NORM     0x00   /* Normal mode (reset reading REFERENCE/DATACAPTURE (26h) register) */
#define ST_LIS2DH_CR2_HPFILT_M_REFSIG   0x40   /* Reference signal for filtering */
#define ST_LIS2DH_CR2_HPFILT_M_NORM2    0x80   /* Normal mode */
#define ST_LIS2DH_CR2_HPFILT_M_AUTOR    0xc0   /* Autoreset on interrupt event */
#define ST_LIS2DH_CR2_FDS               0x08

/* HPIS1 HPFILT ENABLE for INT1 (-------X) */

#define ST_LIS2DH_CR2_HPENABLED_INT1    0x01   /* HP filter enabled for INT1 */

/* HPIS2 HPFILT ENABLE for INT2 (------X-) */

#define ST_LIS2DH_CR2_HPENABLED_INT2    0x02   /* HP filter enabled for INT2 */

#define ST_LIS2DH_CTRL_REG3             0x22

/* I1_AOI1 ENABLE for INT2 (-X------) */

#define ST_LIS2DH_CR3_I1_AOI1_ENABLED   0x40   /* AOI1 interrupt on INT1 pin. */
#define ST_LIS2DH_CR3_I1_AOI2_ENABLED   0x20   /* AOI2 interrupt on INT1 pin. */
#define ST_LIS2DH_CR3_I1_DRDY1          0x10   /* DRDY1 interrupt on INT1 pin. */
#define ST_LIS2DH_CR3_I1_DRDY2          0x08   /* DRDY2 interrupt on INT1 pin. */
#define ST_LIS2DH_CR3_I1_WTM            0x04   /* FIFO Watermark interrupt on INT1 pin. */
#define ST_LIS2DH_CR3_I1_OVERRUN        0x02   /* FIFO Overrun interrupt on INT1 pin. */

#define ST_LIS2DH_CTRL_REG4             0x23

/* BDU ... Block Data Update (X-------) */

#define ST_LIS2DH_CR4_BDU_CONT          0x00   /* Continuous update (Default) */
#define ST_LIS2DH_CR4_BDU_UPD_ON_READ   0x80   /* Output registers not updated until MSB and LSB have been read */
#define ST_LIS2DH_CR4_FULL_SCALE_2G     0x0
#define ST_LIS2DH_CR4_FULL_SCALE_4G     0x10
#define ST_LIS2DH_CR4_FULL_SCALE_8G     0x20
#define ST_LIS2DH_CR4_FULL_SCALE_16G    0x30

/* HR .. Operation mode selector (----X---) */

#define ST_LIS2DH_CR4_HR_ENABLED        0x08   /* See section 2.6.3 in datasheet */

#define ST_LIS2DH_CTRL_REG5             0x24
#define ST_LIS2DH_CR5_BOOT              0x80
#define ST_LIS2DH_CR5_FIFO_EN           0x40
#define ST_LIS2DH_CR5_LIR_INT1          0x08
#define ST_LIS2DH_CR5_D4D_INT1          0x04
#define ST_LIS2DH_CR5_LIR_INT2          0x02
#define ST_LIS2DH_CR5_D4D_INT2          0x01

#define ST_LIS2DH_CTRL_REG6             0x25
#define ST_LIS2DH_REFERENCE_REG         0x26

#define ST_LIS2DH_STATUS_REG            0x27    /* Status Register */
#define ST_LIS2DH_SR_ZYXOR              0x80    /* OR'ed X,Y and Z data over-run  */
#define ST_LIS2DH_SR_ZOR                0x40    /* individual data over-run ... */
#define ST_LIS2DH_SR_YOR                0x20
#define ST_LIS2DH_SR_XOR                0x10
#define ST_LIS2DH_SR_ZYXDA              0x08    /* OR'ed X,Y and Z data available */
#define ST_LIS2DH_SR_ZDA                0x04    /* individual data available ... */
#define ST_LIS2DH_SR_YDA                0x02
#define ST_LIS2DH_SR_XDA                0x01

#define ST_LIS2DH_OUT_X_L_REG           0x28
#define ST_LIS2DH_OUT_X_H_REG           0x29
#define ST_LIS2DH_OUT_Y_L_REG           0x2a
#define ST_LIS2DH_OUT_Y_H_REG           0x2b
#define ST_LIS2DH_OUT_Z_L_REG           0x2c
#define ST_LIS2DH_OUT_Z_H_REG           0x2d

#define ST_LIS2DH_FIFO_CTRL_REG         0x2e
#define ST_LIS2DH_FIFOCR_THRESHOLD_MASK 0x1f
#define ST_LIS2DH_FIFOCR_THRESHOLD(x)   ((x) & ST_LIS2DH_FIFOCR_THRESHOLD_MASK)
#define ST_LIS2DH_FIFOCR_INT1           0x00
#define ST_LIS2DH_FIFOCR_INT2           0x20
#define ST_LIS2DH_FIFOCR_MODE_MASK      0xc0

#define ST_LIS2DH_FIFO_SRC_REG          0x2f
#define ST_LIS2DH_FIFOSR_NUM_SAMP_MASK  0x1f
#define ST_LIS2DH_FIFOSR_EMPTY          0x20
#define ST_LIS2DH_FIFOSR_OVRN_FIFO      0x40
#define ST_LIS2DH_FIFOSR_WTM            0x80

#define ST_LIS2DH_INT1_CFG_REG          0x30
#define ST_LIS2DH_INT_CFG_AOI           0x80
#define ST_LIS2DH_INT_CFG_6D            0x40
#define ST_LIS2DH_INT_CFG_ZHIE          0x20
#define ST_LIS2DH_INT_CFG_ZLIE          0x10
#define ST_LIS2DH_INT_CFG_YHIE          0x08
#define ST_LIS2DH_INT_CFG_YLIE          0x04
#define ST_LIS2DH_INT_CFG_XHIE          0x02
#define ST_LIS2DH_INT_CFG_XLIE          0x01

#define ST_LIS2DH_INT1_SRC_REG          0x31
#define ST_LIS2DH_INT_SR_XLOW           0x01
#define ST_LIS2DH_INT_SR_XHIGH          0x02
#define ST_LIS2DH_INT_SR_YLOW           0x04
#define ST_LIS2DH_INT_SR_YHIGH          0x08
#define ST_LIS2DH_INT_SR_ZLOW           0x10
#define ST_LIS2DH_INT_SR_ZHIGH          0x20
#define ST_LIS2DH_INT_SR_ACTIVE         0x40

#define ST_LIS2DH_INT1_THS_REG          0x32    /* 7-bit value for threshold */

#define ST_LIS2DH_INT1_DUR_REG          0x33    /* 7-bit value for duration */

#define ST_LIS2DH_INT2_CFG_REG          0x34

#define ST_LIS2DH_INT2_SRC_REG          0x35

#define ST_LIS2DH_INT2_THS_REG          0x36    /* 7-bit value for threshold */

#define ST_LIS2DH_INT2_DUR_REG          0x37    /* 7-bit value for duration */

#define ST_LIS2DH_CLICK_CFG_REG         0x38

#define ST_LIS2DH_CLICK_SRC_REG         0x39

#define ST_LIS2DH_CLICK_THS_REG         0x3a

#define ST_LIS2DH_TIME_LIMIT_REG        0x3b

#define ST_LIS2DH_TIME_LATENCY_REG      0x3c

#define ST_LIS2DH_TIME_WINDOW_REG       0x3d

#define ST_LIS2DH_ACT_DUR_REG           0x3f

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum lis2dh_ouput_data_rate
{
  LIS2DH_ODR_POWER_DOWN = 0x00,
  LIS2DH_ODR_1HZ = 0x10,
  LIS2DH_ODR_10HZ = 0x20,
  LIS2DH_ODR_25HZ = 0x30,
  LIS2DH_ODR_50HZ = 0x40,
  LIS2DH_ODR_100HZ = 0x50,
  LIS2DH_ODR_200HZ = 0x60,
  LIS2DH_ODR_400HZ = 0x70,
  LIS2DH_ODR_1620HZ = 0x80,
  LIS2DH_ODR_5376HZ = 0x90,
};

enum lis2dh_high_pass_filter_mode
{
  LIS2DH_REFERENCE_SIGNAL = 0x40,
  LIS2DH_NORMAL_MODE = 0x80,
  LIS2DH_AUTORESET_ON_INTERRUPT = 0xc0,
};

enum lis2dh_scale_range
{
  LIS2DH_RANGE_2G = 0x00,
  LIS2DH_RANGE_4G = 0x10,
  LIS2DH_RANGE_8G = 0x20,
  LIS2DH_RANGE_16G = 0x30,
};

enum lis2dh_self_test
{
  LIS2DH_NORMAL = 0x00,
  LIS2DH_SELF_TEST0 = 0x02,
  LIS2DH_SELF_TEST1 = 0x04,
};

enum lis2dh_fifo_mode
{
  LIS2DH_BYPASS_MODE = 0x00,
  LIS2DH_FIFO_MODE = 0x40,
  LIS2DH_STREAM_MODE = 0x80,
  LIS2DH_TRIGGER_MODE = 0xc0,
};

enum lis2dh_interrupt_mode
{
  LIS2DH_OR_COMBINATION = 0x00,
  LIS2DH_6D_MOVEMENT = 0x40,
  LIS2DH_AND_COMBINATION = 0x80,
  LIS2DH_6D_POSITION = 0xc0,
};

begin_packed_struct struct lis2dh_vector_s
{
  int16_t x;
  int16_t y;
  int16_t z;
} end_packed_struct;

begin_packed_struct struct lis2dh_res_header
{
  uint8_t meas_count;
  bool    int1_occurred;
  uint8_t int1_source;
  bool    int2_occurred;
  uint8_t int2_source;
} end_packed_struct;

begin_packed_struct struct lis2dh_result
{
  struct lis2dh_res_header header;
  struct lis2dh_vector_s measurements[0];
} end_packed_struct;

struct lis2dh_setup
{
  bool temp_enable:1;
  bool xy_axis_fixup:1;

  uint8_t data_rate;
  uint8_t low_power_mode_enable;
  uint8_t zen;
  uint8_t yen;
  uint8_t xen;

  uint8_t hpmode;
  uint8_t hpcf;
  uint8_t fds;
  uint8_t hpclick;
  uint8_t hpis2;
  uint8_t hpis1;

  uint8_t int1_click_enable;
  uint8_t int1_aoi_enable;
  uint8_t int2_aoi_enable;
  uint8_t int1_drdy_enable;
  uint8_t int2_drdy_enable;
  uint8_t int_wtm_enable;
  uint8_t int_overrun_enable;

  uint8_t bdu;
  uint8_t endian;
  uint8_t fullscale;
  uint8_t high_resolution_enable;
  uint8_t selftest;
  uint8_t spi_mode;

  uint8_t reboot;
  uint8_t fifo_enable;
  uint8_t int1_latch;
  uint8_t int1_4d_enable;
  uint8_t int2_latch;
  uint8_t int2_4d_enable;

  uint8_t int2_click_enable;
  uint8_t int_enable;
  uint8_t boot_int1_enable;
  uint8_t high_low_active;

  uint8_t reference;

  uint8_t fifo_mode;
  uint8_t trigger_selection;
  uint8_t fifo_trigger_threshold;

  uint8_t int1_interrupt_mode;
  uint8_t int1_enable_6d;
  uint8_t int1_int_z_high_enable;
  uint8_t int1_int_z_low_enable;
  uint8_t int1_int_y_high_enable;
  uint8_t int1_int_y_low_enable;
  uint8_t int1_int_x_high_enable;
  uint8_t int1_int_x_low_enable;
  uint8_t int1_int_threshold;
  uint8_t int1_int_duration;

  uint8_t int2_interrupt_mode;
  uint8_t int2_enable_6d;
  uint8_t int2_int_z_high_enable;
  uint8_t int2_int_z_low_enable;
  uint8_t int2_int_y_high_enable;
  uint8_t int2_int_y_low_enable;
  uint8_t int2_int_x_high_enable;
  uint8_t int2_int_x_low_enable;
  uint8_t int2_int_threshold;
  uint8_t int2_int_duration;

  uint8_t z_double_click_enable;
  uint8_t z_single_click_enable;
  uint8_t y_double_click_enable;
  uint8_t y_single_click_enable;
  uint8_t x_double_click_enable;
  uint8_t x_single_click_enable;

  uint8_t click_threshold;
  uint8_t click_time_limit;
  uint8_t click_time_latency;
  uint8_t click_time_window;
};

struct lis2dh_config_s
{
  /* Device characterization */

  int      irq;           /* IRQ number received by interrupt handler. */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the lis2dh driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * irq_attach - Attach the lis2dh interrupt handler to the GPIO interrupt
   * irq_enable - Enable or disable the GPIO interrupt
   * clear_irq  - Acknowledge/clear any pending GPIO interrupt
   *
   */

  CODE int  (*irq_attach)(FAR struct lis2dh_config_s *state,
                          xcpt_t isr,
                          FAR void *arg);
  CODE void (*irq_enable)(FAR const struct lis2dh_config_s *state,
                          bool enable);
  CODE void (*irq_clear)(FAR const struct lis2dh_config_s *state);
  CODE bool (*read_int1_pin)(void);
  CODE bool (*read_int2_pin)(void);
};

begin_packed_struct struct lis2dh_raw_data_s
{
  uint16_t out_x;
  uint16_t out_y;
  uint16_t out_z;
} end_packed_struct;

typedef struct lis2dh_raw_data_s lis2dh_raw_data_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lis2dh_register
 *
 * Description:
 *  Register the LIS2DH character device as 'devpath'
 *
 * Input Parameters:
 *  devpath - The full path to the driver to register. E.g., "/dev/acc0"
 *  i2c     - An instance of the I2C interface to use to communicate with
 *            LIS2DH
 *  addr    - The I2C address of the LIS2DH. The base I2C address of the
 *            LIS2DH is 0x18.
 *            Bit 0 can be controlled via SA0 pad - when connected to
 *            voltage supply the address is 0x19.
 *  config  - Pointer to LIS2DH configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis2dh_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, FAR struct lis2dh_config_s *config);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_LIS2DH_H */
