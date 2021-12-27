/******************************************************************************
 *
 * Copyright (c) 2004 Asahi Kasei Microdevices Corporation, Japan
 * All Rights Reserved.
 *
 * This software program is the proprietary program of Asahi Kasei Microdevices
 * Corporation("AKM") licensed to authorized Licensee under the respective
 * agreement between the Licensee and AKM only for use with AKM's electronic
 * compass IC.
 *
 * THIS SOFTWARE IS PROVIDED TO YOU "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABLITY, FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT OF
 * THIRD PARTY RIGHTS, AND WE SHALL NOT BE LIABLE FOR ANY LOSSES AND DAMAGES
 * WHICH MAY OCCUR THROUGH USE OF THIS SOFTWARE.
 *
 ******************************************************************************/
#ifndef __AKM_WRAPPER_H
#define __AKM_WRAPPER_H

typedef struct magCaliDataOutPut ak09919c_cal_output_t;
typedef struct magCaliDataInPut ak09919c_cal_input_t;

struct magCaliDataOutPut
{
  int64_t timeStamp;
  float x;
  float y;
  float z;
  float x_bias;
  float y_bias;
  float z_bias;
  int8_t status;
};

struct magCaliDataInPut
{
  int64_t timeStamp;
  float x;
  float y;
  float z;
  int32_t status;
};

struct magChipInfo
{
  int32_t layout;
  int32_t deviceid;
  int8_t  hwGyro;
};

int akm_wrapper_lib_init(void);
int akm_wrapper_lib_enable(int en);
int akm_wrapper_set_offset(float offset[3]);
int akm_wrapper_get_offset(float offset[3]);
int akm_wrapper_set_gyr(struct magCaliDataInPut *inputData);
int akm_wrapper_set_acc(struct magCaliDataInPut *inputData);
int akm_wrapper_set_mag_null(struct magCaliDataInPut *inputData);
int akm_wrapper_calibrate(struct magCaliDataInPut  *inputData,
                          struct magCaliDataOutPut *outputData);
int akm_wrapper_get_gravity(struct magCaliDataOutPut *outputData);
int akm_wrapper_get_quat(struct magCaliDataOutPut *outputData);
int akm_wrapper_get_ori(struct magCaliDataOutPut *outputData);
int akm_wrapper_get_lacc(struct magCaliDataOutPut *outputData);
int akm_wrapper_get_gyr(struct magCaliDataOutPut *outputData);
void akm_wrapper_set_calibvalue(char *value);
void akm_wrapper_get_calibvalue(char *value);

#endif
