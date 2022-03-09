/****************************************************************************
 * drivers/audio/cs35l41b_fw.h
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

#ifndef __DRIVERS_AUDIO_CS35L41B_FW_H__
#define __DRIVERS_AUDIO_CS35L41B_FW_H__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int cs35l41_dsp_boot(FAR struct cs35l41b_dev_s *priv);
int cs35l41b_is_dsp_processing(FAR struct cs35l41b_dev_s *priv);
uint32_t cs35l41b_get_calibration_result(void);
int cs35l41b_calibrate(FAR struct cs35l41b_dev_s *priv,
                       uint32_t ambient_temp_deg_c);
int cs35l41b_load_calibration_value(FAR struct cs35l41b_dev_s *priv);
int cs35l41b_write_caliberate_ambient(FAR struct cs35l41b_dev_s *priv,
                                      uint32_t ambient_temp_deg_c);
int cs35l41b_set_boot_configuration(FAR struct cs35l41b_dev_s *priv);

#endif