/****************************************************************************
 * arch/arm64/src/imx9/imx9_mu.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#ifndef __ARCH_ARM64_SRC_IMX9_MU_H
#define __ARCH_ARM64_SRC_IMX9_MU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

typedef void (*imx9_mu_msg_callback_t)(int id, uint32_t msg, void *arg);
typedef void (*imx9_mu_gpi_callback_t)(int id, void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx95_mu_init
 *
 * Description:
 *   Initialize mu
 *
 * Input Parameters:
 *   index   - The index of mu
 *
 * Returned Value:
 *   imx9_mu_dev_s struct is returned on success. NULL is returned on
 *   failure.
 *
 ****************************************************************************/

struct imx9_mu_dev_s *imx95_mu_init(int index);

/****************************************************************************
 * Name: imx95_mu_subscribe_msg
 *
 * Description:
 *  Subscribe msg, when the irq occur,the msg callback will be called.
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   msg_int_bitfield - Enable correspond bit receive irq
 *   callback         - The call back will called when the irq occur
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx95_mu_subscribe_msg(struct imx9_mu_dev_s *priv,
                            uint32_t msg_int_bitfield,
                            imx9_mu_msg_callback_t callback);

/****************************************************************************
 * Name: imx95_mu_subscribe_gpi
 *
 * Description:
 *  Subscribe msg, when the irq occur,the msg callback will be called.
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   gpi_int_enable   - Enable correspond bit general purpose irq
 *   callback         - The call back will called when the irq occur
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx95_mu_subscribe_gpi(struct imx9_mu_dev_s *priv,
                            uint32_t gpi_int_enable,
                            imx9_mu_gpi_callback_t callback);

/****************************************************************************
 * Name: imx95_mu_deinit
 *
 * Description:
 *   Deinit mu
 *
 * Input Parameters:
 *   priv             - The mu dev will be deinit
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx95_mu_deinit(struct imx9_mu_dev_s *priv);

/****************************************************************************
 * Name: imx95_mu_send_msg_non_blocking
 *
 * Description:
 *  When the mu is busy, will return err
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   reg_index        - Which one transmit reg to be used
 *   msg              - The msgto be send
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx95_mu_send_msg_non_blocking(struct imx9_mu_dev_s *priv,
                                   uint32_t reg_index, uint32_t msg);

/****************************************************************************
 * Name: imx95_mu_send_msg
 *
 * Description:
 *  Send msg blocking
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   reg_index        - Which one transmit reg to be used
 *   msg              - The msgto be send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx95_mu_send_msg(struct imx9_mu_dev_s *priv, uint32_t reg_index,
                       uint32_t msg);

/****************************************************************************
 * Name: imx95_mu_has_received_msg
 *
 * Description:
 *   Check Mu if has msg
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   reg_index        - Which one receive reg to be used
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx95_mu_has_received_msg(struct imx9_mu_dev_s *priv,
                              uint32_t reg_index);

/****************************************************************************
 * Name: imx95_mu_receive_msg_non_blocking
 *
 * Description:
 *  Non block receive msg
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   reg_index        - Which one receive reg to be used
 *
 * Returned Value:
 *  The value of index receive reg
 *
 ****************************************************************************/

uint32_t imx95_mu_receive_msg_non_blocking(struct imx9_mu_dev_s *priv,
                                           uint32_t reg_index);

/****************************************************************************
 * Name: imx95_mu_receive_msg
 *
 * Description:
 *   Block receive msg
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   reg_index        - Which one receive reg to be used
 *
 * Returned Value:
 *  The value of index receive reg
 *
 ****************************************************************************/

uint32_t imx95_mu_receive_msg(struct imx9_mu_dev_s *priv,
                              uint32_t reg_index);

/****************************************************************************
 * Name: imx95_mu_trigger_interrupts
 *
 * Description:
 *  Send irq to MUB
 *
 * Input Parameters:
 *   priv             - The mu dev will be used
 *   interrupts       - The number of interrupts
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx95_mu_trigger_interrupts(struct imx9_mu_dev_s *priv,
                                uint32_t interrupts);

#endif /* __ARCH_ARM64_SRC_IMX9_MU_H */
