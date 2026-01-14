/****************************************************************************
 * arch/arm/src/imx9/imx9_mu.h
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

#ifndef __ARCH_ARM_SRC_IMX9_IMX9_MU_H
#define __ARCH_ARM_SRC_IMX9_IMX9_MU_H

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

struct imx9_mudev_s *imx95_mu_init(int index);
void imx95_mu_subscribe_msg(struct imx9_mudev_s *priv,
                            uint32_t msg_int_bitfield,
                            imx9_mu_msg_callback_t callback);
void imx95_mu_subscribe_gpi(struct imx9_mudev_s *priv,
                            uint32_t gpi_int_enable,
                            imx9_mu_gpi_callback_t callback);

void imx95_mu_deinit(struct imx9_mudev_s *priv);
int imx95_mu_send_msg_non_blocking(struct imx9_mudev_s *priv,
                                   uint32_t reg_index, uint32_t msg);
void imx95_mu_send_msg(struct imx9_mudev_s *priv, uint32_t reg_index,
                       uint32_t msg);
int imx95_mu_has_received_msg(struct imx9_mudev_s *priv, uint32_t reg_index);
uint32_t imx95_mu_receive_msg_non_blocking(struct imx9_mudev_s *priv,
                                           uint32_t reg_index);
uint32_t imx95_mu_receive_msg(struct imx9_mudev_s *priv, uint32_t reg_index);
int imx95_mu_trigger_interrupts(struct imx9_mudev_s *priv,
                                uint32_t interrupts);

#endif /* __ARCH_ARM_SRC_IMX9_IMX9_MU_H */
