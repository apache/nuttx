/****************************************************************************
 * arch/arm/src/phy62xx/phyplus_stub.h
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

#ifndef __ARCH_ARM_SRC_PHY62XX_PHYPLUS_STUB_H
#define __ARCH_ARM_SRC_PHY62XX_PHYPLUS_STUB_H

enum phyplus_stub_e
{
  PHYPLUS_GPIO_REGISTER = 0,
  PHYPLUS_GPIO_UNREGISTER,
  PHYPLUS_TIMER_REGISTER,
  PHYPLUS_TIMER_UNREGISTER,
  PHYPLUS_MAX
};

#endif /* __ARCH_ARM_SRC_PHY62XX_PHYPLUS_STUB_H */
