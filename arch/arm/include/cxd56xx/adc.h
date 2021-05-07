/****************************************************************************
 * arch/arm/include/cxd56xx/adc.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_ADC_H
#define __ARCH_ARM_INCLUDE_CXD56XX_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#define ANIOC_USER                 (AN_FIRST + AN_NCMDS)

/* Start sampling
 *
 * param None
 * return ioctl return value provides success/failure indication
 */

#define ANIOC_CXD56_START          _ANIOC(ANIOC_USER + 0)

/* Stop sampling
 *
 * param None
 * return ioctl return value provides success/failure indication
 */

#define ANIOC_CXD56_STOP           _ANIOC(ANIOC_USER + 1)

/* Set sampling frequency
 *
 * param None
 * return ioctl return value provides success/failure indication
 */

#define ANIOC_CXD56_FREQ           _ANIOC(ANIOC_USER + 2)

/* Set fifo size
 *
 * param None
 * return ioctl return value provides success/failure indication
 */

#define ANIOC_CXD56_FIFOSIZE       _ANIOC(ANIOC_USER + 3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Initialize valid ADC channels
 *
 * return OK(0) is success. negative value is failure.
 */

int cxd56_adcinitialize(void);

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_ADC_H */
