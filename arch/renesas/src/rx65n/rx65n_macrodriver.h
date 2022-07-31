/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_macrodriver.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_MACRODRIVER_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_MACRODRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "arch/rx65n/iodefine.h"

#ifndef __TYPEDEF__

/* Status list definition */

#define MD_STATUSBASE      (0x00U)
#define MD_OK              (MD_STATUSBASE + 0x00U) /* register setting OK */
#define MD_SPT             (MD_STATUSBASE + 0x01U) /* IIC stop */
#define MD_NACK            (MD_STATUSBASE + 0x02U) /* IIC no ACK */
#define MD_BUSY1           (MD_STATUSBASE + 0x03U) /* busy 1 */
#define MD_BUSY2           (MD_STATUSBASE + 0x04U) /* busy 2 */

/* Error list definition */

#define MD_ERRORBASE   (0x80U)
#define MD_ERROR       (MD_ERRORBASE + 0x00U)  /* error */

/* error argument input error */

#define MD_ARGERROR    (MD_ERRORBASE + 0x01U)
#define MD_ERROR1      (MD_ERRORBASE + 0x02U)  /* error 1 */
#define MD_ERROR2      (MD_ERRORBASE + 0x03U)  /* error 2 */
#define MD_ERROR3      (MD_ERRORBASE + 0x04U)  /* error 3 */
#define MD_ERROR4      (MD_ERRORBASE + 0x05U)  /* error 4 */
#define MD_ERROR5      (MD_ERRORBASE + 0x06U)  /* error 5 */

#define nop()              asm("nop;")
#define brk()              asm("brk;")
#define wait()             asm("wait;")
#endif

#ifndef __TYPEDEF__
#ifndef _STDINT_H
  typedef unsigned char  uint8_t;
  typedef unsigned short uint16_t;
#endif
  typedef unsigned short MD_STATUS;
#define __TYPEDEF__
#endif

#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_MACRODRIVER_H */
