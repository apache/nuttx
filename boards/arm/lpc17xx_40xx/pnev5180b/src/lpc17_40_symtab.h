/****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/src/lpc17_40_symtab.h
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

#ifndef __BOARDS_ARM_LPC17XX_40XX_PNEV5180B_SRC_LPC17_40_SYMTAB_H
#define __BOARDS_ARM_LPC17XX_40XX_PNEV5180B_SRC_LPC17_40_SYMTAB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/symtab.h>

#if defined(CONFIG_ELF) || defined(CONFIG_NXFLAT)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct symtab_s lpc17_40_exports[];
extern const int lpc17_40_nexports;

#endif
#endif /* __BOARDS_ARM_LPC17XX_40XX_PNEV5180B_SRC_LPC17_40_SYMTAB_H */
