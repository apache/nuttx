/****************************************************************************
 * boards/arm/samv7/common/include/sam_automount.h
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

#ifndef __BOARDS_ARM_SAMV7_COMMON_INCLUDE_SAM_AUTOMOUNT_H
#define __BOARDS_ARM_SAMV7_COMMON_INCLUDE_SAM_AUTOMOUNT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SAMV7_HSMCI0_AUTOMOUNT
/* HSMCI0 Automounter defaults */

#  ifndef CONFIG_SAMV7_HSMCI0_AUTOMOUNT_FSTYPE
#    define CONFIG_SAMV7_HSMCI0_AUTOMOUNT_FSTYPE "vfat"
#  endif

#  ifndef CONFIG_SAMV7_HSMCI0_AUTOMOUNT_BLKDEV
#    define CONFIG_SAMV7_HSMCI0_AUTOMOUNT_BLKDEV "/dev/mmcds0"
#  endif

#  ifndef CONFIG_SAMV7_HSMCI0_AUTOMOUNT_MOUNTPOINT
#    define CONFIG_SAMV7_HSMCI0_AUTOMOUNT_MOUNTPOINT "/mnt/sdcard0"
#  endif

#  ifndef CONFIG_SAMV7_HSMCI0_AUTOMOUNT_DDELAY
#    define CONFIG_SAMV7_HSMCI0_AUTOMOUNT_DDELAY 1000
#  endif

#  ifndef CONFIG_SAMV7_HSMCI0_AUTOMOUNT_UDELAY
#    define CONFIG_SAMV7_HSMCI0_AUTOMOUNT_UDELAY 2000
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name:  sam_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enable and so configured HSMCI
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

void sam_automount_initialize(void);

/****************************************************************************
 * Name:  sam_automount_event
 *
 * Description:
 *   The HSMCI card detection logic has detected an insertion or removal
 *   event.  It has already scheduled the MMC/SD block driver operations.
 *   Now we need to schedule the auto-mount event which will occur with a
 *   substantial delay to make sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the HSMCI0 slot: HSMCI0 or HSMCI1_SLOTNO.
 *       There is a terminology problem here:  Each HSMCI supports two slots,
 *      slot A and slot B. Only slot A is used.  So this is not a really a
 *      slot, but an HSCMI peripheral number.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

void sam_automount_event(int slotno, bool inserted);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_COMMON_INCLUDE_SAM_AUTOMOUNT_H */
