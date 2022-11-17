/****************************************************************************
 * arch/or1k/src/common/or1k_cpuinfo.c
 *
 *   Copyright (C) 2018 Extent3D. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>

#include <nuttx/config.h>
#include <sys/types.h>
#include <arch/spr.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int or1k_print_cpuinfo(void)
{
  uint32_t vr;
  uint32_t vr2;
  uint32_t cpucfg;
  uint32_t upr;

  mfspr(SPR_SYS_VR, vr);
  mfspr(SPR_SYS_CPUCFGR, cpucfg);
  mfspr(SPR_SYS_UPR, upr);

  syslog(LOG_INFO, "VR: 0x%08x CPUCFGR: 0x%08x UPR: 0x%08x\n",
         vr, cpucfg, upr);

  syslog(LOG_INFO, "OpenRISC rev %lu ver %lu\n",
       ((vr & SPR_VR_REV_MASK) >> SPR_VR_REV_SHIFT),
       ((vr & SPR_VR_VER_MASK) >> SPR_VR_VER_SHIFT));

  if ((vr & SPR_VR_UVRP) != 0)
    {
      mfspr(SPR_SYS_VR2, vr2);
      syslog(LOG_INFO, "  CPUID:             %d\n",
            (vr2 & SPR_VR2_CPUID_MASK) >> SPR_VR2_CPUID_SHIFT);
      syslog(LOG_INFO, "  V2.VER:            0x%x\n",
            (vr2 & SPR_VR2_VER_MASK) >> SPR_VR2_VER_SHIFT);
    }

  syslog(LOG_INFO, "  AVR/VR2:           %s\n",
         (vr & SPR_VR_UVRP) ? "yes" : "no");
  syslog(LOG_INFO, "  Data Cache:        %s\n",
         (upr & SPR_UPR_DCP) ? "yes" : "no");
  syslog(LOG_INFO, "  Instruction Cache: %s\n",
         (upr & SPR_UPR_ICP) ? "yes" : "no");
  syslog(LOG_INFO, "  Data MMU:          %s\n",
         (upr & SPR_UPR_DMP) ? "yes" : "no");
  syslog(LOG_INFO, "  Instruction MMU:   %s\n",
         (upr & SPR_UPR_IMP) ? "yes" : "no");
  syslog(LOG_INFO, "  DSP MAC:           %s\n",
         (upr & SPR_UPR_MP) ? "yes" : "no");
  syslog(LOG_INFO, "  Debug Unit:        %s\n",
         (upr & SPR_UPR_DUP) ? "yes" : "no");
  syslog(LOG_INFO, "  Performance Count: %s\n",
         (upr & SPR_UPR_PCUP) ? "yes" : "no");
  syslog(LOG_INFO, "  Power Management:  %s\n",
         (upr & SPR_UPR_PMP) ? "yes" : "no");
  syslog(LOG_INFO, "  Interrupt Ctrl:    %s\n",
         (upr & SPR_UPR_PICP) ? "yes" : "no");
  syslog(LOG_INFO, "  Tick Timer:        %s\n",
         (upr & SPR_UPR_TTP) ? "yes" : "no");

  syslog(LOG_INFO, "  Shadow Regs:       %d\n",
         (cpucfg & SPR_CPUCFGR_NSGF_MASK));
  syslog(LOG_INFO, "  Custom GPR:        %s\n",
         (cpucfg & SPR_CPUCFGR_CGF) ? "yes" : "no");
  syslog(LOG_INFO, "  ORBIS32:           %s\n",
         (cpucfg & SPR_CPUCFGR_OB32S) ? "yes" : "no");
  syslog(LOG_INFO, "  ORBIS64:           %s\n",
         (cpucfg & SPR_CPUCFGR_OB64S) ? "yes" : "no");
  syslog(LOG_INFO, "  ORFPX32:           %s\n",
         (cpucfg & SPR_CPUCFGR_OF32S) ? "yes" : "no");
  syslog(LOG_INFO, "  ORFPX64:           %s\n",
         (cpucfg & SPR_CPUCFGR_OF64S) ? "yes" : "no");
  syslog(LOG_INFO, "  ORVDX64:           %s\n",
         (cpucfg & SPR_CPUCFGR_OV64S) ? "yes" : "no");
  syslog(LOG_INFO, "  No Delay Slot:     %s\n",
         (cpucfg & SPR_CPUCFGR_ND) ? "yes" : "no");
  syslog(LOG_INFO, "  AVR Present:       %s\n",
         (cpucfg & SPR_CPUCFGR_AVRP) ? "yes" : "no");
  syslog(LOG_INFO, "  Exception BAR:     %s\n",
         (cpucfg & SPR_CPUCFGR_EVBARP) ? "yes" : "no");
  syslog(LOG_INFO, "  ISR Present:       %s\n",
         (cpucfg & SPR_CPUCFGR_ISRP) ? "yes" : "no");
  syslog(LOG_INFO, "  AE[CS]R Present:   %s\n",
         (cpucfg & SPR_CPUCFGR_AECSRP) ? "yes" : "no");

  return OK;
}
