/****************************************************************************
 * arch/arm/src/lc823450/lc823450_symbols.ld
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2014-2015 ON Semiconductor. All rights reserved.
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

__pTransWork             = 0x02000014;
__gTransInitSts          = 0x02000018;
__Fsfs_Convert_ErrorCode = 0x0221ae07;
__Fsts_search_ffwork     = 0x0221ab7d;
__evfat_readdir          = 0x02219763;
__exFiles_readdir        = 0x02213dcd;
__sjis_to_unicode_s      = 0x02203cfd;
__su_strnlen_s           = 0x02204005;
__su_uni_strcpy_s        = 0x0220403b;
Fsfs_get_drv_no          = 0x0221b341;
Fsfs_uni_stricmp_target  = 0x0221b447;
evfat_rename             = 0x022196d7;
exFiles_UCstat           = 0x02218e9d;
API_AbsolutePath         = 0x02218409;
EXFAT_WCSLEN             = 0x0222a31d;
exFAT_OS_GetTaskContext  = 0x0222a183;
exFAT_Path_Separator     = 0x02000050;
exFiles_confdata         = 0x02000ae0;
exFAT_API_CheckFile      = 0x02210bb7;
exFAT_Cluster_WritebackSectorCache = 0x0222251d;
exFAT_DirEnt_CompareFilename = 0x02225ae9;
exFAT_DirEnt_CreateFile  = 0x02227517;
exFAT_DirEnt_CulcHash    = 0x02225309;
exFAT_DirEnt_DeleteDirEnt = 0x022275bd;
exFAT_DirEnt_GetAccessedTime = 0x02226637;
exFAT_DirEnt_GetCreateTime = 0x022263a1;
exFAT_DirEnt_GetDataClusterNo = 0x02224905;
exFAT_DirEnt_GetDataLength = 0x02225d17;
exFAT_DirEnt_GetFileAttr = 0x0222607f;
exFAT_DirEnt_GetModifiedTime = 0x022264f3;
exFAT_DirEnt_GetSecondaryFlags = 0x02226851;
exFAT_DirEnt_GetValidDataLength = 0x02224a1d;
exFAT_DirEnt_SetDataClusterNo = 0x02225ee5;
exFAT_DirEnt_SetDataLength = 0x02225c8f;
exFAT_DirEnt_SetFileAttr = 0x0222600d;
exFAT_DirEnt_SetCreateTime = 0x0222610b;
exFAT_DirEnt_SetAccessedTime = 0x022262e7;
exFAT_DirEnt_SetFileNameSet = 0x02227bb7;
exFAT_DirEnt_SetModifiedTime = 0x022261f9;
exFAT_DirEnt_SetSecondaryFlags = 0x02224887;
exFAT_DirEnt_SetSetChecksum = 0x02227db7;
exFAT_DirEnt_SetValidDataLength = 0x02225bf3;
exFAT_DirEnt_ReassemblePath = 0x0222748f;
exFAT_DirEnt_WritebackSectorCache = 0x02225b31;
exFAT_DirEnt_locatepath = 0x0222830d;
exFAT_FAT_WritebackSectorCache = 0x02229721;
exFAT_OpenFileInfo = 0x02000030;
exFAT_illegal_filename_chars = 0x02238c9c;
exfat_num_illegal_chars = 0x02238c98;

sdif_enable_clock        = 0x02207ef3;
sdif_disable_clock       = 0x02207f13;
sdif_ref_cd              = 0x02207f49;
sdif_send_cmd            = 0x02208071;

sdif_get_status          = 0x02207f85;
sddr_initialize          = 0x02207103;
sddr_finalize            = 0x0220715b;
sddr_setclock            = 0x0220784f;
sddr_identifycard        = 0x0220718f;
sddr_refmediatype        = 0x02207939;
sddr_getcardsize         = 0x02207375;
sddr_readsector          = 0x022075a5;
sddr_writesector         = 0x02207657;
sddr_cachectrl           = 0x02207c95;
sddr_getcid              = 0x022073d1;
sddr_eraseseq            = 0x022077b7;
sddr_changespeedmode     = 0x022078d5;
sddr_clearcardinfo       = 0x02207575;
