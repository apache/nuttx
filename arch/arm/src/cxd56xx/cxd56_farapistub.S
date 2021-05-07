/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_farapistub.S
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

    .syntax	unified
    .section	.text.stub
    .align	1
1:
    push    {r0-r3}
    adr     r1, 3f
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, 2f
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

    .align	2
2:
    .word   _modulelist_power_mgr

    .align	2
3:

    /* fw_pm_gpspllclocksetsource */
    nop
    nop

    /* fw_pm_pmicgetchargingporttype */
    nop
    nop

    /* fw_pm_pmicsetchargingcurrentvalue */
    nop
    nop

    /* fw_pm_pmicstartcharging */
    nop
    nop

    /* fw_pm_pmicstopcharging */
    nop
    nop

    /* fw_pm_peripoweron */
    nop
    nop

    /* fw_pm_peripoweroff */
    nop
    nop

    /* fw__pm_periclockenable */
    nop
    nop

    /* fw__pm_periclockdisable */
    nop
    nop

    /* fw__pm_updatecpufreqinfo */
    nop
    nop

    /* fw_pm_pmicpoweron */
    nop
    nop

    /* fw_pm_pmicpoweroff */
    nop
    nop

    /* fw_pm_setwakeuptrigger */
    nop
    nop

    /* fw_pm_hostifclockenable */
    nop
    nop

    /* fw_pm_hostifclockdisable */
    nop
    nop

    .global	fw_pm_startcpu
    .thumb_func
fw_pm_startcpu:
    mov     r12, pc
    b       1b

    .global	fw_pm_stopcpu
    .thumb_func
fw_pm_stopcpu:
    mov     r12, pc
    b       1b

    .global	fw_pm_sleepcpu
    .thumb_func
fw_pm_sleepcpu:
    mov     r12, pc
    b       1b

    .global	fw_pm_wakeupcpu
    .thumb_func
fw_pm_wakeupcpu:
    mov     r12, pc
    b       1b

    .global	fw_pm_coldsleep
    .thumb_func
fw_pm_coldsleep:
    mov     r12, pc
    b       1b

    .global	fw_pm_deepsleep
    .thumb_func
fw_pm_deepsleep:
    mov     r12, pc
    b       1b

    .global	fw_pm_reboot
    .thumb_func
fw_pm_reboot:
    mov     r12, pc
    b       1b

    .global	fw_pm_loadimage
    .thumb_func
fw_pm_loadimage:
    mov     r12, pc
    b       1b

    /* fw_pm_loadimagetoaddress */
    nop
    nop

    /* fw_pm_ramcontrolbyaddress */
    nop
    nop

    /* fw_pm_changeclock */
    nop
    nop

    .global	fw_pm_pmiccontrol
    .thumb_func
fw_pm_pmiccontrol:
    mov     r12, pc
    b       1b

    /* fw_pm_bootcpu */
    nop
    nop

    /* fw_pm_bootcpuwithwaitmode */
    nop
    nop

    /* fw_pm_getclock */
    nop
    nop

    .section	.modulelist, "ax"
    .align	2
    .global  _modulelist_power_mgr
_modulelist_power_mgr:
    .weak   _module_power_mgr
    .word   _module_power_mgr
    .word   0
    .word   0
    .word   0

    .syntax	unified
    .section	.text.stub
    .align	1
1:
    push    {r0-r3}
    adr     r1, 3f
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, 2f
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

    .align	2
2:
    .word   _modulelist_flash_mgr

    .align	2
3:

    /* fw_fm_getregistryvalue */
    nop
    nop

    /* fw_fm_setregistryvalue */
    nop
    nop

    /* fw_fm_setregistryvaluelist */
    nop
    nop

    /* fw_fm_deleteregistrykey */
    nop
    nop

    /* fw_fm_generateregistrykey */
    nop
    nop

    /* fw_fm_sync */
    nop
    nop

    /* fw_fm_fileopen */
    nop
    nop

    /* fw_fm_fileclose */
    nop
    nop

    /* fw_fm_fileread */
    nop
    nop

    /* fw_fm_filewrite */
    nop
    nop

    /* fw_fm_fileseek */
    nop
    nop

    /* fw_fm_fileflush */
    nop
    nop

    /* fw_fm_filedelete */
    nop
    nop

    .global	fw_fm_rawwrite
    .thumb_func
fw_fm_rawwrite:
    mov     r12, pc
    b       1b

    .global	fw_fm_rawread
    .thumb_func
fw_fm_rawread:
    mov     r12, pc
    b       1b

    .global	fw_fm_rawerasesector
    .thumb_func
fw_fm_rawerasesector:
    mov     r12, pc
    b       1b

    /* fw_fm_filerename */
    nop
    nop

    /* fw_fm_filegetattr */
    nop
    nop

    /* fw_fm_filesetattr */
    nop
    nop

    /* fw_fm_fileclearattr */
    nop
    nop

    .global	fw_fm_rawverifywrite
    .thumb_func
fw_fm_rawverifywrite:
    mov     r12, pc
    b       1b

    .section	.modulelist, "ax"
    .align	2
    .global  _modulelist_flash_mgr
_modulelist_flash_mgr:
    .weak   _module_flash_mgr
    .word   _module_flash_mgr
    .word   0
    .word   0
    .word   0

    .syntax	unified
    .section	.text.stub
    .align	1
1:
    push    {r0-r3}
    adr     r1, 3f
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, 2f
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

    .align	2
2:
    .word   _modulelist_rtc_mgr

    .align	2
3:

    /* fw_sys_rtcmgrsettime */
    nop
    nop

    /* fw_sys_rtcmgrgettime */
    nop
    nop

    /* fw_sys_rtcmgrconvtime */
    nop
    nop

    /* fw_sys_rtcmgrconvcalendartime */
    nop
    nop

    /* fw_sys_rtcmgrsetcalendartime */
    nop
    nop

    /* fw_sys_rtcmgrgetcalendartime */
    nop
    nop

    /* fw_sys_rtcmgradjusttime */
    nop
    nop

    /* fw_sys_rtcmgrrequestalarm */
    nop
    nop

    /* fw_sys_rtcmgrcancelalarm */
    nop
    nop

    .section	.modulelist, "ax"
    .align	2
    .global  _modulelist_rtc_mgr
_modulelist_rtc_mgr:
    .weak   _module_rtc_mgr
    .word   _module_rtc_mgr
    .word   0
    .word   0
    .word   0

    .syntax	unified
    .section	.text.stub
    .align	1
1:
    push    {r0-r3}
    adr     r1, 3f
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, 2f
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

    .align	2
2:
    .word   _modulelist_gnss_pwr

    .align	2
3:

    /* fw_gnsp_changeclockboot */
    nop
    nop

    /* fw_gnsp_changeclocknormal */
    nop
    nop

    /* fw_gnsp_setattribute */
    nop
    nop

    /* fw_gnsp_getpowerstatus */
    nop
    nop

    /* fw_gnsp_pongnss */
    nop
    nop

    /* fw_gnsp_poffgnss */
    nop
    nop

    /* fw_gnsp_pofflna */
    nop
    nop

    /* fw_gnsp_sleep */
    nop
    nop

    /* fw_gnsp_wakeup */
    nop
    nop

    /* fw_gnsp_setwakeuptime */
    nop
    nop

    /* fw_gnsp_getattribute */
    nop
    nop

    /* fw_gnsp_clockon */
    nop
    nop

    /* fw_gnsp_clockoff */
    nop
    nop

    /* fw_gnsp_changemode */
    nop
    nop

    /* fw_gnsp_enableio */
    nop
    nop

    /* fw_gnsp_controlgpioout */
    nop
    nop

    /* fw_gnsp_debug */
    nop
    nop

    /* fw_gnsp_changeclockrtc */
    nop
    nop

    /* fw_gnsp_changeclocksfcclkup */
    nop
    nop

    /* fw_gnsp_changeclocksfcclkdwn */
    nop
    nop

    /* fw_gnsp_changeclockgpscpuclkup */
    nop
    nop

    /* fw_gnsp_changeclockgpscpuclkdwn */
    nop
    nop

    /* fw_gnsp_getbootcause */
    nop
    nop

    /* fw_gnsp_getwakeuptrigger */
    nop
    nop

    /* fw_gnsp_selfreboot */
    nop
    nop

    /* fw_gnsp_start_keepppsout */
    nop
    nop

    /* fw_gnsp_stop_keepppsout */
    nop
    nop

    /* fw_gnsp_start_keepitpout */
    nop
    nop

    /* fw_gnsp_stop_keepitpout */
    nop
    nop

    /* fw_gnsp_setsleepstate */
    nop
    nop

    /* fw_gnsp_freesysmemory */
    nop
    nop

    /* fw_gnsp_readalmanacfile */
    nop
    nop

    .section	.modulelist, "ax"
    .align	2
    .global  _modulelist_gnss_pwr
_modulelist_gnss_pwr:
    .weak   _module_gnss_pwr
    .word   _module_gnss_pwr
    .word   0
    .word   0
    .word   0

    .syntax	unified
    .section	.text.stub
    .align	1
1:
    push    {r0-r3}
    adr     r1, 3f
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, 2f
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

    .align	2
2:
    .word   _modulelist_aca

    .align	2
3:

    .global	fw_as_acacontrol
    .thumb_func
fw_as_acacontrol:
    mov     r12, pc
    b       1b

    .section	.modulelist, "ax"
    .align	2
    .global  _modulelist_aca
_modulelist_aca:
    .weak   _module_aca
    .word   _module_aca
    .word   0
    .word   0
    .word   0

    .syntax	unified
    .section	.text.stub
    .align	1
1:
    push    {r0-r3}
    adr     r1, 3f
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, 2f
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

    .align	2
2:
    .word   _modulelist_pinconfig

    .align	2
3:

    .global	fw_pd_pinconfigs
    .thumb_func
fw_pd_pinconfigs:
    mov     r12, pc
    b       1b

    .section	.modulelist, "ax"
    .align	2
    .global  _modulelist_pinconfig
_modulelist_pinconfig:
    .weak   _module_pinconfig
    .word   _module_pinconfig
    .word   0
    .word   0
    .word   0

    .syntax	unified
    .section	.text.stub
    .align	1
1:
    push    {r0-r3}
    adr     r1, 3f
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, 2f
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

    .align	2
2:
    .word   _modulelist_uart

    .align	2
3:

    .global	fw_pd_uartinit
    .thumb_func
fw_pd_uartinit:
    mov     r12, pc
    b       1b

    .global	fw_pd_uartuninit
    .thumb_func
fw_pd_uartuninit:
    mov     r12, pc
    b       1b

    .global	fw_pd_uartconfiguration
    .thumb_func
fw_pd_uartconfiguration:
    mov     r12, pc
    b       1b

    .global	fw_pd_uartenable
    .thumb_func
fw_pd_uartenable:
    mov     r12, pc
    b       1b

    .global	fw_pd_uartdisable
    .thumb_func
fw_pd_uartdisable:
    mov     r12, pc
    b       1b

    .global	fw_pd_uartreceive
    .thumb_func
fw_pd_uartreceive:
    mov     r12, pc
    b       1b

    .global	fw_pd_uartsend
    .thumb_func
fw_pd_uartsend:
    mov     r12, pc
    b       1b

    .section	.modulelist, "ax"
    .align	2
    .global  _modulelist_uart
_modulelist_uart:
    .weak   _module_uart
    .word   _module_uart
    .word   0
    .word   0
    .word   0

    .syntax	unified
    .section	.text.stub
    .align	1
1:
    push    {r0-r3}
    adr     r1, 3f
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, 2f
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

    .align	2
2:
    .word   _modulelist_update_mgr

    .align	2
3:

    .global	fw_um_init
    .thumb_func
fw_um_init:
    mov     r12, pc
    b       1b

    .global	fw_um_open
    .thumb_func
fw_um_open:
    mov     r12, pc
    b       1b

    .global	fw_um_commit
    .thumb_func
fw_um_commit:
    mov     r12, pc
    b       1b

    .global	fw_um_close
    .thumb_func
fw_um_close:
    mov     r12, pc
    b       1b

    .global	fw_um_checkpoint
    .thumb_func
fw_um_checkpoint:
    mov     r12, pc
    b       1b

    .global	fw_um_doupdatesequence
    .thumb_func
fw_um_doupdatesequence:
    mov     r12, pc
    b       1b

    .global	fw_um_abort
    .thumb_func
fw_um_abort:
    mov     r12, pc
    b       1b

    .section	.modulelist, "ax"
    .align	2
    .global  _modulelist_update_mgr
_modulelist_update_mgr:
    .weak   _module_update_mgr
    .word   _module_update_mgr
    .word   0
    .word   0
    .word   0

    .syntax	unified
    .section	.text.stub
    .align	1
1:
    push    {r0-r3}
    adr     r1, 3f
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, 2f
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

    .align	2
2:
    .word   _modulelist_gnss

    .align	2
3:

    .global	fw_gd_start
    .thumb_func
fw_gd_start:
    mov     r12, pc
    b       1b

    .global	fw_gd_stop
    .thumb_func
fw_gd_stop:
    mov     r12, pc
    b       1b

    .global	fw_gd_selectsatellitesystem
    .thumb_func
fw_gd_selectsatellitesystem:
    mov     r12, pc
    b       1b

    .global	fw_gd_getsatellitesystem
    .thumb_func
fw_gd_getsatellitesystem:
    mov     r12, pc
    b       1b

    .global	fw_gd_setreceiverpositionellipsoidal
    .thumb_func
fw_gd_setreceiverpositionellipsoidal:
    mov     r12, pc
    b       1b

    .global	fw_gd_setreceiverpositionorthogonal
    .thumb_func
fw_gd_setreceiverpositionorthogonal:
    mov     r12, pc
    b       1b

    .global	fw_gd_setoperationmode
    .thumb_func
fw_gd_setoperationmode:
    mov     r12, pc
    b       1b

    .global	fw_gd_getoperationmode
    .thumb_func
fw_gd_getoperationmode:
    mov     r12, pc
    b       1b

    .global	fw_gd_settcxooffset
    .thumb_func
fw_gd_settcxooffset:
    mov     r12, pc
    b       1b

    .global	fw_gd_gettcxooffset
    .thumb_func
fw_gd_gettcxooffset:
    mov     r12, pc
    b       1b

    .global	fw_gd_settime
    .thumb_func
fw_gd_settime:
    mov     r12, pc
    b       1b

    .global	fw_gd_getalmanac
    .thumb_func
fw_gd_getalmanac:
    mov     r12, pc
    b       1b

    .global	fw_gd_setalmanac
    .thumb_func
fw_gd_setalmanac:
    mov     r12, pc
    b       1b

    .global	fw_gd_getephemeris
    .thumb_func
fw_gd_getephemeris:
    mov     r12, pc
    b       1b

    .global	fw_gd_setephemeris
    .thumb_func
fw_gd_setephemeris:
    mov     r12, pc
    b       1b

    .global	fw_gd_getvarephemeris
    .thumb_func
fw_gd_getvarephemeris:
    mov     r12, pc
    b       1b

    .global	fw_gd_setvarephemeris
    .thumb_func
fw_gd_setvarephemeris:
    mov     r12, pc
    b       1b

    .global	fw_gd_startgpstest
    .thumb_func
fw_gd_startgpstest:
    mov     r12, pc
    b       1b

    .global	fw_gd_stopgpstest
    .thumb_func
fw_gd_stopgpstest:
    mov     r12, pc
    b       1b

    .global	fw_gd_getgpstestresult
    .thumb_func
fw_gd_getgpstestresult:
    mov     r12, pc
    b       1b

    .global	fw_gd_savebackupdata
    .thumb_func
fw_gd_savebackupdata:
    mov     r12, pc
    b       1b

    .global	fw_gd_erasebackup
    .thumb_func
fw_gd_erasebackup:
    mov     r12, pc
    b       1b

    .global	fw_gd_cepsetassistdata
    .thumb_func
fw_gd_cepsetassistdata:
    mov     r12, pc
    b       1b

    .global	fw_gd_ceperaseassistdata
    .thumb_func
fw_gd_ceperaseassistdata:
    mov     r12, pc
    b       1b

    .global	fw_gd_cepcheckassistdata
    .thumb_func
fw_gd_cepcheckassistdata:
    mov     r12, pc
    b       1b

    .global	fw_gd_cepgetagedata
    .thumb_func
fw_gd_cepgetagedata:
    mov     r12, pc
    b       1b

    .global	fw_gd_cepinitassistdata
    .thumb_func
fw_gd_cepinitassistdata:
    mov     r12, pc
    b       1b

    .global	fw_gd_setacquist
    .thumb_func
fw_gd_setacquist:
    mov     r12, pc
    b       1b

    .global	fw_gd_setframetime
    .thumb_func
fw_gd_setframetime:
    mov     r12, pc
    b       1b

    .global	fw_gd_settaugps
    .thumb_func
fw_gd_settaugps:
    mov     r12, pc
    b       1b

    .global	fw_gd_settimegps
    .thumb_func
fw_gd_settimegps:
    mov     r12, pc
    b       1b

    .global	fw_gd_clearreceiverinfo
    .thumb_func
fw_gd_clearreceiverinfo:
    mov     r12, pc
    b       1b

    .global	fw_gd_settowassist
    .thumb_func
fw_gd_settowassist:
    mov     r12, pc
    b       1b

    .global	fw_gd_setutcmodel
    .thumb_func
fw_gd_setutcmodel:
    mov     r12, pc
    b       1b

    .global	fw_gd_spectrumcontrol
    .thumb_func
fw_gd_spectrumcontrol:
    mov     r12, pc
    b       1b

    .global	fw_gd_readbuffer
    .thumb_func
fw_gd_readbuffer:
    mov     r12, pc
    b       1b

    .global	fw_gd_writebuffer
    .thumb_func
fw_gd_writebuffer:
    mov     r12, pc
    b       1b

    .global	fw_gd_setnotifymask
    .thumb_func
fw_gd_setnotifymask:
    mov     r12, pc
    b       1b

    .global	fw_gd_geoaddregion
    .thumb_func
fw_gd_geoaddregion:
    mov     r12, pc
    b       1b

    .global	fw_gd_geomodifyregion
    .thumb_func
fw_gd_geomodifyregion:
    mov     r12, pc
    b       1b

    .global	fw_gd_geodeleteregione
    .thumb_func
fw_gd_geodeleteregione:
    mov     r12, pc
    b       1b

    .global	fw_gd_geodeleteallregion
    .thumb_func
fw_gd_geodeleteallregion:
    mov     r12, pc
    b       1b

    .global	fw_gd_geogetregiondata
    .thumb_func
fw_gd_geogetregiondata:
    mov     r12, pc
    b       1b

    .global	fw_gd_geogetusedregionid
    .thumb_func
fw_gd_geogetusedregionid:
    mov     r12, pc
    b       1b

    .global	fw_gd_geosetopmode
    .thumb_func
fw_gd_geosetopmode:
    mov     r12, pc
    b       1b

    .global	fw_gd_geosetallrgionnotifyrequest
    .thumb_func
fw_gd_geosetallrgionnotifyrequest:
    mov     r12, pc
    b       1b

    .global	fw_gd_registergeofence
    .thumb_func
fw_gd_registergeofence:
    mov     r12, pc
    b       1b

    .global	fw_gd_releasegeofence
    .thumb_func
fw_gd_releasegeofence:
    mov     r12, pc
    b       1b

    .global	fw_gd_registerpvtlog
    .thumb_func
fw_gd_registerpvtlog:
    mov     r12, pc
    b       1b

    .global	fw_gd_releasepvtlog
    .thumb_func
fw_gd_releasepvtlog:
    mov     r12, pc
    b       1b

    .global	fw_gd_pvtlogdeletelog
    .thumb_func
fw_gd_pvtlogdeletelog:
    mov     r12, pc
    b       1b

    .global	fw_gd_pvtloggetlogstatus
    .thumb_func
fw_gd_pvtloggetlogstatus:
    mov     r12, pc
    b       1b

    .global	fw_gd_rtkstart
    .thumb_func
fw_gd_rtkstart:
    mov     r12, pc
    b       1b

    .global	fw_gd_rtkstop
    .thumb_func
fw_gd_rtkstop:
    mov     r12, pc
    b       1b

    .global	fw_gd_rtksetoutputinterval
    .thumb_func
fw_gd_rtksetoutputinterval:
    mov     r12, pc
    b       1b

    .global	fw_gd_rtkgetoutputinterval
    .thumb_func
fw_gd_rtkgetoutputinterval:
    mov     r12, pc
    b       1b

    .global	fw_gd_rtksetgnss
    .thumb_func
fw_gd_rtksetgnss:
    mov     r12, pc
    b       1b

    .global	fw_gd_rtkgetgnss
    .thumb_func
fw_gd_rtkgetgnss:
    mov     r12, pc
    b       1b

    .global	fw_gd_rtksetephnotify
    .thumb_func
fw_gd_rtksetephnotify:
    mov     r12, pc
    b       1b

    .global	fw_gd_rtkgetephnotify
    .thumb_func
fw_gd_rtkgetephnotify:
    mov     r12, pc
    b       1b

    .global	fw_gd_setusecase
    .thumb_func
fw_gd_setusecase:
    mov     r12, pc
    b       1b

    .global	fw_gd_getusecase
    .thumb_func
fw_gd_getusecase:
    mov     r12, pc
    b       1b

    .global	fw_gd_set1ppsoutput
    .thumb_func
fw_gd_set1ppsoutput:
    mov     r12, pc
    b       1b

    .global	fw_gd_get1ppsoutput
    .thumb_func
fw_gd_get1ppsoutput:
    mov     r12, pc
    b       1b

    .section	.modulelist, "ax"
    .align	2
    .global  _modulelist_gnss
_modulelist_gnss:
    .weak   _module_gnss
    .word   _module_gnss
    .word   1
    .word   0
    .word   0

    .section	.modulelist, "ax"
    .align	2
    .global  _modulelist_gnss_geofence
_modulelist_gnss_geofence:
    .weak   _module_gnss_geofence
    .word   _module_gnss_geofence
    .word   1
    .word   0
    .word   0

    .section	.modulelist, "ax"
    .align	2
    .global  _modulelist_gnss_pvtlog
_modulelist_gnss_pvtlog:
    .weak   _module_gnss_pvtlog
    .word   _module_gnss_pvtlog
    .word   1
    .word   0
    .word   0
