/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_farapistub.S
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

    /* PM_GpsPllClockSetSource */
    nop
    nop

    /* PM_PmicGetChargingPortType */
    nop
    nop

    /* PM_PmicSetChargingCurrentValue */
    nop
    nop

    /* PM_PmicStartCharging */
    nop
    nop

    /* PM_PmicStopCharging */
    nop
    nop

    /* PM_PeriPowerOn */
    nop
    nop

    /* PM_PeriPowerOff */
    nop
    nop

    /* _PM_PeriClockEnable */
    nop
    nop

    /* _PM_PeriClockDisable */
    nop
    nop

    /* _PM_UpdateCpuFreqInfo */
    nop
    nop

    /* PM_PmicPowerOn */
    nop
    nop

    /* PM_PmicPowerOff */
    nop
    nop

    /* PM_SetWakeupTrigger */
    nop
    nop

    /* PM_HostifClockEnable */
    nop
    nop

    /* PM_HostifClockDisable */
    nop
    nop

    .global	PM_StartCpu
    .thumb_func
PM_StartCpu:
    mov     r12, pc
    b       1b

    .global	PM_StopCpu
    .thumb_func
PM_StopCpu:
    mov     r12, pc
    b       1b

    .global	PM_SleepCpu
    .thumb_func
PM_SleepCpu:
    mov     r12, pc
    b       1b

    .global	PM_WakeUpCpu
    .thumb_func
PM_WakeUpCpu:
    mov     r12, pc
    b       1b

    .global	PM_ColdSleep
    .thumb_func
PM_ColdSleep:
    mov     r12, pc
    b       1b

    .global	PM_DeepSleep
    .thumb_func
PM_DeepSleep:
    mov     r12, pc
    b       1b

    .global	PM_Reboot
    .thumb_func
PM_Reboot:
    mov     r12, pc
    b       1b

    .global	PM_LoadImage
    .thumb_func
PM_LoadImage:
    mov     r12, pc
    b       1b

    /* PM_LoadImageToAddress */
    nop
    nop

    /* PM_RamControlByAddress */
    nop
    nop

    /* PM_ChangeClock */
    nop
    nop

    .global	PM_PmicControl
    .thumb_func
PM_PmicControl:
    mov     r12, pc
    b       1b

    /* PM_BootCpu */
    nop
    nop

    /* PM_BootCpuWithWaitMode */
    nop
    nop

    /* PM_GetClock */
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

    /* FM_GetRegistryValue */
    nop
    nop

    /* FM_SetRegistryValue */
    nop
    nop

    /* FM_SetRegistryValueList */
    nop
    nop

    /* FM_DeleteRegistryKey */
    nop
    nop

    /* FM_GenerateRegistryKey */
    nop
    nop

    /* FM_Sync */
    nop
    nop

    /* FM_FileOpen */
    nop
    nop

    /* FM_FileClose */
    nop
    nop

    /* FM_FileRead */
    nop
    nop

    /* FM_FileWrite */
    nop
    nop

    /* FM_FileSeek */
    nop
    nop

    /* FM_FileFlush */
    nop
    nop

    /* FM_FileDelete */
    nop
    nop

    .global	FM_RawWrite
    .thumb_func
FM_RawWrite:
    mov     r12, pc
    b       1b

    .global	FM_RawRead
    .thumb_func
FM_RawRead:
    mov     r12, pc
    b       1b

    .global	FM_RawEraseSector
    .thumb_func
FM_RawEraseSector:
    mov     r12, pc
    b       1b

    /* FM_FileRename */
    nop
    nop

    /* FM_FileGetAttr */
    nop
    nop

    /* FM_FileSetAttr */
    nop
    nop

    /* FM_FileClearAttr */
    nop
    nop

    .global	FM_RawVerifyWrite
    .thumb_func
FM_RawVerifyWrite:
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

    /* SYS_RtcMgrSetTime */
    nop
    nop

    /* SYS_RtcMgrGetTime */
    nop
    nop

    /* SYS_RtcMgrConvTime */
    nop
    nop

    /* SYS_RtcMgrConvCalendarTime */
    nop
    nop

    /* SYS_RtcMgrSetCalendarTime */
    nop
    nop

    /* SYS_RtcMgrGetCalendarTime */
    nop
    nop

    /* SYS_RtcMgrAdjustTime */
    nop
    nop

    /* SYS_RtcMgrRequestAlarm */
    nop
    nop

    /* SYS_RtcMgrCancelAlarm */
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

    /* GNSP_ChangeClockBoot */
    nop
    nop

    /* GNSP_ChangeClockNormal */
    nop
    nop

    /* GNSP_SetAttribute */
    nop
    nop

    /* GNSP_GetPowerStatus */
    nop
    nop

    /* GNSP_PonGnss */
    nop
    nop

    /* GNSP_PoffGnss */
    nop
    nop

    /* GNSP_Sleep */
    nop
    nop

    /* GNSP_Wakeup */
    nop
    nop

    /* GNSP_SetWakeupTime */
    nop
    nop

    /* GNSP_GetAttribute */
    nop
    nop

    /* GNSP_ClockOn */
    nop
    nop

    /* GNSP_ClockOff */
    nop
    nop

    /* GNSP_ChangeMode */
    nop
    nop

    /* GNSP_EnableIo */
    nop
    nop

    /* GNSP_ControlGpioOut */
    nop
    nop

    /* GNSP_Debug */
    nop
    nop

    /* GNSP_ChangeClockRtc */
    nop
    nop

    /* GNSP_ChangeClockSFCClkup */
    nop
    nop

    /* GNSP_ChangeClockSFCClkdwn */
    nop
    nop

    /* GNSP_GetBootCause */
    nop
    nop

    /* GNSP_GetWakeupTrigger */
    nop
    nop

    /* GNSP_SelfReboot */
    nop
    nop

    /* GNSP_Start_KeepPPSOut */
    nop
    nop

    /* GNSP_Stop_KeepPPSOut */
    nop
    nop

    /* GNSP_SetSleepState */
    nop
    nop

    /* GNSP_FreeSysMemory */
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

    .global	AS_AcaControl
    .thumb_func
AS_AcaControl:
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

    .global	PD_PinConfigs
    .thumb_func
PD_PinConfigs:
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

    .global	PD_UartInit
    .thumb_func
PD_UartInit:
    mov     r12, pc
    b       1b

    .global	PD_UartUninit
    .thumb_func
PD_UartUninit:
    mov     r12, pc
    b       1b

    .global	PD_UartConfiguration
    .thumb_func
PD_UartConfiguration:
    mov     r12, pc
    b       1b

    .global	PD_UartEnable
    .thumb_func
PD_UartEnable:
    mov     r12, pc
    b       1b

    .global	PD_UartDisable
    .thumb_func
PD_UartDisable:
    mov     r12, pc
    b       1b

    .global	PD_UartReceive
    .thumb_func
PD_UartReceive:
    mov     r12, pc
    b       1b

    .global	PD_UartSend
    .thumb_func
PD_UartSend:
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

    .global	UM_Init
    .thumb_func
UM_Init:
    mov     r12, pc
    b       1b

    .global	UM_Open
    .thumb_func
UM_Open:
    mov     r12, pc
    b       1b

    .global	UM_Commit
    .thumb_func
UM_Commit:
    mov     r12, pc
    b       1b

    .global	UM_Close
    .thumb_func
UM_Close:
    mov     r12, pc
    b       1b

    .global	UM_CheckPoint
    .thumb_func
UM_CheckPoint:
    mov     r12, pc
    b       1b

    .global	UM_DoUpdateSequence
    .thumb_func
UM_DoUpdateSequence:
    mov     r12, pc
    b       1b

    .global	UM_Abort
    .thumb_func
UM_Abort:
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

    .global	GD_Start
    .thumb_func
GD_Start:
    mov     r12, pc
    b       1b

    .global	GD_Stop
    .thumb_func
GD_Stop:
    mov     r12, pc
    b       1b

    .global	GD_SelectSatelliteSystem
    .thumb_func
GD_SelectSatelliteSystem:
    mov     r12, pc
    b       1b

    .global	GD_GetSatelliteSystem
    .thumb_func
GD_GetSatelliteSystem:
    mov     r12, pc
    b       1b

    .global	GD_SetReceiverPositionEllipsoidal
    .thumb_func
GD_SetReceiverPositionEllipsoidal:
    mov     r12, pc
    b       1b

    .global	GD_SetReceiverPositionOrthogonal
    .thumb_func
GD_SetReceiverPositionOrthogonal:
    mov     r12, pc
    b       1b

    .global	GD_SetOperationMode
    .thumb_func
GD_SetOperationMode:
    mov     r12, pc
    b       1b

    .global	GD_GetOperationMode
    .thumb_func
GD_GetOperationMode:
    mov     r12, pc
    b       1b

    .global	GD_SetTcxoOffset
    .thumb_func
GD_SetTcxoOffset:
    mov     r12, pc
    b       1b

    .global	GD_GetTcxoOffset
    .thumb_func
GD_GetTcxoOffset:
    mov     r12, pc
    b       1b

    .global	GD_SetTime
    .thumb_func
GD_SetTime:
    mov     r12, pc
    b       1b

    .global	GD_GetAlmanac
    .thumb_func
GD_GetAlmanac:
    mov     r12, pc
    b       1b

    .global	GD_SetAlmanac
    .thumb_func
GD_SetAlmanac:
    mov     r12, pc
    b       1b

    .global	GD_GetEphemeris
    .thumb_func
GD_GetEphemeris:
    mov     r12, pc
    b       1b

    .global	GD_SetEphemeris
    .thumb_func
GD_SetEphemeris:
    mov     r12, pc
    b       1b

    .global	GD_GetVarEphemeris
    .thumb_func
GD_GetVarEphemeris:
    mov     r12, pc
    b       1b

    .global	GD_SetVarEphemeris
    .thumb_func
GD_SetVarEphemeris:
    mov     r12, pc
    b       1b

    .global	GD_StartGpsTest
    .thumb_func
GD_StartGpsTest:
    mov     r12, pc
    b       1b

    .global	GD_StopGpsTest
    .thumb_func
GD_StopGpsTest:
    mov     r12, pc
    b       1b

    .global	GD_GetGpsTestResult
    .thumb_func
GD_GetGpsTestResult:
    mov     r12, pc
    b       1b

    .global	GD_SaveBackupdata
    .thumb_func
GD_SaveBackupdata:
    mov     r12, pc
    b       1b

    .global	GD_EraseBackup
    .thumb_func
GD_EraseBackup:
    mov     r12, pc
    b       1b

    .global	GD_CepSetAssistData
    .thumb_func
GD_CepSetAssistData:
    mov     r12, pc
    b       1b

    .global	GD_CepEraseAssistData
    .thumb_func
GD_CepEraseAssistData:
    mov     r12, pc
    b       1b

    .global	GD_CepCheckAssistData
    .thumb_func
GD_CepCheckAssistData:
    mov     r12, pc
    b       1b

    .global	GD_CepGetAgeData
    .thumb_func
GD_CepGetAgeData:
    mov     r12, pc
    b       1b

    .global	GD_CepInitAssistData
    .thumb_func
GD_CepInitAssistData:
    mov     r12, pc
    b       1b

    .global	GD_SetAcquist
    .thumb_func
GD_SetAcquist:
    mov     r12, pc
    b       1b

    .global	GD_SetFrameTime
    .thumb_func
GD_SetFrameTime:
    mov     r12, pc
    b       1b

    .global	GD_SetTauGps
    .thumb_func
GD_SetTauGps:
    mov     r12, pc
    b       1b

    .global	GD_SetTimeGps
    .thumb_func
GD_SetTimeGps:
    mov     r12, pc
    b       1b

    .global	GD_ClearReceiverInfo
    .thumb_func
GD_ClearReceiverInfo:
    mov     r12, pc
    b       1b

    .global	GD_SetTowAssist
    .thumb_func
GD_SetTowAssist:
    mov     r12, pc
    b       1b

    .global	GD_SetUtcModel
    .thumb_func
GD_SetUtcModel:
    mov     r12, pc
    b       1b

    .global	GD_SpectrumControl
    .thumb_func
GD_SpectrumControl:
    mov     r12, pc
    b       1b

    .global	GD_ReadBuffer
    .thumb_func
GD_ReadBuffer:
    mov     r12, pc
    b       1b

    .global	GD_WriteBuffer
    .thumb_func
GD_WriteBuffer:
    mov     r12, pc
    b       1b

    .global	GD_SetNotifyMask
    .thumb_func
GD_SetNotifyMask:
    mov     r12, pc
    b       1b

    .global	GD_GeoAddRegion
    .thumb_func
GD_GeoAddRegion:
    mov     r12, pc
    b       1b

    .global	GD_GeoModifyRegion
    .thumb_func
GD_GeoModifyRegion:
    mov     r12, pc
    b       1b

    .global	GD_GeoDeleteRegione
    .thumb_func
GD_GeoDeleteRegione:
    mov     r12, pc
    b       1b

    .global	GD_GeoDeleteAllRegion
    .thumb_func
GD_GeoDeleteAllRegion:
    mov     r12, pc
    b       1b

    .global	GD_GeoGetRegionData
    .thumb_func
GD_GeoGetRegionData:
    mov     r12, pc
    b       1b

    .global	GD_GeoGetUsedRegionId
    .thumb_func
GD_GeoGetUsedRegionId:
    mov     r12, pc
    b       1b

    .global	GD_GeoSetOpMode
    .thumb_func
GD_GeoSetOpMode:
    mov     r12, pc
    b       1b

    .global	GD_GeoSetAllRgionNotifyRequest
    .thumb_func
GD_GeoSetAllRgionNotifyRequest:
    mov     r12, pc
    b       1b

    .global	GD_RegisterGeofence
    .thumb_func
GD_RegisterGeofence:
    mov     r12, pc
    b       1b

    .global	GD_ReleaseGeofence
    .thumb_func
GD_ReleaseGeofence:
    mov     r12, pc
    b       1b

    .global	GD_RegisterPvtlog
    .thumb_func
GD_RegisterPvtlog:
    mov     r12, pc
    b       1b

    .global	GD_ReleasePvtlog
    .thumb_func
GD_ReleasePvtlog:
    mov     r12, pc
    b       1b

    .global	GD_PvtlogDeleteLog
    .thumb_func
GD_PvtlogDeleteLog:
    mov     r12, pc
    b       1b

    .global	GD_PvtlogGetLogStatus
    .thumb_func
GD_PvtlogGetLogStatus:
    mov     r12, pc
    b       1b

    .global	GD_RtkStart
    .thumb_func
GD_RtkStart:
    mov     r12, pc
    b       1b

    .global	GD_RtkStop
    .thumb_func
GD_RtkStop:
    mov     r12, pc
    b       1b

    .global	GD_RtkSetOutputInterval
    .thumb_func
GD_RtkSetOutputInterval:
    mov     r12, pc
    b       1b

    .global	GD_RtkGetOutputInterval
    .thumb_func
GD_RtkGetOutputInterval:
    mov     r12, pc
    b       1b

    .global	GD_RtkSetGnss
    .thumb_func
GD_RtkSetGnss:
    mov     r12, pc
    b       1b

    .global	GD_RtkGetGnss
    .thumb_func
GD_RtkGetGnss:
    mov     r12, pc
    b       1b

    .global	GD_RtkSetEphNotify
    .thumb_func
GD_RtkSetEphNotify:
    mov     r12, pc
    b       1b

    .global	GD_RtkGetEphNotify
    .thumb_func
GD_RtkGetEphNotify:
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
