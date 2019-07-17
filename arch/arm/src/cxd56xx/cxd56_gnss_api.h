/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gnss_api.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_API_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_API_H

#include <arch/chip/gnss_type.h>
#include <arch/chip/gnss.h>

/* GD Start mode */

#define CXD56_GNSS_STMOD_COLD       0
#define CXD56_GNSS_STMOD_WARM       1
#define CXD56_GNSS_STMOD_WARMACC2   2
#define CXD56_GNSS_STMOD_HOT        3
#define CXD56_GNSS_STMOD_HOTACC     4
#define CXD56_GNSS_STMOD_HOTACC2    5
#define CXD56_GNSS_STMOD_HOTACC3    6
#define CXD56_GNSS_STMOD_XTC1       7
#define CXD56_GNSS_STMOD_XTC2       8

/* GD operation mode */

/* GD_SetOperationMode, GD_GetOperationMode */

#define CXD56_GNSS_OPMOD_NORMAL     1
#define CXD56_GNSS_OPMOD_LOWPOWER   2
#define CXD56_GNSS_OPMOD_BALANCE    4
#define CXD56_GNSS_OPMOD_1PSS       5

/* Start a positioning
 * begining to search the satellites and measure the receiver position
 */

int GD_Start(uint8_t startMode);

/* Stop a positioning */

int GD_Stop(void);

/* Select GNSSs to positioning
 * These are able to specified by CXD56_GNSS_B_SAT_XXX defines.
 */

int GD_SelectSatelliteSystem(uint32_t system);

/* Get current using GNSSs to positioning
 * A argument 'satellite' indicates current GNSSs by bit fields defined
 * by CXD56_GNSS_B_SAT_XXX.
 */

int GD_GetSatelliteSystem(FAR uint32_t *system);

/* Set the rough receiver position */

int GD_SetReceiverPositionEllipsoidal(FAR double *dLat, FAR double *dLon,
                                      FAR double *dHeight);

/* Set the rough receiver position as orgothonal */

int GD_SetReceiverPositionOrthogonal(int32_t dX, int32_t dY, int32_t dZ);

/* Set enable or disable the 1PPS output. */

int GD_Set1ppsOutput(uint32_t enable);

/* Get the current 1PPS output setting. */

int GD_Get1ppsOutput(FAR uint32_t *enable);

/* Set the receiver operation mode
 * 1st argument 'mode' is a operation mode defined by CXD56_GNSS_OPMOD_XXX.
 * 2nd argument 'cycle' is a positioning period[ms], default is 1000[ms].
 */

int GD_SetOperationMode(uint32_t mode, uint32_t cycle);

/* Get the receiver operation mode */

int GD_GetOperationMode(FAR uint32_t *mode, FAR uint32_t *cycle);

/* Set the TCXO offset */

int GD_SetTcxoOffset(int32_t offset);

/* Get the TCXO offset */

int GD_GetTcxoOffset(FAR int32_t *offset);

/* Set the estimated current time of the receiver.
 * 1st argument date & time are in UTC.
 */

int GD_SetTime(FAR struct cxd56_gnss_date_s *date,
               FAR struct cxd56_gnss_time_s *time);

/* Set the network time */

int GD_SetFrameTime(uint16_t sec, uint32_t fracSec);

/* Get the almanac data */

int GD_GetAlmanac(uint32_t satellite, FAR uint32_t* almanac,
                  FAR uint32_t *almanacSize);

/* Set the almanac data */

int GD_SetAlmanac(uint32_t satellite, FAR uint32_t *almanac);

/* Get the Ephemeris data */

int GD_GetEphemeris(uint32_t satellite, FAR uint32_t* ephemeris,
                    FAR uint32_t *ephemerisSize);

/* Set the Ephemeris data */

int GD_SetEphemeris(uint32_t satellite, FAR uint32_t *ephemeris);

/* Select to use or not use the initial position calculation supporting
 * information of the QZSS L1-SAIF.
 */

int GD_SetQzssPosAssist(uint32_t enable);

/* Get a setting of the initial position calculation supporting
 * information of the QZSS L1-SAIF.
 */

int GD_GetQzssPosAssist(FAR uint32_t *enable);

/* Set IMES bitrates. */

int GD_SetImesBitrate(uint32_t bitrate);

/* Get IMES bitrates. */

int GD_GetImesBitrate(FAR uint32_t *bitrate);

/* Set IMES center frequency offset. */

int GD_SetImesCenterFreqOffset(uint32_t offset);

/* Set IMES preamble. */

int GD_SetImesPreamble(uint32_t preamble);

/* Start GPS test */

void GD_StartGpsTest(uint32_t satellite, uint32_t reserve1,
                     uint32_t reserve2, uint32_t reserve3);

/* Stop GPS test */

int GD_StopGpsTest(void);

/* Get GPS test result */

int GD_GetGpsTestResult(FAR float* cn, FAR float* doppler);

/* Control Spectrum output */

int GD_SpectrumControl(unsigned long time, unsigned int enable,
                       unsigned char moniPoint1, unsigned char step1,
                       unsigned char moniPoint2, unsigned char step2);

/* Save the backup data to a Flash memory. */

int GD_SaveBackupdata(void);

/* CEP  Check Assist Data Valid  */

int GD_CepCheckAssistData(void);

/* CEP  Get Age Data */

int GD_CepGetAgeData(FAR float *age, FAR float *cepi);

/* CEP  Reset Assist Data init flag & valid flag */

int GD_CepInitAssistData(void);

/* AGPS Set tau */

int GD_SetTauGps(FAR double *tau);

/* AGPS Set Acquist */

int GD_SetAcquist(FAR uint8_t *pAcquistData, uint16_t acquistSize);

/* Set the estimated current time of the receiver.
 * 1st argument date & time are in GPS time.
 */

int GD_SetTimeGps(FAR struct cxd56_gnss_date_s *date,
                  FAR struct cxd56_gnss_time_s *time);

/* Clear Receiver Infomation */

int GD_ClearReceiverInfo(uint32_t type);

/* AGPS Set Tow Assist */

int GD_SetTowAssist(FAR uint8_t *pAssistData, uint16_t dataSize);

/* AGPS Set UTC Model */

int GD_SetUtcModel(FAR uint8_t *pModelData, uint16_t dataSize);

/* Read GNSS data to specified buffer */

int GD_ReadBuffer(uint8_t type, int32_t offset, FAR void *buf,
                  uint32_t length);

/* Write GNSS data from specified buffer */

int GD_WriteBuffer(uint8_t type, int32_t offset, FAR void *buf,
                   uint32_t length);

/* Set notify mask, this mask flag is cleared when notified(poll/signal) */

int GD_SetNotifyMask(uint8_t type, uint8_t clear);

/* Geofence Add Region */

int GD_GeoAddRegion(uint8_t id, long lat, long lon, uint16_t rad);

/* Geofence Modify Region */

int GD_GeoModifyRegion(uint8_t id, long lat, long lon, uint16_t rad);

/* Geofence Delete Region */

int GD_GeoDeleteRegione(uint8_t id);

/* Geofence All delete Region */

int GD_GeoDeleteAllRegion(void);

/* Geofence Region check */

int GD_GeoGetRegionData(uint8_t id, FAR long *lat, FAR long *lon,
                        FAR uint16_t *rad);

/* Geofence Get Used Region ID */

uint32_t GD_GeoGetUsedRegionId(void);

/* Geofence Set mode */

int GD_GeoSetOpMode(uint16_t deadzone, uint16_t dwell_detecttime);

/* Geofence Request All region notify */

int GD_GeoSetAllRgionNotifyRequest(void);

/* Geofence Register to gnss_provider */

int GD_RegisterGeofence(void);

/* Geofence Release from gnss_provider */

int GD_ReleaseGeofence(void);

/* Pvtlog Register to gnss_provider */

int GD_RegisterPvtlog(uint32_t cycle, uint32_t threshold);

/* Pvtlog Release */

int GD_ReleasePvtlog(void);

/* Pvtlog Delete log data */

int GD_PvtlogDeleteLog(void);

/* Pvtlog Get Log status */

int GD_PvtlogGetLogStatus(FAR struct cxd56_gnss_status_s *pLogStatus);

/* Start outputting carrier phase info. */

int GD_RtkStart(FAR struct cxd56_rtk_setting_s *pParam);

/* Stop outputting carrier phase info. */

int GD_RtkStop(void);

/* Set output interval of carrier phase info.
 *
 * interval : CXD56_GNSS_RTK_INTERVAL_XXX (gd_type.h)
 */

int GD_RtkSetOutputInterval(int interval);

/* Get output interval of carrier phase info. [ms] */

int GD_RtkGetOutputInterval(FAR int* interval);

/* Set GNSS of outputting carrier phase info. */

int GD_RtkSetGnss(uint32_t gnss);

/* Get GNSS of outputting carrier phase info. */

int GD_RtkGetGnss(FAR uint32_t* pGnss);

/* Set enable/disable GD to notify updating ephemeris */

int GD_RtkSetEphNotify(int enable);

/* Get enable/disable GD to notify updating ephemeris */

int GD_RtkGetEphNotify(FAR int* enable);

/* Set the Ephemeris data Ephemeris data size is variable. */

int GD_SetVarEphemeris(uint32_t *ephemeris, uint32_t ephemerisSize);

/* Get the Ephemeris data Ephemeris data size is variable. */

int GD_GetVarEphemeris(uint32_t satellite, uint32_t* ephemeris,
                       uint32_t ephemerisSize);

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_API_H */
