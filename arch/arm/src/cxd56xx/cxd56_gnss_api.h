/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gnss_api.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_API_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_API_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/gnss_type.h>
#include <arch/chip/gnss.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/* fw_gd_setoperationmode, fw_gd_getoperationmode */

#define CXD56_GNSS_OPMOD_NORMAL     1
#define CXD56_GNSS_OPMOD_LOWPOWER   2
#define CXD56_GNSS_OPMOD_BALANCE    4
#define CXD56_GNSS_OPMOD_1PSS       5

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Start a positioning
 * beginning to search the satellites and measure the receiver position
 */

int fw_gd_start(uint8_t startmode);

/* Stop a positioning */

int fw_gd_stop(void);

/* Select GNSSs to positioning
 * These are able to specified by CXD56_GNSS_B_SAT_XXX defines.
 */

int fw_gd_selectsatellitesystem(uint32_t system);

/* Get current using GNSSs to positioning
 * A argument 'satellite' indicates current GNSSs by bit fields defined
 * by CXD56_GNSS_B_SAT_XXX.
 */

int fw_gd_getsatellitesystem(FAR uint32_t *system);

/* Set the rough receiver position */

int fw_gd_setreceiverpositionellipsoidal(FAR double *dlat, FAR double *dlon,
                                      FAR double *dheight);

/* Set the rough receiver position as orgothonal */

int fw_gd_setreceiverpositionorthogonal(int32_t dx, int32_t dy, int32_t dz);

/* Set enable or disable the 1PPS output. */

int fw_gd_set1ppsoutput(uint32_t enable);

/* Get the current 1PPS output setting. */

int fw_gd_get1ppsoutput(FAR uint32_t *enable);

/* Set the receiver operation mode
 * 1st argument 'mode' is a operation mode defined by CXD56_GNSS_OPMOD_XXX.
 * 2nd argument 'cycle' is a positioning period[ms], default is 1000[ms].
 */

int fw_gd_setoperationmode(uint32_t mode, uint32_t cycle);

/* Get the receiver operation mode */

int fw_gd_getoperationmode(FAR uint32_t *mode, FAR uint32_t *cycle);

/* Set the TCXO offset */

int fw_gd_settcxooffset(int32_t offset);

/* Get the TCXO offset */

int fw_gd_gettcxooffset(FAR int32_t *offset);

/* Set the estimated current time of the receiver.
 * 1st argument date & time are in UTC.
 */

int fw_gd_settime(FAR struct cxd56_gnss_date_s *date,
               FAR struct cxd56_gnss_time_s *time);

/* Set the network time */

int fw_gd_setframetime(uint16_t sec, uint32_t fracsec);

/* Get the almanac data */

int fw_gd_getalmanac(uint32_t satellite, FAR uint32_t *almanac,
                  FAR uint32_t *almanacsize);

/* Set the almanac data */

int fw_gd_setalmanac(uint32_t satellite, FAR uint32_t *almanac);

/* Get the Ephemeris data */

int fw_gd_getephemeris(uint32_t satellite, FAR uint32_t *ephemeris,
                    FAR uint32_t *ephemerissize);

/* Set the Ephemeris data */

int fw_gd_setephemeris(uint32_t satellite, FAR uint32_t *ephemeris);

/* Select to use or not use the initial position calculation supporting
 * information of the QZSS L1-SAIF.
 */

int fw_gd_setqzssposassist(uint32_t enable);

/* Get a setting of the initial position calculation supporting
 * information of the QZSS L1-SAIF.
 */

int fw_gd_getqzssposassist(FAR uint32_t *enable);

/* Set IMES bitrates. */

int fw_gd_setimesbitrate(uint32_t bitrate);

/* Get IMES bitrates. */

int fw_gd_getimesbitrate(FAR uint32_t *bitrate);

/* Set IMES center frequency offset. */

int fw_gd_setimescenterfreqoffset(uint32_t offset);

/* Set IMES preamble. */

int fw_gd_setimespreamble(uint32_t preamble);

/* Start GPS test */

void fw_gd_startgpstest(uint32_t satellite, uint32_t reserve1,
                     uint32_t reserve2, uint32_t reserve3);

/* Stop GPS test */

int fw_gd_stopgpstest(void);

/* Get GPS test result */

int fw_gd_getgpstestresult(FAR float *cn, FAR float *doppler);

/* Control Spectrum output */

int fw_gd_spectrumcontrol(unsigned long time, unsigned int enable,
                       unsigned char monipoint1, unsigned char step1,
                       unsigned char monipoint2, unsigned char step2);

/* Save the backup data to a Flash memory. */

int fw_gd_savebackupdata(void);

/* CEP  Check Assist Data Valid  */

int fw_gd_cepcheckassistdata(void);

/* CEP  Get Age Data */

int fw_gd_cepgetagedata(FAR float *age, FAR float *cepi);

/* CEP  Reset Assist Data init flag & valid flag */

int fw_gd_cepinitassistdata(void);

/* AGPS Set tau */

int fw_gd_settaugps(FAR double *tau);

/* AGPS Set Acquist */

int fw_gd_setacquist(FAR uint8_t *pacquistdata, uint16_t acquistsize);

/* Set the estimated current time of the receiver.
 * 1st argument date & time are in GPS time.
 */

int fw_gd_settimegps(FAR struct cxd56_gnss_date_s *date,
                  FAR struct cxd56_gnss_time_s *time);

/* Clear Receiver Information */

int fw_gd_clearreceiverinfo(uint32_t type);

/* AGPS Set Tow Assist */

int fw_gd_settowassist(FAR uint8_t *passistdata, uint16_t datasize);

/* AGPS Set UTC Model */

int fw_gd_setutcmodel(FAR uint8_t *pmodeldata, uint16_t datasize);

/* Read GNSS data to specified buffer */

int fw_gd_readbuffer(uint8_t type, int32_t offset, FAR void *buf,
                  uint32_t length);

/* Write GNSS data from specified buffer */

int fw_gd_writebuffer(uint8_t type, int32_t offset, FAR void *buf,
                   uint32_t length);

/* Set notify mask, this mask flag is cleared when notified(poll/signal) */

int fw_gd_setnotifymask(uint8_t type, uint8_t clear);

/* Geofence Add Region */

int fw_gd_geoaddregion(uint8_t id, long lat, long lon, uint16_t rad);

/* Geofence Modify Region */

int fw_gd_geomodifyregion(uint8_t id, long lat, long lon, uint16_t rad);

/* Geofence Delete Region */

int fw_gd_geodeleteregione(uint8_t id);

/* Geofence All delete Region */

int fw_gd_geodeleteallregion(void);

/* Geofence Region check */

int fw_gd_geogetregiondata(uint8_t id, FAR long *lat, FAR long *lon,
                        FAR uint16_t *rad);

/* Geofence Get Used Region ID */

uint32_t fw_gd_geogetusedregionid(void);

/* Geofence Set mode */

int fw_gd_geosetopmode(uint16_t deadzone, uint16_t dwell_detecttime);

/* Geofence Request All region notify */

int fw_gd_geosetallrgionnotifyrequest(void);

/* Geofence Register to gnss_provider */

int fw_gd_registergeofence(void);

/* Geofence Release from gnss_provider */

int fw_gd_releasegeofence(void);

/* Pvtlog Register to gnss_provider */

int fw_gd_registerpvtlog(uint32_t cycle, uint32_t threshold);

/* Pvtlog Release */

int fw_gd_releasepvtlog(void);

/* Pvtlog Delete log data */

int fw_gd_pvtlogdeletelog(void);

/* Pvtlog Get Log status */

int fw_gd_pvtloggetlogstatus(FAR struct cxd56_gnss_status_s *plogstatus);

/* Start outputting carrier phase info. */

int fw_gd_rtkstart(FAR struct cxd56_rtk_setting_s *pparam);

/* Stop outputting carrier phase info. */

int fw_gd_rtkstop(void);

/* Set output interval of carrier phase info.
 *
 * interval : CXD56_GNSS_RTK_INTERVAL_XXX (gd_type.h)
 */

int fw_gd_rtksetoutputinterval(int interval);

/* Get output interval of carrier phase info. [ms] */

int fw_gd_rtkgetoutputinterval(FAR int *interval);

/* Set GNSS of outputting carrier phase info. */

int fw_gd_rtksetgnss(uint32_t gnss);

/* Get GNSS of outputting carrier phase info. */

int fw_gd_rtkgetgnss(FAR uint32_t *pgnss);

/* Set enable/disable GD to notify updating ephemeris */

int fw_gd_rtksetephnotify(int enable);

/* Get enable/disable GD to notify updating ephemeris */

int fw_gd_rtkgetephnotify(FAR int *enable);

/* Set the Ephemeris data Ephemeris data size is variable. */

int fw_gd_setvarephemeris(uint32_t *ephemeris, uint32_t ephemerissize);

/* Get the Ephemeris data Ephemeris data size is variable. */

int fw_gd_getvarephemeris(uint32_t satellite, uint32_t *ephemeris,
                       uint32_t ephemerissize);

/* Set usecase mode */

int fw_gd_setusecase(uint32_t usecase);

/* Get usecase mode */

int fw_gd_getusecase(uint32_t *usecase);

/* Set enable or disable of 1PPS output */

int fw_gd_set1ppsoutput(uint32_t enable);

/* Get the current 1PPS output setting */

int fw_gd_get1ppsoutput(uint32_t *enable);

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_API_H */
