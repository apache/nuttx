/****************************************************************************
 * arch/arm/include/cxd56xx/gnss.h
 *
 *   Copyright 2018,2019 Sony Semiconductor Solutions Corporation
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_GNSS_H
#define __ARCH_ARM_INCLUDE_CXD56XX_GNSS_H

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/gnss_type.h>

/* Start the positioning.
 * This command is used to start the positioning.
 *
 * param arg
 * Start mode value of uint32_t. Not address pointer.
 * One of #CXD56_GNSS_STMOD_COLD,
 *        #CXD56_GNSS_STMOD_WARM,
 *        #CXD56_GNSS_STMOD_HOT
 */

#define CXD56_GNSS_IOCTL_START 1

/* Stop the positioning.
 * This command is used to stop the positioning. GNSS will transition to idle
 * mode.
 *
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GNSS_IOCTL_STOP 2

/* Select the satellite systems.
 * This command is used to select the satellite systems to be used for
 * positioning.
 * The satellite system is defined as a bit, and you can select multiple
 * satellites using OR value.
 * Do not specify zero, please select at least one system.
 * This command must be issued in idle mode.
 *
 * param[in] arg
 * Satellite system bitmap value of uint32_t. Not address pointer.
 * Bit OR of #CXD56_GNSS_SAT_GPS, #CXD56_GNSS_SAT_GLONASS,
 * #CXD56_GNSS_SAT_SBAS, #CXD56_GNSS_SAT_QZ_L1CA, #CXD56_GNSS_SAT_QZ_L1S.
 */

#define CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM 3

/* Get satellite system to be used for measurement GNSS.
 * The satellite system setting of the subscriber that a
 * SF_COMMAND_COMMON_START command was published is acquired.
 *
 * param[out] arg
 * Address pinting to of uint32_t object.
 * Bit OR of #CXD56_GNSS_SAT_GPS, #CXD56_GNSS_SAT_GLONASS,
 * #CXD56_GNSS_SAT_SBAS, #CXD56_GNSS_SAT_QZ_L1CA, #CXD56_GNSS_SAT_QZ_L1S.
 */

#define CXD56_GNSS_IOCTL_GET_SATELLITE_SYSTEM 4

/* Set the receiver approximate position.
 * The receiver position is set using ellipsoidal coordinates (latitude,
 * longitude, altitude).
 * If ellipsoidal coordinates setting, north latitude and east longitude
 * direction as positive.
 * When specifying south latitude or west longitude, set it to a negative
 * value.
 *
 * The receiver position, current time and TCXO offset value, ephemeris data
 * are required in order to initiate a hot start
 * so the position must have been set to GNSS using this command prior to hot
 * start.
 * Set GNSS configure with this command before hot start if receiver position
 * is not stored in the memory.
 * This command must be issued in idle mode.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_ellipsoidal_position_s object.
 */

#define CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ELLIPSOIDAL 5

/* Set the receiver approximate position.
 * The receiver position is set using orthogonal coordinates (x, y, z).
 * The receiver position, current time and TCXO offset value, ephemeris data
 * are required in order to initiate a hot start.
 * Set the position with this command before hot start if no position
 * information in GNSS.
 * This command must be issued in idle mode.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_orthogonal_position_s object.
 */

#define CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ORTHOGONAL 6

/* Set receiver operation mode.
 * Can set operation mode and measurement cycle.
 * The cycle data is 1000msec aligned only and cannot set 0msec.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_ope_mode_param_s object.
 */

#define CXD56_GNSS_IOCTL_SET_OPE_MODE 7

/* Get receiver operation mode.
 *
 * param[out] arg
 * Address pointing to struct #cxd56_gnss_ope_mode_param_s object.
 */

#define CXD56_GNSS_IOCTL_GET_OPE_MODE 8

/* Set receiver TCXO offset value to GNSS.
 * Receiver TCXO offset value set in "Hz". It can specify positive direction
 * or negative direction by a sign int the argument.
 * The receiver position, current time and TCXO offset value, ephemeris data
 * are required in order to initiate a hot start.
 * Set the offset with this command before hot start if no offset information
 * in GNSS.
 * This command must be issued in idle mode.
 *
 * param[in] arg
 * TCXO offset value of uint32_t. Not address pointer.
 * Address pointing to a offset value(int32_t) object.
 */

#define CXD56_GNSS_IOCTL_SET_TCXO_OFFSET 9

/* Get receiver TCXO offset value.
 *
 * param[out] arg
 * Address pointing to int32_t object.
 */

#define CXD56_GNSS_IOCTL_GET_TCXO_OFFSET 10

/* Set receiver time to GNSS.
 * The UTC time standard is used for the receiver time stored in a argument
 * of cxd56_gnss_datetime type.
 * The receiver position, current time and TCXO offset value, ephemeris data
 * are required in order to initiate a hot start.
 * Set the time with this command before hot start if GPS time is not counted
 * in RTC.
 * This command must be issued in idle mode.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_datetime_s object.
 */

#define CXD56_GNSS_IOCTL_SET_TIME 11

/* Get the latest almanac data extracted from the satellite signal.
 * Almanac data size is 2048(GPS) or 576(GLONASS) bytes.
 * This command must be issued in idle mode.
 *
 * param[out] arg
 * Address pointing to struct #cxd56_gnss_orbital_param_s object.
 */

#define CXD56_GNSS_IOCTL_GET_ALMANAC 12

/* Set almanac data.
 * Almanac data size is 2048(GPS) or 576(GLONASS) bytes.
 * This command must be issued in idle mode.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_orbital_param_s object.
 */

#define CXD56_GNSS_IOCTL_SET_ALMANAC 13

/* Get the latest ephemeris data extracted from the satellite signal.
 * Ephemeris data size is 3072(GPS) or 1152(GLONASS) bytes.
 * This command must be issued in idle mode.
 *
 * param[out] arg
 * Address pointing to struct #cxd56_gnss_orbital_param_s object.
 */

#define CXD56_GNSS_IOCTL_GET_EPHEMERIS 14

/* Set ephemeris data.
 * Ephemeris data size is 3072(GPS) or 1152(GLONASS) bytes.
 * This command must be issued in idle mode.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_orbital_param_s object.
 */

#define CXD56_GNSS_IOCTL_SET_EPHEMERIS 15

/* This command is used to save the backup data. The backup data contents are
 * saved in the flash memory.
 * The backup data saved in the flash memory is automatically restored at
 * boot-up of GNSS.
 * The receiver position, ephemeris, almanac, TCXO offset and other
 * information required
 * for hot start are included in the backup data.
 * By saving it with this command, you can restore the information necessary
 * for starting except time at boot time.
 * This command must be issued in idle mode.
 *
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GNSS_IOCTL_SAVE_BACKUP_DATA 16

/* Erase backup data on flash memory.
 * This command must be issued in idle mode.
 *
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GNSS_IOCTL_ERASE_BACKUP_DATA 17

/* Open CEP data file for GNSS device.
 *
 * Open the CEP data file and make the data available from the device.
 * The file name to be opened is specified by Kconfig.
 * This command must be issued in idle mode, between calling open function
 * for GNSS device and issuing the command CXD56_GNSS_IOCTL_START.
 *
 * param[in] arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GNSS_IOCTL_OPEN_CEP_DATA 18

/* Close the CEP data file opened with the command
 * CXD56_GNSS_IOCTL_OPEN_CEP_DATA.
 * This command must be issued in idle mode.
 *
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GNSS_IOCTL_CLOSE_CEP_DATA 19

/* Check the validity of the CEP assist data.
 * Return error code if the data file dose not exist or data is invalid.
 *
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GNSS_IOCTL_CHECK_CEP_DATA 20

/* Get age(lifetime) information of assist data.
 * Age includes start date and time represented by the Julian date(MJD), and
 * validity period.
 * Conversion MJD to YMD is following formula.
 * (Assist data start time must 0:00 (GPS time))\n
 * If the data file dose not exist, an error will be returned.
 *
 * JD12 = floor(MJD + 2400001)
 * e = floor((JD12 - 1867216.25) / 36524.25)
 * f = JD12 + (e - floor(e / 4) + 1)
 * g = f + 1524
 * h = floor((g - 122.1) / 365.25)
 * i = floor(365.25 * h)
 * j = floor((g - i) / 30.6001)
 *
 * d = g - i - floor(30.6001 * j)
 * m = j - 12 * floor(j / 14) - 1
 * w = h - 4716 + floor((14 - m) / 12)
 *
 * if w > 0 then y = w else y = w - 1
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_cep_age_s object.
 */

#define CXD56_GNSS_IOCTL_GET_CEP_AGE 21

/* Reset CEP assist data init flag & valid flag.
 * This command for notifying that the data has been updated.
 * Execute this command before measurement if assist data updated.
 *
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GNSS_IOCTL_RESET_CEP_FLAG 22

/* Currently version not supported. */

#define CXD56_GNSS_IOCTL_RTK_START 23

/* Currently version not supported. */

#define CXD56_GNSS_IOCTL_RTK_STOP 24

/* Currently version not supported. */

#define CXD56_GNSS_IOCTL_RTK_SET_INTERVAL 25

/* Currently version not supported. */

#define CXD56_GNSS_IOCTL_RTK_GET_INTERVAL 26

/* Currently version not supported. */

#define CXD56_GNSS_IOCTL_RTK_SELECT_SATELLITE_SYSTEM 27

/* Currently version not supported. */

#define CXD56_GNSS_IOCTL_RTK_GET_SATELLITE_SYSTEM 28

/* Currently version not supported. */

#define CXD56_GNSS_IOCTL_RTK_SET_EPHEMERIS_ENABLER 29

/* Currently version not supported. */

#define CXD56_GNSS_IOCTL_RTK_GET_EPHEMERIS_ENABLER 30

/* Set acquist data.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_agps_acquist_s object.
 */

#define CXD56_GNSS_IOCTL_AGPS_SET_ACQUIST 31

/* Set frame time.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_agps_frametime_s object.
 */

#define CXD56_GNSS_IOCTL_AGPS_SET_FRAMETIME 32

/* Set tau_GPS(&tau;GPS: Difference of system time between GPS and Glonass
 * system).
 *
 * param[in] arg
 * Address pointing to tau GPS value(&tau;GPS, double) object.
 */

#define CXD56_GNSS_IOCTL_AGPS_SET_TAU_GPS 33

/* Set high precision receiver time.
 *
 * param arg
 * Address pointing to struct #cxd56_gnss_agps_time_gps_s object.
 */

#define CXD56_GNSS_IOCTL_AGPS_SET_TIME_GPS 34

/* Clear info(s) for hot start.
 *
 * param arg
 * Bit OR of #CXD56_GNSS_GCLR_EPH, #CXD56_GNSS_GCLR_ALM, #CXD56_GNSS_GCLR_PV,
 * #CXD56_GNSS_GCLR_TIME, #CXD56_GNSS_GCLR_TCXO, #CXD56_GNSS_GCLR_ALL.
 */

#define CXD56_GNSS_IOCTL_AGPS_CLEAR_RECEIVER_INFO 35

/* Set acquist data.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_agps_tow_assist_s object.
 */

#define CXD56_GNSS_IOCTL_AGPS_SET_TOW_ASSIST 36

/* Set acquist data.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_agps_utc_model_s object.
 */

#define CXD56_GNSS_IOCTL_AGPS_SET_UTC_MODEL 37

/* Enable or not to output spectrum data of GNSS signal.
 *
 * param arg
 * Address pointing to struct #cxd56_gnss_spectrum_control_s object.
 */

#define CXD56_GNSS_IOCTL_SPECTRUM_CONTROL 38

/* Start GPS factory test.
 * Test results can get by CXD56_GNSS_IOCTL_FACTORY_GET_TEST_RESULT command
 * after execute this command and waiting 1 second.
 * This command execute during measurement stop.
 * After executing this command, it is not accepted except for
 * CXD56_GNSS_IOCTL_FACTORY_GET_TEST_RESULT and
 * CXD56_GNSS_IOCTL_FACTORY_STOP_TEST.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_test_info_s object.
 */

#define CXD56_GNSS_IOCTL_FACTORY_START_TEST 39

/* Stop GPS factory test.
 *
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GNSS_IOCTL_FACTORY_STOP_TEST 40

/* Get GPS factory test result.
 *
 * param[out] arg
 * Address pointing to struct #cxd56_gnss_test_result_s object.
 */

#define CXD56_GNSS_IOCTL_FACTORY_GET_TEST_RESULT 41

/* Set signal information for GNSS events from GNSS device driver.
 * The field 'enable' of struct #cxd56_gnss_signal_setting_s to be given
 * as a parameter must be specified as 1 when setting and 0 when unsetting.
 * param[out] arg
 * Address pointing to struct #cxd56_gnss_signal_setting_s object.
 */

#define CXD56_GNSS_IOCTL_SIGNAL_SET 42

/* Start PVTLOG.
 * Automatically saves the PVT log in the GNSS core.
 *
 * param arg
 * Address pointing to #cxd56_pvtlog_setting_s object.
 */

#define CXD56_GNSS_IOCTL_PVTLOG_START 43

/* Stop PVTLOG.
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GNSS_IOCTL_PVTLOG_STOP 44

/* Delete PVTlog data.
 * Delete the log data saved in the GNSS core.
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GNSS_IOCTL_PVTLOG_DELETE_LOG 45

/* Get PVTLOG status.
 * This command is for getting the log stored status.
 * The status data include stored log data count and logging time.
 *
 * param arg
 * Address pointing to #cxd56_pvtlog_status_s.
 */

#define CXD56_GNSS_IOCTL_PVTLOG_GET_STATUS 46

/* Currently version not supported. */

#define CXD56_GNSS_IOCTL_NAVMSG_START 47

/* Set ephemeris data.
 * Only satellites with data are output.
 * Ephemeris data size is variable.
 * Ephemeris data max size is 3072(GPS) or 1152(GLONASS) bytes.
 * This command must be issued in idle mode.
 *
 * param[in] arg
 * Address pointing to struct #cxd56_gnss_set_var_ephemeris_s object.
 */

#define CXD56_GNSS_IOCTL_SET_VAR_EPHEMERIS 48

/* Get the latest ephemeris data extracted from the satellite signal.
 * Only satellites with data are output.
 * Ephemeris data size is variable.
 * Ephemeris data max size is 3072(GPS) or 1152(GLONASS) bytes.
 * This command must be issued in idle mode.
 *
 * param[out] arg
 * Address pointing to struct #cxd56_gnss_get_var_ephemeris_s object.
 */

#define CXD56_GNSS_IOCTL_GET_VAR_EPHEMERIS 49

/* Set usecase mode
 * This command must be issued in idle mode.
 *
 * param[in] arg
 * The usecase definitions
 */

#define CXD56_GNSS_IOCTL_SET_USECASE 50

/* Get usecase mode
 *
 * param[out] arg
 * The usecase definitions
 */

#define CXD56_GNSS_IOCTL_GET_USECASE 51

/* Set enable or disable of 1PPS output
 * This command must be issued in idle mode.
 *
 * param[in] arg
 * enable(1) or disable(0)
 */

#define CXD56_GNSS_IOCTL_SET_1PPS_OUTPUT 52

/**
 * Get the current 1PPS output setting
 *
 * @param[out] arg
 * enable(1) or disable(0)
 */

#define CXD56_GNSS_IOCTL_GET_1PPS_OUTPUT 53

/* check macros for GNSS commands */

#define CXD56_GNSS_IOCTL_INVAL 0
#define CXD56_GNSS_IOCTL_MAX   54

/* Same value to GD Start mode CXD56_GNSS_STMOD_XXXX for fw_gd_start */

#define CXD56_GNSS_STMOD_COLD       0 /* Cold Start */
#define CXD56_GNSS_STMOD_WARM       1 /* Warm Start */
#define CXD56_GNSS_STMOD_WARM_ACC2  2 /* Warm Start, better accuracy, less TTFF than WARM */
#define CXD56_GNSS_STMOD_HOT        3 /* Hot Start */
#define CXD56_GNSS_STMOD_HOT_ACC    4 /* Hot Start, better accuracy, less TTFF than HOT */
#define CXD56_GNSS_STMOD_HOT_ACC2   5 /* Hot Start, better accuracy, less TTFF than ACC */
#define CXD56_GNSS_STMOD_HOT_ACC3   6 /* Optimized hot start, better TTFF than HOT */
#define CXD56_GNSS_STMOD_GSPQ       CXD56_GNSS_STMOD_HOT_ACC3

#define CXD56_GNSS_GPS_ALMANAC_SIZE       2048 /* GPS Almanac Size */

#define CXD56_GNSS_GPS_EPHEMERIS_SIZE     3072 /* GPS Ephemeris Size */

#define CXD56_GNSS_GLONASS_ALMANAC_SIZE   576  /* GLONASS Almanac Size */

#define CXD56_GNSS_GLONASS_EPHEMERIS_SIZE 1152 /* GLONASS Ephemeris Size */

#define CXD56_GNSS_QZSSL1CA_ALMANAC_SIZE  640  /* GPS Almanac Size */

#define CXD56_GNSS_QZSSL1CA_EPHEMERIS_SIZE 960 /* GPS Ephemeris Size */

/* PVTLOG notify threshold of the stored data. */

#define CXD56_GNSS_PVTLOG_THRESHOLD_FULL            0 /* Limit of the storage size */
#define CXD56_GNSS_PVTLOG_THRESHOLD_HALF            1 /* 1/2 of the Storage size */
#define CXD56_GNSS_PVTLOG_THRESHOLD_ONE_DATA        2 /* Each log stored */

/* Offset for last GNSS data */

#define CXD56_GNSS_READ_OFFSET_LAST_GNSS    0x0000

/* Offset for GNSS data */

#define CXD56_GNSS_READ_OFFSET_GNSS(N)      (0x1000 + 0x800 * (N))

/* Offset for AGPS data */

#define CXD56_GNSS_READ_OFFSET_AGPS         0x5000

/* Offset for RTK data */

#define CXD56_GNSS_READ_OFFSET_RTK          0x6000

/* Offset for RTK GPS Ephemeris data */

#define CXD56_GNSS_READ_OFFSET_GPSEPHEMERIS 0x7000

/* Offset for RTK GLONASS Ephemeris data */

#define CXD56_GNSS_READ_OFFSET_GLNEPHEMERIS 0x8000

/* Offset for SBAS data */

#define CXD56_GNSS_READ_OFFSET_SBAS         0x9000

/* Offset for DC report */

#define CXD56_GNSS_READ_OFFSET_DCREPORT     0x9800

/* Offset for SAR/RLM */

#define CXD56_GNSS_READ_OFFSET_SARRLM       0x9900

/* Offset for Spectrum data */

#define CXD56_GNSS_READ_OFFSET_SPECTRUM     0xa000

/* Offset for GNSS info */

#define CXD56_GNSS_READ_OFFSET_INFO         0xf000

/* Offset for PVTLOG data */

#define CXD56_GNSS_READ_OFFSET_PVTLOG       0x10000

/* Signal types from GNSS */

/* Signal type is GNSS */

#define CXD56_GNSS_SIG_GNSS         0

/* Signal type is PVTLog */

#define CXD56_GNSS_SIG_PVTLOG       2

/* Signal type is AGPS */

#define CXD56_GNSS_SIG_AGPS         3

/* Signal type is RTK Career Phase */

#define CXD56_GNSS_SIG_RTK          4

/* Signal type is Soectrum */

#define CXD56_GNSS_SIG_SPECTRUM     5

/* Signal type is RTK GPS Ephemeris */

#define CXD56_GNSS_SIG_GPSEPHEMERIS 11

/* Signal type is RTK GLONASS Ephemeris */

#define CXD56_GNSS_SIG_GLNEPHEMERIS 12

/* Signal type is SBAS */

#define CXD56_GNSS_SIG_SBAS         14

/* Signal type is QZSS DC report */

#define CXD56_GNSS_SIG_DCREPORT     15

/* Signal type is GAL SAR/RLM */

#define CXD56_GNSS_SIG_SARRLM       16

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GNSS operation mode and cycle */

struct cxd56_gnss_ope_mode_param_s
{
  /* receiver operation mode
   *    0: No Change Operation
   *    1: Normal(default)
   */

  uint32_t mode;

  /* Positioning cycle[ms]
   *  The cycle data is 1000msec aligned only and
   *  cannot set 0msec.
   *  (Init value 1000)
   */

  uint32_t cycle;
};

/* Satellite almanac, ephemeris data */

struct cxd56_gnss_orbital_param_s
{
  uint32_t      type; /* One of #CXD56_GNSS_DATA_GPS,
                       *   #CXD56_GNSS_DATA_GLONASS or
                       *   #CXD56_GNSS_DATA_QZSSL1CA.
                       */
  FAR uint32_t *data; /* Address pointing to almanac or ephemeris data buffer */
};

/* date and time */

struct cxd56_gnss_datetime_s
{
  struct cxd56_gnss_date_s date; /* date */
  struct cxd56_gnss_time_s time; /* time */
};

/* ellipsoidal position */

struct cxd56_gnss_ellipsoidal_position_s
{
  double latitude;  /* latitude[degree] */
  double longitude; /* longitude[degree] */
  double altitude;  /* altitude[meter] */
};

/* ellipsoidal position */

struct cxd56_gnss_orthogonal_position_s
{
  int32_t x; /* x[meter] */
  int32_t y; /* y[meter] */
  int32_t z; /* z[meter] */
};

/* Currently version not supported. */

struct cxd56_gnss_cep_data_s
{
  FAR uint32_t *data;
  uint32_t      size;
  uint32_t      counter;
};

/* CEP age info */

struct cxd56_gnss_cep_age_s
{
  float age;  /* age */
  float cepi; /* cepi */
};

/* acquist data and size for AGPS */

struct cxd56_gnss_agps_acquist_s
{
  FAR uint8_t *data; /* Address pointing to aquist data buffer */
  uint16_t     size; /* Aquist data size */
};

/* tow assist data and size for AGPS */

struct cxd56_gnss_agps_tow_assist_s
{
  FAR uint8_t *data; /* Address pointing to tow assist data buffer */
  uint16_t     size; /* assist data size */
};

/* utc model data and size for AGPS */

struct cxd56_gnss_agps_utc_model_s
{
  FAR uint8_t *data; /* Address pointing to utc model data buffer */
  uint16_t     size; /* utc model data size */
};

/* Time from frame start[sec]. */

struct cxd56_gnss_agps_frametime_s
{
  uint16_t sec;  /* Integer part  */
  uint32_t frac; /* Fraction part */
};

/* High precision receiver time */

struct cxd56_gnss_agps_time_gps_s
{
  struct cxd56_gnss_date_s date; /* Date */
  struct cxd56_gnss_time_s time; /* Time */
};

/* different time between AGPS and GLONASS Time */

struct cxd56_gnss_agps_tau_gps_s
{
  double    taugps;     /* tau Time */
};

/* Signal spectrum output control parameter */

struct cxd56_gnss_spectrum_control_s
{
  uint8_t  enable; /* Spectrum data output enable */
  uint32_t time;   /* Passed Time */
  uint8_t  point1; /* Monitor point1 (7-9) */
  uint8_t  point2; /* Monitor point2 (7-9) */
  uint8_t  step1;  /* Sampling step1 (0-7) */
  uint8_t  step2;  /* Sampling step2 (0-7) */
};

struct cxd56_gnss_test_info_s
{
  uint32_t satellite; /* Specify satellite by svID  */
  uint32_t reserve1;  /* Reserve (always specify 0) */
  uint32_t reserve2;  /* Reserve (always specify 0) */
  uint32_t reserve3;  /* Reserve (always specify 0) */
};

struct cxd56_gnss_test_result_s
{
  float cn;      /* CN ratio [dB-Hz] */
  float doppler; /* Doppler [Hz] */
};

/* signal setting for reading data asynchronously
 * The field 'enable' of struct #cxd56_gnss_signal_setting_s to be given as a
 * parameter must be specified as 1 when setting and 0 when unsetting.
 * Field 'gnsssig' specifies the value of 'Signal types from GNSS',
 * this is not POSIX signal.
 * Field 'signo' is application specific number of POSIX signal.
 * 'data' will be passed as an argument to the handler.
 */

struct cxd56_gnss_signal_setting_s
{
  int       fd;      /* The descriptor for signal handler */
  uint8_t   enable;  /* 1 when set this setting, 0 is clear */
  uint8_t   gnsssig; /* GNSS signal as CXD56_GNSS_SIG_GNSS, _AGPS, etc. */
  int       signo;   /* system signal number to notify read completion */
  FAR void *data;    /* user data */
};

/* Information for use after being signaled to read data asynchronously */

struct cxd56_gnss_signal_info_s
{
  int       fd;      /* The file descriptor to use in signal handler */
  uint8_t   gnsssig; /* GNSS signal as CXD56_GNSS_SIG_GNSS, _AGPS, etc. */
  int       signo;   /* system signal number to notify read completion */
  FAR void *data;    /* user data */
};

/* PVTLOG setting Parameter.
 * If the log interval(cycle) is smaller than the positioning interval,
 * it is logged every positioning interval.
 * The output timing is specified by the ratio to the log buffer in the
 * GNSS device by threshold. Possible values are
 * #CXD56_GNSS_PVTLOG_THRESHOLD_FULL, #CXD56_GNSS_PVTLOG_THRESHOLD_HALF,
 * and #CXD56_GNSS_PVTLOG_THRESHOLD_ONE_DATA.
 */

struct cxd56_pvtlog_setting_s
{
  uint32_t cycle;     /* PVT log interval in seconds */
  uint32_t threshold; /* Notification threshold of log storage amount */
};

struct cxd56_pvtlog_status_s
{
  struct cxd56_gnss_status_s status;   /* The stored logs status */
};

struct cxd56_rtk_setting_s
{
  int       interval; /* RTK data output interval */
  uint32_t  gnss;     /* RTK satellite setting */
  int       ephout;   /* Ephemeris notify enable setting */
  uint64_t  sbasout;  /* sbas notify enable setting */
};

struct cxd56_gnss_set_var_ephemeris_s
{
  uint32_t *data; /* Address pointing to ephemeris data buffer */
  uint32_t  size; /* ephemeris data buffer size */
};

struct cxd56_gnss_get_var_ephemeris_s
{
  uint32_t  type; /* One of #CXD56_GNSS_DATA_GPS, #CXD56_GNSS_DATA_GLONASS. */
  uint32_t *data; /* Address pointing to ephemeris data buffer */
  uint32_t  size; /* ephemeris data buffer size */
};

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_GNSS_H */
