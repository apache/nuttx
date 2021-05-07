/****************************************************************************
 * arch/arm/include/cxd56xx/gnss_type.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_GNSS_TYPE_H
#define __ARCH_ARM_INCLUDE_CXD56XX_GNSS_TYPE_H

/* NOTICE:
 * This file defines a structure that stores GNSS positioning
 * data of CXD 56xx. The public header file gnss_type.h for NuttX
 * of the CXD 56xx SDK has been copied as gd_type.h in the nxloader
 * build system and used. Therefore, if you change the definitions
 * in this file, please synchronize and change the other.
 */

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

#include <stdint.h>

/* Max number of satellites */

#define CXD56_GNSS_MAX_SV_NUM 32

/* GNSS satellite system */

#define CXD56_GNSS_SAT_NONE    (0)       /* None */
#define CXD56_GNSS_SAT_GPS     (1U << 0) /* GPS */
#define CXD56_GNSS_SAT_GLONASS (1U << 1) /* Glonass */
#define CXD56_GNSS_SAT_SBAS    (1U << 2) /* SBAS */
#define CXD56_GNSS_SAT_QZ_L1CA (1U << 3) /* QZSS/L1CA */
#define CXD56_GNSS_SAT_IMES    (1U << 4) /* IMES */
#define CXD56_GNSS_SAT_QZ_L1S  (1U << 5) /* QZSS/L1S */
#define CXD56_GNSS_SAT_BEIDOU  (1U << 6) /* BeiDou */
#define CXD56_GNSS_SAT_GALILEO (1U << 7) /* Galileo */

/* GNSS positioning type */

#define CXD56_GNSS_PVT_TYPE_NONE 0 /* Positioning data none */
#define CXD56_GNSS_PVT_TYPE_GNSS 1 /* by GNSS */
#define CXD56_GNSS_PVT_TYPE_IMES 2 /* by IMES */
#define CXD56_GNSS_PVT_TYPE_USER 3 /* API setting */

/* GNSS position fix mode */

#define CXD56_GNSS_PVT_POSFIX_INVALID 1 /* No measurement */
#define CXD56_GNSS_PVT_POSFIX_2D      2 /* 2D fix */
#define CXD56_GNSS_PVT_POSFIX_3D      3 /* 3D fix */

/* GNSS velocity fix mode */

#define CXD56_GNSS_PVT_VELFIX_INVALID  1 /* No measurement */
#define CXD56_GNSS_PVT_VELFIX_2DVZ     2 /* 2D VZ fix */
#define CXD56_GNSS_PVT_VELFIX_2DOFFSET 3 /* 2D Offset fix */
#define CXD56_GNSS_PVT_VELFIX_3D       4 /* 3D fix */
#define CXD56_GNSS_PVT_VELFIX_1D       5 /* 1D fix */
#define CXD56_GNSS_PVT_VELFIX_PRED     6 /* Prediction */

/* GNSS oribital information data type, almanac & ephemeris */

#define CXD56_GNSS_DATA_GPS        0 /* GPS data type */
#define CXD56_GNSS_DATA_GLONASS    1 /* Glonass data type */
#define CXD56_GNSS_DATA_QZSSL1CA   2 /* QZSS/L1CA data type */
#define CXD56_GNSS_DATA_GALILEO    3 /* GALILEO data type */
#define CXD56_GNSS_DATA_BEIDOU_D1  4 /* BEIDOU D1 data type */
#define CXD56_GNSS_DATA_BEIDOU_D2  5 /* BEIDOU D2 data type */
#define CXD56_GNSS_DATA_BEIDOU_ALM 6 /* BEIDOU Almanac data type */

/* GNSS satellite status */

#define CXD56_GNSS_SV_STAT_NONE          (0)      /* None */
#define CXD56_GNSS_SV_STAT_TRACKING      (1 << 0) /* Tracking */
#define CXD56_GNSS_SV_STAT_POSITIONING   (1 << 1) /* Positioning */
#define CXD56_GNSS_SV_STAT_CALC_VELOCITY (1 << 2) /* Calc Velocity */
#define CXD56_GNSS_SV_STAT_VISIBLE       (1 << 3) /* Visible */
#define CXD56_GNSS_SV_STAT_SUB_CH        (1 << 4) /* Sub Ch */

/* GNSS carrier phase status */

#define CXD56_GNSS_SV_PHASESTAT_NONE       (0)      /* None */
#define CXD56_GNSS_SV_PHASESTAT_UNKNOWN    (1 << 0) /* Unknown */
#define CXD56_GNSS_SV_PHASESTAT_VALID      (1 << 1) /* Valid  */
#define CXD56_GNSS_SV_PHASESTAT_RESET      (1 << 2) /* Reset detected */
#define CXD56_GNSS_SV_PHASESTAT_CYCLE_SLIP (1 << 3) /* Cycle Slip detected */
#define CXD56_GNSS_SV_PHASESTAT_HALF_CYCLE_RESOLVED (1 << 4)
                                          /* Half cycle ambiguity resolved */

/* GNSS Validity of raw measurement */

#define CXD56_GNSS_SV_RAWSTAT_NONE         (0)      /* None */
#define CXD56_GNSS_SV_RAWSTAT_DOPPLER      (1 << 0) /* doppler */
#define CXD56_GNSS_SV_RAWSTAT_PSEUDORANGE  (1 << 1) /* pseudorange  */
#define CXD56_GNSS_SV_RAWSTAT_CARRIERPHASE (1 << 2) /* carrier phase */
#define CXD56_GNSS_SV_RAWSTAT_SVTIME       (1 << 3) /* received sv time */
#define CXD56_GNSS_SV_RAWSTAT_SVCLOCKPARAM (1 << 4) /* correction parameter of sv clock*/
#define CXD56_GNSS_SV_RAWSTAT_SVPOSVEL     (1 << 5) /* sv position and velocity */

/* GNSS Cause of measurement invalidity */

#define CXD56_GNSS_SV_INVCAUS_NOT_TRACKED       (1 << 0) /* not tracked */
#define CXD56_GNSS_SV_INVCAUS_NO_SIGNAL         (1 << 1) /* no signal */
#define CXD56_GNSS_SV_INVCAUS_NO_EPHEMERIS      (1 << 2) /* no ephemeris */
#define CXD56_GNSS_SV_INVCAUS_CLK_NOT_ADJUSTED  (1 << 3) /* clock not adjusted */
#define CXD56_GNSS_SV_INVCAUS_BEFORE_TOW_LOCKED (1 << 4) /* before TOW locked */
#define CXD56_GNSS_SV_INVCAUS_SPEED_LIMIT       (1 << 5) /* speed limit */
#define CXD56_GNSS_SV_INVCAUS_NOT_SUPPORTED     (1 << 6) /* not supported */
#define CXD56_GNSS_SV_INVCAUS_OTHER             (1 << 7) /* other */

/* GNSS 1PPS synchronization status (internal use) */

#define CXD56_GNSS_PPS_NOT_ADJUSTED    0 /* not adjusted */
#define CXD56_GNSS_PPS_ADJUSTED        1 /* adjusted */
#define CXD56_GNSS_PPS_ADJUSTED_SSDGLN 2 /* adjusted SSDGLN */
#define CXD56_GNSS_PPS_ADJUSTED_SSD    3 /* adjusted SSD */
#define CXD56_GNSS_PPS_ADJUSTED_POS    4 /* adjusted POS */
#define CXD56_GNSS_PPS_DEGRADE2        5 /* Degrade2 */
#define CXD56_GNSS_PPS_DEGRADE         6 /* Degrade */
#define CXD56_GNSS_PPS_COMPLETE        7 /* Complete */

/* GNSS Output interval of carrier phase info. */

#define CXD56_GNSS_RTK_INTERVAL_1HZ  1000 /* 1Hz */
#define CXD56_GNSS_RTK_INTERVAL_2HZ  500  /* 2Hz */
#define CXD56_GNSS_RTK_INTERVAL_5HZ  200  /* 5Hz */
#define CXD56_GNSS_RTK_INTERVAL_10HZ 100  /* 10Hz */
#define CXD56_GNSS_RTK_INTERVAL_20HZ 50   /* 20H */

/* Carrier phase max satellite number */

#define CXD56_GNSS_RTK_MAX_SV_NUM  24

/* GNSS Spectrum data size */

/* Spectrum Data Max(adjusted as CXD56_GNSS_SPECTRUM_DATA will be 116byte.) */

#define CXD56_GNSS_SPECTRUM_MAXNUM      37

/* Peak Spectrum Data */

#define CXD56_GNSS_PEAK_SPECTRUM_MAXNUM 3

/* SPZ_INT_API */

/* DC Report data size */

#define CXD56_GNSS_QSM_MSG_BIT_NUM      250
#define CXD56_GNSS_QSM_NUM_BITS_IN_BYTE 8

/* AGPS Measurement tracking data */

#define CXD56_GNSS_SUPL_TRK_DATA_SIZE (24)

/* PVTLOG Max stored log number */

#define CXD56_GNSS_PVTLOG_MAXNUM        170

/* PVTLOG notify threshold of the stored data */

#define CXD56_GNSS_PVTLOG_THRESHOLD_FULL     0 /* Limit of the storage size */
#define CXD56_GNSS_PVTLOG_THRESHOLD_HALF     1 /* 1/2 of the Storage size */
#define CXD56_GNSS_PVTLOG_THRESHOLD_ONE_DATA 2 /* Each log stored */

/* assist bit fields */

#define CXD56_GNSS_PVT_RECEIVER_ASSIST_NONE   (0x00)
#define CXD56_GNSS_PVT_RECEIVER_ASSIST_USER   (0x01)
#define CXD56_GNSS_PVT_RECEIVER_ASSIST_CEPPOS (0x02)
#define CXD56_GNSS_PVT_RECEIVER_ASSIST_CEPVEL (0x04)
#define CXD56_GNSS_PVT_RECEIVER_ASSIST_AEPPOS (0x08)
#define CXD56_GNSS_PVT_RECEIVER_ASSIST_AEPVEL (0x10)

/* GNSS positionig data elements */

/*  Day (UTC) */

struct cxd56_gnss_date_s
{
  uint16_t year;  /* year */
  uint8_t  month; /* month */
  uint8_t  day;   /* day */
};

/* Time (UTC) */

struct cxd56_gnss_time_s
{
  uint8_t  hour;   /* hour */
  uint8_t  minute; /* minitue */
  uint8_t  sec;    /* sec */
  uint32_t usec;   /* usec */
};

/* Time (GPS) */

struct cxd56_gnss_wntow_s
{
  uint32_t tow;        /* truncated TOW (1 = 6sec, 0 ... 100799) */
  uint16_t weeknumber; /* week number (0 ... 1023) */
  uint8_t  sec;        /* offset (0 ... 5) */
  uint8_t  rollover;   /* Number of WN Roll Over (0 ... 255) */
  double   frac;       /* fraction */
};

/* struct cxd56_gnss_dop_s - Dilution Of Precision */

struct cxd56_gnss_dop_s
{
  float pdop;   /* Position DOP */
  float hdop;   /* Horizontal DOP */
  float vdop;   /* Vertical DOP */
  float tdop;   /* Time DOP */
  float ewdop;  /* East-West DOP */
  float nsdop;  /* North-South DOP */
  float majdop; /* Stdev of semi-major axis */
  float mindop; /* Stdev of semi-minor axis */
  float oridop; /* Orientation of semi-major axis [deg] */
};

/* struct cxd56_gnss_var_s - Variance */

struct cxd56_gnss_var_s
{
  float hvar; /* Horizontal */
  float vvar; /* Vertical */
};

/* Extra data for debugging */

#define CXD56_GNSS_PVT_RECEIVER_EXTRA_DATA_SIZE (568)
#define CXD56_GNSS_PVT_RECEIVER_EXTRA_DATA \
  uint8_t extra[CXD56_GNSS_PVT_RECEIVER_EXTRA_DATA_SIZE]
#define CXD56_GNSS_PVT_SV_EXTRA_DATA_SIZE       40
#define CXD56_GNSS_PVT_SV_EXTRA_DATA \
  uint8_t extra[CXD56_GNSS_PVT_SV_EXTRA_DATA_SIZE]
#define CXD56_GNSS_FFT_MAXPOOLNUM   (8)

/* GNSS AGPS clear flag */

#define CXD56_GNSS_GCLR_EPH  0x00000001 /* ephemeris */
#define CXD56_GNSS_GCLR_ALM  0x00000002 /* almanac */
#define CXD56_GNSS_GCLR_PV   0x00000004 /* position and velocity */
#define CXD56_GNSS_GCLR_TIME 0x00000008 /* time */
#define CXD56_GNSS_GCLR_TCXO 0x00010000 /* TCXO offset */
#define CXD56_GNSS_GCLR_ALL  0xffffffff /* all of above */

/* GNSS Receiver data */

struct cxd56_gnss_receiver_s
{
  /* [out] Position type; 0:Invalid, 1:GNSS,
   *       2:IMES, 3:user set, 4:previous
   */

  uint8_t type;

  /* [out] FALSE:SGPS, TRUE:DGPS */

  uint8_t dgps;

  /* [out] 1:Invalid, 2:2D, 3:3D */

  uint8_t pos_fixmode;

  /* [out] 1:Invalid, 2:2D VZ, 3:2D Offset,
   *       4:3D, 5:1D, 6:PRED
   */

  uint8_t vel_fixmode;

  /* [out] Nr of visible satellites */

  uint8_t numsv;

  /* [out] Nr of tracking satellites */

  uint8_t numsv_tracking;

  /* [out] Nr of satellites to calculate position */

  uint8_t numsv_calcpos;

  /* [out] Nr of satellites to calculate velocity */

  uint8_t numsv_calcvel;

  /* [out] bit field
   *     [7..5]Reserved
   *      [4]AEP Velocity
   *      [3]AEP Position
   *      [2]CEP Velocity
   *      [1]CEP Position,
   *      [0]user set
   */

  uint8_t assist;

  /* [out] 0:none, 1:exist */

  uint8_t  pos_dataexist;

  /* [out] Using sv system, bit field;
   *   bit0:GPS, bit1:GLONASS, bit2:SBAS,
   *   bit3:QZSS_L1CA, bit4:IMES,
   *   bit5:QZSS_L1SAIF, bit6:BeiDou,
   *   bit7:Galileo
   */

  uint16_t svtype;

  /* [out] using sv system, bit field;
   *   bit0:GPS, bit1:GLONASS, bit2:SBAS,
   *   bit3:QZSS_L1CA, bit4:IMES,
   *   bit5:QZSS_L1SAIF, bit6:BeiDou,
   *   bit7:Galileo
   */

  uint16_t pos_svtype;

  /* [out] using sv system, bit field; bit0:GPS,
   *   bit0:GPS, bit1:GLONASS, bit2:SBAS,
   *   bit3:QZSS_L1CA, bit4:IMES,
   *   bit5:QZSS_L1SAIF, bit6:BeiDou,
   *   bit7:Galileo
   */

  uint16_t vel_svtype;

  /* [out] position source; 0:Invalid, 1:GNSS,
   *   2:IMES, 3:user set, 4:previous
   */

  uint32_t possource;

  /* [out] TCXO offset[Hz] */

  float                      tcxo_offset;

  /* [out] DOPs of Position */

  struct cxd56_gnss_dop_s    pos_dop;

  /* [out] Weighted DOPs of Velocity */

  struct cxd56_gnss_dop_s    vel_idx;

  /* [out] Accuracy of Position */

  struct cxd56_gnss_var_s    pos_accuracy;

  /* [out] Latitude [degree] */

  double                     latitude;

  /* [out] Longitude [degree] */

  double                     longitude;

  /* [out] Altitude [m] */

  double                     altitude;

  /* [out] Geoid height [m] */

  double                     geoid;

  /* [out] Velocity [m/s] */

  float                      velocity;

  /* [out] Direction [degree] */

  float                      direction;

  /* [out] Current day (UTC) */

  struct cxd56_gnss_date_s   date;

  /* [out] Current time (UTC) */

  struct cxd56_gnss_time_s   time;

  /* [out] Current day (GPS) */

  struct cxd56_gnss_date_s   gpsdate;

  /* [out] Current time (GPS) */

  struct cxd56_gnss_time_s   gpstime;

  /* [out] Receive time (UTC) */

  struct cxd56_gnss_time_s   receivetime;

  /* [out] For internal use */

  uint32_t                   priv;

  /* [out] Leap Second[sec] */

  int8_t                     leap_sec;

  /* [out] elapsed time
   *       from reset in ns
   */

  uint64_t                   time_ns;

  /* [out] elapsed time
   *       from GPS epoch in ns
   */

  int64_t                    full_bias_ns;

  /* [out] Receiver extra data */

  CXD56_GNSS_PVT_RECEIVER_EXTRA_DATA;
};

/* satellite position (ECEF) */

struct cxd56_gnss_pvt_sv_pos_s
{
  double dx; /* X [m] */
  double dy; /* Y [m] */
  double dz; /* Z [m] */
};

/* satellite velocity (ECEF) */

struct cxd56_gnss_pvt_sv_vel_s
{
  float fvx; /* Vx [m/s] */
  float fvy; /* Vy [m/s] */
  float fvz; /* Vz [m/s] */
};

/* GNSS satellite data */

struct cxd56_gnss_sv_s
{
  /* [out] Using sv system, bit field; bit0:GPS,
   *     bit1:GLONASS, bit2:SBAS, bit3:QZSS_L1CA,
   *     bit4:IMES, bit5:QZSS_L1SAIF, bit6:BeiDou,
   *     bit7:Galileo
   *     same as struct cxd56_gnss_receiver_s::svtype
   */

  uint16_t type;

  /* [out] Satellite id */

  uint8_t svid;

  /* Using sv info, bit field; bit0:tracking,
   *     bit1:positioning, bit2:calculating velocity,
   *     bit3:visible satellite
   */

  uint8_t stat;

  /* [out] Azimuth [degree] */

  int16_t azimuth;

  /* [out] carrier phase status, bit field;
   *     bit0: unknown, bit1: valid, bit2: reset,
   *     bit3: cycle slip (Loss of Lock Indicator)
   */

  uint8_t phase_stat;

  /* [out] navigation message of the satellite vehicle
   *     bit0: 1 = almanac OK, 0 = almanac NG
   */

  uint8_t nav_message_info;

  /* [out] Validity of raw measurement, bit field;
   *     bit0: doppler, bit1: pseudorange,
   *     bit2: carrierphase, bit3: svtime,
   *     bit4: svClockOffset & svClockDrift
   *     bit5: svpos & svvel
   */

  uint8_t raw_stat;

  /* [out] Cause of measurement invalidity, bit field;
   *     bit0: not tracked, bit1: no signal,
   *     bit2: no ephemeris
   *     bit3: clock not adjusted,
   *     bit4: before TOW locked
   *     bit5: not supported, bit6: speed limit
   */

  uint8_t invalid_cause;

  /* [out] Elevation [degree] */

  uint8_t elevation;

  /* [out] Frequency Channel (Glonass) */

  int8_t  freqchannel;

  /* [out] CN */

  float   siglevel;

  /* [out] Doppler frequency [Hz] */

  float   doppler;

  /* [out] Pseudo range [m] */

  double  pseudo_range;

  /* [out] Carrier phase [cycle] */

  double  carrier_phase;

  /* [out] received sv time in [sec] */

  double  sv_time;

  /* [out] Time tracked[sec] */

  float   timetracked;

  /* [out] Satellite clock offset [m] (Af0 * C) */

  float   svclockoffset;

  /* [out] Satellite clock drift [m/s] (Af1 * C) */

  float   svclockdrift;

  /* [out] satellite position (ECEF) */

  struct cxd56_gnss_pvt_sv_pos_s svpos;

  /* [out] satellite velocity (ECEF) */

  struct cxd56_gnss_pvt_sv_vel_s svvel;

  /* [out] Sv extra data */

  CXD56_GNSS_PVT_SV_EXTRA_DATA;
};

/* Positioning data with SV data */

struct cxd56_gnss_positiondata_s
{
  uint64_t data_timestamp;                          /* [out] Timestamp  */
  uint32_t status;                                  /* [out] Positioning data
                                                     *   status 0 : Valid,
                                                     *         <0 : Invalid
                                                     */
  uint32_t                 svcount;                 /* [out] Sv data count */
  struct cxd56_gnss_receiver_s receiver;            /* [out] Receiver data */
  struct cxd56_gnss_sv_s sv[CXD56_GNSS_MAX_SV_NUM]; /* [out] Sv data array */
};

/* QZSS DC report data */

struct cxd56_gnss_dcreport_data_s
{
  uint8_t sf[CXD56_GNSS_QSM_MSG_BIT_NUM /
              CXD56_GNSS_QSM_NUM_BITS_IN_BYTE + 1]; /* [out] Message body */
  uint8_t svid;                                     /* [out] Satellite id */
};

/* Galileo SAR/RLM data */

struct cxd56_gnss_sarrlm_data_s
{
  uint8_t  svid;        /* SVID */
  uint8_t  rlmtype;     /* 0x0:Short, 0x1:Long */
  uint8_t  datacmplt;   /* 0xf:Short, 0xff:Long */
  uint8_t  msgcode;     /* Massage code(4bit) */
  uint32_t beacon_id1;  /* Beacon ID (bit[31:0]) */
  uint32_t beacon_id2;  /* Beacon ID (bit[59:32]) */
  uint32_t param1;      /* Short:16bit, Long:96bit(bit[31:0]) */
  uint32_t param2;      /* Long:96bit(bit[63:31]) */
  uint32_t param3;      /* Long:96bit(bit[95:63]) */
};

struct cxd56_gnss_gal_sarrlm_s
{
  struct cxd56_gnss_sarrlm_data_s data;
};

/* SF_EVENT_GNSS_MEASUREMENT_VALUE */

/* SUPL tracking data */

struct cxd56_supl_trkdata_s
{
  uint8_t  gnssid;                   /* [out] sv system
                                      *  GPS:       0x01
                                      *  GLONASS:   0x02
                                      *  SBAS :     0x04
                                      *  QZSS_L1C/A:0x08
                                      */
  uint8_t  signalid;                 /* [out] Always 0 */
  uint8_t  svid;                     /* [out] Satellite Id
                                      *  GPS:            1-32
                                      *  GLONASS:        1-24
                                      *  SBAS:           120-158
                                      *  QZSS_L1C/A      193-197
                                      */
  uint8_t  cn;                       /* [out] CN ratio [dBHz] */
  uint8_t  codephase_ambiguty;       /* Currently version not supported. */
  uint8_t  carriorquality_indicator; /* Currently version not supported. */
  uint8_t  codephase_rmserr;         /* Currently version not supported. */
  uint8_t  multipath_indicator;      /* Currently version not supported. */
  uint32_t codephase;                /* [out] Code Phase[ms] scale: 2-21[ms] */
  uint16_t wholechip;                /* [out] Chip integer part */
  uint16_t fracchip;                 /* [out] Chip frac part */
  uint32_t adr;                      /* Currently version not supported. */
  int16_t  doppler;                  /* [out] Doppler [Hz] */
};

/* SUPL positioning data */

struct cxd56_supl_posdata_s
{
  double   uncertainty_semi_major;    /* [out] Uncertainty semi-major */
  double   uncertainty_semi_minor;    /* [out] Uncertainty semi-minor */
  double   orientation_of_major_axis; /* [out] Orientation of major axis */
  double   uncertainty_altitude;      /* [out] Uncertainty Altitude */
  uint32_t tow;                       /* [out] Time of week [sec]
                                       *   acquisition TOW : 0-604799
                                       *   no acquisition TOW : 0xffffffff
                                       */
  float    frac_sec;                  /* [out] Under second part[sec]
                                       * no acquisition TOW : -1
                                       */
  float    horizontal_accuracy;       /* [out] Horizontal accuracy [m]
                                       * disable : -1
                                       */
  uint16_t ref_frame;                 /* Currently version not supported */
  uint8_t  tod_unc;                   /* [out] Acquisition : 1
                                       * no acquisition : 0
                                       */
  uint8_t  num_of_sat;                /* [out] Tracking Sv number */
};

/* SUPL Measurement data struct */

struct cxd56_supl_mesurementdata_s
{
  /* [out] Supl positioning data */

  struct cxd56_supl_posdata_s supl_pos;

  /* [out] Tracking satellite data */

  struct cxd56_supl_trkdata_s trackingsat[CXD56_GNSS_SUPL_TRK_DATA_SIZE];
};

/* struct cxd56_gnss_timetag_s - Internal time tag */

struct cxd56_gnss_timetag_s
{
  uint32_t msec;              /* [ms] whole millisecond part */
  uint32_t frac;              /* Under millisecond part (0 ... cycle-1) */
  uint16_t cycle;             /* Resolution of 1ms */
};

/* Time and frequency information for RTK */

struct cxd56_rtk_info_s
{
  uint64_t                    timestamp;  /* [out] system timestamp */
  uint64_t                    timesnow;   /* [out] system now times */
  struct cxd56_gnss_wntow_s   wntow;      /* [out] GPS time */
  struct cxd56_gnss_date_s    date;       /* [out] Date (UTC time) */
  struct cxd56_gnss_time_s    time;       /* [out] Time (UTC time) */
  struct cxd56_gnss_timetag_s tag;        /* [out] TimeTag */
  double                      clockdrift; /* [out] [Hz] clock drift @1.5GHz
                                           *     (valid only if cdvalidity is 1)
                                           */
  int8_t                      cdvalidity; /* [out] clock drift validity
                                           *  (0: invalid, 1: valid)
                                           */

  uint8_t ppsstatus; /* [out] 1PPS synchronization status */
  int8_t svcount;    /* [out] Num of sv */
};

/* Carrier phase and related data for RTK */

struct cxd56_rtk_sv_s
{
  double   pseudorange;  /* [out] [m] pseudo range */
  double   carrierphase; /* [out] [wave number] carrier phase */
  uint32_t gnss;         /* [out] GNSS type (CXD56_GNSS_GNSS_XXX) */
  int8_t   svid;         /* [out] Satellite id */
  int8_t   fdmch;        /* [out] Frequency slot for GLONASS (-7 ... 6) */
  int8_t   polarity;     /* [out] Carrier polarity
                          * (0: not inverted, 1: inverted)
                          */
  int16_t  cn;           /* [out] [0.01dBHz] CN */
  int8_t   lastpreamble; /* [out] Parity of last premable (0: ok, 1: ng) */
  int8_t   lli;          /* [out] Lock loss indicator
                          * (0: no lock loss, 1: lock loss)
                          */
  int8_t   ch;           /* [out] TRK channel number */
  float    c2p;          /* [out] C2P (0 ... 1.0) */
  uint8_t  phase_stat;   /* [out] carrier phase status, bit field;
                          *    bit0: unknown, bit1: valid, bit2: reset,
                          *    bit3: cycle slip (Loss of Lock Indicator)
                          */
  float    timetracked;  /* [out] Tracked time[sec] */
  double   sv_time;      /* [out] received transmit time of the SV[sec] */
  float    doppler;      /* [out] [Hz] Doppler shift */
};

/* RTK Carrier phase data */

struct cxd56_rtk_carrierphase_s
{
  /* [out] Time and frequency information */

  struct cxd56_rtk_info_s infoout;

  /* [out] Carrier phase and related data */

  struct cxd56_rtk_sv_s svout[CXD56_GNSS_RTK_MAX_SV_NUM];
};

/* Ephemeris data (GPS) */

struct cxd56_rtk_gpsephemeris_s
{
  uint64_t timesnow;   /* [out] system now times */
  uint8_t  ppsstatus;  /* [out] 1PPS synchronization status */
  uint16_t t_oc;       /* [out] SV Clock Correction */
  double   af0;        /* [out] SV Clock Correction */
  double   af1;        /* [out] SV Clock Correction */
  double   af2;        /* [out] SV Clock Correction */
  double   crs;        /* [out] Amplitude correction term of orbital radius(sin) */
  double   delta_n;    /* [out] Average motion difference [rad] */
  double   m0;         /* [out] Average near point separation at t_oe [rad] */
  double   cuc;        /* [out] Latitude amplitude correction term(cos) */
  double   e;          /* [out] Eccentricity of orbit */
  double   cus;        /* [out] Latitude amplitude correction term(sin) */
  double   sqrt_a;     /* [out] Square root of the orbital length radius */
  double   cic;        /* [out] Amplitude correction term of orbital inclination angle(cos) */
  double   omega0;     /* [out] Rise of ascension at Weekly Epoch [rad] */
  double   cis;        /* [out] Amplitude correction term of orbital inclination angle(sin) */
  double   i0;         /* [out] Orbital inclination angle at t_oe */
  double   crc;        /* [out] Amplitude correction term of orbital radius(cos) */
  double   omega;      /* [out] Perigee argument [rad] */
  double   omega_dot;  /* [out] Ascension of ascending node correction [rad] */
  double   i_dot;      /* [out] Orbital inclination angle correction [rad] */
  double   accuracy;   /* [out] nominal URA (User Range Accuracy) [m] */
  double   tgd;        /* [out] Estimated Group Delay Differential */

  /* tocwntow, tocdate, toctime are valid if ppsstatus >= 1(adjusted) */

  struct cxd56_gnss_wntow_s tocwntow; /* [out] toc */
  struct cxd56_gnss_date_s tocdate;   /* [out] toc Date */
  struct cxd56_gnss_time_s toctime;   /* [out] toc Time */

  int32_t  toe;         /* [out] Reference time [s] */
  int32_t  tow;         /* [out] Time of Week (truncated) */
  int16_t  id;          /* [out] Satellite id */
  uint8_t  iode;        /* [out] Issue of Data (Ephemeris) Subframe 2 */
  int8_t   codes_on_l2; /* [out] Code(s) on L2 Channel */
  int16_t  weeknumber;  /* [out] Full week number */
  int8_t   l2p;         /* [out] Data Flag for L2 P-Code */
  uint8_t  health;      /* [out] SV Health (6bit for ephemeris / 8bit for almanac) */
  int16_t  iodc;        /* [out] Issue of Data, Clock (IODC) */
  int8_t   fitinterval; /* [out] Fit interval flag */
};

/* Ephemeris data (GLONASS) */

struct cxd56_rtk_glonassephemeris_s
{
  uint64_t timesnow;  /* [out] system now times */
  uint32_t valid;     /* [out] valid */
  uint8_t  ppsstatus; /* [out] 1PPS synchronization status */
  uint8_t  slot;      /* [out] slot 1...24 (It generates from svid. Usually same as me->n) */
  int8_t   ch;        /* [out] ch -7...6 */
  uint8_t  p1;        /* [out] The difference of t_b from the previous frame */
  uint8_t  tk_h;      /* [out] Current frame first time (hours) */
  uint8_t  tk_m;      /* [out] Current frame first time (minutes) */
  uint8_t  tk_s;      /* [out] Current frame first time (seconds) */
  double   xv;        /* [out] The velocity vector components of t_b */
  float    xa;        /* [out] The acceleration components of t_b */
  double   xp;        /* [out] The position of t_b */
  uint8_t  bn;        /* [out] The health info */
  uint8_t  p2;        /* [out] flag of oddness ("1") or evenness ("0") of the value of t_b */
  uint16_t tb;        /* [out] Reference time t_b (15...1425) */
  uint8_t  hn_e;      /* [out] Carrier frequency number (0...31, (25...31)=(7...-1)) */
  double   yv;        /* [out] The velocity vector components of t_b */
  float    ya;        /* [out] The acceleration components of t_b */
  double   yp;        /* [out] The position of t_b */
  uint8_t  p3;        /* [out] Number of almanacs in the current frame */
  float    gn;        /* [out] Carrier frequency relative deviation of t_b */
  uint8_t  p;         /* [out] Origin of tau variable */
  uint8_t  health;    /* [out] Health flag */
  double   zv;        /* [out] The velocity vector components of t_b */
  float    za;        /* [out] The acceleration components of t_b */
  double   zp;        /* [out] The position of t_b */
  float    tn;        /* [out] Correction to the satellite time t_n relative to GLONASS time t_c */
  float    dtn;       /* [out] Difference in internal delay between L2 and L1 */
  uint8_t  en;        /* [out] Number of days from when data was uploaded until  t_b (0...31) */
  uint8_t  p4;        /* [out] Flag of ephemeris parameters updating */
  uint8_t  ft;        /* [out] The URA (index) of t_b */
  uint16_t nt;        /* [out] Number of days since 1/1 of a leap year */
  uint8_t  n;         /* [out] Slot number of the signaling satellite (0...31) */
  uint8_t  m;         /* [out] Satellite type (0...3) */
};

/* Spectrum Data */

struct cxd56_gnss_spectrum_s
{
  uint8_t  status;       /* FFT Sampling Point 0-1 */
  uint8_t  samplingstep; /* FFT Sampling Step  1-16 */
  uint8_t  sizemode;     /* FFT Sampling Num   0:1024 1:512 2:256 */
  uint8_t  datacount;    /* Spectrum data count  */
  uint8_t  datanum;      /* Spectrum data Size   */
  uint8_t  ifgain;       /* IfGain  0-15         */
  uint16_t dataindex;    /* Spectrum data Inex   */

  uint16_t spectrum[CXD56_GNSS_SPECTRUM_MAXNUM];  /* Spectrum Data Buffer */
  double   peak[CXD56_GNSS_PEAK_SPECTRUM_MAXNUM]; /* Peak Spectrum        */
};

#define CXD56_GNSS_SBAS_MESSAGE_DATA_LEN (27)

/* SBAS Data */

struct cxd56_gnss_sbasdata_s
{
  uint64_t timesnow; /* system now times */
  uint32_t gpstow;   /* GPS Time of Week */
  uint16_t gpswn;    /* GPS week number */
  uint16_t svid;     /* satellite id */
  uint8_t  msgid;    /* sbas message ID */

  uint8_t  sbasmsg[CXD56_GNSS_SBAS_MESSAGE_DATA_LEN]; /* sbas message data */
};

/* RTK Setting Parameter */

struct cxd56_gnss_rtk_setting_s
{
  int      interval; /* Interval */
  uint32_t gnss;     /* Satellite system */
  int      eph_out;  /* Ephemeris out enable */
  uint64_t sbas_out; /* sbas out enable */
};

/* PVTLog Parameter */

/* Latitude of PVT data */

struct cxd56_pvtlog_latitude_s
{
  uint32_t frac   :14;  /* Decimal */
  uint32_t minute :6;   /* Minute */
  uint32_t degree :7;   /* Degree */
  uint32_t sign   :1;   /* Sign */
  uint32_t rsv    :4;   /* Reserved */
};

/* Longitude of PVT data */

struct cxd56_pvtlog_longitude_s
{
  uint32_t frac   :14;  /* Decimal */
  uint32_t minute :6;   /* Minute */
  uint32_t degree :8;   /* Degree */
  uint32_t sign   :1;   /* Sign */
  uint32_t rsv    :3;   /* Reserved */
};

/* Altitude of PVT data */

struct cxd56_pvtlog_altitude_s
{
  uint32_t frac   :4;   /* Decimal */
  uint32_t rsv1   :12;  /* Reserved */
  uint32_t meter  :14;  /* Integer */
  uint32_t sign   :1;   /* Sign */
  uint32_t rsv2   :1;   /* Reserved */
};

/* Velocity of PVT data */

struct cxd56_pvtlog_velocity_s
{
  uint16_t knot   :14;  /* Integer */
  uint16_t rsv    :2;   /* Reserved */
};

/* Direction of PVT data */

struct cxd56_pvtlog_direction_s
{
  uint16_t frac   :4;   /* Decimal */
  uint16_t degree :9;   /* Integer */
  uint16_t rsv    :3;   /* Reserved */
};

/* Time (UTC) of PVT data */

struct cxd56_pvtlog_time_s
{
  uint32_t msec   :7;   /* msec */
  uint32_t rsv1   :1;   /* Reserved */
  uint32_t sec    :6;   /* Second */
  uint32_t rsv2   :2;   /* Reserved */
  uint32_t minute :6;   /* Minute */
  uint32_t rsv3   :2;   /* Reserved */
  uint32_t hour   :5;   /* Hour */
  uint32_t rsv4   :3;   /* Reserved */
};

/* Date (UTC) of PVT data */

struct cxd56_pvtlog_date_s
{
  uint32_t year   :7;   /* Year */
  uint32_t day    :5;   /* Day */
  uint32_t month  :4;   /* Month */
  uint32_t rsv    :16;  /* Reserved */
};

/* PVTLog save data struct */

struct cxd56_pvtlog_data_s
{
  struct cxd56_pvtlog_latitude_s   latitude;   /* Latitude of data   4B */
  struct cxd56_pvtlog_longitude_s  longitude;  /* Longitude of data  4B */
  struct cxd56_pvtlog_altitude_s   altitude;   /* Altitude of data   4B */
  struct cxd56_pvtlog_velocity_s   velocity;   /* Velocity of data   2B */
  struct cxd56_pvtlog_direction_s  direction;  /* Direction of data  2B */
  struct cxd56_pvtlog_time_s       time;       /* Time (UTC)         4B */
  struct cxd56_pvtlog_date_s       date;       /* Date (UTC)         4B */
};

/* PVTLog notification data struct */

struct cxd56_pvtlog_s
{
  uint32_t        log_count;                                      /* [in] Valid log count of log_data */
  struct cxd56_pvtlog_data_s  log_data[CXD56_GNSS_PVTLOG_MAXNUM]; /* [in] Stored log data */
};

/* PVTLog Status Data */

struct cxd56_gnss_status_s
{
  uint32_t                     log_count;    /* [in] Saved log count  */
  struct cxd56_pvtlog_time_s   start_time;   /* [in] Time (UTC)   4B  */
  struct cxd56_pvtlog_date_s   start_date;   /* [in] Date (UTC)   4B  */
  struct cxd56_pvtlog_time_s   end_time;     /* [in] Time (UTC)   4B  */
  struct cxd56_pvtlog_date_s   end_date;     /* [in] Date (UTC)   4B  */
};

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_GNSS_TYPE_H */
