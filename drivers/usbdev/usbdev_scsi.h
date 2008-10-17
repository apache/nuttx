/****************************************************************************
 * drivers/usbdev/usbdev_scsi.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __DRIVERS_USBDV_USBDEV_SCSI_H
#define __DRIVERS_USBDV_USBDEV_SCSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* SCSI commands ************************************************************/

#define SCSI_CMD_TESTUNITREADY                   0x00
#define SCSI_CMD_REZEROUNIT                      0x01
#define SCSI_CMD_REQUESTSENSE                    0x03
#define SCSI_CMD_FORMAT_UNIT                     0x04
#define SCSI_CMD_REASSIGNBLOCKS                  0x07
#define SCSI_CMD_READ6                           0x08
#define SCSI_CMD_WRITE6                          0x0a
#define SCSI_CMD_SEEK6                           0x0b
#define SCSI_CMD_SPACE6                          0x11
#define SCSI_CMD_INQUIRY                         0x12
#define SCSI_CMD_MODESELECT6                     0x15
#define SCSI_CMD_RESERVE6                        0x16
#define SCSI_CMD_RELEASE                         0x17
#define SCSI_CMD_COPY                            0x18
#define SCSI_CMD_MODESENSE6                      0x1a
#define SCSI_CMD_STARTSTOPUNIT                   0x1b
#define SCSI_CMD_RECEIVEDIAGNOSTICRESULTS        0x1c
#define SCSI_CMD_SENDDIAGNOSTIC                  0x1d
#define SCSI_CMD_PREVENTMEDIAREMOVAL             0x1e
#define SCSI_CMD_READFORMATCAPACITIES            0x23
#define SCSI_CMD_READCAPACITY                    0x25
#define SCSI_CMD_READ10                          0x28
#define SCSI_CMD_WRITE10                         0x2a
#define SCSI_CMD_SEEK10                          0x2b
#define SCSI_CMD_WRITEANDVERIFY                  0x2e
#define SCSI_CMD_VERIFY                          0x2f
#define SCSI_CMD_SEARCHDATAHIGH                  0x30
#define SCSI_CMD_SEARCHDATAEQUAL                 0x31
#define SCSI_CMD_SEARCHDATALOW                   0x32
#define SCSI_CMD_SETLIMITS10                     0x33
#define SCSI_CMD_PREFETCH                        0x34
#define SCSI_CMD_SYNCHCACHE                      0x35
#define SCSI_CMD_LOCKCACHE                       0x36
#define SCSI_CMD_READDEFECTDATA                  0x37
#define SCSI_CMD_COMPARE                         0x39
#define SCSI_CMD_COPYANDVERIFY                   0x3a
#define SCSI_CMD_WRITEBUFFER                     0x3b
#define SCSI_CMD_READBUFFER                      0x3c
#define SCSI_CMD_READLONG                        0x3d
#define SCSI_CMD_WRITELONG                       0x3f
#define SCSI_CMD_CHANGEDEFINITION                0x40
#define SCSI_CMD_WRITESAME                       0x41
#define SCSI_CMD_LOGSELECT                       0x4C
#define SCSI_CMD_LOGSENSE                        0x4D
#define SCSI_CMD_XDWRITE                         0x50
#define SCSI_CMD_XPWRITE                         0x51
#define SCSI_CMD_XDREAD                          0x52
#define SCSI_CMD_MODESELECT10                    0x55
#define SCSI_CMD_RESERVE10                       0x56
#define SCSI_CMD_RELEASE10                       0x57
#define SCSI_CMD_MODESENSE10                     0x5a
#define SCSI_CMD_PERSISTENTRESERVEIN             0x5e
#define SCSI_CMD_PERSISTENTRESERVEOUT            0x5f
#define SCSI_CMD_XDWRITEEXTENDED                 0x80
#define SCSI_CMD_REGENERATE                      0x82
#define SCSI_CMD_REPORTLUNS                      0xa0
#define SCSI_CMD_MOVEMEDIUM                      0xa5
#define SCSI_CMD_READ12                          0xa8
#define SCSI_CMD_WRITE12                         0xaa
#define SCSI_CMD_SETLIMITS12                     0xb3
#define SCSI_CMD_READELEMENTSTATUS               0xb4
#define SCSI_CMD_READDEFECTDATA12                0xb7

/* Common SCSI KCQ values (sense Key/additional sense Code/ASC Qualifier) ***
 *
 *   0xnn0386  Write Fault Data Corruption
 *   0xnn0500  Illegal request
 *   0xnn0600  Unit attention
 *   0xnn0700  Data protect
 *   0xnn0800  LUN communication failure
 *   0xnn0801  LUN communication timeout
 *   0xnn0802  LUN communication parity error
 *   0xnn0803  LUN communication CRC error
 *   0xnn0900  vendor specific sense key
 *   0xnn0901  servo fault
 *   0xnn0904  head select fault
 *   0xnn0a00  error log overflow
 *   0xnn0b00  aborted command
 *   0xnn0c00  write error
 *   0xnn0c02  write error - auto-realloc failed
 *   0xnn0e00  data miscompare
 *   0xnn1200  address mark not founf for ID field
 *   0xnn1400  logical block not found
 *   0xnn1500  random positioning error
 *   0xnn1501  mechanical positioning error
 *   0xnn1502  positioning error detected by read of medium
 *   0xnn2700  write protected
 *   0xnn2900  POR or bus reset occurred
 *   0xnn3101  format failed
 *   0xnn3191  format corrupted
 *   0xnn3201  defect list update error
 *   0xnn3202  no spares available
 *   0xnn3501  unspecified enclosure services failure
 *   0xnn3700  parameter rounded
 *   0xnn3d00  invalid bits in identify message
 *   0xnn3e00  LUN not self-configured yet
 *   0xnn4001  DRAM parity error
 *   0xnn4002  DRAM parity error
 *   0xnn4200  power-on or self-test failure
 *   0xnn4c00  LUN failed self-configuration
 *   0xnn5c00  RPL status change
 *   0xnn5c01  spindles synchronized
 *   0xnn5c02  spindles not synchronized
 *   0xnn6500  voltage fault
 *   0xnn8000  general firmware error
 */

/* No sense KCQ values */

#define SCSI_KCQ_NOSENSE                         0x000000  /* No error */
#define SCSI_KCQ_PFATHRESHOLDREACHED             0x005c00  /* No sense - PFA threshold reached */

/* Soft error KCQ values */

#define SCSI_KCQSE_RWENOINDEX                    0x010100  /* Recovered Write error - no index */
#define SCSI_KCQSE_RECOVEREDNOSEEKCOMPLETION     0x010200  /* Recovered no seek completion */
#define SCSI_KCQSE_RWEWRITEFAULT                 0x010300  /* Recovered Write error - write fault */
#define SCSI_KCQSE_TRACKFOLLOWINGERROR           0x010900  /* Track following error */
#define SCSI_KCQSE_TEMPERATUREWARNING            0x010b01  /* Temperature warning */
#define SCSI_KCQSE_RWEWARREALLOCATED             0x010c01  /* Recovered Write error with auto-realloc - reallocated */
#define SCSI_KCQSE_RWERECOMMENDREASSIGN          0x010c03  /* Recovered Write error - recommend reassign */
#define SCSI_KCQSE_RDWOEUSINGPREVLBI             0x011201  /* Recovered data without ECC using prev logical block ID */
#define SCSI_KCQSE_RDWEUSINGPREVLBI              0x011202  /* Recovered data with ECC using prev logical block ID */
#define SCSI_KCQSE_RECOVEREDRECORDNOTFOUND       0x011401  /* Recovered Record Not Found */
#define SCSI_KCQSE_RWEDSME                       0x011600  /* Recovered Write error - Data Sync Mark Error */
#define SCSI_KCQSE_RWEDSEDATAREWRITTEN           0x011601  /* Recovered Write error - Data Sync Error - data rewritten */
#define SCSI_KCQSE_RWEDSERECOMMENDREWRITE        0x011602  /* Recovered Write error - Data Sync Error - recommend rewrite */
#define SCSI_KCQSE_RWEDSEDATAAUTOREALLOCATED     0x011603  /* Recovered Write error - Data Sync Error - data auto-reallocated */
#define SCSI_KCQSE_RWEDSERECOMMENDREASSIGNMENT   0x011604  /* Recovered Write error - Data Sync Error - recommend reassignment */
#define SCSI_KCQSE_RDWNECORRECTIONAPPLIED        0x011700  /* Recovered data with no error correction applied */
#define SCSI_KCQSE_RREWITHRETRIES                0x011701  /* Recovered Read error - with retries */
#define SCSI_KCQSE_RDUSINGPOSITIVEOFFSET         0x011702  /* Recovered data using positive offset */
#define SCSI_KCQSE_RDUSINGNEGATIVEOFFSET         0x011703  /* Recovered data using negative offset */
#define SCSI_KCQSE_RDUSINGPREVIOUSLBI            0x011705  /* Recovered data using previous logical block ID */
#define SCSI_KCQSE_RREWOEAUTOREALLOCATED         0x011706  /* Recovered Read error - without ECC, auto reallocated */
#define SCSI_KCQSE_RREWOERECOMMENDREASSIGN       0x011707  /* Recovered Read error - without ECC, recommend reassign */
#define SCSI_KCQSE_RREWOERECOMMENDREWRITE        0x011708  /* Recovered Read error - without ECC, recommend rewrite */
#define SCSI_KCQSE_RREWOEDATAREWRITTEN           0x011709  /* Recovered Read error - without ECC, data rewritten */
#define SCSI_KCQSE_RREWE                         0x011800  /* Recovered Read error - with ECC */
#define SCSI_KCQSE_RDWEANDRETRIES                0x011801  /* Recovered data with ECC and retries */
#define SCSI_KCQSE_RREWEAUTOREALLOCATED          0x011802  /* Recovered Read error - with ECC, auto reallocated */
#define SCSI_KCQSE_RREWERECOMMENDREASSIGN        0x011805  /* Recovered Read error - with ECC, recommend reassign */
#define SCSI_KCQSE_RDUSINGECCANDOFFSETS          0x011806  /* Recovered data using ECC and offsets */
#define SCSI_KCQSE_RREWEDATAREWRITTEN            0x011807  /* Recovered Read error - with ECC, data rewritten */
#define SCSI_KCQSE_DLNOTFOUND                    0x011c00  /* Defect List not found */
#define SCSI_KCQSE_PRIMARYDLNOTFOUND             0x011c01  /* Primary defect list not found */
#define SCSI_KCQSE_GROWNDLNOTFOUND               0x011c02  /* Grown defect list not found */
#define SCSI_KCQSE_PARTIALDLTRANSFERRED          0x011f00  /* Partial defect list transferred */
#define SCSI_KCQSE_INTERNALTARGETFAILURE         0x014400  /* Internal target failure */
#define SCSI_KCQSE_PFATHRESHOLDREACHED           0x015d00  /* PFA threshold reached */
#define SCSI_KCQSE_PFATESTWARNING                0x015dff  /* PFA test warning */
#define SCSI_KCQSE_INTERNALLOGICFAILURE          0x018100  /* Internal logic failure */

/* Not Ready / Diagnostic Failure KCQ values */

#define SCSI_KCQNR_CAUSENOTREPORTABLE            0x020400  /* Not Ready - Cause not reportable. */
#define SCSI_KCQNR_BECOMINGREADY                 0x020401  /* Not Ready - becoming ready */
#define SCSI_KCQNR_NEEDINITIALIZECOMMAND         0x020402  /* Not Ready - need initialize command (start unit) */
#define SCSI_KCQNR_MANUALINTERVENTIONREQUIRED    0x020403  /* Not Ready - manual intervention required */
#define SCSI_KCQNR_FORMATINPROGRESS              0x020404  /* Not Ready - format in progress */
#define SCSI_KCQNR_SELFTESTINPROGRESS            0x020409  /* Not Ready - self-test in progress */
#define SCSI_KCQNR_MEDIUMFORMATCORRUPTED         0x023100  /* Not Ready - medium format corrupted */
#define SCSI_KCQNR_FORMATCOMMANDFAILED           0x023101  /* Not Ready - format command failed */
#define SCSI_KCQNR_ESUNAVAILABLE                 0x023502  /* Not Ready - enclosure services unavailable */
#define SCSI_KCQNR_MEDIANOTPRESENT               0x023a00  /* Not Ready - media not present */
#define SCSI_KCQDF_BRINGUPFAILORDEGRADEDMODE     0x024080  /* Diagnostic Failure - bring-up fail or degraded mode */
#define SCSI_KCQDF_HARDDISKCONTROLLER            0x024081  /* Diagnostic Failure - Hard Disk Controller */
#define SCSI_KCQDF_RAMMICROCODENOTLOADED         0x024085  /* Diagnostic Failure - RAM microcode not loaded */
#define SCSI_KCQDF_RROCALIBRATION                0x024090  /* Diagnostic Failure - RRO Calibration */
#define SCSI_KCQDF_CHANNELCALIBRATION            0x024091  /* Diagnostic Failure - Channel Calibration */
#define SCSI_KCQDF_HEADLOAD                      0x024092  /* Diagnostic Failure - Head Load */
#define SCSI_KCQDF_WRITEAE                       0x024093  /* Diagnostic Failure - Write AE */
#define SCSI_KCQDF_12VOVERCURRENT                0x024094  /* Diagnostic Failure - 12V over current */
#define SCSI_KCQDF_OTHERSPINDLEFAILURE           0x024095  /* Diagnostic Failure - Other spindle failure */
#define SCSI_KCQDF_SELFRESET                     0x0240b0  /* Diagnostic Failure - self-reset */
#define SCSI_KCQDF_CONFIGNOTLOADED               0x024c00  /* Diagnostic Failure - config not loaded */

/* Medium error KCQ values */

#define SCSI_KCQME_WRITEFAULT                    0x030300  /* Medium Error - write fault */
#define SCSI_KCQME_WRITEFAULTAUTOREALLOCFAILED   0x030c02  /* Medium Error - write error - auto-realloc failed */
#define SCSI_KCQME_WRITERTLIMITEXCEEDED          0x030cbb  /* Medium Error - write recovery time limit exceeded */
#define SCSI_KCQME_IDCRCERROR                    0x031000  /* Medium Error - ID CRC error */
#define SCSI_KCQME_UNRRE1                        0x031100  /* Medium Error - unrecovered read error */
#define SCSI_KCQME_READRETRIESEXHAUSTED          0x031101  /* Medium Error - read retries exhausted */
#define SCSI_KCQME_ERRORTOOLONGTOCORRECT         0x031102  /* Medium Error - error too long to correct */
#define SCSI_KCQME_UREAUTOREALLOCFAILED          0x031104  /* Medium Error - unrecovered read error - auto re-alloc failed */
#define SCSI_KCQME_URERECOMMENDREASSIGN          0x03110b  /* Medium Error - unrecovered read error - recommend reassign */
#define SCSI_KCQME_READRTLIMITEXCEEDED           0x0311ff  /* Medium Error - read recovery time limit exceeded */
#define SCSI_KCQME_RECORDNOTFOUND                0x031401  /* Medium Error - record not found */
#define SCSI_KCQME_DSME                          0x031600  /* Medium Error - Data Sync Mark error */
#define SCSI_KCQME_DSERECOMMENDREASSIGN          0x031604  /* Medium Error - Data Sync Error - recommend reassign */
#define SCSI_KCQME_DLE                           0x031900  /* Medium Error - defect list error */
#define SCSI_KCQME_DLNOTAVAILABLE                0x031901  /* Medium Error - defect list not available */
#define SCSI_KCQME_DLEINPRIMARYLIST              0x031902  /* Medium Error - defect list error in primary list */
#define SCSI_KCQME_DLEINGROWNLIST                0x031903  /* Medium Error - defect list error in grown list */
#define SCSI_KCQME_FEWERTHAN50PCTDLCOPIES        0x03190e  /* Medium Error - fewer than 50% defect list copies */
#define SCSI_KCQME_MEDIUMFORMATCORRUPTED         0x033100  /* Medium Error - medium format corrupted */
#define SCSI_KCQME_FORMATCOMMANDFAILED           0x033101  /* Medium Error - format command failed */
#define SCSI_KCQME_DATAAUTOREALLOCATED           0x038000  /* Medium Error - data auto-reallocated */

/* Hardware Error KCQ values */

#define SCSI_KCQHE_NOINDEXORSECTOR               0x040100  /* Hardware Error - no index or sector */
#define SCSI_KCQHE_NOSEEKCOMPLETE                0x040200  /* Hardware Error - no seek complete */
#define SCSI_KCQHE_WRITEFAULT                    0x040300  /* Hardware Error - write fault */
#define SCSI_KCQHE_COMMUNICATIONFAILURE          0x040800  /* Hardware Error - communication failure */
#define SCSI_KCQHE_TRACKFOLLOWINGERROR           0x040900  /* Hardware Error - track following error */
#define SCSI_KCQHE_UREINRESERVEDAREA             0x041100  /* Hardware Error - unrecovered read error in reserved area */
#define SCSI_KCQHE_DSMEINRESERVEDAREA            0x041600  /* Hardware Error - Data Sync Mark error in reserved area */
#define SCSI_KCQHE_DLE                           0x041900  /* Hardware Error - defect list error */
#define SCSI_KCQHE_DLEINPRIMARYLIST              0x041902  /* Hardware Error - defect list error in Primary List */
#define SCSI_KCQHE_DLEINGROWNLIST                0x041903  /* Hardware Error - defect list error in Grown List */
#define SCSI_KCQHE_REASSIGNFAILED                0x043100  /* Hardware Error - reassign failed */
#define SCSI_KCQHE_NODEFECTSPAREAVAILABLE        0x043200  /* Hardware Error - no defect spare available */
#define SCSI_KCQHE_UNSUPPORTEDENCLOSUREFUNCTION  0x043501  /* Hardware Error - unsupported enclosure function */
#define SCSI_KCQHE_ESUNAVAILABLE                 0x043502  /* Hardware Error - enclosure services unavailable */
#define SCSI_KCQHE_ESTRANSFERFAILURE             0x043503  /* Hardware Error - enclosure services transfer failure */
#define SCSI_KCQHE_ESREFUSED                     0x043504  /* Hardware Error - enclosure services refused */
#define SCSI_KCQHE_SELFTESTFAILED                0x043e03  /* Hardware Error - self-test failed */
#define SCSI_KCQHE_UNABLETOUPDATESELFTEST        0x043e04  /* Hardware Error - unable to update self-test */
#define SCSI_KCQHE_DMDIAGNOSTICFAIL              0x044080  /* Hardware Error - Degrade Mode. Diagnostic Fail */
#define SCSI_KCQHE_DMHWERROR                     0x044081  /* Hardware Error - Degrade Mode. H/W Error */
#define SCSI_KCQHE_DMRAMMICROCODENOTLOADED       0x044085  /* Hardware Error - Degrade Mode. RAM microcode not loaded */
#define SCSI_KCQHE_SEEKTESTFAILURE               0x044090  /* Hardware Error - seek test failure */
#define SCSI_KCQHE_READWRITETESTFAILURE          0x0440a0  /* Hardware Error - read/write test failure */
#define SCSI_KCQHE_DEVICESELFRESET               0x0440b0  /* Hardware Error - device self-reset */
#define SCSI_KCQHE_COMPONENTMISMATCH             0x0440d0  /* Hardware Error - component mismatch */
#define SCSI_KCQHE_INTERNALTARGETFAILURE         0x044400  /* Hardware Error - internal target failure */
#define SCSI_KCQHE_INTERNALLOGICERROR            0x048100  /* Hardware Error - internal logic error */
#define SCSI_KCQHE_COMMANDTIMEOUT                0x048200  /* Hardware Error - command timeout */

/* Illegal Request KCQ values */

#define SCSI_KCQIR_PARMLISTLENGTHERROR           0x051a00  /* Illegal Request - parm list length error */
#define SCSI_KCQIR_INVALIDCOMMAND                0x052000 /* Illegal Request - invalid/unsupported command code */
#define SCSI_KCQIR_LBAOUTOFRANGE                 0x052100  /* Illegal Request - LBA out of range */
#define SCSI_KCQIR_INVALIDFIELDINCBA             0x052400  /* Illegal Request - invalid field in CDB (Command Descriptor Block) */
#define SCSI_KCQIR_INVALIDLUN                    0x052500  /* Illegal Request - invalid LUN */
#define SCSI_KCQIR_INVALIDFIELDSINPARMLIST       0x052600  /* Illegal Request - invalid fields in parm list */
#define SCSI_KCQIR_PARAMETERNOTSUPPORTED         0x052601  /* Illegal Request - parameter not supported */
#define SCSI_KCQIR_INVALIDPARMVALUE              0x052602  /* Illegal Request - invalid parm value */
#define SCSI_KCQIR_IFPTHRESHOLDPARAMETER         0x052603  /* Illegal Request - invalid field parameter - threshold parameter */
#define SCSI_KCQIR_INVALIDRELEASEOFPR            0x052604  /* Illegal Request - invalid release of persistent reservation */
#define SCSI_KCQIR_IFPTMSFIRMWARETAG             0x052697  /* Illegal Request - invalid field parameter - TMS firmware tag */
#define SCSI_KCQIR_IFPCHECKSUM                   0x052698  /* Illegal Request - invalid field parameter - check sum */
#define SCSI_KCQIR_IFPFIRMWARETAG                0x052699  /* Illegal Request - invalid field parameter - firmware tag */
#define SCSI_KCQIR_COMMANDSEQUENCEERROR          0x052c00  /* Illegal Request - command sequence error */
#define SCSI_KCQIR_UNSUPPORTEDENCLOSUREFUNCTION  0x053501  /* Illegal Request - unsupported enclosure function */
#define SCSI_KCQIR_SAVINGPARMSNOTSUPPORTED       0x053900  /* Illegal Request - Saving parameters not supported */
#define SCSI_KCQIR_INVALIDMESSAGE                0x054900  /* Illegal Request - invalid message */
#define SCSI_KCQIR_MEDIALOADOREJECTFAILED        0x055300  /* Illegal Request - media load or eject failed */
#define SCSI_KCQIR_UNLOADTAPEFAILURE             0x055301  /* Illegal Request - unload tape failure */
#define SCSI_KCQIR_MEDIUMREMOVALPREVENTED        0x055302  /* Illegal Request - medium removal prevented */
#define SCSI_KCQIR_SYSTEMRESOURCEFAILURE         0x055500  /* Illegal Request - system resource failure */
#define SCSI_KCQIR_SYSTEMBUFFERFULL              0x055501  /* Illegal Request - system buffer full */
#define SCSI_KCQIR_INSUFFICIENTRR                0x055504  /* Illegal Request - Insufficient Registration Resources */

/* Unit Attention KCQ values */

#define SCSI_KCQUA_NOTREADYTOTRANSITION          0x062800  /* Unit Attention - not-ready to ready transition (format complete) */
#define SCSI_KCQUA_DEVICERESETOCCURRED           0x062900  /* Unit Attention - POR or device reset occurred */
#define SCSI_KCQUA_POROCCURRED                   0x062901  /* Unit Attention - POR occurred */
#define SCSI_KCQUA_SCSIBUSRESETOCCURRED          0x062902  /* Unit Attention - SCSI bus reset occurred */
#define SCSI_KCQUA_TARGETRESETOCCURRED           0x062903  /* Unit Attention - TARGET RESET occurred */
#define SCSI_KCQUA_SELFINITIATEDRESETOCCURRED    0x062904  /* Unit Attention - self-initiated-reset occurred */
#define SCSI_KCQUA_TRANSCEIVERMODECHANGETOSE     0x062905  /* Unit Attention - transceiver mode change to SE */
#define SCSI_KCQUA_TRANSCEIVERMODECHANGETOLVD    0x062906  /* Unit Attention - transceiver mode change to LVD */
#define SCSI_KCQUA_PARAMETERSCHANGED             0x062a00  /* Unit Attention - parameters changed */
#define SCSI_KCQUA_MODEPARAMETERSCHANGED         0x062a01  /* Unit Attention - mode parameters changed */
#define SCSI_KCQUA_LOGSELECTPARMSCHANGED         0x062a02  /* Unit Attention - log select parms changed */
#define SCSI_KCQUA_RESERVATIONSPREEMPTED         0x062a03  /* Unit Attention - Reservations pre-empted */
#define SCSI_KCQUA_RESERVATIONSRELEASED          0x062a04  /* Unit Attention - Reservations released */
#define SCSI_KCQUA_REGISTRATIONSPREEMPTED        0x062a05  /* Unit Attention - Registrations pre-empted */
#define SCSI_KCQUA_COMMANDSCLEARED               0x062f00  /* Unit Attention - commands cleared by another initiator */
#define SCSI_KCQUA_OPERATINGCONDITIONSCHANGED    0x063f00  /* Unit Attention - target operating conditions have changed */
#define SCSI_KCQUA_MICROCODECHANGED              0x063f01  /* Unit Attention - microcode changed */
#define SCSI_KCQUA_CHANGEDOPERATINGDEFINITION    0x063f02  /* Unit Attention - changed operating definition */
#define SCSI_KCQUA_INQUIRYPARAMETERSCHANGED      0x063f03  /* Unit Attention - inquiry parameters changed */
#define SCSI_KCQUA_DEVICEIDENTIFIERCHANGED       0x063f05  /* Unit Attention - device identifier changed */
#define SCSI_KCQUA_INVALIDAPMPARAMETERS          0x063f90  /* Unit Attention - invalid APM parameters */
#define SCSI_KCQUA_WORLDWIDENAMEMISMATCH         0x063f91  /* Unit Attention - world-wide name mismatch */
#define SCSI_KCQUA_PFATHRESHOLDREACHED           0x065d00  /* Unit Attention - PFA threshold reached */
#define SCSI_KCQUA_PFATHRESHOLDEXCEEDED          0x065dff  /* Unit Attention - PFA threshold exceeded */

/* Write Protect KCQ values */

#define SCSI_KCQWP_COMMANDNOTALLOWED             0x072700 /* Write Protect - command not allowed */

/* Aborted Command KCQ values */

#define SCSI_KCQAC_NOADDITIONALSENSECODE         0x0b0000  /* Aborted Command - no additional sense code */
#define SCSI_KCQAC_SYNCDATATRANSFERERROR         0x0b1b00  /* Aborted Command - sync data transfer error (extra ACK) */
#define SCSI_KCQAC_UNSUPPORTEDLUN                0x0b2500  /* Aborted Command - unsupported LUN */
#define SCSI_KCQAC_ECHOBUFFEROVERWRITTEN         0x0b3f0f  /* Aborted Command - echo buffer overwritten */
#define SCSI_KCQAC_MESSAGEREJECTERROR            0x0b4300  /* Aborted Command - message reject error */
#define SCSI_KCQAC_INTERNALTARGETFAILURE         0x0b4400  /* Aborted Command - internal target failure */
#define SCSI_KCQAC_SELECTIONFAILURE              0x0b4500  /* Aborted Command - Selection/Reselection failure */
#define SCSI_KCQAC_SCSIPARITYERROR               0x0b4700  /* Aborted Command - SCSI parity error */
#define SCSI_KCQAC_INITIATORDETECTEDERRORECEIVED 0x0b4800  /* Aborted Command - initiator-detected error message received */
#define SCSI_KCQAC_ILLEGALMESSAGE                0x0b4900  /* Aborted Command - inappropriate/illegal message */
#define SCSI_KCQAC_DATAPHASEERROR                0x0b4b00  /* Aborted Command - data phase error */
#define SCSI_KCQAC_OVERLAPPEDCOMMANDSATTEMPTED   0x0b4e00  /* Aborted Command - overlapped commands attempted */
#define SCSI_KCQAC_LOOPINITIALIZATION            0x0b4f00  /* Aborted Command - due to loop initialization */

/* Other KCQ values: */

#defome SCSO_KCQOTHER_MISCOMPARE                 0x0e1d00  /* Miscompare - during verify byte check operation */

/* SSCSI Status Codes *******************************************************/

#define SCSCI_STATUS_OK                          0x00  /* OK */
#define SCSCI_STATUS_CHECKCONDITION              0x02  /* Check condition */
#define SCSCI_STATUS_CONDITIONMET                0x04  /* Condition met */
#define SCSCI_STATUS_BUSY                        0x08  /* Busy */
#define SCSCI_STATUS_INTERMEDIATE                0x10  /* Intermediate */
#define SCSCI_STATUS_DATAOVERUNDERRUN            0x12  /* Data Under/Over Run? */
#define SCSCI_STATUS_INTERMEDIATECONDITIONMET    0x14  /* Intermediate - Condition met */
#define SCSCI_STATUS_RESERVATIONCONFLICT         0x18  /* Reservation conflict */
#define SCSCI_STATUS_COMMANDTERMINATED           0x22  /* Command terminated */
#define SCSCI_STATUS_QUEUEFULL                   0x28  /* Queue (task set) full */
#define SCSCI_STATUS_ACAACTIVE                   0x30  /* ACA active */
#define SCSCI_STATUS_TASKABORTED                 0x40  /* Task aborted */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __DRIVERS_USBDV_USBDEV_SCSI_H */
