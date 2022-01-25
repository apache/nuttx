/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_version.h
 *
 * @brief   gh3x2x driver version and change log
 *
 * @par  history:
 * @verbatim
 * Date          Author       Version     Notes
 * 2019-10-21    Chris-Chan   v0.0.1.0    1.First version
 * 2019-11-04    Chris-Chan   v0.0.1.1    1.Change name to GH31XX
 * 2019-12-17    Chris-Chan   v0.0.1.2    1.Fix some bugs
 * 2019-12-25    Chris-Chan   v0.0.1.3    1.Add hook function
 * 2019-12-31    Chris-Chan   v0.0.1.4    1.Add hb algorithm support, but use roma ii algorithm now
 * 2020-01-19    Chris-Chan   v0.0.1.5    1.Add inited confirm support
 *                                        2.Add some control api
 * 2020-02-12    Chris-Chan   v0.0.1.6    1.Add debug log support
 *                                        2.Add some control api and some easy to debug
 * 2020-02-17    Chris-Chan   v0.0.1.7    1.Add some control api
 * 2020-02-28    Chris-Chan   v0.0.1.8    1.Add some control api
 * 2020-03-06    Chris-Chan   v0.0.1.9    1.Add universal protocol
 * 2020-03-20    Chris-Chan   v0.0.2.0    1.Add more cmd handler
 *                                        2.Add all header files & move some code to *.h from common.h
 *                                        3.Change to new algorithm api
 * 2020-03-27    Chris-Chan   v0.0.2.1    1.Fix cryp header that distinguish data is encrypted or not
 *                                        2.Add some control api
 *                                        3.Add virtual regs control
 * 2020-04-03    Chris-Chan   v0.1.0.0    1.Add send pkg api with universal protocol
 *                                        2.Add Soft Adjust api
 * 2020-04-07    Chris-Chan   v0.1.0.1    1.Fix algorithm calc hook call, need prevent buffer reverse modification!
 *                                        2.Fix unpack rawdata api, when using 1slot1adc unpack bug
 * 2020-04-08    Chris-Chan   v0.1.0.2    1.Change name to GH3X2X
 * 2020-04-17    Chris-Chan   v0.1.1.0    1.Add incomplete rawdata support
 *                                        2.Change spo2 channel map handle
 * 2020-04-24    Chris-Chan   v0.1.2.0    1.Add dump rawdata support
 *                                        2.Add wait pulse mode off code when exit low power mode, but disable now
 *                                        3.Add verify for encrypt input, input buffer must has magic bytes
 *                                        4.Fix protocol cmd handle and add some cmd
 * 2020-04-30    Chris-Chan   v0.1.3.0    1.Add lead handle support
 * 2020-05-09    Chris-Chan   v0.1.4.0    1.Add macro ctrl channel cnt
 * 2020-05-09    Chris-Chan   v0.1.4.1    1.Fix some bugs
 * 2020-05-12    Chris-Chan   v0.1.4.2    1.Fix some code style
 * 2020-05-13    Chris-Chan   v0.1.4.3    1.Fix slot unpack bug
 *                                        2.Fix wear detect switch
 * 2020-05-18    Chris-Chan   v0.1.4.4    1.Fix all function deinit by add api
 * 2020-05-26    Chris-Chan   v0.1.4.5    1.Change don't send cmd @ start sampling
 * 2020-07-21    wangtao      v0.1.4.6    1.Add AGC code
 *                                        2.Fix some bugs
 * 2020-08-05    wangtao      v0.1.5.0    1.add zip code
 *                                        2.add code for single driver
 * 2020-08-18    wangtao      v0.1.6.0    1.add communicate test code
 *                                        2.change some macro default value
 * 2020-08-20    wangtao      v0.1.6.1    1.fix bug of GH3X2X_Spo2AlgorithmStop,which lost call stop sampling
 * 2020-08-20    wangtao      v0.1.6.2    1.fix bug when confirm connect status
 * 2020-08-20    wanpeng      v0.1.6.3    1.add Itype 0nA code
 * 2020-08-20    zhengxiaoyu  v0.1.6.4    1.add app interface of zip function
 * 2020-08-30    wangtao      v0.1.6.5    1.change method of upload flag data
 *                                        2.fix bug of zip interface when pack g sensor data
 * 2020-08-30    wangtao      v0.1.6.6    1.fix bug of tag value of flag data
 * 2020-08-30    wangtao      v0.1.6.7    1.change  method of upload algorithm result,need upload every algorithm result
 *                                        2.change format of algorithm result and related interfaces
 ************************************************************************************
 * 2020-09-10    wangtao      v1.0.0.0    1.update new algorithm lib of ECG HR and SPO2
 * 2020-09-10    wangtao      v1.0.0.1    1.open spo2 and ecg
 * 2020-09-19    zhengxiaoyu  v1.0.0.2    1.change channel map virtual reg addr
 * 2020-09-21    wangtao      v1.0.1.0    1.add soft wear on/off function & alg
 * 2020-09-21    wangtao      v1.0.2.0    1.add movement detection by gsensor
 * 2020-09-21    wangtao      v1.1.0.0    1.config algorithm memory by user,delete code about algorithm memory
 * 2020-09-27    wangtao      v1.1.1.0    1.fix bug of soft ADT,which will overstep the boundary of array
 * 2020-09-29    wangtao      v1.1.2.0    1.fix bug of memory init
 * 2020-09-29    wangtao      v1.1.3.0    1.update new algorithm
 * 2020-09-29    zhengxiaoyu  v1.1.3.1    1.update function frequency reg virtual receive and analysis
 * 2020-09-29    wanpeng      v1.1.3.2    1.fix softwearoff bug that g_nPpgIdx bigger than 99
 * 2020-09-29    wanpeng      v1.1.3.3    1.change logic soft adt
 * 2020-10-13    wangtao      v1.1.4.0    1.fix bug of soft adt
 * 2020-10-14    wangtao      v1.1.4.1    1.fix some code style
 *                                        2.change some threshold of soft adt
 * 2020-10-15    wangtao      v1.1.4.2    1.cancel restrict of adt enable
 *                                        2.fix hb algorithm bug
 * 2020-10-15    wangtao      v1.1.5.0    1.fix bug of soft adt
 * 2020-10-22    wangtao      v1.1.6.0    1.add HRV algorithm
 * 2020-10-26    zhengxiaoyu  v1.1.6.1    1.fix protocol bug
 * 2020-10-22    wangtao      v1.1.7.0    1.add new algo lib
 *                                        2.fix bug of acc data dislocation when channel map cnt is larger than 8
 * 2020-11-03    zhengxiaoyu  v1.1.7.1    1.Add spo2 Correction Factor input
 * 2020-11-03    zhengxiaoyu  v1.1.7.2    1.Fix HRV error result bug
 * 2020-11-05    zengjiemin   v1.1.8.0    1.Add Android Compile
 * 2020-11-05    wanpeng      v1.1.8.1    1.In ECG 5nA mode, reset ecg module when stop ecg sample
 * 2020-11-11    zhengxiaoyu  v1.1.9.0    1.Add Algo Channel map
 * 2020-11-11    zhengxiaoyu  v1.1.9.1    1.Add Algo Version print
 * 2020-11-17    zhengxiaoyu  v1.1.9.2    1.Fix Ecg 0nA mode lead bug
 * 2020-11-17    zhengxiaoyu  v1.3.0.0    1.Fix Ecg 0nA mode lead bug
 * 2020-11-17    zhengxiaoyu  v1.3.0.1    1.Fix Gsnesor Bug
 * 2020-11-17    qiuchuitong  v1.3.0.2    1.Fix Ecg Rowdata to algorithm bug
 * 2020-12-16    wanpeng      v1.3.0.2    save code and disable ecg and dump mode /change spo2 hr result struct
 * 2020-12-22    wanpeng      v1.3.0.3    add engineering mode sample interface
 * 2020-12-24    wanpeng      v1.3.0.4    add chip reset recovery code
 ************************************************************************************
 * 2021-01-07    wanpeng      v3.0.0.0    modify compile script and start new drvlib
 * 2021-01-08    wanpeng      v3.0.0.1    add frame process code and start function code
 * 2021-01-11    wanpeng      v3.0.0.2    add some code in ecg.c/config.c
 * 2021-01-12    wanpeng      v3.0.0.8    1.add some code about soft agc
 *                                        2.remove code in control_ex.c
 *                                        3.add some code in config.c
 *                                        4.add 2nd level algo api
 *                                        5.use frameinfo pointer array instead of pointers
 *                                        6.add algo init and deinit api
 * 2021-01-13    wanpeng      v3.0.1.1    1.free alo protect arr in alo call api
 *                                        2.set last gain
 *                                        3.add rawdata buffer feature value calculate and check
 * 2021-01-14    wanpeng      v3.0.1.4    1.send flag msg to alg
 *                                        2.send flag msg to alg从
 *                                        3.fix algo call and agc lib
 * 2021-01-15    wanpeng      v3.0.1.5    add upload function
 * 2021-01-19    qiuchuitong  v3.0.1.8    1.modify algo call lib
 *                                        2.fix ecg bug
 *                                        3.fix ecg func return bug
 * 2021-01-20    qiuchuitong  v3.0.1.9    upload algo result
 * 2021-01-21    qiuchuitong  v3.0.2.2    1.fix data pkg bug
 *                                        2.fix hrv channel map config bug
 *                                        3.fix soft adt bug
 * 2021-01-22    wanpeng      v3.0.2.3    separate fucntion GetFrameNum
 * 2021-01-29    qiuchuitong  v3.0.2.6    1.fix ECG bug
 *                                        2.add app work mode
 *                                        3.modify algo result hook func
 * 2021-01-29    zhengxiaoyu  v3.0.2.8    1.Update Zip Protocol
 *                                        2.Fix spo2 Alg bug
 * 2021-02-02    zhengxiaoyu  v3.0.3.1    1.Update zip Protocol
 *                                        2.Fix ECG Flag2 bugs
 *                                        3.Fix Spo2 Rawdata bug
 * 2021-02-02    zhengxiaoyu  v3.0.3.3    1.Update code format
 *                                        2.Update softwear wear on detect
 * 2021-02-08    zhengxiaoyu  v3.0.3.5    1.Update Zip protocol
 *                                        2.Change ResultHook to ResultReport 
 * 2021-02-09    zhengxiaoyu  v3.0.3.6    1.Fix Zip Protocol bug
 * 2021-02-09    zhengxiaoyu  v3.0.3.8    1.Fix Engineering Mode mode
 *                                        2.Fix Decode reg bug & add HRV function Decode
 * 2021-02-25    zhengxiaoyu  v3.0.4.2    1.Fix Build bug when close goodix alg
 *                                        2.Add 0x0b new rawdata protocol
 *                                        3.Add protocol version
 *                                        4.Add Amb section in new rawdata protocol
 * 2021-03-09    qiuchuitong  v3.0.5.0    1.Update soft adt algo lib
 *                                        2.add lead off detect 2 for 800Hz sample rate
 * 2021-03-09    zhengxiaoyu  v3.0.6.0    1.Add AF/HSM/RESP/BT lib
 *                                        2.Update code struct
 * 2021-03-09    zhengxiaoyu  v3.0.6.2    1.Update HSM/AF lib
 *                                        2.Update HSM/AF API
 * 2021-03-10    zhengxiaoyu  v3.0.6.5    1.Update HSM/AF function lib
 *                                        2.Update HSM result protocol
 *                                        3.Update code structure
 * 2021-03-11    qiuchuitong  v3.0.6.6    1.Update HR/SPO2/ECG/ ALGO lib
 *                                        2.add virtual reg for spo2 and soft adt
 * 2021-03-12    zhengxiaoyu  v3.0.6.7    1.Update HSM Report Send
 * 2021-03-12    qiuchuitong  v3.0.6.8    1.Add bootloader version function 
 * 2021-03-12    qiuchuitong  v3.0.6.9    1.fix spo2 corret factor bug
 * 2021-03-17    qiuchuitong  v3.0.7.1    1.Update HSM ALGO lib
 *                                        2.fix warnings 
 * 2021-03-19    qiuchuitong  v3.0.7.3    1.Update HR ALGO lib
 *                                        2.modify spo2 default factor 
 * 2021-03-19    qiuchuitong  v3.0.7.4    1.Update af ALGO lib
 * 2021-03-22    qiuchuitong  v3.0.7.5    1.Update af,HSM,RESP ALGO lib
 * 2021-03-22    zhengxiaoyu  v3.0.7.9    1.Update HSM lib
 *                                        2.Update float cal
 *                                        3.Add FPBP/PWA functions 
 * 2021-03-23    qiuchuitong  v3.0.8.0    1.fix spo2 upload results bug
 * 2021-03-23    qiuchuitong  v3.0.8.1    1.fix ECG  results bug
 * 2021-03-24    wanpeng      v3.0.8.2    1.fix movement detect bug
                                          2.add multi chnl memsize maroc(HR and SPO2)
 * 2021-03-30    zhengxiaoyu  v3.0.8.6    1.Change function id to u32
 *                                        2.Add init/deinit/update error check
 *                                        3.Add test1/2 functions
 *                                        4.Add virtual-reg setting
 * 2021-03-30    zhengxiaoyu  v3.0.8.8    1.Fix RESP gain flag bug
 *                                        2.Add write reg check
 * 2021-04-01    zhengxiaoyu  v3.0.9.1    1.Fix PWA chnl map bug
 *                                        2.Add Hr result get
 *                                        3.Fix gain-get bug
 *                                        4.Add Android define
 * 2021-04-01    zhengxiaoyu  v3.0.9.2    1.Fix PT mode ADT bug
 * 2021-04-06    zhengxiaoyu  v3.0.9.3    1.Update hr result input hrv/hsm
 * 2021-04-16    zhengxiaoyu  v3.0.9.6    1.Fix control bug
 * 2021-04-17    zhengxiaoyu  v3.0.9.8    1.Update softadt alg
 *                                        2.Fix bug & Update struct
 * 2021-04-19    zhengxiaoyu  v3.0.9.9    1.Fix bug
 * 2021-04-20    wanpeng      v3.1.0.0    1.Fix agc bug
 * 2021-05-11    zhengxiaoyu  v3.2.0.0    1.Update FPBP Alg & VirtualReg & something else
 * 2021-05-11    zhengxiaoyu  v3.2.0.1    1.Update Algo Version Update
 * 2021-05-15    zhengxiaoyu  v3.2.0.2    1.Add FPBP Algo chnl init
 * 2021-05-17    zhengxiaoyu  v3.2.0.3    1.Fix FPBP bug
 * 2021-05-17    zhengxiaoyu  v3.2.0.4    1.Update FPBP Input&Output API
 * 2021-05-18    zhengxiaoyu  v3.2.0.5    1.Fix some FPBP bug
 * 2021-05-19    zhengxiaoyu  v3.2.0.6    1.Add FPBP error code update
 * 2021-05-20    zhengxiaoyu  v3.2.0.7    1.Add FPBP result pressure data
 * 2021-05-20    zhengxiaoyu  v3.2.0.8    1.Add update FPBP pressure parameters setting
 * 2021-05-24    zhengxiaoyu  v3.2.1.0    1.Add eeprom handle
 *                                                           2.Fix zip protocol bug
 * 2021-05-25    zhengxiaoyu  v3.2.1.1    1.Change rawdata a0/a1 read
 * 2021-05-26    zhengxiaoyu  v3.2.1.2    1.Update FPBP hook logic
 * 2021-05-27    zhengxiaoyu  v3.2.1.4    1.Update FPBP pressure parameters setting
 *                                                           2.Add PWA program & Alglib
 * 2021-05-28    zhengxiaoyu  v3.2.1.5    1.Fix decode cfg list bug
 * 2021-05-31    zhengxiaoyu  v3.2.1.6    1.Add PWA algo function
 * 2021-06-01    zhengxiaoyu  v3.2.1.7    1.Add SoftAdt IR function
 * 2021-06-01    zhengxiaoyu  v3.2.1.8    1.Add function freauency control api
 * 2021-06-02    zhengxiaoyu  v3.2.1.9    1.Fix virtual alg chnl map bug
 * 2021-06-03    zhengxiaoyu  v3.2.2.0    1.Update PWA new program
 * 2021-06-04    zhengxiaoyu  v3.2.2.1    1.Update some algo weak api
 * 2021-06-08    zhengxiaoyu  v3.2.2.2    1.Add function led current
 * 2021-06-08    wanpeng      v3.2.2.4    1.add hardware spi
 *                                        2.add sample rate modify function
 * 2021-06-19    qiuchuitong  v3.2.2.5    1.add ecg 800Hz to 250Hz resample code
 *                                        2.add Gs gyro code
 *                                        3.disable rx2 and rx3 for 3026
 * 2021-06-21    zhengxiaoyu  v3.2.2.6    1.Fix fifo send progress bug
 * 2021-06-22    zhengxiaoyu  v3.2.2.7    1.Fix zip protocol & fifo send bug
 * 2021-06-22    zhengxiaoyu  v3.2.2.8    1.Update hrv input struct
 * 2021-06-24    zhengxiaoyu  v3.2.2.9    1.Add function infomation send protocol
 * 2021-06-25    zhengxiaoyu  v3.2.3.0    1.Add config protect handle 
 * 2021-06-29    qiuchuitong  v3.2.3.1    1.divide ecg lead off det and resample
 * 2021-06-29    qiuchuitong  v3.2.3.2    1.fix check chip Model bug
 * 2021-06-29    qiuchuitong  v3.2.3.3    1.fix ecg resample bug
 * 2021-06-29    zhengxiaoyu  v3.2.3.4    1.fix spo2 algo channel bug
 * 2021-07-16    zhengxiaoyu  v3.2.3.5    1.update zip protocol
 * 2021-07-21    zhengxiaoyu  v3.2.3.7    1.update hr&spo2 api&struct
 * 2021-07-21    zhengxiaoyu  v3.2.3.8    1.update hr&spo2 memory
 * 2021-07-21    qiuchuitong  v3.2.3.9    1. add bt version code
 * 2021-07-21    qiuchuitong  v3.2.4.0    1. update nadt
 * 2021-08-12    zhengxiaoyu  v3.2.4.1    1.update bp and other
 * @endverbatim  */

#ifndef _GH3X2X_DRV_VERSION_H_
#define _GH3X2X_DRV_VERSION_H_


#define   GH3X2X_DRV_MAJOR_VERSION_NUMBER        3      /**< major version number */
#define   GH3X2X_DRV_MINOR_VERSION_NUMBER        2      /**< minor version number */
#define   GH3X2X_DRV_REVISION_VERSION_NUMBER     4      /**< revision version number */
#define   GH3X2X_DRV_FIXED_VERSION_NUMBER        5      /**< fixed version number */
#define   GH3X2X_DRV_EXTREN_VERSION_NUMBER       8      /**< fixed version number */

#define   GH3X2X_TO_STRING(x)       #x                      /**< number to char */
#define   GH3X2X_STR(x)             GH3X2X_TO_STRING(x)     /**< number to char */

/// makeup version string
#define   GH3X2X_VERSION_STRING     "v"GH3X2X_STR(GH3X2X_DRV_MAJOR_VERSION_NUMBER)\
                                    "."GH3X2X_STR(GH3X2X_DRV_MINOR_VERSION_NUMBER)\
                                    "."GH3X2X_STR(GH3X2X_DRV_REVISION_VERSION_NUMBER)\
                                    "."GH3X2X_STR(GH3X2X_DRV_FIXED_VERSION_NUMBER)\
                                    "."GH3X2X_STR(GH3X2X_DRV_EXTREN_VERSION_NUMBER)\
                                    "_L61A(build:"__DATE__"_"__TIME__")"


#endif /* _GH3X2X_DRV_VERSION_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/
