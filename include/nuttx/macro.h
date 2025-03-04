/****************************************************************************
 * include/nuttx/macro.h
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

#ifndef __INCLUDE_NUTTX_MACRO_H
#define __INCLUDE_NUTTX_MACRO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GET_ARG_VALUE(_00, _01, _02, _03, _04, _05, _06, _07, \
                      _08, _09, _10, _11, _12, _13, _14, _15, \
                      _16, _17, _18, _19, _20, _21, _22, _23, \
                      _24, _25, _26, _27, _28, _29, _30, _31, \
                      _32, name, ...) name

/* Get the number of arguments (up to 32) */

#define GET_ARG_COUNT(...) \
        GET_ARG_VALUE(_0, ##__VA_ARGS__, 32, 31, 30, \
        29, 28, 27, 26, 25, 24, 23, 22, 21, 20, \
        19, 18, 17, 16, 15, 14, 13, 12, 11, 10, \
        9,  8,  7,  6,  5,  4,  3,  2,  1,  0)

/* Reverse the arguments */

#define EXPAND(x) x

#define REVERSE_00()
#define REVERSE_01(a)     a
#define REVERSE_02(a,b)   b,a
#define REVERSE_03(a,...) EXPAND(REVERSE_02(__VA_ARGS__)),a
#define REVERSE_04(a,...) EXPAND(REVERSE_03(__VA_ARGS__)),a
#define REVERSE_05(a,...) EXPAND(REVERSE_04(__VA_ARGS__)),a
#define REVERSE_06(a,...) EXPAND(REVERSE_05(__VA_ARGS__)),a
#define REVERSE_07(a,...) EXPAND(REVERSE_06(__VA_ARGS__)),a
#define REVERSE_08(a,...) EXPAND(REVERSE_07(__VA_ARGS__)),a
#define REVERSE_09(a,...) EXPAND(REVERSE_08(__VA_ARGS__)),a
#define REVERSE_10(a,...) EXPAND(REVERSE_09(__VA_ARGS__)),a
#define REVERSE_11(a,...) EXPAND(REVERSE_10(__VA_ARGS__)),a
#define REVERSE_12(a,...) EXPAND(REVERSE_11(__VA_ARGS__)),a
#define REVERSE_13(a,...) EXPAND(REVERSE_12(__VA_ARGS__)),a
#define REVERSE_14(a,...) EXPAND(REVERSE_13(__VA_ARGS__)),a
#define REVERSE_15(a,...) EXPAND(REVERSE_14(__VA_ARGS__)),a
#define REVERSE_16(a,...) EXPAND(REVERSE_15(__VA_ARGS__)),a
#define REVERSE_17(a,...) EXPAND(REVERSE_16(__VA_ARGS__)),a
#define REVERSE_18(a,...) EXPAND(REVERSE_17(__VA_ARGS__)),a
#define REVERSE_19(a,...) EXPAND(REVERSE_18(__VA_ARGS__)),a
#define REVERSE_20(a,...) EXPAND(REVERSE_19(__VA_ARGS__)),a
#define REVERSE_21(a,...) EXPAND(REVERSE_20(__VA_ARGS__)),a
#define REVERSE_22(a,...) EXPAND(REVERSE_21(__VA_ARGS__)),a
#define REVERSE_23(a,...) EXPAND(REVERSE_22(__VA_ARGS__)),a
#define REVERSE_24(a,...) EXPAND(REVERSE_23(__VA_ARGS__)),a
#define REVERSE_25(a,...) EXPAND(REVERSE_24(__VA_ARGS__)),a
#define REVERSE_26(a,...) EXPAND(REVERSE_25(__VA_ARGS__)),a
#define REVERSE_27(a,...) EXPAND(REVERSE_26(__VA_ARGS__)),a
#define REVERSE_28(a,...) EXPAND(REVERSE_27(__VA_ARGS__)),a
#define REVERSE_29(a,...) EXPAND(REVERSE_28(__VA_ARGS__)),a
#define REVERSE_30(a,...) EXPAND(REVERSE_29(__VA_ARGS__)),a
#define REVERSE_31(a,...) EXPAND(REVERSE_30(__VA_ARGS__)),a
#define REVERSE_32(a,...) EXPAND(REVERSE_31(__VA_ARGS__)),a

#define REVERSE_ARG_(...) \
        GET_ARG_VALUE(0, ##__VA_ARGS__, \
        REVERSE_32, REVERSE_31, REVERSE_30, REVERSE_29, REVERSE_28, REVERSE_27, \
        REVERSE_26, REVERSE_25, REVERSE_24, REVERSE_23, REVERSE_22, REVERSE_21, \
        REVERSE_20, REVERSE_19, REVERSE_18, REVERSE_17, REVERSE_16, REVERSE_15, \
        REVERSE_14, REVERSE_13, REVERSE_12, REVERSE_11, REVERSE_10, REVERSE_09, \
        REVERSE_08, REVERSE_07, REVERSE_06, REVERSE_05, REVERSE_04, REVERSE_03, \
        REVERSE_02, REVERSE_01, REVERSE_00)(__VA_ARGS__)

#define REVERSE_ARG(...) REVERSE_ARG_(##__VA_ARGS__)

/* Apply the macro to each argument */

#define FOREACH_00(action, count, ...)      0
#define FOREACH_01(action, count, arg, ...) action(arg, count - 1 )
#define FOREACH_02(action, count, arg, ...) action(arg, count - 2 ) FOREACH_01(action, count, __VA_ARGS__)
#define FOREACH_03(action, count, arg, ...) action(arg, count - 3 ) FOREACH_02(action, count, __VA_ARGS__)
#define FOREACH_04(action, count, arg, ...) action(arg, count - 4 ) FOREACH_03(action, count, __VA_ARGS__)
#define FOREACH_05(action, count, arg, ...) action(arg, count - 5 ) FOREACH_04(action, count, __VA_ARGS__)
#define FOREACH_06(action, count, arg, ...) action(arg, count - 6 ) FOREACH_05(action, count, __VA_ARGS__)
#define FOREACH_07(action, count, arg, ...) action(arg, count - 7 ) FOREACH_06(action, count, __VA_ARGS__)
#define FOREACH_08(action, count, arg, ...) action(arg, count - 8 ) FOREACH_07(action, count, __VA_ARGS__)
#define FOREACH_09(action, count, arg, ...) action(arg, count - 9 ) FOREACH_08(action, count, __VA_ARGS__)
#define FOREACH_10(action, count, arg, ...) action(arg, count - 10) FOREACH_09(action, count, __VA_ARGS__)
#define FOREACH_11(action, count, arg, ...) action(arg, count - 11) FOREACH_10(action, count, __VA_ARGS__)
#define FOREACH_12(action, count, arg, ...) action(arg, count - 12) FOREACH_11(action, count, __VA_ARGS__)
#define FOREACH_13(action, count, arg, ...) action(arg, count - 13) FOREACH_12(action, count, __VA_ARGS__)
#define FOREACH_14(action, count, arg, ...) action(arg, count - 14) FOREACH_13(action, count, __VA_ARGS__)
#define FOREACH_15(action, count, arg, ...) action(arg, count - 15) FOREACH_14(action, count, __VA_ARGS__)
#define FOREACH_16(action, count, arg, ...) action(arg, count - 16) FOREACH_15(action, count, __VA_ARGS__)
#define FOREACH_17(action, count, arg, ...) action(arg, count - 17) FOREACH_16(action, count, __VA_ARGS__)
#define FOREACH_18(action, count, arg, ...) action(arg, count - 18) FOREACH_17(action, count, __VA_ARGS__)
#define FOREACH_19(action, count, arg, ...) action(arg, count - 19) FOREACH_18(action, count, __VA_ARGS__)
#define FOREACH_20(action, count, arg, ...) action(arg, count - 20) FOREACH_19(action, count, __VA_ARGS__)
#define FOREACH_21(action, count, arg, ...) action(arg, count - 21) FOREACH_20(action, count, __VA_ARGS__)
#define FOREACH_22(action, count, arg, ...) action(arg, count - 22) FOREACH_21(action, count, __VA_ARGS__)
#define FOREACH_23(action, count, arg, ...) action(arg, count - 23) FOREACH_22(action, count, __VA_ARGS__)
#define FOREACH_24(action, count, arg, ...) action(arg, count - 24) FOREACH_23(action, count, __VA_ARGS__)
#define FOREACH_25(action, count, arg, ...) action(arg, count - 25) FOREACH_24(action, count, __VA_ARGS__)
#define FOREACH_26(action, count, arg, ...) action(arg, count - 26) FOREACH_25(action, count, __VA_ARGS__)
#define FOREACH_27(action, count, arg, ...) action(arg, count - 27) FOREACH_26(action, count, __VA_ARGS__)
#define FOREACH_28(action, count, arg, ...) action(arg, count - 28) FOREACH_27(action, count, __VA_ARGS__)
#define FOREACH_29(action, count, arg, ...) action(arg, count - 29) FOREACH_28(action, count, __VA_ARGS__)
#define FOREACH_30(action, count, arg, ...) action(arg, count - 30) FOREACH_29(action, count, __VA_ARGS__)
#define FOREACH_31(action, count, arg, ...) action(arg, count - 31) FOREACH_30(action, count, __VA_ARGS__)
#define FOREACH_32(action, count, arg, ...) action(arg, count - 32) FOREACH_31(action, count, __VA_ARGS__)

#define FOREACH_ARG_(action, count, ...) \
        GET_ARG_VALUE(0, ##__VA_ARGS__, \
        FOREACH_32, FOREACH_31, FOREACH_30, FOREACH_29, FOREACH_28, FOREACH_27, \
        FOREACH_26, FOREACH_25, FOREACH_24, FOREACH_23, FOREACH_22, FOREACH_21, \
        FOREACH_20, FOREACH_19, FOREACH_18, FOREACH_17, FOREACH_16, FOREACH_15, \
        FOREACH_14, FOREACH_13, FOREACH_12, FOREACH_11, FOREACH_10, FOREACH_09, \
        FOREACH_08, FOREACH_07, FOREACH_06, FOREACH_05, FOREACH_04, FOREACH_03, \
        FOREACH_02, FOREACH_01, FOREACH_00)(action, count, ##__VA_ARGS__)

#define FOREACH_ARG(action, ...) \
        FOREACH_ARG_(action, GET_ARG_COUNT(__VA_ARGS__), ##__VA_ARGS__)

/* Stringify the arguments */

#define STRINGIFY_(x) #x
#define STRINGIFY(x)  STRINGIFY_(x)

/* Concatenate the arguments */

#define CONCATENATE(a, b) a##b

#endif /* __INCLUDE_NUTTX_MACRO_H */

