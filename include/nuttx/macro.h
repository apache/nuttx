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

/* Stringify the arguments */

#define STRINGIFY_(x) #x
#define STRINGIFY(x)  STRINGIFY_(x)

/* Concatenate the arguments */

#define CONCATENATE_(a, b) a##b
#define CONCATENATE(a, b) CONCATENATE_(a,b)

#define GET_ARG_VALUE(_00, _01, _02, _03, _04, _05, _06, _07, \
                      _08, _09, _10, _11, _12, _13, _14, _15, \
                      _16, _17, _18, _19, _20, _21, _22, _23, \
                      _24, _25, _26, _27, _28, _29, _30, _31, \
                      _32, name, ...) name

/* Get the number of arguments (up to 32)
 *
 * C++ strict mode drops the GNU ", ##__VA_ARGS__" comma elision, which
 * would shift the selector off by one for the zero-argument case; use the
 * standard __VA_OPT__ there instead.  This is the only place where the
 * empty-argument case needs special handling: every higher-level macro
 * (REVERSE_ARG, FOREACH_ARG, ...) dispatches on top of GET_ARG_COUNT.
 */

#if defined(__cplusplus)
#  define GET_ARG_COUNT(...) \
        GET_ARG_VALUE(_0 __VA_OPT__(,) __VA_ARGS__, 32, 31, 30, \
        29, 28, 27, 26, 25, 24, 23, 22, 21, 20, \
        19, 18, 17, 16, 15, 14, 13, 12, 11, 10, \
        9,  8,  7,  6,  5,  4,  3,  2,  1,  0)
#else
#  define GET_ARG_COUNT(...) \
        GET_ARG_VALUE(_0, ##__VA_ARGS__, 32, 31, 30, \
        29, 28, 27, 26, 25, 24, 23, 22, 21, 20, \
        19, 18, 17, 16, 15, 14, 13, 12, 11, 10, \
        9,  8,  7,  6,  5,  4,  3,  2,  1,  0)
#endif

/* Expand the arguments */

#define EXPAND_(...) __VA_ARGS__
#define EXPAND(...) EXPAND_(__VA_ARGS__)

/* Reverse the arguments */

#define REVERSE_0()
#define REVERSE_1(a)     a
#define REVERSE_2(a,b)   b,a
#define REVERSE_3(a,...) EXPAND(REVERSE_2(__VA_ARGS__)),a
#define REVERSE_4(a,...) EXPAND(REVERSE_3(__VA_ARGS__)),a
#define REVERSE_5(a,...) EXPAND(REVERSE_4(__VA_ARGS__)),a
#define REVERSE_6(a,...) EXPAND(REVERSE_5(__VA_ARGS__)),a
#define REVERSE_7(a,...) EXPAND(REVERSE_6(__VA_ARGS__)),a
#define REVERSE_8(a,...) EXPAND(REVERSE_7(__VA_ARGS__)),a
#define REVERSE_9(a,...) EXPAND(REVERSE_8(__VA_ARGS__)),a
#define REVERSE_10(a,...) EXPAND(REVERSE_9(__VA_ARGS__)),a
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

/* Select the worker through GET_ARG_COUNT so the zero-argument case
 * expands correctly in both C and C++ (see GET_ARG_COUNT).
 */

#define REVERSE_ARG(...) \
        CONCATENATE(REVERSE_, GET_ARG_COUNT(__VA_ARGS__))(__VA_ARGS__)

/* Apply the macro to each argument, passing the position of the argument
 * as a literal (0, 1, 2, ...) rather than as an expression, so that the
 * action can paste it into an identifier, for instance to build a symbol
 * name the linker can sort.  The list is reversed before the expansion
 * and each step emits the recursion before its own action, so both the
 * indexes and the order of the actions follow the original list.
 */

#define FOREACH_0(action, param, ...)
#define FOREACH_1(action, param, arg, ...) action(param, arg, 0)
#define FOREACH_2(action, param, arg, ...) FOREACH_1(action, param, __VA_ARGS__) action(param, arg, 1)
#define FOREACH_3(action, param, arg, ...) FOREACH_2(action, param, __VA_ARGS__) action(param, arg, 2)
#define FOREACH_4(action, param, arg, ...) FOREACH_3(action, param, __VA_ARGS__) action(param, arg, 3)
#define FOREACH_5(action, param, arg, ...) FOREACH_4(action, param, __VA_ARGS__) action(param, arg, 4)
#define FOREACH_6(action, param, arg, ...) FOREACH_5(action, param, __VA_ARGS__) action(param, arg, 5)
#define FOREACH_7(action, param, arg, ...) FOREACH_6(action, param, __VA_ARGS__) action(param, arg, 6)
#define FOREACH_8(action, param, arg, ...) FOREACH_7(action, param, __VA_ARGS__) action(param, arg, 7)
#define FOREACH_9(action, param, arg, ...) FOREACH_8(action, param, __VA_ARGS__) action(param, arg, 8)
#define FOREACH_10(action, param, arg, ...) FOREACH_9(action, param, __VA_ARGS__) action(param, arg, 9)
#define FOREACH_11(action, param, arg, ...) FOREACH_10(action, param, __VA_ARGS__) action(param, arg, 10)
#define FOREACH_12(action, param, arg, ...) FOREACH_11(action, param, __VA_ARGS__) action(param, arg, 11)
#define FOREACH_13(action, param, arg, ...) FOREACH_12(action, param, __VA_ARGS__) action(param, arg, 12)
#define FOREACH_14(action, param, arg, ...) FOREACH_13(action, param, __VA_ARGS__) action(param, arg, 13)
#define FOREACH_15(action, param, arg, ...) FOREACH_14(action, param, __VA_ARGS__) action(param, arg, 14)
#define FOREACH_16(action, param, arg, ...) FOREACH_15(action, param, __VA_ARGS__) action(param, arg, 15)
#define FOREACH_17(action, param, arg, ...) FOREACH_16(action, param, __VA_ARGS__) action(param, arg, 16)
#define FOREACH_18(action, param, arg, ...) FOREACH_17(action, param, __VA_ARGS__) action(param, arg, 17)
#define FOREACH_19(action, param, arg, ...) FOREACH_18(action, param, __VA_ARGS__) action(param, arg, 18)
#define FOREACH_20(action, param, arg, ...) FOREACH_19(action, param, __VA_ARGS__) action(param, arg, 19)
#define FOREACH_21(action, param, arg, ...) FOREACH_20(action, param, __VA_ARGS__) action(param, arg, 20)
#define FOREACH_22(action, param, arg, ...) FOREACH_21(action, param, __VA_ARGS__) action(param, arg, 21)
#define FOREACH_23(action, param, arg, ...) FOREACH_22(action, param, __VA_ARGS__) action(param, arg, 22)
#define FOREACH_24(action, param, arg, ...) FOREACH_23(action, param, __VA_ARGS__) action(param, arg, 23)
#define FOREACH_25(action, param, arg, ...) FOREACH_24(action, param, __VA_ARGS__) action(param, arg, 24)
#define FOREACH_26(action, param, arg, ...) FOREACH_25(action, param, __VA_ARGS__) action(param, arg, 25)
#define FOREACH_27(action, param, arg, ...) FOREACH_26(action, param, __VA_ARGS__) action(param, arg, 26)
#define FOREACH_28(action, param, arg, ...) FOREACH_27(action, param, __VA_ARGS__) action(param, arg, 27)
#define FOREACH_29(action, param, arg, ...) FOREACH_28(action, param, __VA_ARGS__) action(param, arg, 28)
#define FOREACH_30(action, param, arg, ...) FOREACH_29(action, param, __VA_ARGS__) action(param, arg, 29)
#define FOREACH_31(action, param, arg, ...) FOREACH_30(action, param, __VA_ARGS__) action(param, arg, 30)
#define FOREACH_32(action, param, arg, ...) FOREACH_31(action, param, __VA_ARGS__) action(param, arg, 31)

/* The reversed list has to be expanded before the arguments are counted,
 * hence the extra indirection:  a macro counts the tokens it receives, not
 * what they expand to.
 */

#define FOREACH_ARG__(action, param, ...) \
        CONCATENATE(FOREACH_, GET_ARG_COUNT(__VA_ARGS__)) \
        (action, param, __VA_ARGS__)

#define FOREACH_ARG_(...) FOREACH_ARG__(__VA_ARGS__)

#define FOREACH_ARG(action, param, ...) \
        FOREACH_ARG_(action, param, REVERSE_ARG(__VA_ARGS__))

#endif /* __INCLUDE_NUTTX_MACRO_H */
