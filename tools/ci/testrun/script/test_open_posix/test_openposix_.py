#!/usr/bin/python3
############################################################################
# tools/ci/testrun/script/test_open_posix/test_openposix_.py
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################
# encoding: utf-8
import pytest

pytestmark = [pytest.mark.sim, pytest.mark.rv_virt]


def test_ltp_interfaces_mq_send_4_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_4_2", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigprocmask_12_1(p):
#     ret = p.sendCommand(
#         "ltp_interfaces_sigprocmask_12_1",
#         ["PASSED", "passed", "Passed", "PASS"],
#         timeout=10,
#     )
#     retID = p.sendCommand("echo $?", "0", timeout=2)
#     assert ret >= 0
#     assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedwrlock_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedwrlock_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_cancel_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_cancel_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_3_23(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_3_23', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_12_41(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_41",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigismember_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigismember_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_time_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_time_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_unlock_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_unlock_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_37_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_37_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_7_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_7_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_unlink_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_unlink_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_getvalue_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_getvalue_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_getdetachstate_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_getdetachstate_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_97(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_97",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_6_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_6_4", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_strlen_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strlen_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_getcpuclockid_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_getcpuclockid_1_1", ["new thread"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_detach_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_detach_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_tryrdlock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_tryrdlock_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_16(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_16",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sem_open_6_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sem_open_6_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_getschedparam_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_getschedparam_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_104(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_104",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_27_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_27_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_22(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_22",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_15(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_15",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_strcpy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strcpy_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_join_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_join_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_35(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_35",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_23(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_23",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_4_84(p):
#     ret = p.sendCommand(
#         "ltp_interfaces_sigaction_4_84",
#         ["PASSED", "passed", "Passed", "PASS"],
#         timeout=10,
#     )
#     retID = p.sendCommand("echo $?", "0", timeout=2)
#     assert ret >= 0
#     assert retID >= 0


def test_ltp_interfaces_sched_setparam_25_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setparam_25_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_1_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_1_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_gettime_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_gettime_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_12(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_12",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_17_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_17_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sched_setscheduler_19_3(p):
# 	ret = p.sendCommand('ltp_interfaces_sched_setscheduler_19_3', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigqueue_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigqueue_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_gettime_speculative_6_1(p):
    ret = p.sendCommand(
        "ltp_timer_gettime_speculative_6_1", ["errno==EINVAL"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setschedprio_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setschedprio_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_69(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_69",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_21_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_21_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_61(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_61",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_settime_speculative_12_3(p):
    ret = p.sendCommand(
        "ltp_timer_settime_speculative_12_3", ["errno==EINVAL"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_init_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_init_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_102(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_102",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_27(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_27",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_read_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_read_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_2_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_2_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_7(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_7",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_4_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_4_3", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedpolicy_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedpolicy_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_close_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_close_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_nanosleep_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_nanosleep_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_93(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_93",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_getparam_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_getparam_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setinheritsched_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setinheritsched_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_rdlock_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_rdlock_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setscope_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setscope_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_fsync_14_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_fsync_14_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_18_8(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_18_8', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_raise_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_raise_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_51(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_51",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_munlock_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_munlock_7_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setcancelstate_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setcancelstate_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_8(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_8",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_55(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_55",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_75(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_75",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_sigmask_6_1(p):
    ret = p.sendCommand("ltp_interfaces_pthread_sigmask_6_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_destroy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_destroy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_4_85(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_4_85', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_mq_open_7_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_7_3", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_77(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_77",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_13_14(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_13_14', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_kill_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_kill_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_25(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_25",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_clock_settime_speculative_4_4(p):
    ret = p.sendCommand(
        "ltp_clock_settime_speculative_4_4",
        ["Implementation does repeat signals on clock reset", "clock reset"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_nanosleep_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_nanosleep_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_18_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_18_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_21(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_21",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_29_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_29_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_settime_speculative_12_2(p):
    ret = p.sendCommand(
        "ltp_timer_settime_speculative_12_2", ["errno==EINVAL"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_12(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_12",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_8(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_8",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_4_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_4_1", ["Inside handler"], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_7(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_7",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_7(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_7",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_3_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_3_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_23(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_23",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigignore_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigignore_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_53(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_53",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_setpshared_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_setpshared_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrier_wait_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_wait_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigprocmask_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigprocmask_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_detach_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_detach_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_spin_trylock_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_spin_trylock_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigpause_1_1(p):
    pytest.skip("unsupported")
    ret = p.sendCommand(
        "ltp_interfaces_sigpause_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_21(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_21",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_79(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_79",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigprocmask_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigprocmask_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sched_setscheduler_19_2(p):
# 	ret = p.sendCommand('ltp_interfaces_sched_setscheduler_19_2', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_attr_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_errno_h_4_1(p):
    ret = p.sendCommand("ltp_definitions_errno_h_4_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaddset_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaddset_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_fsync_8_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_fsync_8_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_detach_4_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_detach_4_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_4_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_21(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_21",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_8_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_8_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_22(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_22",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaddset_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaddset_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_71(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_71",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setinheritsched_2_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setinheritsched_2_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_50(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_50",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedpolicy_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedpolicy_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_14_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_14_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_20(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_20",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_init_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_init_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_16(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_16",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_16(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_16",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedwrlock_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedwrlock_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigpending_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigpending_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigismember_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigismember_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlockattr_destroy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlockattr_destroy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_detach_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_detach_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_sigmask_8_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_sigmask_8_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_write_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_write_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_99(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_99",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_asctime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_asctime_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutexattr_gettype_1_4(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutexattr_gettype_1_4', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_1_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigfillset_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigfillset_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_write_9_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_write_9_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cancel_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cancel_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigtimedwait_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigtimedwait_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_init_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_init_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_14_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_14_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_kill_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_kill_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_60(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_60",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_max_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_max_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_getattr_speculative_7_1(p):
    ret = p.sendCommand(
        "ltp_mq_getattr_speculative_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_setattr_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_setattr_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_kill_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_kill_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_6_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_behavior_WIFEXITED_1_2(p):
    ret = p.sendCommand(
        "ltp_behavior_WIFEXITED_1_2", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_write_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_write_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_19_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_19_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_return_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_return_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_signal_h_19_1(p):
    ret = p.sendCommand("ltp_definitions_signal_h_19_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_rdlock_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_rdlock_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_read_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_read_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_26(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_26",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_49(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_49",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_40(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_40",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigpause_2_1(p):
    pytest.skip("unsupported")
    ret = p.sendCommand(
        "ltp_interfaces_sigpause_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_4_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_4_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigfillset_2_1(p):
    ret = p.sendCommand("ltp_interfaces_sigfillset_2_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_24_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_24_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigqueue_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigprocmask_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigprocmask_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_notify_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_notify_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_fsync_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_fsync_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_14(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_14",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_1_2", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_key_delete_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_key_delete_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigdelset_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigdelset_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_getparam_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_getparam_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_getres_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_getres_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_20_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_20_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_23(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_23",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_6(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_6",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_close_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_close_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_fsync_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_fsync_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_sigmask_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_sigmask_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cleanup_pop_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cleanup_pop_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_self_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_self_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutexattr_gettype_1_2(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutexattr_gettype_1_2', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_13_12(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_12",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_getpshared_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_getpshared_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_settime_speculative_12_1(p):
    ret = p.sendCommand(
        "ltp_timer_settime_speculative_12_1", ["errno==EINVAL"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_aio_error_3_1(p):
# 	ret = p.sendCommand('ltp_interfaces_aio_error_3_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sched_setscheduler_19_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setscheduler_19_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_error_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_error_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_82(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_82",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_12_33(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_12_33', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedwrlock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedwrlock_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_15(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_15",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_26(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_26",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_raise_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_raise_6_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_72(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_72",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_8_1(p):
    ret = p.sendCommand("ltp_interfaces_sigwaitinfo_8_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedrdlock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedrdlock_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_read_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_read_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_ctime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_ctime_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwaitinfo_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_write_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_write_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_6(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_6",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_32_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_32_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_16(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_16",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_24(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_24",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_8_1(p):
    ret = p.sendCommand("ltp_interfaces_sigqueue_8_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_39_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_39_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_24(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_24",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setparam_22_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setparam_22_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_47(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_47",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_23(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_23",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_notify_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_notify_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_yield_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_yield_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_killpg_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_killpg_8_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setscheduler_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setscheduler_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_80(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_80",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_fsync_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_fsync_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaddset_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaddset_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigpause_4_1(p):
    pytest.skip("unsupported")
    ret = p.sendCommand(
        "ltp_interfaces_sigpause_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_aio_write_2_1(p):
# 	ret = p.sendCommand('ltp_interfaces_aio_write_2_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setprotocol_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setprotocol_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_attr_setstack_4_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_attr_setstack_4_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigpending_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigpending_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_38_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_38_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_spin_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_spin_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigemptyset_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigemptyset_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_4_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_4_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_11_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_11_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_sigmask_4_1(p):
#     ret = p.sendCommand("ltp_interfaces_pthread_sigmask_4_1", [""], timeout=10)
#     retID = p.sendCommand("echo $?", "0", timeout=2)
#     assert ret >= 0
#     assert retID >= 0


def test_ltp_interfaces_sigaction_12_48(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_48",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigprocmask_6_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sigprocmask_6_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_23_21(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_21",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_13_7(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_13_7', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_gmtime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_gmtime_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_destroy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_destroy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_20(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_20",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigtimedwait_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigtimedwait_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_59(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_59",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_fsync_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_fsync_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_6_6(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_6_6", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_setpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_setpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_functional_semaphores_sem_philosopher(p):
# 	ret = p.sendCommand('ltp_functional_semaphores_sem_philosopher', ['ap>'], timeout=70)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0

# def test_ltp_definitions_aio_h_2_1(p):
# 	ret = p.sendCommand('ltp_definitions_aio_h_2_1', [''], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sched_get_priority_max_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_max_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_join_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_join_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_25(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_25",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_getvalue_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_getvalue_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_kill_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_kill_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sighold_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sighold_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_sigmask_14_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_sigmask_14_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedwrlock_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedwrlock_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigtimedwait_4_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sigtimedwait_4_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sem_wait_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_25(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_25",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_notify_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_notify_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_26(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_26",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigignore_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigignore_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sighold_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sighold_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_22(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_22",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_26(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_26",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_gettime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_gettime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_29(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_29",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_getpshared_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_getpshared_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_26(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_26",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_raise_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_raise_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_28_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_28_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_15(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_15",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_raise_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_raise_7_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_cancel_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_cancel_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutexattr_settype_3_3(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutexattr_settype_3_3', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_cancel_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cancel_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_trylock_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_trylock_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigdelset_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigdelset_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_return_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_return_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_14(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_14",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_21_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_21_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_12(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_12",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_getres_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_getres_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_20_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_20_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_timer_settime_8_3(p):
# 	ret = p.sendCommand('ltp_interfaces_timer_settime_8_3', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_killpg_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_killpg_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_getvalue_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_getvalue_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_close_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_close_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigrelse_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigrelse_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_9_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_20(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_20",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_81(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_81",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_8_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_sigmask_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_sigmask_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_close_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_close_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_gmtime_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_gmtime_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigqueue_11_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sigqueue_11_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_timer_getoverrun_speculative_6_2(p):
    ret = p.sendCommand(
        "ltp_timer_getoverrun_speculative_6_2", ["errno=EINVAL"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_20(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_20",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setparam_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setparam_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_4_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_4_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_getcpuclockid_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_getcpuclockid_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_26(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_26",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_56(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_56",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_init_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_init_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_functional_semaphores_sem_conpro(p):
    ret = p.sendCommand(
        "ltp_functional_semaphores_sem_conpro", ["taken 900"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_88(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_88",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_6_1(p):
    ret = p.sendCommand("ltp_interfaces_sigqueue_6_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_setspecific_1_2(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_setspecific_1_2', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_23_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigdelset_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigdelset_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_18_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_18_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_28(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_28",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_detach_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_detach_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_24(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_24",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_setpshared_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_setpshared_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_init_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_init_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_notify_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_notify_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_nanosleep_10000_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_nanosleep_10000_1", ["All tests PASSED"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_destroy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_destroy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_setattr_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_setattr_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_getvalue_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_getvalue_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_20(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_20",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_getpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_getpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_20_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_20_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigset_7_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_getclock_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_getclock_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_mq_unlink_1_1(p):
# 	ret = p.sendCommand('ltp_interfaces_mq_unlink_1_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=5)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sched_getscheduler_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_getscheduler_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_nanosleep_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_nanosleep_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_join_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_join_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_28_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_28_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_26_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_26_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mlock_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mlock_5_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_localtime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_localtime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_init_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_init_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_21(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_21",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_getres_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_getres_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cleanup_push_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cleanup_push_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_21(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_21",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_6(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_6",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_destroy_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_destroy_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_7(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_7",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sched_getparam_3_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sched_getparam_3_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaddset_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaddset_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_19_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_19_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_24(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_24",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigrelse_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigrelse_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_timedlock_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_timedlock_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_6(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_6",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_pthread_key_create_speculative_5_1(p):
    ret = p.sendCommand(
        "ltp_pthread_key_create_speculative_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_9_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_9_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_8(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_8",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setcancelstate_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setcancelstate_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_6(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_6",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_attr_setinheritsched_2_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_attr_setinheritsched_2_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_23_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_write_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_write_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_92(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_92",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_fsync_8_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_fsync_8_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_19_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_19_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutexattr_settype_7_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutexattr_settype_7_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_3_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_gettype_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_gettype_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_83(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_83",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigtimedwait_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigtimedwait_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_delete_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_delete_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigprocmask_10_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sigprocmask_10_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_23_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_cancel_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_cancel_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_39_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_39_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_spin_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_spin_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigprocmask_17_1(p):
#     ret = p.sendCommand(
#         "ltp_interfaces_sigprocmask_17_1",
#         ["PASSED", "passed", "Passed", "PASS"],
#         timeout=10,
#     )
#     retID = p.sendCommand("echo $?", "0", timeout=2)
#     assert ret >= 0
#     assert retID >= 0


def test_ltp_interfaces_sigaction_8_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutex_init_4_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutex_init_4_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaltstack_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaltstack_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_15(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_15",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_3_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_20_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_20_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_22_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_22_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_wrlock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_wrlock_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedrdlock_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedrdlock_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_2_1(p):
    ret = p.sendCommand("ltp_interfaces_sigwaitinfo_2_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_44(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_44",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_clock_settime_17_1(p):
# 	ret = p.sendCommand('ltp_interfaces_clock_settime_17_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_sched_getparam_speculative_7_1(p):
    ret = p.sendCommand("ltp_sched_getparam_speculative_7_1", ["NULL"], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_24(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_24",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_sigmask_8_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_sigmask_8_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_nanosleep_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_nanosleep_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_getpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_getpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_kill_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_kill_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setinheritsched_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setinheritsched_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_timedreceive_speculative_10_2(p):
    ret = p.sendCommand(
        "ltp_mq_timedreceive_speculative_10_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_28_15(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_28_15', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_12_30(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_30",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigset_6_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_3_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_3_1", ["Inside handler"], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_23(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_23",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_21(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_21",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_close_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_close_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sem_getvalue_2_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sem_getvalue_2_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_sigmask_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_sigmask_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_91(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_91",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigismember_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigismember_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setparam_23_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setparam_23_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_sigmask_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_sigmask_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_14(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_14",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigdelset_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigdelset_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_mq_open_speculative_26_1(p):
# 	ret = p.sendCommand('ltp_mq_open_speculative_26_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_mq_timedsend_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_20(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_20",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_17_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_17_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaddset_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaddset_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_gettime_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_gettime_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_getscheduler_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_getscheduler_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_getschedparam_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_getschedparam_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_read_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_read_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_14(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_14",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_errno_h_3_2(p):
    ret = p.sendCommand("ltp_definitions_errno_h_3_2", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_90(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_90",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigprocmask_8_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigprocmask_8_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_7_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setprotocol_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setprotocol_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_rr_get_interval_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_rr_get_interval_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwaitinfo_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_destroy_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_destroy_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mlock_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mlock_10_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_init_4_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_init_4_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlockattr_getpshared_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlockattr_getpshared_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_getpshared_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_getpshared_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_munmap_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_munmap_8_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_74(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_74",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_6(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_6",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigprocmask_8_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigprocmask_8_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_24(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_24",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_getres_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_getres_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_26(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_26",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_key_delete_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_key_delete_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_min_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_min_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_19_11(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_19_11', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_23_22(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_22",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigtimedwait_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigtimedwait_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_mq_open_3_1(p):
# 	ret = p.sendCommand('ltp_interfaces_mq_open_3_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedparam_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedparam_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_attr_setstacksize_1_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_attr_setstacksize_1_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_pthread_rwlock_trywrlock_speculative_3_1(p):
    ret = p.sendCommand(
        "ltp_pthread_rwlock_trywrlock_speculative_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedwrlock_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedwrlock_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sem_open_2_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sem_open_2_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_rwlockattr_init_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlockattr_init_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=5)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_nanosleep_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_nanosleep_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_16(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_16",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_cancel_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_cancel_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_mlockall_8_1(p):
# 	ret = p.sendCommand('ltp_interfaces_mlockall_8_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_12_37(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_37",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setcancelstate_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setcancelstate_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sem_open_4_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sem_open_4_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_2_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setschedparam_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setschedparam_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedparam_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedparam_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=5)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setdetachstate_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setdetachstate_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_5_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_5_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_68(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_68",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_key_create_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_key_create_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutexattr_gettype_1_3(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutexattr_gettype_1_3', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_aio_fsync_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_fsync_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_destroy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_destroy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlockattr_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlockattr_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_62(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_62",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlockattr_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlockattr_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_setclock_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_setclock_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_6_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_6_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_equal_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_equal_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_36(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_36",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_delete_speculative_5_2(p):
    ret = p.sendCommand(
        "ltp_timer_delete_speculative_5_2", ["errno=EINVAL"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_behavior_WIFEXITED_1_1(p):
    ret = p.sendCommand(
        "ltp_behavior_WIFEXITED_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_5_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_3_22(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_3_22', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_munlock_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_munlock_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cleanup_pop_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cleanup_pop_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_unlink_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_unlink_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_fsync_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_fsync_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_23(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_23",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigpause_3_1(p):
    pytest.skip("unsupported")
    ret = p.sendCommand(
        "ltp_interfaces_sigpause_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_unlock_4_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_unlock_4_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigpending_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigpending_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cancel_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cancel_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_26(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_26",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_4_64(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_4_64', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_7_1(p):
    ret = p.sendCommand("ltp_interfaces_sigwaitinfo_7_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_2_8(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_2_8', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigset_8_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_8_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_read_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_read_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_max_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_max_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_16(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_16",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_41_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_41_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_87(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_87",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_create_speculative_15_1(p):
    ret = p.sendCommand(
        "ltp_timer_create_speculative_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_spin_lock_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_spin_lock_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_25(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_25",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_6_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_6_2", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_6_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_6_5", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setscheduler_19_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setscheduler_19_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_create_16_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_create_16_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_setspecific_1_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_setspecific_1_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_6_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_killpg_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_killpg_5_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_21(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_21",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_4_1(p):
    ret = p.sendCommand("ltp_interfaces_sigqueue_4_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sem_open_5_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sem_open_5_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_18_12(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_12",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedrdlock_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedrdlock_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_6_1",
        ["PASSED", "passed", "Passed", "PASS", "skipped"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigpending_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigpending_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaltstack_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaltstack_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=5)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_23_10(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_23_10', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_strchr_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strchr_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_20(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_20",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_key_delete_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_key_delete_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_close_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_close_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_17_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_17_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setscheduler_17_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setscheduler_17_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_nanosleep_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_nanosleep_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_38(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_38",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_22(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_22",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_24(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_24",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_15(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_15",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_22(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_22",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutex_trylock_2_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutex_trylock_2_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_mktime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mktime_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwaitinfo_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_atfork_2_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_atfork_2_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_cleanup_push_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cleanup_push_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_sigmask_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_sigmask_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_read_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_read_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_getinheritsched_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_getinheritsched_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_17_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_17_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_6_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_6_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_strftime_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strftime_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_94(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_94",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_24(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_24",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_unlink_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_unlink_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_once_1_2(p):
    ret = p.sendCommand("ltp_interfaces_pthread_once_1_2", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_25(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_25",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_43(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_43",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_unlink_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_unlink_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_attr_getstacksize_1_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_attr_getstacksize_1_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


# def test_ltp_interfaces_pthread_kill_6_1(p):
#     ret = p.sendCommand("ltp_interfaces_pthread_kill_6_1", ["ESRCH"], timeout=10)
#     retID = p.sendCommand("echo $?", "0", timeout=2)
#     assert ret >= 0
#     assert retID >= 0


def test_ltp_interfaces_sigaction_3_15(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_15",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_42(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_42",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_getclock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_getclock_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_killpg_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_killpg_6_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_14_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_14_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutexattr_setpshared_3_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutexattr_setpshared_3_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0

# def test_ltp_interfaces_aio_return_3_2(p):
# 	ret = p.sendCommand('ltp_interfaces_aio_return_3_2', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_attr_setdetachstate_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setdetachstate_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_write_8_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_write_8_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cleanup_push_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cleanup_push_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_18_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_18_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_spin_lock_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_spin_lock_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_getattr_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_getattr_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_open_speculative_6_1(p):
    ret = p.sendCommand("ltp_mq_open_speculative_6_1", ["does not fail"], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setscheduler_17_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setscheduler_17_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_8_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_8_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_28_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_28_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_key_create_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_key_create_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_70(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_70",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_10_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_once_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_once_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_unlink_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_unlink_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_write_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_write_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_attr_setstacksize_4_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_attr_setstacksize_4_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_mq_timedsend_19_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_19_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_trywrlock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_trywrlock_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_17_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_17_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_3_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_103(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_103",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setstack_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setstack_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_rdlock_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_rdlock_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setdetachstate_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setdetachstate_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_functional_semaphores_sem_sleepingbarber(p):
# 	ret = p.sendCommand('ltp_functional_semaphores_sem_sleepingbarber', ['nice hair.'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_definitions_signal_h_13_1(p):
    ret = p.sendCommand(
        "ltp_definitions_signal_h_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_31(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_31",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_gettime_speculative_6_2(p):
    ret = p.sendCommand(
        "ltp_timer_gettime_speculative_6_2", ["errno==EINVAL"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_52(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_52",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_54(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_54",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_attr_setschedparam_1_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_attr_setschedparam_1_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_raise_10000_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_raise_10000_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_aio_h_4_1(p):
    ret = p.sendCommand("ltp_definitions_aio_h_4_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutex_trylock_1_2(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutex_trylock_1_2', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0

# def test_ltp_interfaces_aio_fsync_5_1(p):
# 	ret = p.sendCommand('ltp_interfaces_aio_fsync_5_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_4_86(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_86",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_12(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_12",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_26(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_26",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_8_17(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_8_17', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_8_15(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_15",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_14(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_14",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_7(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_7",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_76(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_76",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutex_destroy_2_2(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutex_destroy_2_2', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_mq_timedsend_11_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_11_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_getdetachstate_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_getdetachstate_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_signal_h_26_1(p):
    ret = p.sendCommand("ltp_definitions_signal_h_26_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_exit_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_exit_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_22(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_22",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_7(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_7",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_21(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_21",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_12(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_12",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_29_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_29_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigprocmask_4_1(p):
#     ret = p.sendCommand(
#         "ltp_interfaces_sigprocmask_4_1",
#         ["PASSED", "passed", "Passed", "PASS"],
#         timeout=10,
#     )
#     retID = p.sendCommand("echo $?", "0", timeout=2)
#     assert ret >= 0
#     assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedpolicy_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedpolicy_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_getpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_getpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_kill_7_1(p):
    ret = p.sendCommand("ltp_interfaces_pthread_kill_7_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutexattr_gettype_1_5(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutexattr_gettype_1_5', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_condattr_destroy_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_destroy_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedparam_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedparam_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_functional_semaphores_sem_readerwriter(p):
# 	ret = p.sendCommand('ltp_functional_semaphores_sem_readerwriter', ['exit.'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_mq_timedsend_speculative_18_2(p):
    ret = p.sendCommand(
        "ltp_mq_timedsend_speculative_18_2", ["did fail on invalid"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_cancel_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_cancel_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_mq_send_9_1(p):
# 	ret = p.sendCommand('ltp_interfaces_mq_send_9_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_mq_send_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigwait_8_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sigwait_8_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0

# def test_ltp_interfaces_sigwait_1_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sigwait_1_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_4_78(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_78",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_16_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_16_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_attr_setstack_6_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_attr_setstack_6_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedpolicy_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedpolicy_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_kill_2_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_kill_2_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedrdlock_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedrdlock_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_25(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_25",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_strftime_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strftime_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setpshared_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setpshared_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_23_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_23_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_getpid_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_getpid_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_fsync_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_fsync_7_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_8(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_8",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_unlink_10_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_unlink_10_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_27_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_27_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_min_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_min_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_20(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_20",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigqueue_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_12(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_12",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_pthread_join_speculative_6_1(p):
    ret = p.sendCommand(
        "ltp_pthread_join_speculative_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_sigmask_16_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_sigmask_16_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_unlink_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_unlink_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigemptyset_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigemptyset_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_58(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_58",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_wrlock_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_wrlock_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_20(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_20",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_unlink_speculative_7_2(p):
    ret = p.sendCommand(
        "ltp_mq_unlink_speculative_7_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_9_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_9_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_21(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_21",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_close_3_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_close_3_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_rwlock_rdlock_1_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_rwlock_rdlock_1_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigignore_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigignore_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_8(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_8",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_7(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_7",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_nanosleep_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_nanosleep_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_shm_open_25_1(p):
#     ret = p.sendCommand(
#         "ltp_interfaces_shm_open_25_1",
#         ["PASSED", "passed", "Passed", "PASS"],
#         timeout=10,
#     )
#     retID = p.sendCommand("echo $?", "0", timeout=2)
#     assert ret >= 0
#     assert retID >= 0


def test_ltp_interfaces_sem_wait_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_max_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_max_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_pthread_getcpuclockid_speculative_3_1(p):
    ret = p.sendCommand(
        "ltp_pthread_getcpuclockid_speculative_3_1", ["doesn't exist"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_65(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_65",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_getattr_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_getattr_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_getscheduler_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_getscheduler_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_once_2_1(p):
    ret = p.sendCommand("ltp_interfaces_pthread_once_2_1", [""], timeout=20)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setpshared_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setpshared_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_difftime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_difftime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_getschedparam_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_getschedparam_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_23(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_23",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_killpg_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_killpg_4_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_15(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_15",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_setattr_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_setattr_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_19_15(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_19_15', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0

# def test_ltp_interfaces_fsync_5_1(p):
# 	ret = p.sendCommand('ltp_interfaces_fsync_5_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigset_5_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_5_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_14(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_14",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_4_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_6_14(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_6_14', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_18_23(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_23",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_fork_1_1(p):
# 	ret = p.sendCommand('ltp_interfaces_fork_1_1', [''], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_28_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_12(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_12",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrier_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=5)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwaitinfo_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_timer_getoverrun_speculative_6_1(p):
# 	ret = p.sendCommand('ltp_timer_getoverrun_speculative_6_1', ['errno=EINVAL'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_4_89(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_89",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_14(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_14",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_25(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_25",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_100(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_100",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_gettime_speculative_6_3(p):
    ret = p.sendCommand(
        "ltp_timer_gettime_speculative_6_3", ["errno==EINVAL"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_join_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_join_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_24(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_24",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_25(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_25",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_suspend_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_suspend_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigaction_18_16(p):
# 	ret = p.sendCommand('ltp_interfaces_sigaction_18_16', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_condattr_setpshared_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_setpshared_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_setclock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_setclock_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_write_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_write_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_strftime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strftime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_unlock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_unlock_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_73(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_73",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_aio_write_1_2(p):
# 	ret = p.sendCommand('ltp_interfaces_aio_write_1_2', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_mq_getattr_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_getattr_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_setattr_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_setattr_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_min_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_min_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setpshared_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setpshared_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_fsync_8_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_fsync_8_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_8(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_8",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_67(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_67",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_25(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_25",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_create_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_create_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_getschedpolicy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_getschedpolicy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_join_6_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_join_6_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sched_setscheduler_19_4(p):
# 	ret = p.sendCommand('ltp_interfaces_sched_setscheduler_19_4', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_13_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_unlink_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_unlink_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrier_destroy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_destroy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_19_16(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_19_16",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setschedparam_1_2(p):
    ret = p.sendCommand("ltp_interfaces_pthread_setschedparam_1_2", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_63(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_63",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_8_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_34(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_34",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setschedparam_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setschedparam_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_7_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_read_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_read_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_12(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_12",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_39(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_39",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_5_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_5_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_16(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_16",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrier_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_7(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_7",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigprocmask_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigprocmask_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_46(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_46",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_25(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_25",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sigqueue_7_1(p):
#     ret = p.sendCommand(
#         "ltp_interfaces_sigqueue_7_1", ["1, 2, 3", "62, 63,"], timeout=10
#     )
#     retID = p.sendCommand("echo $?", "0", timeout=2)
#     assert ret >= 0
#     assert retID >= 0


def test_ltp_interfaces_mq_open_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_7_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_mq_open_speculative_2_2(p):
# 	ret = p.sendCommand('ltp_mq_open_speculative_2_2', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_pthread_barrier_wait_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_wait_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_timers_clocks_invaliddates(p):
    ret = p.sendCommand(
        "ltp_timers_clocks_invaliddates",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_open_speculative_2_3(p):
    ret = p.sendCommand(
        "ltp_mq_open_speculative_2_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigdelset_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigdelset_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setscope_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setscope_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_getattr_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_getattr_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_14(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_14",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_11_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_11_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_rr_get_interval_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_rr_get_interval_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_attr_getstack_1_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_attr_getstack_1_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_18_7(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_7",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_sched_rr_get_interval_speculative_5_1(p):
    ret = p.sendCommand(
        "ltp_sched_rr_get_interval_speculative_5_1", ["NULL"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_read_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_read_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutexattr_getpshared_1_2(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutexattr_getpshared_1_2', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_raise_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_raise_4_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_strncpy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strncpy_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setinheritsched_2_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setinheritsched_2_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=5)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_10_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_10_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_21_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_21_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_95(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_95",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_nanosleep_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_nanosleep_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_behavior_timers_2_1(p):
    ret = p.sendCommand(
        "ltp_behavior_timers_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_aio_return_3_1(p):
# 	ret = p.sendCommand('ltp_interfaces_aio_return_3_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sighold_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sighold_3_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setcancelstate_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setcancelstate_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_20_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_20_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedrdlock_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedrdlock_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_raise_1_2(p):
# 	ret = p.sendCommand('ltp_interfaces_raise_1_2', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_clock_getres_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_getres_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_getspecific_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_getspecific_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_20(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_20",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_1_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_1_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_fsync_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_fsync_4_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_6(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_6",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_45(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_45",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setdetachstate_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setdetachstate_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_behavior_timers_1_1(p):
    ret = p.sendCommand(
        "ltp_behavior_timers_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_8(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_8",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_25_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_25_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setinheritsched_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setinheritsched_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_66(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_66",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_16(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_16",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_gettime_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_gettime_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_8(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_8",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_sem_unlink_6_1(p):
# 	ret = p.sendCommand('ltp_interfaces_sem_unlink_6_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_timer_getoverrun_speculative_6_3(p):
    ret = p.sendCommand(
        "ltp_timer_getoverrun_speculative_6_3", ["errno==EINVAL"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_mutexattr_settype_1_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_mutexattr_settype_1_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_aio_cancel_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_cancel_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_2_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_2_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_sched_h_10_1(p):
    ret = p.sendCommand("ltp_definitions_sched_h_10_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_3_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_3_1", ["Inside handler"], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_nanosleep_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_nanosleep_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigrelse_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigrelse_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cancel_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cancel_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_20_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_20_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_3_2", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_delete_speculative_5_1(p):
    ret = p.sendCommand(
        "ltp_timer_delete_speculative_5_1", ["errno=EINVAL"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


# def test_ltp_interfaces_pthread_attr_setstack_1_1(p):
# 	ret = p.sendCommand('ltp_interfaces_pthread_attr_setstack_1_1', ['PASSED', 'passed', 'Passed', 'PASS'], timeout=10)
# 	retID = p.sendCommand('echo $?', '0', timeout=2)
# 	assert ret >= 0
# 	assert retID >= 0


def test_ltp_interfaces_sigaction_28_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_getres_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_getres_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_10_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_10_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_12_32(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_12_32",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_testcancel_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_testcancel_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_setclock_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_setclock_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_read_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_read_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_22(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_22",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_stress_signals_sigismember_stress_1(p):
    ret = p.sendCommand("ltp_stress_signals_sigismember_stress_1", [""], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_mqueue_h_1_1(p):
    ret = p.sendCommand(
        "ltp_definitions_mqueue_h_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_setclock_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_setclock_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_killpg_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_killpg_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrier_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlockattr_setpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlockattr_setpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_close_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_close_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_open_14_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_open_14_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mmap_23_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mmap_23_1", ["PASSED", "passed", "Passed", "PASS"], timeout=10
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_close_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_close_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_setpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_setpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigqueue_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cancel_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cancel_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlockattr_getpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlockattr_getpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_6(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_6",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_gettime_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_gettime_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigignore_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigignore_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_22(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_22",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigprocmask_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigprocmask_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_shm_unlink_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_shm_unlink_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_96(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_96",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_6(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_6",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_23(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_23",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_101(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_101",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_gettime_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_gettime_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_6(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_6",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_sigmask_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_sigmask_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_26(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_26",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_aio_read_11_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_aio_read_11_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_lio_listio_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_lio_listio_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_13_24(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_13_24",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_delete_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_delete_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_stress_semaphores_multi_con_pro(p):
    ret = p.sendCommand("ltp_stress_semaphores_multi_con_pro 5", ["exit."], timeout=10)
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setpshared_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setpshared_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_getprotocol_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_getprotocol_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_98(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_98",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_57(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_57",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=10,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_5_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_5_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=200,
    )
    retID = p.sendCommand("echo $?", "0", timeout=2)
    assert ret >= 0
    assert retID >= 0
