#!/usr/bin/python3
# encoding: utf-8
import pytest

pytestmark = [pytest.mark.disable_autouse]


def test_ltp_interfaces_pthread_exit_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_exit_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigdelset_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigdelset_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_difftime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_difftime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setscheduler_19_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setscheduler_19_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_destroy_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_destroy_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setschedprio_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setschedprio_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cleanup_pop_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cleanup_pop_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mktime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mktime_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_pthread_join_speculative_6_1(p):
    ret = p.sendCommand(
        "ltp_pthread_join_speculative_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_getpshared_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_getpshared_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_testcancel_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_testcancel_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigqueue_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_getvalue_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_getvalue_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_close_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_close_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_min_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_min_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_open_speculative_2_3(p):
    ret = p.sendCommand(
        "ltp_mq_open_speculative_2_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_kill_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_kill_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_18_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_18_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_6_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_6_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_10_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_10_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_close_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_close_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_82(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_82",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setpshared_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setpshared_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_destroy_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_destroy_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setdetachstate_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setdetachstate_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_exit_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_exit_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_setattr_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_setattr_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setcancelstate_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setcancelstate_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_23_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_23_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_17_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_17_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_88(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_88",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_create_speculative_5_1(p):
    ret = p.sendCommand(
        "ltp_timer_create_speculative_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_timers_clocks_invaliddates(p):
    ret = p.sendCommand(
        "ltp_timers_clocks_invaliddates",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_max_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_max_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_getres_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_getres_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_init_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_init_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedparam_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedparam_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_getattr_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_getattr_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_destroy_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_destroy_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setinheritsched_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setinheritsched_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_self_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_self_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_yield_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_yield_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setparam_23_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setparam_23_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_21_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_21_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setschedparam_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setschedparam_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigpending_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigpending_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_5_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_5_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_close_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_close_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrier_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_gettime_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_gettime_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_errno_h_3_2(p):
    ret = p.sendCommand("ltp_definitions_errno_h_3_2", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_getvalue_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_getvalue_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_96(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_96",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_9_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_70(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_70",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigqueue_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_4_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedpolicy_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedpolicy_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_54(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_54",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_1_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_1_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_61(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_61",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_9_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_9_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_settype_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_settype_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_init_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_init_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigtimedwait_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigtimedwait_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_functional_mqueues_send_rev_2(p):
    ret = p.sendCommand(
        "ltp_functional_mqueues_send_rev_2", ["in thread receive"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_settype_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_settype_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_2_1(p):
    ret = p.sendCommand("ltp_interfaces_sigwaitinfo_2_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_setpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_setpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setscheduler_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setscheduler_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sighold_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sighold_3_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_1_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_1_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_kill_7_1(p):
    ret = p.sendCommand("ltp_interfaces_pthread_kill_7_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_init_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_init_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_4_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_4_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_wrlock_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_wrlock_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setprotocol_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setprotocol_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_95(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_95",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_unlock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_unlock_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_setclock_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_setclock_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_once_1_2(p):
    ret = p.sendCommand("ltp_interfaces_pthread_once_1_2", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cancel_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cancel_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_getattr_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_getattr_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_strcpy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strcpy_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_create_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_create_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_open_speculative_2_2(p):
    ret = p.sendCommand(
        "ltp_mq_open_speculative_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_4_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_4_3", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_strlen_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strlen_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_6_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_6_2", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaddset_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaddset_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_init_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_init_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_8_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_8_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_8_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cleanup_pop_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cleanup_pop_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setparam_25_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setparam_25_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_once_2_1(p):
    ret = p.sendCommand("ltp_interfaces_pthread_once_2_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_detach_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_detach_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_getspecific_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_getspecific_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_gettime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_gettime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_strftime_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strftime_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_63(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_63",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_getattr_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_getattr_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_getpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_getpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_71(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_71",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setpshared_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setpshared_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigrelse_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigrelse_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_getattr_speculative_7_1(p):
    ret = p.sendCommand(
        "ltp_mq_getattr_speculative_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_3_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_getres_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_getres_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_signal_h_26_1(p):
    ret = p.sendCommand("ltp_definitions_signal_h_26_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_87(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_87",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_gettime_speculative_6_2(p):
    ret = p.sendCommand(
        "ltp_timer_gettime_speculative_6_2",
        ["fcn returned -1", "fcn did not return -1"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_setattr_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_setattr_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaddset_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaddset_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_19_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_19_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_getclock_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_getclock_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setparam_22_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setparam_22_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_join_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_join_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cancel_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cancel_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_3_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_3_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_unlink_speculative_7_2(p):
    ret = p.sendCommand(
        "ltp_mq_unlink_speculative_7_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_3_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_3_1", ["Inside handler"], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cancel_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cancel_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_setattr_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_setattr_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_getparam_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_getparam_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_fsync_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_fsync_4_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedparam_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedparam_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_timedreceive_speculative_10_2(p):
    ret = p.sendCommand(
        "ltp_mq_timedreceive_speculative_10_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigtimedwait_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigtimedwait_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_gettype_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_gettype_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_14_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_14_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_strchr_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strchr_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_stress_signals_sigismember_stress_1(p):
    ret = p.sendCommand("ltp_stress_signals_sigismember_stress_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_getpshared_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_getpshared_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_97(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_97",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_create_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_create_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_getparam_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_getparam_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_notify_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_notify_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_join_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_join_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_close_3_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_close_3_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_close_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_close_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_create_16_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_create_16_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setspecific_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setspecific_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_6_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_6_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_29_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_29_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_gettime_speculative_6_3(p):
    ret = p.sendCommand(
        "ltp_timer_gettime_speculative_6_3",
        ["fcn returned -1", "fcn did not return -1"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_exit_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_exit_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigtimedwait_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigtimedwait_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_17_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_17_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_2_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_2_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_trylock_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_trylock_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_destroy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_destroy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_once_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_once_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_69(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_69",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_delete_speculative_5_2(p):
    ret = p.sendCommand(
        "ltp_timer_delete_speculative_5_2",
        ["fcn returned -1", "fcn did not return -1"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigqueue_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_timedlock_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_timedlock_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_setpshared_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_setpshared_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_getinheritsched_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_getinheritsched_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_89(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_89",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setdetachstate_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setdetachstate_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_4_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_4_1", ["Inside handler"], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrier_destroy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_destroy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setinheritsched_2_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setinheritsched_2_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_errno_h_4_1(p):
    ret = p.sendCommand("ltp_definitions_errno_h_4_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_min_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_min_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_7_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_7_3", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigdelset_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigdelset_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_nanosleep_10000_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_nanosleep_10000_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_destroy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_destroy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaddset_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaddset_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_unlock_4_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_unlock_4_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwaitinfo_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigdelset_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigdelset_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_asctime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_asctime_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_create_speculative_15_1(p):
    ret = p.sendCommand(
        "ltp_timer_create_speculative_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_getschedparam_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_getschedparam_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_17_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_17_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedrdlock_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedrdlock_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_max_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_max_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_close_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_close_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedparam_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedparam_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_getres_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_getres_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setinheritsched_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setinheritsched_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setinheritsched_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setinheritsched_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_9_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_9_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_nanosleep_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_nanosleep_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_5_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_5_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_13_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_13_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_4_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_4_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cleanup_push_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cleanup_push_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_settime_speculative_12_3(p):
    ret = p.sendCommand(
        "ltp_timer_settime_speculative_12_3",
        ["fcn returned -1", "fcn did not return -1"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_5_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_5_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_19_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_19_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_destroy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_destroy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_getscheduler_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_getscheduler_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setschedparam_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setschedparam_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_settype_3_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_settype_3_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_5_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_5_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=200,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_65(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_65",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedparam_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedparam_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_kill_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_kill_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigqueue_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setcancelstate_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setcancelstate_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setparam_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setparam_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_detach_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_detach_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_getschedparam_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_getschedparam_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_min_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_min_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_nanosleep_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_nanosleep_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setdetachstate_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setdetachstate_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_create_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_create_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_getschedparam_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_getschedparam_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedwrlock_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedwrlock_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cancel_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cancel_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_unlink_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_unlink_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaddset_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaddset_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_3_2", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setscheduler_19_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setscheduler_19_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_setscheduler_17_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_setscheduler_17_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_unlink_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_unlink_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_init_4_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_init_4_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_destroy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_destroy_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_getdetachstate_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_getdetachstate_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_setclock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_setclock_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_delete_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_delete_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrier_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_rdlock_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_rdlock_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_8_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_8_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_time_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_time_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_open_speculative_26_1(p):
    ret = p.sendCommand(
        "ltp_mq_open_speculative_26_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_9_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_6_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_getscheduler_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_getscheduler_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwaitinfo_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_nanosleep_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_nanosleep_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_17_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_17_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_5_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_5_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_timedsend_speculative_18_2(p):
    ret = p.sendCommand(
        "ltp_mq_timedsend_speculative_18_2", ["did not fail"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_8_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_8_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigignore_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigignore_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_kill_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_kill_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_gettime_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_gettime_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_sched_h_10_1(p):
    ret = p.sendCommand("ltp_definitions_sched_h_10_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigismember_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigismember_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_20_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_20_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_nanosleep_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_nanosleep_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_8_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigemptyset_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigemptyset_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_3_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_close_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_close_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_detach_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_detach_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_27_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_27_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_timedrdlock_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_timedrdlock_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_9_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_9_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigignore_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigignore_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedpolicy_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedpolicy_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_91(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_91",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_clock_settime_speculative_4_4(p):
    ret = p.sendCommand(
        "ltp_clock_settime_speculative_4_4",
        ["Implementation does repeat signals on clock reset"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigdelset_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigdelset_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_83(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_83",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setpshared_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setpshared_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_18_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_18_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_3_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_3_1", ["Inside handler"], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigfillset_2_1(p):
    ret = p.sendCommand("ltp_interfaces_sigfillset_2_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setschedpolicy_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setschedpolicy_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_get_priority_max_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_get_priority_max_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_56(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_56",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_getvalue_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_getvalue_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigdelset_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigdelset_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setschedparam_1_2(p):
    ret = p.sendCommand("ltp_interfaces_pthread_setschedparam_1_2", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_equal_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_equal_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_key_delete_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_key_delete_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_getvalue_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_getvalue_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_nanosleep_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_nanosleep_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_close_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_close_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrier_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_unlink_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_unlink_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_behavior_timers_2_1(p):
    ret = p.sendCommand(
        "ltp_behavior_timers_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrier_wait_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_wait_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_19(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_19",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_8_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_8_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setcancelstate_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setcancelstate_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_localtime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_localtime_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_sched_getparam_speculative_7_1(p):
    ret = p.sendCommand("ltp_sched_getparam_speculative_7_1", ["NULL"], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_strncpy_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_strncpy_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_20_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_20_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setinheritsched_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setinheritsched_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_3_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_3_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_signal_7_1(p):
    ret = p.sendCommand("ltp_interfaces_signal_7_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigtimedwait_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigtimedwait_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_join_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_join_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_key_delete_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_key_delete_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwaitinfo_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_14_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_14_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_11(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_11",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_2_1(p):
    ret = p.sendCommand("ltp_interfaces_sigset_2_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_init_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_init_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_init_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_init_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cleanup_pop_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cleanup_pop_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_settime_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_settime_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_setclock_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_setclock_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_key_create_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_key_create_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cond_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cond_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_gettime_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_gettime_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_mq_open_speculative_6_1(p):
    ret = p.sendCommand("ltp_mq_open_speculative_6_1", ["does not fail"], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_fsync_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_fsync_5_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_62(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_62",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_18_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_18_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setdetachstate_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setdetachstate_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_equal_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_equal_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_getpshared_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_getpshared_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_4_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cancel_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cancel_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_raise_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_raise_7_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_timer_gettime_speculative_6_1(p):
    ret = p.sendCommand(
        "ltp_timer_gettime_speculative_6_1",
        ["fcn returned -1", "fcn did not return -1"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_3_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_getpshared_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_getpshared_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_ctime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_ctime_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_gettype_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_gettype_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_20_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_20_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_init_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_init_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_28_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_28_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_getparam_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_getparam_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_key_delete_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_key_delete_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_post_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_post_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_nanosleep_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_nanosleep_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cleanup_push_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cleanup_push_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_key_create_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_key_create_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_gettype_1_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_gettype_1_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_open_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_open_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_14_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_14_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_raise_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_raise_6_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_gmtime_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_gmtime_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrier_wait_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrier_wait_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_getvalue_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_getvalue_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_getattr_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_getattr_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_join_6_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_join_6_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_getres_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_getres_6_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_8_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_8_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_6_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigset_6_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_getdetachstate_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_getdetachstate_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_destroy_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_destroy_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_pthread_key_create_speculative_5_1(p):
    ret = p.sendCommand(
        "ltp_pthread_key_create_speculative_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_57(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_57",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_getscheduler_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_getscheduler_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_10_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_10_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwaitinfo_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwaitinfo_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_11_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_11_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigqueue_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutex_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutex_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedreceive_10_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedreceive_10_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_create_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_create_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_setcancelstate_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_setcancelstate_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setpshared_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setpshared_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_condattr_getclock_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_condattr_getclock_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_gmtime_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_gmtime_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_12_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_12_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_9(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_9",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_rr_get_interval_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_rr_get_interval_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_6_10(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_6_10",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_7_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_gettype_1_5(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_gettype_1_5",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_setpshared_1_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_setpshared_1_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_7_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_nanosleep_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_nanosleep_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_detach_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_detach_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_19_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_19_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigtimedwait_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigtimedwait_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_getprotocol_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_getprotocol_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigwait_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigwait_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_receive_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_receive_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_7_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_1_13(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_1_13",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_settime_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_settime_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigqueue_6_1(p):
    ret = p.sendCommand("ltp_interfaces_sigqueue_6_1", [""], timeout=30)
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_destroy_1_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_destroy_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_rwlock_unlock_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_rwlock_unlock_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_timedsend_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_timedsend_8_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_kill_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_kill_2_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_barrierattr_init_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_barrierattr_init_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_4_80(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_4_80",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_init_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_init_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_definitions_mqueue_h_1_1(p):
    ret = p.sendCommand(
        "ltp_definitions_mqueue_h_1_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_clock_gettime_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_clock_gettime_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_setinheritsched_2_4(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_setinheritsched_2_4",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_timedwait_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_timedwait_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_init_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_init_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_3_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_3_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigset_7_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigset_7_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_mutexattr_gettype_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_mutexattr_gettype_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_attr_destroy_3_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_attr_destroy_3_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_open_15_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_open_15_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_23_18(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_23_18",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sched_rr_get_interval_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sched_rr_get_interval_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_send_8_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_send_8_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_setattr_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_setattr_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_2(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_2",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sigaction_2_17(p):
    ret = p.sendCommand(
        "ltp_interfaces_sigaction_2_17",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_sem_wait_5_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_sem_wait_5_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_mq_close_4_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_mq_close_4_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_behavior_timers_1_1(p):
    ret = p.sendCommand(
        "ltp_behavior_timers_1_1", ["PASSED", "passed", "Passed", "PASS"], timeout=30
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_timer_gettime_2_1(p):
    ret = p.sendCommand(
        "ltp_interfaces_timer_gettime_2_1",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0


def test_ltp_interfaces_pthread_cleanup_push_1_3(p):
    ret = p.sendCommand(
        "ltp_interfaces_pthread_cleanup_push_1_3",
        ["PASSED", "passed", "Passed", "PASS"],
        timeout=30,
    )
    retID = p.sendCommand("echo $?", "0")
    assert ret >= 0
    assert retID >= 0
