#!/usr/bin/python3
############################################################################
# tools/ci/testrun/script/test_libuv/test_libuv.py
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


class TestLibuv:
    pytestmark = [pytest.mark.sim]

    def test_test_macros(self, p):
        ret = p.sendCommand(
            "uv_run_tests test_macros", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_close_order(self, p):
        ret = p.sendCommand(
            "uv_run_tests close_order", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_run_once(self, p):
        ret = p.sendCommand(
            "uv_run_tests run_once", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_run_nowait(self, p):
        ret = p.sendCommand(
            "uv_run_tests run_nowait", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_loop_alive(self, p):
        ret = p.sendCommand(
            "uv_run_tests loop_alive", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_loop_close(self, p):
        ret = p.sendCommand(
            "uv_run_tests loop_close", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_loop_instant_close(self, p):
        ret = p.sendCommand(
            "uv_run_tests loop_instant_close", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_loop_stop(self, p):
        ret = p.sendCommand(
            "uv_run_tests loop_stop", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_loop_backend_timeout(self, p):
        ret = p.sendCommand(
            "uv_run_tests loop_backend_timeout", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_default_loop_close(self, p):
        ret = p.sendCommand(
            "uv_run_tests default_loop_close", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_barrier_1(self, p):
        ret = p.sendCommand(
            "uv_run_tests barrier_1", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_barrier_2(self, p):
        ret = p.sendCommand(
            "uv_run_tests barrier_2", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_barrier_3(self, p):
        ret = p.sendCommand(
            "uv_run_tests barrier_3", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_barrier_serial_thread(self, p):
        ret = p.sendCommand(
            "uv_run_tests barrier_serial_thread", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_barrier_serial_thread_single(self, p):
        ret = p.sendCommand(
            "uv_run_tests barrier_serial_thread_single",
            ["not ok 1 -", "ok 1 -"],
            timeout=10,
        )
        assert ret == 1

    def test_condvar_1(self, p):
        ret = p.sendCommand(
            "uv_run_tests condvar_1", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_condvar_2(self, p):
        ret = p.sendCommand(
            "uv_run_tests condvar_2", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_condvar_3(self, p):
        ret = p.sendCommand(
            "uv_run_tests condvar_3", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_condvar_4(self, p):
        ret = p.sendCommand(
            "uv_run_tests condvar_4", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_condvar_5(self, p):
        ret = p.sendCommand(
            "uv_run_tests condvar_5", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_semaphore_1(self, p):
        ret = p.sendCommand(
            "uv_run_tests semaphore_1", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_semaphore_2(self, p):
        ret = p.sendCommand(
            "uv_run_tests semaphore_2", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_semaphore_3(self, p):
        ret = p.sendCommand(
            "uv_run_tests semaphore_3", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_timer(self, p):
        ret = p.sendCommand("uv_run_tests timer", ["not ok 1 -", "ok 1 -"], timeout=10)
        assert ret == 1

    def test_timer_init(self, p):
        ret = p.sendCommand(
            "uv_run_tests timer_init", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_timer_again(self, p):
        ret = p.sendCommand(
            "uv_run_tests timer_again", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_timer_start_twice(self, p):
        ret = p.sendCommand(
            "uv_run_tests timer_start_twice", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_timer_order(self, p):
        ret = p.sendCommand(
            "uv_run_tests timer_order", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_timer_huge_timeout(self, p):
        ret = p.sendCommand(
            "uv_run_tests timer_huge_timeout", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_timer_huge_repeat(self, p):
        ret = p.sendCommand(
            "uv_run_tests timer_huge_repeat", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_timer_run_once(self, p):
        ret = p.sendCommand(
            "uv_run_tests timer_run_once", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_timer_from_check(self, p):
        ret = p.sendCommand(
            "uv_run_tests timer_from_check", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_timer_is_closing(self, p):
        ret = p.sendCommand(
            "uv_run_tests timer_is_closing", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_timer_null_callback(self, p):
        ret = p.sendCommand(
            "uv_run_tests timer_null_callback", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_timer_early_check(self, p):
        ret = p.sendCommand(
            "uv_run_tests timer_early_check", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_loop_handles(self, p):
        ret = p.sendCommand(
            "uv_run_tests loop_handles", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_walk_handles(self, p):
        ret = p.sendCommand(
            "uv_run_tests walk_handles", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_active(self, p):
        ret = p.sendCommand("uv_run_tests active", ["not ok 1 -", "ok 1 -"], timeout=10)
        assert ret == 1

    def test_embed(self, p):
        if p.board in ["sim"]:
            pytest.skip("unsupported at %s" % p.board)
        ret = p.sendCommand("uv_run_tests embed", ["not ok 1 -", "ok 1 -"], timeout=10)
        assert ret == 1

    @pytest.mark.skip(reason="VELAPLATFO-6346")
    def test_async(self, p):
        if p.ci:
            pytest.skip("unsupported at %s" % p.board)
        if p.board in ["sim", "vela"]:
            pytest.skip("unsupported at %s" % p.board)
        ret = p.sendCommand("uv_run_tests async", ["not ok 1 -", "ok 1 -"], timeout=10)
        assert ret == 1

    def test_async_null_cb(self, p):
        if p.ci:
            pytest.skip("unsupported at %s" % p.board)
        if p.board in ["sim", "vela"]:
            pytest.skip("unsupported at %s" % p.board)
        ret = p.sendCommand(
            "uv_run_tests async_null_cb", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_homedir(self, p):
        ret = p.sendCommand(
            "uv_run_tests homedir", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_tmpdir(self, p):
        ret = p.sendCommand("uv_run_tests tmpdir", ["not ok 1 -", "ok 1 -"], timeout=10)
        assert ret == 1

    def test_hrtime(self, p):
        ret = p.sendCommand("uv_run_tests hrtime", ["not ok 1 -", "ok 1 -"], timeout=25)
        assert ret == 1

    def test_gettimeofday(self, p):
        ret = p.sendCommand(
            "uv_run_tests gettimeofday", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_poll_oob(self, p):
        if p.board in ["sim"]:
            pytest.skip("unsupported at %s" % p.board)
        ret = p.sendCommand(
            "uv_run_tests poll_oob", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_threadpool_queue_work_simple(self, p):
        ret = p.sendCommand(
            "uv_run_tests threadpool_queue_work_simple",
            ["not ok 1 -", "ok 1 -"],
            timeout=10,
        )
        assert ret == 1

    def test_threadpool_queue_work_einval(self, p):
        ret = p.sendCommand(
            "uv_run_tests threadpool_queue_work_einval",
            ["not ok 1 -", "ok 1 -"],
            timeout=10,
        )
        assert ret == 1

    def test_threadpool_cancel_getnameinfo(self, p):
        ret = p.sendCommand(
            "uv_run_tests threadpool_cancel_getnameinfo",
            ["not ok 1 -", "ok 1 -"],
            timeout=10,
        )
        assert ret == 1

    def test_threadpool_cancel_random(self, p):
        ret = p.sendCommand(
            "uv_run_tests threadpool_cancel_random",
            ["not ok 1 -", "ok 1 -"],
            timeout=10,
        )
        assert ret == 1

    def test_threadpool_cancel_work(self, p):
        ret = p.sendCommand(
            "uv_run_tests threadpool_cancel_work", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_threadpool_cancel_single(self, p):
        ret = p.sendCommand(
            "uv_run_tests threadpool_cancel_single",
            ["not ok 1 -", "ok 1 -"],
            timeout=10,
        )
        assert ret == 1

    def test_thread_local_storage(self, p):
        ret = p.sendCommand(
            "uv_run_tests thread_local_storage", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_thread_stack_size(self, p):
        ret = p.sendCommand(
            "uv_run_tests thread_stack_size", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_thread_mutex(self, p):
        ret = p.sendCommand(
            "uv_run_tests thread_mutex", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_thread_mutex_recursive(self, p):
        ret = p.sendCommand(
            "uv_run_tests thread_mutex_recursive", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_thread_rwlock(self, p):
        ret = p.sendCommand(
            "uv_run_tests thread_rwlock", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_thread_rwlock_trylock(self, p):
        ret = p.sendCommand(
            "uv_run_tests thread_rwlock_trylock", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_thread_create(self, p):
        ret = p.sendCommand(
            "uv_run_tests thread_create", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_thread_equal(self, p):
        ret = p.sendCommand(
            "uv_run_tests thread_equal", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_queue_foreach_delete(self, p):
        ret = p.sendCommand(
            "uv_run_tests queue_foreach_delete", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_random_async(self, p):
        if p.ci:
            pytest.skip("unsupported at %s" % p.board)
        ret = p.sendCommand(
            "uv_run_tests random_async", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_random_sync(self, p):
        if p.ci:
            pytest.skip("unsupported at %s" % p.board)
        ret = p.sendCommand(
            "uv_run_tests random_sync", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_handle_type_name(self, p):
        ret = p.sendCommand(
            "uv_run_tests handle_type_name", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_req_type_name(self, p):
        ret = p.sendCommand(
            "uv_run_tests req_type_name", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_utf8_decode1(self, p):
        ret = p.sendCommand(
            "uv_run_tests utf8_decode1", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1

    def test_utf8_decode1_overrun(self, p):
        ret = p.sendCommand(
            "uv_run_tests utf8_decode1_overrun", ["not ok 1 -", "ok 1 -"], timeout=10
        )
        assert ret == 1
