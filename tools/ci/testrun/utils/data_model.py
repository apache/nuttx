############################################################################
# tools/ci/testrun/utils/data_model.py
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

import re
from datetime import datetime
from typing import Dict, List

"""
cmocka.json
"""

Passed = "Passed"
Failed = "Failed"
Unexecuted = "Unexecuted"


class CaseInfo:
    def __init__(self, test_suite_name, test_case_name, status, log=None):
        self.test_suite_name = test_suite_name
        # case name，e.g. "TestNuttxMm01"
        self.test_case_name = test_case_name
        # result: Passed or Failed
        self.status = status
        # log
        self.log: List = [""] if log is None else log


class SuiteInfo:
    def __init__(self, test_suite_name):
        # suite name, e.g. "NuttxMmTestSuites"
        self.test_suite_name = test_suite_name
        # all test cases in the current test suite
        self.test_cases: Dict[str, CaseInfo] = dict()
        # number of cases passed in the current test suites
        self.passed_count = 0
        # number of cases failed in the current test suites
        self.failed_count = 0
        # case run count
        self.run_count = 0
        # unexecuted count
        self.unexecuted_count = 0
        # number of cases in the current test suites
        self.cases_count = 0
        # suite run flag
        self.is_suite_run = False


class CmockaSummary:
    def __init__(self, duration=0):
        # number of all test suites
        self.total_suites_count = 0
        # all test cases number
        self.total_cases_count = 0
        # number of all passed cases
        self.total_passed_count = 0
        # number of all failed cases
        self.total_failed_count = 0
        # number of all unknown cases
        self.total_unexecuted_count = 0
        # duration
        self.duration = duration


class CmockaSingleCoreRecord:
    def __init__(self, lines, core="", board="", log="", duration=0):
        # create time
        self.create_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        # core
        self.core = "" if core is None else core
        # board
        self.board = "" if board is None else board
        # cmocka info
        self.test_suites: Dict[str, SuiteInfo] = dict()
        # summary
        self.summary = CmockaSummary(duration)
        # log path
        self.log = "" if log is None else log
        # bad_case
        self.bad_case_tip = ""

        suite_pattern = r"\] (?P<test_suite_name>[a-zA-Z]*TestSuites)"
        case_pattern = r"\]\s+(?P<test_case_name>TestNuttx\w+)"
        lines_iter = iter(lines)

        current_suite = None
        while True:
            try:
                line = next(lines_iter)
                if (suite_match := re.search(suite_pattern, line)) is not None:
                    current_suite = suite_match.group("test_suite_name")

                elif (
                    current_suite is not None
                    and (case_match := re.search(case_pattern, line)) is not None
                ):
                    current_case = case_match.group("test_case_name")
                    self.append(CaseInfo(current_suite, current_case, Unexecuted))

            except StopIteration:
                break

    def append(self, object: CaseInfo):
        suite: SuiteInfo = self.test_suites.get(object.test_suite_name)
        if suite is None:
            suite = SuiteInfo(object.test_suite_name)
            self.test_suites.update({object.test_suite_name: suite})
        suite.test_cases.update({object.test_case_name: object})

        passed_count = 0
        failed_count = 0
        unexecuted_count = 0

        test_case: CaseInfo
        for test_case in suite.test_cases.values():
            if test_case.status == Passed:
                passed_count += 1
            elif test_case.status == Failed:
                failed_count += 1
            else:
                unexecuted_count += 1
        suite.passed_count = passed_count
        suite.failed_count = failed_count
        suite.unexecuted_count = unexecuted_count
        suite.run_count = passed_count + failed_count
        suite.cases_count = passed_count + failed_count + unexecuted_count
        if passed_count + failed_count != 0:
            suite.is_suite_run = True

        total_passed_count = 0
        total_failed_count = 0
        total_unexecuted_count = 0
        total_cases_count = 0

        suite: SuiteInfo
        for suite in self.test_suites.values():
            total_passed_count += suite.passed_count
            total_failed_count += suite.failed_count
            total_unexecuted_count += suite.unexecuted_count
            total_cases_count += suite.cases_count

        self.summary.total_passed_count = total_passed_count
        self.summary.total_failed_count = total_failed_count
        self.summary.total_unexecuted_count = total_unexecuted_count
        self.summary.total_cases_count = total_cases_count
        self.summary.total_suites_count = len(self.test_suites)

    def process(self, lines, err_code):
        # regular expression
        suite_start_pattern = r"\] (?P<test_suite_name>[a-zA-Z]*TestSuites): Running (?P<cases_count>\d+) test\(s\)"
        case_run_pattern = r"\[\s+RUN\s+\] (?P<test_case_name>TestNuttx\w+)"
        case_pass_pattern = r"\[\s+OK\s+\] (?P<test_case_name>TestNuttx\w+)"
        case_fail_pattern = r"\[\s+FAILED\s+\] (?P<test_case_name>TestNuttx\w+)"

        lines_iter = iter(lines)
        line = next(lines_iter)
        while True:
            try:
                interrupt_flag = False
                # matching new test suites
                if (
                    suite_start_match := re.search(suite_start_pattern, line)
                ) is not None:
                    test_suite_name = suite_start_match.group("test_suite_name")
                    cases_count = int(suite_start_match.group("cases_count"))
                    suite_end_pattern = r"{}: {} test(s) run.".format(
                        test_suite_name, cases_count
                    )

                    line = next(lines_iter)
                    while True:
                        if (
                            case_run_match := re.search(case_run_pattern, line)
                        ) is not None:
                            test_case_name = case_run_match.group("test_case_name")
                            log = [line]

                            while True:
                                try:
                                    line = next(lines_iter)
                                    log.append(line)
                                except StopIteration:
                                    self.append(
                                        CaseInfo(
                                            test_suite_name, test_case_name, Failed, log
                                        )
                                    )
                                    if err_code == -3:
                                        self.bad_case_tip = f"This case was not executed, \
                                             because crash occurred after running '{test_case_name}'."
                                    elif err_code == -4:
                                        self.bad_case_tip = f"This case was not executed, \
                                             because no response for a long time after running '{test_case_name}'."
                                    elif err_code == -1:
                                        self.bad_case_tip = f"This case was not executed, \
                                             because the maximum waiting time has been exceeded \
                                             while running '{test_case_name}'."
                                    else:
                                        self.bad_case_tip = "This case was not executed due to unknown reasons."
                                    raise StopIteration

                                if re.search(case_pass_pattern, line) is not None:
                                    self.append(
                                        CaseInfo(
                                            test_suite_name, test_case_name, Passed, log
                                        )
                                    )
                                    break

                                elif re.search(case_fail_pattern, line) is not None:
                                    self.append(
                                        CaseInfo(
                                            test_suite_name, test_case_name, Failed, log
                                        )
                                    )
                                    break

                                elif re.search(suite_start_pattern, line) is not None:
                                    self.append(
                                        CaseInfo(
                                            test_suite_name, test_case_name, Failed, log
                                        )
                                    )
                                    interrupt_flag = True
                                    break

                        elif suite_end_pattern in line:
                            break

                        if interrupt_flag:
                            break

                        line = next(lines_iter)

                    if interrupt_flag:
                        continue
                line = next(lines_iter)
            except StopIteration:
                break
