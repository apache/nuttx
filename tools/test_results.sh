#!/bin/bash -ex

function start_test_report {
  test_suites=$1
  test_report_file=${ARTIFACTDIR}/test/$test_suites.xml
  mkdir -p $(dirname $test_report_file)
  echo "Reporting to $test_report_file"

  # First provide the document encoding
  echo '<?xml version="1.0" encoding="UTF-8"?>' > $test_report_file
  start_test_suites $test_suites
}

function start_test_suites {
  # Attributes that will be set on the test suites:
  #
  #  name        Name of the test suites
  #  tests       Total number of tests in the suites
  #  failures    Total number of failed tests in the suites
  #  errors      Total number of errored tests in the suites
  #  skipped     Total number of skipped tests in the suites
  #  assertions  Total number of assertions for all tests in the suites
  #  time        Aggregated time of all tests in the suites in seconds
  #  timestamp   Date and time of when the suites were executed (in ISO 8601 format)

  # Reset attributes that will be used to fill in on finalize
  test_suites_name=$1
  test_suites_tests=0
  test_suites_failures=0
  test_suites_errors=0
  test_suites_skipped=0
  test_suites_assertions=0
  test_suites_start_time=`date +%s`
  test_suites_timestamp=`date --iso-8601=seconds`

  # Placeholder will be replaced when attributes are known
  echo '<TEST_SUITES_NODE>' >> $test_report_file
  test_suites_node_start_line=`wc -l < $test_report_file`
}

function end_test_suites {
  test_suites_end_time=`date +%s`
  sed -i "$test_suites_node_start_line""s|.*|\
<testsuites\
 name=\""$test_suites_name"\"\
 tests=\""$test_suites_tests"\"\
 failures=\""$test_suites_failures"\"\
 errors=\""$test_suites_errors"\"\
 skipped=\""$test_suites_skipped"\"\
 assertions=\""$test_suites_assertions"\"\
 time=\""$(($test_suites_end_time-$test_suites_start_time))"\"\
 timestamp=\""$test_suites_timestamp"\">|" $test_report_file
  echo "</testsuites>" >> $test_report_file
  cat $test_report_file

}


function start_test_suite {
  # Attributes that will be set on the test suite:
  #
  #  name        Name of the test suite
  #  tests       Total number of tests in this suite
  #  failures    Total number of failed tests in this suite
  #  errors      Total number of errored tests in this suite
  #  skipped     Total number of skipped tests in this suite
  #  assertions  Total number of assertions for all tests in this suite
  #  time        Aggregated time of all tests in this suite in seconds
  #  timestamp   Date and time of when the test suite was executed (in ISO 8601 format)

  # Reset attributes that will be used to fill in on finalize
  test_suite_name=$1
  test_suite_tests=0
  test_suite_failures=0
  test_suite_errors=0
  test_suite_skipped=0
  test_suite_assertions=0
  test_suite_start_time=`date +%s`
  test_suite_timestamp=`date --iso-8601=seconds`

  # Placeholder will be replaced when attributes are known
  echo '    <TEST_SUITE_NODE>' >> $test_report_file
  test_suite_node_start_line=`wc -l < $test_report_file`
}

function end_test_suite {
  test_suite_end_time=`date +%s`
  sed -i "$test_suite_node_start_line""s|.*|\
    <testsuite\
 name=\""$test_suite_name"\"\
 tests=\""$test_suite_tests"\"\
 failures=\""$test_suite_failures"\"\
 errors=\""$test_suite_errors"\"\
 skipped=\""$test_suite_skipped"\"\
 assertions=\""$test_suite_assertions"\"\
 time=\""$(($test_suite_end_time-$test_suite_start_time))"\"\
 timestamp=\""$test_suite_timestamp"\">|" $test_report_file
  echo "    </testsuite>" >> $test_report_file
  # Increment counters for test suites container.
  test_suites_tests=$(($test_suites_tests+$test_suite_tests))
  test_suites_failures=$(($test_suites_failures+$test_suite_failures))
  test_suites_errors=$(($test_suites_errors+$test_suite_errors))
  test_suites_skipped=$(($test_suites_skipped+$test_suite_skipped))
  test_suites_assertions=$(($test_suites_assertions+$test_suite_assertions))
}

function start_test_case {
  # Attributes that will be set on the test case:
  #
  #  name        Name of the test case
  #  classname   Name of the test class
  #  tests       Total number of tests in this case
  #  failures    Total number of failed tests in this case
  #  errors      Total number of errored tests in this case
  #  skipped     Total number of skipped tests in this case
  #  assertions  Total number of assertions for all tests in this case
  #  time        Aggregated time of all tests in this case in seconds
  #  timestamp   Date and time of when the test case was executed (in ISO 8601 format)

  # Reset attributes that will be used to fill in on finalize
  test_case_name=$1
  test_case_classname=$test_suite_name
  test_case_tests=0
  test_case_failures=0
  test_case_errors=0
  test_case_skipped=0
  test_case_assertions=0
  test_case_start_time=`date +%s`
  test_case_timestamp=`date --iso-8601=seconds`

  # Placeholder will be replaced when attributes are known
  echo '        <TEST_CASE_NODE>' >> $test_report_file
  test_case_node_start_line=`wc -l < $test_report_file`
}

function end_test_case {
  test_case_end_time=`date +%s`
  sed -i "$test_case_node_start_line""s|.*|\
        <testcase\
 name=\""$test_case_name"\"\
 classname=\""$test_case_classname"\"\
 tests=\""$test_case_tests"\"\
 failures=\""$test_case_failures"\"\
 errors=\""$test_case_errors"\"\
 skipped=\""$test_case_skipped"\"\
 assertions=\""$test_case_assertions"\"\
 time=\""$(($test_case_end_time-$test_case_start_time))"\"\
 timestamp=\""$test_case_timestamp"\">|" $test_report_file
  echo "        </testcase>" >> $test_report_file

  # Increment counters for test suites container.
  test_suite_tests=$(($test_suite_tests+1))
  test_suite_failures=$(($test_suite_failures+$test_case_failures))
  test_suite_errors=$(($test_suite_errors+$test_case_errors))
  test_suite_skipped=$(($test_suite_skipped+$test_case_skipped))
  test_suite_assertions=$(($test_suite_assertions+$test_case_assertions))
}

function report_test_pass {
    test_case_assertions=$(($test_case_assertions+1))
}
function report_test_skip {
    test_case_skipped=$(($test_case_skipped+1))
    echo "            <skipped message=\"Test was skipped.\" />" >> $test_report_file
}
function report_test_error {
    test_case_assertions=$(($test_case_assertions+1))
    test_case_errors=$(($test_case_errors+1))
    local message="${1:-'Test errored see logs.'}"
    echo "            <error message=\"$message\" />" >> $test_report_file
}
function report_test_failure {
    test_case_assertions=$(($test_case_assertions+1))
    test_case_failures=$(($test_case_failures+1))
    local message="${1:-'Test failed see logs.'}"
    echo "            <failure message=\"$message\" />" >> $test_report_file
}


function demo {
  export ARTIFACTDIR=/tmp/testresults
  start_test_report a_board_config

  start_test_suite a_board_config_1
  start_test_case normalize
  report_test_pass
  end_test_case
  start_test_case build
  report_test_error
  end_test_case
  start_test_case test
  report_test_skip
  end_test_case
  end_test_suite

  start_test_suite a_board_config_2
  start_test_case normalize
  report_test_pass
  end_test_case
  start_test_case build
  report_test_error
  end_test_case
  start_test_case test
  report_test_pass
  report_test_failure
  report_test_failure
  report_test_error
  end_test_case
  end_test_suite

  end_test_suites

  junit2html $test_report_file
}

if [ "${BASH_SOURCE[0]}" -ef "$0" ]
then
  demo
fi
