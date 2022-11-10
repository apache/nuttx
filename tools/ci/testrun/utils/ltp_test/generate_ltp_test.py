#!/usr/bin/python3
# encoding: utf-8
import argparse
import json
import os
import sys
import time

sys.path.append("../..")
print(sys.path)
from utils.common import findFile, readBuildInList


def generate_ltp_test(path, board, logpath, core):

    log_path = os.path.join(logpath, board, core)
    if not os.path.exists(log_path):
        os.makedirs(log_path)

    today = time.strftime("%Y%m%d", time.localtime())
    # get case list from buildin_list.h and write to open_posix_list_20200921.txt
    readBuildInList(path, log_path, board, core)
    # get test suite list from log path
    tsFile = findFile(log_path, "_list_%s.txt" % today, board, match=False)

    # get case list from ts file
    with open(tsFile[0], "r") as fr:
        cList = fr.readlines()

    # Parse json data
    with open("expect.json") as f:
        data = json.load(f)
    # get skip case list from data
    skip_list = []
    for skip in ["crash", "fail"]:
        if "common" in data[skip][board]:
            skip_list += [
                *data[skip]["common"],
                *data[skip][board]["common"],
                *data[skip][board][core],
            ]
        else:
            skip_list += [*data[skip]["common"], *data[skip][board]]

    count = 0
    absPath = os.path.abspath(__file__)
    with open(
        "%s/../../script/test_open_posix/test_openposix_%s.py"
        % (os.path.dirname(absPath), core),
        "w+",
    ) as fw:
        fw.writelines(
            [
                "#!/usr/bin/python3\n",
                "# encoding: utf-8\n",
                "import pytest\n\n\n",
                "pytestmark = [pytest.mark.disable_autouse]\n\n\n",
            ]
        )
        for case in cList:
            case = case.strip("\n")
            expect = None
            # skip crash case
            if case in skip_list:
                continue
            # get case expect from data
            for item in data:
                if case in data[item]:
                    expect = "'%s'" % item
                    continue
            # common expect
            if not expect:
                expect = ["PASSED", "passed", "Passed", "PASS"]

            fw.writelines(
                [
                    "def test_%s(p):\n" % case,
                    "\tret = p.sendCommand('%s', %s, 30)\n" % (case, expect),
                    "\tp.sendCommand('echo $?', '0')\n",
                    "\tassert ret >= 0\n\n",
                ]
            )
            count += 1

    print("The sum of ltp cases: %s" % len(cList))
    print("The sum of ltp skip cases: %s" % len(skip_list))
    print("The sum of ltp run cases: %s" % count)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p", dest="caselist_path", default="", help="specify buildin_list.h path"
    )
    parser.add_argument("-l", dest="log_path", default="", help="specify log path")
    parser.add_argument("-b", dest="board", default="", help="specify board")
    parser.add_argument("-c", dest="core", default="", help="specify core")
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    args = parse_args()
    path = args.caselist_path
    log_path = args.log_path
    board = args.board
    core = args.core

    # python3 generate_ltp_test.py -b best1600_ep -c ap
    # -l <logpath> -p <the dirpath of buildin_list>
    generate_ltp_test(path, board, log_path, core)
