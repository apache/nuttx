#!/usr/bin/env bash

set -e
set -o xtrace

WD=$(cd $(dirname $0) && pwd)
WORKSPACE=$(cd $WD/../../../../../../../ && pwd -P)
nuttx=$WORKSPACE/nuttx
logs=${WD}/logs
BOARD=`echo $WD |awk -F '/' '{print $(NF-2)}'`

echo $WD
echo $WORKSPACE

config=$(basename $WD)
if [ "$BOARD" == "sim" ]; then
  target="sim"
  mark="common or ${BOARD}"
else
  if [ "${config:$((-2))}" == "64" ]; then
  BOARD="${BOARD}64"
  fi
  target="qemu"
  mark=$target
fi

core=$target
image=`find ${nuttx} -type f -name 'nuttx'`
path=${image%/*}
cd ${nuttx}/tools/ci/testrun/script
python3 -m pytest -m "${mark}" ./ -B ${BOARD} -P ${path} -L ${logs}/${BOARD}/${core} -R ${target} -C --json=${logs}/${BOARD}/${core}/pytest.json
ret="$?"

#clean
find ${nuttx}/tools/ci/testrun -name '__pycache__' |xargs rm -rf
find ${nuttx}/tools/ci/testrun -name '.pytest_cache' |xargs rm -rf
rm -rf ${logs}

echo $ret
exit $ret
