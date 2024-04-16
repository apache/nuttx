#!/bin/bash

#****************************************************************************
# tools/simwifi/sim_wifi.sh
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
#****************************************************************************
#

NUTTX_BR_IF="nuttx0"
RUN_DIR="/var/run/simwifi"
LINK_DIR="/usr/bin"
CUR_DIR=""
DBG_LEVEL=1

WPA_PID_FILE="$RUN_DIR/wpa_supplicant.pid"
WPA_CONF_FILE="$RUN_DIR/wpa_supplicant.conf"
HOSTAPD_PID_FILE="$RUN_DIR/hostapd.pid"
HOSTAPD_CONF_FILE="$RUN_DIR/hostapd.conf"
DNSMASQ_PID="$RUN_DIR/dnsmasq.pid"
DNSMASQ_CONF="$RUN_DIR/dnsmasq.conf"
UDHCPC_PID="$RUN_DIR/udhcpc.pid"
UDHCPC_SCRIPT="/var/udhcpc.script"

LOG_FILE="$RUN_DIR/simwifi.log"
STATE_FILE="$RUN_DIR/simwifi.state"
DEFCONF_FILE="$RUN_DIR/simwifi.conf"

DHCP_CLIENT=$(which udhcpc)
DNSMASQ=$(which dnsmasq)
WPA_SUPPLICANT=$(which wpa_supplicant)
HOSTAPD=$(which hostapd)

# print the debug message
sw_dbg()
{
  [ $1 -ge $DBG_LEVEL ] && {
    echo "$2"
  }
}

# get var from the file
# $1:key $2:file
get_var()
{
  cat $2 | grep $1 | awk -F':' '{print $2}'
}

check_state()
{
  old_state=$(get_var state $STATE_FILE)
  sw_dbg 1 "new state:$1, old_state:$old_state"

  if [ "$1" = "$old_state" ]; then
    sw_dbg 1 "cur_state is $1"
    exit 0
  fi
}

# set simwifi state
set_state()
{
  state=$1
  wlan_if=$2
  br_if=$3
  wan_if=$4
  sw_dbg 1 "[set_state] new state:$1"

  if [ "$state" != "SW_INIT" -a "$state" != "SW_HWSIM" -a \
       "$state" != "SW_NET" -a "$state" != "SW_STA" -a \
       "$state" != "SW_AP" -a "$state" != "SW_STAAP" ]; then
    echo "no state: $1"
    exit -1
  fi

  sw_dbg 1 "state:$state wlan_if:$wlan_if, br_if:$br_if wan_if:$wan_if"

  echo "state:$state" > $STATE_FILE
  [ -n "$wlan_if" ] && echo "wlan:$wlan_if" >> $STATE_FILE
  [ -n "$br_if" ] && echo "br:$br_if" >> $STATE_FILE
  [ -n "$wan_if" ] && echo "wan:$wan_if" >> $STATE_FILE
}

# recover the simwifi state to SW_INIT

recovery_to_init()
{
  cur_state=$(get_var state $STATE_FILE)
  wlan_if=$(get_var wlan $STATE_FILE)
  br_if=$(get_var br $STATE_FILE)

  sw_dbg 1 "[recovery_to_init] cur_s:$cur_state"
  case $cur_state in
    SW_INIT) ;;
    SW_HWSIM) stop_hwsim;;
    SW_STAAP) stop_staap;;
    SW_NET) stop_net $wlan_if;;
    SW_STA) stop_sta;;
    SW_AP) stop_ap $wlan_if;;
    *)  set_state SW_INIT;;
  esac
}

# Get the absolute pathname of sim_wifi.sh

get_script_path()
{
  SOURCE=$1

  while [ -h "$SOURCE" ]; do
    SOURCE=$(readlink $SOURCE)
  done

  CUR_DIR=$(cd $(dirname $SOURCE) && pwd)
}

# Copy the configure file to the $RUN_DIR

init_env()
{
  sw_dbg 1 "init env"

  [ -f "$STATE_FILE" ] && {
    check_hwsim_mode
    recovery_to_init
  }

  mkdir -p $RUN_DIR
  touch $STATE_FILE

  if [ "$1" = "hwsim" ]; then
    cp -fr $CUR_DIR/hostapd_hwsim.conf $HOSTAPD_CONF_FILE
    cp -fr $CUR_DIR/wpa_supplicant_hwsim.conf $WPA_CONF_FILE
  else
    cp -fr $CUR_DIR/hostapd.conf $HOSTAPD_CONF_FILE
    cp -fr $CUR_DIR/wpa_supplicant.conf $WPA_CONF_FILE
  fi

  cp -fr $CUR_DIR/dnsmasq.conf $DNSMASQ_CONF
  cp -fr $CUR_DIR/udhcpc.script $UDHCPC_SCRIPT
  chmod +x $UDHCPC_SCRIPT
}

# Rename the interface name

rename_ifdev()
{
  old_name=$1
  new_name=$2

  ifconfig $old_name down && ip link set $old_name name $new_name
  ifconfig $new_name up
}

kill_service()
{
  service_name=$(basename $1 .pid)
  sw_dbg 1 "kill $service_name"
  if [ -f "$1" ]; then
    pid=$(cat $1)
    kill -9 $pid
    rm $1
  else
    sw_dbg 1 "$1 isn't existed."
    killall $service_name
  fi
}

stop_wpa()
{
  kill_service $WPA_PID_FILE
}

stop_hostapd()
{
  kill_service $HOSTAPD_PID_FILE
}

start_hostapd()
{
  sw_dbg 1 "start ap on $1"

  #Waiting 1s. If not, the hostapd starting maybe fail on switching mode.

  sleep 1
  $HOSTAPD -B -i$1 -P $HOSTAPD_PID_FILE $HOSTAPD_CONF_FILE -t &>>$LOG_FILE
}

start_wpa()
{
  sw_dbg 1 "start sta on $1"

  #Waiting 1s. If not, the wap_supplicant starting maybe fail on switching mode.

  sleep 1
  $WPA_SUPPLICANT -B -c $WPA_CONF_FILE -i$1 -P $WPA_PID_FILE  &>>$LOG_FILE
}

start_udhcpc()
{
  sw_dbg 1 "start dhcp client on $1"

  [ -n "$2" ] && script_opt=" -s $UDHCPC_SCRIPT"

  $DHCP_CLIENT -i $1 -p $UDHCPC_PID $script_opt &>>$LOG_FILE &
}

stop_udhcpc()
{
  kill_service $UDHCPC_PID
}

start_bridge()
{
  sw_dbg 1 "start bridge to $1"
  $CUR_DIR/../simhostroute.sh $1 on &>>$LOG_FILE
}

stop_bridge()
{
  sw_dbg 1 "stop bridge to $1"
  sw_dbg 1 "Warning: The $NUTTX_BR_IF will be deleted!"
  $CUR_DIR/../simhostroute.sh $1 off &>>$LOG_FILE
}

start_dhcp_server()
{
  sw_dbg 1 "start dhcp server on $1"
  dbg_option=$($DNSMASQ --help|grep log-debug | awk '{print $1}')

  $DNSMASQ -i$1 -C $DNSMASQ_CONF  $dbg_option -x $DNSMASQ_PID &>>$LOG_FILE
}

stop_dhcp_server()
{
  kill_service $DNSMASQ_PID
}

check_hwsim_mode()
{
  cur_state=$(get_var state $STATE_FILE)

  sw_dbg 1 "[check_hwsim_mode]state: $cur_state"
  if [ "$cur_state" = "SW_HWSIM" ]; then
    sw_dbg 1 "cur_state is hwsim mode. \
          Don't set the sta/ap mode."
    exit 0
  fi
}

start_hwsim()
{
  sta_if=${1:-wlan0}
  ap_if=${2:-wlan1}

  sw_dbg 1 "start hwsim on $sta_if $ap_if"

  init_env hwsim
  start_hostapd $ap_if
  start_wpa $sta_if
  start_dhcp_server $NUTTX_BR_IF

  set_state SW_HWSIM
}

stop_hwsim()
{
  stop_wpa
  stop_hostapd

  stop_dhcp_server

  set_state SW_INIT
}

start_net()
{
  init_env
  start_wpa $1
  start_udhcpc $1
  start_bridge $1
  start_dhcp_server $NUTTX_BR_IF

  set_state SW_NET $1 $NUTTX_BR_IF $1
}

# Warning:The function will delete nuttx0

stop_net()
{
  stop_wpa
  stop_udhcpc
  stop_bridge $1
  stop_dhcp_server

  set_state SW_INIT
}

start_sta()
{
  [ -z "$(ifconfig | grep "$NUTTX_BR_IF")" ] && {
    sw_dbg 1 "Please ensure that the $NUTTX_BR_IF is existed."
    exit -2
  }

  check_state SW_STA
  check_state SW_STAAP

  init_env
  start_wpa $1
  start_udhcpc $1 s
  start_dhcp_server $NUTTX_BR_IF

  set_state SW_STA $1 $NUTTX_BR_IF;

  exit 0
}

del_gw_wlan()
{
  wlan_if=$1
  router=$(ip route show | grep default | grep $wlan_if)

  [ -n "$router" ] && {
    ip route del $router
    sw_dbg 1 "del the default router on $wlan_if"
  }

  ifconfig $wlan_if 0.0.0.0
}

stop_sta()
{
  stop_wpa
  stop_udhcpc
  stop_dhcp_server

  #check and delete default on wlan0

  wlan_if=$(get_var wlan $STATE_FILE)
  del_gw_wlan $wlan_if

  set_state SW_INIT
}

start_ap()
{
  [ -z "$1" ] && {
    echo "Missing parameter wlan interface."
    exit -1
  }

  [ -z "$2" ] && {
    [ -z "$(ifconfig | grep "$NUTTX_BR_IF")" ] && {
      echo "Missing parameter wan interface."
      echo "Please ensure that the $NUTTX_BR_IF is existed."
      exit -2
    }

    wan_if=""
  } || {
    wan_if=$2
  }

  check_state SW_AP
  check_state SW_STAAP

  init_env
  start_hostapd $1

  # nuttx0 doesn't exist and wan_if is configured.

  [ -z "$(ifconfig | grep "$NUTTX_BR_IF")" -a -n "$wan_if" ] && {
    start_bridge $wan_if
  }

  ip link set dev $1 master $NUTTX_BR_IF
  start_dhcp_server $NUTTX_BR_IF

  set_state SW_AP $1 $wan_if;

  exit 0
}

stop_ap()
{
  stop_hostapd
  [ -z "$2" ] || stop_bridge $2

  sw_dbg 1 "stop_ap $1"
  ip link set dev $1 nomaster
  stop_dhcp_server

  set_state SW_INIT
}

check_ifname()
{
  [ -z "$(ifconfig | grep "$1")" ] && {
    echo "The $1 does not exist."
    exit -1
  }
}

start_staap()
{
  sta_if=${1:-wlan0}
  ap_if=${2:-wlan1}
  sw_dbg 1 "start staap $sta_if $ap_if"

  for i in $sta_if $ap_if $NUTTX_BR_IF; do
    check_ifname $i
  done

  check_state SW_STAAP

  init_env
  start_hostapd $ap_if

  sleep 1
  start_wpa $sta_if

  ip link set dev $ap_if master $NUTTX_BR_IF

  start_udhcpc $sta_if
  start_dhcp_server $NUTTX_BR_IF

  set_state SW_STAAP "$sta_if,$ap_if" $NUTTX_BR_IF
}

stop_staap()
{
  stop_wpa
  stop_hostapd

  stop_udhcpc
  stop_dhcp_server
  ip link set dev wlan1 nomaster

  set_state SW_INIT
}

show_process()
{
  ps -ef | grep "$1 " | grep -v grep
}

show_status()
{
  #1.env conf
  [ -d "$RUN_DIR" ] && {
    echo "$RUN_DIR"
    ls $RUN_DIR
  }

  #2. key services
  echo -e "\nservices list"
  for i in wpa_supplicant hostapd dnsmasq udhcpc; do
    show_process $i
  done

  #3. bridge nuttx0 info
  [ -n "$(ifconfig | grep $NUTTX_BR_IF)" ] && {
    echo -e "\nbridge $NUTTX_BR_IF"
    ip link show master $NUTTX_BR_IF
  }

  #4.show DEFCONF_FILE
  echo -e "\ndefault config"
  cat $DEFCONF_FILE

  #5. show state
  echo ""
  cat $STATE_FILE

  #6. show router
  echo ""
  ip route show

}

# $1 is the default wan interface for start_sta
# $2 is the simwifi mode, (rnc/hwsim)

init()
{
  [ -z "$1" ] && {
    echo "Missing the default wan interface."
    exit -1
  }

  [ -z "$2" ] && {
    echo "Missing the simwifi mode."
    exit -2
  }

  init_env

  ln -s $CUR_DIR/sim_wifi.sh $LINK_DIR/sim_wifi.sh

  echo "defwan:$1" > $DEFCONF_FILE
  [ -n "$1" -a  -n "$(ifconfig | grep $1)" ] && start_bridge $1

  echo "mode:$2" >> $DEFCONF_FILE
  [ "$2" = "hwsim" ] &&  modprobe  mac80211_hwsim

  set_state SW_INIT  "" $NUTTX_BR_IF $1
}

clean()
{
  [ -z "$1" ] && {
    echo "Missing the default wan interface."
    exit -1
  }

  recovery_to_init

  rm $LINK_DIR/sim_wifi.sh

  cur_mode=$(get_var mode $DEFCONF_FILE)
  [ "$cur_mode" = "hwsim" ] &&  modprobe -r mac80211_hwsim

  echo "defwan:$1" > $DEFCONF_FILE
  [ -n "$1" -a  -n "$(ifconfig | grep $1)" ] && stop_bridge $1

  rm -fr $RUN_DIR
  rm -f $UDHCPC_SCRIPT
}

usage()
{
  echo "$(basename $SOURCE) (rename <old> <new> |"
  echo -e "\t init <wan> <mode> |clean <wan> |"
  echo -e "\t start_wpa <wlan0> |stop_wpa |"
  echo -e "\t start_hostapd <wlan0> |stop_hostapd |"
  echo -e "\t start_udhcpc <wlan0> |stop_udhcpc |"
  echo -e "\t start_dhcp <wlan0> |stop_dhcp |"
  echo -e "\t start_hwsim |stop_hwsim |up_hwsim |"
  echo -e "\t start_staap |stop_staap |"
  echo -e "\t start_net <wlan0> |stop_net <wlan0> |"
  echo -e "\t start_sta <wlan0> |stop_sta |"
  echo -e "\t start_ap <wlan0> [eth0] |stop_ap <wlan0> [eth0] |"
  echo -e "\t start_bridge <eth0> |stop_bridge <eth0> |"
  echo -e "\t show | help)"
}

# locate the directory of the sim_wifi.sh.

get_script_path $0

case $1 in
  init) init $2 $3;;
  clean) clean $2;;
  start_bridge) start_bridge $2;;
  stop_bridge) stop_bridge $2;;
  start_hwsim)  start_hwsim $2 $3;;
  stop_hwsim)   stop_hwsim;;
  up_hwsim) ifconfig hwsim0 up;;
  start_wpa) start_wpa $2;;
  stop_wpa) stop_wpa;;
  start_hostapd) start_hostapd $2;;
  stop_hostapd) stop_hostapd;;
  rename) rename_ifdev $2 $3;;
  start_udhcpc) start_udhcpc $2;;
  stop_udhcpc) stop_udhcpc;;
  start_dhcp) start_dhcp_server $2;;
  stop_dhcp) stop_dhcp_server;;
  start_net) start_net $2;;
  stop_net) stop_net $2;;
  start_sta) start_sta $2;;
  stop_sta) stop_sta;;
  start_ap) start_ap $2 $3;;
  stop_ap) stop_ap $2 $3;;
  start_staap) start_staap;;
  stop_staap) stop_staap;;
  show) show_status;;
  help|*) usage;;
esac
