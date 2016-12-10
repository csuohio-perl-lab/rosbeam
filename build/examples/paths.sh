##### Utility functions #####
log_start()
{
  START_TIME="${1}"
}
log_message()
{
  local now="$(date +%s)"
  local delta="$(expr ${now} - ${START_TIME})"
  echo
  date
  echo "===> (+${delta})" "$*" | tee /dev/kmsg
}

pci_device() {
  pci_device_count=0 # this will only share with the caller
  local status=1
  for device in /sys/bus/pci/devices/*; do
    if [ "$(cat ${device}/vendor)" = "${1}" ] && [ "$(cat ${device}/device)" = "${2}" ]; then
      pci_device_count=$((pci_device_count + 1))
      status=0
    fi
  done

  return $status
}
wait_for_site_survey() {
  while ! $(grep -q "^[[:space:]]*[0-9]\+:[[:space:]]*0100007F:1BA8 00000000:0000 0A" /proc/net/tcp); do
    # Above corresponds to 127.0.0.1:7080 in listening state
    sleep 0.1
  done
}

##### RPD software paths #####
export RPD_ROOT="/home/st/sw-dev/install"
export RPD_RESOURCES="${RPD_ROOT}/resources"
export RPD_SCRIPTS="${RPD_ROOT}/scripts"
export RPD_BIN="${RPD_ROOT}/bin"
export RPD_ENV="${RPD_ROOT}/env"
export ST_PROTO_ROOT="${RPD_ROOT}/proto_root"
export PYTHONPATH="${RPD_ROOT}/pyshared"
export PROTOCOL_BUFFER_COMPILER="${RPD_BIN}/proto_compiler"

##### Texclient paths #####
export STRESTART=/var/st/restart
if [ -f /store/config/PERMANENT_LOG_ALL ]; then
  export STLOG=/var/log_permanent/st/
  export ROAMLOG=/var/log_permanent/roam/
else
  export STLOG=/var/log/st/
  export ROAMLOG=/var/log/roam/
fi
export STLOG_PERMANENT=/var/log_permanent/st/
export STACTIVE=/etc/st-active-release
export SYSTEM_CHECK=/var/lib/st/system_check
export TO_TEXCLIENT=/var/lib/st/properties/to_texclient
export FROM_TEXCLIENT=/var/lib/st/properties/from_texclient
export PERSISTENT_PROPERTIES=/var/lib/st/properties/persistent_private
export PERMANENT_PROPERTIES=/store/config/texclient_permanent

export ST_RESTART_ID_FILE=/var/lib/st/st_restart_id
export BOOT_ID_FILE=/var/st/boot_id

##### Platform detection #####
export BEAM_PLATFORM="pro"
if pci_device 0x10ec 0x8168; then # Realtek ethernet controller
  export BEAM_PLATFORM="cleveland"
elif pci_device 0x80ee 0xcafe; then # VirtualBox
  export BEAM_PLATFORM="vm"
fi

##### Add Beam shared object path #####
export LD_LIBRARY_PATH="${RPD_BIN}:${LD_LIBRARY_PATH}"

##### Set path #####
export PATH="${RPD_SCRIPTS}:${RPD_BIN}:${PATH}"

#### Overlay #####
if [ -e /store/config/wifi_dev_mode ]; then
  if [ ! -e /var/st/password_written ]; then
    >/var/st/password_written
    sed -i '/PasswordAuthentication/d' /etc/ssh/sshd_config
    sed -i '/^ListenAddress/d' /etc/ssh/sshd_config
    sed -i "s#'usermod'[^]]*#'usermod', '-p' '\$6\$PasswordIsst\$DIRmCphmakWh8VunZU/roAkHBpBs3ArsofO85taMcp77Fp7b3fZ3wy9W5yTvT/CXA96Jbsw9okC4WStHwqC.T0', 'st'#" /home/st/sw-dev/install/scripts/rpd_setup.py
    sed -i 's#"${RPD_BIN}/texclient"#LD_PRELOAD=libtexclient-inject.so "${RPD_BIN}/texclient"#' /home/st/sw-dev/install/scripts/texspawner
    sed -i '/-A INPUT -j DROP/d' /home/st/sw-dev/install/scripts/setup_firewall.py
    sed -i '/-A FORWARD -j DROP/d' /home/st/sw-dev/install/scripts/setup_firewall.py
    #sleep 20 && iptables -t mangle -I INPUT -p icmp -m icmp --icmp-type ping -m string --algo bm --from 28 --to 128 --string "WATCHDOG" -m hashlimit --hashlimit-mode dstport --hashlimit-upto 1/second --hashlimit-htable-expire 200 --hashlimit-htable-gcinterval 100 --hashlimit-name watchdog -j ACCEPT &
  fi
fi
