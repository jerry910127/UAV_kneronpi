#!/bin/sh

export STAGING_DIR=/usr

COUNTRY_CODE=US

if [ -z "${DRV_IF}" ];  then
    DRV_IF=SDIO
fi

#cd /work/morse/morse_driver

file_name=$( basename "$0" )
case ${file_name} in
build_morse_driver.sh)
    make MORSE_TRACE_PATH=`pwd` ARCH=arm SRCARCH=arm64 KERNEL_SRC=/work/KnWS/Kernel_v1.1.4 CONFIG_WLAN_VENDOR_MORSE=m CONFIG_MORSE_${DRV_IF}=y CONFIG_MORSE_USER_ACCESS=y CONFIG_MORSE_VENDOR_COMMAND=y
    ;;

build_hostap.sh)
    CFLAGS="-I $STAGING_DIR/include/" \
    LDFLAGS="-L $STAGING_DIR/lib/" \
    DESTDIR="$STAGING_DIR" \
    BINDIR=/usr/sbin \
    LIBS="-lnl-3 -lm -lpthread -lcrypto -lssl" \
    make MORSE_VERSION=rel_1_15_3_2025_Apr_16 -C hostapd/
    ;;

build_wpa_suppl.sh)
    CFLAGS="-I $STAGING_DIR/include/ -Wno-error=deprecated-declarations" \
    LDFLAGS="-L $STAGING_DIR/lib/" \
    DESTDIR="$STAGING_DIR" \
    BINDIR=/usr/sbin \
    LIBS="-lnl-3 -lm -lpthread -lcrypto -lssl" \
    make MORSE_VERSION=rel_1_15_3_2025_Apr_16 -C wpa_supplicant/
    ;;

build_morsectrl.sh)
    CFLAGS="-I $STAGING_DIR/include/libnl3/" \
    LDFLAGS="-L $STAGING_DIR/lib/" \
    make CONFIG_MORSE_TRANS_NL80211=1
    ;;

mnt_sdfs.sh)
    mount -o loop -t vfat /dev/mmcblk0 /mnt/sd
    ;;

umnt_sdfs.sh)
    umount /mnt/sd
    ;;

install_morse.sh)
    ETH0_MAC_SUFFIX=`cat /sys/class/net/eth0/address | cut -d: -f4-`

    #cd /work/morse/drivers
    cd /work/morse/drv_1.16.4	#ker_v1p1p4
    sudo insmod dot11ah.ko
    sudo insmod morse.ko country=${COUNTRY_CODE} macaddr_suffix=${ETH0_MAC_SUFFIX}
    ;;

run_wpa_suppl.sh)
    wpa_supplicant_s1g -t -D nl80211 -s -i wlan0 -c /work/morse/bin/wpa_supplicant-wlan0.conf -B

    CLI_IP_DOMAIN=192.168
    #CLI_IP_DOMAIN=100.10
    net_try=0
    while [ ${net_try} -lt 5 ];  do
        sleep 1
        net_st=`iw wlan0 link | grep SSID`
        if [ -n "${net_st}" ];  then
            echo "${net_st}"
            ifconfig wlan0 ${CLI_IP_DOMAIN}.1.111
            route add default gw ${CLI_IP_DOMAIN}.1.5
            break
        fi
        net_try=$((${net_try} + 1))
    done
    ;;
esac
