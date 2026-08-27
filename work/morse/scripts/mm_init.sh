#!/bin/sh

/morse/scripts/S99morse restart 2>&1 > /dev/null

mcFile="/morse/configs/morse.conf"
if grep -q "bridge=0" $mcFile; then

    if grep -q "eth_ip_method=1" $mcFile; then
        . /home/root/udhcpd-eth.sh
    fi

    if grep -q "halow_ip_method=1" $mcFile; then
        . /home/root/udhcpd-wlan.sh
    fi
fi

if grep -q "eth_iface=eth0" $mcFile; then
    ifconfig eth0 up
elif grep -q "eth_iface=eth1" $mcFile; then
    ifconfig eth1 up
fi
