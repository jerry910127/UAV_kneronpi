#!/bin/bash

NET_DOMAIN=100.10.1
NET_IP=${NET_DOMAIN}.111
GW_IP=${NET_DOMAIN}.5

# Shutdown existing process
sudo killall hostapd_s1g
if [ $? -ne 0 ]; then
    echo "Failed to kill hostapd_s1g or it was not running."
else
	echo "hostapd_s1g process killed successfully."
fi
sudo killall wpa_supplicant_s1g
if [ $? -ne 0 ]; then
    echo "Failed to kill wpa_supplicant_s1g or it was not running."
else
    echo "wpa_supplicant_s1g process killed successfully."
fi

# Remove old log file if it exists
if [ -f /var/log/wpa_supplicant_s1g.log ]; then
    sudo rm /var/log/wpa_supplicant_s1g.log
    if [ $? -ne 0 ]; then
        echo "Failed to remove old log file or it did not exist."
        exit 1
    else
        echo "Old log file removed successfully."
    fi
fi

# Run wpa_supplicant_s1g to start halow STA mode
sudo wpa_supplicant_s1g -d -D nl80211 -i wlan0 -c /work/morse/wpa_supplicant_s1g.Alltek.conf -f /var/log/wpa_supplicant_s1g.log -B
if [ $? -ne 0 ]; then
    echo "Failed to start wpa_supplicant_s1g."
    exit 1
else
    echo 'wpa_supplicant_s1g started successfully. You can run "sudo iw dev wlan0 link" to check link status, or check the log for details.'
fi

# Set IP address for wlan0
sudo ifconfig wlan0 ${NET_IP}
if [ $? -ne 0 ]; then
    echo "Failed to set IP address for wlan0."
    exit 1
else
    echo "IP address for wlan0 set to ${NET_IP} successfully."
fi

# Add default gateway
sudo route add default gw ${GW_IP}
if [ $? -ne 0 ]; then
    echo "Failed to add default gateway."
    exit 1
else
    echo "Default gateway set to ${GW_IP} successfully."
fi

