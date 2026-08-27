#!/bin/sh
#
# Copyright (C) 2021 Morse Micro Pty Ltd. All rights reserved.
#
# Starts Morse WiFi interface
#

# utility function: kill a process, waiting for up to 30s for it to terminate.
# Return 0 if the process dies, or does not exist.
# Otherwise return error code.
kill_wait()
{
    local names=$*
    local count=30

    for pid in $(pidof $names)
    do
        kill $pid &> /dev/null
    done

    while pidof $names &> /dev/null;
    do
        sleep 0.1
        let "count--"
        if [ $count -eq 0 ]
        then
            echo "$names failed to terminate normally, force quitting" >&2
            kill -9 $(pidof $names)
            return 1
        fi
    done
    return 0
}

# Usage: country_op_class_ch_validate COUNTRY OP_CLASS CHANNEL MODE
country_op_class_ch_validate()
{
    local DC_MIN="0.01"
    local DC_MAX="100.00"

    while IFS=, read -r CC BW CH L_OP G_OP FREQ DC_AP DC_STA; do
        num_test $BW
        if [ $? -eq 1 ]; then
            continue
        fi
        num_test $CH
        if [ $? -eq 1 ]; then
            continue
        fi
        num_test $L_OP
        if [ $? -eq 1 ]; then
            continue
        fi
        num_test $G_OP
        if [ $? -eq 1 ]; then
            continue
        fi

        float_test $DC_AP $DC_MIN $DC_MAX
        if [ $? -eq 1 ]; then
            continue
        fi
        float_test $DC_STA $DC_MIN $DC_MAX
        if [ $? -eq 1 ]; then
            continue
        fi

        if [ "$CC" == "$1" ] && [ $CH -eq "$3" ]; then
            if [ $L_OP -eq "$2" ] || [ $G_OP -eq "$2" ]; then
                HALOW_BW=$BW
                FREQ_C_MHZ=$FREQ

                case $4 in
                    AP|ap)
                        DUTY_CYCLE=$DC_AP
                        ;;
                    STA|sta)
                        DUTY_CYCLE=$DC_STA
                        ;;
                esac
            fi
        fi
    done < /morse/configs/channels.csv
}

# Usage: country_op_class_validate COUNTRY OP_CLASS MODE
country_op_class_validate()
{
    local DC_MIN="0.01"
    local DC_MAX="100.00"

    while IFS=, read -r CC BW CH L_OP G_OP FREQ DC_AP DC_STA; do
        num_test $BW
        if [ $? -eq 1 ]; then
            continue
        fi
        num_test $CH
        if [ $? -eq 1 ]; then
            continue
        fi
        num_test $L_OP
        if [ $? -eq 1 ]; then
            continue
        fi
        num_test $G_OP
        if [ $? -eq 1 ]; then
            continue
        fi

        float_test $DC_AP $DC_MIN $DC_MAX
        if [ $? -eq 1 ]; then
            continue
        fi
        float_test $DC_STA $DC_MIN $DC_MAX
        if [ $? -eq 1 ]; then
            continue
        fi

        if [ "$CC" == "$1" ]; then
            if [ $L_OP -eq "$2" ] || [ $G_OP -eq "$2" ]; then
                HALOW_BW=$BW

                # Assume the Duty Cycle is the same for all channels for the given country and
                # operating class
                case $3 in
                    AP|ap)
                        DUTY_CYCLE=$DC_AP
                        ;;
                    STA|sta)
                        DUTY_CYCLE=$DC_STA
                        ;;
                esac
            fi
        fi
    done < /morse/configs/channels.csv
}

# usage: num_test NUM
num_test()
{
    case $1 in
        ''|*[!0-9]*)
            return 1
            ;;
        *)
            ;;
    esac
    return 0
}

# usage: ip_test IP
ip_test()
{
    local ip=$1

    if expr "$ip" : '[0-9][0-9]*\.[0-9][0-9]*\.[0-9][0-9]*\.[0-9][0-9]*$' > /dev/null; then
        for i in 1 2 3 4; do
            if [ $(echo "$ip" | cut -d. -f$i) -gt 255 ]; then
                return 1
            fi
        done
        return 0
    else
        return 1
    fi
}

# usage: gen_gateway IP
gen_gateway()
{
    local i1=`echo $1 | cut -d'.' -f1`
    local i2=`echo $1 | cut -d'.' -f2`
    local i3=`echo $1 | cut -d'.' -f3`
    GATEWAY=$i1.$i2.$i3.1
}

# usage: gen_net IP NETMASK
gen_net()
{
    local i1=`echo $1 | cut -d'.' -f1`
    local i2=`echo $1 | cut -d'.' -f2`
    local i3=`echo $1 | cut -d'.' -f3`
    local i4=`echo $1 | cut -d'.' -f4`
    local m1=`echo $2 | cut -d'.' -f1`
    local m2=`echo $2 | cut -d'.' -f2`
    local m3=`echo $2 | cut -d'.' -f3`
    local m4=`echo $2 | cut -d'.' -f4`
    NET=$(( $i1 & $m1 )).$(( $i2 & $m2 )).$(( $i3 & $m3 )).$(( $i4 & $m4 ))
}

# usage: check_dhcp_range IP_ADDR RANGE_START RANGE_END
check_dhcp_range()
{
    local IP_ADDR=$1
    DHCP_RANGE_START=$2
    DHCP_RANGE_END=$3

    # Check range bounds are numbers
    num_test $DHCP_RANGE_START
    if [ $? -eq 1 ]; then
        DHCP_RANGE_START=1
        echo "MORSE INIT - DHCP start value not a number, using $DHCP_RANGE_START" | tee /dev/kmsg
    fi
    num_test $DHCP_RANGE_END
    if [ $? -eq 1 ]; then
        DHCP_RANGE_END=254
        echo "MORSE INIT - DHCP end value not a number, using $DHCP_RANGE_END" | tee /dev/kmsg
    fi

    # Reorder if DHCP_RANGE_START is larger than DHCP_RANGE_END
    if [ $DHCP_RANGE_START -gt $DHCP_RANGE_END ]; then
        local TMP_SWAP=$DHCP_RANGE_START
        echo "MORSE INIT - DHCP start cannot be higher than DHCP end, swapping values" | tee /dev/kmsg
        DHCP_RANGE_START=$DHCP_RANGE_END
        DHCP_RANGE_END=$TMP_SWAP
    fi

    # Check that range being served doesn't include assigned IP
    local i1=`echo $IP_ADDR | cut -d'.' -f1`
    local i2=`echo $IP_ADDR | cut -d'.' -f2`
    local i3=`echo $IP_ADDR | cut -d'.' -f3`
    local i4=`echo $IP_ADDR | cut -d'.' -f4`
    if [ $i4 -ge $DHCP_RANGE_START -a $i4 -le $DHCP_RANGE_END ]; then
        if [ $i4 -lt 128 ]; then
            DHCP_RANGE_START=$(( $i4 + 1 ))
            echo "MORSE INIT - Assigned IP inside given DHCP range, changing DHCP start to $DHCP_RANGE_START" | tee /dev/kmsg
        else
            DHCP_RANGE_END=$(( $i4 - 1 ))
            echo "MORSE INIT - Assigned IP inside given DHCP range, changing DHCP end to $DHCP_RANGE_END" | tee /dev/kmsg
        fi
    fi
    DHCP_RANGE_START=$i1.$i2.$i3.$DHCP_RANGE_START
    DHCP_RANGE_END=$i1.$i2.$i3.$DHCP_RANGE_END
}


# usage: check_float VALUE MIN MAX
float_test()
{
    local val=$1
    local min=$2
    local max=$3

    if expr "$val" : '^[0-9]*[.]*[0-9])*$' > /dev/null; then
        if [ `echo $val '>=' $min | bc -l` -eq 1 ]; then
            if [ `echo $val '<=' $max | bc -l` -eq 1 ]; then
                return 0
            fi
        fi
    fi
    return 1
}

parse_config()
{
    local CONF_FILE=/morse/configs/morse.conf
    MOD_PARAMS=""
    while read LINE; do
        if [ ${#LINE} -eq 0 ]; then
            continue
        fi
        if [ ${LINE:0:1} = "#" ]; then
            continue
        fi
        KEY=`echo $LINE | cut -d'=' -f1 -`
        VAL=`echo $LINE | cut -d'=' -f2 - | sed -e 's/ *#.*//'`
        STRIPPED_VAL=`echo $LINE | sed ':begin; s/=//2; t begin' | cut -d'=' -f2 | tr -cd "[A-Za-z0-9 \-_\.]"`
        case $KEY in
            dnsmasq_template)
                DNSMASQ_TEMPLATE=$VAL
                ;;
            hostapd_template)
                HOSTAPD_TEMPLATE=$VAL
                ;;
            wpa_supplicant_template)
                WPA_SUPPLICANT_TEMPLATE=$VAL
                ;;
            halow_iface)
                HALOW_IFACE=$VAL
                ;;
            eth_iface)
                ETH_IFACE=$VAL
                ;;
            mode)
                MODE=$VAL
                ;;
            ssid)
                SSID=$STRIPPED_VAL
                ;;
            security)
                SECURITY=$VAL
                ;;
            password)
                PASSWORD=$STRIPPED_VAL
                ;;
            enable_pmf)
                ENABLE_PMF=$VAL
                ;;
            beacon_int)
                BEACON_INT=$VAL
                ;;
            dtim_period)
                DTIM_PERIOD=$VAL
                ;;
            ap_max_inactivity)
                AP_MAX_INACTIVITY=$VAL
                ;;
            halow_primary_bandwidth)
                HALOW_PRIMARY_BANDWIDTH=$VAL
                ;;
            halow_primary_channel_index)
                HALOW_PRIMARY_CHANNEL_INDEX=$VAL
                ;;
            halow_channel)
                HALOW_CHANNEL=$VAL
                ;;
            op_class)
                HALOW_OP_CLASS=$VAL
                ;;
            bridge)
                BRIDGE=$VAL
                ;;
            bridge_ip_method)
                BRIDGE_IP_METHOD=$VAL
                ;;
            bridge_ip)
                BRIDGE_IP=$VAL
                ;;
            bridge_netmask)
                BRIDGE_NETMASK=$VAL
                ;;
            bridge_gateway)
                BRIDGE_GATEWAY=$VAL
                ;;
            forwarding)
                FORWARDING=$VAL
                ;;
            halow_ip_method)
                HALOW_IP_METHOD=$VAL
                ;;
            halow_ip)
                HALOW_IP=$VAL
                ;;
            halow_netmask)
                HALOW_NETMASK=$VAL
                ;;
            halow_gateway)
                HALOW_GATEWAY=$VAL
                ;;
            halow_dhcp_range_start)
                HALOW_DHCP_RANGE_START=$VAL
                ;;
            halow_dhcp_range_end)
                HALOW_DHCP_RANGE_END=$VAL
                ;;
            eth_ip_method)
                ETH_IP_METHOD=$VAL
                ;;
            eth_ip)
                ETH_IP=$VAL
                ;;
            eth_netmask)
                ETH_NETMASK=$VAL
                ;;
            eth_gateway)
                ETH_GATEWAY=$VAL
                ;;
            eth_dhcp_range_start)
                ETH_DHCP_RANGE_START=$VAL
                ;;
            eth_dhcp_range_end)
                ETH_DHCP_RANGE_END=$VAL
                ;;
            test_mode)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            debug_mask)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            serial)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            macaddr_octet)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_otp_check)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            no_hwcrypt)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            mcs_mask)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_survey)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_subbands)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_ps)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_sgi_rc)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_trav_pilot)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_rts_8mhz)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            country)
                COUNTRY=$VAL
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_watchdog)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            watchdog_interval_secs)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            max_rates)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            max_rate_tries)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            spi_clock_speed)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            max_txq_len)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_watchdog_reset)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            max_aggregation_count)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_raw)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_arp_offload)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            mcs10_mode)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_dynamic_ps_offload)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            enable_mac80211_connection_monitor)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
            virtual_sta_max)
                MOD_PARAMS="$MOD_PARAMS $KEY=$VAL"
                ;;
        esac
    done < $CONF_FILE

    # trim leading spaces from MOD_PARAMS
    MOD_PARAMS=`echo $MOD_PARAMS | xargs`

    # Read in options from PARAM_DIR and use those
    PARAM_DIR=/morse/mod_params/
    if [ -d "$PARAM_DIR" ]
    then
        if [ "$(ls -A $PARAM_DIR)" ]; then
            for FILE in $PARAM_DIR/* ; do
                echo "MORSE INIT - Found mod_param: $FILE=`cat $FILE`" | tee /dev/kmsg
                PARAM=$(basename $FILE)
                VAL=$(cat $FILE)
                PARAM_FOUND=`echo $MOD_PARAMS | grep -w $PARAM`
                if [ -z "$PARAM_FOUND" ]; then
                    # Not already in MOD_PARAMS, append it
                    MOD_PARAMS="${MOD_PARAMS} ${PARAM}=${VAL}"
                else
                    # Already in MOD_PARAMS, edit existing MOD_PARAMS
                    MOD_PARAMS=`echo $MOD_PARAMS | sed "s/\<$PARAM=[^ ]*/$PARAM=$VAL/g"`
                fi
            done
        fi
    fi

    case $BRIDGE in
        0|1)
            ;;
        *)
            BRIDGE=0
            echo "MORSE INIT - Bridge mode not given or invalid, defaulting to $BRIDGE" | tee /dev/kmsg
            ;;
    esac

    if [ $BRIDGE -eq 1 ]; then
        case $BRIDGE_IP_METHOD in
            0|1)
                ;;
            *)
                BRIDGE_IP_METHOD=0
                echo "MORSE INIT - Bridge IP method not given or invalid, using $BRIDGE_IP_METHOD" | tee /dev/kmsg
                ;;
        esac

        if [ $BRIDGE_IP_METHOD -eq 0 ]; then
            # Check IP addresses for bridge STATIC, NETMASK, GATEWAY
            ip_test "$BRIDGE_IP"
            if [ $? -eq 1 ]; then
                BRIDGE_IP="10.42.1.1"
                echo "MORSE INIT - Bridge IP not given or invalid, defaulting to $BRIDGE_IP" | tee /dev/kmsg
            fi

            ip_test "$BRIDGE_NETMASK"
            if [ $? -eq 1 ]; then
                BRIDGE_NETMASK="255.255.255.0"
                echo "MORSE INIT - Bridge netmask not given or invalid, defaulting to $BRIDGE_NETMASK" | tee /dev/kmsg
            fi

            ip_test "$BRIDGE_GATEWAY"
            if [ $? -eq 1 ]; then
                gen_gateway "$BRIDGE_IP"
                BRIDGE_GATEWAY=$GATEWAY
                echo "MORSE INIT - Bridge gateway not given or invalid, defaulting to $BRIDGE_GATEWAY" | tee /dev/kmsg
            fi
        fi
    else
        case $FORWARDING in
            router|extender|none)
                ;;
            *)
                FORWARDING="none"
                echo "MORSE INIT - Forwarding value not given or invalid, using forwarding $FORWARDING" | tee /dev/kmsg
                ;;
        esac

        case $HALOW_IP_METHOD in
            0|1|2)
                ;;
            *)
                HALOW_IP_METHOD=0
                echo "MORSE INIT - HaLow IP method not given or invalid, using $HALOW_IP_METHOD" | tee /dev/kmsg
                ;;
        esac

        if [ $HALOW_IP_METHOD -ne 2 ]; then
            # Check IP addresses for HaLow STATIC, NETMASK, GATEWAY
            ip_test "$HALOW_IP"
            if [ $? -eq 1 ]; then
                HALOW_IP="10.42.1.1"
                echo "MORSE INIT - HaLow IP not given or invalid, defaulting to $HALOW_IP" | tee /dev/kmsg
            fi

            ip_test "$HALOW_NETMASK"
            if [ $? -eq 1 ]; then
                HALOW_NETMASK="255.255.255.0"
                echo "MORSE INIT - HaLow netmask not given or invalid, defaulting to $HALOW_NETMASK" | tee /dev/kmsg
            fi

            ip_test "$HALOW_GATEWAY"
            if [ $? -eq 1 ]; then
                gen_gateway "$HALOW_IP"
                HALOW_GATEWAY=$GATEWAY
                echo "MORSE INIT - HaLow gateway not given or invalid, defaulting to $HALOW_GATEWAY" | tee /dev/kmsg
            fi

            # Generate HALOW_NET
            gen_net "$HALOW_IP" "$HALOW_NETMASK"
            HALOW_NET=$NET

            # Error check HaLow DHCP range
            if [ $HALOW_IP_METHOD -eq 1 ]; then
                check_dhcp_range "$HALOW_IP" "$HALOW_DHCP_RANGE_START" "$HALOW_DHCP_RANGE_END"
                HALOW_DHCP_RANGE_START=$DHCP_RANGE_START
                HALOW_DHCP_RANGE_END=$DHCP_RANGE_END
            fi
        fi

        case $ETH_IP_METHOD in
            0|1|2)
                ;;
            *)
                ETH_IP_METHOD=0
                echo "MORSE INIT - Eth IP method not given or invalid, using $ETH_IP_METHOD" | tee /dev/kmsg
                ;;
        esac

        if [ $ETH_IP_METHOD -ne 2 ]; then
            # Check IP addresses for Eth STATIC, NETMASK, GATEWAY
            ip_test "$ETH_IP"
            if [ $? -eq 1 ]; then
                ETH_IP="10.42.0.3"
                echo "MORSE INIT - Eth IP not given or invalid, defaulting to $ETH_IP" | tee /dev/kmsg
            fi

            ip_test "$ETH_NETMASK"
            if [ $? -eq 1 ]; then
                ETH_NETMASK="255.255.255.0"
                echo "MORSE INIT - Eth netmask not given or invalid, defaulting to $ETH_NETMASK" | tee /dev/kmsg
            fi

            ip_test "$ETH_GATEWAY"
            if [ $? -eq 1 ]; then
                gen_gateway "$ETH_IP"
                ETH_GATEWAY=$GATEWAY
                echo "MORSE INIT - Eth gateway not given or invalid, defaulting to $ETH_GATEWAY" | tee /dev/kmsg
            fi

            # Generate ETH_NET
            gen_net "$ETH_IP" "$ETH_NETMASK"
            ETH_NET=$NET

            # Error check Eth DHCP range
            if [ $ETH_IP_METHOD -eq 1 ]; then
                check_dhcp_range "$ETH_IP" "$ETH_DHCP_RANGE_START" "$ETH_DHCP_RANGE_END"
                ETH_DHCP_RANGE_START=$DHCP_RANGE_START
                ETH_DHCP_RANGE_END=$DHCP_RANGE_END
            fi
        fi
    fi

    if [ -z "$DNSMASQ_TEMPLATE" ]; then
        DNSMASQ_TEMPLATE=/morse/configs/dnsmasq.conf
        echo "MORSE INIT - No dnsmasq template given, using $DNSMASQ_TEMPLATE" | tee /dev/kmsg
    fi

    if [ -z $HALOW_IFACE ]; then
        HALOW_IFACE="wlan0"
        echo "MORSE INIT - No HaLow interface given, defaulting to $HALOW_IFACE" | tee /dev/kmsg
    fi

    if [ -z $ETH_IFACE ]; then
        ETH_IFACE="eth0"
        echo "MORSE INIT - No wired interface given, defaulting to $ETH_IFACE" | tee /dev/kmsg
    fi

    if [ -z $COUNTRY ]; then
        echo "MORSE INIT - Country not set, disabling radio. Please set country code" | tee /dev/kmsg
    else
        # Error check values and assign defaults if necessary
        if [ \( "$MODE" != "ap" -a "$MODE" != "sta" -a "$MODE" != "none" -a "$MODE" != "wnm-sta" \) -o -z "$MODE" ]; then
            MODE="none"
            echo "MORSE INIT - Mode is not AP or STA, defaulting to $MODE" | tee /dev/kmsg
        fi

        if [ "$MODE" == "ap" ]; then
            if [ -z "$HOSTAPD_TEMPLATE" ]; then
                HOSTAPD_TEMPLATE=/morse/configs/hostapd_s1g.conf
                echo "MORSE INIT - No hostapd template given, using $HOSTAPD_TEMPLATE" | tee /dev/kmsg
            fi
        fi

        case $SECURITY in
            owe|sae|open)
                ;;
            *)
                SECURITY=owe
                echo "MORSE INIT - Security mode not given or invalid, defaulting to $SECURITY" | tee /dev/kmsg
                ;;
        esac

        if [ "$SECURITY" != "owe" ]; then
            if [ -n "$PASSWORD" ]; then
                if [ ${#PASSWORD} -lt 8 ]; then
                    PASSWORD=12345678
                    echo "MORSE INIT - Password too short, using $PASSWORD" | tee /dev/kmsg
                elif [ ${#PASSWORD} -gt 63 ]; then
                    PASSWORD=${PASSWORD:0:63}
                    echo "MORSE INIT - Password too long, truncating to 64 characters ($PASSWORD)" | tee /dev/kmsg
                fi
            fi
        fi

        if [ "$MODE" == "sta" ]; then
            if [ -z "$WPA_SUPPLICANT_TEMPLATE" ]; then
                WPA_SUPPLICANT_TEMPLATE=/morse/configs/wpa_supplicant_s1g.conf
                echo "MORSE INIT - No wpa_supplicant template given, using $WPA_SUPPLICANT_TEMPLATE" | tee /dev/kmsg
            fi
        fi

        if [ "$MODE" != "none" ]; then
            if [ ${#SSID} -gt 32 ]; then
                SSID=${SSID:0:32}
                echo "MORSE INIT - SSID too long, truncating to $SSID" | tee /dev/kmsg
            fi

            if [ -n "$PASSWORD" ]; then
                if [ ${#PASSWORD} -lt 8 ]; then
                    PASSWORD=12345678
                    echo "MORSE INIT - Password too short, using $PASSWORD" | tee /dev/kmsg
                elif [ ${#PASSWORD} -gt 63 ]; then
                    PASSWORD=${PASSWORD:0:63}
                    echo "MORSE INIT - Password too long, truncating to 64 characters ($PASSWORD)" | tee /dev/kmsg
                fi
            fi
        fi

        if [ "$MODE" == "ap" ]; then
            if [ -z "$ENABLE_PMF" ]; then
                ENABLE_PMF=0
                echo "MORSE INIT - Enable PMF not set, using $ENABLE_PMF" | tee /dev/kmsg
            fi
            num_test "$ENABLE_PMF"
            if [ $? -eq 1 ]; then
                ENABLE_PMF=0
                echo "MORSE INIT - Enable PMF invalid, using $ENABLE_PMF" | tee /dev/kmsg
            fi

            if [ ! -z "$BEACON_INT" ]; then
                num_test $BEACON_INT
                if [ $? -eq 1 ]; then
                    echo "MORSE INIT - Beacon interval not given or invalid, using value in template" | tee /dev/kmsg
                    BEACON_INT=
                elif [ $BEACON_INT -lt 15 ]; then
                    BEACON_INT=15
                    echo "MORSE INIT - Beacon interval too low, using $BEACON_INT" | tee /dev/kmsg
                elif [ $BEACON_INT -gt 65535 ]; then
                    BEACON_INT=65535
                    echo "MORSE INIT - Beacon interval too high, using $BEACON_INT" | tee /dev/kmsg
                fi
            fi

            if [ ! -z "$DTIM_PERIOD" ]; then
                num_test $DTIM_PERIOD
                if [ $? -eq 1 ]; then
                    echo "MORSE INIT - DTIM period not given or invalid, using value in template" | tee /dev/kmsg
                    DTIM_PERIOD=
                elif [ $DTIM_PERIOD -lt 1 ]; then
                    DTIM_PERIOD=1
                    echo "MORSE INIT - DTIM period too low, using $DTIM_PERIOD" | tee /dev/kmsg
                elif [ $DTIM_PERIOD -gt 255 ]; then
                    DTIM_PERIOD=255
                    echo "MORSE INIT - DTIM period too high, using $DTIM_PERIOD" | tee /dev/kmsg
                fi
            fi

            if [ ! -z "$AP_MAX_INACTIVITY" ]; then
                num_test $AP_MAX_INACTIVITY
                if [ $? -eq 1 ]; then
                    echo "MORSE INIT - AP max inactivity not given or invalid, using value in template" | tee /dev/kmsg
                    AP_MAX_INACTIVITY=
                fi
            fi

            num_test $HALOW_PRIMARY_CHANNEL_INDEX
            if [ $? -eq 1 ]; then
                HALOW_PRIMARY_CHANNEL_INDEX=0
                echo "MORSE INIT - Primary channel index not given or invalid, using $HALOW_PRIMARY_CHANNEL_INDEX" | tee /dev/kmsg
            fi

            num_test $HALOW_CHANNEL
            if [ $? -eq 1 ]; then
                HALOW_CHANNEL=0
                echo "MORSE INIT - Channel not set, using $HALOW_CHANNEL" | tee /dev/kmsg
            fi

            # Verify the channel is valid for given op class and country, by finding the bandwidth.
            # Note, also sets the regulatory duty cycle for the given operating mode and country.
            HALOW_BW=
            DUTY_CYCLE=
            if [ $HALOW_CHANNEL -eq 0 ]; then
                echo "MORSE INIT - Automatic Channel Selection" | tee /dev/kmsg
                country_op_class_validate $COUNTRY $HALOW_OP_CLASS $MODE
            else
                country_op_class_ch_validate $COUNTRY $HALOW_OP_CLASS $HALOW_CHANNEL $MODE
            fi

<< EOF
##comment start
            # Channel not valid for given country and op class
            if [ -z "$HALOW_BW" ]; then
                echo "MORSE INIT - Invalid channel $HALOW_CHANNEL for country $COUNTRY operating class $HALOW_OP_CLASS. STOPPING" | tee /dev/kmsg
                return 1
            fi
####end
EOF
            echo "MORSE INIT - Using country $COUNTRY operating class $HALOW_OP_CLASS channel $HALOW_CHANNEL with bandwidth $HALOW_BW MHz center frequency $FREQ_C_MHZ MHz and $DUTY_CYCLE% duty cycle" | tee /dev/kmsg

            num_test $HALOW_PRIMARY_BANDWIDTH
            if [ $? -eq 1 ]; then
                if [ $HALOW_BW -eq 4 ] || [ $HALOW_BW -eq 8 ]; then
                    HALOW_PRIMARY_BANDWIDTH=2
                else
                    HALOW_PRIMARY_BANDWIDTH=1
                fi
                echo "MORSE INIT - Primary bandwidth not given or invalid, using $HALOW_PRIMARY_BANDWIDTH" | tee /dev/kmsg
            fi
        fi
    fi

    if [ "$MODE" == "sta" ]; then

        # Note, set the regulatory duty cycle for the given operating mode and country.
        HALOW_BW=
        DUTY_CYCLE=
        country_op_class_validate $COUNTRY $HALOW_OP_CLASS $MODE

        echo "MORSE INIT - Using country $COUNTRY operating class $HALOW_OP_CLASS and $DUTY_CYCLE% duty cycle" | tee /dev/kmsg
    fi
}

start_bridge()
{
    # Remove old bridge interface if there is one
    if ifconfig -a | grep -q br0; then
        ifconfig br0 down
        brctl delbr br0
    fi
    # If bridging is enabled, set up the bridge interface
    if [ $BRIDGE -eq 1 ]; then
        case $MODE in
            ap)
                brctl addbr br0
                brctl addif br0 $ETH_IFACE
                ip link set br0 up
                ;;
            sta)
                iw $HALOW_IFACE set 4addr on
                brctl addbr br0
                brctl addif br0 $ETH_IFACE $HALOW_IFACE
                ip link set br0 up
                ;;
        esac
    fi
}

# start_ap expects the following variables to exist (parse_config may be called to set them):
#   AP_MAX_INACTIVITY
#   BEACON_INT
#   BRIDGE
#   DTIM_PERIOD
#   ENABLE_PMF
#   HALOW_CHANNEL
#   HALOW_IFACE
#   HALOW_OP_CLASS
#   HALOW_PRIMARY_BANDWIDTH
#   HOSTAPD_TEMPLATE
#   COUNTRY
#   PASSWORD
#   SECURITY
#   SSID
start_ap()
{
    local CONF_FILE=/tmp/hostapd_s1g.conf
    local SED_FILE=/tmp/ap_replacements.sed
    local LOG_FILE=/var/log/morse_hostapd.log
    local MSG_FILE=/morse/scripts/msgap.txt
    local HOSTAPD_PRIM_CHWIDTH=$(($HALOW_PRIMARY_BANDWIDTH - 1))

    # Edit hostapd_s1g.conf
    echo "s/\(\<\)ssid=.*/\1ssid=$SSID/"                    >  $SED_FILE
    echo "s/\(\<\)interface=.*/\1interface=$HALOW_IFACE/"   >> $SED_FILE
    echo "s/\(\<\)channel=.*/\1channel=$HALOW_CHANNEL/"     >> $SED_FILE
    echo "s/\(\<\)op_class=.*/\op_class=$HALOW_OP_CLASS/"   >> $SED_FILE
    echo "s/\(\<\)country_code=.*/\country_code=$COUNTRY/"   >> $SED_FILE
    echo "s/\(\<\)s1g_prim_chwidth=.*/\s1g_prim_chwidth=$HOSTAPD_PRIM_CHWIDTH/"   >> $SED_FILE

    if [ "$SECURITY" == "open" ]; then
        echo "s/\(\<\)#\?wpa=.*/\1#wpa=2/"                      >> $SED_FILE
        echo "s/\(\<\)#\?wpa_key_mgmt=.*/\1#wpa_key_mgmt=SAE/"  >> $SED_FILE
        echo "s/\(\<\)#\?rsn_pairwise=.*/\1#rsn_pairwise=CCMP/" >> $SED_FILE
        echo "s/\(\<\)#\?sae_password=/\1#sae_password=/"       >> $SED_FILE
    elif [ "$SECURITY" == "owe" ]; then
        echo "s/\(\<\)wpa_key_mgmt=.*/\1wpa_key_mgmt=OWE/" >> $SED_FILE
        echo "s/\(\<\)#\?sae_password=/\1#sae_password=/"  >> $SED_FILE
    else
        echo "s/\(\<\)wpa_key_mgmt=.*/\1wpa_key_mgmt=SAE/"       >> $SED_FILE
        echo "/sae_password=/s/#//"                              >> $SED_FILE
        echo "s/\(\<\)sae_password=.*/\1sae_password=$PASSWORD/" >> $SED_FILE
    fi
    if [ -n "$BEACON_INT" ]; then
        echo "s/\(\<\)beacon_int=.*/\1beacon_int=$BEACON_INT/" >> $SED_FILE
    fi
    if [ -n "$DTIM_PERIOD" ]; then
        echo "s/\(\<\)dtim_period=.*/\1dtim_period=$DTIM_PERIOD/" >> $SED_FILE
    fi
    if [ -n "$AP_MAX_INACTIVITY" ]; then
        echo "s/\(\<\)ap_max_inactivity=.*/\1ap_max_inactivity=$AP_MAX_INACTIVITY/" >> $SED_FILE
    fi
    if [ $ENABLE_PMF -eq 0 ]; then
        echo "s/\(\<\)#\?ieee80211w=2/#ieee80211w=2/" >> $SED_FILE
    else
        echo "s/\(\<\)#\?ieee80211w=2/ieee80211w=2/" >> $SED_FILE
    fi
    sed -f $SED_FILE $HOSTAPD_TEMPLATE > $CONF_FILE
    rm $SED_FILE

    echo "MORSE INIT - Starting HaLow AP - log file is $LOG_FILE" | tee /dev/kmsg
    hostapd_s1g -t -B -f $LOG_FILE $CONF_FILE

    sleep 1
    if ! pidof hostapd_s1g > /dev/null; then
        echo "MORSE INIT - Failed to start hostapd_s1g, check $LOG_FILE" | tee /dev/kmsg
    fi

    if [ $BRIDGE -eq 1 ]; then
        hostapd_cli_s1g -p /var/run/hostapd_s1g set wds_sta 1
        hostapd_cli_s1g -p /var/run/hostapd_s1g set wds_bridge br0
    fi

    if [ ! -z "$DUTY_CYCLE" ]; then
        morsectrl duty_cycle $DUTY_CYCLE
    fi

    cat $MSG_FILE | tee /dev/kmsg
}

# start_sta expects the following variables to exist (parse_config may be called to set them):
#   BRIDGE
#   ENABLE_PMF
#   HALOW_IFACE
#   PASSWORD
#   SECURITY
#   SSID
#   WPA_SUPPLICANT_TEMPLATE
start_sta()
{
    local CONF_FILE=/tmp/wpa_supplicant_s1g.conf
    local SED_FILE=/tmp/sta_replacements.sed
    local LOG_FILE=/var/log/morse_wpa_supplicant.log
    local MSG_FILE=/morse/scripts/msgsta.txt

    # Edit wpa_supplicant_s1g.conf
    echo "s/\(\<\)ssid=.*/\1ssid=\"$SSID\"/" > $SED_FILE
    if [ "$SECURITY" == "open" ]; then
        echo "s/\(\<\)key_mgmt=.*/\1key_mgmt=NONE/" >> $SED_FILE
        echo "s/\(\<\)#\?psk=/\1#psk=/"             >> $SED_FILE
    elif [ "$SECURITY" == "owe" ]; then
        echo "s/\(\<\)key_mgmt=.*/\1key_mgmt=OWE/" >> $SED_FILE
        echo "s/\(\<\)#\?psk=/\1#psk=/"            >> $SED_FILE
        echo "/proto=RSN/s/#//"                    >> $SED_FILE
        echo "/pairwise=CCMP/s/#//"                >> $SED_FILE
    else
        echo "s/\(\<\)key_mgmt=.*/\1key_mgmt=SAE/"        >> $SED_FILE
        echo "/psk=/s/#//"                                >> $SED_FILE
        echo "s/\(\<\)psk=.*/\1psk=\"$PASSWORD\"/"        >> $SED_FILE
        echo "s/\(\<\)#\?proto=RSN/\1#proto=RSN/"         >> $SED_FILE
        echo "s/\(\<\)#\?pairwise=CCMP/\1#pairwise=CCMP/" >> $SED_FILE
    fi
    if [ $ENABLE_PMF -eq 0 ]; then
        echo "s/\(\<\)#\?pmf=2/#pmf=2/" >> $SED_FILE
    else
        echo "s/\(\<\)#\?pmf=2/pmf=2/" >> $SED_FILE
    fi
    sed -f $SED_FILE $WPA_SUPPLICANT_TEMPLATE > $CONF_FILE
    rm $SED_FILE

    echo "MORSE INIT - Starting HaLow STA - log file is $LOG_FILE" | tee /dev/kmsg

    if [ $BRIDGE -eq 1 ]; then
        wpa_supplicant_s1g -t -D nl80211 -i $HALOW_IFACE -b br0 -c $CONF_FILE -B -f $LOG_FILE
    else
        wpa_supplicant_s1g -t -D nl80211 -i $HALOW_IFACE -c $CONF_FILE -B -f $LOG_FILE
    fi

    if [ ! -z "$DUTY_CYCLE" ]; then
        morsectrl duty_cycle $DUTY_CYCLE
    fi

    cat $MSG_FILE | tee /dev/kmsg
}

start_ip_routing()
{
    ip route flush all

    if [ $BRIDGE -eq 1 ]; then
        ifconfig $HALOW_IFACE 0.0.0.0
        ifconfig $ETH_IFACE 0.0.0.0

        if [ $BRIDGE_IP_METHOD -eq 0 ]; then
            echo "MORSE INIT - Setting IP on br0 to $BRIDGE_IP" | tee /dev/kmsg
            ifconfig br0 0.0.0.0
            ifconfig br0 $BRIDGE_IP netmask $BRIDGE_NETMASK up
        else
            echo "MORSE INIT - Send DHCP request on br0" | tee /dev/kmsg
            udhcpc -t 2 -T 2 -b -i br0
        fi

        cat /morse/scripts/msgbridge.txt | tee /dev/kmsg
        return
    fi

    local DNSMASQ_CONF="/tmp/dnsmasq.conf"
    local SED_FILE="/tmp/dnsmasq_replacements.sed"

    rm -f $SED_FILE
    touch $SED_FILE

    # Forwarding
    iptables --flush
    iptables -t nat --flush
    case $FORWARDING in
        router)
            echo "MORSE INIT - Enabling IPv4 forwarding from $HALOW_IFACE to $ETH_IFACE"
            echo 1 > /proc/sys/net/ipv4/ip_forward
            iptables -t nat -A POSTROUTING --out-interface $ETH_IFACE -j MASQUERADE
            iptables -A FORWARD --in-interface $HALOW_IFACE -j ACCEPT
            ;;
        extender)
            echo "MORSE INIT - Enabling IPv4 forwarding from $ETH_IFACE to $HALOW_IFACE"
            echo 1 > /proc/sys/net/ipv4/ip_forward
            iptables -t nat -A POSTROUTING --out-interface $HALOW_IFACE -j MASQUERADE
            iptables -A FORWARD --in-interface $ETH_IFACE -j ACCEPT
            ;;
        none)
            echo "MORSE INIT - IPv4 forwarding is disabled"
            echo 0 > /proc/sys/net/ipv4/ip_forward
            ;;
    esac

    if [ $HALOW_IP_METHOD -eq 0            \
        -a $ETH_IP_METHOD -eq 0            \
        -a "$HALOW_IP" != "$HALOW_GATEWAY" \
        -a "$ETH_IP" != "$ETH_GATEWAY" ]; then
        HALOW_METRIC="metric 100"
        ETH_METRIC="metric 50"
    fi

    # HaLow IP/Routing
    case $HALOW_IP_METHOD in
        0)
            # Static IP
            echo "MORSE INIT - Setting IP on $HALOW_IFACE to $HALOW_IP" | tee /dev/kmsg
            # Remove previous IP address so setting an IP through ifconfig will also create a route
            ifconfig $HALOW_IFACE 0.0.0.0
            ifconfig $HALOW_IFACE $HALOW_IP netmask $HALOW_NETMASK
            if [ "$HALOW_IP" != "$HALOW_GATEWAY" ]; then
                route add default gw $HALOW_GATEWAY $HALOW_METRIC $HALOW_IFACE
            fi
            # Disable DHCP on halow interface
            echo "s/\(\<\)#\?dhcp-option=$HALOW_IFACE/\1#dhcp-option=$HALOW_IFACE/" >> $SED_FILE
            echo "s/\(\<\)#\?dhcp-range=$HALOW_IFACE/\1#dhcp-range=$HALOW_IFACE/"   >> $SED_FILE
            ;;
        1)
            # DHCP Server
            echo "MORSE INIT - Setting IP on $HALOW_IFACE to $HALOW_IP" | tee /dev/kmsg
            # Remove previous IP address so setting an IP through ifconfig will also create a route
            ifconfig $HALOW_IFACE 0.0.0.0
            ifconfig $HALOW_IFACE $HALOW_IP netmask $HALOW_NETMASK
            # Set DHCP range and gateway
            local RANGE_PATTERN=#\\?dhcp-range=$HALOW_IFACE,.*
            local DHCP_RANGE=$HALOW_DHCP_RANGE_START,$HALOW_DHCP_RANGE_END
            local RANGE_REPLACEMENT=dhcp-range=$HALOW_IFACE,$DHCP_RANGE,$HALOW_NETMASK,12h
            local GW_PATTERN=#\\?dhcp-option=$HALOW_IFACE,3,.*
            local GW_REPLACEMENT=dhcp-option=$HALOW_IFACE,3,$HALOW_GATEWAY
            echo "s/\(\<\)$RANGE_PATTERN/\1$RANGE_REPLACEMENT/" >> $SED_FILE
            echo "s/\(\<\)$GW_PATTERN/\1$GW_REPLACEMENT/"       >> $SED_FILE
            ;;
        2)
            # DHCP Client
            echo "MORSE INIT - Sending DHCP request on $HALOW_IFACE" | tee /dev/kmsg
            udhcpc -t 2 -T 2 -b -i $HALOW_IFACE
            # Disable DHCP on halow interface
            echo "s/\(\<\)#\?dhcp-option=$HALOW_IFACE/\1#dhcp-option=$HALOW_IFACE/" >> $SED_FILE
            echo "s/\(\<\)#\?dhcp-range=$HALOW_IFACE/\1#dhcp-range=$HALOW_IFACE/"   >> $SED_FILE
            ;;
    esac

    # Eth IP/Routing
    case $ETH_IP_METHOD in
        0)
            # Static IP
            echo "MORSE INIT - Setting IP on $ETH_IFACE to $ETH_IP" | tee /dev/kmsg
            # Remove previous IP address so setting an IP through ifconfig will also create a route
            ifconfig $ETH_IFACE 0.0.0.0
            ifconfig $ETH_IFACE $ETH_IP netmask $ETH_NETMASK
            if [ "$ETH_IP" != "$ETH_GATEWAY" ]; then
                route add default gw $ETH_GATEWAY $ETH_METRIC $ETH_IFACE
            fi
            # Disable DHCP on eth interface
            echo "s/\(\<\)#\?dhcp-option=$ETH_IFACE/\1#dhcp-option=$ETH_IFACE/" >> $SED_FILE
            echo "s/\(\<\)#\?dhcp-range=$ETH_IFACE/\1#dhcp-range=$ETH_IFACE/"   >> $SED_FILE
            ;;
        1)
            # DHCP Server
            echo "MORSE INIT - Setting IP on $ETH_IFACE to $ETH_IP" | tee /dev/kmsg
            # Remove previous IP address so setting an IP through ifconfig will also create a route
            ifconfig $ETH_IFACE 0.0.0.0
            ifconfig $ETH_IFACE $ETH_IP netmask $ETH_NETMASK
            # Set DHCP range and gateway
            local RANGE_PATTERN=#\\?dhcp-range=$ETH_IFACE,.*
            local DHCP_RANGE=$ETH_DHCP_RANGE_START,$ETH_DHCP_RANGE_END
            local RANGE_REPLACEMENT=dhcp-range=$ETH_IFACE,$DHCP_RANGE,$ETH_NETMASK,12h
            local GW_PATTERN=#\\?dhcp-option=$ETH_IFACE,3,.*
            local GW_REPLACEMENT=dhcp-option=$ETH_IFACE,3,$ETH_GATEWAY
            echo "s/\(\<\)$RANGE_PATTERN/\1$RANGE_REPLACEMENT/" >> $SED_FILE
            echo "s/\(\<\)$GW_PATTERN/\1$GW_REPLACEMENT/"       >> $SED_FILE
            ;;
        2)
            # DHCP Client
            echo "MORSE INIT - Send DHCP request on $ETH_IFACE" | tee /dev/kmsg
            udhcpc -t 2 -T 2 -b -i $ETH_IFACE
            # Disable DHCP on eth interface
            echo "s/\(\<\)#\?dhcp-option=$ETH_IFACE/\1#dhcp-option=$ETH_IFACE/" >> $SED_FILE
            echo "s/\(\<\)#\?dhcp-range=$ETH_IFACE/\1#dhcp-range=$ETH_IFACE/"   >> $SED_FILE
            ;;
    esac

    # Start dnsmasq for DNS and DHCP
    echo "MORSE INIT - Starting dnsmasq for DHCP and DNS" | tee /dev/kmsg
    echo "s/\(\<\)listen-address=.*/\1listen-address=$HALOW_IP,$ETH_IP,127.0.0.1/" >> $SED_FILE
    sed -f $SED_FILE $DNSMASQ_TEMPLATE > $DNSMASQ_CONF
    rm $SED_FILE
    dnsmasq -C $DNSMASQ_CONF

    case $FORWARDING in
        router)
            cat /morse/scripts/msgrouter.txt | tee /dev/kmsg
            ;;
        extender)
            cat /morse/scripts/msgextender.txt | tee /dev/kmsg
            ;;
    esac
}
