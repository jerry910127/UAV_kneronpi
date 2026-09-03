#!/bin/sh

echo "$0 $1 $2..." >/tmp/rgbir_ctl.txt
case $1 in
alpha)
    bash -lc 'timeout 0.2s printf "%s\n" alpha_next >/run/rgbir_ctl'
    ;;

cmap)
    bash -lc 'timeout 0.2s printf "%s\n" cmap_next >/run/rgbir_ctl'
    ;;

reset)
    bash -lc 'timeout 0.2s printf "%s\n" reset >/run/rgbir_ctl'
    ;;

left)
    bash -lc 'timeout 0.2s echo "move -2 0" >/run/rgbir_ctl'
    ;;

right)
    bash -lc 'timeout 0.2s echo "move 2 0" >/run/rgbir_ctl'
    ;;

up)
    bash -lc 'timeout 0.2s echo "move 0 -2" >/run/rgbir_ctl'
    ;;

down)
    bash -lc 'timeout 0.2s echo "move 0 2" >/run/rgbir_ctl'
    ;;

alpha_pct | alpha_percent)
    bash -lc "timeout 0.2s echo 'alpha $2' >/run/rgbir_ctl"
    ;;

cmap_idx)
    bash -lc "timeout 0.2s echo 'cmap $2' >/run/rgbir_ctl"
    ;;

movex | dx)
    bash -lc "timeout 0.2s echo 'movex $2' >/run/rgbir_ctl"
    ;;

movey | dy)
    bash -lc "timeout 0.2s echo 'movey $2' >/run/rgbir_ctl"
    ;;

snap)
    bash -lc 'timeout 0.2s echo snap >/run/rgbir_ctl'
    ;;

hotmask)
    bash -lc 'timeout 0.2s echo hotmark_toggle >/run/rgbir_ctl'
    ;;

key)
    echo $2 >/run/rgbir_ctl
    ;;

mon_x | mon_y | mon_w | mon_h)
    echo "$1 $2" >/run/rgbir_ctl
    ;;

help | -h | *)
    echo "Usage: $0 OPTION"
    echo -e "  OPTION: alpha               Alpha Blending"
    echo -e "          cmap                Color Map"
    echo -e "          up|down|left|right  Shift 2 pixels"
    echo -e "          dx n|dy n           Shift axis n pixels"
    echo -e "          alpha_pct n         Set the Level of Alpha Blending,"
    echo -e "                                  {0, 25, 50, 75, 100}"
    echo -e "          cmap_idx n          Set the Index of Color Map,"
    echo -e "                                  {0, 1, 2, 3, 4, 5}"
    echo -e "          snap                Screenshot"
    echo -e "          hotmask             Mask the max. temperature"
    ;;
esac
