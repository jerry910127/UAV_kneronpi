#!/bin/bash

BGREEN='\033[1;32m'
BWHITE='\033[1;37m'
NC='\033[0m'

# 0: 佔位用；1~9 為各範例，10 = 全部
BUILD_ARR=("none" \
           "example_nnm_image" \
           "example_nnm_webcam" \
           "example_nnm_sensor" \
           "example_nnm_rtsp" \
           "rgbir" \
           "rgbir_with_Temperature_monitoring" \
           "rgbir_correction" \
           "ir_fisheye_correction" \
           "example_nnm_webcam_ACG_CLAHE")

rm -rf build
rm -rf bin
mkdir build
mkdir bin

# ------------------ 處理參數 ------------------
if [ "$1" = "-a" ]; then
    BUILD_NUM=10                 # -a = build all
else
    echo ""
    echo "Please select the example(s) to build:"
    echo "---------------- Basic Examples ----------------"
    echo "[1] example_nnm_image"
    echo "---------------- OpenCV Examples ----------------"
    echo "[2] example_nnm_webcam"
    echo "[3] example_nnm_sensor"
    echo "[4] example_nnm_rtsp"
    echo "---------------- Our Project ----------------"
    echo "[5] rgbir"
    echo "[6] rgbir_with_Temperature_monitoring"
    echo "[7] rgbir_correction"
    echo "[8] ir_fisheye_correction"
    echo "[9] webcam_AGC_CLAHE"
    echo "---------------- All Examples -------------------"
    echo "[10] All examples"
    echo -e "$BWHITE"
    read -p "Please enter 1-10: " BUILD_NUM
    echo -e "$NC"
fi

# ------------------ 編譯流程 ------------------
case "$BUILD_NUM" in
    "1"|"2"|"3"|"4"|"5"|"6"|"7"|"8"|"9")
        echo -e "\n$BGREEN[BUILD] ${BUILD_ARR[$BUILD_NUM]}$NC\n"
        rm -rf build
        mkdir build
        cd build
        cmake "../${BUILD_ARR[$BUILD_NUM]}" || exit 1
        make || exit 1
        cp -rf "./bin" "../"
        cd ".."
        ;;
    "10")
        # 逐一把 1~9 全部編譯
        for i in {1..9}; do
            echo -e "\n$BGREEN[BUILD] ${BUILD_ARR[$i]}$NC\n"
            rm -rf build
            mkdir build
            cd build
            cmake "../${BUILD_ARR[$i]}" || exit 1
            make || exit 1
            cp -rf "./bin" "../"
            cd ".."
        done
        ;;
    *)
        echo "The input is not valid"
        exit 1
        ;;
esac

#rm -rf build
