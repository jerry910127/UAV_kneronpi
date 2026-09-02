#!/bin/bash

BGREEN='\033[1;32m'
BWHITE='\033[1;37m'
NC='\033[0m'

# 0: 佔位用；1~7 為各範例，8 = 全部
BUILD_ARR=("none" \
           "example_nnm_image" \
           "example_nnm_webcam" \
           "example_nnm_sensor" \
           "example_nnm_rtsp" \
           "rgbir" \
           "rgbir_correction" \
           "ir_fisheye_correction" \
           "rgbir_correction_y16_114"
           )

rm -rf build
rm -rf bin
mkdir build
mkdir bin

# ------------------ 處理參數 ------------------
if [ "$1" = "-a" ]; then
    BUILD_NUM=8                 # -a = build all
elif [[ "$1" =~ ^[1-8]$ ]]; then
    BUILD_NUM="$1"              # 直接讀取傳入的數字 1~8
else
    # 若無參數或參數無效，顯示互動選單
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
    echo "[6] rgbir_correction"
    echo "[7] ir_fisheye_correction"
    echo "[8] ir_fisheye_correction_y16_114"
    echo "---------------- All Examples -------------------"
    echo "[9] All examples"
    echo -e "$BWHITE"
    read -p "Please enter 1-8: " BUILD_NUM
    echo -e "$NC"
fi

# ------------------ 編譯流程 ------------------
case "$BUILD_NUM" in
    "1"|"2"|"3"|"4"|"5"|"6"|"7"|"8")
        echo -e "\n$BGREEN[BUILD] ${BUILD_ARR[$BUILD_NUM]}$NC\n"
        rm -rf build
        mkdir build
        cd build
        cmake "../${BUILD_ARR[$BUILD_NUM]}" || exit 1
        make || exit 1
        
        # 假設你的 CMake 會把執行檔放在 build/bin 裡面
        if [ -d "./bin" ]; then
            cp -rf "./bin/"* "../bin/"
        fi
        cd ".."
        ;;
    "9")
        # 逐一把 1~7 全部編譯
        for i in {1..7}; do
            echo -e "\n$BGREEN[BUILD] ${BUILD_ARR[$i]}$NC\n"
            rm -rf build
            mkdir build
            cd build
            cmake "../${BUILD_ARR[$i]}" || exit 1
            make  || exit 1
            
            if [ -d "./bin" ]; then
                cp -rf "./bin/"* "../bin/"
            fi
            cd ".."
        done
        ;;
    *)
        echo "The input is not valid"
        exit 1
        ;;
esac

# 註：如果想要在編譯完自動清理 build 資料夾，可以把這行解開
# rm -rf build