#!/bin/bash
# 直接執行此腳本可以編譯 TOF 測試程式,輸出到 bin 資料夾中
rm -rf build
# rm -rf bin
mkdir build
# mkdir bin
cd build
cmake ../
make
cp -rf "./bin" "../"
cd ".."
rm -rf build
