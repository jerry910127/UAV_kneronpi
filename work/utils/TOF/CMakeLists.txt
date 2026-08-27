cmake_minimum_required(VERSION 3.22)
project(tofproject)

# 建立執行檔
add_executable(tof main.cpp tof_lib.cpp)

# 設定輸出目錄
set_target_properties(tof PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)