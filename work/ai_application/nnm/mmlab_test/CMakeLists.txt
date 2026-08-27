cmake_minimum_required(VERSION 3.16)

project(To_C_PLUS LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)

add_executable(temperature_demo
    src/main.cpp
    src/TemperatureProcessor.cpp
)

target_include_directories(temperature_demo
    PRIVATE
        src
)

target_link_libraries(temperature_demo
    PRIVATE
        ${OpenCV_LIBS}
)

if(MSVC)
    target_compile_options(temperature_demo PRIVATE /W4)
else()
    target_compile_options(temperature_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()
