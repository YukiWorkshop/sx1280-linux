cmake_minimum_required(VERSION 3.14)
project(sx1280_driver)

set(CMAKE_CXX_STANDARD 17)

include(cmake/CPM.cmake)

CPMAddPackage(
        NAME GPIOPlusPlus
        GITHUB_REPOSITORY YukiWorkshop/GPIOPlusPlus
        VERSION 0.0.2
)

CPMAddPackage(
        NAME SPPI
        GITHUB_REPOSITORY YukiWorkshop/SPPI
        VERSION 0.0.3
)

include_directories(${GPIOPlusPlus_SOURCE_DIR})
include_directories(${SPPI_SOURCE_DIR})

include_directories(rangingCorrection)

add_library(Driver_Semtech_SX1280
        rangingCorrection/rangingCorrection_defines.h
        rangingCorrection/rangingCorrectionSF10BW0400.h
        rangingCorrection/rangingCorrectionSF10BW0800.h
        rangingCorrection/rangingCorrectionSF10BW1600.h
        rangingCorrection/rangingCorrectionSF5BW0400.h
        rangingCorrection/rangingCorrectionSF5BW0800.h
        rangingCorrection/rangingCorrectionSF5BW1600.h
        rangingCorrection/rangingCorrectionSF6BW0400.h
        rangingCorrection/rangingCorrectionSF6BW0800.h
        rangingCorrection/rangingCorrectionSF6BW1600.h
        rangingCorrection/rangingCorrectionSF7BW0400.h
        rangingCorrection/rangingCorrectionSF7BW0800.h
        rangingCorrection/rangingCorrectionSF7BW1600.h
        rangingCorrection/rangingCorrectionSF8BW0400.h
        rangingCorrection/rangingCorrectionSF8BW0800.h
        rangingCorrection/rangingCorrectionSF8BW1600.h
        rangingCorrection/rangingCorrectionSF9BW0400.h
        rangingCorrection/rangingCorrectionSF9BW0800.h
        rangingCorrection/rangingCorrectionSF9BW1600.h
        radio.h
        RangingCorrection.cpp
        RangingCorrection.h
        sx1280_uart.cpp
        sx1280_uart.hpp
        sx1280_spi.cpp
        sx1280_spi.hpp
        sx1280.cpp
        sx1280.hpp)

target_link_libraries(Driver_Semtech_SX1280 pthread GPIOPlusPlus SPPI)

add_executable(SX1280_Test test.cpp)
target_link_libraries(SX1280_Test Driver_Semtech_SX1280)