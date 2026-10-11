cmake_minimum_required(VERSION 3.20)

project(APPS_UnitTests LANGUAGES C)

set(CMAKE_C_STANDARD 11)

add_executable(test_adc_conversion
        unit/test_adc_conversion.c
        ../Core/Src/apps_conversion.c
        unity/src/unity.c
)

target_include_directories(test_adc_conversion PRIVATE
        ../Core/Inc
        unity/src
)

enable_testing()

add_test(
        NAME ADC_Conversion
        COMMAND test_adc_conversion
)