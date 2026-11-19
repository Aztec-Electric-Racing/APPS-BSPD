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

add_executable(test_sensor_permille
        unit/test_sensor_permille.c
        ../Core/Src/apps_logic.c
        unity/src/unity.c
)

target_include_directories(test_sensor_permille PRIVATE
        ../Core/Inc
        unity/src
)

add_executable(test_clampdac
        unit/test_clampdac.c
        ../Core/Src/apps_logic.c
        unity/src/unity.c
)

target_include_directories(test_clampdac PRIVATE
        ../Core/Inc
        unity/src
)

add_executable(test_throttle_permille
        unit/test_throttle_permille.c
        ../Core/Src/apps_logic.c
        unity/src/unity.c
)

target_include_directories(test_throttle_permille PRIVATE
        ../Core/Inc
        unity/src
)

add_executable(test_sensors_valid
        unit/test_sensors_valid.c
        ../Core/Src/apps_logic.c
        unity/src/unity.c
)

target_include_directories(test_sensors_valid PRIVATE
        ../Core/Inc
        unity/src
)

add_executable(test_calibration_valid
        unit/test_calibration_valid.c
        ../Core/Src/apps_logic.c
        unity/src/unity.c
)

target_include_directories(test_calibration_valid PRIVATE
        ../Core/Inc
        unity/src
)

add_executable(test_mismatch_timer
        unit/test_mismatch_timer.c
        ../Core/Src/apps_logic.c
        unity/src/unity.c
)

target_include_directories(test_mismatch_timer PRIVATE
        ../Core/Inc
        unity/src
)

add_executable(test_throttle_brake_latch
        unit/test_throttle_brake_latch.c
        ../Core/Src/apps_logic.c
        unity/src/unity.c
)

target_include_directories(test_throttle_brake_latch PRIVATE
        ../Core/Inc
        unity/src
)

enable_testing()

add_test(
        NAME ADC_Conversion
        COMMAND test_adc_conversion
)

add_test(
        NAME Sensor_Permille
        COMMAND test_sensor_permille
)

add_test(
        NAME Clampdac
        COMMAND test_clampdac
)

add_test(
        NAME Throttle_Permille
        COMMAND test_throttle_permille
)

add_test(
        NAME Sensors_Valid
        COMMAND test_sensors_valid
)

add_test(
        NAME Calibration_Valid
        COMMAND test_calibration_valid
)

add_test(
        NAME Mismatch_Timer
        COMMAND test_mismatch_timer
)

add_test(
        NAME Throttle_Brake_Latch
        COMMAND test_throttle_brake_latch
)