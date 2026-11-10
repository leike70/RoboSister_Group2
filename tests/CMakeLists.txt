cmake_minimum_required(VERSION 3.22)
project(FrameworkRegressions C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
enable_testing()
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

foreach(vehicle IN ITEMS engineer turret)
    if(vehicle STREQUAL "engineer")
        set(vehicle_dir "${REPO_ROOT}/project file/Engineering robots")
        set(vehicle_sources
            "${vehicle_dir}/App/Arm/app_arm.c"
            "${vehicle_dir}/App/Arm/app_arm_task.c"
            "${vehicle_dir}/App/Arm/app_arm_kinematics.c"
            "${vehicle_dir}/App/Vehicle/app_vehicle.c")
    else()
        set(vehicle_dir "${REPO_ROOT}/project file/Turret Robot")
        set(vehicle_sources
            "${vehicle_dir}/App/Gimbal/app_gimbal.c"
            "${vehicle_dir}/App/Shooting/app_shooting.c"
            "${vehicle_dir}/App/Vehicle/app_vehicle.c")
    endif()
    add_executable(regression_${vehicle}
        framework_regression.c
        "${vehicle_dir}/App/Common/app_types.c"
        "${vehicle_dir}/App/Remote/app_remote.c"
        "${vehicle_dir}/User_Driver/Remote/dbus.c"
        "${vehicle_dir}/Bsp/remote_uart.c"
        "${vehicle_dir}/App/Attitude/app_attitude.c"
        "${vehicle_dir}/App/Protection/app_protection.c"
        ${vehicle_sources})
    target_include_directories(regression_${vehicle} PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/uart_stub"
        "${vehicle_dir}/Bsp"
        "${vehicle_dir}/App" "${vehicle_dir}/Config" "${vehicle_dir}/User_Driver")
    if(vehicle STREQUAL "engineer")
        target_compile_definitions(regression_${vehicle} PRIVATE TEST_ENGINEER)
    endif()
    target_compile_options(regression_${vehicle} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${vehicle}_regressions COMMAND regression_${vehicle})
    # Compile the actual independent IMU implementation from each vehicle.
    add_executable(imu_${vehicle}_regressions
        imu_regression.c
        "${vehicle_dir}/User_Driver/IMU/bmi088.c"
        "${vehicle_dir}/User_Driver/IMU/imu_fusion.c"
        "${vehicle_dir}/User_Driver/IMU/imu_matrix.c"
        "${vehicle_dir}/User_Driver/IMU/kalman_filter.c"
        "${vehicle_dir}/User_Driver/IMU/QuaternionEKF.c"
        "${vehicle_dir}/Config/imu_config.c"
        "${vehicle_dir}/App/Attitude/app_attitude.c")
    target_include_directories(imu_${vehicle}_regressions PRIVATE
        "${vehicle_dir}/User_Driver" "${vehicle_dir}/Config" "${vehicle_dir}/App")
    target_compile_options(imu_${vehicle}_regressions PRIVATE -Wall -Wextra -Werror)
    add_test(NAME imu_${vehicle}_regressions COMMAND imu_${vehicle}_regressions)
endforeach()

add_executable(regression_critical critical_regression.c
    "${REPO_ROOT}/project file/Engineering robots/App/Common/app_critical.c")
target_include_directories(regression_critical PRIVATE cmsis_stub
    "${REPO_ROOT}/project file/Engineering robots/App")
target_compile_options(regression_critical PRIVATE -Wall -Wextra -Werror)
add_test(NAME critical_mask_regressions COMMAND regression_critical)

add_executable(imu_bsp_regressions imu_bsp_regression.c
    "${REPO_ROOT}/project file/Engineering robots/Bsp/imu_bsp.c")
target_include_directories(imu_bsp_regressions PRIVATE imu_stub
    "${REPO_ROOT}/project file/Engineering robots/Bsp"
    "${REPO_ROOT}/project file/Engineering robots/User_Driver")
target_compile_options(imu_bsp_regressions PRIVATE -Wall -Wextra -Werror)
add_test(NAME imu_bsp_regressions COMMAND imu_bsp_regressions)

add_executable(imu_thermal_regressions imu_thermal_regression.c
    "${REPO_ROOT}/project file/Engineering robots/User_Driver/IMU/imu_temperature.c"
    "${REPO_ROOT}/project file/Engineering robots/Bsp/imu_heater.c")
target_include_directories(imu_thermal_regressions PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/tim_stub"
    "${REPO_ROOT}/project file/Engineering robots/User_Driver"
    "${REPO_ROOT}/project file/Engineering robots/Bsp")
target_compile_options(imu_thermal_regressions PRIVATE -Wall -Wextra -Werror)
add_test(NAME imu_thermal_regressions COMMAND imu_thermal_regressions)

# CAN/DJI production code with HAL+RTOS boundary stubs. Do not replace the BSP or
# the DJI lifecycle implementation with a fixed-success fake.
add_executable(can_dji_regressions
    can_dji_regression.cpp
    can_stub/can_stub.c
    "${REPO_ROOT}/project file/Engineering robots/Bsp/CAN_bsp.cpp"
    "${REPO_ROOT}/project file/Engineering robots/User_Driver/Motor_Driver/DJI_motor_driver.cpp"
    "${REPO_ROOT}/project file/Engineering robots/User_Driver/Motor_Driver/motor_driver.cpp"
    "${REPO_ROOT}/project file/Engineering robots/User_Driver/pid_driver.cpp")
target_include_directories(can_dji_regressions PRIVATE
    can_stub
    "${REPO_ROOT}/project file/Engineering robots/Bsp"
    "${REPO_ROOT}/project file/Engineering robots/App"
    "${REPO_ROOT}/project file/Engineering robots/Config"
    "${REPO_ROOT}/project file/Engineering robots/User_Driver"
    "${REPO_ROOT}/project file/Engineering robots/User_Driver/Motor_Driver")
target_compile_options(can_dji_regressions PRIVATE -Wall -Wextra -Wno-reorder -Wno-invalid-offsetof)
add_test(NAME can_dji_regressions COMMAND can_dji_regressions)

# Real Config/Motor/Power/Protection implementation, no fixed-success stubs.
add_executable(production_chain_regressions
    production_chain_regression.c
    "${REPO_ROOT}/project file/Engineering robots/App/Common/app_types.c"
    "${REPO_ROOT}/project file/Engineering robots/Config/app_config.c"
    "${REPO_ROOT}/project file/Engineering robots/App/Motor/app_motor_registry.c"
    "${REPO_ROOT}/project file/Engineering robots/App/Motor/app_motor_output.c"
    "${REPO_ROOT}/project file/Engineering robots/App/Motor/app_unit.c"
    "${REPO_ROOT}/project file/Engineering robots/App/Power/app_power.c"
    "${REPO_ROOT}/project file/Engineering robots/App/Protection/app_protection.c"
    power_stub/power_runtime_stub.c)
target_include_directories(production_chain_regressions PRIVATE
    cmsis_stub power_stub
    "${REPO_ROOT}/project file/Engineering robots/App"
    "${REPO_ROOT}/project file/Engineering robots/Config")
target_compile_options(production_chain_regressions PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(production_chain_regressions PRIVATE m)
add_test(NAME production_chain_regressions COMMAND production_chain_regressions)

add_executable(power_motor_contract_regressions
    power_motor_contract_regression.c
    "${REPO_ROOT}/project file/Engineering robots/App/Common/app_types.c"
    "${REPO_ROOT}/project file/Engineering robots/App/Motor/app_motor_registry.c"
    "${REPO_ROOT}/project file/Engineering robots/App/Motor/app_motor_output.c"
    "${REPO_ROOT}/project file/Engineering robots/App/Motor/app_unit.c"
    "${REPO_ROOT}/project file/Engineering robots/App/Power/app_power.c"
    power_stub/power_runtime_stub.c)
target_include_directories(power_motor_contract_regressions PRIVATE
    cmsis_stub power_stub
    "${REPO_ROOT}/project file/Engineering robots/App"
    "${REPO_ROOT}/project file/Engineering robots/Config")
target_compile_definitions(power_motor_contract_regressions PRIVATE APP_POWER_TEST)
target_compile_options(power_motor_contract_regressions PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(power_motor_contract_regressions PRIVATE m)
add_test(NAME power_motor_contract_regressions COMMAND power_motor_contract_regressions)

foreach(vehicle IN ITEMS engineer turret)
    if(vehicle STREQUAL "engineer")
        set(vehicle_dir "${REPO_ROOT}/project file/Engineering robots")
    else()
        set(vehicle_dir "${REPO_ROOT}/project file/Turret Robot")
    endif()
    add_executable(config_power_${vehicle}
        config_power_limits_regression.c
        "${vehicle_dir}/Config/app_config.c"
        "${vehicle_dir}/App/Common/app_types.c")
    target_include_directories(config_power_${vehicle} PRIVATE
        cmsis_stub
        "${vehicle_dir}/App" "${vehicle_dir}/Config")
    if(vehicle STREQUAL "turret")
        target_compile_definitions(config_power_${vehicle} PRIVATE TEST_TURRET)
    endif()
    target_compile_options(config_power_${vehicle} PRIVATE -Wall -Wextra)
    add_test(NAME config_power_${vehicle}_regressions COMMAND config_power_${vehicle})
endforeach()
