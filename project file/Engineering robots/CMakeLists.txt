cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME dx_assignment5_freertos)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
#
# 显式收集 .c/.cpp/.s：避免把各目录下的 README.md、参考工程或生成输出误当源文件。
# App/ 与 Config/ 是本轮新增的应用层与配置目录；CONFIGURE_DEPENDS 让新增文件
# 无需手动重新配置即可被 Ninja 发现。
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Bsp/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Bsp/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/User_Driver/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/User_Driver/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Task/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Task/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/App/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/App/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Config/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Config/*.cpp"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        App
        Config
        Bsp
        User_Driver
        User_Driver/Motor_Driver
        Task
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
