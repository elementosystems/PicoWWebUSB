# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==

add_compile_definitions(CFG_TUSB_MCU=OPT_MCU_RP2040)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")
set(FREERTOS_KERNEL_PATH "${USERHOME}/FreeRTOS-Kernel" CACHE PATH "Path to the FreeRTOS Kernel")
# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(PicoWWEBUSB C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

#add_subdirectory(tinyusb/src)
# Add executable. Default name is the project name, version 0.1
SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/src/configs/FreeRTOS-Kernel" CACHE STRING "Local Config")
 # FreeRTOS config files

add_executable(PicoWWEBUSB src/PicoWWEBUSB.c 
                            src/usb_descriptors.c
                            src/ssd1306.c)
#tinyusb_target_add(PicoWWEBUSB)

pico_set_program_name(PicoWWEBUSB "PicoWWEBUSB")
pico_set_program_version(PicoWWEBUSB "0.2.0")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(PicoWWEBUSB 0)
pico_enable_stdio_usb(PicoWWEBUSB 0)

# Add the standard library to the build
target_link_libraries(PicoWWEBUSB
        pico_stdlib
        tinyusb_board
        tinyusb_device
        pico_cyw43_arch_none        
        hardware_adc
        hardware_i2c
        FREERTOS_PORT
        FreeRTOS-Kernel-Heap4
        )

# Add the standard include files to the build
target_include_directories(PicoWWEBUSB PRIVATE BEFORE
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/tinyusb/src
        ${FREERTOS_CONFIG_FILE_DIRECTORY}
)

# Add any user requested libraries
# target_link_libraries(PicoWWEBUSB 
#         pico_cyw43_arch_none
#         pico_stdlib
#         FreeRTOS
#         )

pico_add_extra_outputs(PicoWWEBUSB)
