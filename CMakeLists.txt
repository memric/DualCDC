cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE toolchain.cmake)

###################### VARIABLES ######################################
set (PROJECT_NAME             "DualCDC")
set (LINKER_SCRIPT            "../STM32F103C8TX_FLASH.ld")
set (MCPU                     "-mcpu=cortex-m3")
set (MFPU                 	  "")
set (MFLOAT_ABI               "-mfloat-abi=soft")
set (RUNTIME_LIBRARY          "--specs=nano.specs")
set (RUNTIME_LIBRARY_SYSCALLS "--specs=nosys.specs")

if(CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_CONFIGURATION_TYPES Debug Release)
  set(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES}" CACHE STRING
    "Reset the configurations to what we need"
    FORCE)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(WERR "")
else()
  # all warnings being treated as errors
  set(WERR "-Werror")
endif()

set (PROJECT_SOURCES
	# LIST SOURCE FILES HERE
	Core/Startup/startup_stm32f103c8tx.s
	Core/Src/main.c
	Core/Src/ring_buf.c
	Core/Src/stm32f1xx_hal_msp.c
	Core/Src/stm32f1xx_it.c
	Core/Src/syscalls.c
	Core/Src/sysmem.c
	Core/Src/system_stm32f1xx.c
	Core/Src/us_timer.c
	USB_DEVICE/App/usb_device.c
	USB_DEVICE/App/usbd_cdc_if.c
	USB_DEVICE/App/usbd_desc.c
	USB_DEVICE/Target/usbd_conf.c
	)

set (PROJECT_DEFINES
	# LIST COMPILER DEFINITIONS HERE
	# USE_FULL_LL_DRIVER
	USE_HAL_DRIVER
	STM32F103xB
	$<$<CONFIG:Debug>:DEBUG>
	USE_IWDG=$<IF:$<CONFIG:Debug>,0,1>
	# HSE_VALUE=8000000
	# HSI_VALUE=8000000
	# HSE_STARTUP_TIMEOUT=100
	# LSE_STARTUP_TIMEOUT=5000
	# LSE_VALUE=32768
	# LSI_VALUE=40000
	# VDD_VALUE=3300
	# PREFETCH_ENABLE=1
	# INSTRUCTION_CACHE_ENABLE=0
	# DATA_CACHE_ENABLE=0
    )

set (PROJECT_INCLUDES
	# LIST INCLUDE DIRECTORIES HERE
	Core/Inc 
	Drivers/STM32F1xx_HAL_Driver/Inc 
	Drivers/CMSIS/Device/ST/STM32F1xx/Include 
	Drivers/CMSIS/Include
	USB_DEVICE/App
	USB_DEVICE/Target
    )

############ MODIFY ACCORDING TO REQUIREMENTS) ########################

#######################################################################

################## PROJECT SETUP ######################################
project(${PROJECT_NAME} VERSION 0.0.0)
enable_language(ASM)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)

add_compile_definitions (${PROJECT_DEFINES})
include_directories (${PROJECT_INCLUDES})

set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g3 -Og")

set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall ${WERR} -fdata-sections -ffunction-sections -fstack-usage")
set (CMAKE_EXECUTABLE_SUFFIX ".elf")
set (CMAKE_STATIC_LIBRARY_SUFFIX ".a")
set (CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} ${RUNTIME_LIBRARY_SYSCALLS} -Wl,-Map=test.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group -Wl,--no-warn-rwx-segment -Wl,--print-memory-usage")
set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")

add_subdirectory("Drivers/STM32F1xx_HAL_Driver")
target_link_libraries(${PROJECT_NAME} PRIVATE HAL)

add_subdirectory("Middlewares/ST/STM32_USB_Device_Library")
target_link_libraries(${PROJECT_NAME} PRIVATE USB_Library)

