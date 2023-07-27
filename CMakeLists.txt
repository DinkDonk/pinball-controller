cmake_minimum_required(VERSION 3.12)

set(APP_NAME icm-20948)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include the subsidiary .cmake file to get the SDK
include(pico_sdk_import.cmake)

# Set the name and version of the project
project(${APP_NAME} VERSION 1.0.0)

set(CMAKE_C_STANDARD 11)

# Initalise the SDK
pico_sdk_init()

# Link the project to source files
add_executable(${APP_NAME}
	${CMAKE_CURRENT_LIST_DIR}/main.c
	${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
	# ${CMAKE_CURRENT_LIST_DIR}/xinput_driver.c
	# ${CMAKE_CURRENT_LIST_DIR}/xinput_pad.c
	# ${CMAKE_CURRENT_LIST_DIR}/tusb_driver.c
	)

target_include_directories(${APP_NAME} PUBLIC
	${CMAKE_CURRENT_LIST_DIR}
	)

# Link the Project to libraries
target_link_libraries(${APP_NAME}
	pico_stdlib
	pico_multicore
	hardware_gpio
	hardware_i2c
	tinyusb_device
	tinyusb_board
	)

# Get ALL the warnings!
set_target_properties(${APP_NAME} PROPERTIES COMPILE_FLAGS "-Wall")

# Set optimization to minimize size
# target_compile_options(${APP_NAME} PRIVATE "-Os")
# target_link_options(${APP_NAME} PRIVATE "-Os")

# Enable USB, UART output
pico_enable_stdio_usb(${APP_NAME} 0)
pico_enable_stdio_uart(${APP_NAME} 0)

# Enable extra outputs
pico_add_extra_outputs(${APP_NAME})

# Generate `make flash` command
add_custom_target(flash
	COMMAND picotool load ${APP_NAME}.bin -f
	COMMENT "Loading binary onto device..."
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	)

add_custom_target(ccls
	COMMAND sh -c "jq --raw-output '.[].command' ${CMAKE_BINARY_DIR}/compile_commands.json | tr ' ' '\\n' | awk '\$1 ~ /arm-none-eabi-gcc/ || /^-D/ || /^-I/ || /^-f/ || /^-m/ || /^-std/' | awk '!seen[\$0]++' > .ccls && ${CMAKE_C_COMPILER} ${CMAKE_C_FLAGS} -xc /dev/null -E -Wp,-v 2>&1 | sed -n 's,^ ,,p' | sed 's/^/-I/' >> .ccls"
	COMMENT "Generatinf .ccls file..."
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	USES_TERMINAL
	)
