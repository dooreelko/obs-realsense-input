project(mac-input-realsense VERSION 1.0.0)

set(PLUGIN_AUTHOR "Dooreelko")

find_path(RS_DIR
	NAMES include/librealsense2/rs.h
)

set(mac-input-realsense_SOURCES
	mac-input-realsense.cpp
)

add_library(mac-input-realsense MODULE
	${mac-input-realsense_SOURCES})
target_link_libraries(mac-input-realsense
	libobs
	${mac-input-realsense_PLATFORM_DEPS})
set_target_properties(mac-input-realsense PROPERTIES FOLDER "plugins")

target_include_directories(mac-input-realsense SYSTEM PRIVATE ${RS_DIR}/include)
target_link_libraries(mac-input-realsense ${RS_DIR}/lib/librealsense2.dylib)

install_obs_plugin_with_data(mac-input-realsense data)
