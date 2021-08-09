project(mac-input-realsense VERSION 1.0.0)

set(PLUGIN_AUTHOR "Dooreelko")

find_path(RS_DIR
	NAMES include/librealsense2/rs.h
)

set(mac-input-realsense_SOURCES
	cam-source.hpp
	mac-input-realsense.cpp
)

add_library(mac-input-realsense MODULE
	${mac-input-realsense_SOURCES})
target_link_libraries(mac-input-realsense
	libobs
	${mac-input-realsense_PLATFORM_DEPS})
set_target_properties(mac-input-realsense PROPERTIES FOLDER "plugins")

find_package(OpenGL REQUIRED)

target_include_directories(mac-input-realsense SYSTEM PRIVATE ${RS_DIR}/include ${OPENGL_INCLUDE_DIR})
target_link_libraries(mac-input-realsense ${RS_DIR}/lib/librealsense2.dylib  ${OPENGL_LIBRARIES} )

set(DEPENDENCIES realsense2 glfw ${OPENGL_LIBRARIES})

install_obs_plugin_with_data(mac-input-realsense data)
