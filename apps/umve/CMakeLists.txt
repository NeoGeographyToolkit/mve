CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

set ( APP_NAME umve) 
project(${APP_NAME})

#####################################################################################
#
# Find other libraries specific to this project
#
#####################################################################################
# Find the QtWidgets library
find_package(OpenGL REQUIRED)
find_package(Qt5Widgets)
find_package(Qt5OpenGL REQUIRED)

#####################################################################################
#
# Source files 
#
#####################################################################################
#SET(${APP_NAME}_HEADERS 
#            			mainwindow.h glwidget.h)
#SET(${APP_NAME}_SOURCES main.cpp 
#						mainwindow.cpp glwidget.cpp )

# Source files 
file(GLOB SOURCES *.cc *.h scene_addins/*.cc scene_addins/*.h scene_inspect/*.cc scene_inspect/*.h viewinspect/*.cc viewinspect/*.h)

# Resources:
set(umve_resource_files
    "images/icon_about.svg"
    "images/icon_broken.svg"
    "images/icon_clean.svg"
    "images/icon_close.svg"
    "images/icon_copy.svg"
    "images/icon_delete.svg"
    "images/icon_exec.svg"
    "images/icon_exit.svg"
    "images/icon_export.svg"
    "images/icon_export_ply.svg"
    "images/icon_eye.svg"
    "images/icon_file.svg"
    "images/icon_folder.svg"
    "images/icon_image_inspect.svg"
    "images/icon_large_minus.png"
    "images/icon_large_plus.png"
    "images/icon_new_dir.svg"
    "images/icon_new_file.svg"
    "images/icon_open_file.svg"
    "images/icon_player_eject.svg"
    "images/icon_player_end.svg"
    "images/icon_player_fwd.svg"
    "images/icon_player_pause.svg"
    "images/icon_player_play.svg"
    "images/icon_player_record.svg"
    "images/icon_player_rew.svg"
    "images/icon_player_start.svg"
    "images/icon_player_stop.svg"
    "images/icon_refresh.svg"
    "images/icon_revert.svg"
    "images/icon_save.svg"
    "images/icon_screenshot.svg"
    "images/icon_small_minus.png"
    "images/icon_small_plus.png"
    "images/icon_toolbox.svg"
    "images/icon_video.svg"
    "images/icon_window.png"
    "images/icon_zoom_in.svg"
    "images/icon_zoom_out.svg"
    "images/icon_zoom_page.svg"
    "images/icon_zoom_reset.svg"
    "shaders/overlay_330.frag"
    "shaders/overlay_330.vert"
    "shaders/surface_330.frag"
    "shaders/surface_330.vert"
    "shaders/texture_330.frag"
    "shaders/texture_330.vert"
    "shaders/wireframe_330.frag"
    "shaders/wireframe_330.vert"
)

qt_add_resources(umve "umve"
    # PREFIX
    #     "/"
    FILES
        ${umve_resource_files}
)

SET(QT5_UIS )

#QT5_ADD_RESOURCES ( ${APP_NAME}_RCCS ${QT5_RCCS} )
QT5_WRAP_UI(UIS_HDRS
    ${QT5_UIS}
)

SET( SHADER_SOURCES 

)
SOURCE_GROUP(shaders REGULAR_EXPRESSION ".*/shaders/.*" )

# Additional dependencies
include_directories(../../libs)
include_directories(.)

# Need this to find libOpenGL on Linux
set(EXTRA_SEARCH_DIR "")
if (NOT APPLE)
  set(EXTRA_SEARCH_DIR "-L/usr/lib64")
endif()

add_executable(${APP_NAME} 
	MACOSX_BUNDLE
	WIN32
	${SOURCES} ${${APP_NAME}_RCCS} ${UIS_HDRS} 
	${SHADER_SOURCES})
 
TARGET_LINK_LIBRARIES ( ${APP_NAME}
    mveCore mveUtil mveDmrecon mveOgl
    ${OPENGL_LIBRARIES}
    Qt5::Widgets
	Qt5::OpenGL
    ${EXTRA_SEARCH_DIR}
)

# Create the launcher for VS2010
if (MSVC_IDE)
# Set ENVIRONMENT. This will make VS debugger to refer qt5 related DLLs properly
create_target_launcher(${APP_NAME} WORKING_DIRECTORY "${APP_BIN_DIR}" ENVIRONMENT  "PATH=${QTDIR}/bin;%PATH%")
#create_target_launcher(sample1 ENVIRONMENT  ${RUN_ENV})
endif(MSVC_IDE)
set_target_properties(${APP_NAME} PROPERTIES XCODE_ATTRIBUTE_CONFIGURATION_BUILD_DIR "${APP_BIN_DIR}")


if(WIN32 AND MSVC)
  add_custom_command(TARGET ${APP_NAME} PRE_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_directory
                         ${CMAKE_CURRENT_SOURCE_DIR}/shaders ${APP_BIN_DIR})

  add_custom_command(TARGET ${APP_NAME} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different 
                     "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/${APP_NAME}${CMAKE_EXECUTABLE_SUFFIX}" 
					 "${APP_BIN_DIR}")
endif(WIN32 AND MSVC)


