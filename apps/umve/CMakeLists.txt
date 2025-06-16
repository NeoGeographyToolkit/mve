CMAKE_MINIMUM_REQUIRED(VERSION 3.15)

set (APP_NAME sfm_view)
project(${APP_NAME})

set(CMAKE_AUTOMOC ON)

add_definitions(-DGL_GLEXT_PROTOTYPES) # Need this to make OpenGL compile

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

# SET(QT5_UIS )

qt_add_resources(RCC_SOURCES umve.qrc)

#QT5_ADD_RESOURCES ( ${APP_NAME}_RCCS ${umve_resource_files})
# QT5_WRAP_UI(UIS_HDRS
#     ${QT5_UIS}
# )

# Additional dependencies
include_directories(../../libs)
include_directories(.)

# Need this to find OpenGL
set(GL_OPT "")
if (NOT APPLE)
  set(GL_OPT "-L/usr/lib64")
  else()
  set(GL_OPT "-lOSMesa")
endif()

add_executable(${APP_NAME}
  WIN32
  ${SOURCES} ${${APP_NAME}_RCCS} ${UIS_HDRS}
  ${SHADER_SOURCES}
  ${RCC_SOURCES})

TARGET_LINK_LIBRARIES (${APP_NAME}
  mveCore mveUtil mveDmrecon mveOgl
  ${OPENGL_LIBRARIES}
  Qt5::Widgets
  Qt5::OpenGL
  ${GL_OPT}
)

if (LINUX)
  find_package(X11 REQUIRED)
  target_link_libraries(${APP_NAME} ${X11_LIBRARIES})
endif()

# Create the launcher for VS2010
if (MSVC_IDE)
  # Set ENVIRONMENT. This will make VS debugger to refer qt5 related DLLs properly
  create_target_launcher(${APP_NAME} WORKING_DIRECTORY "${APP_BIN_DIR}" ENVIRONMENT  "PATH=${QTDIR}/bin;%PATH%")
  #create_target_launcher(sample1 ENVIRONMENT  ${RUN_ENV})
endif(MSVC_IDE)

set_target_properties(${APP_NAME} PROPERTIES XCODE_ATTRIBUTE_CONFIGURATION_BUILD_DIR
    "${APP_BIN_DIR}")

if(WIN32 AND MSVC)
  add_custom_command(TARGET ${APP_NAME} PRE_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_directory
                         ${CMAKE_CURRENT_SOURCE_DIR}/shaders ${APP_BIN_DIR})

  add_custom_command(TARGET ${APP_NAME} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/${APP_NAME}${CMAKE_EXECUTABLE_SUFFIX}"
					 "${APP_BIN_DIR}")
endif(WIN32 AND MSVC)

# Set RPATHS
if(APPLE)
  set_target_properties(${APP_NAME} PROPERTIES
    INSTALL_RPATH "@loader_path;@loader_path/../lib;${MULTIVIEW_DEPS_DIR}/lib")
elseif(UNIX) # Unix which is not Apple
  set_target_properties(${APP_NAME} PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MULTIVIEW_DEPS_DIR}/lib")
endif()

# Install
install(TARGETS ${APP_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Install shaders. These will need to be shipped. See scene_addins/addin_state.cc.
install(DIRECTORY "shaders" DESTINATION ${CMAKE_INSTALL_PREFIX}/share/sfm_view)
