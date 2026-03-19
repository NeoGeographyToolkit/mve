CMAKE_MINIMUM_REQUIRED(VERSION 3.15)

set (APP_NAME sfm_view)
project(${APP_NAME})

set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

file(GLOB SOURCES *.cc *.h)

include_directories(.)

add_executable(${APP_NAME} ${SOURCES})

target_link_libraries(${APP_NAME}
  Qt5::Widgets
  Qt5::OpenGL
)

if (LINUX)
  find_package(X11 REQUIRED)
  target_link_libraries(${APP_NAME} ${X11_LIBRARIES})
endif()

# Set RPATHS
if(APPLE)
  set_target_properties(${APP_NAME} PROPERTIES
    INSTALL_RPATH "@loader_path;@loader_path/../lib;${MULTIVIEW_DEPS_DIR}/lib")
elseif(UNIX)
  set_target_properties(${APP_NAME} PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MULTIVIEW_DEPS_DIR}/lib")
endif()

install(TARGETS ${APP_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
