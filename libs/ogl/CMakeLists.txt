file (GLOB HEADERS "*.h")
file (GLOB SOURCES "[^_]*.cc")

set(LIBRARY mveOgl)

find_package(OpenGL REQUIRED)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")

# Need this to find libOpenGL
set(EXTRA_SEARCH_DIR "")
if (NOT APPLE)
  set(EXTRA_SEARCH_DIR "-L/usr/lib64")
else()
  set(EXTRA_SEARCH_DIR "-lOSMesa")
endif()

add_library(${LIBRARY} SHARED ${SOURCES})

target_link_libraries(${LIBRARY} mveCore mveUtil
  ${OPENGL_LIBRARIES}
  ${EXTRA_SEARCH_DIR}
)

target_compile_options(${LIBRARY} PRIVATE -fPIC)

# Set RPATHS 
if(APPLE)
  set_target_properties(mveOgl PROPERTIES
    INSTALL_RPATH "@loader_path;@loader_path/../lib;${MULTIVIEW_DEPS_DIR}/lib")
elseif(UNIX) # Unix which is not Apple
  set_target_properties(mveOgl PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MULTIVIEW_DEPS_DIR}/lib")
endif()

# Install the lib
install(TARGETS mveOgl LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
