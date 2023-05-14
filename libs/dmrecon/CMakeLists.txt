file (GLOB HEADERS "*.h")
file (GLOB SOURCES "[^_]*.cc")

set(LIBRARY mveDmrecon)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")

add_library(${LIBRARY} SHARED ${SOURCES})

target_link_libraries(${LIBRARY} mveCore mveUtil)

target_compile_options(${LIBRARY} PRIVATE -fPIC)

# Set RPATHS 
if(APPLE)
  set_target_properties(mveDmrecon PROPERTIES
    INSTALL_RPATH "@loader_path;@loader_path/../lib;${MULTIVIEW_DEPS_DIR}/lib")
elseif(UNIX) # Unix which is not Apple
  set_target_properties(mveDmrecon PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MULTIVIEW_DEPS_DIR}/lib")
endif()

# Install the lib
install(TARGETS mveDmrecon LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
