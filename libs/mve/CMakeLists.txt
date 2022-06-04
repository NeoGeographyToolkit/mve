file (GLOB HEADERS "*.h")
file (GLOB SOURCES "[^_]*.cc")

set(LIBRARY mve)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")

add_library(${LIBRARY} STATIC ${SOURCES})

target_link_libraries(${LIBRARY} mve_utils ${JPEG_LIBRARIES} ${PNG_LIBRARIES} ${TIFF_LIBRARIES})

target_compile_options(${LIBRARY} PRIVATE -fPIC)


