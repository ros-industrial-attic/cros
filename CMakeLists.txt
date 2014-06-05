project(cros)

cmake_minimum_required (VERSION 2.6) 

cmake_policy(SET CMP0015 NEW)	

set(CMAKE_C_FLAGS " -Wall -Wno-unused -Wno-strict-aliasing -Wno-unused-but-set-variable")
set(CMAKE_C_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-DNDEBUG -O1")

include_directories ( include )
#link_directories ( ${OpenCV_LIB_DIR} )
aux_source_directory(${PROJECT_SOURCE_DIR}/src CROSLIB_SRCS)

add_library(cros STATIC ${CROSLIB_SRCS} )

add_executable( test_node test_node.c )
target_link_libraries( test_node cros )

add_executable( greek_node greek_node.c )
target_link_libraries( greek_node cros )

set_target_properties(cros PROPERTIES ARCHIVE_OUTPUT_DIRECTORY lib)
set_target_properties(test_node PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin)
set_target_properties(greek_node PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin)

find_package(Doxygen)
if (NOT DOXYGEN_FOUND)
  message(ERROR "Doxygen is needed to build the documentation. Please install it correctly")
else()

  configure_file(Doxyfile ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE)
  #-- Add a custom target to run Doxygen when ever the project is built
  add_custom_target (docs COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
                          WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                          COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()
