project(cros)

cmake_minimum_required (VERSION 2.8) 

cmake_policy(SET CMP0015 NEW)	

execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                OUTPUT_VARIABLE GCC_VERSION)

set(CMAKE_C_FLAGS "-Wall -Wno-unused -Wno-strict-aliasing -Wno-unused-but-set-variable")
if (GCC_VERSION VERSION_GREATER 4.8 OR GCC_VERSION VERSION_EQUAL 4.8)
  set(CMAKE_C_FLAGS_DEBUG "-g -pg -DDEBUG -fsanitize=address")
else()
  set(CMAKE_C_FLAGS_DEBUG "-g -pg -DDEBUG")
endif()

set(LINK_FLAGS_DEBUG_DEBUG "-pg")
set(CMAKE_C_FLAGS_RELEASE "-DNDEBUG -O1")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin) 

include_directories (include)
aux_source_directory(${PROJECT_SOURCE_DIR}/src CROSLIB_SRCS)

add_library(cros STATIC ${CROSLIB_SRCS} )

add_subdirectory(samples)

set_target_properties(cros PROPERTIES ARCHIVE_OUTPUT_DIRECTORY lib)

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
