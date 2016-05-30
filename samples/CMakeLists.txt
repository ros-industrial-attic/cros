set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
file(COPY rosdb DESTINATION ${EXECUTABLE_OUTPUT_PATH})

add_executable (raw-api-test raw-api-test.c)
target_link_libraries(raw-api-test cros)

add_executable(parameters-test parameters-test.c)
target_link_libraries(parameters-test cros)

add_executable(api-test api-test.c)
target_link_libraries(api-test cros)
