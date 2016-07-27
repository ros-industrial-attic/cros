file(COPY rosdb DESTINATION ${EXECUTABLE_OUTPUT_PATH})

add_executable (raw-api-test raw-api-test.c)
target_link_libraries(raw-api-test cros)

add_executable(parameters-test parameters-test.c)
target_link_libraries(parameters-test cros)

add_executable(api-test api-test.c)
target_link_libraries(api-test cros)

add_executable(ros-i-trajectory-test ros-i-trajectory-test.c)
target_link_libraries(ros-i-trajectory-test cros)

add_executable(talker talker.c)
target_link_libraries(talker cros)

add_executable(listener listener.c)
target_link_libraries(listener cros)
