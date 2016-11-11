###############################################
# apps/gestureIK
file(GLOB gestureIK_srcs "*.cpp")
file(GLOB gestureIK_hdrs "*.h")
add_executable(gestureIK ${gestureIK_srcs} ${gestureIK_hdrs})
target_link_libraries(gestureIK dart)
set_target_properties(gestureIK PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
