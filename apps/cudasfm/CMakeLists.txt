add_executable(cudasfm cudasfm.cpp)
target_include_directories(cudasfm PRIVATE ${SHOMAGICK_INCLUDE_DIRS})
message("shomagick libraries are ${SHOMAGICK_LIBRARIES}")
target_link_libraries(cudasfm PUBLIC ${SHOMAGICK_LIBRARIES})

#target_link_libraries(cudasfm PRIVATE "/home/theartfuldodger/Documents/projects/shomagik/build/lib/libshomagick.a")