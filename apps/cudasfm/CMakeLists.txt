add_executable(cudasfm cudasfm.cpp)
find_package(
	OpenCV 
	REQUIRED 
	COMPONENTS
		core 
		imgproc
		highgui 
		calib3d 
		sfm 
		viz 
		imgcodecs
		features2d 
		xfeatures2d 
	OPTIONAL_COMPONENTS
		cudafeatures2d
		cudaimgproc 
)

if (CUDA_FOUND)
	#cuda_add_executable(cudasfm cudasfm.cpp)
	target_link_libraries (cudasfm ${CUDA_LIBRARIES} ${CUDA_cudadevrt_LIBRARY})
	target_include_directories(cudasfm PRIVATE ${CUDA_INCLUDE_DIRS})
else()
	#add_executable(cudasfm cudasfm.cpp)
endif (CUDA_FOUND)
target_include_directories(cudasfm PRIVATE ${SHOMAGICK_INCLUDE_DIRS})
target_link_libraries(cudasfm ${SHOMAGICK_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(cudasfm ${GFLAGS_LIBRARIES})


get_target_property(LINKED_LIBS cudasfm LINK_LIBRARIES)
message(STATUS "Libraries linked to my_program: ${LINKED_LIBS}")

get_target_property(LINK_DIRECTORIES cudasfm INTERFACE_LINK_DIRECTORIES)

if (LINK_DIRECTORIES)
    message(STATUS "Link directories for my_program: ${LINK_DIRECTORIES}")
else()
    message(STATUS "No link directories set for my_program")
endif()


target_compile_features(cudasfm PRIVATE cxx_std_17)