
include_directories(
  SYSTEM ${PCL_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}

)


add_executable(show_objects_kitti show_objects_kitti.cpp)
target_link_libraries(show_objects_kitti
                      identifiable
                      image_labeler
                      velodyne_utils
                      projections
                      ground_remove
                      folder_reader
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                     ${CMAKE_THREAD_LIBS_INIT})



add_executable(show_objects_png show_objects_png.cpp)
target_link_libraries(show_objects_png

                      identifiable
                      image_labeler
                      velodyne_utils
                      projections
                      ground_remove
                      folder_reader
#                      visualization
#                      ${MY_QT_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${CMAKE_THREAD_LIBS_INIT})
