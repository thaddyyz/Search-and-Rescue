cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorial)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(

)

include_directories(

  ${catkin_INCLUDE_DIRS}
)
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(controlsub src/controlsub.cpp)
target_link_libraries(controlsub ${catkin_LIBRARIES})
add_dependencies(controlsub beginner_tutorials_generate_messages_cpp)

add_executable(pubsub src/pubsub.cpp)
target_link_libraries(pubsub ${catkin_LIBRARIES})
add_dependencies(pubsub beginner_tutorials_generate_messages_cpp)

add_executable(alexpi src/alexpi.cpp)
target_link_libraries(alexpi ${catkin_LIBRARIES})
add_dependencies(alexpi beginner_tutorials_generate_messages_cpp)


add_executable(teleop_turtle src/teleop_turtle.cpp)
target_link_libraries(teleop_turtle ${catkin_LIBRARIES})
add_dependencies(teleop_turtle beginner_tutorials_generate_messages_cpp)