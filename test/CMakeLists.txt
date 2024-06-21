
add_executable(
  swarm_test
  drone_test.cpp
  behaviour_registry_test.cpp
  parameter_test.cpp
  test_queue_test.cpp
  sim_test.cpp
  mock_behaviour.h
  mock_drone.h
)
target_link_libraries(swarm_test PRIVATE GTest::gtest_main GTest::gmock_main box2d swarm-sim)

include(GoogleTest)
gtest_discover_tests(swarm_test)
