
add_executable(
  swarm_test
  drone_test.cpp
)
target_link_libraries(swarm_test PRIVATE GTest::gtest_main GTest::gmock_main box2d libswarm-sim)

include(GoogleTest)
gtest_discover_tests(swarm_test)
