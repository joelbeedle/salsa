
add_executable(
  salsa_test
  drone_test.cpp
  behaviour_registry_test.cpp
  parameter_test.cpp
  test_queue_test.cpp
  sim_test.cpp
  mock_behaviour.h
  mock_drone.h
)
target_link_libraries(salsa_test PRIVATE GTest::gtest_main GTest::gmock_main spdlog::spdlog nlohmann_json::nlohmann_json box2d salsa)

include(GoogleTest)
gtest_discover_tests(salsa_test)
