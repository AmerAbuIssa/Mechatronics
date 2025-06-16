#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
enable_testing()
###################################

# Just in case someone had the include files in seperate directory
include_directories(../include)
include_directories(..)


# ACKERMAN TEST
add_executable(testAudiReachGoals test_audi_reach_goals.cpp)
target_link_libraries(testAudiReachGoals ${GTEST_LIBRARIES} pfmshog controller)

# QUADCOPTER TEST
add_executable(testQuadcopterReachGoals test_quad_reach_goals.cpp)
target_link_libraries(testQuadcopterReachGoals ${GTEST_LIBRARIES} controller pfmshog)

# MISSION TEST
add_executable(testMissionReachGoals test_mission_reach_goals.cpp)
target_link_libraries(testMissionReachGoals ${GTEST_LIBRARIES} controller mission pfmshog)

# TSP Ackerman
add_executable(testTspReachGoalsAck tsp_reach_goals_ack.cpp)
target_link_libraries(testTspReachGoalsAck ${GTEST_LIBRARIES} controller mission pfmshog)

# TSP Quadcopter
add_executable(testTspReachGoalsQuad tsp_reach_goals_quad.cpp)
target_link_libraries(testTspReachGoalsQuad ${GTEST_LIBRARIES} controller mission pfmshog)

