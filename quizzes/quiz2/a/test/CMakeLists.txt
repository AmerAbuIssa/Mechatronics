#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
enable_testing()
###################################

# Add the unit test executable against the source code of the test
add_executable(utest utest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(utest GTest::GTest GTest::Main analysis ${OpenCV_LIBS} car)
# We can finally add the test to a list of tests to be run with make test
add_test(Q2A utest)
