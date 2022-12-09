add_library(msf_timing Timer.cc)

add_executable(msf_timing_tests test/testMSFTiming.cc)
target_link_libraries(msf_timing_tests msf_timing Threads::Threads ${GTEST_BOTH_LIBRARIES})