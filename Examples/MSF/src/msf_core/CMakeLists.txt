add_library(msf_core msf_tools.cc falsecolor.cc
        gps_conversion.cc config.cpp)
target_link_libraries(msf_core ${YAML_CPP_LIBRARIES})

add_library(similaritytransform similaritytransform.cc)

add_executable(test_similaritytransform test/test_similaritytransform.cc)
target_link_libraries(test_similaritytransform similaritytransform)
#
#add_executable(test_static_statelist test/test_staticstatelist.cc)
#target_link_libraries(test_static_statelist pthread msf_core)
