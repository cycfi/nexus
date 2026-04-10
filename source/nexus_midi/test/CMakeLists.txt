add_executable(test_dynamic_smoother test_dynamic_smoother.cpp)

target_include_directories(test_dynamic_smoother PRIVATE
    mock                                          # provides energia.h stub
    ${CMAKE_SOURCE_DIR}                           # provides util.hpp
)

add_test(NAME dynamic_smoother COMMAND test_dynamic_smoother)

add_executable(test_gate test_gate.cpp)

target_include_directories(test_gate PRIVATE
    mock                                          # provides energia.h stub
    ${CMAKE_SOURCE_DIR}                           # provides util.hpp
)

add_test(NAME gate COMMAND test_gate)
