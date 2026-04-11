add_executable(test_dynamic_smoother test_dynamic_smoother.cpp)

target_include_directories(test_dynamic_smoother PRIVATE
    mock                                          # provides energia.h stub
    ${CMAKE_CURRENT_LIST_DIR}/..                  # provides util.hpp
)

add_test(NAME dynamic_smoother COMMAND test_dynamic_smoother)
