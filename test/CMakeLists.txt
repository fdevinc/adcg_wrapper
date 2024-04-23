include(CTest)

add_executable(adcg_wrapper.function.test autodiff/function.test.cpp)
target_link_libraries(adcg_wrapper.function.test adcg_wrapper::adcg_wrapper
                      GTest::gtest)
add_test(NAME adcg_wrapper.function.test COMMAND adcg_wrapper.function.test)
