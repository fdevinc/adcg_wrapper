add_executable(adcg_wrapper.function.example autodiff/function.example.cpp)
target_link_libraries(adcg_wrapper.function.example adcg_wrapper::adcg_wrapper)

# Add examples to the test targets if testing is enabled.
if(ADCG_WRAPPER_BUILD_TESTS)
  add_test(NAME adcg_wrapper.function.example
           COMMAND adcg_wrapper.function.example)
endif()
