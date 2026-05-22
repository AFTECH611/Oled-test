add_executable(oled_example main.cpp)

target_link_libraries(oled_example PRIVATE oled_driver)

target_compile_options(oled_example PRIVATE -Wall -Wextra -O2)