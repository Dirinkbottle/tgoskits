cmake_minimum_required(VERSION 3.20)
project(bug-prctl-get-auxv C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_executable(bug-prctl-get-auxv src/main.c)
target_compile_options(bug-prctl-get-auxv PRIVATE -Wall -Wextra -Werror)

install(TARGETS bug-prctl-get-auxv RUNTIME DESTINATION usr/bin/starry-test-suit)
