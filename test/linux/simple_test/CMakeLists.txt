
set(SOURCES simple_test.c)
add_executable(simple_test ${SOURCES})
target_link_libraries(simple_test soem)
target_link_libraries(simple_test 
                      /lib/x86_64-linux-gnu/libm.so)
#link /usr/lib
target_link_directories(simple_test PUBLIC 
                        /usr/lib
                        /lib/x86_64-linux-gnu)
install(TARGETS simple_test DESTINATION bin)
