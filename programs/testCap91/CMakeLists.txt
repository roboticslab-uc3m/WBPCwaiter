option(ENABLE_testCap91 "Enable/disable testCap91 program" ON)

if(ENABLE_testCap91)

    add_executable(object_Balance main.cpp
                                  global.h
                                  TestCap91.hpp
                                  TestCap91.cpp
                                  ThreadImpl.hpp
                                  ThreadImpl.cpp)

    target_link_libraries(object_Balance YARP::YARP_init
                                        YARP::YARP_os
                                        YARP::YARP_dev
                                        ROBOTICSLAB::ColorDebug
                                        ROBOTICSLAB::KinematicsDynamicsInterfaces
                                        ROBOTICSLAB::KinematicRepresentationLib)

    install(TARGETS object_Balance)

endif(ENABLE_testCap91)
