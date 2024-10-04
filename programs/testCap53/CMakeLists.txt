option(ENABLE_testCap53 "Enable/disable testCap53 program" ON)

if(ENABLE_testCap53)

    add_executable(testingDLIPM main.cpp
                                global.h
                                TestCap53.hpp
                                TestCap53.cpp
                                PeriodicThreadImpl.hpp
                                PeriodicThreadImpl.cpp
                                LIPM2d.h
                                LIPM2d.cpp)

    target_link_libraries(testingDLIPM YARP::YARP_init
                                       YARP::YARP_os
                                       YARP::YARP_dev)

    install(TARGETS testingDLIPM)

endif(ENABLE_testCap53)
