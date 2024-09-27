file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

yarp_install(FILES ${conf}
             DESTINATION ${WBPCWAITER_CONTEXTS_INSTALL_DIR}/kinematics)
