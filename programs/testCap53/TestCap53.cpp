// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**

in this test, we pretend to compute the different erros of the robot. Therefore, we
propose to generate a dataset for the zmp values and the FT sensor.


first, it is not need to apply the function "evaluateModel()". the purpose is generate
different test (changing the ZMP_ref from 0.01m to 0.09m). and then calculate the error
related to the LIPM model.


sencond. with the error equation inside "evaluateModel()" and the DLIPM (in th space state),
we apply the same test (ZMP_ref from 0.01m to 0.09m) to verify if (ZMP_ref = ZMP_ft).

**/

#include "TestCap53.hpp"

constexpr auto DEFAULT_PERIOD = 0.05; // [s]

namespace roboticslab
{

/************************************************************************/
bool TestCap53::configure(ResourceFinder &rf) {

    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();
    double period = rf.check("period",yarp::os::Value(DEFAULT_PERIOD),"period of the thread").asFloat64();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("testCap53 options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--robot ('teo' or 'teoSim'; default: %s)\n",DEFAULT_ROBOT);
        printf("\t--period (default: %f)\n",DEFAULT_PERIOD);
    }
    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }
    std::string waiterStr("/bodyBal");
    /** **************************************************************************************
     * ******************************************************************************** **/

    // ------ LEFT LEG DEV -------
    yarp::os::Property leftLegOptions;
    leftLegOptions.put("device","remote_controlboard");
    leftLegOptions.put("remote",robot+"/leftLeg");
    leftLegOptions.put("local",waiterStr+"/teo/leftLeg");
    leftLegDevice.open(leftLegOptions);
    if(!leftLegDevice.isValid()) {
        printf("robot leftLeg device not available.\n");
        leftLegDevice.close();
        yarp::os::Network::fini();
        return false;    }
    if (!leftLegDevice.view(leftLegIEncoders) ) { // connecting our device with "IEncoders" interface
        printf("[warning] Problems acquiring leftLegIEncoders interface\n");
        return false;
    } else printf("[success] Acquired leftLegIEncoders interface\n");
    if (!leftLegDevice.view(leftLegIControlMode) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring leftLegIControlMode interface\n");
        return false;
    } else printf("[success] Acquired leftLegIControlMode interface\n");

    if (!leftLegDevice.view(leftLegIPositionDirect) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring leftLegIPositionDirect interface\n");
        return false;
    } else printf("[success] Acquired leftLegIPositionDirect interface\n");
    /** **************************************************************************************
     * ******************************************************************************** **/

    // ------ RIGHT LEG DEV -------
    yarp::os::Property rightLegOptions;
    rightLegOptions.put("device","remote_controlboard");
    rightLegOptions.put("remote","/teo/rightLeg");
    rightLegOptions.put("local",waiterStr+robot+"/rightLeg");
    rightLegDevice.open(rightLegOptions);
    if(!rightLegDevice.isValid()) {
        printf("robot rightLeg device not available.\n");
        rightLegDevice.close();
        yarp::os::Network::fini();
        return false;    }
    if (!rightLegDevice.view(rightLegIEncoders) ) { // connecting our device with "IEncoders" interface
        printf("[warning] Problems acquiring rightLegIEncoders interface\n");
        return false;
    } else printf("[success] Acquired rightLegIEncoders interface\n");
    if (!rightLegDevice.view(rightLegIControlMode) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring rightLegIControlMode interface\n");
        return false;
    } else printf("[success] Acquired rightLegIControlMode interface\n");
    if (!rightLegDevice.view(rightLegIPositionDirect) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring rightLegIPositionDirect interface\n");
        return false;
    } else printf("[success] Acquired rightLegIPositionDirect interface\n");
    /** **************************************************************************************
     * ******************************************************************************** **/

    // ----- SET CONTROL MODES -----
    leftLegIPositionDirect->getAxes(&numLeftLegJoints);
    std::vector<int> leftLegControlModes(numLeftLegJoints,VOCAB_CM_POSITION_DIRECT);
    if(! leftLegIControlMode->setControlModes( leftLegControlModes.data() )){
        printf("[warning] Problems setting position direct control mode of: left-Leg\n");
        return false;
    }
    rightLegIPositionDirect->getAxes(&numRightLegJoints);
    std::vector<int> rightLegControlModes(numRightLegJoints,VOCAB_CM_POSITION_DIRECT);
    if(! rightLegIControlMode->setControlModes(rightLegControlModes.data())){
        printf("[warning] Problems setting position direct control mode of: right-Leg\n");
        return false;
    }
    /** **************************************************************************************
     * ******************************************************************************** **/

    //-- Conection between TestCap53 & PeriodicThreadImpl
    periodicThreadImpl = new PeriodicThreadImpl(period);
    periodicThreadImpl->setIEncodersControl(rightLegIEncoders,leftLegIEncoders);
    periodicThreadImpl->setIPositionDirect(rightLegIPositionDirect,leftLegIPositionDirect);
    periodicThreadImpl->setInputPorts(&portft0,&portft1);

    periodicThreadImpl->start();

    return true;
}

/************************************************************************/
bool TestCap53::close() {
    delete periodicThreadImpl;
    periodicThreadImpl = nullptr;
    return true;
}

/************************************************************************/
double TestCap53::getPeriod() {
    return 4.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool TestCap53::updateModule() {
    printf("TestCap53 alive...\n");
    return true;
}

/************************************************************************/
bool TestCap53::interruptModule() {
    printf("Test51 closing...\n");

    periodicThreadImpl->stop();

    rightLegDevice.close();
    leftLegDevice.close();

    return true;
}

/************************************************************************/
} // namespace roboticslab
