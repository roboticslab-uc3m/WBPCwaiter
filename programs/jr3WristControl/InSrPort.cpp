// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InSrPort.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <iomanip>

#include <fstream>

//static FILE *fp;

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

namespace teo
{

/************************************************************************/
void InSrPort::setFollow(int value)
{
    follow = value;
}

/************************************************************************/
void InSrPort::onRead(Bottle& FTsensor) {

/*
    if (a==0)    {
        preprogrammedInitTrajectory();
        iPositionControl->setPositionMode();
        a=1;
    }
*/


    if (b!=250)    {
        printf("........................\n");
        offSetJR3(FTsensor);
        //iPositionControl->setPositionMode();
    }

    if (b==250)   {
        ReadFTSensor(FTsensor);
        AxesTransform1();
//        AxesTransform2();
        ZMPcomp();
        //LIPM3d();
        saveToFile();
    }

    /*
     * if (umbral>rzmp)
     *      LIPM3d(_tray.xzmp, _tray.yzmp);
     *      pepinito = 0;
     * else                     // aun por desarrollar
     *      pepinito ++;
     *      if (pepinito>25)
     *          poseRefReturn();
     *
    */

}

/************************************************************************/
void InSrPort::strategyPositionDirect(Bottle& FTsensor)
{

    if (a==0)    {
        preprogrammedInitTrajectory();
        //iPositionDirect->setPositionDirectMode();
        iPositionControl->setPositionMode();
        a=1;
    }

/** -------------------READING INPUT MESSAGES FROM VISION SENSOR-------------------**/
    //double x = b.get(0).asDouble(); //Data pxXpos
    //double y = b.get(1).asDouble(); //Data pxYpos
    double angle = FTsensor.get(2).asDouble(); //Angle
    /** --------------------------------------------------- **/


    // (0.526938 0.346914 0.312769 -1.0 0.00 -0.00 90.00) -- posicion inicial en cartesiano--

/** ----- Obtain current joint position --------------- **/
    std::vector<double> currentQ(numRobotJoints);
    std::vector<double> desireQ(numRobotJoints);

    if ( ! iEncoders->getEncoders( currentQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }
    /** --------------------------------------------------- **/


/** ----- Obtain current cartesian position ---------- **/
    std::vector<double> currentX, desireX;
    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");
    }
/*    CD_DEBUG_NO_HEADER("[currentX]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",currentX[i]);
    CD_DEBUG_NO_HEADER("\n ");*/
    /** --------------------------------------------------- **/


/** -----------------------CONTROL------------------------ **/

    desireX = currentX; // 6 = 6

    if ( (angle >= 70) && (angle < 85) )    {   //Correction 01. Move arm Y right.
        if( currentX[1] > 0.30 )        {
            printf("THE BOTTLE GOES RIGHT \n");
            desireX[1] = currentX[1] - (0.2*cos(angle));
            pepinito = 1;
        }
        else if( currentX[1] <= 0.30 )        {
            printf("¡¡¡ BOTTLE FALL RIGHT !!! \n");
            return;
/*
            //desireX[1] = 0.346914;
            desireX[1] = currentX[1];
            pepinito = 3;
*/

            //if( ! iPositionControl->positionMove( beforeQ.data() ))            {
            //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
            //
            // beforeQ = currentQ; ???? por confirmar
            //return;
        }
    }
    else if( (angle > 95) && (angle <= 110) )    {   //Correction 02. Move arm Y left.
        if( currentX[1] < 0.45 )        {
             printf("THE BOTTLE GOES LEFT \n");
             desireX[1] = currentX[1] - (0.2*cos(angle));
             pepinito = 1;
        }
        else if( currentX[1] >= 0.45 )                {
             printf("¡¡¡ BOTTLE FALL LEFT !!! \n");
             return;

/*
             //desireX[1] = 0.346914;
             desireX[1] = currentX[1];
             pepinito = 3;
*/

             //if( ! iPositionControl->positionMove( beforeQ.data() ))                    {
             //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
             //}
             // beforeQ = currentQ; ???? por confirmar
             //return;
        }
    }
    else    {      //if(z>=88 && z<=92)
        printf("THE BOTTLE IS IN EQUILIBRIUM \n");
        pepinito = 3;
        //return;
/*
        desireX[1] = 0.346914;
        pepinito = 3;
*/
        //if( ! iPositionControl->positionMove( beforeQ.data() ))        {
        //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
        //}
        // beforeQ = currentQ; ???? por confirmar
        //return;
    }
    desireX[0] = 0.526938;
    desireX[2] = 0.312769;
    desireX[3] = -1;
    desireX[4] = 0;
    desireX[5] = 0;
    desireX[6] = 90;

    if ( ! iCartesianSolver->invKin(desireX,currentQ,desireQ) )    {
        CD_ERROR("invKin failed.\n");
    }

    CD_DEBUG_NO_HEADER("[desireX]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",desireX[i]);
    CD_DEBUG_NO_HEADER("\n ");

    //--------------------------------------------------------------
/*
    CD_DEBUG_NO_HEADER("[beforeQ]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",currentQ[i]);
    CD_DEBUG_NO_HEADER("\n ");
    CD_DEBUG_NO_HEADER("[currentQ]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",currentQ[i]);
    CD_DEBUG_NO_HEADER("\n ");
    CD_DEBUG_NO_HEADER("[desireQ]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",desireQ[i]);
    CD_DEBUG_NO_HEADER("\n");
*/
    double initpos[7] = {-30,0,0,-90,0,30,0};
    switch (pepinito)    {
    case 1:
        if( ! iPositionControl->positionMove( desireQ.data() ))        {
            CD_WARNING("setPositions failed, not updating control this iteration.\n");
        }
        break;
    case 2: // no se usa
        if( ! iPositionControl->positionMove( currentQ.data() ))        {
            CD_WARNING("setPositions failed, not updating control this iteration.\n");
        }
        break;
    case 3:
        /*currentQ = std::vector<double>(initpos);
        for(int i=0;i<numRobotJoints;i++)
            currentQ.data(i) = initpos[i];
        CD_DEBUG_NO_HEADER("[currentQ]");
        for(int i=0;i<numRobotJoints;i++)
            CD_DEBUG_NO_HEADER("%f ",currentQ[i]);
        CD_DEBUG_NO_HEADER("\n ");
        double initpos[7] = {-30,0,0,-90,0,30,0};
        */

        /*
        iPositionControl->positionMove(0, -30);
        iPositionControl->positionMove(1, 0);
        iPositionControl->positionMove(2, 0);
        iPositionControl->positionMove(3, -90);
        iPositionControl->positionMove(4, 0);
        iPositionControl->positionMove(5, 30);
        iPositionControl->positionMove(6, 0);
        */

        if( ! iPositionControl->positionMove( initpos ))        {
            CD_WARNING("setPositions failed, not updating control this iteration.\n");
        }

        /*bool done = false;     // para comprobar si se hace el movimiento correctamente
        while( ! done )    {
            yarp::os::Time::delay(0.5);
            iPositionControl->checkMotionDone(&done);
            printf(".");
            fflush(stdout);
        }*/

        break;
    default:
        break;
    }

    //beforeQ = currentQ;

    return;

}

/************************************************************************/
void InSrPort::strategyVelocity(Bottle& FTsensor)
{

    if (a==0)    {
        preprogrammedInitTrajectory();
        iVelocityControl->setVelocityMode();
        a=1;
    }

//-------------------READING INPUT MESSAGES FROM VISION SENSOR--------------------

    //double x = b.get(0).asDouble(); //Data pxXpos
    //double y = b.get(1).asDouble(); //Data pxYpos
    double angle = FTsensor.get(2).asDouble(); //Angle

//------------------------CONTROL------------------------

    //-- Obtain current joint position
    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }

    std::vector<double> currentX, xdotd(6, 0.0); // derivada de la posicion cartesiana deseada

    //-- Perform forward kinematics to obtain cartesian position
    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");
        return;
    }

    //-- 0.526938 0.346914 0.312769 -1.0 0.00 -0.00 90.00

/*    if ( x[0] > 0.526938+0.001 )
        xdotd[0] = -0.01;
    if ( x[0] < 0.526938-0.001 )
        xdotd[0] = 0.01;
    if ( x[2] > 0.312769+0.001 )
        xdotd[2] = -0.01;
    if ( x[2] < 0.312769-0.001 )
        xdotd[2] = 0.01;
*/

    std::vector<double> desireX;
    if ( (angle >= 70) && (angle < 88) )  //Correction 01. Move arm Y right.
    {
        if( currentX[1] >= 0.25 )        {
            printf("THE BOTTLE GOES RIGHT \n");
            xdotd[1] = -0.05; // [1] corresponds to Y axis
            desireX[1] = currentX[1] - 0.02;
            pepinito = 1;
        }
        else if( currentX[1] < 0.25 )        {
            printf("¡¡¡ BOTTLE FALL RIGHT !!! \n");
            return;
            desireX[1] = 0.346914;
            pepinito = 3;
        }
    }
    else if( (angle > 92) && (angle <= 110) )  //Correction 02. Move arm Y right.
    {
        if( currentX[1] <= 0.45 )        {
             printf("THE BOTTLE GOES LEFT \n");
             desireX[1] = currentX[1] + 0.02;
             xdotd[1] = 0.05; // [1] corresponds to Y axis
             pepinito = 1;
        }
        else if( currentX[1] > 0.45 )                {
             printf("¡¡¡ BOTTLE FALL LEFT !!! \n");
             return;
             desireX[1] = 0.346914;
             pepinito = 3;
        }
    }
    else //if(z>=88 && z<=92)
    {
        printf("THE BOTTLE IS IN EQUILIBRIUM \n");
        return;
        desireX[1] = 0.346914;
        pepinito = 3;
    }

    //-- Compute joint velocity commands and send to robot.
    std::vector<double> commandQdot;
    if (! iCartesianSolver->diffInvKin(currentQ,xdotd,commandQdot) )    {
        CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        return;
    }

    for(int i=0;i<commandQdot.size();i++)    {
        if( fabs(commandQdot[i]) > DEFAULT_QDOT_LIMIT)        {
            CD_ERROR("diffInvKin too dangerous, STOP!!!.\n");
            for(int i=0;i<commandQdot.size();i++)
                commandQdot[i] = 0;
        }
    }

    CD_DEBUG_NO_HEADER("[MOVV] ");
    for(int i=0;i<6;i++)
        CD_DEBUG_NO_HEADER("%f ",xdotd[i]);
    CD_DEBUG_NO_HEADER("-> ");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",commandQdot[i]);
    CD_DEBUG_NO_HEADER("[deg/s]\n");

    commandQdot[0] = 0.1;
    if( ! iVelocityControl->velocityMove( commandQdot.data() ) )    {
        CD_WARNING("velocityMove failed, not updating control this iteration.\n");
    }

    return;
}

/************************************************************************/
bool InSrPort::preprogrammedInitTrajectory()
{
    iEncoders->getAxes(&numRobotJoints);
    CD_INFO("numRobotJoints: %d.\n",numRobotJoints);


/** ----- generate initial movement --------------- **/
    iPositionControl->setPositionMode();
    printf("begin MOVE TO START POSITION\n");
    double initpos[7] = {-30,0,0,-90,0,30,0};
    //iPositionControl->positionMove(initpos);
    //posicionamiento temporal hasta arreglar set poss
    iPositionControl->positionMove(0,-30);
    iPositionControl->positionMove(1,0);
    iPositionControl->positionMove(2,0);
    iPositionControl->positionMove(3,-90);
    iPositionControl->positionMove(4,0);
    iPositionControl->positionMove(5,30);

    yarp::os::Time::delay(10);  // provisional !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    /*bool done = false;
    while( ! done )    {
        yarp::os::Time::delay(0.5);
        iPositionControl->checkMotionDone(&done);
        printf(".");
        fflush(stdout);
    }*/
    printf("end MOVE TO START POSITION\n");

/** ---- designate initial position --------------- **/ //comprobar funcionalidad
    std::vector<double> beforeQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( beforeQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return false;
    }
    /** --------------------------------------------------- **/


/** ----- set NEW ref speed --------------- **/
    double initspe[7] = {100.0,100.0,100.0,100.0,100.0,100.0,0.0};
    iPositionControl->setRefSpeeds(initspe);
    /** --------------------------------------------------- **/


/** ----- set NEW ref accelaration --------------- **/
    double initacc[7] = {50.0,50.0,50.0,50.0,50.0,50,0.0};
    iPositionControl->setRefAccelerations(initacc);
    /** --------------------------------------------------- **/


    yarp::os::Time::delay(3);
    return true;
}

/************************************************************************/
void InSrPort::poseRefCalculate(Bottle& FTsensor){

    /**     * Configurating the pose and F/t references    **/

    ReadFTSensor(FTsensor);

    _tray._F.fx = + _jr3._initF.fz;
    _tray._F.fy = + _jr3._initF.fy;
    _tray._F.fz = + _jr3._initF.fx;
    _tray._M.mx = + _jr3._initT.mz;
    _tray._M.my = + _jr3._initT.my;
    _tray._M.mz = + _jr3._initT.mx;

    cout << _tray._F.fx << "\t, " << _tray._F.fy << "\t, " << _tray._F.fz << "\t " << "]" << endl;//<< FF[1] << "]" << endl;
    if(_tray._F.fx>0.075) iPositionControl->relativeMove(5, -0.5);
    if(_tray._F.fx<-0.075) iPositionControl->relativeMove(5, 0.5);
    yarp::os::Time::delay(5);

    if(_tray._F.fy>0.075) iPositionControl->relativeMove(4, 0.5);
    if(_tray._F.fy<-0.075) iPositionControl->relativeMove(4, -0.5);
    yarp::os::Time::delay(5);

    if((_tray._F.fx<0.075)&&(_tray._F.fx>-0.075)&&(_tray._F.fy<0.075)&&(_tray._F.fy>-0.075)) {
        b=1;
        printf("eeeeeeeeeeyyyyyyyyyyyy........................\n");

    }

}

/************************************************************************/
void InSrPort::ReadFTSensor(Bottle& FTsensor){
    /**     * Reading input messages from JR3 SENSOR    **/

    _jr3._initF.fx = FTsensor.get(0).asDouble();
    _jr3._initF.fy = FTsensor.get(1).asDouble();
    _jr3._initF.fz = FTsensor.get(2).asDouble();
    _jr3._initT.mx = FTsensor.get(3).asDouble() / 10;
    _jr3._initT.my = FTsensor.get(4).asDouble() / 10;
    _jr3._initT.mz = FTsensor.get(5).asDouble() / 10;

}

/************************************************************************/
void InSrPort::AxesTransform1(){
/**     * Transformation matrix between TEO_body_axes (world) and Jr3_axes
     * with horizontal tray (waiter)    **/

    _tray._F.fx = + _jr3._initF.fz - _off._F.fz;
    _tray._F.fy = + _jr3._initF.fy - _off._F.fy;
    _tray._F.fz = + _jr3._initF.fx + _off._F.fx;
    _tray._M.mx = + _jr3._initT.mz - _off._M.mz;
    _tray._M.my = + _jr3._initT.my + _off._M.my;
    _tray._M.mz = + _jr3._initT.mx - _off._M.mx;
/*
    _tray._F.fx = std::setprecision(2) << std::fixed << _tray._F.fx;
    _tray._F.fy << std::setprecision(2) << std::fixed << _tray._F.fy;
    _tray._F.fz << std::setprecision(2) << std::fixed << _tray._F.fz;
*/

    if ((_tray._F.fx<0.1)&&(_tray._F.fx>-0.1)) { //filtro para valores menores de 0.1 N
        _tray._F.fx=0.0;}
    if ((_tray._F.fy<0.1)&&(_tray._F.fy>-0.1)) { //filtro para valores menores de 0.1 N
        _tray._F.fy=0.0;}
    if ((_tray._F.fz<0.1)&&(_tray._F.fz>-0.1)) { //filtro para valores menores de 0.1 N
        _tray._F.fz=0.0;}

}

/************************************************************************/
void InSrPort::AxesTransform2(){
    /**     * Force/torque Transformation depending on the TCP orientation.    **/

    std::vector<double> currentQ(7);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        std::cout << "ZMP: [dentro]" << endl;
        return;    }

    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");    }

/*   //normalizando (no hace falta xq ya esta predefinido)
    _normVector = sqrt(pow((currentX[3]),2) + pow((currentX[4]),2) + pow((currentX[5]),2));
    currentX[3] = currentX[3] / _normVector;
    currentX[4] = currentX[4] / _normVector;
    currentX[5] = currentX[5] / _normVector;*/
    /*
    //redondeando los quaternios
    int i;
    for (i=3;i<=6;i++)   {
        //if (currentX[i]>0)  {
            currentX[i] = round (currentX[i]); //} // ceil or floor
        //else{    currentX[i] = ceil (currentX[i]);}
    }*/

    //convirtiendo de grados a radianes
    angle= currentX[6];
    currentX[6]=(currentX[6]/180)*3.1415926;
    //convirtiendo en quaternios
    quat[0] = cos(currentX[6]/2); // angulo theta
    quat[1] = currentX[3] * sin(currentX[6]/2); // eje X
    quat[2] = currentX[4] * sin(currentX[6]/2); // eje Y
    quat[3] = currentX[5] * sin(currentX[6]/2); // eje Z, siempre a cero

    if (quat[0]==0 && quat[1]==0 && quat[2]==0 && quat[3]==0) {
        quat[1]=1;
    }

    //calculando quaternio conjugado
    quatC = quat;
    quatC[1] = - quat[1];
    quatC[2] = - quat[2];
    quatC[2] = - quat[3];

    //transformada      preFinalForce   = quat              *   _initF
    //                  Final_F     = preFF    *   quatC
/*    preFF[0]=((-quat[1]*_tray._F.fx) - (quat[2]*_tray._F.fy) - (quat[3]*_tray._F.fz));
    preFF[1]=((quat[0]*_tray._F.fx) + (quat[2]*_tray._F.fz) - (quat[3]*_tray._F.fy));
    preFF[2]=((quat[0]*_tray._F.fy) - (quat[1]*_tray._F.fz) + (quat[3]*_tray._F.fx));
    preFF[3]=((quat[0]*_tray._F.fz) + (quat[1]*_tray._F.fy) - (quat[2]*_tray._F.fx));
    FF[0]=((preFF[0]*quatC[0]) - (preFF[1]*quatC[1]) - (preFF[2]*quatC[2])) - (preFF[3]*quatC[3]);
    FF[1]=((preFF[0]*quatC[1]) + (preFF[1]*quatC[0]) + (preFF[2]*quatC[3])) - (preFF[3]*quatC[2]); //fx
    FF[2]=((preFF[0]*quatC[2]) - (preFF[1]*quatC[3]) + (preFF[2]*quatC[0])) + (preFF[3]*quatC[1]); //fy
    FF[3]=((preFF[0]*quatC[3]) + (preFF[1]*quatC[2]) - (preFF[2]*quatC[1])) + (preFF[3]*quatC[0]); //fz
*/
/*    //transformada      preFM   = quat              *   _initM
    //                  FM     = _preFM    *   quatC
    preFM[0]=((-quat[1]*_tray._M.mx) - (quat[2]*_tray._M.my) - (quat[3]*_tray._M.mz));
    preFM[1]=((quat[0]*_tray._M.mx) + (quat[2]*_tray._M.mz) - (quat[3]*_tray._M.my));
    preFM[2]=((quat[0]*_tray._M.my) - (quat[1]*_tray._M.mz) + (quat[3]*_tray._M.mx));
    preFM[3]=((quat[0]*_tray._M.mz) + (quat[1]*_tray._M.my) - (quat[2]*_tray._M.mx));
    FM[0]=((preFM[0]*quatC[0]) - (preFM[1]*quatC[1]) - (preFM[2]*quatC[2])) - (preFM[3]*quatC[3]);
    FM[1]=((preFM[0]*quatC[1]) + (preFM[1]*quatC[0]) + (preFM[2]*quatC[3])) - (preFM[3]*quatC[2]); //mx
    FM[2]=((preFM[0]*quatC[2]) - (preFM[1]*quatC[3]) + (preFM[2]*quatC[0])) + (preFM[3]*quatC[1]); //my
    FM[3]=((preFM[0]*quatC[3]) + (preFM[1]*quatC[2]) - (preFM[2]*quatC[1])) + (preFM[3]*quatC[0]); //mz
*/

}

/************************************************************************/
void InSrPort::ZMPcomp(){
    /**     * Bottle ZMP measurements    **/

    if ( ! iEncoders->getEncoders( currentQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        std::cout << "ZMP: [dentro]" << endl;
        return;    }

    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");    }

    currentX[6]=(currentX[6]/180)*3.1415926; // transformando a rad

    _thetaX = -(atan((fabs(_tray._F.fz)/_tray._F.fx))); //sobre el plano YZ en rad
    _thetaY = (atan((fabs(_tray._F.fz)/_tray._F.fy))); //sobre el plano XZ en rad
    if (_thetaX < 0)    {
        _thetaX += + 3.1415926/2;}
    else    {
        _thetaX -= + 3.1415926/2;}
    if (_thetaY < 0)    {
        _thetaY += + 3.1415926/2;}
    else    {
        _thetaY -= + 3.1415926/2;}


    if (_tray._F.fz==0)    {
        _tray._zmp.x_zmp = _tray._zmp.x_zmp; // Metros
        _tray._zmp.y_zmp = _tray._zmp.y_zmp; // Metros
    }else{
        _tray._zmp.x_zmp = ( - _tray._M.my / ((_tray._F.fz)*cos(_thetaX)) - (_l*sin(_thetaX)) - _d); // Metros
        _tray._zmp.y_zmp = ( _tray._M.mx / ((_tray._F.fz)*cos(_thetaY)) - (_l*sin(_thetaY)) ); // Metros
    }
    _tray._zmp.x_zmp = _tray._zmp.x_zmp*cos(_thetaX);
    _tray._zmp.y_zmp = _tray._zmp.y_zmp*cos(_thetaY);

/*    if (_tray._zmp.x_zmp>0.075){ //limitando el maximo ZMP en X positivo
        _tray._zmp.x_zmp = 0.075;
    }if (_tray._zmp.x_zmp<-0.075)    {//limitando el maximo ZMP en X negativo
        _tray._zmp.x_zmp = -0.075;
    }*/if ((_tray._zmp.x_zmp<0.001) && (_tray._zmp.x_zmp>-0.001))    {
        _tray._zmp.x_zmp = 0;} //limitando el minimo ZMP en X

/*    if (_tray._zmp.y_zmp>0.075)    {//limitando el maximo ZMP en Y positivo
        _tray._zmp.y_zmp = 0.075;
    }if (_tray._zmp.y_zmp<-0.075)    {//limitando el maximo ZMP en Y negativo
        _tray._zmp.y_zmp = -0.075;
    }*/if ((_tray._zmp.y_zmp<0.001) && (_tray._zmp.y_zmp>-0.001))    {
        _tray._zmp.y_zmp = 0;} //limitando el minimo ZMP en Y positivo


    _rzmp = sqrt(pow(_tray._zmp.x_zmp,2) + pow(_tray._zmp.y_zmp,2));

}

/************************************************************************/
void InSrPort::LIPM3d()
{
    //Generacion de la actuacion a los motores (CONTROL)

    std::vector<double> currentQ(numRobotJoints), desireQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;    }

    std::vector<double> currentX, desireX;
    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");    }

    desireX = currentX; // 6 = 6
    _rWorkSpace = sqrt(pow((currentX[0]-0.5),2) + pow((currentX[1]-0.375),2));

/*
    if ( (angle >= 70) && (angle < 85) )    {   //Correction 01. Move arm Y right.
        if( currentX[1] > 0.30 )        {
            printf("THE BOTTLE GOES RIGHT \n");
            desireX[1] = currentX[1] - (0.2*cos(angle));
            pepinito = 1;
        }
        else if( currentX[1] <= 0.30 )        {
            printf("¡¡¡ BOTTLE FALL RIGHT !!! \n");
            return;

            //desireX[1] = 0.346914;
            //desireX[1] = currentX[1];
            //pepinito = 3;


            //if( ! iPositionControl->positionMove( beforeQ.data() ))            {
            //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
            //
            // beforeQ = currentQ; ???? por confirmar
            //return;
        }
    }
    else if( (angle > 95) && (angle <= 110) )    {   //Correction 02. Move arm Y left.
        if( currentX[1] < 0.45 )        {
             printf("THE BOTTLE GOES LEFT \n");
             desireX[1] = currentX[1] - (0.2*cos(angle));
             pepinito = 1;
        }
        else if( currentX[1] >= 0.45 )                {
             printf("¡¡¡ BOTTLE FALL LEFT !!! \n");
             return;

             //desireX[1] = 0.346914;
             //desireX[1] = currentX[1];
             //pepinito = 3;

             //if( ! iPositionControl->positionMove( beforeQ.data() ))                    {
             //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
             //}
             // beforeQ = currentQ; ???? por confirmar
             //return;
        }
    }
    else    {      //if(z>=88 && z<=92)
        printf("THE BOTTLE IS IN EQUILIBRIUM \n");
        pepinito = 3;
        //return;

        //desireX[1] = 0.346914;
        //pepinito = 3;

        //if( ! iPositionControl->positionMove( beforeQ.data() ))        {
        //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
        //}
        // beforeQ = currentQ; ???? por confirmar
        //return;
    }
*/

    if ((_rzmp>0.020) && (_rWorkSpace<0.075))   { // será necesario programar los limites en los ejes X e Y.
        desireX[0] = currentX[0] + _tray._zmp.x_zmp; // new X position
        desireX[1] = currentX[1] + _tray._zmp.y_zmp; // new Y position
    }
    if ((_rzmp>0.020) && (_rWorkSpace>0.075))   { // será necesario programar los limites en los ejes X e Y.
        printf("¡¡¡ BOTTLE FALL !!! \n");
        //return;
    }
    if ((_rzmp<0.020))   { // será necesario programar los limites en los ejes X e Y.
        desireX[0] = 0.526565; // new X position
        desireX[1] = 0.346227; // new Y position
        printf("THE BOTTLE IS IN EQUILIBRIUM \n");
    }

    desireX[2] = 0.309705; // Z position

    // ref: 0.526565 0.346227 0.309705 -0.91063 0.409172 -0.057711 0.610729
    desireX[3] = -0.351340;    //
    desireX[4] = 0.707474;     //
    desireX[5] = 0.613221;     //  Orientation
    desireX[6] = 0.351825;    //

    if ( ! iCartesianSolver->invKin(desireX,currentQ,desireQ) )    {
        CD_ERROR("invKin failed.\n");
    }

    CD_DEBUG_NO_HEADER("[currentX]");
    for(int i=0;i<7;i++)
        CD_DEBUG_NO_HEADER("%f ",currentX[i]);
    CD_DEBUG_NO_HEADER("\n ");

    CD_DEBUG_NO_HEADER("[desireX]");
    for(int i=0;i<7;i++)
        CD_DEBUG_NO_HEADER("%f ",desireX[i]);
    CD_DEBUG_NO_HEADER("\n ");

    CD_DEBUG_NO_HEADER("[ZMP values]");
    CD_DEBUG_NO_HEADER("%f ",_tray._zmp.x_zmp);
    CD_DEBUG_NO_HEADER("%f ",_tray._zmp.y_zmp);
    CD_DEBUG_NO_HEADER("%f ",_rWorkSpace);
    CD_DEBUG_NO_HEADER("\n ");


    if( ! iPositionControl->positionMove( desireQ.data() )) {
        CD_WARNING("setPositions failed, not updating control this iteration.\n");    }

    return;

}

/************************************************************************/
void InSrPort::saveToFile(){
    _modFS = sqrt(pow((_tray._F.fx),2) + pow((_tray._F.fy),2) + pow((_tray._F.fz),2));
    _modFF = sqrt(pow((FF[1]),2) + pow((FF[2]),2) + pow((FF[3]),2));
/*    _thetaX = -(((atan((fabs(_tray._F.fz)/_tray._F.fx)))*180)/3.1415926);
    _thetaY = (((atan((fabs(_tray._F.fz)/_tray._F.fy)))*180)/3.1415926);
    if (_thetaX < 0)
        _thetaX += + 90;
    else
        _thetaX -= + 90;

    if (_thetaY < 0)
        _thetaY += + 90;
    else
        _thetaY -= + 90;*/


    cout << "CurX: [" << currentX[3] << "\t, " << currentX[4] << "\t, " << currentX[5] << "\t, " << currentX[6] << "]" << endl;
//    cout << "Quat: [" << quatC[0] << "\t, " << quatC[1] << "\t, " << quatC[2] << "\t, " << quatC[3] << "]" << endl;
    cout << "F_init: [" << _tray._F.fx << "\t, " << _tray._F.fy << "\t, " << _tray._F.fz << "\t " << "]" << endl;//<< FF[1] << "]" << endl;
//    cout << "F_finl: [" << FF[1] << "\t, " << FF[2] << "\t, " << FF[3] << "\t " << "]" << endl;//<< FF[1] << "]" << endl;
    cout << "torque: [" << _tray._M.mx << "\t, " << _tray._M.my << "\t, " << _tray._M.mz << "\t " << "]" << endl;//<< FF[1] << "]" << endl;

//    cout << "F_X: [" << _tray._F.fx << "\t, " << FF[1] << "]" << endl;//<< FF[1] << "]" << endl;
//    cout << "F_Y: [" << _tray._F.fy << "\t, " << FF[2] << "]" << endl;//<< FF[2] << "]" << endl;
//    cout << "F_Z: [" << _tray._F.fz << "\t, " << FF[3] << "]" << endl;//<< FF[3] << "]" << endl;
    cout << "ZMP: [" << _tray._zmp.x_zmp << "\t, " << _tray._zmp.y_zmp << "]" << endl;
//    cout << "mod: [" << _modFS << "\t, " << _modFF << "]" << endl;
    cout << "the: [" << _thetaX << "\t, " << _thetaY << "]" << endl;

    /*CD_DEBUG_NO_HEADER("F_init:[");
    CD_DEBUG_NO_HEADER("%f \t",_tray._F.fx);
    CD_DEBUG_NO_HEADER("%f \t",_tray._F.fy);
    CD_DEBUG_NO_HEADER("%f \t",_tray._F.fz);
    CD_DEBUG_NO_HEADER("] - F_final:[ ");
    CD_DEBUG_NO_HEADER("%f \t",FF[1]);
    CD_DEBUG_NO_HEADER("%f \t",FF[2]);
    CD_DEBUG_NO_HEADER("%f \t",FF[3]);
    CD_DEBUG_NO_HEADER("]\n ");
    CD_DEBUG_NO_HEADER("%f \t",_modFS);
    CD_DEBUG_NO_HEADER("%f \t",_modFF);
    CD_DEBUG_NO_HEADER("\n ");*/

/*
    ofstream out;
    if(iteration==1) {
        out.open("cabeza.txt",ios::trunc);
    }
    else {
        out.open("cabeza.txt",ios::app);
    }
    out <<  _tray._zmp.x_zmp << " " <<  _tray._zmp.y_zmp << " " << endl;
    out.close();
    iteration ++;
*/
//    fprintf(fp,"\n%d", n);
//    fprintf(fp,",%.4f", _dt);
//    fprintf(fp,",%.15f", X);
//    fprintf(fp,",%.15f", _eval_x.y);
//    fprintf(fp,",%.15f", _eval_x._zmp_error);
//    fprintf(fp,",%.15f", _eval_x._zmp_ref);
//    fprintf(fp,",%10f", _eval_x._u);
//    fprintf(fp,",%10f", _tray.xzmp);
//    fprintf(fp,",%10f", _tray.yzmp);
//    fprintf(fp,",%f", angle_x);
//    fprintf(fp,",%f", vel);

    //        fprintf(fp,",%.15f", _eval_y._r);
    //        fprintf(fp,",%.15f", _yzmp);
    //        fprintf(fp,",%.15f", _eval_y.y);
    //        fprintf(fp,",%f", angle_y);
}

/************************************************************************/
void InSrPort::offSetJR3(Bottle& FTsensor){

    /**     * Offset JR3 measurements    **/

    ReadFTSensor(FTsensor);
    _off._F.fx += -9.72 - _jr3._initF.fx;
    _off._F.fy += + _jr3._initF.fy;
    _off._F.fz += + _jr3._initF.fz; // No interesa eliminar
    _off._M.mx += + _jr3._initT.mx;
    _off._M.my += 0.243 - _jr3._initT.my;
    _off._M.mz += + _jr3._initT.mz;

    b++;
    printf("\n%d", b);

    if (b==250) {
        _off._F.fx = _off._F.fx / b;
        _off._F.fy = _off._F.fy / b;
        _off._F.fz = _off._F.fz / b; // No interesa eliminar
        _off._M.mx = _off._M.mx / b;
        _off._M.my = _off._M.my / b;
        _off._M.mz = _off._M.mz / b;
    }

}

/************************************************************************/
}   // namespace teo
