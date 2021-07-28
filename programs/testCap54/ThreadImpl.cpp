// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ThreadImpl.hpp"

namespace roboticslab
{
/************************************************************************/
bool ThreadImpl::threadInit() {
    printf("[success] entrando en ratethread -> init/run\n");

    a = 0; // pendiente de chequear cuantos sobran
    b = 0;
    n = 1;

    zmp_ref = 0; // initial value and our zmp reference
    _ang_ref = 0; // initial value and our angle reference
    _ang_out = 0; // initial value and our output

    double initspe[7] = {2.0,2.0,2.0,2.0,2.0,2.0,0.0}; // --set NEW ref speed
    //double initspe[7] = {10.0,10.0,10.0,10.0,10.0,10.0,0.0}; // --set NEW ref speed
    //double initspe[7] = {20.0,20.0,20.0,20.0,20.0,20.0,0.0}; // --set NEW ref speed
    leftLegIPositionControl->setRefSpeeds(initspe);
    rightLegIPositionControl->setRefSpeeds(initspe);
    double initacc[7] = {2.0,2.0,2.0,2.0,2.0,2.0,0.0}; // --set NEW ref accelaration
    //double initacc[7] = {10.0,10.0,10.0,10.0,10.0,10,0.0}; // --set NEW ref accelaration
    //double initacc[7] = {20.0,20.0,20.0,20.0,20.0,20,0.0}; // --set NEW ref accelaration
    leftLegIPositionControl->setRefAccelerations(initacc);
    rightLegIPositionControl->setRefAccelerations(initacc);

    leftLegIEncoders->getAxes(&numLeftLegJoints);
    rightLegIEncoders->getAxes(&numRightLegJoints);

return true;
}

/************************************************************************/
void ThreadImpl::run()
{
    std::vector<double> rightLegQs(numRightLegJoints);
    std::vector<double> leftLegQs(numLeftLegJoints);

    while(!isStopping()) {

        if (a!=1)    {  // STEP 1 - Creating & Configuring CSV file
            confCSVfile();
            a=1;    }
        if (a==1 && b!=1)    {  // STEP 2 - waiting for waiter pose of the left arm
            cout << "[waiting arm pose] ... STEP 2 " << endl;
            getchar();
            b=1;    }

        // ------------------------------------------------------------------------
        if (a==1 && b==1)   {   // STEP 3 - main code

            getInitialTime();

            readSensorsFT0();
            readSensorsFT1();
            readSensorsIMU();
            zmpCompFT(); // calculation of the ZMP_FT

            if (n>1000)  {
                evaluateModel(rightLegQs,leftLegQs); // evaluacion the model and the angle output
                setJoints(); // applying the ankle movement
            }

            printData();
            cout << endl << "Press ENTER to exit..." << endl;
            cout << "*******************************" << endl << endl;
            saveInFileCsv();  // saving the information ¡
            n++;
            cout << n << endl << endl;

            getCurrentTime();
        }
    }
}

/************************************************************************/
void ThreadImpl::readSensorsFT0()       /** Reading input messages from FT0 SENSORS    **/
{
    //--- FT-Sensor 0 right leg

    yarp::sig::Vector ch0;
    int ret = ft0AnalogSensor->read(ch0); // lecture from sensor JR3 ch0 - right leg
    if (ret == yarp::dev::IAnalogSensor::AS_OK)
    {
        _RL._F.fx = ch0[0];
        _RL._F.fy = ch0[1];
        _RL._F.fz = ch0[2];
        _RL._T.mx = ch0[3];
        _RL._T.my = ch0[4];
        _RL._T.mz = ch0[5];

        //std::printf("Good read, got: %s\n",ch0.toString().c_str()); // print on terminal vector ch0
    }
}

/************************************************************************/
void ThreadImpl::readSensorsFT1()       /** Reading input messages from FT1 SENSORS    **/
{
    //--- FT-Sensor 1 left leg

    yarp::sig::Vector ch1;
    int ret = ft1AnalogSensor->read(ch1); // lecture from sensor JR3 ch1 - left leg
    if (ret == yarp::dev::IAnalogSensor::AS_OK)
    {
        _LL._F.fx = ch1[0];
        _LL._F.fy = ch1[1];
        _LL._F.fz = ch1[2];
        _LL._T.mx = ch1[3];
        _LL._T.my = ch1[4];
        _LL._T.mz = ch1[5];

        //std::printf("Good read, got: %s\n",ch1.toString().c_str()); // print on terminal vector ch1
    }
}

/************************************************************************/
void ThreadImpl::readSensorsIMU()       /** Reading input messages from IMU SENSORS    **/
{
        //--- Inertial-Sensor
    Bottle imu;
    portImu->read(imu); // lectura del sensor IMU
    ang_x = imu.get(0).asDouble(); // Angulo en X [deg]
    //ang_y = imu.get(1).asDouble(); // Angulo en Y [deg]
    //ang_z = imu.get(2).asDouble(); // Angulo en Z [deg]
    acc_x = imu.get(3).asDouble(); //Linear acceleration in X [m/s^2]
    x_sensor.push_front(acc_x);
    x_sensor.pop_back();
    //acc_y = imu.get(4).asDouble(); //Linear acceleration in Y [m/s^2]
    //y_sensor.push_front(acc_y);
    //y_sensor.pop_back();
    //acc_z = imu.get(5).asDouble(); //Linear acceleration in Z [m/s^2]
    //z_sensor.push_front(acc_z);
    //z_sensor.pop_back();
    //spd_x=imu.get(6).asDouble(); // Velocidad angular en X [deg/s]
    //spd_y=imu.get(7).asDouble(); // Velocidad angular en Y [deg/s]
    //spd_z=imu.get(8).asDouble(); // Velocidad angular en Z [deg/s]
    //mag_x=imu.get(9).asDouble(); // Campo magnetico en X
    //mag_y=imu.get(10).asDouble(); // Campo magnetico en Y
    //mag_z=imu.get(11).asDouble(); // Campo magnetico en Z

    //LOW-PASS FILTER
    ddx = 0.0;
    //ddy = 0.0;
    //ddz = 0.0;
    for(deque<double>::iterator it = x_sensor.begin(); it != x_sensor.end(); it++)
        ddx = ddx + *it;
    //for(deque<double>::iterator it = y_sensor.begin(); it != y_sensor.end(); it++)
        //ddy = ddy + *it;
    //for(deque<double>::iterator it = z_sensor.begin(); it != z_sensor.end(); it++)
        //ddz = ddz + *it;
    ddx = ddx / samples;
    //ddy = ddy / samples;
    //ddz = ddz / samples;

    //CONVERSION FROM IMU SENSOR COORDINATES TO ROBOT COORDINATES
     ddx_robot = ddx;
     //ddy_robot = -ddy;
     //ddz_robot = ddz;

}

/************************************************************************/
void ThreadImpl::zmpCompFT()        /** Calculating ZMP-FT of the body . **/
{

    //ZMP Equations : Double Support - FT

/*      //Con el coeficiente 1000 - unids in milimeters
    _xzmp0 = -(((_my0/10) + e*_fx0)*1000) / _fz0; // xzmp0 in [mm]
    _yzmp0 = (((_mx0/10) + e*_fy0)*1000) / _fz0; // yzmp0 in [mm]

    _xzmp1 = -(((_my1/10) + e*_fx1)*1000) / _fz1; // xzmp1 in [mm]
    _yzmp1 = (((_mx1/10) + e*_fy1)*1000) / _fz1; // yzmp1 in [mm]
*/

    //without the coeficient 1000 - unids in meters
    _xzmp_ft0 = -(((_RL._T.my/10) + e*_RL._F.fx)) / _RL._F.fz; // xzmp0_ft in [m] right foot
    //_yzmp0_ft = -(((_RF._T.mx/10) + e*_RF._F.fy)) / _RF._F.fz; // xzmp0_ft in [m] right foot
    _xzmp_ft1 = -(((_LL._T.my/10) + e*_LL._F.fx)) / _LL._F.fz; // xzmp1_ft in [m] left foot
    //_yzmp1_ft = -(((_LF._T.mx/10) + e*_LF._F.fy)) / _LF._F.fz; // xzmp1_ft in [m] left foot

    _xzmp_ft01 = (_xzmp_ft0 * _RL._F.fz + _xzmp_ft1 * _LL._F.fz) / (_RL._F.fz + _LL._F.fz); // xzmp_ft in [m] robot
    //_yzmp01_ft = (_yzmp0_ft * _RF._F.fz + _yzmp1_ft * _LF._F.fz) / (_RF._F.fz + _LF._F.fz); // xzmp_ft in [m] robot

    // OFFSET FT - deleting the offset of _xzmp01_ft ( both ZMPs from sensor in frontal plane)
    if (n >=0 && n < 500){
        sum_x_ft = _xzmp_ft01 + sum_x_ft;
        offs_x_ft = sum_x_ft / n;
        zmp_ref = offs_x_ft;
        printf("offs = %f\n", offs_x_ft);
    }

    Xzmp_ft  = _xzmp_ft01 - offs_x_ft; // frontal plane
    //Yzmp_ft = _yzmp01_ft - offs_y_ft; // saggital plane

    if ((_xzmp_ft01 != _xzmp_ft01) || (_yzmp_ft01 != _yzmp_ft01)){
        printf ("Warning: No zmp data\n");
    }

}

/************************************************************************/
void ThreadImpl::evaluateModel(std::vector<double> &rightLegQs,std::vector<double> &leftLegQs)        /** Calculating OUTPUT (Qi) of the legs. **/
{
    // obtaining the angle error for the D-LIPM space-state
/*    // opcion 1
    if(abs(Xzmp_ft)<0.02){
        //_ang_out = 0;
    }
    else {
        _evalLIPM.model(Xzmp_ft,zmp_ref);

        ka = 0.25 * zmp_ref + 9.95;
        _ang_ref = (zmp_ref*(-G))/ (L*(ka-G));

        //_ang_out =  1*(_evalLIPM.ang_error_out + _ang_ref)/10;      }

        _ang_out =  (_evalLIPM.ang_error_out + _ang_ref);     } // original*/

    // opcion 2
    if(abs(Xzmp_ft)<0.015){
        //_ang_out = 0;
    }
    else {
        _evalLIPM.model(Xzmp_ft,zmp_ref);

        if ( ! leftLegIEncoders->getEncoders( leftLegQs.data() ) )    {
            CD_WARNING("getEncoders failed, not updating control this iteration.\n");
            return;    }
        if ( ! rightLegIEncoders->getEncoders( rightLegQs.data() ) )    {
            CD_WARNING("getEncoders failed, not updating control this iteration.\n");
            return;    }

        //_ang_out =  1*(_evalLIPM.ang_error_out + _ang_ref)/10;      }

        _ang_out =  (_evalLIPM.ang_error_out + leftLegQs[4]); }// original

/*    como otra posibilidad para calcular:  _angle_ref
    if ( ! leftArmIEncoders->getEncoders( encLegs.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;    }
    _angle_ref = encLegs[4];*/

}

/************************************************************************/
void ThreadImpl::setJoints()        /** Position control **/
{
    rightLegIPositionControl->positionMove(4, _ang_out); // position in degrees
    leftLegIPositionControl->positionMove(4, _ang_out);
}

/************************************************************************/
void ThreadImpl::printData()
{
    /*        cout << endl << "El angulo 1 es: " << _angle_ref_a << endl;
        cout << endl << "El angulo 2 es: " << _angle_ref_b << endl;
        cout << endl << "El angulo 3 es: " << _angle_ref_c << endl;
        cout << endl << "El angulo 4 es: " << _angle_ref_d << endl;
        cout << endl << "El angulo 4 es: " << g << endl;
        cout << endl << "El angulo 4 es: " << ka << endl;
    */
        //cout << endl << "El ANKLE pid es: " << pid_output_ankle << endl;
        //cout << endl << "El HIP pid es: " << pid_output_hip << endl;
        //cout << endl << "El ZMP REF es: " << setpoint << endl;

        cout << endl << "El ZMP REF es: " << zmp_ref << endl;
        cout << endl << "El ZMP FT  es: " << Xzmp_ft << endl;
        cout << endl << "El ANG REF es: " << _ang_ref << endl;
        cout << endl << "El ANG OUT es: " << _ang_out << endl;

        //cout << endl << "La capture_point es: " << capture_point << endl;
        //cout << endl << "ZMP_Error_Loli = ("<< _eval_x._zmp_error << ") [mm]" << endl;
        //cout << endl << "ZMP model es: " << _eval_x.y << endl;
        //cout << endl << "Num es: " << _num << endl;  _u_ref
        //cout << endl << "El _u_ref x es: " << _eval_x._u_ref << endl;
        //cout << endl << "El _angle_error x es: " << _eval_x._angle_error << endl;

}


/************************************************************************/
      //--  LESS IMPORTANT  --//
/************************************************************************/
void ThreadImpl::confCSVfile(){       /** Configuring CSV file    **/

    cout << "[configuring] ... STEP 1 " << endl;
    fp = fopen("../data_dlipm_threat.csv","w+");
    fprintf(fp,"Time,Fx_ft0,Fz_ft0,My_ft0,Fx_ft1,Fz_ft1,My_ft1,Ang_imu,Acc_imu,Xzmp_ft,zmp_RF,zmp_LF,Angle,iter");
    yarp::os::Time::delay(1);

    cout << "[success] data_dlipm_threat.csv file configured." << endl;
}

/************************************************************************/
void ThreadImpl::saveInFileCsv()
{

    fprintf(fp,"\n%.4f", act_time);

    fprintf(fp,",%.10f", _RL._F.fx); // f_x - sensor ft 0
    fprintf(fp,",%.10f", _RL._F.fz); // f_z - sensor ft 0
    fprintf(fp,",%.10f", _RL._T.my); // m_y - sensor ft 0

    fprintf(fp,",%.10f", _LL._F.fx); // f_x - sensor ft 1
    fprintf(fp,",%.10f", _LL._F.fz); // f_z - sensor ft 1
    fprintf(fp,",%.10f", _LL._T.my); // m_y - sensor ft 1

    fprintf(fp,",%.10f", ang_x); // angle X imu
    fprintf(fp,",%.10f", acc_x); // acceleration X imu

    fprintf(fp,",%.8f", Xzmp_ft); // ZMP body (double support) (frontal plane)
    fprintf(fp,",%.8f", _xzmp_ft0); // zmp (right foot)
    fprintf(fp,",%.8f", _xzmp_ft1); // zmp (leftt foot)

    fprintf(fp,",%.8f", _ang_out); // angle for the ankles

    fprintf(fp,",%i", n);

}

/************************************************************************/
void ThreadImpl::getInitialTime()       /** Get Initial Time    **/
{
    if (n==1){init_time = Time::now();}
    init_loop = Time::now();
    it_time = init_loop - it_prev;
    it_prev = init_loop;
}

/************************************************************************/
void ThreadImpl::getCurrentTime()       /** Get Current Time    **/
{
    act_time = Time::now() - init_time;
    act_loop = Time::now() - init_loop;
}



/************************************************************************/
      //--  NOT USED  --//
/************************************************************************/
void ThreadImpl::readSensorsFT2()       /** Reading input messages from FT2 SENSORS    **/
{
    //--- FT-Sensor 2 right arm

    yarp::sig::Vector ch2;
    int ret = ft2AnalogSensor->read(ch2); // lecture from sensor JR3 ch2 - right arm
    if (ret == yarp::dev::IAnalogSensor::AS_OK)
    {
        _RA._F.fx = ch2[0];
        _RA._F.fy = ch2[1];
        _RA._F.fz = ch2[2];
        _RA._T.mx = ch2[3];
        _RA._T.my = ch2[4];
        _RA._T.mz = ch2[5];

        //std::printf("Good read, got: %s\n",ch2.toString().c_str()); // print on terminal vector ch2
    }
}

/************************************************************************/
void ThreadImpl::readSensorsFT3()       /** Reading input messages from FT3 SENSORS    **/
{
    //--- FT-Sensor 3 left arm

    yarp::sig::Vector ch3;
    int ret = ft3AnalogSensor->read(ch3); // lecture from sensor JR3 ch3 - left arm
    if (ret == yarp::dev::IAnalogSensor::AS_OK)
    {
        _LA._F.fx = ch3[0];
        _LA._F.fy = ch3[1];
        _LA._F.fz = ch3[2];
        _LA._T.mx = ch3[3];
        _LA._T.my = ch3[4];
        _LA._T.mz = ch3[5];

        //std::printf("Good read, got: %s\n",ch3.toString().c_str()); // print on terminal vector ch3
    }
}

/************************************************************************/
void ThreadImpl::zmpCompIMU()       /** Calculating ZMP-IMU of the body . **/
{

    //ZMP Equations : Double Support - IMU

    // OFFSET IMU - eliminando el offset de ddx_robot (aceleracion en X)
    if (n >=100 && n < 250){
        sum_x_imu = ddx_robot + sum_x_imu;
        offs_x_imu = sum_x_imu / (n-150);
        printf("offs = %f\n", offs_x_imu);
    }

    ddx_robot = ddx_robot - offs_x_imu; // frontal plane
    //ddy_robot = ddy_robot - offs_y_imu; // saggital plane
    //ZERO MOMENT POINT COMPUTATION - IMU
    Xzmp_imu = Xcom - (Zcom / ddz_robot) * ddx_robot; //ZMP X coordinate [m]
    //Yzmp_imu = Ycom - (Zcom / ddz_robot) * ddy_robot; //ZMP Y coordinate [m]

}



}   // namespace roboticslab
