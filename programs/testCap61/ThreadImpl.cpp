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

    _LF._F.fx = 0; // creo que esto no es necesario
    _LF._F.fy = 0;
    _LF._F.fz = 0;
    _LF._T.mx = 0;
    _LF._T.my = 0;
    _LF._T.mz = 0;

    _RF._F.fx = 0; // creo que esto no es necesario
    _RF._F.fy = 0;
    _RF._F.fz = 0;
    _RF._T.mx = 0;
    _RF._T.my = 0;
    _RF._T.mz = 0;


    //double initspe[7] = {2.0,2.0,2.0,2.0,2.0,2.0,0.0}; // --set NEW ref speed
    double initspe[7] = {10.0,10.0,10.0,10.0,10.0,10.0,0.0}; // --set NEW ref speed
    //double initspe[7] = {20.0,20.0,20.0,20.0,20.0,20.0,0.0}; // --set NEW ref speed
    leftLegIPositionControl->setRefSpeeds(initspe);
    //double initacc[7] = {2.0,2.0,2.0,2.0,2.0,2.0,0.0}; // --set NEW ref accelaration
    double initacc[7] = {10.0,10.0,10.0,10.0,10.0,10,0.0}; // --set NEW ref accelaration
    //double initacc[7] = {20.0,20.0,20.0,20.0,20.0,20,0.0}; // --set NEW ref accelaration
    leftLegIPositionControl->setRefAccelerations(initacc);

return true;
}

/************************************************************************/
void ThreadImpl::run()
{
    while(!isStopping()) {

        if (a!=1)    {    // STEP 1 - Creating & Configuring CSV file
            confCSVfile();
            a=1;    }
        if (a==1 && b!=1)    {    // STEP 2 - Opening & Connecting Ports
            openingPorts();
            b=1;    }

        // ------------------------------------------------------------------------
        if (a==1 && b==1)
        {
            // STEP 3 - main code

            getInitialTime();

            readSensorsFT0();
            readSensorsFT1();

            if (n>300)  {
                zmpCompFT(); // calculation of the ZMP_FT
                evaluateModel(); // evaluacion the model and the angle output
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
void ThreadImpl::confCSVfile(){       /** Configuring CSV file    **/

    cout << "[configuring] ... STEP 1 " << endl;
    fp = fopen("../data_dlipm_threat.csv","w+");
    fprintf(fp,"Time,ang_x_imu,acc_x_imu,acc_y_imu,acc_z_imu,acc_x_robot,acc_y_robot,acc_z_robot,fx_ft0,fz_ft0,my_ft0,fx_ft1,fz_ft1,my_ft1,zmp_imu,Xzmp_ft,Xzmp_ft0,Xzmp_ft1,iter");
    yarp::os::Time::delay(1);

    cout << "[success] data_dlipm_threat.csv file configured." << endl;
}

/************************************************************************/
void ThreadImpl::openingPorts(){       /** Opening Ports & Connecting with sensor programs **/

    cout << "[configuring] ... STEP 2 " << endl;

    //-- OPEN YARP PORTS
    //portImu->open("/bodyBal/inertial:i");
    portFt0->open("/bodyBal/jr3ch0:i");
    portFt1->open("/bodyBal/jr3ch1:i");
    //portFt2->open("/bodyBal/jr3ch2:i");
    //portFt3->open("/bodyBal/jr3ch3:i");

    //getchar(); // waiting for the arm waiter pose

    cout << "\n [atention] User must activate sensor programs." << endl;
    yarp::os::Time::delay(5);

    //-- CONNECTIONS PORTS

    // ft right foot
    Network::connect("/jr3/ch0:o","/waiter/jr3ch0:i");
    if ( NetworkBase::isConnected("/jr3/ch0:o","/waiter/jr3ch0:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /waiter/jr3ch0:i." << endl;
    } else cout << "[success] Connected to /jr3ch0:i." << endl;
    yarp::os::Time::delay(0.5);
    // ft left foot
    Network::connect("/jr3/ch1:o","/waiter/jr3ch1:i");
    if ( NetworkBase::isConnected("/jr3/ch1:o","/waiter/jr3ch1:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /waiter/jr3ch1:i." << endl;
    } else cout << "[success] Connected to /jr3ch1:i." << endl;
    yarp::os::Time::delay(0.5);
/*    // ft right hand
    Network::connect("/jr3/ch2:o","/waiter/jr3ch2:i");
    if ( NetworkBase::isConnected("/jr3/ch2:o","/waiter/jr3ch2:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /waiter/jr3ch2:i." << endl;
    } else cout << "[success] Connected to /jr3ch2:i." << endl;
    yarp::os::Time::delay(0.5);
    // ft left hand
    Network::connect("/jr3/ch3:o","/waiter/jr3ch3:i");
    if ( NetworkBase::isConnected("/jr3/ch3:o","/waiter/jr3ch3:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /waiter/jr3ch3:i." << endl;
    } else cout << "[success] Connected to /jr3ch3:i." << endl;
    yarp::os::Time::delay(0.5);
    // imu trunk
    Network::connect("/inertial", "/waiter/inertial:i");
    if ( NetworkBase::isConnected("/inertial", "/waiter/inertial:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /waiter/inertial:i." << endl;
    } else cout << "[success] Connected to IMU." << endl;
    yarp::os::Time::delay(0.5);*/

    return;
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
void ThreadImpl::readSensorsFT0()       /** Reading input messages from FT0 SENSORS    **/
{
        //--- FT-Sensor 0 right leg
    Bottle ch0;
    portFt0->read(ch0); // lectura del sensor JR3 ch0 - right foot
    _RF._F.fx = ch0.get(0).asFloat64();
    _RF._F.fy = ch0.get(1).asFloat64();
    _RF._F.fz = ch0.get(2).asFloat64();
    _RF._T.mx = ch0.get(3).asFloat64();
    _RF._T.my = ch0.get(4).asFloat64();
    _RF._T.mz = ch0.get(5).asFloat64();

}

/************************************************************************/
void ThreadImpl::readSensorsFT1()       /** Reading input messages from FT1 SENSORS    **/
{
        //--- FT-Sensor 1 left leg
    Bottle ch1;
    portFt1->read(ch1); // lectura del sensor JR3 ch1 - left foot
    _LF._F.fx = ch1.get(0).asFloat64();
    _LF._F.fy = ch1.get(1).asFloat64();
    _LF._F.fz = ch1.get(2).asFloat64();
    _LF._T.mx = ch1.get(3).asFloat64();
    _LF._T.my = ch1.get(4).asFloat64();
    _LF._T.mz = ch1.get(5).asFloat64();

}

/************************************************************************/
void ThreadImpl::readSensorsFT2()       /** Reading input messages from FT2 SENSORS    **/
{
        //--- FT-Sensor 2 right hand
    Bottle ch2;
    portFt2->read(ch2); // lectura del sensor JR3 ch2 - right hand
    _RH._F.fx = ch2.get(0).asFloat64();
    _RH._F.fy = ch2.get(1).asFloat64();
    _RH._F.fz = ch2.get(2).asFloat64();
    _RH._T.mx = ch2.get(3).asFloat64();
    _RH._T.my = ch2.get(4).asFloat64();
    _RH._T.mz = ch2.get(5).asFloat64();

}

/************************************************************************/
void ThreadImpl::readSensorsFT3()       /** Reading input messages from FT3 SENSORS    **/
{
    //--- FT-Sensor 3 left hand
    Bottle ch3;
    portFt3->read(ch3); // lectura del sensor JR3 ch3 - left hand
    _LH._F.fx = ch3.get(0).asFloat64();
    _LH._F.fy = ch3.get(1).asFloat64();
    _LH._F.fz = ch3.get(2).asFloat64();
    _LH._T.mx = ch3.get(3).asFloat64();
    _LH._T.my = ch3.get(4).asFloat64();
    _LH._T.mz = ch3.get(5).asFloat64();

}

/************************************************************************/
void ThreadImpl::readSensorsIMU()       /** Reading input messages from IMU SENSORS    **/
{
        //--- Inertial-Sensor
    Bottle imu;
    portImu->read(imu); // lectura del sensor IMU
    ang_x = imu.get(0).asFloat64(); // Angulo en X [deg]
    ang_y = imu.get(1).asFloat64(); // Angulo en Y [deg]
    ang_z = imu.get(2).asFloat64(); // Angulo en Z [deg]
    acc_x = imu.get(3).asFloat64(); //Linear acceleration in X [m/s^2]
    x_sensor.push_front(acc_x);
    x_sensor.pop_back();
    acc_y = imu.get(4).asFloat64(); //Linear acceleration in Y [m/s^2]
    y_sensor.push_front(acc_y);
    y_sensor.pop_back();
    acc_z = imu.get(5).asFloat64(); //Linear acceleration in Z [m/s^2]
    z_sensor.push_front(acc_z);
    z_sensor.pop_back();
    spd_x=imu.get(6).asFloat64(); // Velocidad angular en X [deg/s]
    spd_y=imu.get(7).asFloat64(); // Velocidad angular en Y [deg/s]
    spd_z=imu.get(8).asFloat64(); // Velocidad angular en Z [deg/s]
    //mag_x=imu.get(9).asFloat64(); // Campo magnetico en X
    //mag_y=imu.get(10).asFloat64(); // Campo magnetico en Y
    //mag_z=imu.get(11).asFloat64(); // Campo magnetico en Z

    //LOW-PASS FILTER
    ddx = 0.0;
    ddy = 0.0;
    ddz = 0.0;
    for(deque<double>::iterator it = x_sensor.begin(); it != x_sensor.end(); it++)
        ddx = ddx + *it;
    for(deque<double>::iterator it = y_sensor.begin(); it != y_sensor.end(); it++)
        ddy = ddy + *it;
    for(deque<double>::iterator it = z_sensor.begin(); it != z_sensor.end(); it++)
        ddz = ddz + *it;
    ddx = ddx / samples;
    ddy = ddy / samples;
    ddz = ddz / samples;

    //CONVERSION FROM IMU SENSOR COORDINATES TO ROBOT COORDINATES
     ddx_robot = ddx;
     ddy_robot = -ddy;
     ddz_robot = ddz;

}

/************************************************************************/
void ThreadImpl::getCurrentTime()       /** Get Current Time    **/
{
    act_time = Time::now() - init_time;
    act_loop = Time::now() - init_loop;
}

/************************************************************************/
void ThreadImpl::zmpCompFT()        /** Calculating ZMP-FT of the body . **/
{

    //ZMP Equations : Double Support - FT

/*      //Con el coeficiente 1000 - unidades en milimetros
    _xzmp0 = -(((_my0/10) + e*_fx0)*1000) / _fz0; // xzmp0 in [mm]
    _yzmp0 = (((_mx0/10) + e*_fy0)*1000) / _fz0; // yzmp0 in [mm]

    _xzmp1 = -(((_my1/10) + e*_fx1)*1000) / _fz1; // xzmp1 in [mm]
    _yzmp1 = (((_mx1/10) + e*_fy1)*1000) / _fz1; // yzmp1 in [mm]
*/

    //Sin el coeficiente 1000 - unidades en metros
    _xzmp_ft0 = -(((_RF._T.my/10) + e*_RF._F.fx)) / _RF._F.fz; // xzmp0_ft in [m] right foot
    //_yzmp0_ft = -(((_RF._T.mx/10) + e*_RF._F.fy)) / _RF._F.fz; // xzmp0_ft in [m] right foot
    _xzmp_ft1 = -(((_LF._T.my/10) + e*_LF._F.fx)) / _LF._F.fz; // xzmp1_ft in [m] left foot
    //_yzmp1_ft = -(((_LF._T.mx/10) + e*_LF._F.fy)) / _LF._F.fz; // xzmp1_ft in [m] left foot

    _xzmp_ft01 = (_xzmp_ft0 * _RF._F.fz + _xzmp_ft1 * _LF._F.fz) / (_RF._F.fz + _LF._F.fz); // xzmp_ft in [m] robot
    //_yzmp01_ft = (_yzmp0_ft * _RF._F.fz + _yzmp1_ft * _LF._F.fz) / (_RF._F.fz + _LF._F.fz); // xzmp_ft in [m] robot

    // OFFSET FT - eliminando el offset de _xzmp01_ft (ZMP de ambos sensores en X)
    if (n >=1 && n < 50){
        sum_x_ft = _xzmp_ft01 + sum_x_ft;
        offs_x_ft = sum_x_ft / n;
        printf("offs = %f\n", offs_x_ft);
    }

    Xzmp_ft  = _xzmp_ft01 - offs_x_ft; // frontal plane
    //Yzmp_ft = _yzmp01_ft - offs_y_ft; // saggital plane

    if ((_xzmp_ft01 != _xzmp_ft01) || (_yzmp_ft01 != _yzmp_ft01)){
        printf ("Warning: No zmp data\n");
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

/************************************************************************/
void ThreadImpl::evaluateModel()        /** Calculating OUTPUT (Qi) of the legs. **/
{
    // obtaining the angle error for the D-LIPM
    _evalLIPM.model(Xzmp_ft,zmp_ref);

    ka = 0.25 * zmp_ref + 9.95; // dudo entre zmp_ref o Xzmp_ft
    _ang_ref = (zmp_ref*(-G))/ (L*(ka-G));
/*    como otra posibilidad para calcular:  _angle_ref
    if ( ! leftArmIEncoders->getEncoders( encLegs.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;    }
    _angle_ref = encLegs[4];*/

    _ang_out =  -_evalLIPM.ang_error_out + _ang_ref;

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
    cout << endl << "El angulo imu es: " << ang_x << endl;
    //cout << endl << "El ANKLE pid es: " << pid_output_ankle << endl;
    //cout << endl << "El HIP pid es: " << pid_output_hip << endl;
    //cout << endl << "El ZMP REF es: " << setpoint << endl;
    cout << endl << "El ZMP IMU es: " << Xzmp_imu << endl;
    cout << endl << "El ZMP FT es: " << Xzmp_ft << endl;
    //cout << endl << "La capture_point es: " << capture_point << endl;
    //cout << endl << "ZMP_Error_Loli = ("<< _eval_x._zmp_error << ") [mm]" << endl;
    //cout << endl << "ZMP model es: " << _eval_x.y << endl;
    //cout << endl << "Num es: " << _num << endl;  _u_ref
    //cout << endl << "El _u_ref x es: " << _eval_x._u_ref << endl;
    //cout << endl << "El _angle_error x es: " << _eval_x._angle_error << endl;

}

/************************************************************************/
void ThreadImpl::saveInFileCsv()
{

    fprintf(fp,"\n%.2f", act_time);
    fprintf(fp,",%.10f", ang_x); // angulo x del sensor imu
    fprintf(fp,",%.10f", acc_x); // acc_x del sensor imu
    fprintf(fp,",%.10f", acc_y); // acc_y del sensor imu
    fprintf(fp,",%.10f", acc_z); // acc_z del sensor imu
    fprintf(fp,",%.10f", ddx_robot); // acc_x del robot
    fprintf(fp,",%.10f", ddy_robot); // acc_y del robot
    fprintf(fp,",%.10f", ddz_robot); // acc_z del robot
    fprintf(fp,",%.10f", _RF._F.fx); // f_x del sensor ft 0
    fprintf(fp,",%.10f", _RF._F.fz); // f_z del sensor ft 0
    fprintf(fp,",%.10f", _RF._T.my); // m_y del sensor ft 0
    fprintf(fp,",%.10f", _LF._F.fx); // f_x del sensor ft 1
    fprintf(fp,",%.10f", _LF._F.fz); // f_z del sensor ft 1
    fprintf(fp,",%.10f", _LF._T.my); // m_y del sensor ft 1
    //fprintf(fp,",%.10f", pid_output_ankle);
    //fprintf(fp,",%.10f", pid_output_hip);
    //fprintf(fp,",%.10f", setpoint);
    //fprintf(fp,",%.8f", Xzmp_total);
    fprintf(fp,",%.8f", Xzmp_imu);
    fprintf(fp,",%.8f", Xzmp_ft);
    fprintf(fp,",%.8f", _xzmp_ft0);
    fprintf(fp,",%.8f", _xzmp_ft1);
    //fprintf(fp,",%.10f", capture_point);
    fprintf(fp,",%i", n);

}

/************************************************************************/
void ThreadImpl::setIEncodersControl(IEncoders *iRightLegEncoders, IEncoders *iLeftLegEncoders)
{
    this->rightLegIEncoders = iRightLegEncoders;
    this->leftLegIEncoders = iLeftLegEncoders;
}

/************************************************************************/
void ThreadImpl::setIPositionControl(IPositionControl *iRightLegPositionControl,IPositionControl *iLeftLegPositionControl)
{
    this->rightLegIPositionControl = iRightLegPositionControl;
    this->leftLegIPositionControl = iLeftLegPositionControl;
}

/************************************************************************/
void ThreadImpl::setIVelocityControl(IVelocityControl *iRightLegVelocityControl,IVelocityControl *iLeftLegVelocityControl)
{
    this->rightLegIVelocityControl = iRightLegVelocityControl;
    this->leftLegIVelocityControl = iLeftLegVelocityControl;
}

/************************************************************************/
void ThreadImpl::setInputPorts(Port *inputPortImu,Port *inputPortFt0,Port *inputPortFt1,Port *inputPortFt2,Port *inputPortFt3)
{
    this->portImu = inputPortImu;
    this->portFt0 = inputPortFt0;
    this->portFt1 = inputPortFt1;
    this->portFt2 = inputPortFt2;
    this->portFt3 = inputPortFt3;
}

}   // namespace roboticslab
