#ifndef ROBOT_H
#define ROBOT_H

//!  ROBOT Class
/*!
  Class incorporating the robot. This class is used to define state machines, control algorithms, sensor readings,...
  It should be interfaced with the communicator to send data to the world.
*/

#include <inttypes.h>
#include "math.h"

#include <microOS.h>
#include <encoder_sensor.h>
#include <difference_sensor.h>
#include <sharp41S.h>
#include <l293d.h>
#include <joystick.h>
#include "kalman_filter.h"

class Robot
{
  private:
    uint8_t _ID;			//give the robot an ID so that you can recognize it
    uint8_t _type;

    float _position1[2];
    float _position2[2];
    float _input1[3];
    float _error1[3];
    float _input2[3];
    float _error2[3];

    // Give the robot some sensors
    Sensor1D* _encoder1;
    Sensor1D* _encoder2;
    Sensor1D* _speed1;
    Sensor1D* _speed2;
    Sensor1D* _distance1;
    Sensor1D* _distance2;

    // Give the robot some motors
    HBridgeInterface* _motor1;
    HBridgeInterface* _motor2;

    // Interface the buttons
    bool _button_states[8] = {false, false, false, false, false, false, false, false};
    bool toggleButton(uint8_t button);
    bool KalmanFilterEnabled();
    bool csvInputEnabled();

    // Kalman filter
    Matrix<1> _xhat;
    Matrix<1, 1> _Phat;

    // Position controller
    Matrix<1> xref;
    Matrix<1, 1> K;

  public:
    Robot(uint8_t ID = 0);

    ////////
    /// FUNC
    void init();			//set up the robot
    void controllerHook();	//update function which can be executed continuously
    void resetEncoders();	//reset the encoders
    double wheelSpeedA();
    double wheelSpeedB();

    void resetKalmanFilter();

    ///////
    /// SET
    void setID(uint8_t ID);

    ///////
    /// GET
    uint8_t id();
    uint8_t type();

    // Event callbacks
    void button1callback();
    void button2callback();
    void button3callback();
    void button4callback();
    void button5callback();
    void button6callback();
    void button7callback();
    void button8callback();
};

#endif //ROBOT_H
