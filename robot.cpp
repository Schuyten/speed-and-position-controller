#include "robot.h"
#include <math.h>
#include <stdio.h>

#define ENC1_PINA 18
#define ENC1_PINB 32
#define ENC2_PINA 19
#define ENC2_PINB 42
#define DIST1_PIN A0
#define DIST2_PIN A1
#define MOT1_PIN_IN1 8u
#define MOT1_PIN_IN2 12u
#define MOT1_PIN_EN 6u
#define MOT2_PIN_IN1 7u
#define MOT2_PIN_IN2 4u
#define MOT2_PIN_EN 5u
#define PENDULUM_PIN A5
#define LED1_PIN 34
#define LED2_PIN 40
#define BATTERY_VOLTAGE 6000

#define RAD_TO_ENC    (34.0 * 11.0 * 2.0) / M_PI  // transforms radians -> encoder counts
#define ENC_TO_RAD    M_PI / (34.0 * 11.0 * 2.0)  // transforms encoder counts -> radians
#define RWHEEL        0.0325                      // wheel radius in meter

Robot::Robot(uint8_t ID) : _ID(ID),
                           _type(20),
                           _encoder1(new EncoderSensor(ENC1_PINA, ENC1_PINB)),
                           _encoder2(new EncoderSensor(ENC2_PINA, ENC2_PINB)),
                           _speed1(new DifferenceSensor(_encoder1, 10)), //,0.2f
                           _speed2(new DifferenceSensor(_encoder2, 11)), //,0.2f
                           _distance1(new Sharp41S(DIST1_PIN)),
                           _distance2(new Sharp41S(DIST2_PIN)),
                           _motor1(new L293D(MOT1_PIN_IN1, MOT1_PIN_IN2, MOT1_PIN_EN, BATTERY_VOLTAGE)),
                           _motor2(new L293D(MOT2_PIN_IN1, MOT2_PIN_IN2, MOT2_PIN_EN, BATTERY_VOLTAGE))
{

  _encoder1->setScale(ENC_TO_RAD);
  _encoder2->setScale(ENC_TO_RAD);
}

void Robot::init()
{
  //initialize the robot - sort of starting procedure
  resetKalmanFilter();
  resetEncoders();

  for(int k = 0; k < 2; k++) {
    _position1[k] = 0;
    _position2[k] = 0; 
  }
  for(int k = 0; k < 3; k++) {
    _error1[k] = 0;
    _input1[k] = 0;
    _error2[k] = 0;
    _input2[k] = 0; 
  }
}

void Robot::controllerHook()
{
  //do something that is periodic
  //update speed sensors
  _speed1->readCalibratedValue();
  _speed2->readCalibratedValue();

  float x1 = _encoder1->readCalibratedValue();
  float x2 = _encoder2->readCalibratedValue();

  
  // the speed controller parameters
  float Ki1 = 3800/2;
  float Kp1 = 100/2;
  float Kd1 = 0;
  float Ki2 = 3800/2;
  float Kp2 = 100/2;
  float Kd2 = 0;
  float Ts = 0.01;

  float velocity1 = -(_position1[1] - _position1[0])/(Ts);
  float velocity2 = -(_position2[1] - _position2[0])/(Ts);

  _position1[1] = _position1[0];
  _position1[0] = x1;

  _position2[1] = _position2[0];
  _position2[0] = x2;

  int u_bridge_left = 0;
  int u_bridge_right = 0;
  
  // initialize values for position controller to zero

  //Kalman filtering
  if (KalmanFilterEnabled()) {
    //prediction step
    //use measured wheel speed as input to TimeUpdate
    Matrix<1> vmeas;
    vmeas(0) = (-wheelSpeedA() + wheelSpeedB()) / 2;
    TimeUpdate(vmeas, _xhat, _Phat);
    //correction step
    Matrix<1> distance_measurement;
    distance_measurement(0) = _distance1->readCalibratedValue();
    MeasurementUpdate(distance_measurement, _xhat, _Phat);

    // fixed reference
    xref(0) = -0.1;
    // or use input
    //xref(0) = System.getGPinFloat(0);
    
    K(0) = 100.0; // Tune the K matrix for better performance
    Matrix<1> v = K * (xref - _xhat);

    _error1[2] = _error1[1];
    _error1[1] = _error1[0];
    _error1[0] = v(0) - velocity1;
  
    _error2[2] = _error2[1];
    _error2[1] = _error2[0];
    _error2[0] = v(0) - velocity2;
    
    u_bridge_left = _input1[2] + (-Kp1+Ki1*Ts/2+Kd1*2/Ts)*_error1[2] + (Ki1*Ts-4*Kd1/Ts)*_error1[1] + (Kd1*2/Ts+Ki1*Ts/2+Kp1)*_error1[0]; //<--- with v(0) as reference, implement the speed controller designed in assignment 2.
    u_bridge_right = _input2[2] + (-Kp2+Ki2*Ts/2+Kd2*2/Ts)*_error2[2] + (Ki2*Ts-4*Kd2/Ts)*_error2[1] + (Kd2*2/Ts+Ki2*Ts/2+Kp2)*_error2[0]; //<--- with v(0) as reference, implement the speed controller designed in assignment 2.

    _input1[2] = _input1[1];
    _input1[1] = _input1[0];
    _input1[0] = u_bridge_left;

    _input2[2] = _input2[1];
    _input2[1] = _input2[0];
    _input2[0] = u_bridge_right;

    if (u_bridge_left > 6000)
      u_bridge_left = 6000;
    if (u_bridge_left < -6000)
      u_bridge_left = -6000;

    if (u_bridge_right > 6000)
      u_bridge_right = 6000;
    if (u_bridge_right < -6000)
      u_bridge_right = -6000;
    
    //send wheel speed command
    _motor1->setBridgeVoltage(u_bridge_left);
    _motor2->setBridgeVoltage(u_bridge_right);
    
  } else {
    _motor1->setBridgeVoltage(0);
    _motor2->setBridgeVoltage(0);
  }

  // int outputs
  System.setGPoutInt(0, _motor1->getBridgeVoltage());
  System.setGPoutInt(1, _motor2->getBridgeVoltage());

  // float outputs
  System.setGPoutFloat(0, wheelSpeedA());
  System.setGPoutFloat(1, wheelSpeedB());
  System.setGPoutFloat(2, _xhat(0));
  System.setGPoutFloat(3, _Phat(0));
}

void Robot::resetEncoders()
{
  _encoder1->init();
  _encoder2->init();
}

// returns the wheel speed in [m/s]
double Robot::wheelSpeedA()
{
  return _speed1->peekRawValue() * ENC_TO_RAD * RWHEEL;
}

// returns the wheel speed in [m/s]
double Robot::wheelSpeedB()
{
  return _speed2->peekRawValue() * ENC_TO_RAD * RWHEEL;
}

void Robot::resetKalmanFilter()
{
  // Initialize state covariance matrix
  _Phat.Fill(0);
  _Phat(0, 0) = 1e-6;

  // Initialize state estimate
  _xhat(0) = -0.30;
}

void Robot::setID(uint8_t ID)
{
  _ID = ID;
}

uint8_t Robot::id()
{
  return _ID;
}

uint8_t Robot::type()
{
  return _type;
}

bool Robot::toggleButton(uint8_t button)
{
  _button_states[button] = !_button_states[button];
  return _button_states[button];
}

bool Robot::KalmanFilterEnabled()
{
  return _button_states[1];
}

void Robot::button1callback()
{
  toggleButton(0);

  resetEncoders();
  
  // reset reference and gain
  xref.Fill(0.0);
  K.Fill(0.0);

  System.println("Reset.");
}

void Robot::button2callback()
{
  resetKalmanFilter();
  if (toggleButton(1))
  {
    System.println("Kalman filter enabled.");
  }
  else
  {
    System.println("Kalman filter disabled.");
  }
}
void Robot::button3callback()
{
  toggleButton(2);
}

void Robot::button4callback()
{
  toggleButton(3);
}

void Robot::button5callback()
{
  toggleButton(4);
}

void Robot::button6callback()
{
  toggleButton(5);
}

void Robot::button7callback()
{
  toggleButton(6);
}

void Robot::button8callback()
{
  toggleButton(7);
}

